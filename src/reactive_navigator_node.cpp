#include <algorithm>
#include <functional>
#include <reactive_uav_navigation/reactive_navigator_node.hpp>

namespace reactive_uav_navigation {

// straight line distance between two points
static double distance_between(double x1, double y1, double z1, double x2, double y2, double z2)
{
	double dx = x1 - x2;
	double dy = y1 - y2;
	double dz = z1 - z2;
	return std::sqrt(dx * dx + dy * dy + dz * dz);
}

ReactiveNavigator::ReactiveNavigator()
: Node("reactive_navigator")
{
// QoS (Quality of Service) to keep history queue of last 10 messages
auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

// tolerances and ranges can be changed with ROS parameters
waypoint_tolerance_ = this->declare_parameter("waypoint_tolerance", 0.3);
search_range_ = this->declare_parameter("search_range", 3.0);
double drone_radius = this->declare_parameter("drone_radius", 0.3);

// set to false to fly without a depth camera, every direction then counts as clear
use_depth_camera_ = this->declare_parameter("use_depth_camera", true);

// a depth image older than this is not trusted
depth_timeout_ = this->declare_parameter("depth_timeout", 1.0);

// how long an obstacle is remembered after it leaves the camera's view
obstacle_memory_.memory_time = this->declare_parameter("obstacle_memory_time", 15.0);

// a coarser velocity grid than the vss defaults, so every candidate can be
// checked against the obstacle points within one control period
this->declare_parameter("vss.vx_step", 0.1);
this->declare_parameter("vss.vz_step", 0.1);
this->declare_parameter("vss.wz_step_deg", 5.0);

// We are handling the interface nav_msgs/msg/path (this is how the global planner is defined)
path_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
	"planned_path", // topic name
	qos,
	std::bind(&ReactiveNavigator::path_callback, this, std::placeholders::_1) // bind the current object instance to a function, which will handle the incoming arguments
);
RCLCPP_INFO(this->get_logger(), "Listening for global path...");

// Create drone state object
drone_state_ = std::make_shared<DroneState>(this);

// DWA-3D components. Built once and reused every control period.
//   depth_processor_    : depth camera images -> obstacle points
//   distance_function_  : ray fan used by Va (vss_admissible) to find d_col
//   vss_collector_      : Vr = Vs ∩ Vd ∩ Va, the candidate velocities to score
//   objective_function_ : G(v) = alpha*Heading + beta*Dist + gamma*Vel for one candidate
if (use_depth_camera_) {
	depth_processor_ = std::make_unique<DepthRayProcessor>(this);
}

DistanceParams distance_params;
distance_params.R_drone = drone_radius;
distance_params.r_search = search_range_;
distance_function_ = std::make_unique<DistanceFunction>(distance_params);

vss_collector_ = std::make_unique<VssCollector>(this);

// No path has arrived yet, so start with a placeholder goal; control_loop()
// sets the real one (the current waypoint) every cycle with setGoal().
objective_function_ = std::make_unique<ObjectiveFunction>(
  *drone_state_, geometry_msgs::msg::Point{}, *distance_function_);

// navigation state machine, the goal uses the same tolerance as the waypoints
NavStateParams nav_params;
nav_params.goal_tolerance = waypoint_tolerance_;

// turning around at a sharp corner takes several seconds without getting
// closer, so the stall timeout is longer than the state machine's default
nav_params.stall_timeout = this->declare_parameter("stall_timeout", 10.0);
nav_state_ = std::make_unique<NavigationStateMachine>(nav_params);

// Publisher for drone movement commands
cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

// Publisher for the navigation state, e.g. FOLLOWING_PATH or BLOCKED
state_publisher_ = this->create_publisher<std_msgs::msg::String>("navigation_state", 10);

control_timer_ = this->create_wall_timer(
	std::chrono::milliseconds(static_cast<int>(CONTROL_PERIOD * 1000)),
	std::bind(&ReactiveNavigator::control_loop, this)
);
RCLCPP_INFO(this->get_logger(), "Control loop initialized...");
}

// Accessor method for the latest global path
nav_msgs::msg::Path ReactiveNavigator::get_global_path() const
{
std::lock_guard<std::mutex> lock(path_mutex_);
return global_path_;
}

void ReactiveNavigator::path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
std::lock_guard<std::mutex> lock(path_mutex_);

// a new path always replaces the old one, so the global planner can send a
// new route while the drone is flying or blocked
global_path_ = *msg;
current_index_ = 0;
nav_state_->reset();
detour_planner_.reset();
detour_index_ = 0;
obstacle_in_the_way_ = false;

// start the metrics over for the new path
path_start_time_ = this->now().seconds();
distance_flown_ = 0.0;
blocked_count_ = 0;
has_last_position_ = false;

RCLCPP_INFO(
	this->get_logger(),
	"Received new path with frame_id: '%s' containing %zu poses. Beginning execution to first pose.",
	msg->header.frame_id.c_str(),
	msg->poses.size()
);
}

void ReactiveNavigator::updateObstacles(const geometry_msgs::msg::Point& position, double yaw)
{
	if (!use_depth_camera_) {
		return;
	}

	// without a recent image nothing is known, so no direction counts as
	// visible and the drone stays where it is
	if (!depth_processor_->hasFreshImage(depth_timeout_)) {
		RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
			"No recent depth image. Hovering. Set use_depth_camera:=false to fly without a camera.");
		distance_function_->setVisibleSector(yaw, 0.0, 0.0);
		return;
	}

	// points a little past the search range still matter for the predicted positions
	std::vector<geometry_msgs::msg::Vector3> points = depth_processor_->makeWorldPoints(
		position, drone_state_->get_orientation(), search_range_ + 1.0);

	// the camera looks along the drone's heading
	const double half_fov = depth_processor_->horizontalFov() / 2.0;
	const double half_vertical_fov = depth_processor_->verticalFov() / 2.0;
	distance_function_->setVisibleSector(yaw, half_fov, half_vertical_fov);

	// obstacles that just left the view are remembered for a while
	obstacle_memory_.update(points, this->now().seconds(),
		position.x, position.y, position.z, yaw, half_fov, half_vertical_fov);
	distance_function_->setObstaclePoints(obstacle_memory_.points());

	// the detour planner looks for the end of an obstacle in the same points
	detour_planner_.setObstacles(obstacle_memory_.points(), half_fov);
}

void ReactiveNavigator::control_loop()
{
nav_msgs::msg::Path current_path = get_global_path();

// Do nothing if there is no odometry or no path
if (!drone_state_->has_odometry() || current_path.poses.empty()) {
	NavInputs in;
	in.now = this->now().seconds();
	in.has_path = false;
	updateNavState(in);
	RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for odometry and a path. Stopping the drone.");
	publishHover();
	return;
}

// Gets the latest position and yaw from drone state
auto current_position = drone_state_->get_current_position();

double current_x = current_position.x;
double current_y = current_position.y;
double current_z = current_position.z;
double current_yaw = drone_state_->get_yaw();

// add up the distance flown, for the metrics
if (has_last_position_) {
	distance_flown_ += distance_between(current_x, current_y, current_z,
		last_position_.x, last_position_.y, last_position_.z);
}
last_position_ = current_position;
has_last_position_ = true;

// ######################################################################################
// DWA-3D: choose the velocity for this control period
//   1. advance past waypoints already reached
//   2. Vr = Vs ∩ Vd ∩ Va            (VssCollector)
//   3. score each candidate with G(v) and keep the best (ObjectiveFunction)
//   4. publish it, or stop if no candidate is available
// DetourPlanner chooses the point the velocity is aimed at: the waypoint, a
// point past the end of an obstacle, or a side step along a wall.
// ######################################################################################

// 1. Advance past any waypoints already reached. The last waypoint is the
//    goal, which the navigation state machine checks.
const size_t last_index = current_path.poses.size() - 1;
while (current_index_ < last_index) {
	const auto &p = current_path.poses[current_index_].pose.position;
	if (distance_between(p.x, p.y, p.z, current_x, current_y, current_z) >= waypoint_tolerance_) break;
	RCLCPP_INFO(get_logger(), "Reached waypoint %zu, moving to next.", current_index_);
	++current_index_;
}

// a new waypoint starts without a detour
if (current_index_ != detour_index_) {
	detour_planner_.reset();
	detour_index_ = current_index_;
}

// latest obstacle points and camera view
updateObstacles(current_position, current_yaw);

// The point to head for: the waypoint, a point past the end of an obstacle
// that is in the way, or a side step when the drone is stuck in front of one.
const auto &waypoint = current_path.poses[current_index_].pose.position;
const geometry_msgs::msg::Point target = detour_planner_.update(
	this->now().seconds(), current_position, current_yaw, waypoint, obstacle_in_the_way_);
const double distance_to_target = distance_between(
	target.x, target.y, target.z, current_x, current_y, current_z);

// 2. Candidate velocities Vr = Vs ∩ Vd ∩ Va
//    Pose in the world frame (ENU). Current velocity in the drone-centric frame
//    [vx forward, vz up, wz yaw rate]: /mavros/local_position/odom reports its
//    twist in the body frame, so linear.x is forward speed.
DronePose pose;
pose.position.x = current_x;
pose.position.y = current_y;
pose.position.z = current_z;
pose.yaw = current_yaw;

const geometry_msgs::msg::Twist measured = drone_state_->get_current_velocity();
CandidateVelocity current_velocity;
current_velocity.vx = measured.linear.x;
current_velocity.vz = measured.linear.z;
current_velocity.wz = measured.angular.z;

const std::vector<CandidateVelocity> vr =
	vss_collector_->computeVr(*distance_function_, current_velocity, pose);

// 3. Best candidate by G(v)
//    Heading(v) scores how well each candidate points at the current waypoint,
//    so the goal must be updated before the candidates are scored.
objective_function_->setGoal(target);
objective_function_->setMaxSpeed(vss_collector_->maximum().limits().vx_max);
const BestVelocity best = objective_function_->selectBest(vr);

// clearance straight toward the waypoint, tells if an obstacle is in the way.
// If the camera cannot see that way yet, nothing is known about it.
double goal_clearance = 1.0;
const double goal_yaw = std::atan2(target.y - current_y, target.x - current_x);
const double goal_pitch = std::atan2(target.z - current_z,
	std::hypot(target.x - current_x, target.y - current_y));
if (distance_function_->isDirectionVisible(goal_yaw)) {
	goal_clearance = distance_function_->computeDistanceUpTo(
		pose.position, goal_yaw, goal_pitch, distance_to_target);
}

// 4. Navigation state: FOLLOWING_PATH, AVOIDING_OBSTACLE, BLOCKED or GOAL_REACHED
const auto &goal = current_path.poses[last_index].pose.position;
NavInputs in;
in.now = this->now().seconds();
in.has_path = true;
in.distance_to_goal = distance_between(goal.x, goal.y, goal.z, current_x, current_y, current_z);
in.on_last_waypoint = (current_index_ == last_index);
in.remaining_path_length = remainingPathLength(current_path.poses, current_index_,
	current_x, current_y, current_z);

// hovering and turning on the spot are always safe, so a safe velocity only
// counts if it moves the drone
in.safe_velocity_found = best.found && best.can_move;

// an obstacle is near if it limits the chosen velocity or is in the way to the waypoint
in.chosen_dist = best.found ? std::min(best.scores.distance, goal_clearance) : 0.0;

// the detour planner uses this in the next cycle
obstacle_in_the_way_ = in.chosen_dist < nav_state_->params().avoid_enter_dist;
in.detouring = detour_planner_.isDetouring();
updateNavState(in);

// 5. BLOCKED (no safe velocity) or GOAL_REACHED: hover in place
//    rather than send a command.
if (!best.found || nav_state_->mustHover()) {
	// One exception while BLOCKED: turning on the spot moves the drone
	// toward nothing, so it stays allowed. Without it a drone that stopped
	// right in front of a wall could never turn away from it again.
	const bool turn_only =
		std::abs(best.velocity.vx) < 1e-9 && std::abs(best.velocity.vz) < 1e-9;
	if (best.found && turn_only && nav_state_->state() == NavState::BLOCKED) {
		publishVelocity(best.velocity, current_yaw);
		return;
	}

	publishHover();
	return;
}

publishVelocity(best.velocity, current_yaw);

//######################################################################################
// END of DWA-3D velocity selection
// ######################################################################################

// Verify the loop is functioning
RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
"[%s] idx=%zu/%zu pos=(%.2f,%.2f,%.2f) target=(%.2f,%.2f,%.2f) dist_to_target=%.2fm yaw=%.2f |Vr|=%zu best=(vx %.2f, vz %.2f, wz %.2f) H=%.3f D=%.3f V=%.3f G=%.3f",
toString(nav_state_->state()), current_index_, current_path.poses.size(), current_x, current_y, current_z,
target.x, target.y, target.z, distance_to_target, current_yaw, vr.size(),
best.velocity.vx, best.velocity.vz, best.velocity.wz,
best.scores.heading, best.scores.distance, best.scores.velocity,
best.scores.total);
}

void ReactiveNavigator::updateNavState(const NavInputs& in)
{
	const bool changed = nav_state_->update(in);

	// the state is published every cycle so other nodes can follow it
	std_msgs::msg::String state_msg;
	state_msg.data = toString(nav_state_->state());
	state_publisher_->publish(state_msg);

	if (!changed) {
		return;  // no change: print nothing
	}

	if (nav_state_->state() == NavState::BLOCKED) {
		blocked_count_++;
		RCLCPP_WARN(this->get_logger(), "Navigation state -> BLOCKED (%s)",
			toString(nav_state_->blockedReason()));
	} else {
		RCLCPP_INFO(this->get_logger(), "Navigation state -> %s",
			toString(nav_state_->state()));
	}

	// report the metrics once the path is finished
	if (nav_state_->state() == NavState::GOAL_REACHED) {
		RCLCPP_INFO(this->get_logger(), "Path finished: time %.1f s, distance flown %.2f m, blocked %d time(s)",
			in.now - path_start_time_, distance_flown_, blocked_count_);
	}
}

void ReactiveNavigator::publishVelocity(const CandidateVelocity& v, double yaw)
{
	// cmd_vel_unstamped is interpreted in the world frame (ENU) by MAVROS by
	// default, but v.vx is forward speed along the drone's heading. Rotate it
	// into world x/y with the current yaw.
	geometry_msgs::msg::Twist cmd;
	cmd.linear.x = v.vx * std::cos(yaw);
	cmd.linear.y = v.vx * std::sin(yaw);
	cmd.linear.z = v.vz;
	cmd.angular.z = v.wz;
	cmd_vel_publisher->publish(cmd);
}

void ReactiveNavigator::publishHover()
{
	// a zero velocity command
	geometry_msgs::msg::Twist cmd;
	cmd_vel_publisher->publish(cmd);
}

}
