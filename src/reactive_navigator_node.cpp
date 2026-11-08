#include<reactive_uav_navigation/reactive_navigator_node.hpp>

namespace reactive_uav_navigation {

ReactiveNavigator::ReactiveNavigator() : Node("reactive_navigator")
{
// QoS (Quality of Service) to keep history queue of last 10 messages
auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

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
//   distance_function_  : ray fan used by Va (vss_admissible) to find d_col
//   vss_collector_      : Vr = Vs ∩ Vd ∩ Va, the candidate velocities to score
//   objective_function_ : G(v) = alpha*Heading + beta*Dist + gamma*Vel for one candidate
distance_function_ = std::make_unique<DistanceFunction>(DistanceParams{});
vss_collector_ = std::make_unique<VssCollector>(this);
// No path has arrived yet, so start with a placeholder goal; control_loop()
// sets the real one (the current waypoint) every cycle with setGoal().
objective_function_ = std::make_unique<ObjectiveFunction>(this, *drone_state_, geometry_msgs::msg::Point{});

// Publisher for drone movement commands
cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

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
bool is_executing_path = !global_path_.poses.empty() && current_index_ < global_path_.poses.size();

if (is_executing_path) {
	RCLCPP_WARN(
	this->get_logger(),
	"Received a new path while executing the current one. Ignoring the new path."
	);
	return;
}

global_path_ = *msg;
current_index_ = 0;
nav_state_.reset();
RCLCPP_INFO(
	this->get_logger(),
	"Received new path with frame_id: '%s' containing %zu poses. Beginning execution to first pose.",
	msg->header.frame_id.c_str(),
	msg->poses.size()
);
}

void ReactiveNavigator::control_loop()
{
nav_msgs::msg::Path current_path = get_global_path();
geometry_msgs::msg::Twist cmd;

// Do nothing if the path is empty
if (!drone_state_->has_odometry() || current_path.poses.empty() || current_index_ >= current_path.poses.size()) {
	// Past the last waypoint = GOAL_REACHED; otherwise still WAITING_FOR_PATH
	const bool path_done = !current_path.poses.empty() && current_index_ >= current_path.poses.size();
	NavInputs in;
	in.now = this->now().seconds();
	in.has_path = path_done;
	updateNavState(in);
	RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Path is empty or completed. Stopping the drone.");
	cmd.linear.x = 0.0;
	cmd.angular.z = 0.0;
	cmd_vel_publisher->publish(cmd);
	return;
}

// Gets the latest position and yaw from drone state
auto current_position = drone_state_->get_current_position();

double current_x = current_position.x;
double current_y = current_position.y;
double current_yaw = drone_state_->get_yaw();


// ######################################################################################
// DWA-3D: choose the velocity for this control period
//   1. advance past waypoints already reached
//   2. Vr = Vs ∩ Vd ∩ Va            (VssCollector)
//   3. score each candidate with G(v) and keep the best (ObjectiveFunction)
//   4. publish it, or stop if no candidate is available
// ######################################################################################

// 1. Advance past any waypoints already reached
while (current_index_ < current_path.poses.size()) {
	const auto &p = current_path.poses[current_index_].pose.position;
	if (std::hypot(p.x - current_x, p.y - current_y) >= 0.3) break;
	RCLCPP_INFO(get_logger(), "Reached waypoint %zu, moving to next.", current_index_);
	++current_index_;
}

if (current_index_ >= current_path.poses.size()) {
	NavInputs in;
	in.now = this->now().seconds();
	in.has_path = true;  // distance_to_goal = 0 -> GOAL_REACHED
	updateNavState(in);
	cmd_vel_publisher->publish(cmd);  // zero command
	return;
}

const auto &target = current_path.poses[current_index_].pose.position;

// 2. Candidate velocities Vr = Vs ∩ Vd ∩ Va
//    Pose in the world frame (ENU). Current velocity in the drone-centric frame
//    [vx forward, vz up, wz yaw rate]: /mavros/local_position/odom reports its
//    twist in the body frame, so linear.x is forward speed.
DronePose pose;
pose.position.x = current_position.x;
pose.position.y = current_position.y;
pose.position.z = current_position.z;
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
//    so the goal must be updated before the candidates are prepared and scored.
objective_function_->setGoal(target);
const std::optional<ScoredVelocity> best = selectBestVelocity(vr);

// 4. Navigation state: FOLLOWING_PATH, AVOIDING_OBSTACLE, BLOCKED or GOAL_REACHED
const auto &goal = current_path.poses.back().pose.position;
NavInputs in;
in.now = this->now().seconds();
in.has_path = true;
in.distance_to_goal = std::hypot(goal.x - current_x, goal.y - current_y, goal.z - current_position.z);
in.remaining_path_length = remainingPathLength(current_path.poses, current_index_,
	current_x, current_y, current_position.z);
in.safe_velocity_found = best.has_value();
in.chosen_dist = best ? best->scores.distance : 0.0;
updateNavState(in);

// 5. BLOCKED (no safe velocity, or no progress) or GOAL_REACHED: hover in place
//    rather than send a command.
if (!best || nav_state_.mustHover()) {
	cmd_vel_publisher->publish(cmd);  // zero command
	return;
}

publishVelocity(best->velocity, current_yaw);

//######################################################################################
// END of DWA-3D velocity selection
// ######################################################################################

// Verify the loop is functioning
RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
"[%s] idx=%zu/%zu pos=(%.2f,%.2f,%.2f) target=(%.2f,%.2f,%.2f) yaw=%.2f |Vr|=%zu best=(vx %.2f, vz %.2f, wz %.2f) G=%.3f",
toString(nav_state_.state()), current_index_, current_path.poses.size(), current_x, current_y, current_position.z,
target.x, target.y, target.z, current_yaw, vr.size(),
best->velocity.vx, best->velocity.vz, best->velocity.wz, best->score);
}

std::optional<ReactiveNavigator::ScoredVelocity> ReactiveNavigator::selectBestVelocity(
	const std::vector<CandidateVelocity>& vr)
{
	if (vr.empty()) {
		return std::nullopt;
	}

	// Heading(v) normalizes its z term by the largest z error across all of Vr,
	// so this must run once per cycle before any candidate is scored.
	objective_function_->prepareCandidates(vr);

	std::optional<ScoredVelocity> best;
	for (const CandidateVelocity& v : vr) {
		const ObjectiveScores scores = objective_function_->computeScores(v);
		const double g = scores.total;
		if (std::isnan(g)) {
			continue;
		}
		if (!best || g > best->score) {
			best = ScoredVelocity{v, g, scores};
		}
	}
	return best;
}

void ReactiveNavigator::updateNavState(const NavInputs& in)
{
	if (!nav_state_.update(in)) {
		return;  // no change: print nothing
	}
	if (nav_state_.state() == NavState::BLOCKED) {
		RCLCPP_WARN(this->get_logger(), "Navigation state -> BLOCKED (%s)",
			toString(nav_state_.blockedReason()));
	} else {
		RCLCPP_INFO(this->get_logger(), "Navigation state -> %s",
			toString(nav_state_.state()));
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

}
