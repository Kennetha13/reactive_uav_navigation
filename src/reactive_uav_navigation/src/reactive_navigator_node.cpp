#include<reactive_uav_navigation/reactive_navigator_node.hpp>

namespace reactive_uav_navigation {

ReactiveNavigator::ReactiveNavigator(std::shared_ptr<DroneState> drone_state) : Node("reactive_navigator"), drone_state_(drone_state)
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

// Publisher for drone movement commands
cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

control_timer_ = this->create_wall_timer(
	std::chrono::milliseconds(100), // 10 Hz
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
// AI-Generated Code as POC. This is where the reactive navigation logic would be
// to calculate the result velocity for each control period directed at the current pose
// ######################################################################################

// Advance past any waypoints already reached

while (current_index_ < current_path.poses.size()) {
	const auto &p = current_path.poses[current_index_].pose.position;
	if (std::hypot(p.x - current_x, p.y - current_y) >= 0.3) break;
	RCLCPP_INFO(get_logger(), "Reached waypoint %zu, moving to next.", current_index_);
	++current_index_;
}

if (current_index_ >= current_path.poses.size()) {
	cmd_vel_publisher->publish(cmd);  // zero command
	return;
}

const auto &target = current_path.poses[current_index_].pose.position;
const double dx = target.x - current_x;
const double dy = target.y - current_y;
const double distance = std::hypot(dx, dy);

double heading_error = std::atan2(dy, dx) - current_yaw;
while (heading_error > M_PI)  heading_error -= 2.0 * M_PI;
while (heading_error < -M_PI) heading_error += 2.0 * M_PI;

const double k_v = 0.5, k_w = 2.0, max_speed = 0.4;
double vx = k_v * dx;
double vy = k_v * dy;
const double speed = std::hypot(vx, vy);
if (speed > max_speed) {
	vx *= max_speed / speed;
	vy *= max_speed / speed;
}

cmd.linear.x = vx;
cmd.linear.y = vy;
cmd.angular.z = k_w * heading_error;

//######################################################################################
// END of POC code
// ######################################################################################

RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
"idx=%zu/%zu pos=(%.2f,%.2f) target=(%.2f,%.2f) dist=%.2f yaw=%.2f err=%.2f cmd=(%.2f,%.2f,%.2f)",
current_index_, current_path.poses.size(), current_x, current_y,
target.x, target.y, distance, current_yaw, heading_error,
cmd.linear.x, cmd.linear.y, cmd.angular.z);

cmd_vel_publisher->publish(cmd);
}

}
