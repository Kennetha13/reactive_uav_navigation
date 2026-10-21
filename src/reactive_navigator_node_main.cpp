#include "reactive_uav_navigation/reactive_navigator_node.hpp"
#include "reactive_uav_navigation/drone_state.hpp"

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);

	auto drone_state = std::make_shared<DroneState>(); // Shared object to receive the odometry data

	auto node = std::make_shared<reactive_uav_navigation::ReactiveNavigator>(drone_state);

	// Allows multiple ROS nodes to receive messages and run callbacks in the same program
	rclcpp::executors::SingleThreadedExecutor executor;
	executor.add_node(drone_state); // Adds DroneState to allow its odometry callback to run
	executor.add_node(node); // Adds ReactiveNavigator so its path callback and control timer can run
	executor.spin();

	rclcpp::shutdown();
	return 0;
}