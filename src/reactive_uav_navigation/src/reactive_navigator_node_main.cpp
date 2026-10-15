#include "reactive_uav_navigation/reactive_navigator_node.hpp"

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<reactive_uav_navigation::ReactiveNavigator>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}