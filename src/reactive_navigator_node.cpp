#include "reactive_navigator.hpp"

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<ReactiveNavigator>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}