#include "reactive_navigator.hpp"
#include <Eigen/Dense>

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<ReactiveNavigator>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}

// vel(v) from the objective function
// candidate velocities
double velocity(const Eigen::Vector3d& candidate, double vel_max) {
	// edge case, no negatives
	if (vel_max <= 0.0) {
		return 0.0;
	}

	double speed = candidate.norm();
	return clamp(speed / vel_max, 0.0, 1.0);
}