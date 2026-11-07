#include "reactive_uav_navigation/heading_function.hpp"
#include <cmath>
#include <stdexcept>
#include <algorithm>

HeadingFunction::HeadingFunction(const DroneState& drone_state,
                               const geometry_msgs::msg::Point& goal)
    : drone_state_(drone_state), goal_(goal) {
    if (K_yaw < 0.0 || K_z < 0.0) {
        throw std::invalid_argument(
            "K_yaw and K_z must be non-negative");
    }
    if (K_yaw + K_z != 1.0) {
        throw std::invalid_argument(
            "K_yaw and K_z must sum to 1.0");
    }
}

double HeadingFunction::computeHeading(CandidateVelocity velocity, double max_z_error) {
    double yawHeading = computeYawHeading(velocity);
    double zHeading = computeZHeading(velocity, max_z_error);
    return (K_yaw * yawHeading) + (K_z * zHeading);
}

double HeadingFunction::computeYawHeading(const CandidateVelocity& velocity) {

    const auto position = drone_state_.get_current_position();

    const double predicted_yaw = drone_state_.get_yaw() + (CONTROL_PERIOD * velocity.wz);


    double predicted_x = position.x + CONTROL_PERIOD * velocity.vx * std::cos(predicted_yaw);
    double delta_x = goal_.x - predicted_x;

    double predicted_y = position.y + CONTROL_PERIOD * velocity.vx * std::sin(predicted_yaw);
    double delta_y = goal_.y - predicted_y;

    double desired_yaw = std::atan2(delta_y, delta_x);

    double yaw_error = std::remainder(desired_yaw - predicted_yaw, 2 * M_PI);
    return 1.0 - std::abs(yaw_error) / M_PI;
}

double HeadingFunction::computeZHeading(CandidateVelocity velocity, double max_z_error) {
    double current_z = drone_state_.get_current_position().z;
    double predicted_z = current_z + (CONTROL_PERIOD * velocity.vz);
    double delta_z = std::abs(goal_.z - predicted_z);

    if (max_z_error <= 0.0) {
        return 1.0;
    }

    return 1.0 - (delta_z / max_z_error);
}

double HeadingFunction::getMaxZError(std::vector<CandidateVelocity> candidate_velocities) {
    double max_z_error = 0.0;
    double current_z = drone_state_.get_current_position().z;

    for (const auto& velocity : candidate_velocities) {
        double predicted_z = current_z + (CONTROL_PERIOD * velocity.vz);
        double delta_z = std::abs(goal_.z - predicted_z);
        if (delta_z > max_z_error) {
            max_z_error = delta_z;
        }
    }

    return max_z_error;
}