#include "reactive_uav_navigation/heading_function.hpp"
#include <cmath>

HeadingFunction::HeadingFunction(const DroneState& drone_state)
    : drone_state_(drone_state) {
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

double HeadingFunction::computeYawHeading(CandidateVelocity velocity) {

    double current_x = drone_state_.get_current_position().x;
    double x_goal = current_x + (CONTROL_PERIOD * current_x * std::cos(velocity.wz));
    double delta_x = x_goal - current_x;

    double current_y = drone_state_.get_current_position().y;
    double y_goal = current_y + (CONTROL_PERIOD * current_y * std::sin(velocity.wz));
    double delta_y = y_goal - current_y;

    double desired_yaw = std::atan2(delta_y, delta_x);
    double current_yaw = drone_state_.get_yaw();

    double yaw_error = std::abs(desired_yaw - current_yaw);
    return 1 - (yaw_error / M_PI);
}

double HeadingFunction::computeZHeading(CandidateVelocity velocity, double max_z_error) {
    double current_z = drone_state_.get_current_position().z;
    double z_goal = current_z + (CONTROL_PERIOD * velocity.vz);
    double delta_z = std::abs(z_goal - current_z);

    return 1 - (delta_z / max_z_error);
}

double HeadingFunction::getMaxZError(std::vector<CandidateVelocity> candidate_velocities) {
    double max_z_error = 0.0;
    double current_z = drone_state_.get_current_position().z;

    for (const auto& velocity : candidate_velocities) {
        double z_goal = current_z + (CONTROL_PERIOD * velocity.vz);
        double delta_z = std::abs(z_goal - current_z);
        if (delta_z > max_z_error) {
            max_z_error = delta_z;
        }
    }

    return max_z_error;
}