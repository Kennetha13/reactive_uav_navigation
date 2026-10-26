#include "reactive_uav_navigation/velocity_function.hpp"


// Vel(v) from the objective function
// Get a single candidate velocity in 3d vector (Eigen) form, with reference max velocity
// Return normalized double in [0, 1] range
double VelocityFunction::computeVelocity(geometry_msgs::msg::Vector3 candidate_velocity) const {
    // avoiding div by 0
    if (vel_max_x_ <= 0.0) {
        return 0.0;
    }

    return std::clamp(candidate_velocity.x / vel_max_x_, 0.0, 1.0);
}