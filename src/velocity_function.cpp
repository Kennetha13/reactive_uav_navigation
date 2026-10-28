#include "reactive_uav_navigation/velocity_function.hpp"


// Vel(v) from the objective function
// Get a single candidate velocity in 3d vector (Eigen) form, with reference max velocity
// Return normalized double in [0, 1] range
double VelocityFunction::computeVelocity(
    geometry_msgs::msg::Vector3 candidate_velocity,
    double k_z,
    double k_psi,
    double head_psi) const {
    // avoiding div by 0
    if (vel_max_x_ <= 0.0) {
        return 0.0;
    }

    const bool forward = (k_z > k_psi) || (head_psi > 0.5 && k_z < k_psi);

    if (!forward) {
        return 0.0;
    }

    return std::clamp(candidate_velocity.x / vel_max_x_, 0.0, 1.0);
}