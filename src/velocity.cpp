#include <algorithm>
#include <Eigen/Dense>


// Vel(v) from the objective function
// Get a single candidate velocity in 3d vector (Eigen) form, with reference max velocity
// Return normalized double in [0, 1] range
double velocity(const Eigen::Vector3d& candidate, double vel_max) {
    // avoiding div by 0
    if (vel_max <= 0.0) {
        return 0.0;
    }

    double speed = candidate.norm();
    return std::clamp(speed / vel_max, 0.0, 1.0);
}

/*
example usage:
velocity({0.3, 0.0, 0.0}, 0.6);
*/