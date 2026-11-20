#include "reactive_uav_navigation/velocity_function.hpp"

// Vel(v) from the objective function (Fig. 21)
// Returns candidate_vx / vel_max, in [0, 1]
double VelocityFunction::computeVelocity(
    double candidate_vx,
    double vel_max,
    double k_z,
    double k_psi,
    double head_psi) const {

    // speed is only rewarded when the drone is roughly facing the goal
    bool forward = (k_z > k_psi) || (head_psi > 0.5 && k_z < k_psi);

    // avoiding div by 0
    if (!forward || vel_max <= 0.0) {
        return 0.0;
    }

    double score = candidate_vx / vel_max;

    // keep the score inside [0, 1]
    if (score < 0.0) {
        score = 0.0;
    }
    if (score > 1.0) {
        score = 1.0;
    }
    return score;
}
