// admissible_velocities.hpp
//
// Admissible velocity set Va of DWA-3D (eq. 9). The
// objective function
//
//     G(v) = alpha * Heading(v) + beta * Dist(v) + gamma * Vel(v)
//
// is only evaluated over velocities in Vs ∩ Vd ∩ Va:
//
//   Vs  (maximum velocity space)  -> computed elsewhere
//   Vd  (dynamic window)          -> computed elsewhere
//   Va  (admissible velocities)   -> this header
//
// A velocity is admissible if the drone can still stop before the closest
// obstacle.
//
//     Va = { (v_x, v_z) | v_xz <= sqrt(2 * d_col * a_max) }
//     v_xz = sqrt(v_x^2 + v_z^2)
//
// where a_max is the maximum deceleration and d_col is the distance from the
// *predicted* drone position to its closest obstacle. The predicted position
// is obtained by applying v for a prediction time-step dt (eq. 13):
//
//     yaw' = yaw + w_z * dt
//     x'   = x + v_x * dt * cos(yaw')
//     y'   = y + v_x * dt * sin(yaw')
//     z'   = z + v_z * dt
//
// d_col is taken from DistanceFunction::computeDistMin (distance_function.hpp)
// evaluated at that predicted position. The paper evaluates Dist(v) at the same
// predicted position, so the result is returned for reuse.
//
// Input
// -----
// filterAdmissible() takes the candidates that already passed Vs and Vd. The
// input format (std::vector<CandidateVelocity>) is a placeholder until the
// Vs/Vd code is in the repo; adapt it to whatever that code produces.
//
// Conventions
// -----------
// Candidate velocities use the paper's drone-centric control vector
// [v_x, v_z, w_z] (eq. 6): v_x is forward speed along the drone's heading,
// v_z is vertical speed (positive up), w_z is yaw rate. There is no lateral
// velocity. Positions and yaw are in the ENU world frame, as in
// distance_function.hpp. Meters, seconds, radians.

#pragma once

#include <vector>

#include "reactive_uav_navigation/common.hpp"
#include "reactive_uav_navigation/distance_function.hpp"

namespace reactive_uav_navigation {

// Configuration for AdmissibleVelocities. Fixed at startup, like
// DistanceParams.
struct AdmissibleParams {
    double a_max = 1.0;          // maximum deceleration used for braking (m/s^2); not given in the paper
    double dt = 1.0;             // prediction time-step (s); paper recommends 10x the control period
    double safety_margin = 0.0;  // extra clearance subtracted from d_col (m); not in the paper, 0 matches eq. 9

    // Throws std::invalid_argument if the configuration is unusable.
    void validate() const;
};

// Drone position and yaw in the world frame, either current or predicted.
struct DronePose {
    geometry_msgs::msg::Vector3 position;
    double yaw = 0.0;  // rad
};

// A candidate that passed the Va check, along with its predicted pose and
// d_col so the caller can reuse them for Heading(v) and Dist(v) without
// predicting or casting the rays again.
struct AdmissibleCandidate {
    CandidateVelocity velocity;
    DronePose predicted;
    double dCol = 0.0;
};

// Applies the Va filter. Holds only configuration, so a single instance can
// be reused across planning cycles.
class AdmissibleVelocities {
public:
    // Throws std::invalid_argument if `params` fails AdmissibleParams::validate().
    explicit AdmissibleVelocities(const AdmissibleParams& params);

    // Predicted pose after applying `v` from `current` for dt (eq. 13).
    DronePose predictPose(const DronePose& current, const CandidateVelocity& v) const;

    // Returns the highest speed v_xz from which the drone can stop within
    // `dCol`. Returns 0 if there is no clearance.
    double maxAdmissibleSpeed(double dCol) const;

    // Va check for one candidate (eq. 9). Yaw rate is not part of the check,
    // so a pure rotation (v_x = v_z = 0) is always admissible.
    bool isAdmissible(const CandidateVelocity& v, double dCol) const;

    // Keeps the candidates (already in Vs ∩ Vd) that pass the Va check. For
    // each one, predicts the pose and casts the DistanceFunction ray fan
    // from it, centered on the candidate's direction of motion.
    //
    // May return an empty vector (e.g. moving fast toward an obstacle); the
    // caller should then brake.
    std::vector<AdmissibleCandidate> filterAdmissible(
        const std::vector<CandidateVelocity>& candidates,
        const DronePose& current,
        const DistanceFunction& distanceFunction) const;

    const AdmissibleParams& params() const { return params_; }

private:
    AdmissibleParams params_;
};

} // namespace reactive_uav_navigation
