#include "reactive_uav_navigation/admissible_velocities.hpp"

#include <cmath>
#include <stdexcept>

namespace reactive_uav_navigation {

void AdmissibleParams::validate() const {
    // maxAdmissibleSpeed() takes sqrt(2 * a_max * d_col).
    if (a_max <= 0.0) {
        throw std::invalid_argument("a_max must be > 0");
    }
    if (dt <= 0.0) {
        throw std::invalid_argument("dt must be > 0");
    }
    if (safety_margin < 0.0) {
        throw std::invalid_argument("safety_margin must be non-negative");
    }
}

AdmissibleVelocities::AdmissibleVelocities(const AdmissibleParams& params)
    : params_(params) {
    params_.validate();
}

DronePose AdmissibleVelocities::predictPose(const DronePose& current,
                                            const CandidateVelocity& v) const {
    // Eq. 13: uniform motion over dt, yaw updated first.
    DronePose predicted;
    predicted.yaw = current.yaw + v.wz * params_.dt;
    predicted.position.x = current.position.x + v.vx * params_.dt * std::cos(predicted.yaw);
    predicted.position.y = current.position.y + v.vx * params_.dt * std::sin(predicted.yaw);
    predicted.position.z = current.position.z + v.vz * params_.dt;
    return predicted;
}

double AdmissibleVelocities::maxAdmissibleSpeed(double dCol) const {
    const double clearance = dCol - params_.safety_margin;
    if (clearance <= 0.0) {
        return 0.0;
    }
    // Stopping from speed v at deceleration a takes v^2 / (2a) meters.
    return std::sqrt(2.0 * params_.a_max * clearance);
}

bool AdmissibleVelocities::isAdmissible(const CandidateVelocity& v, double dCol) const {
    const double speed = std::hypot(v.vx, v.vz);  // v_xz
    // Hovering or rotating in place never closes the distance to an obstacle.
    constexpr double kEps = 1e-9;
    if (speed < kEps) {
        return true;
    }
    return speed <= maxAdmissibleSpeed(dCol);
}

std::vector<AdmissibleCandidate> AdmissibleVelocities::filterAdmissible(
    const std::vector<CandidateVelocity>& candidates,
    const DronePose& current,
    const DistanceCalculator& distanceCalculator,
    const IRayCaster& rayCaster) const {
    std::vector<AdmissibleCandidate> admissible;
    admissible.reserve(candidates.size());

    for (const CandidateVelocity& v : candidates) {
        const DronePose predicted = predictPose(current, v);

        // Center the ray fan on the direction of motion: horizontally along
        // the predicted heading, tilted up or down by the climb angle.
        const double pitch = std::atan2(v.vz, v.vx);
        const double dCol = distanceCalculator.computeDistMin(
            predicted.position, predicted.yaw, pitch, rayCaster);

        if (isAdmissible(v, dCol)) {
            admissible.push_back(AdmissibleCandidate{v, predicted, dCol});
        }
    }
    return admissible;
}

} // namespace reactive_uav_navigation
