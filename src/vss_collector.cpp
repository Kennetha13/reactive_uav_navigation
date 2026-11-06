#include "reactive_uav_navigation/vss_collector.hpp"

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <utility>

namespace reactive_uav_navigation {

// Calculate Vs ∩ Vd (per-axis highest minimum, lowest maximum)
IntersectionResult VssCollector::intersectVelocitySpaces(const VelocitySpace& vs,
                                                         const VelocitySpace& vd) {
    VelocitySpace result;

    result.min.vx = std::max(vs.min.vx, vd.min.vx);
    result.max.vx = std::min(vs.max.vx, vd.max.vx);

    result.min.vz = std::max(vs.min.vz, vd.min.vz);
    result.max.vz = std::min(vs.max.vz, vd.max.vz);

    result.min.wz = std::max(vs.min.wz, vd.min.wz);
    result.max.wz = std::min(vs.max.wz, vd.max.wz);

    // Check whether all three dimensions overlap
    bool valid =
        result.min.vx <= result.max.vx &&
        result.min.vz <= result.max.vz &&
        result.min.wz <= result.max.wz;

    return {valid, result};
}

namespace {

// The window time of Vd is the prediction time of Va: one value for both.
DynamicWindowParams windowParams(const AccelerationLimits& accel, const AdmissibleParams& admissible) {
    DynamicWindowParams params;
    params.accel = accel;
    params.dt = admissible.dt;
    return params;
}

std::unique_ptr<MaximumVelocitiesSpace> requireMaximum(std::unique_ptr<MaximumVelocitiesSpace> maximum) {
    if (!maximum) {
        throw std::invalid_argument("VssCollector needs a MaximumVelocitiesSpace");
    }
    return maximum;
}

}  // namespace

#ifndef VSS_NO_ROS
VssCollector::VssCollector(rclcpp::Node* node)
    : VssCollector(std::make_unique<MaximumVelocitiesSpace>(node),
                   std::make_unique<DynamicWindow>(node, AdmissibleParams{}.dt),
                   AdmissibleParams{}) {}
#endif

VssCollector::VssCollector(std::unique_ptr<MaximumVelocitiesSpace> maximum,
                           const AccelerationLimits& accel,
                           const AdmissibleParams& admissible)
    : VssCollector(std::move(maximum),
                   std::make_unique<DynamicWindow>(windowParams(accel, admissible)),
                   admissible) {}

VssCollector::VssCollector(std::unique_ptr<MaximumVelocitiesSpace> maximum,
                           std::unique_ptr<DynamicWindow> window,
                           const AdmissibleParams& admissible)
    : maximum_(requireMaximum(std::move(maximum))),
      window_(std::move(window)),
      admissible_(admissible) {}

std::vector<CandidateVelocity> VssCollector::generateCandidates(
    const CandidateVelocity& current) const {
    const VelocitySpace vs = maximum_->compute();
    const VelocitySpace vd = window_->compute(current);
    const IntersectionResult vr = intersectVelocitySpaces(vs, vd);
    if (!vr.valid) {
        return {};
    }
    return maximum_->sample(vr.space);
}

std::vector<CandidateVelocity> VssCollector::computeVr(
    const DistanceFunction& distanceFunction,
    const CandidateVelocity& current_velocity,
    const DronePose& current_pose) const {
    // Vs ∩ Vd, sampled
    const std::vector<CandidateVelocity> candidates =
        generateCandidates(current_velocity);

    // ∩ Va
    return admissible_.filterAdmissible(candidates, current_pose, distanceFunction);
}

}  // namespace reactive_uav_navigation
