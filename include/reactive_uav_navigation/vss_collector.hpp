// vss_collector.hpp
//
// Vr = Vs ∩ Vd ∩ Va of DWA-3D: the set of velocities the objective function
// scores. This is the single entry point that joins
//
//   Vs  (maximum velocities)  -> vss_maximum.hpp
//   Vd  (dynamic window)      -> vss_dynamic_window.hpp
//   Va  (admissible)          -> vss_admissible.hpp
//
// Call it once per control cycle (every CONTROL_PERIOD).
//
// Pipeline
// --------
//   Vs ∩ Vd   box intersection, then sampled on the VelocitySteps grid
//   ∩ Va      per-candidate braking test at the predicted pose
//
// Output pattern
// --------------
// Returns std::vector<CandidateVelocity> (common.hpp). Every element is one
// velocity that is inside Vs, reachable (Vd) and safe to stop from (Va):
//
//     CandidateVelocity {
//         double vx;   // forward speed (m/s), >= 0
//         double vz;   // vertical speed (m/s)
//         double wz;   // yaw rate (rad/s)
//     }                // drone-centric frame
//
// vx >= 0 for every element (Vs). The vector is empty when nothing is safe, so
// the caller must treat that as BLOCKED / NO_SAFE_VELOCITY and hover or brake.
//
// How the rest of the codebase uses it
// ------------------------------------
// Each element becomes one `Candidate` for selectBestVelocity()
// (objective_function.hpp):
//
//     // once, at startup (reused every cycle), like DroneState
//     VssCollector collector{this};             // this = the rclcpp::Node
//
//     // every control cycle
//     DronePose pose{position, yaw};            // from DroneState (ENU world frame)
//     CandidateVelocity current{vx, vz, wz};    // last command or measured velocity
//     std::vector<CandidateVelocity> vr =
//         collector.computeVr(distanceFunction, current, pose);
//
//     const double maxZError = heading.getMaxZError(vr);   // heading z-normalizer
//
//     std::vector<Candidate> scored;
//     for (const CandidateVelocity& v : vr) {
//         Candidate c;
//         c.vx = v.vx;  c.vy = 0.0;                     // no lateral velocity
//         c.vz = v.vz;  c.wz = v.wz;
//         c.heading = heading.computeHeading(v, maxZError)
//         c.dist    = Dist(v): the pose after VELOCITY_HORIZON is
//                     collector.admissible().predictPose(pose, v); then
//                     distanceFunction.computeDistance(predicted.position,
//                         predicted.yaw, atan2(v.vz, v.vx))
//         c.vel     = Vel(v): compute head_psi = heading.computeYawHeading(v)
//                     first, then velocityFunction.computeVelocity(
//                         Vector3{v.vx, 0, v.vz},
//                         heading.K_z, heading.K_yaw, head_psi)
//         scored.push_back(c);
//     }
//     BestCandidate best = selectBestVelocity(scored, weights);
//     if (!best.found) { /* BLOCKED: publish zero velocity */ }
//     else             { /* command = {best.candidate.vx, vz, wz} */ }
//


#pragma once

#include <memory>
#include <vector>

#include "reactive_uav_navigation/vss_admissible.hpp"
#include "reactive_uav_navigation/distance_function.hpp"
#include "reactive_uav_navigation/vss_dynamic_window.hpp"
#include "reactive_uav_navigation/vss_maximum.hpp"

namespace reactive_uav_navigation {

struct IntersectionResult {
    bool valid;
    VelocitySpace space;
};

// Owns the three velocity search spaces (Vs, Vd, Va) and joins them into Vr.
// Built once and reused every control cycle. Vd and Va are built from the same
// dt, so they always look ahead over the same horizon.
class VssCollector {
public:
    // Builds the search spaces from the node: Vs caps and steps and Vd
    // accelerations from ROS parameters, and their limits from PX4 (see
    // MaximumVelocitiesSpace and DynamicWindow). The node must outlive this
    // object and be spun. Not available when built with VSS_NO_ROS.
    explicit VssCollector(rclcpp::Node* node);

    // Without ROS: takes a ready Vs. The window time is admissible.dt, shared
    // by Vd and Va. Throws std::invalid_argument if `maximum` is null or a
    // parameter is invalid.
    explicit VssCollector(std::unique_ptr<MaximumVelocitiesSpace> maximum,
                          const AccelerationLimits& accel = AccelerationLimits{},
                          const AdmissibleParams& admissible = AdmissibleParams{});

    // Returns Vr = Vs ∩ Vd ∩ Va (see output pattern above). Empty if nothing is
    // admissible.
    std::vector<CandidateVelocity> computeVr(
        const DistanceFunction& distanceFunction,
        const CandidateVelocity& current_velocity,
        const DronePose& current_pose) const;

    // Vs ∩ Vd. Va is not a box (it depends on each candidate's predicted pose),
    // so it is applied afterwards by AdmissibleVelocities::filterAdmissible.
    static IntersectionResult intersectVelocitySpaces(const VelocitySpace& vs,
                                                      const VelocitySpace& vd);

    const MaximumVelocitiesSpace& maximum() const { return *maximum_; }
    const DynamicWindow& window() const { return *window_; }
    const AdmissibleVelocities& admissible() const { return admissible_; }

private:
    VssCollector(std::unique_ptr<MaximumVelocitiesSpace> maximum,
                 std::unique_ptr<DynamicWindow> window,
                 const AdmissibleParams& admissible);

    // Vs ∩ Vd sampled on the step grid; empty if the boxes do not overlap.
    // The result still has to go through AdmissibleVelocities::filterAdmissible (Va).
    std::vector<CandidateVelocity> generateCandidates(const CandidateVelocity& current) const;

    std::unique_ptr<MaximumVelocitiesSpace> maximum_;
    std::unique_ptr<DynamicWindow> window_;  // holds a lock, so it can't be a plain member
    AdmissibleVelocities admissible_;
};

}  // namespace reactive_uav_navigation
