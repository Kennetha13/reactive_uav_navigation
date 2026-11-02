// velocity_search_space.hpp
//
// Vr = Vs ∩ Vd ∩ Va of DWA-3D: the set of velocities the objective function
// scores. This is the single entry point that joins
//
//   Vs  (maximum velocities)  -> velocity_space.hpp
//   Vd  (dynamic window)      -> dynamic_window.hpp
//   Va  (admissible)          -> admissible_velocities.hpp
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
// Returns std::vector<AdmissibleCandidate> (admissible_velocities.hpp). Every
// element is one velocity that is inside Vs, reachable (Vd) and safe to stop
// from (Va):
//
//     AdmissibleCandidate {
//         CandidateVelocity velocity;   // {vx, vz, wz}, drone-centric: m/s, m/s, rad/s
//         DronePose         predicted;  // pose after VELOCITY_HORIZON at this velocity
//                                       // (ENU world frame, yaw in rad)
//         double            dCol;       // closest obstacle from `predicted` (m),
//                                       // = DistanceFunction::computeDistMin()
//     }
//
// vx >= 0 for every element (Vs). The vector is empty when nothing is safe, so
// the caller must treat that as BLOCKED / NO_SAFE_VELOCITY and hover or brake.
//
// How the rest of the codebase uses it
// ------------------------------------
// Each element becomes one `Candidate` for selectBestVelocity()
// (objective_function.hpp):
//
//     // once, at startup (reused every cycle)
//     VelocityLimits limits;  VelocitySteps steps;
//     DynamicWindow window{DynamicWindowParams{}};
//     AdmissibleVelocities admissible{AdmissibleParams{}};  // same dt as window
//
//     // every control cycle
//     DronePose pose{position, yaw};            // from DroneState (ENU world frame)
//     CandidateVelocity current{vx, vz, wz};    // last command or measured velocity
//     std::vector<AdmissibleCandidate> vr =
//         computeVr(limits, steps, window, admissible, distanceFunction, current, pose);
//
//     std::vector<CandidateVelocity> velocities;      // for the heading z-normalizer
//     for (const auto& a : vr) velocities.push_back(a.velocity);
//     const double maxZError = heading.getMaxZError(velocities);
//
//     std::vector<Candidate> scored;
//     for (const auto& a : vr) {
//         Candidate c;
//         c.vx = a.velocity.vx;  c.vy = 0.0;            // no lateral velocity
//         c.vz = a.velocity.vz;  c.wz = a.velocity.wz;
//         c.heading = heading term (HeadingFunction::computeHeading(a.velocity, maxZError))
//         c.dist    = Dist(v), normalized to [0, 1], from a.dCol
//                     (max(0, (dCol - R_drone) / (r_search - R_drone)))
//         c.vel     = Vel(v): compute head_psi = heading.computeYawHeading(a.velocity)
//                     first, then velocityFunction.computeVelocity(
//                         Vector3{a.velocity.vx, 0, a.velocity.vz},
//                         heading.K_z, heading.K_yaw, head_psi)
//         scored.push_back(c);
//     }
//     BestCandidate best = selectBestVelocity(scored, weights);
//     if (!best.found) { /* BLOCKED: publish zero velocity */ }
//     else             { /* command = {best.candidate.vx, vz, wz} */ }
//


#pragma once

#include <stdexcept>
#include <string>
#include <vector>

#include "reactive_uav_navigation/admissible_velocities.hpp"
#include "reactive_uav_navigation/distance_function.hpp"
#include "reactive_uav_navigation/dynamic_window.hpp"
#include "reactive_uav_navigation/velocity_space.hpp"

namespace reactive_uav_navigation {

// Returns Vr = Vs ∩ Vd ∩ Va as scored-ready candidates (see output pattern above).
// Throws std::invalid_argument if `window` and `admissible` use different dt.
inline std::vector<AdmissibleCandidate> computeVr(
    const VelocityLimits& limits,
    const VelocitySteps& steps,
    const DynamicWindow& window,
    const AdmissibleVelocities& admissible,
    const DistanceFunction& distanceFunction,
    const CandidateVelocity& current_velocity,
    const DronePose& current_pose) {
    // Vd and Va must look ahead over the same horizon
    if (window.params().dt != admissible.params().dt) {
        throw std::invalid_argument(
            "DynamicWindow dt (" + std::to_string(window.params().dt) +
            ") must equal AdmissibleVelocities dt (" +
            std::to_string(admissible.params().dt) + ")");
    }

    // Vs ∩ Vd, sampled
    const std::vector<CandidateVelocity> candidates =
        generateCandidates(limits, steps, current_velocity, window);

    // ∩ Va
    return admissible.filterAdmissible(candidates, current_pose, distanceFunction);
}

}  // namespace reactive_uav_navigation
