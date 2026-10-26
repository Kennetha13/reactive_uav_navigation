// objective_function.hpp
//
// Objective function G(v) of DWA-3D (arXiv:2409.05421, eq. 10):
//
//     G(v) = alpha * Heading(v) + beta * Dist(v) + gamma * Vel(v)
//     v*   = argmax over candidates of G(v)
//
// This header only combines the three terms and picks the best candidate.
// The terms themselves are computed elsewhere:
//   Heading(v)  -> heading function (not yet in the repo)
//   Dist(v)     -> DistanceFunction::computeDistance()   (distance_function.hpp)
//   Vel(v)      -> velocity()                              (velocity.cpp)
// Each term must already be normalized to [0, 1].
//
// Typical use in ReactiveNavigator::control_loop():
//
//     std::vector<Candidate> candidates;
//     for (each candidate velocity v from the dynamic window) {
//         Candidate c;
//         c.vx = ...; c.vy = ...; c.vz = ...; c.wz = ...;
//         c.heading = heading(...);
//         c.dist    = distanceCalc.computeDistance(...);
//         c.vel     = velocity(...);
//         candidates.push_back(c);
//     }
//     BestCandidate best = selectBestVelocity(candidates, weights);
//     if (!best.found) { /* BLOCKED: publish zero velocity */ }
//     else { cmd.linear.x = best.candidate.vx; ... }
//
// Conventions match distance_function.hpp: velocities in m/s, yaw rate in rad/s.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

namespace reactive_uav_navigation {

// Weights of the objective function.
//
// Defaults are the values recommended in the paper (Table 2). The paper's
// constraints (eqs. 12, 23, 25) are enforced by validate():
//   alpha + beta + gamma = 1   (keeps G(v) in [0, 1])
//   beta > alpha               (avoiding obstacles beats staying on course)
//   beta > gamma               (avoiding obstacles beats going fast)
struct ObjectiveWeights {
    double alpha = 0.3;  // weight of Heading(v)
    double beta = 0.6;   // weight of Dist(v)
    double gamma = 0.1;  // weight of Vel(v)

    // Throws std::invalid_argument if the weights are unusable.
    void validate() const {
        if (alpha < 0.0 || beta < 0.0 || gamma < 0.0) {
            throw std::invalid_argument("objective weights must be non-negative");
        }
        constexpr double kTol = 1e-6;
        if (std::fabs(alpha + beta + gamma - 1.0) > kTol) {
            throw std::invalid_argument(
                "alpha + beta + gamma must equal 1 (got " +
                std::to_string(alpha + beta + gamma) + ")");
        }
        if (!(beta > alpha)) {
            throw std::invalid_argument("beta must be greater than alpha");
        }
        if (!(beta > gamma)) {
            throw std::invalid_argument("beta must be greater than gamma");
        }
    }
};

// One candidate velocity together with its three normalized term values.
// The velocity fields are carried along untouched so the winner can be
// published directly; this header never reads them.
struct Candidate {
    // Candidate velocity (as produced by the dynamic-window sampler)
    double vx = 0.0;  // m/s
    double vy = 0.0;  // m/s
    double vz = 0.0;  // m/s
    double wz = 0.0;  // yaw rate, rad/s

    // Normalized objective terms, each in [0, 1]
    double heading = 0.0;  // Heading(v)
    double dist = 0.0;     // Dist(v)
    double vel = 0.0;      // Vel(v)
};

// Result of selectBestVelocity().
struct BestCandidate {
    bool found = false;     // false -> no usable candidate: report BLOCKED, publish zero
    std::size_t index = 0;  // position of the winner in the input vector
    double score = 0.0;     // G(v) of the winner
    Candidate candidate;    // the winner itself (velocity + term values)
};

// G(v) = alpha * heading + beta * dist + gamma * vel.
//
// Term values are clamped to [0, 1] so a slightly out-of-range input (e.g.
// floating-point noise from a term function) cannot dominate the sum.
// Returns NaN if any term is NaN, so the caller can skip that candidate.
inline double computeObjective(double heading, double dist, double vel,
                               const ObjectiveWeights& w) {
    if (std::isnan(heading) || std::isnan(dist) || std::isnan(vel)) {
        return std::nan("");
    }
    heading = std::clamp(heading, 0.0, 1.0);
    dist = std::clamp(dist, 0.0, 1.0);
    vel = std::clamp(vel, 0.0, 1.0);
    return w.alpha * heading + w.beta * dist + w.gamma * vel;
}

inline double computeObjective(const Candidate& c, const ObjectiveWeights& w) {
    return computeObjective(c.heading, c.dist, c.vel, w);
}

// Scores every candidate and returns the one with the highest G(v).
//
//   - Candidates with a NaN term are skipped.
//   - Ties go to the candidate that appears first in the vector.
//   - If the vector is empty, or every candidate was skipped, returns
//     found = false. The caller should then treat the drone as BLOCKED and
//     publish a zero velocity rather than an unsafe command.
//
// `candidates` should contain only admissible velocities (those that pass the
// braking-distance check, paper eq. 9); filtering happens before this call.
inline BestCandidate selectBestVelocity(const std::vector<Candidate>& candidates,
                                        const ObjectiveWeights& w) {
    w.validate();
    BestCandidate best;
    for (std::size_t i = 0; i < candidates.size(); ++i) {
        const double g = computeObjective(candidates[i], w);
        if (std::isnan(g)) {
            continue;
        }
        if (!best.found || g > best.score) {
            best.found = true;
            best.index = i;
            best.score = g;
            best.candidate = candidates[i];
        }
    }
    return best;
}

}  // namespace reactive_uav_navigation
