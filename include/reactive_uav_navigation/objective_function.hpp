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

#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "reactive_uav_navigation/common.hpp"
#include "reactive_uav_navigation/drone_state.hpp"

class HeadingFunction;
class VelocityFunction;

namespace reactive_uav_navigation
{

class DistanceFunction;

class ObjectiveFunction
{
public:

    // Constructor accepts the ROS2 node and existing drone state.
    ObjectiveFunction(
        rclcpp::Node* node,
        const DroneState& drone_state,
        const geometry_msgs::msg::Point& goal
    );

    // Defined in objective_function.cpp, where HeadingFunction, DistanceFunction
    // and VelocityFunction are complete types (required by std::unique_ptr).
    ~ObjectiveFunction();
    
    void setGoal(const geometry_msgs::msg::Point& goal);

    // Calculates the maximum vertical error for the candidate set.
    void prepareCandidates(
        const std::vector<CandidateVelocity>& candidates
    );

    // Calculates the combined score for one candidate velocity.
    double computeObjective(const CandidateVelocity& candidate);

private:

    // Scoring functions
    std::unique_ptr<HeadingFunction> heading_function_;
    std::unique_ptr<DistanceFunction> distance_function_;
    std::unique_ptr<VelocityFunction> velocity_function_;

    // Reference to existing drone state
    const DroneState& drone_state_;

    // Objective weights
    const double alpha_ = 0.3;
    const double beta_ = 0.6;
    const double gamma_ = 0.1;

    // Used to normalize vertical heading scores
    double max_z_error_ = 0.0;
};

} // namespace reactive_uav_navigation
