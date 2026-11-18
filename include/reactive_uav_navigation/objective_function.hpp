// objective_function.hpp
//
// Objective function G(v) of DWA-3D (arXiv:2409.05421, eq. 10):
//
//     G(v) = alpha * Heading(v) + beta * Dist(v) + gamma * Vel(v)
//     v*   = argmax over candidates of G(v)
//
// ObjectiveFunction combines the three terms and picks the best candidate.
// The terms themselves are computed elsewhere:
//   Heading(v)  -> HeadingFunction::computeHeading()      (heading_function.hpp)
//   Dist(v)     -> DistanceFunction::computeDistance()    (distance_function.hpp)
//   Vel(v)      -> VelocityFunction::computeVelocity()    (velocity_function.hpp)
//
// Use in ReactiveNavigator::control_loop(), once per control cycle:
//
//     objective_function_->setGoal(target);
//     objective_function_->setMaxSpeed(max forward speed of Vs);
//     BestVelocity best = objective_function_->selectBest(vr);
//     if (!best.can_move) { /* BLOCKED: publish zero velocity */ }
//     else { /* publish best.velocity */ }
//
// Conventions match distance_function.hpp: velocities in m/s, yaw rate in rad/s.


#pragma once

#include <memory>
#include <vector>

#include <geometry_msgs/msg/point.hpp>

#include "reactive_uav_navigation/common.hpp"
#include "reactive_uav_navigation/drone_state.hpp"

class HeadingFunction;
class VelocityFunction;

namespace reactive_uav_navigation
{

class DistanceFunction;

// The three terms of G(v) for one candidate, each clamped to [0, 1], and their
// weighted sum. Kept together so the chosen velocity's individual scores can be
// passed on (e.g. distance -> NavInputs::chosen_dist) or logged.
struct ObjectiveScores {
    double heading = 0.0;   // Heading(v)
    double distance = 0.0;  // Dist(v)
    double velocity = 0.0;  // Vel(v)
    double total = 0.0;     // G(v) = alpha*heading + beta*distance + gamma*velocity
};

// The winner of one control cycle.
struct BestVelocity {
    bool found = false;         // false when there was no candidate to choose from
    bool can_move = false;      // false when every safe candidate only hovers or turns on the spot
    CandidateVelocity velocity; // the chosen [vx, vz, wz]
    ObjectiveScores scores;     // its Heading, Dist and Vel scores and G(v)
};

class ObjectiveFunction
{
public:

    // Constructor accepts the existing drone state.
    ObjectiveFunction(
        const DroneState& drone_state,
        const geometry_msgs::msg::Point& goal,
        DistanceFunction& distance_function
    );

    // Defined in objective_function.cpp, where HeadingFunction, DistanceFunction
    // and VelocityFunction are complete types (required by std::unique_ptr).
    ~ObjectiveFunction();
    
    void setGoal(const geometry_msgs::msg::Point& goal);

    // Sets the largest forward speed a candidate can have (the Vs limit),
    // used to normalize Vel(v).
    void setMaxSpeed(double vel_max);

    // Scores every candidate and returns the one with the highest G(v).
    // Candidates with a NaN score are skipped; ties go to the first.
    // If the waypoint is behind the drone or outside the camera's view,
    // returns a turn on the spot toward it instead.
    BestVelocity selectBest(const std::vector<CandidateVelocity>& candidates);

    // Calculates the maximum vertical error for the candidate set.
    void prepareCandidates(
        const std::vector<CandidateVelocity>& candidates
    );

    // Calculates Heading, Dist and Vel for one candidate velocity, and G(v).
    ObjectiveScores computeScores(const CandidateVelocity& candidate);

    // Calculates the combined score for one candidate velocity.
    // Same as computeScores(candidate).total.
    double computeObjective(const CandidateVelocity& candidate);

private:

    // +1 if the drone must first turn left on the spot because the waypoint
    // is behind it or outside the camera's view, -1 for a right turn, 0 if
    // the waypoint is in front and in view.
    double turnDirectionToGoal() const;

    // Scoring functions
    std::unique_ptr<HeadingFunction> heading_function_;
    std::unique_ptr<VelocityFunction> velocity_function_;

    // Shared with the velocity admissibility check so both use the same
    // obstacle points.
    DistanceFunction& distance_function_;

    // Reference to existing drone state
    const DroneState& drone_state_;

    // Objective weights
    const double alpha_ = 0.3;
    const double beta_ = 0.6;
    const double gamma_ = 0.1;

    // Used to normalize vertical heading scores
    double max_z_error_ = 0.0;

    // Used to normalize velocity scores
    double vel_max_ = 0.3;

    // The waypoint the drone is heading for
    geometry_msgs::msg::Point goal_;
};

} // namespace reactive_uav_navigation
