// navigation_state.hpp
//
// Navigation state of the reactive planner, and detection of situations where
// safe progress is impossible (CS 4800 Project 3 requirements:
// "Detect situations where safe progress is impossible" and
// "Expose navigation state, such as FOLLOWING PATH, AVOIDING OBSTACLE,
//  GOAL REACHED, and BLOCKED").
//
// Implementation: src/navigation_state.cpp
//
// States
// ------
//   WAITING_FOR_PATH   no global path received yet (or odometry not ready)
//   FOLLOWING_PATH     a safe velocity was chosen and nothing is close
//   AVOIDING_OBSTACLE  a safe velocity was chosen but an obstacle is within the
//                      search range (Dist(v) of the chosen candidate < 1)
//   BLOCKED            safe progress is impossible, for either reason below
//   GOAL_REACHED       within goal_tolerance of the last waypoint
//
// When is the drone BLOCKED?
// --------------------------
//   1. No safe velocity: after the Vs/Vd/Va filters and selectBestVelocity(),
//      no candidate is left (BestCandidate::found == false). Every reachable
//      velocity would risk a collision.
//   2. No progress: safe velocities exist, but the remaining distance to the
//      goal has not shrunk by at least min_progress in stall_timeout seconds.
//      This catches the drone hovering or circling in front of an obstacle it
//      cannot get around (a DWA local minimum, e.g. a wall across the path).
//
// While BLOCKED the caller must publish a zero velocity (hover) instead of the
// chosen candidate, so the planner reports the condition rather than
// generating unsafe commands. It leaves BLOCKED automatically:
//   - reason 1 clears as soon as a safe velocity exists again;
//   - reason 2 clears once the drone has made min_progress since it was blocked
//     (for example after a teammate's global planner sends a new path).
//
// This component has no ROS dependencies. The caller passes plain numbers in
// and decides how to report the state (terminal log now, a ROS topic later).
//
// Typical use in ReactiveNavigator::control_loop():
//
//     NavInputs in;
//     in.now = this->now().seconds();
//     in.has_path = true;
//     in.distance_to_goal = /* drone to LAST waypoint */;
//     in.safe_velocity_found = best.found;
//     in.chosen_dist = best.found ? best.candidate.dist : 0.0;
//     if (nav_state_.update(in)) {
//         RCLCPP_INFO(get_logger(), "Navigation state -> %s (%s)",
//                     toString(nav_state_.state()), toString(nav_state_.blockedReason()));
//     }
//     if (nav_state_.mustHover()) { /* publish zero velocity */ }

#pragma once

#include <limits>

namespace reactive_uav_navigation {

enum class NavState {
    WAITING_FOR_PATH,
    FOLLOWING_PATH,
    AVOIDING_OBSTACLE,
    BLOCKED,
    GOAL_REACHED
};

// Why the drone is BLOCKED (NONE in every other state).
enum class BlockedReason { NONE, NO_SAFE_VELOCITY, NO_PROGRESS };

// Text for logging, e.g. "AVOIDING_OBSTACLE".
const char* toString(NavState s);
const char* toString(BlockedReason r);

// Tuning values. Fixed at startup, like DistanceParams and ObjectiveWeights.
struct NavStateParams {
    double goal_tolerance = 0.3;   // m: closer than this to the last waypoint = GOAL_REACHED
    double stall_timeout = 5.0;    // s: no progress for this long = BLOCKED
    double min_progress = 0.2;     // m: how much closer to the goal counts as progress
    // Dist(v) of the chosen candidate below this means an obstacle is near.
    // Dist(v) = 1 means nothing within r_search, so anything below 1 is "near".
    double avoid_enter_dist = 0.95;
    // Dist(v) at or above this switches back to FOLLOWING_PATH. Slightly higher
    // than avoid_enter_dist so the state does not flicker at the boundary.
    double avoid_exit_dist = 0.99;

    // Throws std::invalid_argument if the configuration is unusable.
    void validate() const;
};

// Everything the state machine needs from one control cycle.
struct NavInputs {
    double now = 0.0;                  // s, e.g. node->now().seconds()
    bool has_path = false;             // a global path is loaded and odometry is ready
    double distance_to_goal = 0.0;     // m, from the drone to the LAST waypoint of the path
    bool safe_velocity_found = false;  // BestCandidate::found from selectBestVelocity()
    double chosen_dist = 1.0;          // Dist(v) of the chosen candidate, in [0, 1]
};

class NavigationStateMachine {
public:
    // Throws std::invalid_argument if `params` fails NavStateParams::validate().
    explicit NavigationStateMachine(const NavStateParams& params = NavStateParams());

    // Call once per control cycle. Returns true if the state changed this
    // cycle, so the caller can log transitions without spamming the terminal.
    bool update(const NavInputs& in);

    // Call when a new global path arrives: forgets GOAL_REACHED and the
    // progress history so the new path starts fresh.
    void reset();

    NavState state() const { return state_; }
    BlockedReason blockedReason() const { return reason_; }

    // True when the caller must publish zero velocity instead of a command.
    bool mustHover() const;

    const NavStateParams& params() const { return params_; }

private:
    NavStateParams params_;
    NavState state_ = NavState::WAITING_FOR_PATH;
    BlockedReason reason_ = BlockedReason::NONE;

    // Progress tracking: the best (smallest) distance to goal seen so far and
    // when it last improved by at least min_progress.
    bool progress_started_ = false;
    double best_distance_ = std::numeric_limits<double>::infinity();
    double last_progress_time_ = 0.0;

    NavState decide(const NavInputs& in);
    void updateProgress(const NavInputs& in);
};

}  // namespace reactive_uav_navigation
