#include "reactive_uav_navigation/navigation_state.hpp"

#include <cmath>
#include <stdexcept>

namespace reactive_uav_navigation {

double remainingPathLength(const std::vector<geometry_msgs::msg::PoseStamped>& waypoints,
                           std::size_t current_index,
                           double x, double y, double z) {
    if (current_index >= waypoints.size()) {
        return 0.0;
    }
    const auto& next = waypoints[current_index].pose.position;
    double total = std::sqrt((next.x - x) * (next.x - x) +
                             (next.y - y) * (next.y - y) +
                             (next.z - z) * (next.z - z));
    for (std::size_t i = current_index; i + 1 < waypoints.size(); ++i) {
        const auto& a = waypoints[i].pose.position;
        const auto& b = waypoints[i + 1].pose.position;
        total += std::sqrt((b.x - a.x) * (b.x - a.x) +
                           (b.y - a.y) * (b.y - a.y) +
                           (b.z - a.z) * (b.z - a.z));
    }
    return total;
}

const char* toString(NavState s) {
    switch (s) {
        case NavState::WAITING_FOR_PATH:  return "WAITING_FOR_PATH";
        case NavState::FOLLOWING_PATH:    return "FOLLOWING_PATH";
        case NavState::AVOIDING_OBSTACLE: return "AVOIDING_OBSTACLE";
        case NavState::BLOCKED:           return "BLOCKED";
        case NavState::GOAL_REACHED:      return "GOAL_REACHED";
    }
    return "UNKNOWN";
}

const char* toString(BlockedReason r) {
    switch (r) {
        case BlockedReason::NONE:             return "none";
        case BlockedReason::NO_SAFE_VELOCITY: return "no safe velocity: every reachable velocity risks a collision";
        case BlockedReason::NO_PROGRESS:      return "no progress toward the goal";
    }
    return "unknown";
}

void NavStateParams::validate() const {
    if (goal_tolerance <= 0.0) throw std::invalid_argument("goal_tolerance must be > 0");
    if (stall_timeout <= 0.0) throw std::invalid_argument("stall_timeout must be > 0");
    if (min_progress <= 0.0) throw std::invalid_argument("min_progress must be > 0");
    if (avoid_enter_dist < 0.0 || avoid_exit_dist > 1.0 || avoid_enter_dist > avoid_exit_dist) {
        throw std::invalid_argument("need 0 <= avoid_enter_dist <= avoid_exit_dist <= 1");
    }
}

NavigationStateMachine::NavigationStateMachine(const NavStateParams& params)
    : params_(params) {
    params_.validate();
}

bool NavigationStateMachine::update(const NavInputs& in) {
    const NavState before = state_;
    state_ = decide(in);
    return state_ != before;
}

void NavigationStateMachine::reset() {
    state_ = NavState::WAITING_FOR_PATH;
    reason_ = BlockedReason::NONE;
    progress_started_ = false;
}

bool NavigationStateMachine::mustHover() const {
    // NO_PROGRESS is still reported as BLOCKED, but the drone keeps flying the
    // best velocity: hovering would guarantee no progress, so the state could
    // never clear. Only "no safe velocity" (collision risk) forces a hover.
    if (state_ == NavState::BLOCKED) {
        return reason_ == BlockedReason::NO_SAFE_VELOCITY;
    }
    return state_ == NavState::GOAL_REACHED || state_ == NavState::WAITING_FOR_PATH;
}

NavState NavigationStateMachine::decide(const NavInputs& in) {
    reason_ = BlockedReason::NONE;

    if (!in.has_path) {
        progress_started_ = false;
        return NavState::WAITING_FOR_PATH;
    }

    // GOAL_REACHED is latched until reset() (a new path).
    if (state_ == NavState::GOAL_REACHED) {
        return NavState::GOAL_REACHED;
    }
    if (in.distance_to_goal < params_.goal_tolerance) {
        return NavState::GOAL_REACHED;
    }

    updateProgress(in);

    // Blocked reason 1: nothing safe to do.
    if (!in.safe_velocity_found) {
        reason_ = BlockedReason::NO_SAFE_VELOCITY;
        return NavState::BLOCKED;
    }

    // Blocked reason 2: safe velocities exist but we are getting nowhere.
    if (in.now - last_progress_time_ > params_.stall_timeout) {
        reason_ = BlockedReason::NO_PROGRESS;
        return NavState::BLOCKED;
    }

    // Moving safely: near an obstacle or not? Two thresholds (hysteresis)
    // keep the state from flickering when Dist(v) hovers near one value.
    const bool was_avoiding = (state_ == NavState::AVOIDING_OBSTACLE);
    const double threshold = was_avoiding ? params_.avoid_exit_dist : params_.avoid_enter_dist;
    return (in.chosen_dist < threshold) ? NavState::AVOIDING_OBSTACLE
                                        : NavState::FOLLOWING_PATH;
}

void NavigationStateMachine::updateProgress(const NavInputs& in) {
    if (!progress_started_) {
        progress_started_ = true;
        best_distance_ = in.remaining_path_length;
        last_progress_time_ = in.now;
        return;
    }
    if (in.remaining_path_length < best_distance_ - params_.min_progress) {
        best_distance_ = in.remaining_path_length;
        last_progress_time_ = in.now;
    }
}

}  // namespace reactive_uav_navigation
