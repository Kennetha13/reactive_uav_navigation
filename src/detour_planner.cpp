#include "reactive_uav_navigation/detour_planner.hpp"

#include <algorithm>
#include <cmath>

namespace reactive_uav_navigation {

// straight line distance between two points
static double distanceBetween(const geometry_msgs::msg::Point& a,
                              const geometry_msgs::msg::Point& b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void DetourPlanner::reset() {
    started_ = false;
    side_stepping_ = false;
    detouring_ = false;
    given_up_ = false;
    obstacle_seen_ = false;
    side_ = 1.0;
    steps_on_side_ = 0;
    sides_tried_ = 0;
    going_around_ = false;
    around_side_ = 0.0;
    nearer_side_ = 0.0;
}

void DetourPlanner::setObstacles(const std::vector<geometry_msgs::msg::Vector3>& points,
                                 double half_view) {
    obstacle_points_ = points;
    half_view_ = half_view;
}

geometry_msgs::msg::Point DetourPlanner::update(double now,
                                                const geometry_msgs::msg::Point& position,
                                                double yaw,
                                                const geometry_msgs::msg::Point& waypoint,
                                                bool obstacle_in_the_way) {
    double distance = distanceBetween(position, waypoint);

    if (!started_) {
        started_ = true;
        best_distance_ = distance;
        stuck_seconds_ = 0.0;
        last_update_time_ = now;
    }

    // time since the last control cycle
    double time_step = now - last_update_time_;
    last_update_time_ = now;

    going_around_ = false;

    if (given_up_) {
        return waypoint;
    }

    // ---------------- on a side step
    if (side_stepping_) {
        // The side step is measured by how far the drone has come to the
        // side, not by how close it is to the target point. When the obstacle
        // is not exactly square to the way to the waypoint, the target point
        // lies a little inside it and can never be reached.
        double side_distance = side_reach_ - sideProgress(position);
        if (side_distance < side_best_distance_ - params.min_progress) {
            side_best_distance_ = side_distance;
            side_progress_time_ = now;
        }

        bool reached = side_distance < params.reach_distance;
        bool stuck = now - side_progress_time_ > params.stuck_time;
        if (!reached && !stuck) {
            return side_target_;
        }

        // the side step is over: try the waypoint again from here
        side_stepping_ = false;
        best_distance_ = distance;
        stuck_seconds_ = 0.0;

        // this side goes no further, so the next side step uses the other one
        if (stuck) {
            switchSide();
        }
        return waypoint;
    }

    // ---------------- heading for the waypoint

    // angle from the drone's heading to the waypoint, wrapped into [-pi, pi]
    double yaw_error = std::atan2(waypoint.y - position.y, waypoint.x - position.x) - yaw;
    while (yaw_error > M_PI) {
        yaw_error = yaw_error - 2.0 * M_PI;
    }
    while (yaw_error < -M_PI) {
        yaw_error = yaw_error + 2.0 * M_PI;
    }

    // While the drone is still turning toward the waypoint it cannot get
    // closer yet, so only the time it spends facing the waypoint counts as
    // being stuck.
    bool facing_waypoint = std::fabs(yaw_error) <= M_PI / 2.0;
    if (facing_waypoint) {
        stuck_seconds_ += time_step;
    }

    // remember when an obstacle was last in the way. Once the drone has
    // turned aside it no longer sees the obstacle that stopped it.
    if (obstacle_in_the_way) {
        obstacle_seen_ = true;
        last_obstacle_time_ = now;
    }

    if (distance < best_distance_ - params.min_progress) {
        best_distance_ = distance;
        stuck_seconds_ = 0.0;

        // closer than when the detour began: the way around was found
        if (detouring_ && distance < detour_start_distance_ - params.min_progress) {
            detouring_ = false;
            side_ = 1.0;
            steps_on_side_ = 0;
            sides_tried_ = 0;
        }
    }

    // an obstacle on the straight line to the waypoint whose end is in sight
    geometry_msgs::msg::Point around;
    bool way_around = findWayAround(position, yaw, waypoint, around);

    // the drone is getting no closer and an obstacle was in the way recently
    bool stuck = stuck_seconds_ > params.stuck_time;
    bool obstacle_recently = obstacle_seen_ && (now - last_obstacle_time_ <= params.obstacle_memory_time);
    if (stuck && obstacle_recently) {
        startSideStep(now, position, waypoint, distance);
        if (side_stepping_) {
            return side_target_;
        }
    }

    if (way_around) {
        going_around_ = true;
        return around;
    }

    return waypoint;
}

bool DetourPlanner::findWayAround(const geometry_msgs::msg::Point& position,
                                  double yaw,
                                  const geometry_msgs::msg::Point& waypoint,
                                  geometry_msgs::msg::Point& around) {
    // direction to the waypoint seen from above, and the direction to its right
    double dx = waypoint.x - position.x;
    double dy = waypoint.y - position.y;
    double length = std::hypot(dx, dy);
    if (length < 1e-6) {
        around_side_ = 0.0;
        nearer_side_ = 0.0;
        return false;
    }
    dx = dx / length;
    dy = dy / length;
    double right_x = dy;
    double right_y = -dx;

    // only points near the height the drone flies at can be in its way
    double lowest = std::min(position.z, waypoint.z) - params.block_distance;
    double highest = std::max(position.z, waypoint.z) + params.block_distance;

    // A point closer to the straight line than this is in the way. Once the
    // drone is going around an obstacle it keeps on until the obstacle is
    // pass_distance away, so that it does not turn back toward it too early.
    double in_the_way_distance = params.block_distance;
    if (around_side_ != 0.0) {
        in_the_way_distance = params.pass_distance;
    }

    // For every point between the drone and the waypoint: how far it lies to
    // the right of the straight line to the waypoint (negative: to the left).
    std::vector<double> sides;
    bool in_the_way = false;
    double left_end = 0.0;
    double right_end = 0.0;
    double obstacle_ahead = length;
    for (size_t i = 0; i < obstacle_points_.size(); i++) {
        if (obstacle_points_[i].z < lowest || obstacle_points_[i].z > highest) {
            continue;
        }

        double from_x = obstacle_points_[i].x - position.x;
        double from_y = obstacle_points_[i].y - position.y;
        double ahead = from_x * dx + from_y * dy;
        double side = from_x * right_x + from_y * right_y;
        if (ahead <= 0.0 || ahead >= length) {
            continue;
        }
        sides.push_back(side);

        if (std::fabs(side) < in_the_way_distance) {
            if (!in_the_way || side < left_end) {
                left_end = side;
            }
            if (!in_the_way || side > right_end) {
                right_end = side;
            }
            if (ahead < obstacle_ahead) {
                obstacle_ahead = ahead;
            }
            in_the_way = true;
        }
    }

    if (!in_the_way) {
        around_side_ = 0.0;
        nearer_side_ = 0.0;
        return false;
    }

    // The obstacle reaches as far to each side as its points lie close
    // together. It ends where the next point is so far away that the drone
    // fits in between.
    double gap = params.pass_distance + params.block_distance;
    std::sort(sides.begin(), sides.end());
    for (size_t i = 0; i < sides.size(); i++) {
        if (sides[i] > right_end && sides[i] - right_end < gap) {
            right_end = sides[i];
        }
    }
    for (size_t i = sides.size(); i > 0; i--) {
        if (sides[i - 1] < left_end && left_end - sides[i - 1] < gap) {
            left_end = sides[i - 1];
        }
    }

    // the points to aim at lie pass_distance past each end
    double right_pass = right_end + params.pass_distance;
    double left_pass = left_end - params.pass_distance;

    // the side where the obstacle ends sooner
    nearer_side_ = 1.0;
    if (std::fabs(left_pass) < std::fabs(right_pass)) {
        nearer_side_ = -1.0;
    }

    // The aiming point is level with the front of the obstacle, so the drone
    // is pass_distance to the side by the time it gets there. It is never
    // right beside the drone, which would mean a sharp turn.
    double aim_ahead = std::max(obstacle_ahead, params.min_aim_ahead);
    double right_aim_x = position.x + dx * aim_ahead + right_x * right_pass;
    double right_aim_y = position.y + dy * aim_ahead + right_y * right_pass;
    double left_aim_x = position.x + dx * aim_ahead + right_x * left_pass;
    double left_aim_y = position.y + dy * aim_ahead + right_y * left_pass;

    bool right_possible = canHeadFor(position, yaw, right_aim_x, right_aim_y,
                                     right_pass, lowest, highest);
    bool left_possible = canHeadFor(position, yaw, left_aim_x, left_aim_y,
                                    left_pass, lowest, highest);

    // stay on the side chosen before, otherwise take the nearer one
    if (around_side_ > 0.0 && right_possible) {
        around_side_ = 1.0;
    } else if (around_side_ < 0.0 && left_possible) {
        around_side_ = -1.0;
    } else if (right_possible && left_possible) {
        around_side_ = nearer_side_;
    } else if (right_possible) {
        around_side_ = 1.0;
    } else if (left_possible) {
        around_side_ = -1.0;
    } else {
        around_side_ = 0.0;
        return false;
    }

    if (around_side_ > 0.0) {
        around.x = right_aim_x;
        around.y = right_aim_y;
    } else {
        around.x = left_aim_x;
        around.y = left_aim_y;
    }
    around.z = waypoint.z;
    return true;
}

bool DetourPlanner::canHeadFor(const geometry_msgs::msg::Point& position, double yaw,
                               double x, double y, double side_distance,
                               double lowest, double highest) const {
    // an end this far to the side belongs to a wall, the side steps deal with it
    if (std::fabs(side_distance) > params.max_side) {
        return false;
    }

    double dx = x - position.x;
    double dy = y - position.y;
    double length = std::hypot(dx, dy);
    if (length < 1e-6) {
        return false;
    }
    dx = dx / length;
    dy = dy / length;

    // The camera must see that way. Where it has not looked, no points are
    // known, and that does not mean the space is free.
    double yaw_error = std::atan2(dy, dx) - yaw;
    while (yaw_error > M_PI) {
        yaw_error = yaw_error - 2.0 * M_PI;
    }
    while (yaw_error < -M_PI) {
        yaw_error = yaw_error + 2.0 * M_PI;
    }
    if (std::fabs(yaw_error) > half_view_) {
        return false;
    }

    // no point may lie on the straight way from the drone to (x, y)
    for (size_t i = 0; i < obstacle_points_.size(); i++) {
        if (obstacle_points_[i].z < lowest || obstacle_points_[i].z > highest) {
            continue;
        }

        double from_x = obstacle_points_[i].x - position.x;
        double from_y = obstacle_points_[i].y - position.y;

        // the spot on the way that is closest to the point
        double along = from_x * dx + from_y * dy;
        if (along < 0.0) {
            along = 0.0;
        }
        if (along > length) {
            along = length;
        }

        double off_x = from_x - along * dx;
        double off_y = from_y - along * dy;
        if (std::hypot(off_x, off_y) < params.free_width) {
            return false;
        }
    }

    return true;
}

void DetourPlanner::startSideStep(double now,
                                  const geometry_msgs::msg::Point& position,
                                  const geometry_msgs::msg::Point& waypoint,
                                  double distance_to_waypoint) {
    if (!detouring_) {
        // direction to the waypoint, seen from above
        double dx = waypoint.x - position.x;
        double dy = waypoint.y - position.y;
        double length = std::hypot(dx, dy);
        if (length < 1e-6) {
            return;
        }

        // Every side step of this detour is measured from this spot, along
        // the line that crosses the way to the waypoint at a right angle.
        // An obstacle that lies across the way runs along that line too, so
        // the side steps follow it instead of turning into it.
        detouring_ = true;
        detour_start_distance_ = distance_to_waypoint;
        detour_start_ = position;
        detour_dx_ = dx / length;
        detour_dy_ = dy / length;

        // the first side steps go to the side where the obstacle ends sooner
        if (nearer_side_ != 0.0) {
            side_ = nearer_side_;
        }
    }

    if (steps_on_side_ >= params.max_steps_per_side) {
        switchSide();
    }
    if (given_up_) {
        return;
    }

    // each side step reaches one step_length further out than the one before
    steps_on_side_++;
    side_reach_ = steps_on_side_ * params.step_length;

    // The side steps run along a line a little behind the spot where the
    // drone got stuck. From further back the camera sees more of the
    // obstacle, and the drone has room to turn into an opening.
    double line_x = detour_start_.x - detour_dx_ * params.back_distance;
    double line_y = detour_start_.y - detour_dy_ * params.back_distance;

    // a quarter turn to the right of the direction (dx, dy) is (dy, -dx)
    side_target_.x = line_x + side_ * detour_dy_ * side_reach_;
    side_target_.y = line_y - side_ * detour_dx_ * side_reach_;
    side_target_.z = waypoint.z;

    side_stepping_ = true;
    side_best_distance_ = side_reach_ - sideProgress(position);
    side_progress_time_ = now;
}

double DetourPlanner::sideProgress(const geometry_msgs::msg::Point& position) const {
    // the direction of the side steps on the current side
    double side_x = side_ * detour_dy_;
    double side_y = -side_ * detour_dx_;

    // how much of the way from the start of the detour lies in that direction
    return (position.x - detour_start_.x) * side_x + (position.y - detour_start_.y) * side_y;
}

void DetourPlanner::switchSide() {
    sides_tried_++;

    // both sides tried: nothing more to do
    if (sides_tried_ >= 2) {
        given_up_ = true;
        detouring_ = false;
        side_stepping_ = false;
        return;
    }

    side_ = -side_;
    steps_on_side_ = 0;
}

}  // namespace reactive_uav_navigation
