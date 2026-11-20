#include "reactive_uav_navigation/obstacle_memory.hpp"

#include <cmath>

namespace reactive_uav_navigation {

// the most remembered points kept, so the control loop stays fast
const size_t MAX_REMEMBERED_POINTS = 600;

void ObstacleMemory::update(const std::vector<geometry_msgs::msg::Vector3>& seen_points,
                            double now, double x, double y, double z,
                            double view_yaw, double half_fov, double half_vertical_fov) {
    std::vector<geometry_msgs::msg::Vector3> kept_points;
    std::vector<double> kept_times;

    // keep the old points that are outside the view and not too old
    for (size_t i = 0; i < points_.size(); i++) {
        if (now - seen_times_[i] > memory_time) {
            continue;
        }

        // angle between the point and the middle of the view, wrapped into [-pi, pi]
        double point_yaw = std::atan2(points_[i].y - y, points_[i].x - x);
        double difference = point_yaw - view_yaw;
        while (difference > M_PI) {
            difference = difference - 2.0 * M_PI;
        }
        while (difference < -M_PI) {
            difference = difference + 2.0 * M_PI;
        }

        // angle of the point above or below the camera
        double horizontal_distance = std::hypot(points_[i].x - x, points_[i].y - y);
        double point_pitch = std::atan2(points_[i].z - z, horizontal_distance);

        // inside the view: the camera sees this spot now, so the fresh points replace it
        if (std::fabs(difference) <= half_fov && std::fabs(point_pitch) <= half_vertical_fov) {
            continue;
        }

        kept_points.push_back(points_[i]);
        kept_times.push_back(seen_times_[i]);
    }

    // Too many: the oldest points, at the front, are dropped. The newest ones
    // belong to what the drone has just turned away from, such as an obstacle
    // it is going around.
    if (kept_points.size() > MAX_REMEMBERED_POINTS) {
        size_t too_many = kept_points.size() - MAX_REMEMBERED_POINTS;
        kept_points.erase(kept_points.begin(), kept_points.begin() + too_many);
        kept_times.erase(kept_times.begin(), kept_times.begin() + too_many);
    }

    // add everything the camera sees now
    for (size_t i = 0; i < seen_points.size(); i++) {
        kept_points.push_back(seen_points[i]);
        kept_times.push_back(now);
    }

    points_ = kept_points;
    seen_times_ = kept_times;
}

const std::vector<geometry_msgs::msg::Vector3>& ObstacleMemory::points() const {
    return points_;
}

void ObstacleMemory::clear() {
    points_.clear();
    seen_times_.clear();
}

}  // namespace reactive_uav_navigation
