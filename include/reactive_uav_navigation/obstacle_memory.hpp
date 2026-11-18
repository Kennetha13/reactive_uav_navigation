// obstacle_memory.hpp
//
// Remembers obstacle points for a short time after they leave the camera's
// view. Without this an obstacle is forgotten the moment the drone turns away
// from it, and the drone could then swing its side into it.
//
// Points inside the current view are always replaced by what the camera sees
// now, so an obstacle that moved away does not stay behind as a ghost.
// Points outside the view are kept for memory_time seconds.
//
// No ROS dependencies. Points are in the world frame.

#ifndef OBSTACLE_MEMORY_HPP_
#define OBSTACLE_MEMORY_HPP_

#include <vector>
#include <geometry_msgs/msg/vector3.hpp>

namespace reactive_uav_navigation {

class ObstacleMemory {
public:
    // how long a point outside the view is remembered (s). Long enough to
    // still know a corner while the drone flies around it.
    double memory_time = 15.0;

    // Call once per control cycle.
    //   seen_points: obstacle points the camera sees right now
    //   now:         current time (s)
    //   x, y, z:     drone position
    //   view_yaw:    direction the camera looks in (rad)
    //   half_fov:    half of the camera's horizontal field of view (rad)
    //   half_vertical_fov: half of the camera's vertical field of view (rad)
    void update(const std::vector<geometry_msgs::msg::Vector3>& seen_points,
                double now, double x, double y, double z,
                double view_yaw, double half_fov, double half_vertical_fov);

    // the points seen now plus the remembered ones
    const std::vector<geometry_msgs::msg::Vector3>& points() const;

    // forgets everything
    void clear();

private:
    std::vector<geometry_msgs::msg::Vector3> points_;

    // when each point in points_ was last seen
    std::vector<double> seen_times_;
};

}  // namespace reactive_uav_navigation

#endif  // OBSTACLE_MEMORY_HPP_
