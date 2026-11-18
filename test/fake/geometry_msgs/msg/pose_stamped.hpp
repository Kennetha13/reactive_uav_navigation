// Test-only stand-in for geometry_msgs/msg/pose_stamped.hpp so the navigation
// tests build without ROS.
#pragma once

#include <geometry_msgs/msg/point.hpp>

namespace geometry_msgs {
namespace msg {
struct Pose {
    Point position;
};
struct PoseStamped {
    Pose pose;
};
}  // namespace msg
}  // namespace geometry_msgs
