// Test-only stand-in for geometry_msgs/msg/vector3.hpp so the velocity tests
// build without ROS.
#pragma once

namespace geometry_msgs {
namespace msg {
struct Vector3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};
}  // namespace msg
}  // namespace geometry_msgs
