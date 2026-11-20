// Test-only stand-in for drone_state.hpp. The real one reads the pose from
// MAVROS odometry; this one is set directly by the simulation test.
#pragma once

#include <geometry_msgs/msg/point.hpp>

class DroneState {
public:
    void setPose(double x, double y, double z, double yaw) {
        position_.x = x;
        position_.y = y;
        position_.z = z;
        yaw_ = yaw;
    }

    geometry_msgs::msg::Point get_current_position() const { return position_; }
    double get_yaw() const { return yaw_; }

private:
    geometry_msgs::msg::Point position_;
    double yaw_ = 0.0;
};
