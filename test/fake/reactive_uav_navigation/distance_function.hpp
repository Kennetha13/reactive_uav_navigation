// Test-only stand-in for distance_function.hpp. The real one needs ROS and PCL
// for ray casting; this one models a single wall at x = wall_x so computeVr can
// be tested without them. Only the part AdmissibleVelocities uses is provided,
// so it must be kept in step with DistanceFunction::computeDistMin().
#pragma once

#include <algorithm>

#include <geometry_msgs/msg/vector3.hpp>

namespace reactive_uav_navigation {

class DistanceFunction {
public:
    explicit DistanceFunction(double wall_x) : wall_x_(wall_x) {}

    // Distance to the wall along +x, capped at the search range. Negative if
    // the wall is behind the position (the real function never returns that,
    // but maxAdmissibleSpeed() treats it as "no clearance", which is the case
    // under test).
    double computeDistMin(const geometry_msgs::msg::Vector3& position,
                          double /*headingYaw*/, double /*headingPitch*/) const {
        return std::min(wall_x_ - position.x, kSearchRange);
    }

private:
    static constexpr double kSearchRange = 10.0;
    double wall_x_;
};

}  // namespace reactive_uav_navigation
