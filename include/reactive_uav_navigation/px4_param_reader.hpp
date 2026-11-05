// px4_param_reader.hpp
//
// ROS helpers shared by the velocity search spaces (vss_*). Implemented in
// px4_param_reader.cpp, which needs ROS and is left out of the unit tests.

#ifndef PX4_PARAM_READER_HPP_
#define PX4_PARAM_READER_HPP_

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace rclcpp {
class Node;
}

namespace reactive_uav_navigation {

// Reads a double ROS parameter, declaring it with `default_value` first if the
// node does not have it yet.
double rosDoubleParam(rclcpp::Node* node, const std::string& name, double default_value);

// Reads PX4 parameters through the MAVROS service /mavros/param/get, the same
// way VelocityFunction reads MPC_XY_VEL_MAX. A 1 s timer retries each
// parameter until MAVROS answers with a positive value, then reports it once.
// The node must outlive the reader and be spun.
class Px4ParamReader {
public:
    // Called once per parameter, with its name and value as PX4 stores it.
    using Callback = std::function<void(const std::string& name, double value)>;

    Px4ParamReader(rclcpp::Node* node, const std::vector<std::string>& names, Callback on_value);

private:
    struct Impl;
    std::shared_ptr<Impl> impl_;
};

}  // namespace reactive_uav_navigation

#endif  // PX4_PARAM_READER_HPP_
