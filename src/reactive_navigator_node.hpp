#ifndef REACTIVE_NAVIGATOR_HPP_
#define REACTIVE_NAVIGATOR_HPP_

#include <mutex>
#include <cmath>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "drone_state/drone_state.hpp"

namespace reactive_uav_navigation {

class ReactiveNavigator : public rclcpp::Node
{
public:
  ReactiveNavigator(std::shared_ptr<DroneState> drone_state);
  nav_msgs::msg::Path get_global_path() const;

private:
  void path_callback(const nav_msgs::msg::Path::SharedPtr msg);

  void control_loop();

  // Subscription and variables for handling global path
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
  nav_msgs::msg::Path global_path_;
  mutable std::mutex path_mutex_;
  size_t current_index_{0};

  // Shared object for the drone's current state
  std::shared_ptr<DroneState> drone_state_;



  // Publisher for drone movement commands
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}

#endif  // REACTIVE_NAVIGATOR_HPP_