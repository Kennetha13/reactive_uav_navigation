#ifndef REACTIVE_NAVIGATOR_HPP_
#define REACTIVE_NAVIGATOR_HPP_

#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"

class ReactiveNavigator : public rclcpp::Node
{
public:
  ReactiveNavigator() : Node("reactive_navigator")
  {
    // QoS (Quality of Service) to keep history queue of last 10 messages
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    // We are handling the interface nav_msgs/msg/path (this is how the global planner is defined)
    path_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
      "planned_path", // topic name
      qos,
      std::bind(&ReactiveNavigator::path_callback, this, std::placeholders::_1) // bind the current object instance to a function, which will handle the incoming arguments 
    );
    RCLCPP_INFO(this->get_logger(), "Listening for global path...");
  }

  // Accessor method for the latest global path
  nav_msgs::msg::Path get_global_path() const
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    return global_path_;
  }

private:
  // Removed 'const' because this method updates the global_path_ member variable
  void path_callback(const nav_msgs::msg::Path::SharedPtr msg) 
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Received path with frame_id: '%s' containing %zu poses.",
      msg->header.frame_id.c_str(),
      msg->poses.size()
    );

    {
      std::lock_guard<std::mutex> lock(path_mutex_);
      global_path_ = *msg;
    }
  }

  // Subscription and variables for handling global path
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
  nav_msgs::msg::Path global_path_; // Fixed single colon to double colon (::Path)
  mutable std::mutex path_mutex_;
};

#endif  // REACTIVE_NAVIGATOR_HPP_