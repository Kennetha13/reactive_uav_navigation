#ifndef REACTIVE_NAVIGATOR_HPP_
#define REACTIVE_NAVIGATOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"

class ReactiveNavigator : public rclcpp::Node
{
public:
  ReactiveNavigator() : Node("reactive_navigator")
  {
    
  }
};

#endif  // REACTIVE_NAVIGATOR_HPP_