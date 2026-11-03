#ifndef REACTIVE_NAVIGATOR_HPP_
#define REACTIVE_NAVIGATOR_HPP_

#include <mutex>
#include <cmath>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float32.hpp"
#include "reactive_uav_navigation/common.hpp"
#include "reactive_uav_navigation/depth_ray.hpp"
#include "reactive_uav_navigation/drone_state.hpp"

namespace reactive_uav_navigation {

class ReactiveNavigator : public rclcpp::Node
{
public:
  ReactiveNavigator();
  nav_msgs::msg::Path get_global_path() const;

private:
  void path_callback(const nav_msgs::msg::Path::SharedPtr msg);

  void control_loop();

  void depth_image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  void camera_info_callback(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg);

  // Subscription and variables for handling global path
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
  nav_msgs::msg::Path global_path_;
  mutable std::mutex path_mutex_;
  size_t current_index_{0};

  // Shared object for the drone's current state
  std::shared_ptr<DroneState> drone_state_;

  DepthRayProcessor depth_ray_processor_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr obstacle_distance_publisher_;
  std::string depth_topic_;
  std::string camera_info_topic_;
  double depth_ray_max_distance_{15.0};

  // Publisher for drone movement commands
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}

#endif  // REACTIVE_NAVIGATOR_HPP_
