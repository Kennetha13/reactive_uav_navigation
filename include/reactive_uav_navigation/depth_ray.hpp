#ifndef DEPTH_RAY_HPP_
#define DEPTH_RAY_HPP_

#include <cstdint>
#include <mutex>
#include <optional>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace reactive_uav_navigation {

// Subscribes to the depth camera and turns its latest image into obstacle
// points in the world frame. The points are handed to DistanceFunction, which
// casts its rays against them.
//
// Frames
// ------
// A depth pixel is first a point in the camera optical frame:
//   +X right, +Y down, +Z forward.
// The camera looks straight ahead of the drone, so in the drone's body frame
// (+X forward, +Y left, +Z up) the same point is
//   body_x = camera_z, body_y = -camera_x, body_z = -camera_y.
// The drone's position and orientation then place it in the world frame.
class DepthRayProcessor
{
public:
  explicit DepthRayProcessor(rclcpp::Node *node);

  // true once a depth image and the camera calibration have both arrived
  bool hasImage() const;

  // true if the latest depth image arrived no more than max_age seconds ago.
  // An old image shows where obstacles were, not where they are.
  bool hasFreshImage(double max_age) const;

  // Obstacle points seen in the latest depth image, in the world frame.
  // Points further than max_range from the camera are left out.
  std::vector<geometry_msgs::msg::Vector3> makeWorldPoints(
    const geometry_msgs::msg::Point &drone_position,
    const geometry_msgs::msg::Quaternion &drone_orientation,
    double max_range) const;

  // Horizontal field of view of the camera in radians, 0 until the camera
  // calibration has arrived.
  double horizontalFov() const;

  // Vertical field of view of the camera in radians, 0 until the camera
  // calibration has arrived.
  double verticalFov() const;

  void update_depth_image(
    sensor_msgs::msg::Image::ConstSharedPtr image);

  void update_camera_info(
    sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info);

private:
  std::optional<float> read_depth_meters(
    const sensor_msgs::msg::Image &image,
    int u,
    int v) const;

  rclcpp::Node *node_;

  // when the latest depth image arrived, in seconds of the node's clock
  double last_image_time_{0.0};

  mutable std::mutex mutex_;
  sensor_msgs::msg::Image latest_depth_image_;
  sensor_msgs::msg::CameraInfo latest_camera_info_;
  bool depth_image_received_{false};
  bool camera_info_received_{false};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
};

}  // namespace reactive_uav_navigation

#endif  // DEPTH_RAY_HPP_
