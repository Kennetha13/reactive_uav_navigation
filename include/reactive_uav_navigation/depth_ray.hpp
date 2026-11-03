#ifndef DEPTH_RAY_HPP_
#define DEPTH_RAY_HPP_

#include <cstdint>
#include <mutex>
#include <optional>

#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace reactive_uav_navigation {

class DepthRayProcessor
{
public:
  // The direction passed to castRay is in the camera optical frame:
  // +X right, +Y down, +Z forward.
  std::optional<float> castRay(
    const geometry_msgs::msg::Vector3 &direction,
    double max_distance) const;

  void update_depth_image(
    const sensor_msgs::msg::Image::SharedPtr &image);

  void update_camera_info(
    const sensor_msgs::msg::CameraInfo::SharedPtr &camera_info);

private:
  std::optional<float> read_depth_meters(
    const sensor_msgs::msg::Image &image,
    int u,
    int v) const;

  mutable std::mutex mutex_;
  sensor_msgs::msg::Image latest_depth_image_;
  sensor_msgs::msg::CameraInfo latest_camera_info_;
  bool depth_image_received_{false};
  bool camera_info_received_{false};
  int patch_radius_{2};
};

}  // namespace reactive_uav_navigation

#endif  // DEPTH_RAY_HPP_
