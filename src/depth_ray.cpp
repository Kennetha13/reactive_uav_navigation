#include "reactive_uav_navigation/depth_ray.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace reactive_uav_navigation {

std::optional<float> DepthRayProcessor::castRay(
  const geometry_msgs::msg::Vector3 &direction,
  double max_distance) const
{
  sensor_msgs::msg::Image image;
  sensor_msgs::msg::CameraInfo camera_info;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!depth_image_received_ || !camera_info_received_) {
      return std::nullopt;
    }
    image = latest_depth_image_;
    camera_info = latest_camera_info_;
  }

  if (image.width == 0 || image.height == 0 || max_distance <= 0.0) {
    return std::nullopt;
  }

  const double fx = camera_info.k[0];
  const double fy = camera_info.k[4];
  const double cx = camera_info.k[2];
  const double cy = camera_info.k[5];

  if (fx <= 0.0 || fy <= 0.0) {
    return std::nullopt;
  }

  const double magnitude = std::sqrt(
    direction.x * direction.x +
    direction.y * direction.y +
    direction.z * direction.z);

  if (magnitude < 1e-6) {
    return std::nullopt;
  }

  const double camera_x = direction.x / magnitude;
  const double camera_y = direction.y / magnitude;
  const double camera_z = direction.z / magnitude;

  if (camera_z <= 0.0) {
    return std::nullopt;
  }

  const int center_u = static_cast<int>(std::round(
    fx * camera_x / camera_z + cx));
  const int center_v = static_cast<int>(std::round(
    fy * camera_y / camera_z + cy));

  const int radius = std::max(
    0,
    patch_radius_);

  std::optional<float> closest_distance;

  for (int v = center_v - radius; v <= center_v + radius; ++v) {
    for (int u = center_u - radius; u <= center_u + radius; ++u) {
      if (u < 0 || v < 0 ||
        u >= static_cast<int>(image.width) ||
        v >= static_cast<int>(image.height)) {
        continue;
      }

      const auto depth = read_depth_meters(image, u, v);
      if (!depth.has_value() || !std::isfinite(depth.value()) ||
        depth.value() <= 0.0 || depth.value() > max_distance) {
        continue;
      }

      const double z = depth.value();
      const double x = (static_cast<double>(u) - cx) * z / fx;
      const double y = (static_cast<double>(v) - cy) * z / fy;
      const float distance = static_cast<float>(std::sqrt(x * x + y * y + z * z));

      if (distance <= max_distance &&
        (!closest_distance.has_value() || distance < closest_distance.value())) {
        closest_distance = distance;
      }
    }
  }

  return closest_distance;
}

void DepthRayProcessor::update_depth_image(
  const sensor_msgs::msg::Image::SharedPtr &image)
{
  if (!image) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  latest_depth_image_ = *image;
  depth_image_received_ = true;
}

void DepthRayProcessor::update_camera_info(
  const sensor_msgs::msg::CameraInfo::SharedPtr &camera_info)
{
  if (!camera_info) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  latest_camera_info_ = *camera_info;
  camera_info_received_ = true;
}

std::optional<float> DepthRayProcessor::read_depth_meters(
  const sensor_msgs::msg::Image &image,
  int u,
  int v) const
{
  const std::size_t bytes_per_pixel =
    image.encoding == "32FC1" ? sizeof(float) : sizeof(std::uint16_t);
  const std::size_t offset =
    static_cast<std::size_t>(v) * image.step +
    static_cast<std::size_t>(u) * bytes_per_pixel;

  if (offset + bytes_per_pixel > image.data.size()) {
    return std::nullopt;
  }

  const std::uint8_t *row = image.data.data() + v * image.step;

  if (image.encoding == "32FC1") {
    float value;
    std::memcpy(&value, row + u * sizeof(float), sizeof(float));
    return value;
  }

  if (image.encoding == "16UC1") {
    std::uint16_t value;
    std::memcpy(&value, row + u * sizeof(std::uint16_t), sizeof(value));
    return static_cast<float>(value) / 1000.0f;
  }

  return std::nullopt;
}

}  // namespace reactive_uav_navigation
