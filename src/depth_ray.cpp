#include "reactive_uav_navigation/depth_ray.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"

namespace reactive_uav_navigation {

// the depth image is reduced to at most GRID_COLUMNS x GRID_ROWS points
const int GRID_COLUMNS = 20;
const int GRID_ROWS = 15;

// inside a grid cell every PIXEL_STEP-th pixel is looked at
const int PIXEL_STEP = 4;

// closer readings than this are the drone's own body
const double MIN_DEPTH = 0.2;

DepthRayProcessor::DepthRayProcessor(rclcpp::Node *node)
: node_(node)
{
  const auto depth_topic = node->declare_parameter<std::string>(
    "depth_topic", "/depth_camera");
  const auto camera_info_topic = node->declare_parameter<std::string>(
    "camera_info_topic", "/camera_info");

  rclcpp::QoS camera_qos(rclcpp::KeepLast(10));
  camera_qos.reliable();
  camera_qos.durability_volatile();

  depth_subscription_ = node->create_subscription<sensor_msgs::msg::Image>(
    depth_topic,
    camera_qos,
    std::bind(
      &DepthRayProcessor::update_depth_image,
      this,
      std::placeholders::_1));

  camera_info_subscription_ =
    node->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic,
      camera_qos,
      std::bind(
        &DepthRayProcessor::update_camera_info,
        this,
        std::placeholders::_1));
}

bool DepthRayProcessor::hasImage() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return depth_image_received_ && camera_info_received_;
}

bool DepthRayProcessor::hasFreshImage(double max_age) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!depth_image_received_ || !camera_info_received_) {
    return false;
  }

  const double age = node_->now().seconds() - last_image_time_;
  return age <= max_age;
}

double DepthRayProcessor::horizontalFov() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!camera_info_received_) {
    return 0.0;
  }

  const double fx = latest_camera_info_.k[0];
  const double width = latest_camera_info_.width;
  if (fx <= 0.0 || width <= 0.0) {
    return 0.0;
  }

  // half of the image width seen from the focal length, times two
  return 2.0 * std::atan((width / 2.0) / fx);
}

double DepthRayProcessor::verticalFov() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!camera_info_received_) {
    return 0.0;
  }

  const double fy = latest_camera_info_.k[4];
  const double height = latest_camera_info_.height;
  if (fy <= 0.0 || height <= 0.0) {
    return 0.0;
  }

  // half of the image height seen from the focal length, times two
  return 2.0 * std::atan((height / 2.0) / fy);
}

std::vector<geometry_msgs::msg::Vector3> DepthRayProcessor::makeWorldPoints(
  const geometry_msgs::msg::Point &drone_position,
  const geometry_msgs::msg::Quaternion &drone_orientation,
  double max_range) const
{
  std::vector<geometry_msgs::msg::Vector3> points;

  // the image is only read here, so keep the lock instead of copying the image
  std::lock_guard<std::mutex> lock(mutex_);
  if (!depth_image_received_ || !camera_info_received_) {
    return points;
  }

  const sensor_msgs::msg::Image &image = latest_depth_image_;
  const double fx = latest_camera_info_.k[0];
  const double fy = latest_camera_info_.k[4];
  const double cx = latest_camera_info_.k[2];
  const double cy = latest_camera_info_.k[5];

  if (image.width == 0 || image.height == 0 || fx <= 0.0 || fy <= 0.0) {
    return points;
  }

  // the image is split into a grid of cells and each cell gives one point,
  // which keeps the number of points small enough for the control loop
  const int width = static_cast<int>(image.width);
  const int height = static_cast<int>(image.height);
  const int cell_width = std::max(1, width / GRID_COLUMNS);
  const int cell_height = std::max(1, height / GRID_ROWS);

  const tf2::Quaternion rotation(
    drone_orientation.x, drone_orientation.y,
    drone_orientation.z, drone_orientation.w);

  for (int cell_v = 0; cell_v < height; cell_v += cell_height) {
    for (int cell_u = 0; cell_u < width; cell_u += cell_width) {
      // find the closest valid depth inside this cell
      bool found = false;
      double closest_depth = 0.0;
      int closest_u = 0;
      int closest_v = 0;

      for (int v = cell_v; v < cell_v + cell_height && v < height; v += PIXEL_STEP) {
        for (int u = cell_u; u < cell_u + cell_width && u < width; u += PIXEL_STEP) {
          const auto depth = read_depth_meters(image, u, v);
          if (!depth.has_value() || !std::isfinite(depth.value()) ||
            depth.value() < MIN_DEPTH || depth.value() > max_range) {
            continue;
          }
          if (!found || depth.value() < closest_depth) {
            found = true;
            closest_depth = depth.value();
            closest_u = u;
            closest_v = v;
          }
        }
      }

      if (!found) {
        continue;
      }

      // pixel and depth to a point in the camera optical frame
      const double camera_z = closest_depth;
      const double camera_x = (static_cast<double>(closest_u) - cx) * camera_z / fx;
      const double camera_y = (static_cast<double>(closest_v) - cy) * camera_z / fy;

      // camera optical frame to the drone's body frame
      const double body_x = camera_z;
      const double body_y = -camera_x;
      const double body_z = -camera_y;

      // body frame to the world frame
      const tf2::Vector3 rotated = tf2::quatRotate(
        rotation, tf2::Vector3(body_x, body_y, body_z));

      geometry_msgs::msg::Vector3 point;
      point.x = drone_position.x + rotated.x();
      point.y = drone_position.y + rotated.y();
      point.z = drone_position.z + rotated.z();
      points.push_back(point);
    }
  }

  return points;
}

void DepthRayProcessor::update_depth_image(
  sensor_msgs::msg::Image::ConstSharedPtr image)
{
  if (!image) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  latest_depth_image_ = *image;
  depth_image_received_ = true;
  last_image_time_ = node_->now().seconds();
}

void DepthRayProcessor::update_camera_info(
  sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info)
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
