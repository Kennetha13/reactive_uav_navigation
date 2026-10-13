#include <cmath>
#include <limits>
#include <optional>

#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

struct RayHit
{
    double distance;  // Distance along the ray, meters
    pcl::PointXYZ point;
};

std::optional<RayHit> castRayThroughPointCloud(
    const sensor_msgs::msg::PointCloud2& cloud,
    const geometry_msgs::msg::Vector3& ray_direction,
    double max_range = 20.0,
    double ray_radius = 0.20)
{
    pcl::PointCloud<pcl::PointXYZ> points;
    pcl::fromROSMsg(cloud, points);

    // Normalize the ray direction.
    const double magnitude = std::sqrt(
        ray_direction.x * ray_direction.x +
        ray_direction.y * ray_direction.y +
        ray_direction.z * ray_direction.z);

    if (magnitude < 1e-6) {
        return std::nullopt;
    }

    const double dx = ray_direction.x / magnitude;
    const double dy = ray_direction.y / magnitude;
    const double dz = ray_direction.z / magnitude;

    const double radius_squared = ray_radius * ray_radius;

    double closest_distance = std::numeric_limits<double>::max();
    pcl::PointXYZ closest_point;
    bool found_hit = false;

    for (const auto& point : points.points) {
        if (!std::isfinite(point.x) ||
            !std::isfinite(point.y) ||
            !std::isfinite(point.z)) {
            continue;
        }

        // Projection of the point onto the ray.
        const double distance_along_ray =
            point.x * dx +
            point.y * dy +
            point.z * dz;

        // Ignore points behind the LiDAR or beyond the search range.
        if (distance_along_ray <= 0.0 ||
            distance_along_ray > max_range) {
            continue;
        }

        // Point on the ray closest to the LiDAR point.
        const double ray_x = distance_along_ray * dx;
        const double ray_y = distance_along_ray * dy;
        const double ray_z = distance_along_ray * dz;

        // Perpendicular distance from the LiDAR point to the ray.
        const double error_x = point.x - ray_x;
        const double error_y = point.y - ray_y;
        const double error_z = point.z - ray_z;

        const double perpendicular_distance_squared =
            error_x * error_x +
            error_y * error_y +
            error_z * error_z;

        // Treat the ray as a thin cylinder to account for LiDAR sparsity.
        if (perpendicular_distance_squared <= radius_squared &&
            distance_along_ray < closest_distance) {
            closest_distance = distance_along_ray;
            closest_point = point;
            found_hit = true;
        }
    }

    if (!found_hit) {
        return std::nullopt;
    }

    return RayHit{
        closest_distance,
        closest_point
    };
}

/*

EXAMPLE USAGE

geometry_msgs::msg::Vector3 forward_ray;
forward_ray.x = 1.0;
forward_ray.y = 0.0;
forward_ray.z = 0.0;

auto hit = castRayThroughPointCloud(
    latest_cloud,
    forward_ray,
    15.0,   // Maximum range: 15 meters
    0.25    // Ray radius: 25 centimeters
);

if (hit.has_value()) {
    RCLCPP_INFO(
        this->get_logger(),
        "Obstacle detected %.2f meters along ray",
        hit->distance);
} else {
    RCLCPP_INFO(
        this->get_logger(),
        "No obstacle detected along ray");
}
        


// Forward
{x = 1.0, y = 0.0, z = 0.0}

// Forward-left
{x = 1.0, y = 0.5, z = 0.0}

// Forward-right
{x = 1.0, y = -0.5, z = 0.0}

// Forward-up
{x = 1.0, y = 0.0, z = 0.5}

// Forward-down
{x = 1.0, y = 0.0, z = -0.5}

*/

/*
DEPENDENCIES INSTALL

sudo apt install \
  libpcl-dev \
  ros-humble-pcl-conversions \
  ros-humble-pcl-ros
  
*/