#include "reactive_uav_navigation/ray_cast.hpp"
#include <cmath>
#include <limits>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

std::optional<RayCast::RayHit> RayCast::castRay(
    const geometry_msgs::msg::Vector3& ray_direction,
    double max_range) const
{
    // Collect the latest point cloud from the LiDAR sensor.
    sensor_msgs::msg::PointCloud2 cloud = collectPointCloud();

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

    const double radius_squared = RAY_RADIUS * RAY_RADIUS;

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

sensor_msgs::msg::PointCloud2 RayCast::collectPointCloud() const {
    // Placeholder implementation.
    // I imagine this function would have to subscribe to a ROS2 topic to gather this data
    // If that is the case, please reference drone_state.hpp and drone_state.cpp for an internal example of this
    sensor_msgs::msg::PointCloud2 cloud;
    return cloud;
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