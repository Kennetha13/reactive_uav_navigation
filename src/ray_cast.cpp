#include "reactive_uav_navigation/ray_cast.hpp"
#include <cmath>

void RayCast::setPoints(const std::vector<geometry_msgs::msg::Vector3>& points)
{
    points_ = points;
}

std::optional<RayCast::RayHit> RayCast::castRay(
    const geometry_msgs::msg::Vector3& origin,
    const geometry_msgs::msg::Vector3& ray_direction,
    double max_range) const
{
    // normalize the ray direction
    double magnitude = std::sqrt(
        ray_direction.x * ray_direction.x +
        ray_direction.y * ray_direction.y +
        ray_direction.z * ray_direction.z);

    if (magnitude < 1e-6) {
        return std::nullopt;
    }

    double dx = ray_direction.x / magnitude;
    double dy = ray_direction.y / magnitude;
    double dz = ray_direction.z / magnitude;

    double radius_squared = RAY_RADIUS * RAY_RADIUS;

    double closest_distance = max_range;
    geometry_msgs::msg::Vector3 closest_point;
    bool found_hit = false;

    for (size_t i = 0; i < points_.size(); i++) {
        // point relative to the start of the ray
        double px = points_[i].x - origin.x;
        double py = points_[i].y - origin.y;
        double pz = points_[i].z - origin.z;

        // how far along the ray the point is
        double distance_along_ray = px * dx + py * dy + pz * dz;

        // ignore points behind the start or beyond the search range
        if (distance_along_ray <= 0.0 || distance_along_ray > max_range) {
            continue;
        }

        // perpendicular distance from the point to the ray
        double error_x = px - distance_along_ray * dx;
        double error_y = py - distance_along_ray * dy;
        double error_z = pz - distance_along_ray * dz;
        double perpendicular_distance_squared =
            error_x * error_x + error_y * error_y + error_z * error_z;

        // treat the ray as a cylinder and keep the closest point inside it
        if (perpendicular_distance_squared <= radius_squared &&
            distance_along_ray < closest_distance) {
            closest_distance = distance_along_ray;
            closest_point = points_[i];
            found_hit = true;
        }
    }

    if (!found_hit) {
        return std::nullopt;
    }

    return RayHit{closest_distance, closest_point};
}

double RayCast::nearestPointDistance(const geometry_msgs::msg::Vector3& position) const
{
    double nearest = 1.0e9;

    for (size_t i = 0; i < points_.size(); i++) {
        double dx = points_[i].x - position.x;
        double dy = points_[i].y - position.y;
        double dz = points_[i].z - position.z;
        double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

        if (distance < nearest) {
            nearest = distance;
        }
    }

    return nearest;
}
