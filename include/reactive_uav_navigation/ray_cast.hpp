#ifndef RAY_CAST_H
#define RAY_CAST_H

#include <optional>
#include <vector>
#include <geometry_msgs/msg/vector3.hpp>

class RayCast {
    public:
        struct RayHit
        {
            double distance;  // Distance along the ray, meters
            geometry_msgs::msg::Vector3 point;
        };

        // replaces the stored obstacle points, given in the world frame
        void setPoints(const std::vector<geometry_msgs::msg::Vector3>& points);

        // casts one ray from origin and returns the closest obstacle point it hits
        std::optional<RayHit> castRay(
            const geometry_msgs::msg::Vector3& origin,
            const geometry_msgs::msg::Vector3& ray_direction,
            double max_range = 20.0) const;

        // distance from a position to the nearest stored obstacle point,
        // a very large number when there are no points
        double nearestPointDistance(const geometry_msgs::msg::Vector3& position) const;
    private:
        std::vector<geometry_msgs::msg::Vector3> points_;

        // How far to the side of a ray an obstacle still counts as in the
        // way. The drone is 0.3 m in radius, so this leaves about 0.4 m of
        // free space beside it when it passes a corner or a pillar, enough
        // to see that it cleared it.
        double RAY_RADIUS = 0.70;
};

#endif  // RAY_CAST_H
