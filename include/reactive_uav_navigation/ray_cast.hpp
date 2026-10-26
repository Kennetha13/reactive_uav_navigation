#ifndef RAY_CAST_H
#define RAY_CAST_H

#include <optional>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>

class RayCast {
    public:
        struct RayHit
        {
            double distance;  // Distance along the ray, meters
            pcl::PointXYZ point;
        };
        std::optional<RayHit> castRay(
            const geometry_msgs::msg::Vector3& ray_direction,
            double max_range = 20.0) const;
    private:
        // Collects the latest point cloud.
        // I saw this being passed as an argument in castRay. 
        // castRay is a public method meant to be called within distance_function.cpp.
        // the distance_function shouldn't concern itself with how the point cloud is collected, instead the point cloud should be collected internally.
        sensor_msgs::msg::PointCloud2 collectPointCloud() const;
        double RAY_RADIUS = 0.20;
};

#endif  // RAY_CAST_H
