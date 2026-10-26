// distance_function.hpp
//
// Obstacle-clearance term Dist(v) of the DWA-3D objective function
// (arXiv:2409.05421):
//
//     G(v) = alpha * Heading(v) + beta * Dist(v) + gamma * Vel(v)
//
// This component implements only the Dist(v) term. Heading(v), Vel(v), the
// weighting and the search over candidate velocities live elsewhere.
//
// Method
// ------
// For one candidate velocity v, a fan of rays is cast from the drone's
// position, centered on the direction of v. Each ray (i, j) is offset from
// that direction by a horizontal angle psi_i and a vertical angle theta_j,
// and its search length is shortened the further it points away from v,
// since obstacles directly along v matter most:
//
//     rho_psi_i   = 1 - lambda_psi   * |psi_i|   / psi_max_beam
//     rho_theta_j = 1 - lambda_theta * |theta_j| / theta_max_beam
//     L_ij        = r_search * rho_psi_i * rho_theta_j
//
// dist_min is the shortest obstacle hit distance over all rays. It is then
// normalized to [0, 1], where 1 means clear out to the full search range
// and 0 means an obstacle is within the drone's own radius:
//
//     Distance(v) = max(0, (dist_min - R_drone) / (r_search - R_drone))
//
// Ray casting against the environment is provided by RayCast.
//
// Conventions
// -----------
// Positions and directions are expressed in a right-handed, z-up world frame
// (ENU), which matches /mavros/local_position/odom. Yaw is measured about +Z
// with 0 along +X; pitch is elevation above the XY plane, positive upward.
// Angles are in radians and distances in meters.

#pragma once

#include <cmath>
#include <geometry_msgs/msg/vector3.hpp>

#include "reactive_uav_navigation/ray_cast.hpp"

namespace reactive_uav_navigation {

// Configuration for DistanceFunction.
//
// These are physical constants of the vehicle and sensor plus tuning
// choices. They are fixed at startup and are not read from telemetry, so
// one instance is typically built once and reused for every planning cycle.
struct DistanceParams {
    double R_drone = 0.3;          // physical radius of the drone (m)
    double r_search = 10.0;        // max ray length; should not exceed the sensor's range (m)
    double psi_max_beam = M_PI_4;  // max horizontal angle of a ray from the center ray (rad)
    double theta_max_beam = M_PI_4;// max vertical angle of a ray from the center ray (rad)
    double lambda_psi = 1.0;       // how strongly rays are shortened with horizontal offset, in [0, 1]
    double lambda_theta = 1.0;     // how strongly rays are shortened with vertical offset, in [0, 1]
    int numPsiBeams = 9;           // number of horizontal ray samples (more = finer, slower)
    int numThetaBeams = 5;         // number of vertical ray samples (more = finer, slower)

    // Throws std::invalid_argument if the configuration is unusable.
    void validate() const;
};

// Computes Dist(v) for candidate velocities. Holds only configuration, so a
// single instance can be shared across planning cycles. Methods are const
// and keep no per-call state.
class DistanceFunction {
public:
    // Throws std::invalid_argument if `params` fails DistanceParams::validate().
    explicit DistanceFunction(const DistanceParams& params);

    // Returns the normalized clearance term Distance(v) in [0, 1] for one
    // candidate velocity.
    //
    //   position:     current drone position
    //   headingYaw:   yaw of the candidate velocity v (rad), e.g.
    //                 atan2(vy, vx)
    //   headingPitch: pitch of the candidate velocity v (rad), e.g.
    //                 atan2(vz, hypot(vx, vy)); use 0 for planar motion
    double computeDistance(const geometry_msgs::msg::Vector3& position,
                           double headingYaw,
                           double headingPitch) const;

    // Same as above, but takes the candidate velocity as Cartesian components
    // and derives yaw and pitch internally. Convenient when the components
    // are already at hand, e.g. position from the odometry callback and
    // (vx, vy, vz) from the velocity sampler.
    //
    // Note that v is the candidate being scored, not the drone's measured
    // velocity; it is produced by the DWA velocity sampling, not read from a
    // topic. A zero velocity yields yaw = pitch = 0.
    double computeDistance(double x, double y, double z,
                           double vx, double vy, double vz) const;

    // Returns dist_min: the shortest obstacle hit distance (m) over all
    // sampled rays, before subtracting R_drone or normalizing. If no ray hits
    // anything within its search length, returns r_search.
    //
    // Public so callers can log the unnormalized clearance, e.g. for
    // navigation metrics or debugging. Arguments are as for computeDistance().
    double computeDistMin(const geometry_msgs::msg::Vector3& position,
                          double headingYaw,
                          double headingPitch) const;

private:
    DistanceParams params_;
    RayCast rayCaster_;

    // Returns the angular offset of sample `index` out of `count`, spaced
    // evenly over [-maxAngle, +maxAngle]. A single sample is the center ray
    // (offset 0).
    static double sampleOffset(int index, int count, double maxAngle);

    // Returns the rho factor in [0, 1] for a ray offset by `angle`:
    // 1 for the center ray, decreasing linearly to (1 - lambda) at maxAngle.
    static double shortenFactor(double angle, double maxAngle, double lambda);

    // Converts yaw and pitch to a unit direction vector using the
    // conventions in the file header. Any change to the world frame
    // convention must be reflected here.
    static geometry_msgs::msg::Vector3 rayDirectionFromAngles(
        double yaw, double pitch);

    // Maps dist_min to Distance(v): subtracts the drone's radius and scales
    // by the usable range, clamping at 0 when an obstacle is within R_drone.
    double normalize(double distMin) const;
};

} // namespace reactive_uav_navigation
