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
// Ray reuse
// ---------
// Candidates are scored many times per control period, but the environment
// does not change within a period, so rays are not cast per candidate.
// castRays() casts a fixed grid of directions (yaw x pitch, full sphere) once,
// at the start of the period. Each ray of a candidate's fan is then answered
// by looking up the nearest grid direction instead of calling RayCast again.
// The grid rays are cast out to r_search, so a result is valid for any
// shortened length L_ij <= r_search.
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
#include <vector>
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
    double rayGridStepYaw = M_PI / 18.0;    // yaw spacing of the grid cast by castRays() (rad); no coarser than the fan spacing
    double rayGridStepPitch = M_PI / 18.0;  // pitch spacing of the grid cast by castRays() (rad); smaller = more accurate, more casts

    // Throws std::invalid_argument if the configuration is unusable.
    void validate() const;
};

// Computes Dist(v) for candidate velocities. Holds the configuration and the
// ray results of the latest castRays(). Call castRays() once at the start of
// every control period, then score any number of candidates against those
// results. castRays() must not run concurrently with computeDistance() or
// computeDistMin().
class DistanceFunction {
public:
    // Throws std::invalid_argument if `params` fails DistanceParams::validate().
    explicit DistanceFunction(const DistanceParams& params);

    // Casts the grid of rays and stores the results, replacing those of the
    // previous call. Call once per control period, before scoring candidates.
    // The grid spans every yaw and pitch, spaced by rayGridStepYaw and
    // rayGridStepPitch (rounded so the cells divide the range evenly).
    void castRays();

    // Returns the normalized clearance term Distance(v) in [0, 1] for one
    // candidate velocity. Uses the results of the latest castRays(); throws
    // std::logic_error if it has not been called yet.
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
    // Uses the results of the latest castRays(); throws std::logic_error if it
    // has not been called yet.
    double computeDistMin(const geometry_msgs::msg::Vector3& position,
                          double headingYaw,
                          double headingPitch) const;

private:
    DistanceParams params_;
    RayCast rayCaster_;

    // Results of the latest castRays(): distance of the first hit for each
    // grid direction, or infinity if the ray hit nothing within r_search.
    // Stored row by row, one row per pitch, one column per yaw.
    std::vector<double> rayHitDistances_;
    int numYawCells_ = 0;
    int numPitchCells_ = 0;
    double yawStep_ = 0.0;    // actual yaw spacing of the grid (rad)
    double pitchStep_ = 0.0;  // actual pitch spacing of the grid (rad)

    // Returns the stored hit distance of the grid direction nearest to
    // `direction` (a unit vector), or infinity if that ray hit nothing.
    double cachedHitDistance(const geometry_msgs::msg::Vector3& direction) const;

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