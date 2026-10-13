// distance_function.hpp
//
// Obstacle-clearance term Dist(v) of the DWA-3D objective function
// (arXiv:2409.05421):
//
//     G(v) = alpha * Heading(v) + beta * Dist(v) + gamma * Vel(v)
//
// This header implements only the Dist(v) term. Heading(v), Vel(v), the
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
// Dependencies
// ------------
// Ray casting against the environment (point cloud, octomap, LiDAR scan,
// etc.) is not implemented here. It is abstracted behind IRayCaster, so
// whichever component owns the obstacle representation provides an
// implementation and this class applies the DWA-3D math on top of it.
//
// Conventions
// -----------
// Positions and directions are expressed in a right-handed, z-up world frame
// (ENU), which matches /mavros/local_position/odom. Yaw is measured about +Z
// with 0 along +X; pitch is elevation above the XY plane, positive upward.
// Angles are in radians and distances in meters.

#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace reactive_uav_navigation {

// Minimal 3D vector, defined here so this header has no external
// dependencies. If the project standardizes on another vector type (e.g.
// Eigen::Vector3d or geometry_msgs::msg::Vector3), replace this definition
// or add a thin adapter; only the x/y/z fields are relied upon.
struct Vector3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

// ---------------------------------------------------------------------
// Obstacle lookup interface used by DistanceCalculator.
//
// Implementations wrap the project's obstacle representation (point cloud,
// occupancy map, LiDAR scan, ...) and answer one question: how far along a
// given ray is the nearest obstacle?
//
// Contract for castRay():
//   - `direction` is a unit vector in the same frame as `origin`.
//   - Returns the distance from `origin` to the first obstacle hit along
//     `direction`, if that hit is within `maxRange`.
//   - If nothing is hit within `maxRange`, returns exactly `maxRange`.
//     Do not return a sentinel such as -1 or infinity: DistanceCalculator
//     treats "result == maxRange" as "no obstacle on this ray".
// ---------------------------------------------------------------------
class IRayCaster {
public:
    virtual ~IRayCaster() = default;

    virtual double castRay(const Vector3& origin,
                            const Vector3& direction,
                            double maxRange) const = 0;
};

// Configuration for DistanceCalculator.
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
    void validate() const {
        // The normalization divides by (r_search - R_drone).
        if (r_search <= R_drone) {
            throw std::invalid_argument(
                "r_search must be strictly greater than R_drone");
        }
        if (numPsiBeams < 1 || numThetaBeams < 1) {
            throw std::invalid_argument(
                "numPsiBeams and numThetaBeams must be >= 1");
        }
        if (psi_max_beam < 0.0 || theta_max_beam < 0.0) {
            throw std::invalid_argument(
                "beam half-angles must be non-negative");
        }
    }
};

// Computes Dist(v) for candidate velocities. Holds only configuration, so a
// single instance can be shared across planning cycles. Methods are const
// and keep no per-call state.
class DistanceCalculator {
public:
    // Throws std::invalid_argument if `params` fails DistanceParams::validate().
    explicit DistanceCalculator(const DistanceParams& params)
        : params_(params) {
        params_.validate();
    }

    // Returns the normalized clearance term Distance(v) in [0, 1] for one
    // candidate velocity.
    //
    //   position:     current drone position
    //   headingYaw:   yaw of the candidate velocity v (rad), e.g.
    //                 atan2(vy, vx)
    //   headingPitch: pitch of the candidate velocity v (rad), e.g.
    //                 atan2(vz, hypot(vx, vy)); use 0 for planar motion
    //   rayCaster:    obstacle lookup (see IRayCaster)
    double computeDistance(const Vector3& position,
                            double headingYaw,
                            double headingPitch,
                            const IRayCaster& rayCaster) const {
        const double distMin = computeDistMin(position, headingYaw,
                                               headingPitch, rayCaster);
        return normalize(distMin);
    }

    // Same as above, but takes the candidate velocity as Cartesian components
    // and derives yaw and pitch internally. Convenient when the components
    // are already at hand, e.g. position from the odometry callback and
    // (vx, vy, vz) from the velocity sampler.
    //
    // Note that v is the candidate being scored, not the drone's measured
    // velocity; it is produced by the DWA velocity sampling, not read from a
    // topic. A zero velocity yields yaw = pitch = 0.
    double computeDistance(double x, double y, double z,
                            double vx, double vy, double vz,
                            const IRayCaster& rayCaster) const {
        const Vector3 position{x, y, z};
        const double yaw = std::atan2(vy, vx);
        const double pitch = std::atan2(vz, std::hypot(vx, vy));
        return computeDistance(position, yaw, pitch, rayCaster);
    }

    // Returns dist_min: the shortest obstacle hit distance (m) over all
    // sampled rays, before subtracting R_drone or normalizing. If no ray hits
    // anything within its search length, returns r_search.
    //
    // Public so callers can log the unnormalized clearance, e.g. for
    // navigation metrics or debugging. Arguments are as for computeDistance().
    double computeDistMin(const Vector3& position,
                           double headingYaw,
                           double headingPitch,
                           const IRayCaster& rayCaster) const {
        double distMin = std::numeric_limits<double>::infinity();

        for (int i = 0; i < params_.numPsiBeams; ++i) {
            const double psi_i = sampleOffset(
                i, params_.numPsiBeams, params_.psi_max_beam);
            const double rho_psi = shortenFactor(
                psi_i, params_.psi_max_beam, params_.lambda_psi);

            for (int j = 0; j < params_.numThetaBeams; ++j) {
                const double theta_j = sampleOffset(
                    j, params_.numThetaBeams, params_.theta_max_beam);
                const double rho_theta = shortenFactor(
                    theta_j, params_.theta_max_beam, params_.lambda_theta);

                // Search length for this ray: L_ij = r_search * rho_psi_i * rho_theta_j
                const double L_ij = params_.r_search * rho_psi * rho_theta;
                if (L_ij <= 0.0) {
                    continue;  // ray has been shortened to nothing
                }

                const Vector3 direction = rayDirectionFromAngles(
                    headingYaw + psi_i, headingPitch + theta_j);

                const double hit = rayCaster.castRay(position, direction, L_ij);

                // A result equal to L_ij means the ray found nothing within
                // its search length (see the IRayCaster contract), so only
                // strictly shorter results count as obstacles. The epsilon
                // guards against floating-point noise in that comparison.
                constexpr double kEps = 1e-9;
                if (hit < L_ij - kEps) {
                    distMin = std::min(distMin, hit);
                }
            }
        }

        // No ray detected an obstacle: report full search range, which
        // normalizes to a Distance(v) of 1.
        if (!std::isfinite(distMin)) {
            distMin = params_.r_search;
        }
        return distMin;
    }

private:
    DistanceParams params_;

    // Returns the angular offset of sample `index` out of `count`, spaced
    // evenly over [-maxAngle, +maxAngle]. A single sample is the center ray
    // (offset 0).
    static double sampleOffset(int index, int count, double maxAngle) {
        if (count == 1) {
            return 0.0;
        }
        const double t = static_cast<double>(index) / (count - 1);  // 0..1
        return -maxAngle + t * (2.0 * maxAngle);
    }

    // Returns the rho factor in [0, 1] for a ray offset by `angle`:
    // 1 for the center ray, decreasing linearly to (1 - lambda) at maxAngle.
    static double shortenFactor(double angle, double maxAngle, double lambda) {
        if (maxAngle <= 0.0) {
            return 1.0;
        }
        const double rho = 1.0 - lambda * std::fabs(angle) / maxAngle;
        return std::max(0.0, rho);
    }

    // Converts yaw and pitch to a unit direction vector using the
    // conventions in the file header. Any change to the world frame
    // convention must be reflected here.
    static Vector3 rayDirectionFromAngles(double yaw, double pitch) {
        const double cp = std::cos(pitch);
        return Vector3{
            cp * std::cos(yaw),
            cp * std::sin(yaw),
            std::sin(pitch)
        };
    }

    // Maps dist_min to Distance(v): subtracts the drone's radius and scales
    // by the usable range, clamping at 0 when an obstacle is within R_drone.
    double normalize(double distMin) const {
        const double numerator = distMin - params_.R_drone;
        const double denominator = params_.r_search - params_.R_drone;
        return std::max(0.0, numerator / denominator);
    }
};

} // namespace reactive_uav_navigation
