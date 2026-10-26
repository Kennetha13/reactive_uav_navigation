#include "reactive_uav_navigation/distance_function.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>

namespace reactive_uav_navigation {

void DistanceParams::validate() const {
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

DistanceFunction::DistanceFunction(const DistanceParams& params)
    : params_(params) {
    params_.validate();
    rayCaster_ = RayCast();
}

double DistanceFunction::computeDistance(
    const geometry_msgs::msg::Vector3& position,
    double headingYaw,
    double headingPitch) const {
    const double distMin = computeDistMin(
        position, headingYaw, headingPitch);
    return normalize(distMin);
}

double DistanceFunction::computeDistance(
    double x, double y, double z,
    double vx, double vy, double vz) const {
    geometry_msgs::msg::Vector3 position;
    position.x = x;
    position.y = y;
    position.z = z;
    const double yaw = std::atan2(vy, vx);
    const double pitch = std::atan2(vz, std::hypot(vx, vy));
    return computeDistance(position, yaw, pitch);
}

double DistanceFunction::computeDistMin(
    const geometry_msgs::msg::Vector3& position,
    double headingYaw,
    double headingPitch) const {
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
                continue;
            }

            const geometry_msgs::msg::Vector3 direction = rayDirectionFromAngles(
                headingYaw + psi_i, headingPitch + theta_j);
            const auto hit = rayCaster_.castRay(direction, L_ij);

            // Only strictly shorter results count as obstacles; the epsilon
            // guards against floating-point noise at the search limit.
            constexpr double kEps = 1e-9;
            if (hit && hit->distance < L_ij - kEps) {
                distMin = std::min(distMin, hit->distance);
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

double DistanceFunction::sampleOffset(
    int index, int count, double maxAngle) {
    if (count == 1) {
        return 0.0;
    }
    const double t = static_cast<double>(index) / (count - 1);
    return -maxAngle + t * (2.0 * maxAngle);
}

double DistanceFunction::shortenFactor(
    double angle, double maxAngle, double lambda) {
    if (maxAngle <= 0.0) {
        return 1.0;
    }
    const double rho = 1.0 - lambda * std::fabs(angle) / maxAngle;
    return std::max(0.0, rho);
}

geometry_msgs::msg::Vector3 DistanceFunction::rayDirectionFromAngles(
    double yaw, double pitch) {
    const double cp = std::cos(pitch);
    geometry_msgs::msg::Vector3 direction;
    direction.x = cp * std::cos(yaw);
    direction.y = cp * std::sin(yaw);
    direction.z = std::sin(pitch);
    return direction;
}

double DistanceFunction::normalize(double distMin) const {
    const double numerator = distMin - params_.R_drone;
    const double denominator = params_.r_search - params_.R_drone;
    return std::max(0.0, numerator / denominator);
}

} // namespace reactive_uav_navigation
