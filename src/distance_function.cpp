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
    if (rayGridStepYaw <= 0.0 || rayGridStepPitch <= 0.0) {
        throw std::invalid_argument(
            "ray grid steps must be > 0");
    }
}

DistanceFunction::DistanceFunction(const DistanceParams& params)
    : params_(params) {
    params_.validate();
    rayCaster_ = RayCast();
}

void DistanceFunction::castRays() {
    // Round to whole cells so the grid wraps seamlessly in yaw and includes
    // both poles in pitch.
    numYawCells_ = std::max(
        1, static_cast<int>(std::lround(2.0 * M_PI / params_.rayGridStepYaw)));
    numPitchCells_ = std::max(
        2, static_cast<int>(std::lround(M_PI / params_.rayGridStepPitch)) + 1);
    yawStep_ = 2.0 * M_PI / numYawCells_;
    pitchStep_ = M_PI / (numPitchCells_ - 1);

    rayHitDistances_.assign(
        static_cast<std::size_t>(numYawCells_) * numPitchCells_,
        std::numeric_limits<double>::infinity());

    for (int m = 0; m < numPitchCells_; ++m) {
        const double pitch = -M_PI_2 + m * pitchStep_;
        const std::size_t row = static_cast<std::size_t>(m) * numYawCells_;
        // At a pole every yaw is the same direction: cast once, copy the rest.
        const bool pole = (m == 0 || m == numPitchCells_ - 1);

        for (int k = 0; k < numYawCells_; ++k) {
            if (pole && k > 0) {
                rayHitDistances_[row + k] = rayHitDistances_[row];
                continue;
            }
            const geometry_msgs::msg::Vector3 direction =
                rayDirectionFromAngles(-M_PI + k * yawStep_, pitch);
            // Cast to the full search range; shorter fan rays reuse this.
            const auto hit = rayCaster_.castRay(direction, params_.r_search);
            if (hit) {
                rayHitDistances_[row + k] = hit->distance;
            }
        }
    }
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
    if (rayHitDistances_.empty()) {
        throw std::logic_error(
            "castRays() must be called before computeDistMin()");
    }
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
            const double hitDistance = cachedHitDistance(direction);

            // Only strictly shorter results count as obstacles; the epsilon
            // guards against floating-point noise at the search limit.
            constexpr double kEps = 1e-9;
            if (hitDistance < L_ij - kEps) {
                distMin = std::min(distMin, hitDistance);
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

double DistanceFunction::cachedHitDistance(
    const geometry_msgs::msg::Vector3& direction) const {
    // Convert back to yaw and pitch so directions past a pole or outside
    // [-pi, pi) land on the right cell.
    const double yaw = std::atan2(direction.y, direction.x);
    const double pitch = std::asin(std::clamp(direction.z, -1.0, 1.0));
    const int k = static_cast<int>(std::lround((yaw + M_PI) / yawStep_)) %
                  numYawCells_;
    const int m = std::clamp(
        static_cast<int>(std::lround((pitch + M_PI_2) / pitchStep_)),
        0, numPitchCells_ - 1);
    return rayHitDistances_[static_cast<std::size_t>(m) * numYawCells_ + k];
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