#include "reactive_uav_navigation/vss_maximum.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#ifndef VSS_NO_ROS
#include <rclcpp/rclcpp.hpp>

#include "reactive_uav_navigation/px4_param_reader.hpp"
#endif

namespace reactive_uav_navigation {

void VelocityLimits::validate() const {
    if (!std::isfinite(vx_max) || vx_max <= 0.0) {
        throw std::invalid_argument(
            "vx_max must be greater than 0 (got " + std::to_string(vx_max) + ")");
    }
    if (!std::isfinite(vz_max) || vz_max < 0.0) {
        throw std::invalid_argument(
            "vz_max must be 0 or greater (got " + std::to_string(vz_max) + ")");
    }
    if (!std::isfinite(wz_max) || wz_max < 0.0) {
        throw std::invalid_argument(
            "wz_max must be 0 or greater (got " + std::to_string(wz_max) + ")");
    }
}

void VelocitySteps::validate() const {
    if (!std::isfinite(vx_step) || vx_step <= 0.0) {
        throw std::invalid_argument(
            "vx_step must be greater than 0 (got " + std::to_string(vx_step) + ")");
    }
    if (!std::isfinite(vz_step) || vz_step <= 0.0) {
        throw std::invalid_argument(
            "vz_step must be greater than 0 (got " + std::to_string(vz_step) + ")");
    }
    if (!std::isfinite(wz_step) || wz_step <= 0.0) {
        throw std::invalid_argument(
            "wz_step must be greater than 0 (got " + std::to_string(wz_step) + ")");
    }
}

VelocityLimits effectiveLimits(const VelocityLimits& caps, const Px4Limits& px4) {
    VelocityLimits limits = caps;

    if (px4.vx_max) {
        limits.vx_max = std::min(limits.vx_max, *px4.vx_max);
    }
    // Vs is symmetric in vz, so it has to fit both climbing and descending
    if (px4.vz_max_up) {
        limits.vz_max = std::min(limits.vz_max, *px4.vz_max_up);
    }
    if (px4.vz_max_down) {
        limits.vz_max = std::min(limits.vz_max, *px4.vz_max_down);
    }
    if (px4.wz_max) {
        limits.wz_max = std::min(limits.wz_max, *px4.wz_max);
    }
    return limits;
}

std::vector<double> sampleInterval(double lo, double hi, double step) {
    if (!std::isfinite(step) || step <= 0.0) {
        throw std::invalid_argument("step must be greater than 0");
    }

    std::vector<double> values;
    if (lo > hi) {
        return values;
    }

    const double eps = 1e-9;  // floating-point tolerance

    values.push_back(lo);

    // multiples of step strictly between lo and hi
    const int firstK = static_cast<int>(std::ceil((lo + eps) / step));
    const int lastK = static_cast<int>(std::floor((hi - eps) / step));
    for (int k = firstK; k <= lastK; ++k) {
        values.push_back(k * step);
    }

    if (hi > lo + eps) {
        values.push_back(hi);
    }
    return values;
}

MaximumVelocitiesSpace::MaximumVelocitiesSpace(const VelocityLimits& caps,
                                               const VelocitySteps& steps)
    : caps_(caps), steps_(steps), limits_(caps) {
    caps_.validate();
    steps_.validate();
}

VelocitySpace MaximumVelocitiesSpace::compute() const {
    const VelocityLimits limits = this->limits();

    VelocitySpace vs;
    vs.min = CandidateVelocity{0.0, -limits.vz_max, -limits.wz_max};
    vs.max = CandidateVelocity{limits.vx_max, limits.vz_max, limits.wz_max};
    return vs;
}

std::vector<CandidateVelocity> MaximumVelocitiesSpace::sample(const VelocitySpace& space) const {
    const std::vector<double> vxValues = sampleInterval(space.min.vx, space.max.vx, steps_.vx_step);
    const std::vector<double> vzValues = sampleInterval(space.min.vz, space.max.vz, steps_.vz_step);
    const std::vector<double> wzValues = sampleInterval(space.min.wz, space.max.wz, steps_.wz_step);

    std::vector<CandidateVelocity> candidates;
    candidates.reserve(vxValues.size() * vzValues.size() * wzValues.size());

    for (double vx : vxValues) {
        for (double vz : vzValues) {
            for (double wz : wzValues) {
                candidates.push_back(CandidateVelocity{vx, vz, wz});
            }
        }
    }
    return candidates;
}

void MaximumVelocitiesSpace::setPx4Limits(const Px4Limits& px4) {
    const VelocityLimits limits = effectiveLimits(caps_, px4);
    limits.validate();

    std::lock_guard<std::mutex> lock(mutex_);
    limits_ = limits;
}

VelocityLimits MaximumVelocitiesSpace::limits() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return limits_;
}

#ifndef VSS_NO_ROS

namespace {

VelocityLimits readCaps(rclcpp::Node* node) {
    VelocityLimits caps;
    caps.vx_max = rosDoubleParam(node, "vss.vx_max_cap", caps.vx_max);
    caps.vz_max = rosDoubleParam(node, "vss.vz_max_cap", caps.vz_max);
    caps.wz_max = degToRad(rosDoubleParam(node, "vss.wz_max_cap_deg", radToDeg(caps.wz_max)));
    return caps;
}

VelocitySteps readSteps(rclcpp::Node* node) {
    VelocitySteps steps;
    steps.vx_step = rosDoubleParam(node, "vss.vx_step", steps.vx_step);
    steps.vz_step = rosDoubleParam(node, "vss.vz_step", steps.vz_step);
    steps.wz_step = degToRad(rosDoubleParam(node, "vss.wz_step_deg", radToDeg(steps.wz_step)));
    return steps;
}

}  // namespace

MaximumVelocitiesSpace::MaximumVelocitiesSpace(rclcpp::Node* node)
    : MaximumVelocitiesSpace(readCaps(node), readSteps(node)) {
    auto known = std::make_shared<Px4Limits>();

    px4_reader_ = std::make_shared<Px4ParamReader>(
        node,
        std::vector<std::string>{
            "MPC_XY_VEL_MAX", "MPC_Z_VEL_MAX_UP", "MPC_Z_VEL_MAX_DN", "MPC_YAWRAUTO_MAX"},
        [this, node, known](const std::string& name, double value) {
            if (name == "MPC_XY_VEL_MAX") {
                known->vx_max = value;
            } else if (name == "MPC_Z_VEL_MAX_UP") {
                known->vz_max_up = value;
            } else if (name == "MPC_Z_VEL_MAX_DN") {
                known->vz_max_down = value;
            } else if (name == "MPC_YAWRAUTO_MAX") {
                known->wz_max = degToRad(value);  // PX4 stores deg/s
            }
            setPx4Limits(*known);

            const VelocityLimits now = limits();
            RCLCPP_INFO(node->get_logger(),
                        "%s = %.2f; Vs limits now vx %.2f m/s, vz %.2f m/s, wz %.1f deg/s",
                        name.c_str(), value, now.vx_max, now.vz_max, radToDeg(now.wz_max));
        });
}

#endif  // VSS_NO_ROS

}  // namespace reactive_uav_navigation
