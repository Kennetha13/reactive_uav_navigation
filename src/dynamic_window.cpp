#include "reactive_uav_navigation/dynamic_window.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace reactive_uav_navigation {

void DynamicWindowParams::validate() const {
    if (!std::isfinite(dt) || dt <= 0.0 ||
        !std::isfinite(accel.ax) || accel.ax < 0.0 ||
        !std::isfinite(accel.az) || accel.az < 0.0 ||
        !std::isfinite(accel.yaw_accel) || accel.yaw_accel < 0.0) {
        throw std::invalid_argument("Invalid Dynamic Window parameters");
    }
}

DynamicWindow::DynamicWindow(const DynamicWindowParams& params) : params_(params) {
    params_.validate();
}

// Calculate Dynamic Window (Vd)
VelocitySpace DynamicWindow::compute(const CandidateVelocity& current) const {
    if (!std::isfinite(current.vx) ||
        !std::isfinite(current.vz) ||
        !std::isfinite(current.wz)) {
        throw std::invalid_argument("Current velocity must be finite");
    }

    const AccelerationLimits& accel = params_.accel;
    const double dt = params_.dt;

    VelocitySpace vd;

    // Forward velocity
    vd.min.vx = current.vx - accel.ax * dt;
    vd.max.vx = current.vx + accel.ax * dt;

    // Vertical velocity
    vd.min.vz = current.vz - accel.az * dt;
    vd.max.vz = current.vz + accel.az * dt;

    // Yaw rate
    vd.min.wz = current.wz - accel.yaw_accel * dt;
    vd.max.wz = current.wz + accel.yaw_accel * dt;

    return vd;
}

// Calculate Vs ∩ Vd (per-axis highest minimum, lowest maximum)
IntersectionResult DynamicWindow::intersect(const VelocitySpace& vs, const VelocitySpace& vd) {
    VelocitySpace result;

    result.min.vx = std::max(vs.min.vx, vd.min.vx);
    result.max.vx = std::min(vs.max.vx, vd.max.vx);

    result.min.vz = std::max(vs.min.vz, vd.min.vz);
    result.max.vz = std::min(vs.max.vz, vd.max.vz);

    result.min.wz = std::max(vs.min.wz, vd.min.wz);
    result.max.wz = std::min(vs.max.wz, vd.max.wz);

    // Check whether all three dimensions overlap
    bool valid =
        result.min.vx <= result.max.vx &&
        result.min.vz <= result.max.vz &&
        result.min.wz <= result.max.wz;

    return {valid, result};
}

}  // namespace reactive_uav_navigation
