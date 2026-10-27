#include "reactive_uav_navigation/dynamic_window.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace reactive_uav_navigation {

// Calculate Dynamic Window (Vd)
VelocitySpace calculateDynamicWindow(
    const CandidateVelocity& current,
    const AccelerationLimits& accel,
    double dt
) {
    if (!std::isfinite(dt) || dt <= 0.0 ||
        !std::isfinite(current.vx) ||
        !std::isfinite(current.vz) ||
        !std::isfinite(current.wz) ||
        !std::isfinite(accel.ax) || accel.ax < 0.0 ||
        !std::isfinite(accel.az) || accel.az < 0.0 ||
        !std::isfinite(accel.yaw_accel) ||
        accel.yaw_accel < 0.0) {
        throw std::invalid_argument("Invalid Dynamic Window inputs");
    }

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


// Calculate Vr = Vs ∩ Vd ∩ Va
// Assumes Vs, Vd, and Va are available as VelocitySpace objects
IntersectionResult intersectVelocitySpaces(
    const VelocitySpace& vs,
    const VelocitySpace& vd,
    const VelocitySpace& va
) {
    VelocitySpace result;

    // Forward velocity intersection
    result.min.vx = std::max({
        vs.min.vx,
        vd.min.vx,
        va.min.vx
    });

    result.max.vx = std::min({
        vs.max.vx,
        vd.max.vx,
        va.max.vx
    });

    // Vertical velocity intersection
    result.min.vz = std::max({
        vs.min.vz,
        vd.min.vz,
        va.min.vz
    });

    result.max.vz = std::min({
        vs.max.vz,
        vd.max.vz,
        va.max.vz
    });

    // Yaw rate intersection
    result.min.wz = std::max({
        vs.min.wz,
        vd.min.wz,
        va.min.wz
    });

    result.max.wz = std::min({
        vs.max.wz,
        vd.max.wz,
        va.max.wz
    });

    // Check whether all three dimensions overlap
    bool valid =
        result.min.vx <= result.max.vx &&
        result.min.vz <= result.max.vz &&
        result.min.wz <= result.max.wz;

    return {valid, result};
}

} 