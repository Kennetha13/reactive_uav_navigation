#include "reactive_uav_navigation/vss_dynamic_window.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#ifndef VSS_NO_ROS
#include <rclcpp/rclcpp.hpp>

#include "reactive_uav_navigation/px4_param_reader.hpp"
#endif

namespace reactive_uav_navigation {

void DynamicWindowParams::validate() const {
    if (!std::isfinite(dt) || dt <= 0.0 ||
        !std::isfinite(accel.ax) || accel.ax < 0.0 ||
        !std::isfinite(accel.az) || accel.az < 0.0 ||
        !std::isfinite(accel.yaw_accel) || accel.yaw_accel < 0.0) {
        throw std::invalid_argument("Invalid Dynamic Window parameters");
    }
}

AccelerationLimits effectiveAcceleration(const AccelerationLimits& caps,
                                         const Px4AccelerationLimits& px4) {
    AccelerationLimits accel = caps;

    if (px4.ax) {
        accel.ax = std::min(accel.ax, *px4.ax);
    }
    // the window is symmetric in vz, so it has to fit both climbing and descending
    if (px4.az_up) {
        accel.az = std::min(accel.az, *px4.az_up);
    }
    if (px4.az_down) {
        accel.az = std::min(accel.az, *px4.az_down);
    }
    if (px4.yaw_accel) {
        accel.yaw_accel = std::min(accel.yaw_accel, *px4.yaw_accel);
    }
    return accel;
}

DynamicWindow::DynamicWindow(const DynamicWindowParams& params)
    : params_(params), accel_(params.accel) {
    params_.validate();
}

// Calculate Dynamic Window (Vd)
VelocitySpace DynamicWindow::compute(const CandidateVelocity& current) const {
    if (!std::isfinite(current.vx) ||
        !std::isfinite(current.vz) ||
        !std::isfinite(current.wz)) {
        throw std::invalid_argument("Current velocity must be finite");
    }

    const AccelerationLimits accel = this->accel();
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

void DynamicWindow::setPx4Limits(const Px4AccelerationLimits& px4) {
    DynamicWindowParams effective = params_;
    effective.accel = effectiveAcceleration(params_.accel, px4);
    effective.validate();

    std::lock_guard<std::mutex> lock(mutex_);
    accel_ = effective.accel;
}

AccelerationLimits DynamicWindow::accel() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return accel_;
}

#ifndef VSS_NO_ROS

namespace {

DynamicWindowParams readParams(rclcpp::Node* node, double dt) {
    DynamicWindowParams params;
    params.dt = dt;
    params.accel.ax = rosDoubleParam(node, "vss.ax_cap", params.accel.ax);
    params.accel.az = rosDoubleParam(node, "vss.az_cap", params.accel.az);
    params.accel.yaw_accel = degToRad(
        rosDoubleParam(node, "vss.yaw_accel_cap_deg", radToDeg(params.accel.yaw_accel)));
    return params;
}

}  // namespace

DynamicWindow::DynamicWindow(rclcpp::Node* node, double dt)
    : DynamicWindow(readParams(node, dt)) {
    auto known = std::make_shared<Px4AccelerationLimits>();

    px4_reader_ = std::make_shared<Px4ParamReader>(
        node,
        std::vector<std::string>{
            "MPC_ACC_HOR", "MPC_ACC_UP_MAX", "MPC_ACC_DOWN_MAX", "MPC_YAWRAUTO_ACC"},
        [this, node, known](const std::string& name, double value) {
            if (name == "MPC_ACC_HOR") {
                known->ax = value;
            } else if (name == "MPC_ACC_UP_MAX") {
                known->az_up = value;
            } else if (name == "MPC_ACC_DOWN_MAX") {
                known->az_down = value;
            } else if (name == "MPC_YAWRAUTO_ACC") {
                known->yaw_accel = degToRad(value);  // PX4 stores deg/s^2
            }
            setPx4Limits(*known);

            const AccelerationLimits now = accel();
            RCLCPP_INFO(node->get_logger(),
                        "%s = %.2f; Vd accelerations now ax %.2f m/s^2, az %.2f m/s^2, "
                        "yaw %.1f deg/s^2",
                        name.c_str(), value, now.ax, now.az, radToDeg(now.yaw_accel));
        });
}

#endif  // VSS_NO_ROS

}  // namespace reactive_uav_navigation
