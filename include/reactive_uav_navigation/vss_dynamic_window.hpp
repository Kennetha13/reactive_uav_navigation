// vss_dynamic_window.hpp
//
// Dynamic window Vd of DWA-3D: the velocities reachable from the current one
// within dt, given the maximum accelerations
//
//     vx in [vx_c - ax * dt, vx_c + ax * dt]   (same for vz and wz)
//
// The accelerations are the smaller of
//   - a cap (ROS parameters, defaulting to the values in the paper), and
//   - what the flight controller allows (PX4 parameters read through MAVROS).
// Until PX4 has answered, the caps alone are used.
//
// ROS parameters read by DynamicWindow(rclcpp::Node*, dt) (name: default):
//   vss.ax_cap: 1.0 (m/s^2)   vss.az_cap: 1.0 (m/s^2)
//   vss.yaw_accel_cap_deg: 100 (deg/s^2)
//
// PX4 parameters read through MAVROS:
//   MPC_ACC_HOR                             -> ax
//   MPC_ACC_UP_MAX, MPC_ACC_DOWN_MAX        -> az (the smaller of the two)
//   MPC_YAWRAUTO_ACC (deg/s^2)              -> yaw_accel
//
// The ROS part is excluded when VSS_NO_ROS is defined (used by the unit tests).

#ifndef VSS_DYNAMIC_WINDOW_HPP_
#define VSS_DYNAMIC_WINDOW_HPP_

#include <memory>
#include <mutex>
#include <optional>

#include "reactive_uav_navigation/common.hpp"

namespace rclcpp {
class Node;
}

namespace reactive_uav_navigation {

class Px4ParamReader;

// Maximum accelerations used to build Vd (DWA-3D paper: 1 m/s^2, 1 m/s^2, 100 deg/s^2)
struct AccelerationLimits {
    double ax = 1.0;                       // forward (m/s^2)
    double az = 1.0;                       // vertical (m/s^2)
    double yaw_accel = degToRad(100.0);    // yaw rate (rad/s^2)
};

// Accelerations reported by the flight controller, in SI units. A value that
// has not been received yet is empty.
struct Px4AccelerationLimits {
    std::optional<double> ax;          // m/s^2
    std::optional<double> az_up;       // m/s^2
    std::optional<double> az_down;     // m/s^2
    std::optional<double> yaw_accel;   // rad/s^2
};

// The accelerations to use: each one is the cap, or the flight controller's
// value if that is smaller.
AccelerationLimits effectiveAcceleration(const AccelerationLimits& caps,
                                         const Px4AccelerationLimits& px4);

// A box in velocity space: lowest and highest allowed [vx, vz, wz]
struct VelocitySpace {
    CandidateVelocity min;
    CandidateVelocity max;
};

// Configuration for DynamicWindow. Fixed at startup, like AdmissibleParams.
struct DynamicWindowParams {
    AccelerationLimits accel;      // the caps
    double dt = VELOCITY_HORIZON;  // window time (s); shared with Va

    // Throws std::invalid_argument if the configuration is unusable.
    void validate() const;
};

// Builds the dynamic window Vd. A single instance can be reused across
// planning cycles. Thread-safe: the accelerations can be updated by the
// service callback while the control loop reads. Intersecting Vd with Vs is
// done in vss_collector.hpp.
class DynamicWindow {
public:
    // Fixed caps, no ROS. The caps are the accelerations until
    // setPx4Limits() is called. Throws std::invalid_argument if invalid.
    explicit DynamicWindow(const DynamicWindowParams& params);

    // Reads the caps from ROS parameters and the flight controller's
    // accelerations through MAVROS. The node must outlive this object and be
    // spun. Not available when built with VSS_NO_ROS.
    explicit DynamicWindow(rclcpp::Node* node, double dt = VELOCITY_HORIZON);

    // Vd: velocities reachable from `current` within dt. Throws
    // std::invalid_argument if `current` is not finite.
    VelocitySpace compute(const CandidateVelocity& current) const;

    // Replaces what is known about the flight controller's accelerations.
    void setPx4Limits(const Px4AccelerationLimits& px4);

    AccelerationLimits accel() const;  // the accelerations in force
    const DynamicWindowParams& params() const { return params_; }

private:
    const DynamicWindowParams params_;

    mutable std::mutex mutex_;
    AccelerationLimits accel_;  // effective accelerations, guarded by mutex_

    std::shared_ptr<Px4ParamReader> px4_reader_;
};

}  // namespace reactive_uav_navigation

#endif  // VSS_DYNAMIC_WINDOW_HPP_
