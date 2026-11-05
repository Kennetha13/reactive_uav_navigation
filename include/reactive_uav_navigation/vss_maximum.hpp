// vss_maximum.hpp
//
// Maximum velocity space Vs of DWA-3D and the sampling of velocity boxes into
// candidates.
//
//     vx in [0, vx_max]   vz in [-vz_max, vz_max]   wz in [-wz_max, wz_max]
//
// The limits are the smaller of
//   - a cap (ROS parameters, defaulting to the values in the paper), and
//   - what the flight controller allows (PX4 parameters read through MAVROS).
// Until PX4 has answered, the caps alone are used. The cap keeps the candidate
// grid small: PX4's own defaults (e.g. 12 m/s forward) would give about a
// million candidates per cycle.
//
// ROS parameters read by MaximumVelocitiesSpace(rclcpp::Node*) (name: default):
//   vss.vx_max_cap: 0.3 (m/s)       vss.vx_step: 0.05 (m/s)
//   vss.vz_max_cap: 0.3 (m/s)       vss.vz_step: 0.05 (m/s)
//   vss.wz_max_cap_deg: 45 (deg/s)  vss.wz_step_deg: 2.5 (deg/s)
//
// PX4 parameters read through MAVROS:
//   MPC_XY_VEL_MAX                      -> vx_max
//   MPC_Z_VEL_MAX_UP, MPC_Z_VEL_MAX_DN  -> vz_max (the smaller of the two)
//   MPC_YAWRAUTO_MAX (deg/s)            -> wz_max
//
// The implementation is in vss_maximum.cpp. Its ROS part is excluded when
// VSS_NO_ROS is defined (used by the unit tests).

#ifndef VSS_MAXIMUM_HPP_
#define VSS_MAXIMUM_HPP_

#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <vector>

#include "reactive_uav_navigation/common.hpp"
#include "reactive_uav_navigation/vss_dynamic_window.hpp"

namespace rclcpp {
class Node;
}

namespace reactive_uav_navigation {

class Px4ParamReader;

// maximum velocities of the drone
struct VelocityLimits {
    double vx_max = 0.3;             // m/s
    double vz_max = 0.3;             // m/s
    double wz_max = degToRad(45.0);  // rad/s

    // throws std::invalid_argument if any limit is invalid
    void validate() const;
};

// spacing between sampled velocities
struct VelocitySteps {
    double vx_step = 0.05;           // m/s
    double vz_step = 0.05;           // m/s
    double wz_step = degToRad(2.5);  // rad/s

    // throws std::invalid_argument if any step is invalid
    void validate() const;
};

// Limits reported by the flight controller, in SI units. A value that has not
// been received yet is empty.
struct Px4Limits {
    std::optional<double> vx_max;       // m/s
    std::optional<double> vz_max_up;    // m/s
    std::optional<double> vz_max_down;  // m/s
    std::optional<double> wz_max;       // rad/s
};

// The limits to use: each one is the cap, or the flight controller's value if
// that is smaller.
VelocityLimits effectiveLimits(const VelocityLimits& caps, const Px4Limits& px4);

// returns values from lo to hi spaced by step, including both ends and 0 (if inside)
std::vector<double> sampleInterval(double lo, double hi, double step);

// Builds Vs and samples velocity boxes on the step grid. Thread-safe: the
// limits can be updated by the service callback while the control loop reads.
class MaximumVelocitiesSpace {
public:
    // Fixed caps and steps, no ROS. The caps are the limits until
    // setPx4Limits() is called. Throws std::invalid_argument if invalid.
    MaximumVelocitiesSpace(const VelocityLimits& caps, const VelocitySteps& steps);

    // Reads caps and steps from ROS parameters and the flight controller's
    // limits through MAVROS. The node must outlive this object and be spun.
    // Not available when built with VSS_NO_ROS.
    explicit MaximumVelocitiesSpace(rclcpp::Node* node);

    // Vs: vx in [0, vx_max], vz in [-vz_max, vz_max], wz in [-wz_max, wz_max]
    VelocitySpace compute() const;

    // every combination of sampled vx, vz and wz values inside the box
    std::vector<CandidateVelocity> sample(const VelocitySpace& space) const;

    // Replaces what is known about the flight controller's limits.
    void setPx4Limits(const Px4Limits& px4);

    VelocityLimits limits() const;
    const VelocitySteps& steps() const { return steps_; }

private:
    const VelocityLimits caps_;
    const VelocitySteps steps_;

    mutable std::mutex mutex_;
    VelocityLimits limits_;  // effective limits, guarded by mutex_

    std::shared_ptr<Px4ParamReader> px4_reader_;
};

}  // namespace reactive_uav_navigation

#endif  // VSS_MAXIMUM_HPP_
