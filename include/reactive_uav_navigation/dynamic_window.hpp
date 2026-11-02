#ifndef DYNAMIC_WINDOW_HPP_
#define DYNAMIC_WINDOW_HPP_

#include "reactive_uav_navigation/common.hpp"

namespace reactive_uav_navigation {

// Maximum accelerations used to build Vd (DWA-3D paper: 1 m/s^2, 1 m/s^2, 100 deg/s^2)
struct AccelerationLimits {
    double ax = 1.0;         // forward (m/s^2)
    double az = 1.0;         // vertical (m/s^2)
    double yaw_accel = 1.7453292519943295;  // yaw rate (rad/s^2), = 100 deg/s^2
};

// A box in velocity space: lowest and highest allowed [vx, vz, wz]
struct VelocitySpace {
    CandidateVelocity min;
    CandidateVelocity max;
};

struct IntersectionResult {
    bool valid;
    VelocitySpace space;
};

// Configuration for DynamicWindow. Fixed at startup, like AdmissibleParams.
struct DynamicWindowParams {
    AccelerationLimits accel;
    double dt = VELOCITY_HORIZON;  // window time (s); shared with Va

    // Throws std::invalid_argument if the configuration is unusable.
    void validate() const;
};

// Builds the dynamic window Vd and intersects it with Vs. Holds only
// configuration, so a single instance can be reused across planning cycles.
class DynamicWindow {
public:
    // Throws std::invalid_argument if `params` fails DynamicWindowParams::validate().
    explicit DynamicWindow(const DynamicWindowParams& params);

    // Vd: velocities reachable from `current` within dt. Throws
    // std::invalid_argument if `current` is not finite.
    VelocitySpace compute(const CandidateVelocity& current) const;

    // Vs ∩ Vd. Va is not a box (it depends on each candidate's predicted
    // pose), so it is applied afterwards by AdmissibleVelocities::filterAdmissible.
    static IntersectionResult intersect(const VelocitySpace& vs, const VelocitySpace& vd);

    const DynamicWindowParams& params() const { return params_; }

private:
    DynamicWindowParams params_;
};

}  // namespace reactive_uav_navigation

#endif
