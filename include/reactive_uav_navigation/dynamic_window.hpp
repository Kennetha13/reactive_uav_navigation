#ifndef DYNAMIC_WINDOW_HPP_
#define DYNAMIC_WINDOW_HPP_

#include "reactive_uav_navigation/common.hpp"

namespace reactive_uav_navigation {

struct AccelerationLimits {
    double ax;
    double az;
    double yaw_accel;
};

struct VelocitySpace {
    CandidateVelocity min;
    CandidateVelocity max;
};

struct IntersectionResult {
    bool valid;
    VelocitySpace space;
};

// Calculate Vd
VelocitySpace calculateDynamicWindow(
    const CandidateVelocity& current,
    const AccelerationLimits& accel,
    double dt
);

// Calculate Vr = Vs ∩ Vd ∩ Va
IntersectionResult intersectVelocitySpaces(
    const VelocitySpace& vs,
    const VelocitySpace& vd,
    const VelocitySpace& va
);

} 

#endif