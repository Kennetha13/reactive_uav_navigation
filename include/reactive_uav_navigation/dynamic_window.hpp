#ifndef DYNAMIC_WINDOW_HPP_
#define DYNAMIC_WINDOW_HPP_

namespace reactive_uav_navigation {

struct Velocity {
    double vx;
    double vz;
    double yaw_rate;
};

struct AccelerationLimits {
    double ax;
    double az;
    double yaw_accel;
};

struct VelocitySpace {
    Velocity min;
    Velocity max;
};

struct IntersectionResult {
    bool valid;
    VelocitySpace space;
};

// Calculate Vd
VelocitySpace calculateDynamicWindow(
    const Velocity& current,
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