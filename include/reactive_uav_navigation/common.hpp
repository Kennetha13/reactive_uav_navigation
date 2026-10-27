#ifndef COMMON_H_
#define COMMON_H_

const double CONTROL_PERIOD = 0.1; // seconds
const double CONTROL_FREQUENCY = 1.0 / CONTROL_PERIOD; // Hz

// One candidate from the dynamic window, as the drone-centric control vector
// [v_x, v_z, w_z].
struct CandidateVelocity {
    double vx = 0.0;  // forward speed (m/s)
    double vz = 0.0;  // vertical speed (m/s)
    double wz = 0.0;  // yaw rate (rad/s)
};

#endif