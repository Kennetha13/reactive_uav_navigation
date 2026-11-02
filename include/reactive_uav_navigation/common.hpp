#ifndef COMMON_H_
#define COMMON_H_

const double CONTROL_PERIOD = 0.1; // seconds
const double CONTROL_FREQUENCY = 1.0 / CONTROL_PERIOD; // Hz

// Prediction horizon shared by the dynamic window (Vd) and the admissible
// velocities (Va): 10 control periods, as in the DWA-3D paper.
const double VELOCITY_HORIZON = 10.0 * CONTROL_PERIOD; // seconds

// One candidate from the dynamic window, as the drone-centric control vector
// [v_x, v_z, w_z].
struct CandidateVelocity {
    double vx = 0.0;  // forward speed (m/s)
    double vz = 0.0;  // vertical speed (m/s)
    double wz = 0.0;  // yaw rate (rad/s)
};

#endif