#ifndef HEADING_FUNCTION_H
#define HEADING_FUNCTION_H

#include "vector"
#include "reactive_uav_navigation/common.hpp"
#include "reactive_uav_navigation/drone_state.hpp"

class HeadingFunction {
    public:
        explicit HeadingFunction(const DroneState& drone_state);
        double K_yaw = 0.8; // Prioritize staying on course toward goal, favor vertical avoidance
        double K_z = 0.2; // Prioritize maintaining altitude with the goal, favor lateral avoidance

        // Computes the maximum z error among all candidate velocities, which is used to normalize the z heading score.
        // You must store the result of #getMaxZError then subsequently pass it to #computeHeading
        double getMaxZError(vector<CandidateVelocity> candidate_velocities);
        double computeHeading(CandidateVelocity velocity, double max_z_error);
    private:
        const DroneState& drone_state_;
        double computeYawHeading(CandidateVelocity velocity);
        double computeZHeading(CandidateVelocity velocity, double max_z_error);

};

#endif