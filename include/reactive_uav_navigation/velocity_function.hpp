#ifndef VELOCITY_FUNCTION_H
#define VELOCITY_FUNCTION_H

#include <geometry_msgs/msg/vector3.hpp>

class VelocityFunction {
    public:
        double computeVelocity(geometry_msgs::msg::Vector3 candidate_velocity) const;
    private:
        // Reference drone_state.hpp and drone_state.cpp for an example.
        // drone_state uses a topic (which is more complex) but you only need to use a one-time service call
        // You will need to have VelocityFunction accept a node in its constructor
        // Then you will need to create a service call and initialize vel_max after object initialization
        double vel_max_x_;
};
#endif