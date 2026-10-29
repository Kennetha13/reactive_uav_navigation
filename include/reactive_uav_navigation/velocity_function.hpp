#ifndef VELOCITY_FUNCTION_H
#define VELOCITY_FUNCTION_H

#include <atomic>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/srv/param_get.hpp>

class VelocityFunction {
    public:
        explicit VelocityFunction(rclcpp::Node *node);

        // Vel(v), Fig. 21
        // HeadingFunction::computeYawHeading will need to be calculated first
        // and fed into #computeVelocity as head_psi
        double computeVelocity(
            geometry_msgs::msg::Vector3 candidate_velocity,
            double k_z,
            double k_psi,
            double head_psi) const;
    private:
        // Reference drone_state.hpp and drone_state.cpp for an example.
        // drone_state uses a topic (which is more complex) but you only need to use a one-time service call
        // You will need to have VelocityFunction accept a node in its constructor
        // Then you will need to create a service call and initialize vel_max after object initialization

        // Reads vel_max from MAVROS
        void getMaxVelocity();

        rclcpp::Node *node_;
        rclcpp::Client<mavros_msgs::srv::ParamGet>::SharedPtr param_client_;
        rclcpp::TimerBase::SharedPtr retry_timer_;

        std::atomic<double> vel_max_x_{0.0};
};
#endif