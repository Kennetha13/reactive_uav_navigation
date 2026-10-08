#ifndef DRONE_STATE_HPP // Following code is compiled if not defined, goes to end if if it has been defined
#define DRONE_STATE_HPP // Executes if ifndef is true

#include <mutex>
#include "rclcpp/rclcpp.hpp" // The client libraries for ROS 2
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"

class DroneState : public rclcpp::Node{ // Inherits from ROS 2 Node

    public:
            DroneState(); // Constructor

            geometry_msgs::msg::Point get_current_position() const; // Drone's current position
            geometry_msgs::msg::Twist get_current_velocity() const; // Drone's latest linear and angular velocity

            double get_roll() const; // Latest roll angle in radians
            double get_pitch() const; // Latest pitch angle in radians
            double get_yaw() const; // Latest yaw angle in radians

    private:

            void odometry_listener(const nav_msgs::msg::Odometry::SharedPtr message); // Runs when a new odometry message arrives
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_; // Subscription to recieve drone odometry messages

            geometry_msgs::msg::Point current_position_; // Stores the current position of the drone
            geometry_msgs::msg::Twist current_velocity_; // Stores the current velocity of the drone;

            // Stores roll, pitch, and yaw in radians
            double roll_{0.0};
            double pitch_{0.0};
            double yaw_{0.0};

            mutable std::mutex state_mutex_;
};

#endif