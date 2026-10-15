#include "drone_state/drone_state.hpp"
#include <algorithm>
#include <cmath>
#include <functional>

// Converts quaternion to roll
float quaternion_to_roll(float x, float y, float z, float w){
    return std::atan2(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y)); // Returns roll in radians
}

//Converts quaternion to pitch
float quaternion_to_pitch(float x, float y, float z, float w){
    float pitch = 2.0f * (w * y - z * x);
    pitch = std::max(-1.0f, std::min(1.0f, pitch)); // Protects from small floating point errors

    return std::asin(pitch); // Returns pitch in radians
}

float quaternion_to_yaw(float x, float y, float z, float w){
    return std::atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
}

// Constructor, calls the constructor of the base class Node
DroneState::DroneState():Node("drone_state_node"){
    auto odometry_qos = rclcpp::SensorDataQoS();// Keeps the 10 most recent odometry messages

    odometry_subscription_= this->create_subscription<nav_msgs::msg::Odometry>("odom", odometry_qos, std::bind(&DroneState::odometry_listener, this, std::placeholders::_1)); // Subscribes to ROS 2 topic odometry

    RCLCPP_INFO(this->get_logger(), "Starting DroneState"); // Message when drone starts
}

geometry_msgs::msg::Point DroneState::get_current_position() const{
    std::lock_guard<std::mutex> lock(state_mutex_);

    return current_position_; // Return copy of current position
}

geometry_msgs::msg::Twist DroneState::get_current_velocity() const{
    std::lock_guard<std::mutex> lock(state_mutex_);

    return current_velocity_; // Return copy of current velocity
}

double DroneState::get_roll() const{ // Returns roll in radians
    std::lock_guard<std::mutex> lock(state_mutex_);

    return roll_;
}

double DroneState::get_pitch() const{ // Returns pitch in radians
    std::lock_guard<std::mutex> lock(state_mutex_);

    return pitch_;
}

double DroneState::get_yaw() const{ // Returns yaw in radians
    std::lock_guard<std::mutex> lock(state_mutex_);

    return yaw_;
}

void DroneState::odometry_listener(const nav_msgs::msg::Odometry::SharedPtr message){
    const auto& quaternion = message->pose.pose.orientation; // Get the orientation quaternion

    std::lock_guard<std::mutex> lock(state_mutex_);

    current_position_ = message->pose.pose.position; // Save the current position
    current_velocity_ = message->twist.twist; // Save the current linear and angular velocity

    roll_ = quaternion_to_roll(quaternion.x, quaternion.y, quaternion.z, quaternion.w); // Converting quaternion to roll
    pitch_ = quaternion_to_pitch(quaternion.x, quaternion.y, quaternion.z, quaternion.w); // Converting quaternion to pitch
    yaw_ = quaternion_to_yaw(quaternion.x, quaternion.y, quaternion.z, quaternion.w); // Converting quaternion to yaw

// Prints the drones state one per second
RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,"Position: x=%.2f, y=%.2f, z=%.2f | Velocity: x=%.2f, y=%.2f, z=%.2f |  Roll: %.2f | Pitch: %.2f | Yaw: %.2f", current_position_.x, current_position_.y, current_position_.z, current_velocity_.linear.x, current_velocity_.linear.y, current_velocity_.linear.z, roll_ , pitch_, yaw_);
}

int main(int argc, char* argv[]){ // Accepts argc argument count and argv argument vector

    rclcpp::init(argc,argv);

    auto node = std::make_shared<DroneState>(); // Shared pointer for efficiency and safety

    rclcpp::spin(node); // Keeps the node running so it can recieve ROS 2 messages
    rclcpp::shutdown(); // Shuts down the ROS 2 after node stops running

    return 0;
}