#include "reactive_uav_navigation/velocity_function.hpp"
#include <chrono>

VelocityFunction::VelocityFunction(rclcpp::Node *node) : node_(node){
    param_client_ = node_->create_client<mavros_msgs::srv::ParamGet>("/mavros/param/get");
    retry_timer_ = node_->create_wall_timer(std::chrono::seconds(1), [this]() {
        getMaxVelocity();
    });
}

void VelocityFunction::getMaxVelocity() {
    if (!param_client_->service_is_ready()) {
        // MAVROS not available
        return;
    }

    auto request = std::make_shared<mavros_msgs::srv::ParamGet::Request>();
    request->param_id = "MPC_XY_VEL_MAX";

    param_client_->async_send_request(
        request,
        [this](rclcpp::Client<mavros_msgs::srv::ParamGet>::SharedFuture future){
            auto response = future.get();
            if (response->success && response->value.real > 0.0) {
                vel_max_x_ = response->value.real;
                retry_timer_->cancel();
                RCLCPP_INFO(node_->get_logger(), "vel_max_x_ = %.2f m/s", vel_max_x_.load());
            }
        });
}

// Vel(v) from the objective function (Fig. 21)
// Get a single candidate velocity in 3d vector form
// Returns candidate_velocity.x / vel_max, negative value if the velocity is backwards
double VelocityFunction::computeVelocity(
    geometry_msgs::msg::Vector3 candidate_velocity,
    double k_z,
    double k_psi,
    double head_psi) const {

    const bool forward = (k_z > k_psi) || (head_psi > 0.5 && k_z < k_psi);
    const double vel_max = vel_max_x_.load();

    // avoiding div by 0
    if (!forward || vel_max <= 0.0) {
        return 0.0;
    }

    return candidate_velocity.x / vel_max;
}