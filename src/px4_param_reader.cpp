#include "reactive_uav_navigation/px4_param_reader.hpp"

#include <chrono>
#include <mutex>
#include <set>
#include <utility>

#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rclcpp/rclcpp.hpp>

namespace reactive_uav_navigation {

double rosDoubleParam(rclcpp::Node* node, const std::string& name, double default_value) {
    if (!node->has_parameter(name)) {
        node->declare_parameter(name, default_value);
    }
    return node->get_parameter(name).as_double();
}

struct Px4ParamReader::Impl {
    Impl(rclcpp::Node* node, const std::vector<std::string>& names, Callback on_value)
        : node_(node), on_value_(std::move(on_value)), pending_(names.begin(), names.end()) {
        // MAVROS offers the PX4 parameters as ROS parameters of its /mavros/param node
        client_ = node_->create_client<rcl_interfaces::srv::GetParameters>("/mavros/param/get_parameters");
        retry_timer_ = node_->create_wall_timer(std::chrono::seconds(1), [this]() {
            requestPending();
        });
    }

    void requestPending() {
        if (!client_->service_is_ready()) {
            // MAVROS not available
            return;
        }

        std::vector<std::string> to_request;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            for (const std::string& name : pending_) {
                if (in_flight_.insert(name).second) {
                    to_request.push_back(name);
                }
            }
        }

        for (const std::string& name : to_request) {
            auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
            request->names.push_back(name);

            client_->async_send_request(
                request,
                [this, name](rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture future) {
                    auto response = future.get();

                    // the value is a double or an integer, or not set while
                    // MAVROS has not received the parameter from PX4 yet
                    double value = 0.0;
                    if (!response->values.empty()) {
                        if (response->values[0].type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE) {
                            value = response->values[0].double_value;
                        }
                        if (response->values[0].type == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER) {
                            value = static_cast<double>(response->values[0].integer_value);
                        }
                    }
                    const bool ok = value > 0.0;

                    bool all_received = false;
                    {
                        std::lock_guard<std::mutex> lock(mutex_);
                        in_flight_.erase(name);
                        if (ok) {
                            pending_.erase(name);
                        }
                        all_received = pending_.empty();
                    }

                    if (ok) {
                        on_value_(name, value);
                    } else {
                        RCLCPP_WARN_THROTTLE(
                            node_->get_logger(), *node_->get_clock(), 10000,
                            "Could not read PX4 parameter %s, will retry", name.c_str());
                    }
                    if (all_received) {
                        retry_timer_->cancel();
                    }
                });
        }
    }

    rclcpp::Node* node_;
    Callback on_value_;
    rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr retry_timer_;

    std::mutex mutex_;
    std::set<std::string> pending_;    // not received yet
    std::set<std::string> in_flight_;  // request sent, no answer yet
};

Px4ParamReader::Px4ParamReader(rclcpp::Node* node, const std::vector<std::string>& names,
                               Callback on_value)
    : impl_(std::make_shared<Impl>(node, names, std::move(on_value))) {}

}  // namespace reactive_uav_navigation
