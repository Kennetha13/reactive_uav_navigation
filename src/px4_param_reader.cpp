#include "reactive_uav_navigation/px4_param_reader.hpp"

#include <chrono>
#include <mutex>
#include <set>
#include <utility>

#include <mavros_msgs/srv/param_get.hpp>
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
        client_ = node_->create_client<mavros_msgs::srv::ParamGet>("/mavros/param/get");
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
            auto request = std::make_shared<mavros_msgs::srv::ParamGet::Request>();
            request->param_id = name;

            client_->async_send_request(
                request,
                [this, name](rclcpp::Client<mavros_msgs::srv::ParamGet>::SharedFuture future) {
                    auto response = future.get();
                    const bool ok = response->success && response->value.real > 0.0;

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
                        on_value_(name, response->value.real);
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
    rclcpp::Client<mavros_msgs::srv::ParamGet>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr retry_timer_;

    std::mutex mutex_;
    std::set<std::string> pending_;    // not received yet
    std::set<std::string> in_flight_;  // request sent, no answer yet
};

Px4ParamReader::Px4ParamReader(rclcpp::Node* node, const std::vector<std::string>& names,
                               Callback on_value)
    : impl_(std::make_shared<Impl>(node, names, std::move(on_value))) {}

}  // namespace reactive_uav_navigation
