#ifndef REACTIVE_NAVIGATOR_HPP_
#define REACTIVE_NAVIGATOR_HPP_

#include <mutex>
#include <cmath>
#include <memory>
#include <optional>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "reactive_uav_navigation/common.hpp"
#include "reactive_uav_navigation/drone_state.hpp"
#include "reactive_uav_navigation/distance_function.hpp"
#include "reactive_uav_navigation/objective_function.hpp"
#include "reactive_uav_navigation/navigation_state.hpp"
#include "reactive_uav_navigation/vss_collector.hpp"

namespace reactive_uav_navigation {

class ReactiveNavigator : public rclcpp::Node
{
public:
  ReactiveNavigator();
  nav_msgs::msg::Path get_global_path() const;

private:
  void path_callback(const nav_msgs::msg::Path::SharedPtr msg);

  void control_loop();

  // One candidate velocity, its objective score G(v), and the individual
  // Heading/Dist/Vel scores behind it (scores.distance feeds the navigation state)
  struct ScoredVelocity {
    CandidateVelocity velocity;
    double score;
    ObjectiveScores scores;
    double ray_cast_distance;
  };

  // Scores every candidate in Vr with the objective function and returns the
  // one with the highest G(v). Candidates with a NaN score are skipped; ties
  // go to the first. Returns std::nullopt if there is nothing to choose from.
  std::optional<ScoredVelocity> selectBestVelocity(const std::vector<CandidateVelocity>& vr);

  // Publishes a drone-centric velocity [vx, vz, wz] on cmd_vel_unstamped,
  // which MAVROS reads in the world frame (ENU): forward speed is rotated by
  // the current yaw into world x/y.
  void publishVelocity(const CandidateVelocity& v, double yaw);

  // Feeds one control cycle into the navigation state machine and prints the
  // new state to the terminal when it changes.
  void updateNavState(const NavInputs& in);

  // Subscription and variables for handling global path
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
  nav_msgs::msg::Path global_path_;
  mutable std::mutex path_mutex_;
  size_t current_index_{0};

  // Shared object for the drone's current state
  std::shared_ptr<DroneState> drone_state_;

  // DWA-3D components, built once in the constructor and reused every cycle
  std::unique_ptr<DistanceFunction> distance_function_;  // Dist(v) rays, used by Va
  std::unique_ptr<VssCollector> vss_collector_;          // Vr = Vs ∩ Vd ∩ Va
  std::unique_ptr<ObjectiveFunction> objective_function_; // G(v)

  // FOLLOWING_PATH / AVOIDING_OBSTACLE / BLOCKED / GOAL_REACHED / WAITING_FOR_PATH
  NavigationStateMachine nav_state_;

  // Publisher for drone movement commands
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}

#endif  // REACTIVE_NAVIGATOR_HPP_
