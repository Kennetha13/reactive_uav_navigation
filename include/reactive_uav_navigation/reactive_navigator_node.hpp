#ifndef REACTIVE_NAVIGATOR_HPP_
#define REACTIVE_NAVIGATOR_HPP_

#include <mutex>
#include <cmath>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "reactive_uav_navigation/common.hpp"
#include "reactive_uav_navigation/depth_ray.hpp"
#include "reactive_uav_navigation/detour_planner.hpp"
#include "reactive_uav_navigation/drone_state.hpp"
#include "reactive_uav_navigation/distance_function.hpp"
#include "reactive_uav_navigation/objective_function.hpp"
#include "reactive_uav_navigation/obstacle_memory.hpp"
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

  // Gives the distance function the obstacle points from the latest depth
  // image and tells it which directions the camera can see.
  void updateObstacles(const geometry_msgs::msg::Point& position, double yaw);

  // Publishes a drone-centric velocity [vx, vz, wz] on cmd_vel_unstamped,
  // which MAVROS reads in the world frame (ENU): forward speed is rotated by
  // the current yaw into world x/y.
  void publishVelocity(const CandidateVelocity& v, double yaw);

  // Publishes a zero velocity, which makes the drone hover.
  void publishHover();

  // Feeds one control cycle into the navigation state machine, publishes the
  // state and prints it to the terminal when it changes.
  void updateNavState(const NavInputs& in);

  // Subscription and variables for handling global path
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
  nav_msgs::msg::Path global_path_;
  mutable std::mutex path_mutex_;
  size_t current_index_{0};

  // Shared object for the drone's current state
  std::shared_ptr<DroneState> drone_state_;

  // DWA-3D components, built once in the constructor and reused every cycle
  std::unique_ptr<DepthRayProcessor> depth_processor_;    // depth camera -> obstacle points
  ObstacleMemory obstacle_memory_;                        // points that left the camera's view
  std::unique_ptr<DistanceFunction> distance_function_;  // Dist(v) rays, used by Va
  std::unique_ptr<VssCollector> vss_collector_;          // Vr = Vs ∩ Vd ∩ Va
  std::unique_ptr<ObjectiveFunction> objective_function_; // G(v)

  // Side steps around an obstacle that lies across the path
  DetourPlanner detour_planner_;
  size_t detour_index_{0};           // waypoint the detour planner is working on
  bool obstacle_in_the_way_{false};  // result of the last control cycle

  // FOLLOWING_PATH / AVOIDING_OBSTACLE / BLOCKED / GOAL_REACHED / WAITING_FOR_PATH
  std::unique_ptr<NavigationStateMachine> nav_state_;

  // Tolerances and ranges, read from ROS parameters
  double waypoint_tolerance_{0.3};
  double search_range_{3.0};
  bool use_depth_camera_{true};
  double depth_timeout_{1.0};

  // Metrics for the path being flown
  double path_start_time_{0.0};
  double distance_flown_{0.0};
  int blocked_count_{0};
  bool has_last_position_{false};
  geometry_msgs::msg::Point last_position_;

  // Publishers for drone movement commands and the navigation state
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}

#endif  // REACTIVE_NAVIGATOR_HPP_
