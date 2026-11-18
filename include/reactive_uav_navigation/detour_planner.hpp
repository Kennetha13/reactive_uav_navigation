// detour_planner.hpp
//
// Finds a way around an obstacle that lies across the path.
//
// The velocity planner only looks a few meters ahead and has no map. Facing a
// wall it turns a little to one side and then stops, even when there is an
// opening a few meters along the wall. DetourPlanner gives it somewhere else
// to head for.
//
// An obstacle whose end is in sight, such as a block in a corridor:
//
//   1. An obstacle point closer than block_distance to the straight line to
//      the waypoint is in the way.
//   2. The obstacle reaches to the left and to the right as far as its points
//      lie close together. Where they stop is an end of it.
//   3. The drone heads for a point pass_distance past the nearer end. An end
//      only counts if the camera sees the space past it and nothing lies on
//      the way there.
//   4. When the obstacle is pass_distance away from the straight line to the
//      waypoint, the drone heads for the waypoint again.
//
// An obstacle whose end is not in sight, such as a long wall:
//
//   1. The drone flies straight at the waypoint.
//   2. If an obstacle is in the way and the drone gets no closer for
//      stuck_time seconds, it takes a side step: a temporary target
//      step_length meters to the side of the spot where it got stuck. The
//      first side is the one where the obstacle seemed to end sooner, or the
//      right if nothing is known.
//   3. At the end of the side step it tries the waypoint again. If it is
//      still stuck, the next side step goes one step_length further out
//      on the same side: 2.5 m, 5 m, 7.5 m and 10 m from that spot.
//      The side steps also back off back_distance from the obstacle.
//   4. If a side step itself is blocked, or max_steps_per_side steps did not
//      help, the other side is tried.
//   5. When both sides have been tried it gives up, and the caller reports
//      BLOCKED.
//
// The detour is over as soon as the drone is closer to the waypoint than it
// was when the detour began.
//
// No ROS dependencies. Positions are in the world frame.

#ifndef DETOUR_PLANNER_HPP_
#define DETOUR_PLANNER_HPP_

#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace reactive_uav_navigation {

struct DetourParams {
    double stuck_time = 4.0;      // s without getting closer before a side step starts
    double min_progress = 0.2;    // m closer that counts as progress
    double step_length = 2.5;     // m, each side step reaches this much further to the side
    double reach_distance = 0.5;  // m, a side step is finished when this little of it is left
    double back_distance = 1.0;   // m, the side steps run this far behind the spot where the drone got stuck
    int max_steps_per_side = 4;   // side steps on one side before the other side is tried
    double obstacle_memory_time = 10.0;  // s an obstacle counts as "in the way" after it was last seen

    // going around an obstacle whose end is in sight
    double block_distance = 1.0;  // m, a point this close to the straight line to the waypoint is in the way
    double pass_distance = 1.3;   // m, the drone aims this far past the end of the obstacle
    double min_aim_ahead = 0.5;   // m, the aiming point is at least this far ahead of the drone
    double free_width = 0.5;      // m, a point this close to the way to the aiming point blocks that way
    double max_side = 4.0;        // m, an aiming point further to the side than this is not used
};

class DetourPlanner {
public:
    DetourParams params;

    // Forgets everything. Call when the waypoint changes.
    void reset();

    // Gives the planner the obstacle points known right now and half of the
    // camera's horizontal field of view (rad). Call before update().
    void setObstacles(const std::vector<geometry_msgs::msg::Vector3>& points, double half_view);

    // Call once per control cycle. Returns the point to head for now: the
    // waypoint itself, a point past the end of an obstacle, or the target of
    // a side step.
    //   now:                 current time (s)
    //   position, yaw:       drone position and heading
    //   waypoint:            the waypoint of the path the drone is heading for
    //   obstacle_in_the_way: an obstacle is close in the direction being flown
    geometry_msgs::msg::Point update(double now,
                                     const geometry_msgs::msg::Point& position,
                                     double yaw,
                                     const geometry_msgs::msg::Point& waypoint,
                                     bool obstacle_in_the_way);

    // true while the drone goes around an obstacle or searches a way around
    bool isDetouring() const { return detouring_ || going_around_; }

    // true when both sides were tried without success
    bool hasGivenUp() const { return given_up_; }

private:
    // Looks for the end of an obstacle that lies on the straight line to the
    // waypoint. Returns true and fills in the point to head for when the
    // drone can go around it.
    bool findWayAround(const geometry_msgs::msg::Point& position,
                       double yaw,
                       const geometry_msgs::msg::Point& waypoint,
                       geometry_msgs::msg::Point& around);

    // true when the drone can head for the point (x, y): it is not too far
    // to the side, the camera sees that way and nothing lies in between.
    // Only points between the heights lowest and highest count.
    bool canHeadFor(const geometry_msgs::msg::Point& position, double yaw,
                    double x, double y, double side_distance,
                    double lowest, double highest) const;

    // begins the next side step
    void startSideStep(double now,
                       const geometry_msgs::msg::Point& position,
                       const geometry_msgs::msg::Point& waypoint,
                       double distance_to_waypoint);

    // how far to the current side the drone is from where the detour began (m)
    double sideProgress(const geometry_msgs::msg::Point& position) const;

    // moves on to the other side, or gives up if both were tried
    void switchSide();

    bool started_ = false;
    bool side_stepping_ = false;   // false: heading for the waypoint, true: on a side step
    bool detouring_ = false;
    bool given_up_ = false;

    // heading for the waypoint
    double best_distance_ = 0.0;        // closest the drone has been to the waypoint
    double stuck_seconds_ = 0.0;        // time spent facing the waypoint without getting closer
    double last_update_time_ = 0.0;     // time of the previous update() call
    bool obstacle_seen_ = false;        // an obstacle has been in the way
    double last_obstacle_time_ = 0.0;   // when one was last in the way
    double detour_start_distance_ = 0.0;

    // where the detour began and the direction to the waypoint from there
    geometry_msgs::msg::Point detour_start_;
    double detour_dx_ = 1.0;
    double detour_dy_ = 0.0;

    // side steps
    double side_ = 1.0;                 // +1 right of the line to the waypoint, -1 left
    int steps_on_side_ = 0;
    int sides_tried_ = 0;
    geometry_msgs::msg::Point side_target_;
    double side_reach_ = 0.0;           // how far to the side the current side step goes
    double side_best_distance_ = 0.0;   // least that was still left of it
    double side_progress_time_ = 0.0;

    // going around an obstacle whose end is in sight
    std::vector<geometry_msgs::msg::Vector3> obstacle_points_;
    double half_view_ = 0.0;            // half of the camera's horizontal field of view (rad)
    bool going_around_ = false;
    double around_side_ = 0.0;          // side the drone goes around on: +1 right, -1 left, 0 none
    double nearer_side_ = 0.0;          // side where the obstacle in the way ends sooner, 0 if nothing is in the way
};

}  // namespace reactive_uav_navigation

#endif  // DETOUR_PLANNER_HPP_
