// Repeatable simulation tests of the whole planner without ROS. A simple
// drone model follows the chosen velocity each control period, the same way
// ReactiveNavigator::control_loop() does. The drone only sees obstacle points
// inside its camera's field of view.
//
//   1. open space:        the drone follows the path and reaches the goal
//   2. pillar on the path: the drone goes around it without touching it
//   3. wall across the path: the drone reports BLOCKED and never touches it
//   4. loop path:         a path that ends where it starts is flown all the way
//   5. goal near a wall:  a goal just in front of a wall is still reached
//   6. turn beside a wall: the drone turns around on the spot next to a wall
//   7. through a gap:     a path that knows about the walls leads through a
//                         2.5 m gap, the drone keeps clear of both sides
//   8. wall with a gap:   the path runs through a wall that has a gap to the
//                         right, the drone side-steps to it and goes through
//   9. boxed in at a wall: the drone starts so close to a wall that nothing
//                         may move it, and still turns away and leaves
//  10. gap on the left:   the right side is closed off, the drone sees the
//                         left end of the wall and goes around it
//  11. doorway on the left: the path runs through a long wall, the doorway
//                         is 6 m to the left
//  12. block in a corridor: the drone goes around a block on the side where
//                         it ends sooner, without stopping in front of it

// keep the asserts even in a Release build
#undef NDEBUG
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

#include "reactive_uav_navigation/detour_planner.hpp"
#include "reactive_uav_navigation/distance_function.hpp"
#include "reactive_uav_navigation/drone_state.hpp"
#include "reactive_uav_navigation/navigation_state.hpp"
#include "reactive_uav_navigation/objective_function.hpp"
#include "reactive_uav_navigation/obstacle_memory.hpp"
#include "reactive_uav_navigation/vss_collector.hpp"

using namespace reactive_uav_navigation;

const double DRONE_RADIUS = 0.3;
const double SEARCH_RANGE = 3.0;
const double WAYPOINT_TOLERANCE = 0.3;

// the drone's center must stay this far from walls it passes (m)
const double CORNER_CLEARANCE = 0.6;

// horizontal and vertical field of view of the simulated depth camera (rad)
const double CAMERA_FOV = 1.274;
const double CAMERA_VERTICAL_FOV = 1.013;

struct SimResult {
    bool goal_reached = false;
    bool was_blocked = false;
    bool was_avoiding = false;
    double closest_obstacle = 1000.0;  // smallest distance from the drone center to an obstacle point
    double closest_x = 0.0;            // where the drone was at that moment
    double closest_y = 0.0;
    double distance_flown = 0.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

geometry_msgs::msg::Vector3 makePoint(double x, double y, double z) {
    geometry_msgs::msg::Vector3 p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

double distanceBetween(double x1, double y1, double z1, double x2, double y2, double z2) {
    return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));
}

// angle a - b wrapped into [-pi, pi]
double angleDifference(double a, double b) {
    double difference = a - b;
    while (difference > M_PI) {
        difference = difference - 2.0 * M_PI;
    }
    while (difference < -M_PI) {
        difference = difference + 2.0 * M_PI;
    }
    return difference;
}

// flies the path for `seconds` of simulated time, starting at the first
// waypoint and facing the direction start_yaw
SimResult runSim(const std::vector<geometry_msgs::msg::Vector3>& waypoints,
                 const std::vector<geometry_msgs::msg::Vector3>& obstacles,
                 double seconds, double start_yaw = 0.0) {
    // same grid as the node: 4 forward speeds, 7 vertical speeds, 19 yaw rates
    VelocitySteps steps;
    steps.vx_step = 0.1;
    steps.vz_step = 0.1;
    steps.wz_step = degToRad(5.0);

    // same safety margin as the node
    AdmissibleParams admissible_params;
    admissible_params.safety_margin = 0.5;
    VssCollector collector{std::make_unique<MaximumVelocitiesSpace>(VelocityLimits{}, steps),
                           AccelerationLimits{}, admissible_params};

    DistanceParams distance_params;
    distance_params.R_drone = DRONE_RADIUS;
    distance_params.r_search = SEARCH_RANGE;
    DistanceFunction distance_function{distance_params};

    ObstacleMemory obstacle_memory;
    DetourPlanner detour_planner;
    size_t detour_index = 0;
    bool obstacle_in_the_way = false;

    DroneState drone_state;
    ObjectiveFunction objective_function{drone_state, geometry_msgs::msg::Point{}, distance_function};

    NavStateParams nav_params;
    nav_params.goal_tolerance = WAYPOINT_TOLERANCE;
    nav_params.stall_timeout = 10.0;
    NavigationStateMachine nav_state{nav_params};

    // the path as poses, like the node receives it
    std::vector<geometry_msgs::msg::PoseStamped> poses;
    for (size_t i = 0; i < waypoints.size(); i++) {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = waypoints[i].x;
        pose.pose.position.y = waypoints[i].y;
        pose.pose.position.z = waypoints[i].z;
        poses.push_back(pose);
    }

    SimResult result;
    double x = waypoints[0].x;
    double y = waypoints[0].y;
    double z = waypoints[0].z;
    double yaw = start_yaw;
    CandidateVelocity last_command;
    size_t current_index = 0;
    size_t last_index = waypoints.size() - 1;

    int total_steps = static_cast<int>(seconds / CONTROL_PERIOD);
    for (int step = 0; step < total_steps; step++) {
        double now = step * CONTROL_PERIOD;
        drone_state.setPose(x, y, z, yaw);

        // the camera only sees points that are near and inside its field of view
        std::vector<geometry_msgs::msg::Vector3> visible;
        for (size_t i = 0; i < obstacles.size(); i++) {
            double d = distanceBetween(x, y, z, obstacles[i].x, obstacles[i].y, obstacles[i].z);
            if (d < result.closest_obstacle) {
                result.closest_obstacle = d;
                result.closest_x = x;
                result.closest_y = y;
            }
            double point_yaw = std::atan2(obstacles[i].y - y, obstacles[i].x - x);
            double point_pitch = std::atan2(obstacles[i].z - z,
                                            std::hypot(obstacles[i].x - x, obstacles[i].y - y));
            if (d < SEARCH_RANGE + 1.0 &&
                std::fabs(angleDifference(point_yaw, yaw)) <= CAMERA_FOV / 2.0 &&
                std::fabs(point_pitch) <= CAMERA_VERTICAL_FOV / 2.0) {
                visible.push_back(obstacles[i]);
            }
        }
        // points that just left the view are remembered for a while
        obstacle_memory.update(visible, now, x, y, z, yaw,
                               CAMERA_FOV / 2.0, CAMERA_VERTICAL_FOV / 2.0);
        distance_function.setObstaclePoints(obstacle_memory.points());
        distance_function.setVisibleSector(yaw, CAMERA_FOV / 2.0, CAMERA_VERTICAL_FOV / 2.0);
        detour_planner.setObstacles(obstacle_memory.points(), CAMERA_FOV / 2.0);

        // move on to the next waypoint once this one is reached
        while (current_index < last_index &&
               distanceBetween(x, y, z, waypoints[current_index].x, waypoints[current_index].y,
                               waypoints[current_index].z) < WAYPOINT_TOLERANCE) {
            current_index++;
        }

        // a new waypoint starts without a detour
        if (current_index != detour_index) {
            detour_planner.reset();
            detour_index = current_index;
        }

        // the point to head for: the waypoint, or a side step around an obstacle
        geometry_msgs::msg::Point position;
        position.x = x;
        position.y = y;
        position.z = z;
        const geometry_msgs::msg::Point target = detour_planner.update(
            now, position, yaw, poses[current_index].pose.position, obstacle_in_the_way);

        // candidates, then the best one by G(v)
        DronePose pose;
        pose.position = makePoint(x, y, z);
        pose.yaw = yaw;
        std::vector<CandidateVelocity> vr = collector.computeVr(distance_function, last_command, pose);

        objective_function.setGoal(target);
        objective_function.setMaxSpeed(collector.maximum().limits().vx_max);
        BestVelocity best = objective_function.selectBest(vr);

        // clearance straight toward the waypoint, if the camera can see that way
        double goal_clearance = 1.0;
        double goal_yaw = std::atan2(target.y - y, target.x - x);
        double goal_pitch = std::atan2(target.z - z, std::hypot(target.x - x, target.y - y));
        if (distance_function.isDirectionVisible(goal_yaw)) {
            double distance_to_target = distanceBetween(x, y, z, target.x, target.y, target.z);
            goal_clearance = distance_function.computeDistanceUpTo(
                pose.position, goal_yaw, goal_pitch, distance_to_target);
        }

        NavInputs in;
        in.now = now;
        in.has_path = true;
        in.distance_to_goal = distanceBetween(x, y, z, waypoints[last_index].x,
                                              waypoints[last_index].y, waypoints[last_index].z);
        in.on_last_waypoint = (current_index == last_index);
        in.remaining_path_length = remainingPathLength(poses, current_index, x, y, z);
        in.safe_velocity_found = best.found && best.can_move;
        in.chosen_dist = best.found ? std::min(best.scores.distance, goal_clearance) : 0.0;
        obstacle_in_the_way = in.chosen_dist < nav_params.avoid_enter_dist;
        in.detouring = detour_planner.isDetouring();
        nav_state.update(in);

        if (nav_state.state() == NavState::BLOCKED) {
            result.was_blocked = true;
        }
        if (nav_state.state() == NavState::AVOIDING_OBSTACLE) {
            result.was_avoiding = true;
        }
        if (nav_state.state() == NavState::GOAL_REACHED) {
            result.goal_reached = true;
            break;
        }

        // hover when told to, otherwise fly the chosen velocity for one period.
        // While BLOCKED the drone may still turn on the spot.
        bool turn_only = std::fabs(best.velocity.vx) < 1e-9 && std::fabs(best.velocity.vz) < 1e-9;
        if (!best.found || nav_state.mustHover()) {
            if (best.found && turn_only && nav_state.state() == NavState::BLOCKED) {
                last_command = best.velocity;
            } else {
                last_command = CandidateVelocity{};
            }
        } else {
            last_command = best.velocity;
        }
        yaw = yaw + last_command.wz * CONTROL_PERIOD;
        double step_x = last_command.vx * std::cos(yaw) * CONTROL_PERIOD;
        double step_y = last_command.vx * std::sin(yaw) * CONTROL_PERIOD;
        double step_z = last_command.vz * CONTROL_PERIOD;
        x = x + step_x;
        y = y + step_y;
        z = z + step_z;
        result.distance_flown += std::sqrt(step_x * step_x + step_y * step_y + step_z * step_z);
    }

    result.x = x;
    result.y = y;
    result.z = z;
    return result;
}

void printResult(const char* name, const SimResult& r) {
    std::cout << name << ": goal_reached=" << r.goal_reached << " blocked=" << r.was_blocked
              << " avoiding=" << r.was_avoiding << " closest_obstacle=" << r.closest_obstacle
              << " at (" << r.closest_x << ", " << r.closest_y << ")"
              << " flown=" << r.distance_flown << " end=(" << r.x << ", " << r.y << ", " << r.z
              << ")" << std::endl;
}

int main() {
    // The obstacle memory holds 600 points outside the view. When more leave
    // the view, the newest must stay: 600 points leave it at time 0, then 100
    // more at time 1, and those 100 must all still be remembered.
    {
        ObstacleMemory memory;
        std::vector<geometry_msgs::msg::Vector3> first;
        for (int i = 0; i < 600; i++) {
            first.push_back(makePoint(3.0, 0.0, 0.0));
        }
        std::vector<geometry_msgs::msg::Vector3> second;
        for (int i = 0; i < 100; i++) {
            second.push_back(makePoint(0.0, 3.0, 0.0));
        }
        std::vector<geometry_msgs::msg::Vector3> nothing;

        // the camera at (0, 0, 0) looks toward +x, then +y, then -x
        memory.update(first, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.5);
        memory.update(second, 1.0, 0.0, 0.0, 0.0, M_PI / 2.0, 0.5, 0.5);
        memory.update(nothing, 2.0, 0.0, 0.0, 0.0, M_PI, 0.5, 0.5);

        int newest = 0;
        for (size_t i = 0; i < memory.points().size(); i++) {
            if (memory.points()[i].y > 2.0) {
                newest++;
            }
        }
        std::cout << "obstacle memory: " << memory.points().size() << " points, "
                  << newest << " of the newest 100" << std::endl;
        assert(memory.points().size() == 600);
        assert(newest == 100);
    }

    // 1. open space, with a turn and a change of altitude
    {
        std::vector<geometry_msgs::msg::Vector3> path;
        path.push_back(makePoint(0.0, 0.0, 2.5));
        path.push_back(makePoint(5.0, 0.0, 2.0));
        path.push_back(makePoint(5.0, 5.0, 2.0));
        std::vector<geometry_msgs::msg::Vector3> no_obstacles;

        SimResult r = runSim(path, no_obstacles, 120.0);
        printResult("open space", r);
        assert(r.goal_reached);
        assert(!r.was_blocked);
        assert(!r.was_avoiding);
        assert(distanceBetween(r.x, r.y, r.z, 5.0, 5.0, 2.0) < WAYPOINT_TOLERANCE);
    }

    // 2. a pillar standing on the straight line to the goal
    {
        std::vector<geometry_msgs::msg::Vector3> path;
        path.push_back(makePoint(0.0, 0.0, 2.0));
        path.push_back(makePoint(8.0, 0.0, 2.0));

        std::vector<geometry_msgs::msg::Vector3> pillar;
        for (double pz = 0.0; pz <= 4.0; pz += 0.25) {
            pillar.push_back(makePoint(4.0, 0.0, pz));
            pillar.push_back(makePoint(4.0, 0.2, pz));
            pillar.push_back(makePoint(4.0, -0.2, pz));
        }

        SimResult r = runSim(path, pillar, 180.0);
        printResult("pillar", r);
        assert(r.goal_reached);
        assert(r.was_avoiding);
        assert(r.closest_obstacle > DRONE_RADIUS);
    }

    // 3. a wall too wide and too tall to get around
    {
        std::vector<geometry_msgs::msg::Vector3> path;
        path.push_back(makePoint(0.0, 0.0, 2.0));
        path.push_back(makePoint(8.0, 0.0, 2.0));

        std::vector<geometry_msgs::msg::Vector3> wall;
        for (double py = -40.0; py <= 40.0; py += 0.3) {
            for (double pz = -20.0; pz <= 40.0; pz += 0.3) {
                wall.push_back(makePoint(4.0, py, pz));
            }
        }

        SimResult r = runSim(path, wall, 400.0);
        printResult("wall", r);
        assert(!r.goal_reached);
        assert(r.was_blocked);
        assert(r.closest_obstacle > DRONE_RADIUS);
    }

    // 4. a square loop that ends where it starts
    {
        std::vector<geometry_msgs::msg::Vector3> path;
        path.push_back(makePoint(0.0, 0.0, 2.0));
        path.push_back(makePoint(3.0, 0.0, 2.0));
        path.push_back(makePoint(3.0, 3.0, 2.0));
        path.push_back(makePoint(0.0, 3.0, 2.0));
        path.push_back(makePoint(0.0, 0.0, 2.0));
        std::vector<geometry_msgs::msg::Vector3> no_obstacles;

        SimResult r = runSim(path, no_obstacles, 180.0);
        printResult("loop", r);
        assert(r.goal_reached);
        assert(r.distance_flown > 10.0);
    }

    // 5. the goal is 1.5 m in front of a wall
    {
        std::vector<geometry_msgs::msg::Vector3> path;
        path.push_back(makePoint(0.0, 0.0, 2.0));
        path.push_back(makePoint(3.0, 0.0, 2.0));

        std::vector<geometry_msgs::msg::Vector3> wall;
        for (double py = -10.0; py <= 10.0; py += 0.3) {
            for (double pz = -5.0; pz <= 15.0; pz += 0.3) {
                wall.push_back(makePoint(4.5, py, pz));
            }
        }

        SimResult r = runSim(path, wall, 120.0);
        printResult("goal near wall", r);
        assert(r.goal_reached);
        assert(r.closest_obstacle > DRONE_RADIUS);
    }

    // 6. the drone faces away from the goal with a wall 1.3 m to its side.
    //    It faces a little toward the wall, so the shorter way to turn around
    //    swings its nose past the wall.
    {
        std::vector<geometry_msgs::msg::Vector3> path;
        path.push_back(makePoint(3.2, 7.0, 2.0));
        path.push_back(makePoint(3.2, 3.0, 2.0));

        std::vector<geometry_msgs::msg::Vector3> wall;
        for (double py = -10.0; py <= 10.0; py += 0.3) {
            for (double pz = -5.0; pz <= 15.0; pz += 0.3) {
                wall.push_back(makePoint(4.5, py, pz));
            }
        }

        SimResult r = runSim(path, wall, 120.0, 1.36);
        printResult("turn beside wall", r);
        assert(r.goal_reached);
        assert(r.closest_obstacle > DRONE_RADIUS);
    }

    // 7. and 8. use the walls of the Gazebo test world: wall B lies along
    //    y = 5 and ends 2.5 m before wall A, which leaves a gap
    std::vector<geometry_msgs::msg::Vector3> gap_walls;
    for (double pz = -1.0; pz <= 6.0; pz += 0.25) {
        // wall A, its face toward the drone
        for (double py = -10.0; py <= 10.0; py += 0.25) {
            gap_walls.push_back(makePoint(4.5, py, pz));
        }
        // wall B, both faces and its end
        for (double px = -8.0; px <= 2.0; px += 0.25) {
            gap_walls.push_back(makePoint(px, 4.5, pz));
            gap_walls.push_back(makePoint(px, 5.5, pz));
        }
        for (double py = 4.5; py <= 5.5; py += 0.25) {
            gap_walls.push_back(makePoint(2.0, py, pz));
        }
    }

    // 7. waypoints lead through the middle of the gap and back out
    {
        std::vector<geometry_msgs::msg::Vector3> path;
        path.push_back(makePoint(0.0, 8.0, 2.0));
        path.push_back(makePoint(3.2, 7.0, 2.0));
        path.push_back(makePoint(3.2, 3.0, 2.0));
        path.push_back(makePoint(0.0, 0.0, 2.0));

        SimResult r = runSim(path, gap_walls, 240.0);
        printResult("through gap", r);
        assert(r.goal_reached);
        assert(r.closest_obstacle > CORNER_CLEARANCE);
    }

    // 8. the path runs straight through wall B. The drone side-steps along
    //    the wall until it finds the gap, then goes through it.
    {
        std::vector<geometry_msgs::msg::Vector3> path;
        path.push_back(makePoint(0.0, 0.0, 2.0));
        path.push_back(makePoint(0.0, 8.0, 2.0));

        SimResult r = runSim(path, gap_walls, 240.0);
        printResult("wall with gap", r);
        assert(r.goal_reached);
        assert(r.closest_obstacle > CORNER_CLEARANCE);
    }

    // 9. the drone faces a wall 0.7 m away, the goal is behind it
    {
        std::vector<geometry_msgs::msg::Vector3> path;
        path.push_back(makePoint(3.8, 0.0, 2.0));
        path.push_back(makePoint(0.0, 0.0, 2.0));

        std::vector<geometry_msgs::msg::Vector3> wall;
        for (double py = -10.0; py <= 10.0; py += 0.25) {
            for (double pz = -1.0; pz <= 6.0; pz += 0.25) {
                wall.push_back(makePoint(4.5, py, pz));
            }
        }

        SimResult r = runSim(path, wall, 120.0);
        printResult("boxed in at wall", r);
        assert(r.goal_reached);
    }

    // 10. a wall across the path that meets a second wall on its right, so
    //     the only way around is past its left end
    {
        std::vector<geometry_msgs::msg::Vector3> path;
        path.push_back(makePoint(0.0, 0.0, 2.0));
        path.push_back(makePoint(0.0, 8.0, 2.0));

        std::vector<geometry_msgs::msg::Vector3> walls;
        for (double pz = -1.0; pz <= 6.0; pz += 0.25) {
            // the wall across the path, from x = -2 to the side wall
            for (double px = -2.0; px <= 4.5; px += 0.25) {
                walls.push_back(makePoint(px, 4.5, pz));
                walls.push_back(makePoint(px, 5.5, pz));
            }
            for (double py = 4.5; py <= 5.5; py += 0.25) {
                walls.push_back(makePoint(-2.0, py, pz));
            }
            // the side wall on the right
            for (double py = -10.0; py <= 10.0; py += 0.25) {
                walls.push_back(makePoint(4.5, py, pz));
            }
        }

        SimResult r = runSim(path, walls, 400.0);
        printResult("gap on the left", r);
        assert(r.goal_reached);
        assert(r.closest_obstacle > CORNER_CLEARANCE);
    }

    // 11. a long wall with a doorway: the path runs through the wall, the
    //     doorway is 6 m to the left and a second wall closes off the right
    {
        std::vector<geometry_msgs::msg::Vector3> path;
        path.push_back(makePoint(26.0, 0.0, 2.0));
        path.push_back(makePoint(26.0, 6.0, 2.0));
        path.push_back(makePoint(18.0, 6.0, 2.0));

        std::vector<geometry_msgs::msg::Vector3> walls;
        for (double pz = -1.0; pz <= 6.0; pz += 0.25) {
            // both faces of the door wall, with the doorway from y = -2 to 2
            for (double py = 2.0; py <= 10.0; py += 0.25) {
                walls.push_back(makePoint(22.0, py, pz));
                walls.push_back(makePoint(23.0, py, pz));
                walls.push_back(makePoint(22.0, -py, pz));
                walls.push_back(makePoint(23.0, -py, pz));
            }
            // the two sides of the doorway
            for (double px = 22.0; px <= 23.0; px += 0.25) {
                walls.push_back(makePoint(px, 2.0, pz));
                walls.push_back(makePoint(px, -2.0, pz));
            }
            // the wall that closes off the right
            for (double px = 23.0; px <= 31.0; px += 0.25) {
                walls.push_back(makePoint(px, 9.0, pz));
            }
        }

        SimResult r = runSim(path, walls, 400.0);
        printResult("doorway on the left", r);
        assert(r.goal_reached);
        assert(r.closest_obstacle > CORNER_CLEARANCE);
    }

    // 12. a block in the middle of a corridor 8 m wide. Its right end is
    //     nearer to the path than its left end, so the drone must go around
    //     it on the right, in one move and without stopping in front of it.
    {
        std::vector<geometry_msgs::msg::Vector3> path;
        path.push_back(makePoint(0.0, 0.0, 2.0));
        path.push_back(makePoint(14.0, 0.0, 2.0));

        std::vector<geometry_msgs::msg::Vector3> walls;
        for (double pz = -1.0; pz <= 6.0; pz += 0.25) {
            // the two sides of the corridor
            for (double px = -2.0; px <= 16.0; px += 0.25) {
                walls.push_back(makePoint(px, 4.0, pz));
                walls.push_back(makePoint(px, -4.0, pz));
            }
            // the block, from x = 6 to 7 and from y = -0.5 to 1
            for (double py = -0.5; py <= 1.0; py += 0.25) {
                walls.push_back(makePoint(6.0, py, pz));
                walls.push_back(makePoint(7.0, py, pz));
            }
            for (double px = 6.0; px <= 7.0; px += 0.25) {
                walls.push_back(makePoint(px, -0.5, pz));
                walls.push_back(makePoint(px, 1.0, pz));
            }
        }

        SimResult r = runSim(path, walls, 240.0);
        printResult("block in a corridor", r);
        assert(r.goal_reached);
        assert(!r.was_blocked);
        assert(r.closest_obstacle > CORNER_CLEARANCE);
        // the right of a drone that flies toward +x is the side with negative y
        assert(r.closest_y < -0.5);
        // the straight way is 14 m long
        assert(r.distance_flown < 17.0);
    }

    std::cout << "navigation simulation tests passed\n";
    return 0;
}
