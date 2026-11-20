# Project parts

How the tasks of the original task list show up in the project as it is now, and what changed compared with GitHub.

All twelve tasks are still in the project. Together they are the DWA core that picks a velocity ten times a second. One difference from the list: there is no LIDAR, the depth camera does that job.

## The tasks from the list

| Task | File | What it does now |
|---|---|---|
| Objective function | [objective_function.cpp](src/objective_function.cpp) | Scores every candidate velocity with `G = 0.3 * Heading + 0.6 * Dist + 0.1 * Vel` and returns the best one. It also holds the rule "turn to look before flying". |
| Control periods | [common.hpp](include/reactive_uav_navigation/common.hpp), timer in [reactive_navigator_node.cpp](src/reactive_navigator_node.cpp) | `CONTROL_PERIOD = 0.1` s. A timer runs `control_loop()` every 0.1 s. |
| Send optimal velocity back to drone | `publishVelocity()` and `publishHover()` in [reactive_navigator_node.cpp](src/reactive_navigator_node.cpp) | Publishes the chosen velocity to `/mavros/setpoint_velocity/cmd_vel_unstamped`. The forward speed is turned into world x/y with the yaw. A zero velocity is sent when the drone is blocked, at the goal, or has no path. |
| Get current position, velocity, roll, yaw, pitch | [drone_state.cpp](src/drone_state.cpp) | Reads `/mavros/local_position/odom`. Every other part asks it for the position, yaw, orientation and current velocity. |
| Maximum velocities Vs | [vss_maximum.cpp](src/vss_maximum.cpp) | The outer speed limits: forward 0 to 0.3 m/s, yaw up to 45 deg/s. The 0.3 m/s cap (`vss.vx_max_cap`) is why the drone flies slowly. |
| Dynamic window Vd | [vss_dynamic_window.cpp](src/vss_dynamic_window.cpp) | Narrows the limits to the speeds the drone can reach from its current speed in one step, given its maximum accelerations. |
| Admissible velocities Va | [vss_admissible.cpp](src/vss_admissible.cpp) | Throws out every candidate that could not brake before the nearest obstacle in its direction. This is the hard safety check, it keeps the drone about 0.8 m from walls. |
| Heading function | [heading_function.cpp](src/heading_function.cpp) | Scores how well a candidate points the drone at the target. The target comes from `DetourPlanner`: the waypoint, a point past a block, or a side step. |
| LIDAR, cast ray | [depth_ray.cpp](src/depth_ray.cpp), [ray_cast.cpp](src/ray_cast.cpp) | `depth_ray` turns each depth image into obstacle points up to 4 m ahead. `castRay` sends a ray through those points and reports the first one within 0.70 m of the ray. |
| Distance function | [distance_function.cpp](src/distance_function.cpp) | Casts a fan of 9 rays (45 deg to each side) around a direction and returns the shortest free distance. Va uses it for braking, and it is the `Dist` score, the heaviest of the three. A direction the camera cannot see counts as blocked. |
| Velocity function | [velocity_function.cpp](src/velocity_function.cpp) | Scores `forward speed / top speed`, so faster is better, but only when the drone roughly faces the target. |
| Subscribe/handle global path input | `path_callback()` in [reactive_navigator_node.cpp](src/reactive_navigator_node.cpp) | Listens on `planned_path`. A new path replaces the old one and resets the waypoint index, the navigation state and any detour. The control loop moves on to the next waypoint when the drone is within 0.3 m of the current one. |

## What changed compared with GitHub

Compared on 2026-11-16 with the branch `candidate_v2` of `Kennetha13/reactive_uav_navigation` (last commit `66d2013`, "added extra logging info"). This folder is on the same commit, so every change below is local and not committed. 27 files were changed and 14 files are new.

### Changes to the tasks from the list

| Task | On GitHub | Now |
|---|---|---|
| LIDAR, cast ray | `castRay` read the depth image directly, one ray at a time, always from where the camera is now. | Each image is turned into up to 300 obstacle points in the world frame (`makeWorldPoints`). The new `ray_cast.cpp` casts rays against those points from any position. The camera's field of view and the age of the image can be asked for. |
| Distance function | Owned the camera. The position given to it was ignored, so a ray from a predicted position was really cast from the current one. | Gets the obstacle points handed in and casts from the given position. A direction outside the camera's view counts as blocked. An obstacle behind the waypoint is ignored. An obstacle beside the drone, closer than 1 m, lowers the score. |
| Objective function | The node picked the best candidate itself. Each candidate was predicted 0.1 s ahead, so all of them ended at almost the same place. | `selectBest()` is part of the objective function. Candidates are predicted 1 s ahead. If the target is behind the drone or out of view, the drone first turns on the spot toward it. |
| Velocity function | Read PX4's top speed (default 12 m/s) by itself, so the score of a 0.3 m/s drone was close to 0. | Gets the top speed from Vs, so the score runs from 0 to 1. It no longer needs ROS. |
| Admissible velocities Va | No extra clearance: the braking check used the distance to the obstacle as it was. | The drone's radius (0.3 m) and a safety margin (`vss.safety_margin`, 0.5 m) are taken off first, so the drone stops about 0.8 m from a wall. |
| Get current position, velocity, roll, yaw, pitch | Position, velocity and the three angles. | Also gives the orientation as a quaternion, used to turn the depth points into the world frame. |
| Send optimal velocity back to drone | Hovered whenever the state was `BLOCKED`. | `publishHover()` is a function of its own. While `BLOCKED` with no safe velocity the drone may still turn on the spot, so it can turn away from a wall. |
| Subscribe/handle global path input | A new path was ignored while the drone was flying one. | A new path always replaces the current one. The last waypoint is the goal. |
| Control periods, Maximum velocities Vs, Dynamic window Vd, Heading function | | Not changed. |

### Other changes

| Part | What changed |
|---|---|
| Node ([reactive_navigator_node.cpp](src/reactive_navigator_node.cpp)) | Uses `ObstacleMemory` and `DetourPlanner`. Hovers when there is no recent depth image. Publishes the state on the topic `navigation_state`. New ROS parameters: `waypoint_tolerance`, `search_range`, `drone_radius`, `use_depth_camera`, `depth_timeout`, `obstacle_memory_time`, `stall_timeout`. Prints the time, distance flown and number of blocks when the path is finished. |
| Navigation state ([navigation_state.cpp](src/navigation_state.cpp)) | The goal only counts as reached while heading for the last waypoint, so a loop path is flown all the way. Going around an obstacle is reported as `AVOIDING_OBSTACLE` and is not counted as a stall. Hovering with no safe velocity is not counted as a stall either. |
| PX4 parameters ([px4_param_reader.cpp](src/px4_param_reader.cpp)) | Reads them through `/mavros/param/get_parameters` instead of `/mavros/param/get`. |
| Velocity search ([vss_collector.cpp](src/vss_collector.cpp)) | Reads `drone_radius` and `vss.safety_margin` for Va. |
| Build ([CMakeLists.txt](CMakeLists.txt), [package.xml](package.xml)) | C++17, optimized build by default, the `pcl_conversions` dependency is gone, the new files and the new test are added. |
| [README.md](README.md) | New parts: Test, Automated Gazebo tests, Topics and parameters, how the drone gets around an obstacle, how the camera is used. |
| [waypoints.yml](waypoints.yml) | Is now a valid path message that can be published with `ros2 topic pub`. |

### New files

| File | What it is |
|---|---|
| [detour_planner.hpp](include/reactive_uav_navigation/detour_planner.hpp), [detour_planner.cpp](src/detour_planner.cpp) | `DetourPlanner`, finds a way around an obstacle. |
| [obstacle_memory.hpp](include/reactive_uav_navigation/obstacle_memory.hpp), [obstacle_memory.cpp](src/obstacle_memory.cpp) | `ObstacleMemory`, remembers obstacle points that left the camera's view. |
| [ray_cast.hpp](include/reactive_uav_navigation/ray_cast.hpp), [ray_cast.cpp](src/ray_cast.cpp) | `RayCast`, casts a ray against the obstacle points. |
| [test_navigation_sim.cpp](test/test_navigation_sim.cpp) | 12 simulated flights with the real planner, without ROS, PX4 or Gazebo. |
| [test/sim](test/sim) | The automated Gazebo tests: `run_sim_tests.sh`, `sim_test_runner.py` and the 9 maps in `scenarios.yml`. |
| [test/fake](test/fake), [test/fake_drone_state](test/fake_drone_state) | Stand-ins for the ROS message types and for `DroneState`, used by the fast tests. |
| [.gitattributes](.gitattributes) | Keeps Linux line endings for the scripts. |
| PROJECT_PARTS.md | This file. |

## Not included

Parts of the project that are not on the task list.

| Part | File | What it does | On GitHub |
|---|---|---|---|
| `DetourPlanner` | [detour_planner.cpp](src/detour_planner.cpp) | Chooses the target when an obstacle is in the way: a point past the end of a block, or side steps along a wall. | no, new |
| `ObstacleMemory` | [obstacle_memory.cpp](src/obstacle_memory.cpp) | Keeps obstacle points for 15 s after they leave the camera's view. | no, new |
| `NavigationStateMachine` | [navigation_state.cpp](src/navigation_state.cpp) | Reports `WAITING_FOR_PATH`, `FOLLOWING_PATH`, `AVOIDING_OBSTACLE`, `BLOCKED` or `GOAL_REACHED`. | yes, changed |
| Velocity search collector | [vss_collector.cpp](src/vss_collector.cpp) | Combines Vs, Vd and Va into the final list of candidate velocities, Vr. | yes, changed |
| PX4 parameter reader | [px4_param_reader.cpp](src/px4_param_reader.cpp) | Reads PX4's own speed and acceleration limits for Vs and Vd. | yes, changed |
| Fast tests | [test](test) | Unit tests and simulated flights that run without ROS. | the unit tests yes, the simulated flights are new |
| Gazebo tests | [test/sim](test/sim) | Fly the drone through 9 maps in Gazebo and check each flight. One has a block that appears during the flight, one is a dead end that must be reported as `BLOCKED`. | no, new |
