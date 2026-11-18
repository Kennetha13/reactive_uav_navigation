## Prerequisites

- ROS 2 with `rclcpp`, `nav_msgs`, `geometry_msgs`, `sensor_msgs`, `std_msgs`, `tf2`
- `ros_gz_bridge` built for Gazebo Harmonic (to bring the Gazebo depth camera into ROS 2): `sudo apt install ros-humble-ros-gzharmonic-bridge`. The plain `ros-humble-ros-gz-bridge` package is built for the older Gazebo Fortress and receives nothing from PX4's Gazebo.
- MAVROS 
- PX4 Autopilot with SITL and Gazebo
- QGroundControl (optional - to view drone moving from p2p)

## Build

```bash
colcon build --packages-select reactive_uav_navigation
source install/setup.bash
```

## Test

The unit tests and the simulated flights (open path, obstacle on the path, blocked path, loop path) run without ROS, PX4 or Gazebo:

```bash
colcon test --packages-select reactive_uav_navigation --ctest-args -R "test_"
colcon test-result --verbose
```

## Automated Gazebo tests

`test/sim/run_sim_tests.sh` flies the drone in Gazebo with the real depth camera and checks the result of each flight. It starts PX4, Gazebo, MAVROS, the camera bridge and the navigator node by itself, and stops them again at the end.

```bash
source install/setup.bash
bash src/reactive_uav_navigation/test/sim/run_sim_tests.sh
```

It needs PX4-Autopilot (in `~/PX4-Autopilot`, or set `PX4_DIR`), MAVROS, `ros-humble-ros-gzharmonic-bridge` and `tmux`. Nothing else may be using PX4 at the same time. Set `SIM_GUI=1` in front of the command to watch the flights in the Gazebo window. The view stays 25 m above the drone and looks straight down with north up, so the drone is always in the middle of the window.

The scenarios are listed in `test/sim/scenarios.yml`. They are flown in PX4's `walls` world. The four walls that world comes with are taken out at the beginning, because every scenario brings a map of its own:

- The start (0, 0), where the drone spawns, is in the middle of a room of 8 m by 8 m.
- A corridor, 4 m wide (8 m in `test6` and `test8`), leads out of the room and around one or more turns to the goal.
- The path gives the corner points of the corridor.

Every scenario is flown by itself:

1. The drone is at the start point, with nothing near it.
2. The walls of the scenario are put into the world. The nearest one is 4 m from the drone.
3. The path is published and the drone flies it.
4. The walls are taken away again and the drone goes back to the start. PX4 flies this part itself with its return mode, which is much faster than the planner.

So no scenario depends on the one before it, and the drone never starts on top of a wall or close to one. The test refuses to run a scenario that has a wall within 3 m of the start (`min_start_clearance`).

| Scenario | The corridor leads | Turns | Must end in |
|---|---|---|---|
| `test1` | east, north | 1 | `GOAL_REACHED` |
| `test2` | south, east, north | 2 | `GOAL_REACHED` |
| `test3` | west, north, west | 2 | `GOAL_REACHED` |
| `test4` | east, north, east, then a long way south | 3 | `GOAL_REACHED` |
| `test5` | north, east, south, east, north | 4 | `GOAL_REACHED` |
| `test6` | as `test5`, but the corridor is 8 m wide and two blocks stand in the middle of it, the drone must go around each one | 4 | `GOAL_REACHED` |
| `test7` | east, south-east at an angle, east | 2 | `GOAL_REACHED` |
| `test8` | straight east in a corridor 8 m wide, a block is put on the path during the flight, 3.5 m in front of the drone | 0 | `GOAL_REACHED` |
| `test9` | east into a corridor that is closed at its end, the goal lies behind the end wall | 0 | `BLOCKED` |

A scenario passes when the flight ends in the expected state, the drone never comes closer to a wall than its own radius, and, if the goal should be reached, it ends within 0.6 m of it.

To try another map, change the `walls` and the `path` of a scenario, or add a scenario to the list. A block in the way is just a short wall. A wall can be given an `angle` in degrees, it is then turned about its center by that much. A wall with `appears_at` is not there at the start: it is put into the world during the flight, when the drone comes that close to it (m).

Results are written to `/tmp/reactive_uav_sim_tests` (or the folder given as the first argument):

- `results.csv`: one row per scenario with the result, final state, time, path length, distance flown, closest wall distance, goal error and the time spent blocked and avoiding
- `track_<scenario>.csv`: the position and navigation state five times a second
- `px4.log`, `mavros.log`, `bridge.log`, `node.log`: output of each program

The script exits with 0 when every scenario passed, 1 when one failed and 2 when the test could not be carried out: the simulation did not start, a wall could not be put in or taken out, or the drone did not get back to the start.

Known limit: these tests give the drone a path that leads through the corridor. When a path runs straight through a wall instead, the drone searches for a way around by itself. It finds an opening that is a few meters to the side of its path. An opening further along a wall, about 6 m or more, it finds less reliably. It can stop in the entrance of that opening and report `BLOCKED`, because it turns to face the waypoint as soon as the camera no longer sees toward it, and inside an entrance that points it at the wall beside the opening.

## Topics and parameters

| Name | Type | Direction | Meaning |
|---|---|---|---|
| `planned_path` | `nav_msgs/msg/Path` | in | global path to follow; a new path replaces the current one |
| `/depth_camera` | `sensor_msgs/msg/Image` | in | depth image (`32FC1` in meters or `16UC1` in millimeters) |
| `/camera_info` | `sensor_msgs/msg/CameraInfo` | in | calibration of the depth camera |
| `/mavros/local_position/odom` | `nav_msgs/msg/Odometry` | in | drone pose |
| `/mavros/setpoint_velocity/cmd_vel_unstamped` | `geometry_msgs/msg/Twist` | out | velocity command |
| `navigation_state` | `std_msgs/msg/String` | out | `WAITING_FOR_PATH`, `FOLLOWING_PATH`, `AVOIDING_OBSTACLE`, `BLOCKED` or `GOAL_REACHED` |

| Parameter | Default | Meaning |
|---|---|---|
| `use_depth_camera` | true | set to false to fly without a camera; nothing is then seen as an obstacle |
| `depth_topic`, `camera_info_topic` | `/depth_camera`, `/camera_info` | camera topics |
| `waypoint_tolerance` | 0.3 | how close (m) counts as reaching a waypoint or the goal |
| `search_range` | 3.0 | how far (m) the planner looks for obstacles |
| `drone_radius` | 0.3 | radius of the drone (m) |
| `obstacle_memory_time` | 15.0 | how long (s) an obstacle is remembered after it leaves the camera's view |
| `depth_timeout` | 1.0 | a depth image older than this (s) is not trusted and the drone hovers |
| `vss.safety_margin` | 0.5 | extra clearance (m) kept in the braking check, the drone stops about 0.8 m from a wall |
| `stall_timeout` | 10.0 | no progress for this long (s) is reported as `BLOCKED` |
| `vss.vx_step`, `vss.vz_step`, `vss.wz_step_deg` | 0.1, 0.1, 5.0 | spacing of the candidate velocities |

How the drone gets around an obstacle:

- The velocity planner (DWA) steers around things that are a little off the straight line.
- When an obstacle lies on the straight line to the waypoint and the camera sees where it ends, `DetourPlanner` aims the drone at a point 1.3 m past the nearer end. The drone goes around the obstacle without stopping in front of it, and heads for the waypoint again once the obstacle is 1.3 m clear of the straight line to it. An end only counts if the camera sees the space past it and nothing lies on the way there.
- When no end is in sight (a long wall) and the drone gets no closer for 4 s, `DetourPlanner` gives it a temporary target 2.5 m to the side of the spot where it got stuck and 1 m back from the obstacle, then lets it try the waypoint again. The first side is the one where the obstacle seemed to end sooner, or the right if nothing is known. If that does not help, the next target is 5 m to the side of that spot, then 7.5 m and 10 m. After that, or when the way on that side is closed, it tries the same four distances on the other side.
- While it goes around or searches the state is `AVOIDING_OBSTACLE`. If neither side leads anywhere it gives up and the state becomes `BLOCKED`.
- Before it flies toward a point the camera cannot see, the drone turns on the spot to look at it.
- A velocity that would bring the drone within 1 m of a known obstacle point scores lower, so it keeps visible space to walls and corners.

How the camera is used:

- Each depth image is turned into obstacle points in the world frame, so the rays of the distance function and the obstacles use the same frame.
- The drone only moves in directions inside the camera's field of view. A direction the camera cannot see is treated as blocked, not clear, so the drone turns to look first. Climbing or descending while moving forward is limited to slopes the camera can see along.
- Moving straight up or down is the one exception, so the drone can correct its altitude in place. Nothing is known above or below the drone.
- An obstacle further away than the current waypoint is ignored in the clearance score, so a goal just in front of a wall can still be reached. The braking check still applies.
- With `use_depth_camera` true and no recent image, the drone hovers and reports `BLOCKED`.
- The camera is taken to look straight ahead from the center of the drone.

## Run (simulation)

Use a separate terminal for each step. Remember to `source ~/ros2_ws/install/setup.bash` in terminals that use this package.

### 1. Start PX4 SITL with Gazebo

Run from your PX4-Autopilot directory (adjust the target to match your setup):

```bash
make px4_sitl gz_x500_depth
```

`gz_x500_depth` has the depth camera. With the plain `gz_x500` model, start the node in step 3 with `--ros-args -p use_depth_camera:=false`.

Then start the camera bridge so the depth image and the calibration reach ROS 2:

```bash
ros2 run ros_gz_bridge parameter_bridge   /depth_camera@sensor_msgs/msg/Image@gz.msgs.Image   /camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo
```

### 2. Start MAVROS

```bash
ros2 launch mavros px4.launch fcu_url:="udp://127.0.0.1:14540@" gcs_url:="udp://@127.0.0.1:14550"
```

### 3. Start the navigator node

Start it before switching to OFFBOARD. PX4 only accepts OFFBOARD if setpoints are already streaming. The node publishes zero velocity until it receives a path.

```bash
ros2 run reactive_uav_navigation reactive_navigator_node
```

### 4. Arm and take off

In the PX4 console (`pxh>`):

```
commander arm
commander takeoff
```

You can also use QGroundControl to takeoff.

Wait until the drone is hovering.

### 5. Switch to OFFBOARD

```bash
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'OFFBOARD'}"
```

Verify:

```bash
ros2 topic echo /mavros/state --once   # mode: OFFBOARD, armed: true
```

If you're running into difficulties setting it to OFFBOARD, try running the following then repeat the OFFBOARD service call afterward. You want the drone to be in OFFBOARD mode before continuing.

```bash
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
```

### 6. Publish a path

You can use the bag that plays the global path provided by Professor Ramirez, or for simplicity and testing purposes, you can run this command to publish a custom list of poses to planned_path.

```bash
ros2 topic pub --once /planned_path nav_msgs/msg/Path "{
  header: {frame_id: 'map'},
  poses: [
    {pose: {position: {x: 0.0, y: 0.0, z: 2.0}}},
    {pose: {position: {x: 5.0, y: 0.0, z: 2.0}}},
    {pose: {position: {x: 5.0, y: 5.0, z: 2.0}}}
  ]
}"
```

The node logs `Received new path...`, then `Reached waypoint N` and each change of navigation state as it progresses. When the path is complete it hovers and logs the time taken, the distance flown and how often it was blocked. You should be able to see the drone moving in Gazebo and its path in QGroundControl as well.

To watch the navigation state:

```bash
ros2 topic echo /navigation_state
```
