## Prerequisites

- ROS 2 with `rclcpp`, `nav_msgs`, `geometry_msgs`, `sensor_msgs`, `std_msgs`, `tf2`
- MAVROS 
- PX4 Autopilot with SITL and Gazebo
- `ros_gz_bridge` for Gazebo Harmonic, to bring the depth camera into ROS 2: `sudo apt install ros-humble-ros-gzharmonic-bridge` (the plain `ros-humble-ros-gz-bridge` is built for an older Gazebo and receives nothing from PX4's Gazebo)
- tmux (only for the automated Gazebo tests)
- QGroundControl (optional - to view drone moving from p2p)

## Build

```bash
colcon build --packages-select reactive_uav_navigation
source install/setup.bash
```

## Test

Run this from your workspace (for example `~/ros2_ws`) after the build.

### Automated Gazebo tests

Flies the drone through 9 maps (`test1` to `test9`, listed in `test/sim/scenarios.yml`) and checks each flight. The script starts PX4, Gazebo, MAVROS, the camera bridge and the navigator node by itself and stops them again at the end, so do not start any of them yourself. It takes about 23 minutes.

```bash
SIM_GUI=1 bash src/reactive_uav_navigation/test/sim/run_sim_tests.sh ~/sim_out/run_1
```

## Run (simulation)

Use a separate terminal for each step. Remember to `source ~/ros2_ws/install/setup.bash` in terminals that use this package.

### 1. Start PX4 SITL with Gazebo

Run from your PX4-Autopilot directory (adjust the target to match your setup):

```bash
make px4_sitl gz_x500_depth
```

`gz_x500_depth` is the drone with the depth camera the node uses to see obstacles. With the plain `gz_x500`, start the node in step 3 with `--ros-args -p use_depth_camera:=false` (it then sees no obstacles).

Then start the camera bridge so the depth image reaches ROS 2:

```bash
ros2 run ros_gz_bridge parameter_bridge /depth_camera@sensor_msgs/msg/Image@gz.msgs.Image /camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo
```

### 2. Start MAVROS

```bash
ros2 launch mavros px4.launch fcu_url:="udp://127.0.0.1:14540@" gcs_url:="udp://@127.0.0.1:14550"
```

### 3. Start the navigator node

Start it before switching to OFFBOARD. PX4 only accepts OFFBOARD if setpoints are already streaming. The node publishes zero velocity until it receives a path, and it hovers whenever no recent depth image arrives.

```bash
ros2 run reactive_uav_navigation reactive_navigator_node
```

### 4. Arm and take off

In the PX4 console (`pxh>`):

```
commander arm
commander takeoff
```

If PX4 refuses to arm because no ground station is connected, run `param set NAV_DLL_ACT 0` in the same console first.

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

You can use the bag that plays the global path provided by Professor Ramirez, or for simplicity and testing purposes, you can run this command to publish a custom list of poses to planned_path. A new path replaces the one the drone is flying.

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

The example path in `waypoints.yml` can also be sent as it is, from the package folder:

```bash
ros2 topic pub --once /planned_path nav_msgs/msg/Path "$(cat waypoints.yml)"
```

The node logs `Received new path...`, then `Reached waypoint N` and each change of navigation state (`FOLLOWING_PATH`, `AVOIDING_OBSTACLE`, `BLOCKED`, `GOAL_REACHED`) as it progresses. When the path is complete it hovers and logs the time taken, the distance flown and how often it was blocked. You should be able to see the drone moving in Gazebo and its path in QGroundControl as well.

To watch the navigation state:

```bash
ros2 topic echo /navigation_state
```
