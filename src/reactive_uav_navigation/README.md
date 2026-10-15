## Prerequisites

- ROS 2 with `rclcpp`, `nav_msgs`, `geometry_msgs`, `tf2`
- MAVROS 
- PX4 Autopilot with SITL and Gazebo
- QGroundControl (optional - to view drone moving from p2p)

## Build

```bash
colcon build --packages-select reactive_uav_navigation
source install/setup.bash
```

## Run (simulation)

Use a separate terminal for each step. Remember to `source ~/ros2_ws/install/setup.bash` in terminals that use this package.

### 1. Start PX4 SITL with Gazebo

Run from your PX4-Autopilot directory (adjust the target to match your setup):

```bash
make px4_sitl gz_x500
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

The node logs `Received new path...`, then `Reached waypoint N` as it progresses, and stops when the path is complete. You should be able to see the drone moving in Gazebo and its path in QGroundControl as well.
