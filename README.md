## prerequisites

- ROS 2 with `rclcpp`, `nav_msgs`, `geometry_msgs`, `sensor_msgs`, `std_msgs`, `tf2`
- MAVROS 
- PX4 Autopilot with SITL and Gazebo
- `ros_gz_bridge` for Gazebo Harmonic, to bring the depth camera into ROS 2: `sudo apt install ros-humble-ros-gzharmonic-bridge` (the plain `ros-humble-ros-gz-bridge` is built for an older Gazebo and receives nothing from PX4's Gazebo)
- tmux (only for the automated Gazebo tests)
- QGroundControl (optional - to view drone moving from p2p)

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive ~/PX4-Autopilot
bash ~/PX4-Autopilot/Tools/setup/ubuntu.sh
sudo apt install ros-humble-mavros ros-humble-mavros-extras ros-humble-ros-gzharmonic-bridge tmux
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```

## build

```bash
colcon build --packages-select reactive_uav_navigation
source install/setup.bash
```

## test

run this from your workspace (for example `~/ros2_ws`) after the build.

### automated gazebo tests

flies the drone through 9 maps (`test1` to `test9`, listed in `test/sim/scenarios.yml`) and checks each flight. the script starts PX4, Gazebo, MAVROS, the camera bridge and the navigator node by itself and stops them again at the end, so do not start any of them yourself. ~23mins of runtime

```bash
SIM_GUI=1 bash src/reactive_uav_navigation/test/sim/run_sim_tests.sh ~/sim_out/run_1
```

## run (simulation)

use a separate terminal for each step. remember to `source ~/ros2_ws/install/setup.bash` in terminals that use this package.

### 1. start PX4 SITL with Gazebo

run from your PX4-Autopilot directory (adjust the target to match your setup):

```bash
make px4_sitl gz_x500_depth
```

`gz_x500_depth` is the drone with the depth camera the node uses to see obstacles. with the plain `gz_x500`, start the node in step 3 with `--ros-args -p use_depth_camera:=false` (it then sees no obstacles).

then start the camera bridge so the depth image reaches ROS 2:

```bash
ros2 run ros_gz_bridge parameter_bridge /depth_camera@sensor_msgs/msg/Image@gz.msgs.Image /camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo
```

### 2. start MAVROS

```bash
ros2 launch mavros px4.launch fcu_url:="udp://127.0.0.1:14540@" gcs_url:="udp://@127.0.0.1:14550"
```

### 3. start the navigator node

start it before switching to OFFBOARD. PX4 only accepts OFFBOARD if setpoints are already streaming. the node publishes zero velocity until it receives a path, and it hovers whenever no recent depth image arrives.

```bash
ros2 run reactive_uav_navigation reactive_navigator_node
```

### 4. arm and take off

In the PX4 console (`pxh>`):

```
commander arm
commander takeoff
```

if PX4 refuses to arm because no ground station is connected, run `param set NAV_DLL_ACT 0` in the same console first.

you can also use QGroundControl to takeoff.

wait until the drone is hovering.

### 5. switch to OFFBOARD

```bash
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'OFFBOARD'}"
```

verify:

```bash
ros2 topic echo /mavros/state --once   # mode: OFFBOARD, armed: true
```

if you're running into difficulties setting it to OFFBOARD, try running the following then repeat the OFFBOARD service call afterward. you want the drone to be in OFFBOARD mode before continuing.

```bash
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
```

### 6. publish a path

you can run this command to publish a custom list of poses to planned_path. a new path replaces the one the drone is flying.

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

the example path in `waypoints.yml` can also be sent as it is, from the package folder:

```bash
ros2 topic pub --once /planned_path nav_msgs/msg/Path "$(cat waypoints.yml)"
```

the node logs `Received new path...`, then `Reached waypoint N` and each change of navigation state (`FOLLOWING_PATH`, `AVOIDING_OBSTACLE`, `BLOCKED`, `GOAL_REACHED`) as it progresses. when the path is complete it hovers and logs the time taken, the distance flown and how often it was blocked. you should be able to see the drone moving in Gazebo and its path in QGroundControl as well.

to watch the navigation state:

```bash
ros2 topic echo /navigation_state
```

## record and view a flight

a bag saves the planned path, the drone position and the navigation state of a flight, and rviz2 shows them afterwards. run these from your workspace.

### 1. record a bag

start it in its own terminal before the flight and stop it with Ctrl+C when the flight is done. during the automated Gazebo tests, run `export ROS_DOMAIN_ID=77` in that terminal first, the tests use that domain.

```bash
ros2 bag record -o ~/bags/flight_1 /planned_path /mavros/local_position/odom /navigation_state
```

### 2. open rviz2

```bash
rviz2 -d src/reactive_uav_navigation/rviz/path_view.rviz
```

### 3. play the bag

play it in another terminal after rviz2 is open. the planned path shows in green and the flown path in red.

```bash
ros2 bag play ~/bags/flight_1
```

the bags of the current 9 test flights are in `bags.zip` on the [Releases](https://github.com/Kennetha13/reactive_uav_navigation/releases) page.
