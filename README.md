# Depth Camera and PX4 Simulation

This guide explains how to build and run the depth-camera portion of
reactive_uav_navigation directly from this project directory:

~~~
~/reactive_uav_navigation
~~~

The camera processor runs inside the existing reactive_navigator node. It is
not a separate ROS 2 node.

## What the camera code does

DepthRayProcessor:

- subscribes to the depth image;
- subscribes to the camera calibration;
- projects a requested camera direction onto the image;
- checks a small pixel region around that projection;
- returns the nearest valid obstacle distance.

The default camera direction is:

~~~
x = 0.0
y = 0.0
z = 1.0
~~~

In the camera optical frame:

~~~
+X = right
+Y = down
+Z = forward
~~~

DroneState separately subscribes to MAVROS odometry, and ReactiveNavigator
subscribes to the global path.

## Prerequisites

- Ubuntu 22.04
- ROS 2 Humble
- PX4 Autopilot with Gazebo SITL
- MAVROS
- ros_gz_bridge
- The recorded path bag

Install the ROS build tools and Gazebo bridge if needed:

~~~
sudo apt update
sudo apt install \
  python3-colcon-common-extensions \
  ros-humble-ros-gz-bridge
~~~

## Build from this project

Open a terminal and run:

~~~
cd ~/reactive_uav_navigation

source /opt/ros/humble/setup.bash

colcon build \
  --packages-select reactive_uav_navigation \
  --symlink-install

source ~/reactive_uav_navigation/install/setup.bash
~~~

Run the build from the project root, not from a separate ros2_ws directory.

## Start the simulation

Use a separate terminal for each process.

### 1. Start PX4 and Gazebo

~~~
cd ~/PX4-Autopilot

make px4_sitl gz_x500_depth
~~~

The gz_x500_depth model is required because it provides the simulated depth
camera. The ordinary gz_x500 model does not provide the same camera input.

### 2. Start MAVROS

~~~
source /opt/ros/humble/setup.bash

ros2 launch mavros px4.launch \
  fcu_url:="udp://127.0.0.1:14540@" \
  gcs_url:="udp://@127.0.0.1:14550"
~~~

The gcs_url argument helps PX4 recognize a ground-control connection during
SITL.

### 3. Start the Gazebo camera bridge

~~~
source /opt/ros/humble/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
  /depth_camera@sensor_msgs/msg/Image@gz.msgs.Image \
  /camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo
~~~

This makes the Gazebo depth image and camera-info messages available as ROS 2
topics.

### 4. Start the navigation node

~~~
source /opt/ros/humble/setup.bash
source ~/reactive_uav_navigation/install/setup.bash

ros2 run reactive_uav_navigation reactive_navigator_node \
  --ros-args \
  -p ray_direction_x:=0.0 \
  -p ray_direction_y:=0.0 \
  -p ray_direction_z:=1.0
~~~

For a camera-only test, do not arm or switch to OFFBOARD. The node can process
camera data while the simulated drone remains on the ground.

## Verify the camera

Check that Gazebo is publishing depth images:

~~~
ros2 topic hz /depth_camera
~~~

Check the image type and encoding:

~~~
ros2 topic type /depth_camera
ros2 topic echo /depth_camera --once --field encoding
~~~

Expected output:

~~~
sensor_msgs/msg/Image
32FC1
~~~

Check the calibration:

~~~
ros2 topic type /camera_info
ros2 topic echo /camera_info --once
~~~

Expected type:

~~~
sensor_msgs/msg/CameraInfo
~~~

Check the subscriptions created by the complete navigation node:

~~~
ros2 node info /reactive_navigator
~~~

The output should include:

~~~
/camera_info
/depth_camera
/mavros/local_position/odom
/planned_path
~~~

The ROS graph lists all subscriptions under reactive_navigator because the
classes use the same underlying ROS 2 node. Internally, the camera topics are
managed by DepthRayProcessor.

## Use the recorded path

The path bag can be stored anywhere. For example, copy it out of the old
workspace so the test no longer depends on ros2_ws:

~~~
cp ~/ros2_ws/path_capture_20260810_095030_0.db3 \
  ~/path_capture_20260810_095030_0.db3
~~~

Inspect the bag:

~~~
ros2 bag info ~/path_capture_20260810_095030_0.db3
~~~

The expected path topic is:

~~~
/planned_path
~~~

Replay only the path:

~~~
ros2 bag play ~/path_capture_20260810_095030_0.db3 \
  --topics /planned_path \
  --rate 5.0
~~~

The navigator should print:

~~~
Received new path with frame_id: ...
~~~

The path is needed because the current control loop only performs the depth
calculation when it has valid odometry and an active path.

## Monitor the camera distance

In another terminal:

~~~
source /opt/ros/humble/setup.bash

ros2 topic echo /obstacle_distance_forward
~~~

Place a cylinder directly in front of the simulated camera. The output should
be near the cylinder's distance, for example:

~~~
data: 0.297
~~~

The default maximum distance is 15 meters. A value of exactly 15 means that
the ray did not find a valid depth measurement and the node published its
fallback value.

## Test a different camera direction

The ray direction can be changed when starting the node. For example, this
checks forward and to the right:

~~~
source ~/reactive_uav_navigation/install/setup.bash

ros2 run reactive_uav_navigation reactive_navigator_node \
  --ros-args \
  -p ray_direction_x:=0.5 \
  -p ray_direction_y:=0.0 \
  -p ray_direction_z:=1.0
~~~

Positive x points right. Positive y points down. These values are directions,
not pixel coordinates.

The maximum accepted range can be changed with:

~~~
-p depth_ray_max_distance:=15.0
~~~

## Optional flight test

Only use this after the camera-only test works.

Keep the navigation node running so it is already publishing setpoints. In
the PX4 console:

~~~
commander arm
commander takeoff
~~~

Wait for the simulated vehicle to hover, then replay the path bag.

To land:

~~~
commander land
~~~

Do not use SITL arming commands on the real drone.

## Real ZED 2i

On the Jetson, start the ZED ROS 2 wrapper:

~~~
source /opt/ros/humble/setup.bash

ros2 launch zed_wrapper zed_camera.launch.py \
  camera_model:=zed2i
~~~

Find the actual topics:

~~~
ros2 topic list | grep zed
~~~

Start the navigator with the matching depth and camera-info topics:

~~~
source ~/reactive_uav_navigation/install/setup.bash

ros2 run reactive_uav_navigation reactive_navigator_node \
  --ros-args \
  -p depth_topic:=/zed/zed_node/depth/depth_registered \
  -p camera_info_topic:=/zed/zed_node/depth/camera_info
~~~

Topic names can vary by ZED wrapper version and namespace. Always use the
camera-info topic associated with the selected depth image.
