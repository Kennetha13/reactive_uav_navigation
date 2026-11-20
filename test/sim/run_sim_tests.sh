#!/bin/bash
# Automated Gazebo simulation tests.
#
# Starts PX4 SITL with the depth camera drone in PX4's "walls" world, MAVROS,
# the camera bridge and the navigator node, flies every scenario of
# scenarios.yml and reports pass or fail. Every scenario brings its own walls,
# the ones the world comes with are taken out. Everything it started is
# stopped again at the end.
#
# usage:  bash test/sim/run_sim_tests.sh [output_dir] [scenario_file]
#
# A scenario file of your own can be given in place of scenarios.yml.
#
# Before running, build the package and source its workspace
# (source install/setup.bash).
#
# needs:  PX4-Autopilot (set PX4_DIR if it is not in ~/PX4-Autopilot), MAVROS,
#         ros_gz_bridge for Gazebo Harmonic (ros-humble-ros-gzharmonic-bridge), tmux
# set SIM_GUI=1 to watch the flight in the Gazebo window, seen from above

HERE="$(cd "$(dirname "$0")" && pwd)"
OUTPUT_DIR="${1:-/tmp/reactive_uav_sim_tests}"
SCENARIO_FILE="${2:-$HERE/scenarios.yml}"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
mkdir -p "$OUTPUT_DIR"

# the tests use their own ROS domain so other ROS programs do not interfere
export ROS_DOMAIN_ID=77
export GZ_IP=127.0.0.1

# check that everything needed is there
if ! command -v tmux > /dev/null; then
    echo "tmux is not installed (sudo apt install tmux)"
    exit 2
fi
if [ ! -d "$PX4_DIR" ]; then
    echo "PX4-Autopilot not found in $PX4_DIR (set PX4_DIR)"
    exit 2
fi
PACKAGE_PREFIX="$(ros2 pkg prefix reactive_uav_navigation 2> /dev/null)"
if [ -z "$PACKAGE_PREFIX" ]; then
    echo "reactive_uav_navigation not found, build it and source install/setup.bash first"
    exit 2
fi
if [ ! -f "$SCENARIO_FILE" ]; then
    echo "scenario file $SCENARIO_FILE not found"
    exit 2
fi
if pgrep -x px4 > /dev/null; then
    echo "PX4 is already running, stop it first"
    exit 2
fi
WORKSPACE_SETUP="$PACKAGE_PREFIX/../setup.bash"

# Gazebo runs without its window unless SIM_GUI=1. With the window, PX4 leaves
# the view alone, it is set further down.
WINDOW_SETTING="HEADLESS=1"
if [ "$SIM_GUI" = "1" ]; then
    WINDOW_SETTING="PX4_GZ_NO_FOLLOW=1"
fi

# one small script per program, each runs in its own tmux session
cat > "$OUTPUT_DIR/run_px4.sh" << EOF
cd "$PX4_DIR"
$WINDOW_SETTING PX4_GZ_WORLD=walls make px4_sitl gz_x500_depth 2>&1 | tee "$OUTPUT_DIR/px4.log"
EOF

cat > "$OUTPUT_DIR/run_mavros.sh" << EOF
export ROS_DOMAIN_ID=$ROS_DOMAIN_ID
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 launch mavros px4.launch fcu_url:=udp://127.0.0.1:14540@ gcs_url:=udp://@127.0.0.1:14550 > "$OUTPUT_DIR/mavros.log" 2>&1
EOF

cat > "$OUTPUT_DIR/run_bridge.sh" << EOF
export ROS_DOMAIN_ID=$ROS_DOMAIN_ID
export GZ_IP=127.0.0.1
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 run ros_gz_bridge parameter_bridge /depth_camera@sensor_msgs/msg/Image@gz.msgs.Image /camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo > "$OUTPUT_DIR/bridge.log" 2>&1
EOF

cat > "$OUTPUT_DIR/run_node.sh" << EOF
export ROS_DOMAIN_ID=$ROS_DOMAIN_ID
source "$WORKSPACE_SETUP"
ros2 run reactive_uav_navigation reactive_navigator_node > "$OUTPUT_DIR/node.log" 2>&1
EOF

# stops everything this script started
stop_everything() {
    tmux kill-session -t sim_node 2> /dev/null
    tmux kill-session -t sim_bridge 2> /dev/null
    tmux kill-session -t sim_mavros 2> /dev/null
    tmux kill-session -t sim_px4 2> /dev/null
    sleep 1
    pkill -x px4 2> /dev/null
    pkill -f "gz sim" 2> /dev/null
    pkill -f "mavros_node" 2> /dev/null
    pkill -f "parameter_bridge" 2> /dev/null
}

stop_everything
rm -f "$OUTPUT_DIR/px4.log"

echo "starting PX4 and Gazebo..."
tmux new -d -s sim_px4 "bash $OUTPUT_DIR/run_px4.sh"

# PX4 is up when its startup script has finished, a first build takes a few minutes
PX4_READY=0
for i in $(seq 1 180); do
    sleep 5
    if grep -q "Startup script returned successfully" "$OUTPUT_DIR/px4.log" 2> /dev/null; then
        PX4_READY=1
        break
    fi
done
if [ "$PX4_READY" = "0" ]; then
    echo "PX4 did not start, see $OUTPUT_DIR/px4.log"
    stop_everything
    exit 2
fi

# without a ground station PX4 only arms when this check is switched off
tmux send-keys -t sim_px4 "param set NAV_DLL_ACT 0" Enter

# After a scenario the drone goes back to the start with PX4's return mode.
# It shall fly back at 2 m and stay in the air there, not climb and land.
# PX4 gets its own values back at the end.
tmux send-keys -t sim_px4 "param set RTL_RETURN_ALT 2" Enter
tmux send-keys -t sim_px4 "param set RTL_DESCEND_ALT 2" Enter
tmux send-keys -t sim_px4 "param set RTL_LAND_DELAY -1" Enter

echo "starting MAVROS, the camera bridge and the navigator node..."
tmux new -d -s sim_mavros "bash $OUTPUT_DIR/run_mavros.sh"
tmux new -d -s sim_bridge "bash $OUTPUT_DIR/run_bridge.sh"
sleep 15
tmux new -d -s sim_node "bash $OUTPUT_DIR/run_node.sh"
sleep 5

# The view of the Gazebo window stays 25 m above the drone and looks straight
# down with north up, so that the tall walls of a map never hide the drone.
# Gazebo does not answer every call. So the camera of the window is asked
# where it is, and the view is set again until it is really up there:
# the first call makes it follow the drone, the second one turns it down.
if [ "$SIM_GUI" = "1" ]; then
    VIEW_SET=""
    for i in $(seq 1 10); do
        gz topic -t /gui/track -m gz.msgs.CameraTrack \
            -p "track_mode: FOLLOW_FREE_LOOK, follow_target: {name: 'x500_depth_0'}, follow_offset: {x: 0, y: 0, z: 25}, follow_pgain: 1.0"
        gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean --timeout 5000 \
            --req "pose: {orientation: {x: -0.5, y: 0.5, z: 0.5, w: 0.5}}" > /dev/null
        sleep 3

        # ok when the camera is 20 to 30 m up (it follows the drone) and is
        # turned the way that looks down with north up
        VIEW_SET="$(timeout 5 gz topic -e -t /gui/camera/pose -n 1 | awk '
            /^position/ {part = "position"}
            /^orientation/ {part = "orientation"}
            $1 == "z:" && part == "position" {height = $2}
            $1 == "z:" && part == "orientation" {turn = $2}
            END {if (height > 20 && height < 30 && (turn > 0.4 || turn < -0.4)) print "ok"}')"
        if [ "$VIEW_SET" = "ok" ]; then
            break
        fi
    done
    if [ "$VIEW_SET" != "ok" ]; then
        echo "could not set the view of the Gazebo window, the tests go on without it"
    fi
fi

python3 "$HERE/sim_test_runner.py" "$SCENARIO_FILE" "$OUTPUT_DIR"
TEST_RESULT=$?

# PX4 keeps what "param set" changed for its next start, so the return mode
# gets its own values back
tmux send-keys -t sim_px4 "param reset RTL_RETURN_ALT RTL_DESCEND_ALT RTL_LAND_DELAY" Enter
sleep 2

echo "stopping the simulation..."
stop_everything
exit $TEST_RESULT
