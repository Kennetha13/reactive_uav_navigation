"""Flies the scenarios of a scenario file in a running PX4 + Gazebo simulation
and checks each one.

The simulation, MAVROS, the camera bridge and the navigator node must already
be running (run_sim_tests.sh starts them). This script takes the walls that
PX4's "walls" world comes with out of the world, takes off and switches to
OFFBOARD. Then, for every scenario:

  1. it checks that the drone is well away from every wall of the scenario
  2. it puts the walls of the scenario into the world
  3. it publishes the path, records the flight and decides pass or fail
  4. it takes the walls away again and sends the drone back to the start.
     PX4 flies that part itself with its return mode, which is much faster
     than the navigator node.

A wall with appears_at is not there at the start of the flight. It is put
into the world while the drone flies, when the drone comes that close to it.

A flight passes when:

  - it ends in the expected navigation state
  - the drone never touched a wall
  - if the goal is expected to be reached, the drone ended close to it

usage: sim_test_runner.py <scenarios.yml> <output_dir>
"""
import csv
import math
import os
import subprocess
import sys
import time

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String

# how often the flight is sampled (s)
SAMPLE_PERIOD = 0.2

# BLOCKED must be held this long before the flight counts as ended in BLOCKED (s)
BLOCKED_SETTLE_TIME = 20.0

# the drone must end this close to the goal when GOAL_REACHED is expected (m)
GOAL_ERROR_LIMIT = 0.6

# the PX4 world the tests are flown in
WORLD = 'walls'

# the walls this world comes with, they are taken out so that every scenario
# can bring its own
WORLD_WALLS = ['box1', 'box2', 'box3', 'box4']

# height of the walls of a scenario (m)
WALL_HEIGHT = 15.0

# longest the flight back to the start may take (s)
RETURN_TIMEOUT = 90

# the drone is back at the start when it is this close to it (m)
START_TOLERANCE = 0.3

# a wall as a Gazebo model: name, then center x y z and the angle the wall is
# turned by, then size x y z twice
WALL_SDF = """<?xml version="1.0"?>
<sdf version="1.9">
  <model name="%s">
    <static>true</static>
    <pose>%.2f %.2f %.2f 0 0 %.4f</pose>
    <link name="link">
      <collision name="collision">
        <geometry><box><size>%.2f %.2f %.2f</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>%.2f %.2f %.2f</size></box></geometry>
        <material>
          <ambient>0.3 0.3 0.3 1</ambient>
          <diffuse>0.7 0.7 0.7 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


class SimTestRunner(Node):
    def __init__(self):
        super().__init__('sim_test_runner')
        self.position = None
        self.nav_state = ''
        self.mavros_state = None

        # folder the wall files are written to, set by main()
        self.output_dir = ''

        self.create_subscription(Odometry, '/mavros/local_position/odom',
                                 self.on_odometry, qos_profile_sensor_data)
        self.create_subscription(String, 'navigation_state', self.on_nav_state, 10)
        self.create_subscription(State, '/mavros/state', self.on_mavros_state, 10)
        self.path_publisher = self.create_publisher(Path, 'planned_path', 10)
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

    def on_odometry(self, msg):
        self.position = msg.pose.pose.position

    def on_nav_state(self, msg):
        self.nav_state = msg.data

    def on_mavros_state(self, msg):
        self.mavros_state = msg

    # lets the callbacks run for a while
    def wait(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    # waits until MAVROS is connected and the navigator node is publishing
    def wait_until_ready(self, timeout):
        end = time.time() + timeout
        while time.time() < end:
            self.wait(0.5)
            connected = self.mavros_state is not None and self.mavros_state.connected
            if connected and self.position is not None and self.nav_state != '':
                return True
        return False

    def set_mode(self, mode):
        request = SetMode.Request()
        request.custom_mode = mode
        future = self.mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

    def arm(self):
        request = CommandBool.Request()
        request.value = True
        future = self.arming_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

    # arms, takes off and hands control to the navigator node
    def take_off(self):
        # PX4 refuses to arm until its checks pass, so keep trying for a while
        for attempt in range(30):
            if self.mavros_state.armed:
                break
            self.arm()
            self.set_mode('AUTO.TAKEOFF')
            self.wait(2.0)
        if not self.mavros_state.armed:
            return False

        # wait until the drone is up
        end = time.time() + 60.0
        while time.time() < end and self.position.z < 2.0:
            self.wait(0.5)
        if self.position.z < 2.0:
            return False
        self.wait(4.0)
        return self.switch_to_offboard()

    # hands control to the navigator node, returns True when PX4 took it.
    # OFFBOARD is only accepted while the node is sending commands.
    def switch_to_offboard(self):
        for attempt in range(10):
            self.set_mode('OFFBOARD')
            self.wait(1.0)
            if self.mavros_state.mode == 'OFFBOARD':
                return True
        return False

    def land(self):
        self.set_mode('AUTO.LAND')
        self.wait(15.0)

    def publish_path(self, waypoints):
        path = Path()
        path.header.frame_id = 'map'
        for waypoint in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = float(waypoint[0])
            pose.pose.position.y = float(waypoint[1])
            pose.pose.position.z = float(waypoint[2])
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path_publisher.publish(path)

    # puts in the walls that only appear during the flight, each one when the
    # drone has come closer to it than its appears_at (m)
    def add_appearing_walls(self, scenario):
        for wall in scenario.get('walls', []):
            if 'appears_at' not in wall or wall.get('in_world', False):
                continue
            distance = distance_to_wall(self.position.x, self.position.y, wall)
            if distance < wall['appears_at']:
                print('  wall %s appears %.1f m from the drone' % (wall['name'], distance), flush=True)
                if not add_wall(scenario, wall, self.output_dir):
                    print('FAIL: could not put the wall into the Gazebo world')
                    sys.exit(2)

    # flies one scenario and returns the recorded samples and the final state
    def fly(self, scenario):
        self.publish_path(scenario['path'])

        # give the node a moment to start on the new path, its state still
        # shows how the previous path ended
        self.wait(3.0)

        # While the drone searches for a way around it may report BLOCKED for
        # a while, so BLOCKED only ends a flight that is expected to end blocked.
        expected_states = scenario['expect']
        if isinstance(expected_states, str):
            expected_states = [expected_states]
        blocked_ends_flight = 'BLOCKED' in expected_states

        samples = []
        final_state = 'TIMEOUT'
        blocked_since = None
        start = time.time()

        while time.time() - start < scenario['timeout']:
            self.wait(SAMPLE_PERIOD)
            now = time.time() - start
            samples.append([now, self.position.x, self.position.y, self.position.z, self.nav_state])
            self.add_appearing_walls(scenario)

            if self.nav_state == 'GOAL_REACHED':
                final_state = 'GOAL_REACHED'
                break

            # BLOCKED only counts once it has been held for a while
            if self.nav_state == 'BLOCKED' and blocked_ends_flight:
                if blocked_since is None:
                    blocked_since = now
                if now - blocked_since >= BLOCKED_SETTLE_TIME:
                    final_state = 'BLOCKED'
                    break
            else:
                blocked_since = None

        return samples, final_state

    # Back to the start, returns True when the drone got there. PX4 flies this
    # part itself with its return mode (the start is where the drone took
    # off). That is much faster than the navigator node, and the world is
    # empty at this point, so there is nothing to avoid.
    def fly_to_start(self, start):
        self.set_mode('AUTO.RTL')

        # the drone is back once it has stayed at the start for 2 s
        end = time.time() + RETURN_TIMEOUT
        time_at_start = 0.0
        while time.time() < end and time_at_start < 2.0:
            self.wait(0.5)
            distance = math.hypot(self.position.x - start[0], self.position.y - start[1])
            if distance < START_TOLERANCE and abs(self.position.z - start[2]) < START_TOLERANCE:
                time_at_start += 0.5
            else:
                time_at_start = 0.0
        if time_at_start < 2.0:
            return False

        # the node may still be on the path of the scenario, this path ends it
        self.publish_path([start])
        return self.switch_to_offboard()


# calls a Gazebo service, returns True when Gazebo answered "done".
# Now and then a call gets no answer at all, so it is tried up to five times.
# Trying again is safe: putting in a wall that is already there or taking out
# one that is already gone changes nothing.
def call_gazebo(service, request_type, request):
    for attempt in range(5):
        answer = subprocess.run(
            ['gz', 'service', '-s', service,
             '--reqtype', request_type, '--reptype', 'gz.msgs.Boolean',
             '--timeout', '5000', '--req', request],
            capture_output=True, text=True)
        if 'data: true' in answer.stdout:
            return True
        print('Gazebo service ' + service + ' did not answer: ' + answer.stdout + answer.stderr, flush=True)
    return False


# takes one model out of the Gazebo world
def remove_model(name):
    request = 'name: "%s", type: MODEL' % name
    return call_gazebo('/world/' + WORLD + '/remove', 'gz.msgs.Entity', request)


# the name of a wall's model in Gazebo
def wall_model_name(scenario, wall):
    return 'wall_' + scenario['name'] + '_' + wall['name']


# puts one wall into the Gazebo world and notes on the wall that it is there
def add_wall(scenario, wall, output_dir):
    name = wall_model_name(scenario, wall)
    size_x = wall['x_max'] - wall['x_min']
    size_y = wall['y_max'] - wall['y_min']
    center_x = (wall['x_min'] + wall['x_max']) / 2.0
    center_y = (wall['y_min'] + wall['y_max']) / 2.0

    # a wall with an angle is turned about its center, Gazebo wants radians
    angle = math.radians(wall.get('angle', 0.0))

    # Gazebo reads the model from a file
    sdf_file = os.path.join(output_dir, name + '.sdf')
    with open(sdf_file, 'w') as f:
        f.write(WALL_SDF % (name, center_x, center_y, WALL_HEIGHT / 2.0, angle,
                            size_x, size_y, WALL_HEIGHT,
                            size_x, size_y, WALL_HEIGHT))

    request = 'sdf_filename: "%s", name: "%s"' % (sdf_file, name)
    if not call_gazebo('/world/' + WORLD + '/create', 'gz.msgs.EntityFactory', request):
        return False
    wall['in_world'] = True
    return True


# puts the walls of a scenario into the Gazebo world, except the ones that
# only appear during the flight
def add_walls(scenario, output_dir):
    for wall in scenario['walls']:
        if 'appears_at' in wall:
            continue
        if not add_wall(scenario, wall, output_dir):
            return False
    return True


# takes the walls of a scenario out of the Gazebo world again,
# returns False when one of them could not be taken out
def remove_walls(scenario):
    for wall in scenario['walls']:
        if not wall.get('in_world', False):
            continue
        if not remove_model(wall_model_name(scenario, wall)):
            return False
        wall['in_world'] = False
    return True


# distance from a point to a wall seen from above, 0 inside the wall
def distance_to_wall(x, y, wall):
    # A wall with an angle is turned about its center. The point is turned
    # back the other way about the same center, after that the wall can be
    # treated as if it was not turned.
    angle = math.radians(wall.get('angle', 0.0))
    center_x = (wall['x_min'] + wall['x_max']) / 2.0
    center_y = (wall['y_min'] + wall['y_max']) / 2.0
    from_center_x = x - center_x
    from_center_y = y - center_y
    x = center_x + from_center_x * math.cos(angle) + from_center_y * math.sin(angle)
    y = center_y - from_center_x * math.sin(angle) + from_center_y * math.cos(angle)

    dx = max(wall['x_min'] - x, 0.0, x - wall['x_max'])
    dy = max(wall['y_min'] - y, 0.0, y - wall['y_max'])
    return math.hypot(dx, dy)


# works out the metrics of one flight and whether it passed
def evaluate(scenario, samples, final_state, walls, drone_radius):
    goal = scenario['path'][-1]

    # length of the path as given
    path_length = 0.0
    for i in range(1, len(scenario['path'])):
        path_length += math.dist(scenario['path'][i - 1], scenario['path'][i])

    distance_flown = 0.0
    blocked_time = 0.0
    avoiding_time = 0.0
    min_clearance = 1000.0
    closest_wall = ''

    for i in range(len(samples)):
        t, x, y, z, state = samples[i]

        for wall in walls:
            d = distance_to_wall(x, y, wall)
            if d < min_clearance:
                min_clearance = d
                closest_wall = wall['name']

        if i > 0:
            previous = samples[i - 1]
            distance_flown += math.dist([x, y, z], previous[1:4])
            if state == 'BLOCKED':
                blocked_time += t - previous[0]
            if state == 'AVOIDING_OBSTACLE':
                avoiding_time += t - previous[0]

    last = samples[-1]
    goal_error = math.dist(last[1:4], goal)

    # expect is one state, or a list of states when more than one is acceptable
    expected_states = scenario['expect']
    if isinstance(expected_states, str):
        expected_states = [expected_states]
    expected_text = ' or '.join(expected_states)

    # the three checks
    problems = []
    if final_state not in expected_states:
        problems.append('ended in ' + final_state + ', expected ' + expected_text)
    if min_clearance < drone_radius:
        problems.append('touched wall ' + closest_wall)
    if final_state == 'GOAL_REACHED' and goal_error > GOAL_ERROR_LIMIT:
        problems.append('ended %.2f m from the goal' % goal_error)

    result = {
        'scenario': scenario['name'],
        'result': 'PASS' if len(problems) == 0 else 'FAIL',
        'final_state': final_state,
        'expected_state': expected_text,
        'time_s': round(last[0], 1),
        'path_length_m': round(path_length, 2),
        'distance_flown_m': round(distance_flown, 2),
        'min_wall_clearance_m': round(min_clearance, 2),
        'closest_wall': closest_wall,
        'goal_error_m': round(goal_error, 2),
        'blocked_time_s': round(blocked_time, 1),
        'avoiding_time_s': round(avoiding_time, 1),
        'problems': '; '.join(problems),
    }
    return result


def save_track(output_dir, name, samples):
    with open(os.path.join(output_dir, 'track_' + name + '.csv'), 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['t', 'x', 'y', 'z', 'state'])
        for t, x, y, z, state in samples:
            writer.writerow(['%.1f' % t, '%.3f' % x, '%.3f' % y, '%.3f' % z, state])


def save_results(output_dir, results):
    with open(os.path.join(output_dir, 'results.csv'), 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(results[0].keys()))
        writer.writeheader()
        for result in results:
            writer.writerow(result)


def main():
    scenario_file = sys.argv[1]
    output_dir = sys.argv[2]
    os.makedirs(output_dir, exist_ok=True)

    with open(scenario_file) as f:
        config = yaml.safe_load(f)

    rclpy.init()
    runner = SimTestRunner()
    runner.output_dir = output_dir

    print('waiting for MAVROS and the navigator node...', flush=True)
    if not runner.wait_until_ready(120.0):
        print('FAIL: the simulation is not ready (MAVROS, odometry or navigation_state missing)')
        sys.exit(2)

    # the walls the world comes with are not part of any scenario
    for name in WORLD_WALLS:
        if not remove_model(name):
            print('FAIL: could not take wall ' + name + ' out of the Gazebo world')
            sys.exit(2)

    print('taking off...', flush=True)
    if not runner.take_off():
        print('FAIL: could not take off and switch to OFFBOARD')
        sys.exit(2)

    results = []
    for scenario in config['scenarios']:
        print('flying ' + scenario['name'] + ': ' + scenario['about'], flush=True)

        # the drone must never start on top of a wall or close to one
        for wall in scenario['walls']:
            clearance = distance_to_wall(runner.position.x, runner.position.y, wall)
            if clearance < config['min_start_clearance']:
                print('FAIL: the drone would start %.1f m from wall %s, at least %.1f m are needed' % (
                    clearance, wall['name'], config['min_start_clearance']))
                sys.exit(2)

        if not add_walls(scenario, output_dir):
            print('FAIL: could not put the walls into the Gazebo world')
            sys.exit(2)

        samples, final_state = runner.fly(scenario)
        result = evaluate(scenario, samples, final_state, scenario['walls'], config['drone_radius'])
        save_track(output_dir, scenario['name'], samples)
        results.append(result)
        print('  %s  ended %s after %.0f s, flew %.1f m, closest wall %s at %.2f m  %s' % (
            result['result'], result['final_state'], result['time_s'], result['distance_flown_m'],
            result['closest_wall'], result['min_wall_clearance_m'], result['problems']), flush=True)

        # back to the start through the empty world, for the next scenario
        if not remove_walls(scenario):
            print('FAIL: could not take the walls out of the Gazebo world')
            save_results(output_dir, results)
            sys.exit(2)
        if not runner.fly_to_start(config['start']):
            print('FAIL: the drone did not get back to the start')
            save_results(output_dir, results)
            sys.exit(2)

    print('landing...', flush=True)
    runner.land()
    save_results(output_dir, results)

    failed = 0
    for result in results:
        if result['result'] != 'PASS':
            failed += 1
    print('%d of %d scenarios passed, results in %s' % (len(results) - failed, len(results), output_dir))
    sys.exit(1 if failed > 0 else 0)


if __name__ == '__main__':
    main()
