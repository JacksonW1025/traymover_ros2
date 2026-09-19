#!/usr/bin/env python3
"""Exercise real option 20 ROS nodes with a synthetic room, never hardware.

Source ROS and this workspace first, then run with /usr/bin/python3. The test
uses a separate localhost-only DDS domain, validates AMCL TF and NavFn planning,
and sends commands only to prove that missing sensor evidence keeps output zero.
This is an integration check, not a substitute for physical stopping tests.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time


def assert_domain_unused():
    """Refuse to share even the test domain with a pre-existing ROS graph."""
    import rclpy
    rclpy.init()
    node = rclpy.create_node('option20_domain_check')
    try:
        until = time.monotonic() + 1.
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.1)
        others = [name for name, namespace in node.get_node_names_and_namespaces()
                  if (name, namespace) != ('option20_domain_check', '/')]
        if others:
            raise RuntimeError('test domain already has ROS nodes: ' + ', '.join(others))
    finally:
        node.destroy_node()
        rclpy.shutdown()


def synthetic_map(directory):
    import numpy as np
    import yaml
    from PIL import Image

    directory.mkdir()
    grid = np.full((60, 80), 254, dtype=np.uint8)
    grid[[0, -1], :] = 0
    grid[:, [0, -1]] = 0
    for name in ('localization', 'navigation'):
        Image.fromarray(grid).save(directory / (name + '.pgm'))
        (directory / (name + '.yaml')).write_text(yaml.safe_dump({
            'image': name + '.pgm', 'resolution': .1, 'origin': [-4., -3., 0.],
            'negate': 0, 'occupied_thresh': .65, 'free_thresh': .25,
            'mode': 'trinary'}))
    pcd = directory / 'room.pcd'
    pcd.write_text('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n'
                   'COUNT 1 1 1\nWIDTH 4\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
                   'POINTS 4\nDATA ascii\n-4 -3 1.5\n4 -3 1.5\n4 3 1.5\n-4 3 1.5\n')
    stat = pcd.stat()
    (directory / 'manifest.json').write_text(json.dumps({
        'schema_version': 1, 'pcd_path': str(pcd), 'pcd_size': stat.st_size,
        'pcd_mtime_ns': stat.st_mtime_ns, 'original_to_map': np.eye(4).tolist(),
        'map_frame': 'map', 'bands': [[1.1, 1.5], [1.5, 1.9], [1.9, 2.3]],
        'floor_z': 0., 'origin': [-4., -3., 0.], 'resolution': .1}))


def exercise(process, timeout, log_path, startup_only=False):
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from rclpy.time import Time
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped, Twist
    from lifecycle_msgs.srv import GetState
    from nav_msgs.msg import Odometry, OccupancyGrid
    from nav2_msgs.action import ComputePathToPose, NavigateToPose
    from sensor_msgs.msg import LaserScan, PointCloud2
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header, String
    from tf2_ros import Buffer, TransformBroadcaster, TransformListener

    rclpy.init()
    node = Node('option20_offline_probe')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    broadcaster = TransformBroadcaster(node)
    odom_pub = node.create_publisher(Odometry, '/odom', 10)
    scan_pub = node.create_publisher(LaserScan, '/scan_localization', qos_profile_sensor_data)
    initial_pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    command_pub = node.create_publisher(Twist, '/option20/cmd_vel_nav', 10)
    clearing_pub = node.create_publisher(PointCloud2, '/option20/lidar_clearing',
                                         qos_profile_sensor_data)
    probe_commands_enabled = [True]
    received = {'safe': [], 'health': [], 'pose': [], 'nav': [], 'feedback': [],
                'maps': {}, 'pcd_points': 0}
    map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def capture_map(topic, msg):
        received['maps'][topic] = {'width': msg.info.width, 'height': msg.info.height,
                                  'resolution': msg.info.resolution,
                                  'frame': msg.header.frame_id, 'cells': len(msg.data)}

    for topic in ('/map', '/localization_map'):
        node.create_subscription(OccupancyGrid, topic,
                                 lambda msg, name=topic: capture_map(name, msg), map_qos)
    node.create_subscription(PointCloud2, '/option20/pcd_map',
                             lambda msg: received.update(pcd_points=msg.width), map_qos)
    node.create_subscription(Twist, '/option20/cmd_vel_safe',
                             lambda msg: received['safe'].append((msg.linear.x, msg.angular.z)), 10)
    node.create_subscription(String, '/option20/health',
                             lambda msg: received['health'].append(json.loads(msg.data)), 10)
    node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                             lambda msg: received['pose'].append(msg), 10)
    node.create_subscription(Twist, '/option20/cmd_vel_nav',
                             lambda msg: received['nav'].append((msg.linear.x, msg.angular.z)), 10)
    deadline = time.monotonic() + timeout
    last_log_check = [0.]
    timer = None

    def step(predicate, label, seconds=30):
        end = min(deadline, time.monotonic() + seconds)
        while time.monotonic() < end:
            if process.poll() is not None:
                raise AssertionError('launch exited before ' + label)
            if time.monotonic() - last_log_check[0] > .5:
                log_text = log_path.read_text(errors='replace')
                if ('Failed to bring up all requested nodes' in log_text
                        or 'process has died' in log_text
                        or 'Traceback (most recent call last)' in log_text):
                    raise AssertionError('ROS bringup/node failure before ' + label
                                         + '; inspect launch.log')
                last_log_check[0] = time.monotonic()
            rclpy.spin_once(node, timeout_sec=.05)
            if predicate():
                return
        raise AssertionError('timed out waiting for ' + label)

    def has_tf():
        return buffer.can_transform('map', 'base_footprint', Time())

    def publish():
        stamp = node.get_clock().now().to_msg()
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'
        transform.transform.rotation.w = 1.
        broadcaster.sendTransform(transform)
        odom = Odometry()
        odom.header = transform.header
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.orientation.w = 1.
        odom_pub.publish(odom)
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = 'option20_scan'
        scan.angle_min = -math.pi
        scan.angle_increment = math.pi / 360
        scan.angle_max = scan.angle_min + 719 * scan.angle_increment
        scan.range_min = .1
        scan.range_max = 15.
        scan.scan_time = .1
        # Nearest occupied-cell center in the rectangular room, including the
        # actual .435 m scan-origin displacement used by option20.launch.py.
        for i in range(720):
            angle = scan.angle_min + i * scan.angle_increment
            c, s = math.cos(angle), math.sin(angle)
            xr = (3.95 - .435) / c if c > 1e-9 else (-3.95 - .435) / c if c < -1e-9 else math.inf
            yr = 2.95 / abs(s) if abs(s) > 1e-9 else math.inf
            scan.ranges.append(min(xr, yr))
        scan_pub.publish(scan)
        # Synthetic observed floor rays clear the local costmap. This does not
        # provide any guard health, EStop, or required obstacle observations.
        points = [(x / 10., y / 10., 0.) for x in range(-38, 39, 2)
                  for y in range(-28, 29, 2)]
        clearing_pub.publish(point_cloud2.create_cloud_xyz32(
            Header(stamp=stamp, frame_id='base_footprint'), points))
        if probe_commands_enabled[0]:
            command = Twist()
            command.linear.x = .1
            command_pub.publish(command)

    try:
        # No test inputs yet: startup must produce fresh zeros by itself.
        step(lambda: len(received['safe']) >= 10 and received['health'],
             'guard startup output', seconds=35)
        assert all(v == (0., 0.) for v in received['safe'])
        assert not received['health'][-1]['ready']
        if startup_only:
            step(lambda: len(received['maps']) == 2 and received['pcd_points'] > 0,
                 'real occupancy maps and display PCD', seconds=65)
            states = {}
            for name in ('map_server', 'localization_map_server', 'amcl'):
                client = node.create_client(GetState, '/' + name + '/get_state')
                step(client.service_is_ready, name + ' lifecycle service')
                while True:
                    future = client.call_async(GetState.Request())
                    step(future.done, name + ' lifecycle state')
                    states[name] = future.result().current_state.label
                    if future.result().current_state.id == 3:
                        break
                    wake = time.monotonic() + .2
                    step(lambda: time.monotonic() >= wake, name + ' activation retry')
            assert all(v == (0., 0.) for v in received['safe'])
            assert all(value['frame'] == 'map' and value['cells'] > 0
                       for value in received['maps'].values())
            return {'result': 'PASS', 'mode': 'prepared_map_startup_only',
                    'lifecycle_states': states, 'maps': received['maps'],
                    'display_pcd_points': received['pcd_points'],
                    'safe_zero_samples': len(received['safe'])}
        timer = node.create_timer(.1, publish)
        client = node.create_client(GetState, '/amcl/get_state')
        step(lambda: client.service_is_ready(), 'AMCL lifecycle service')
        while True:
            future = client.call_async(GetState.Request())
            step(future.done, 'AMCL lifecycle response')
            if future.result().current_state.id == 3:
                break
            wake = time.monotonic() + .2
            step(lambda: time.monotonic() >= wake, 'AMCL activation retry')
        initial = PoseWithCovarianceStamped()
        initial.header.frame_id = 'map'
        initial.header.stamp = node.get_clock().now().to_msg()
        initial.pose.pose.orientation.w = 1.
        initial.pose.covariance[0] = .0025
        initial.pose.covariance[7] = .0025
        initial.pose.covariance[35] = .0025
        initial_pub.publish(initial)
        step(lambda: received['pose'] and has_tf(), 'AMCL pose and map-to-base TF')
        pose = received['pose'][-1].pose.pose.position
        assert math.hypot(pose.x, pose.y) < .3, (pose.x, pose.y)
        states = {}
        for name in ('map_server', 'localization_map_server', 'amcl', 'planner_server',
                     'controller_server', 'behavior_server', 'bt_navigator'):
            lifecycle = node.create_client(GetState, '/' + name + '/get_state')
            step(lifecycle.service_is_ready, name + ' service')
            while True:
                future = lifecycle.call_async(GetState.Request())
                step(future.done, name + ' state')
                states[name] = future.result().current_state.label
                if future.result().current_state.id == 3:
                    break
                wake = time.monotonic() + .2
                step(lambda: time.monotonic() >= wake, name + ' activation retry')
        # Lifecycle activation precedes the costmap's first scheduled update.
        wake = time.monotonic() + 2.
        step(lambda: time.monotonic() >= wake, 'first costmap updates')
        action = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
        step(action.server_is_ready, 'planner action')
        goal = ComputePathToPose.Goal()
        goal.goal = PoseStamped()
        goal.goal.header.frame_id = 'map'
        goal.goal.header.stamp = node.get_clock().now().to_msg()
        goal.goal.pose.position.x = 1.5
        goal.goal.pose.orientation.w = 1.
        goal.planner_id = 'GridBased'
        sent = action.send_goal_async(goal)
        step(sent.done, 'planner acceptance')
        assert sent.result().accepted
        result_future = sent.result().get_result_async()
        step(result_future.done, 'planner result')
        result = result_future.result()
        assert result.status == 4, result.status
        path = result.result.path
        assert len(path.poses) > 3
        assert abs(path.poses[-1].pose.position.x - 1.5) < .2
        # Exercise the actual default BT and controller; do not count the probe's
        # earlier intentional commands as controller output.
        probe_commands_enabled[0] = False
        node.destroy_publisher(command_pub)
        wake = time.monotonic() + .5
        step(lambda: time.monotonic() >= wake, 'probe command queue drain')
        received['nav'].clear()
        navigator = ActionClient(node, NavigateToPose, '/navigate_to_pose')
        step(navigator.server_is_ready, 'navigation action')
        navigation_goal = NavigateToPose.Goal()
        navigation_goal.pose = goal.goal
        sent = navigator.send_goal_async(navigation_goal,
                                        feedback_callback=received['feedback'].append)
        step(sent.done, 'navigation goal acceptance')
        handle = sent.result()
        assert handle.accepted, 'NavigateToPose rejected synthetic goal'
        navigation_result = handle.get_result_async()

        def controller_executing():
            if navigation_result.done():
                raise AssertionError('navigation terminated before controller execution: '
                                     + str(navigation_result.result().status))
            return (received['feedback'] and
                    any(abs(v) > .001 or abs(w) > .001 for v, w in received['nav']))

        step(controller_executing, 'real BT feedback and controller command', seconds=20)
        cancellation = handle.cancel_goal_async()
        step(cancellation.done, 'navigation cancellation acceptance')
        assert cancellation.result().goals_canceling, 'navigation cancellation was rejected'
        step(navigation_result.done, 'navigation cancellation result')
        assert navigation_result.result().status == 5, navigation_result.result().status
        assert len(received['safe']) > 10
        assert all(v == (0., 0.) for v in received['safe']), 'guard passed command without complete sensors'
        assert any('depth' in reason for reason in received['health'][-1]['reasons'])
        return {'result': 'PASS', 'lifecycle_states': states,
                'amcl_position': [pose.x, pose.y], 'path_poses': len(path.poses),
                'navigation_feedback_samples': len(received['feedback']),
                'controller_commands': len(received['nav']), 'navigation_cancelled': True,
                'safe_zero_samples': len(received['safe']),
                'last_guard_reasons': received['health'][-1]['reasons']}
    finally:
        if timer is not None:
            node.destroy_timer(timer)
        listener.unregister()
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain-id', type=int, default=93)
    parser.add_argument('--timeout', type=float, default=110.)
    parser.add_argument('--output-directory', type=Path)
    parser.add_argument('--map-directory', type=Path, help=(
        'Verify startup with these prepared real maps; no synthetic inputs or goals.'))
    args = parser.parse_args()
    if not 86 <= args.domain_id <= 100:
        parser.error('use a dedicated test domain in 86..100')
    os.environ.update(ROS_DOMAIN_ID=str(args.domain_id), ROS_LOCALHOST_ONLY='1',
                      ROS2CLI_NO_DAEMON='1')
    directory = (args.output_directory or Path(tempfile.mkdtemp(prefix='option20-offline-'))).resolve()
    directory.mkdir(parents=True, exist_ok=True)
    os.environ['ROS_LOG_DIR'] = str(directory / 'ros-logs')
    maps = args.map_directory.resolve() if args.map_directory else directory / 'map'
    if args.map_directory is None:
        synthetic_map(maps)
    log = directory / 'launch.log'
    process = None
    report = {}
    try:
        assert_domain_unused()
        with log.open('w') as output:
            process = subprocess.Popen([
                'ros2', 'launch', 'traymover_robot_nav', 'option20.launch.py',
                'map_directory:=' + str(maps), 'bringup_hardware:=false',
                'launch_rviz:=false'], stdout=output, stderr=subprocess.STDOUT,
                start_new_session=True, env=os.environ.copy())
            report = exercise(process, args.timeout, log,
                              startup_only=args.map_directory is not None)
            before_shutdown = log.read_text(errors='replace')
            assert 'process has died' not in before_shutdown, 'ROS node died; inspect launch.log'
            assert 'Traceback (most recent call last)' not in before_shutdown
    except Exception as error:
        report = {'result': 'FAIL', 'error': str(error)}
    finally:
        if process is not None:
            try:
                # Launch forwards SIGINT to its nodes; signaling the whole group
                # here would interrupt node teardown twice.
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
            # A launch failure can leave descendants after the launch parent
            # exits. The private process group belongs exclusively to this test.
            try:
                os.killpg(process.pid, signal.SIGTERM)
                time.sleep(.2)
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        report.update(domain_id=args.domain_id, localhost_only=True, hardware_started=False,
                      launch_log=str(log))
        (directory / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2))
    return 0 if report.get('result') == 'PASS' else 1


if __name__ == '__main__':
    sys.exit(main())
