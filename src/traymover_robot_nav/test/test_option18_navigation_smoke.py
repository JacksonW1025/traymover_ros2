"""Offline public-goal smoke test with real Nav2 and synthetic map/odometry."""

import importlib.util
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TransformStamped, Twist
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.action import ActionClient, get_action_server_names_and_types_by_node
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
import yaml
import pytest


PKG = Path(__file__).resolve().parents[1]


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('startup', ['immediate', 'delayed_localization', 'shutdown_waiting'])
def test_public_goal_plans_and_produces_velocity_with_real_nav2(tmp_path, monkeypatch, startup):
    launch = load_module('smoke_option18_launch', PKG / 'launch' /
                         'traymover_nav_lidar_realsense_detour.launch.py')
    proxy_module = load_module('smoke_detour_goal_proxy', PKG / 'scripts' / 'detour_goal_proxy.py')
    (tmp_path / 'map.pgm').write_bytes(b'P5\n200 200\n255\n' + bytes([254]) * 40000)
    map_path = tmp_path / 'map.yaml'
    map_path.write_text(yaml.safe_dump(dict(
        image='map.pgm', resolution=0.05, origin=[-5.0, -5.0, 0.0],
        negate=0, occupied_thresh=0.65, free_thresh=0.25)))
    parameters = yaml.safe_load((PKG / 'config' / 'nav2_params.yaml').read_text())
    parameters['map_server']['ros__parameters']['yaml_filename'] = str(map_path)
    navigator = parameters['bt_navigator']['ros__parameters']
    for kind in ('to_pose', 'through_poses'):
        navigator[f'default_nav_{kind}_bt_xml'] = str(
            PKG / 'behavior_trees' / f'simple_navigate_{kind}.xml')
    params_path = tmp_path / 'navigation.yaml'
    params_path.write_text(yaml.safe_dump(parameters))
    domain = 100 + os.getpid() % 20
    if domain == int(os.environ.get('ROS_DOMAIN_ID', '0')):
        domain = 120
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    environment = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY='1',
                       ROS_LOG_DIR=str(tmp_path / 'ros_logs'))
    context = Context()
    context.init(args=[], domain_id=domain)
    executor = SingleThreadedExecutor(context=context)
    node = Node('offline_option18_test', context=context)
    executor.add_node(node)
    static_tf, dynamic_tf = StaticTransformBroadcaster(node), TransformBroadcaster(node)
    map_tf = TransformStamped()
    map_tf.header.frame_id, map_tf.child_frame_id = 'map', 'odom'
    map_tf.transform.rotation.w = 1.0
    if startup == 'immediate':
        static_tf.sendTransform(map_tf)
    odometry = node.create_publisher(Odometry, '/odom', 10)
    robot_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}

    def publish_pose():
        transform = TransformStamped()
        transform.header.stamp = node.get_clock().now().to_msg()
        transform.header.frame_id, transform.child_frame_id = 'odom', 'base_link'
        transform.transform.translation.x = robot_pose['x']
        transform.transform.translation.y = robot_pose['y']
        transform.transform.rotation.z = math.sin(robot_pose['yaw'] / 2.0)
        transform.transform.rotation.w = math.cos(robot_pose['yaw'] / 2.0)
        dynamic_tf.sendTransform(transform)
        message = Odometry()
        message.header, message.child_frame_id = transform.header, 'base_link'
        message.pose.pose.position.x = robot_pose['x']
        message.pose.pose.position.y = robot_pose['y']
        message.pose.pose.orientation = transform.transform.rotation
        odometry.publish(message)

    node.create_timer(0.05, publish_pose)
    plans, velocities = [], []
    node.create_subscription(NavPath, '/plan', plans.append, 10)
    node.create_subscription(Twist, '/cmd_vel_nav', velocities.append, 10)
    proxy = proxy_module.GoalProxy(node, lambda: False, lambda: None)
    client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    processes, outputs = [], []

    def start(package, executable, arguments):
        binary = Path(get_package_prefix(package)) / 'lib' / package / executable
        assert binary.is_file(), f'Build required before smoke test: {binary}'
        output = (tmp_path / f'{executable}.log').open('w')
        outputs.append(output)
        processes.append(subprocess.Popen(
            [str(binary), '--ros-args', *arguments], env=environment,
            stdout=output, stderr=subprocess.STDOUT))

    def logs():
        return '\n'.join(path.read_text() for path in tmp_path.glob('*.log'))

    def until(predicate, timeout=30.0):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            assert all(process.poll() is None for process in processes), logs()
            executor.spin_once(timeout_sec=0.02)
        assert predicate(), logs()

    try:
        arguments = ['--params-file', str(params_path)]
        for source, destination in launch._navigation_remappings():
            arguments.extend(['-r', f'{source}:={destination}'])
        names = ['map_server', 'planner_server', 'controller_server', 'bt_navigator']
        packages = ['nav2_map_server', 'nav2_planner', 'nav2_controller', 'nav2_bt_navigator']
        for package, name in zip(packages, names):
            start(package, name, arguments)
        start('traymover_robot_nav', 'option18_lifecycle_manager', [
            '-p', 'autostart:=true', '-p', f'node_names:=[{", ".join(names)}]',
            '-p', 'bond_timeout:=4.0'])
        if startup != 'immediate':
            # RViz needs the active map while the operator is still deciding
            # where to set the initial pose. Do not publish localization yet.
            state_client = node.create_client(GetState, '/map_server/get_state')
            until(state_client.service_is_ready)
            deadline = time.monotonic() + 15.0
            while True:
                state = state_client.call_async(GetState.Request())
                until(state.done)
                if state.result().current_state.id == State.PRIMARY_STATE_ACTIVE:
                    break
                assert time.monotonic() < deadline, logs()
                executor.spin_once(timeout_sec=0.05)
            if startup == 'shutdown_waiting':
                until(lambda: 'Waiting for initial localization' in logs())
                processes[-1].send_signal(signal.SIGINT)
                assert processes[-1].wait(timeout=5.0) == 0, logs()
                return
            # Longer than the previous 3 x 5 s retry budget, and the observed
            # manual-initial-pose delay. No activation failure during this wait.
            deadline = time.monotonic() + 45.0
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.05)
                assert all(process.poll() is None for process in processes), logs()
                assert 'startup failed' not in logs(), logs()
            assert not plans and not velocities
            map_state = state_client.call_async(GetState.Request())
            until(map_state.done)
            assert map_state.result().current_state.id == State.PRIMARY_STATE_ACTIVE
            static_tf.sendTransform(map_tf)
        until(lambda: 'lifecycle bond monitoring enabled' in logs())
        for name in names:
            state_client = node.create_client(GetState, f'/{name}/get_state')
            until(state_client.service_is_ready)
            state = state_client.call_async(GetState.Request())
            until(state.done)
            assert state.result().current_state.id == State.PRIMARY_STATE_ACTIVE, logs()
        actual = {name for name, _ in get_action_server_names_and_types_by_node(
            node, 'bt_navigator', '/')}
        assert actual == {'/navigate_to_pose_detour_backend',
                          '/navigate_through_poses_detour_backend'}
        until(client.server_is_ready)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = 1.5
        goal.pose.pose.orientation.w = 1.0
        accepted = client.send_goal_async(goal)
        until(accepted.done)
        handle = accepted.result()
        assert handle.accepted
        until(lambda: proxy.motion_authorized and any(path.poses for path in plans)
              and any(twist.linear.x > 0.0 for twist in velocities))
        if startup == 'immediate':
            # Core tests integrate the measured forward segment. Here the real
            # Nav2 backend sees its resulting displaced pose and must compute
            # a NEW path, while the public action remains the same goal.
            robot_pose.update(x=2.2 / math.sqrt(2.0), y=-2.2 / math.sqrt(2.0),
                              yaw=-math.pi / 4.0)
            publish_pose()
            plans.clear()
            velocities.clear()
            proxy.notify_handback(force_replan=True)
            assert not proxy.motion_authorized
            until(lambda: proxy.motion_authorized and proxy._active.attempt == 2
                  and any(path.poses and path.poses[0].pose.position.x > 1.4
                          and path.poses[0].pose.position.y < -1.4 for path in plans))
            assert proxy.saved_request.pose.pose.position.x == 1.5
        result = handle.get_result_async()
        canceled = handle.cancel_goal_async()
        until(canceled.done)
        assert canceled.result().goals_canceling
        until(result.done)
        assert result.result().status == GoalStatus.STATUS_CANCELED
        assert not proxy.motion_authorized
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
        for process in reversed(processes):
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5.0)
        for output in outputs:
            output.close()
        proxy.destroy()
        client.destroy()
        executor.shutdown(timeout_sec=0.0)
        node.destroy_node()
        context.shutdown()
