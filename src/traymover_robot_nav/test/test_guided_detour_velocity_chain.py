"""Exercise the installed Collision Monitor in the map-free guided DDS chain.

Only synthetic scans, a static sensor transform, and bounded commands are sent.
No chassis driver, map, localization, or navigation server is launched. The
guided adapter is tested separately; its publisher is deliberately replaced so
these checks only pass when Collision Monitor itself gates the final output.
Sensor freshness belongs to the guided adapter/core: the installed Humble
Collision Monitor ignores stale scan sources rather than stopping the robot.
"""

import math
import os
from pathlib import Path
import random
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TransformStamped, Twist
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener


def test_guided_chain_without_map_or_odom_transform(monkeypatch, tmp_path):
    prefix = Path(get_package_prefix('nav2_collision_monitor'))
    executable = (prefix / 'lib' / 'nav2_collision_monitor' /
                  'collision_monitor')
    assert executable.is_file() and os.access(executable, os.X_OK)
    parameters = (Path(__file__).resolve().parents[1] / 'config' /
                  'collision_monitor_guided_detour_test.yaml')
    domain_id = random.SystemRandom().randrange(50, 150)
    monkeypatch.setenv('ROS_DOMAIN_ID', str(domain_id))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    rclpy.init(args=[], domain_id=domain_id)
    io = Node('guided_detour_velocity_chain_test')
    executor = SingleThreadedExecutor()
    executor.add_node(io)
    process = None
    log_path = tmp_path / 'guided_collision_monitor.log'
    output_messages = []
    inputs = {'linear': 0.03, 'angular': 0.0, 'obstacle': None}

    def diagnostics():
        return (log_path.read_text() if log_path.exists()
                else 'CM has not started')

    def until(predicate, label, timeout=8.0):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            assert process is None or process.poll() is None, diagnostics()
            executor.spin_once(timeout_sec=0.01)
        assert predicate(), (
            f'{label} did not complete within {timeout} s\n{diagnostics()}')

    def spin_for(duration):
        deadline = time.monotonic() + duration
        until(lambda: time.monotonic() >= deadline,
              'bounded observation', duration + 1.0)

    def latest_is_zero():
        return bool(output_messages) and output_messages[-1][1] == Twist()

    def latest_is_commanded():
        if not output_messages:
            return False
        command = output_messages[-1][1]
        return (command.linear.x == pytest.approx(inputs['linear']) and
                command.linear.y == 0.0 and
                command.angular.z == pytest.approx(inputs['angular']))

    def assert_sustained_zero():
        # Longer than production's 0.5 s stop-publication timeout. The guided
        # session needs continued downstream zero heartbeats while stopped.
        output_messages.clear()
        spin_for(0.8)
        assert len(output_messages) >= 8, diagnostics()
        assert output_messages[-1][0] - output_messages[0][0] > 0.5
        assert all(message == Twist() for _, message in output_messages), (
            diagnostics())

    try:
        io.create_subscription(
            Twist, '/cmd_vel', lambda message: output_messages.append(
                (time.monotonic(), message)), 10)
        commands = io.create_publisher(Twist, '/cmd_vel_detour_test', 1)
        scans = {topic: io.create_publisher(
            LaserScan, topic, qos_profile_sensor_data)
                 for topic in ('/scan', '/scan_realsense')}
        tf_buffer = Buffer()
        tf_listener = TransformListener(tf_buffer, io)
        broadcaster = StaticTransformBroadcaster(io)
        transform = TransformStamped()
        transform.header.stamp = io.get_clock().now().to_msg()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'guided_chain_test_laser'
        transform.transform.rotation.w = 1.0
        broadcaster.sendTransform(transform)

        def publish_inputs():
            command = Twist()
            command.linear.x = inputs['linear']
            command.angular.z = inputs['angular']
            commands.publish(command)
            for topic, publisher in scans.items():
                scan = LaserScan()
                scan.header.stamp = io.get_clock().now().to_msg()
                scan.header.frame_id = 'guided_chain_test_laser'
                scan.angle_min, scan.angle_max = -math.pi, math.pi
                scan.angle_increment = math.pi / 180.0
                scan.range_min, scan.range_max = 0.1, 8.0
                scan.ranges = [4.0] * 361
                if topic == inputs['obstacle']:
                    # Nine returns within the production stop polygon. Only
                    # this source changes; command publication stays nonzero.
                    for index in range(176, 185):
                        scan.ranges[index] = 0.60
                publisher.publish(scan)

        io.create_timer(0.025, publish_inputs)
        with log_path.open('w') as output:
            process = subprocess.Popen(
                [str(executable), '--ros-args', '--params-file',
                 str(parameters)],
                stdout=output, stderr=subprocess.STDOUT, env=os.environ.copy())
        lifecycle = io.create_client(
            ChangeState, '/collision_monitor/change_state')
        until(lifecycle.service_is_ready,
              'Collision Monitor lifecycle discovery')

        def transition(transition_id):
            request = ChangeState.Request()
            request.transition.id = transition_id
            future = lifecycle.call_async(request)
            until(future.done, f'Collision Monitor transition {transition_id}')
            assert future.result().success, diagnostics()

        transition(Transition.TRANSITION_CONFIGURE)
        spin_for(0.3)
        assert not output_messages, 'Inactive CM published velocity'
        transition(Transition.TRANSITION_ACTIVATE)
        until(latest_is_commanded, 'map-free clear forward pass-through')
        until(lambda: tf_buffer.can_transform(
            'base_link', 'guided_chain_test_laser', Time()),
            'sensor static transform')
        assert not tf_buffer.can_transform('base_link', 'odom', Time())
        assert not tf_buffer.can_transform('base_link', 'map', Time())

        inputs['linear'], inputs['angular'] = 0.0, 0.15
        output_messages.clear()
        until(latest_is_commanded, 'map-free pure-turn pass-through')
        inputs['linear'], inputs['angular'] = 0.03, 0.0
        output_messages.clear()
        until(latest_is_commanded, 'return to clear forward pass-through')

        for topic in scans:
            inputs['obstacle'] = topic
            output_messages.clear()
            until(latest_is_zero,
                  f'{topic} independently activates CM stop polygon')
            assert_sustained_zero()
            inputs['obstacle'] = None
            output_messages.clear()
            until(latest_is_commanded, f'{topic} polygon clearing')

        inputs['linear'] = 0.0
        output_messages.clear()
        until(latest_is_zero, 'explicit guided publisher zero passes through')
        transition(Transition.TRANSITION_DEACTIVATE)
        spin_for(0.1)
        output_messages.clear()
        inputs['linear'] = 0.03
        spin_for(0.3)
        assert not output_messages, 'Inactive CM emitted new commands'
        tf_listener.unregister()
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2.0)
        io.destroy_node()
        executor.shutdown(timeout_sec=0.0)
        rclpy.shutdown()
