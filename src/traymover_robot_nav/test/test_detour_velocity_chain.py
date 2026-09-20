"""Exercise the actual DDS velocity chain and installed Collision Monitor.

No driver, camera, localization stack, or physical robot is launched. The goal
proxy and supervisor's sensor/localization observation are deterministic doubles;
command and EStop subscriptions, supervisor state/velocity production, Collision
Monitor lifecycle/polygons, LaserScan/TF ingestion, and final Twist output are real.
"""

import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TransformStamped, Twist
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import detour_supervisor as supervisor  # noqa: E402


class AcceptedGoal:
    """An already accepted original Nav2 goal; goal transport is tested separately."""

    def __init__(self, *args, **kwargs):
        self.has_active_goal = True
        self.motion_authorized = True

    def tick(self):
        pass

    def notify_handback(self, force_replan=False):
        # This fixture only recovers sensor holds, never a completed maneuver.
        assert not force_replan

    def destroy(self):
        pass


@pytest.mark.parametrize('odom_delay', [None, 0.4], ids=['static_odom', 'delayed_odom'])
def test_real_collision_monitor_chain_passthrough_polygon_stop_and_manual_estop(
        monkeypatch, tmp_path, odom_delay):
    prefix = Path(get_package_prefix('nav2_collision_monitor'))
    executable = prefix / 'lib' / 'nav2_collision_monitor' / 'collision_monitor'
    assert executable.is_file() and os.access(executable, os.X_OK)
    parameters = (Path(__file__).resolve().parents[1] / 'config' /
                  'collision_monitor_lidar_realsense_detour.yaml')
    domain_id = 160 + os.getpid() % 60
    monkeypatch.setenv('ROS_DOMAIN_ID', str(domain_id))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setattr(supervisor, 'GoalProxy', AcceptedGoal)
    # Keep the supervisor's perception independent of the CM obstacle stimulus,
    # so a CM polygon stop cannot accidentally pass via the supervisor's gate.
    monkeypatch.setattr(supervisor.DetourSupervisor, 'on_scan', lambda *args: None)
    rclpy.init(args=['--ros-args', '-p', 'simple_detour:=true',
                     '-p', 'realsense_timeout_sec:=1.5', '-p', 'scan_timeout_sec:=2.0'],
               domain_id=domain_id)
    node = supervisor.DetourSupervisor()
    io = Node('detour_velocity_chain_test')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(io)
    process = None
    log_path = tmp_path / 'collision_monitor.log'
    detour_messages, final_messages = [], []
    inputs = {'obstacle': None, 'obstacle_range': 0.60,
              'manual': False, 'paused_source': None}
    scan_stamps = dict.fromkeys(('lidar', 'realsense'), time.monotonic())

    def diagnostics():
        return log_path.read_text() if log_path.exists() else 'CM has not started'

    def until(predicate, label):
        deadline = time.monotonic() + 20.0
        while not predicate() and time.monotonic() < deadline:
            assert process is None or process.poll() is None, diagnostics()
            executor.spin_once(timeout_sec=0.01)
        assert predicate(), f'{label} did not complete within 20 s\n{diagnostics()}'

    def moving(message):
        return (message.linear.x == pytest.approx(0.12)
                and message.linear.y == pytest.approx(0.01)
                and message.angular.z == pytest.approx(0.07))

    def clear_observation(now):
        rays = tuple(supervisor.Ray(0.0, 0.0, math.radians(deg), 4.0)
                     for deg in range(-180, 181))
        scan = supervisor.Scan(now, rays)
        return supervisor.Observation(
            pose=supervisor.Pose(0.0, 0.0, 0.0),
            lidar=supervisor.Scan(scan_stamps['lidar'], rays),
            realsense=supervisor.Scan(scan_stamps['realsense'], rays), perimeter=scan,
            nav_cmd=supervisor.Velocity(node.cmd.linear.x, node.cmd.angular.z),
            nav_cmd_stamp=node.cmd_at, nav_goal_active=node.proxy.has_active_goal,
            aggregate_estop=node.estop['aggregate'], manual_estop=node.estop['manual'],
            auto_estop=node.estop['auto'], auto_reason=node.estop['reason'],
            estop_fresh=all(now - stamp <= 1.0 for stamp in node.estop_at.values()),
            tf_healthy=True, localization_healthy=True)

    monkeypatch.setattr(node, 'observation', clear_observation)
    try:
        io.create_subscription(Twist, '/cmd_vel_detour', detour_messages.append, 10)
        io.create_subscription(Twist, '/cmd_vel', final_messages.append, 10)
        nav = io.create_publisher(Twist, '/cmd_vel_nav', 1)
        scans = {key: io.create_publisher(LaserScan, topic, qos_profile_sensor_data)
                 for key, topic in (('lidar', '/scan'), ('realsense', '/scan_realsense'))}
        state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        states = {key: io.create_publisher(
            String if key == 'auto_reason' else Bool, '/traymover_estop/' + key, state_qos)
            for key in ('state', 'manual_state', 'auto_state', 'auto_reason')}
        broadcaster = StaticTransformBroadcaster(io)
        odom_broadcaster = TransformBroadcaster(io)
        transforms = []
        frames = [('base_link', 'chain_test_lidar'), ('base_link', 'chain_test_realsense')]
        if odom_delay is None:
            frames.append(('odom', 'base_link'))
        for parent, child in frames:
            transform = TransformStamped()
            transform.header.stamp = io.get_clock().now().to_msg()
            transform.header.frame_id, transform.child_frame_id = parent, child
            transform.transform.rotation.w = 1.0
            if child == 'chain_test_realsense':
                transform.transform.translation.x = 0.10
            transforms.append(transform)
        broadcaster.sendTransform(transforms)

        def publish_inputs():
            if odom_delay is not None:
                transform = TransformStamped()
                transform.header.stamp = (
                    io.get_clock().now() - Duration(seconds=odom_delay)).to_msg()
                transform.header.frame_id, transform.child_frame_id = 'odom', 'base_link'
                transform.transform.rotation.w = 1.0
                odom_broadcaster.sendTransform(transform)
            command = Twist()
            command.linear.x, command.linear.y, command.angular.z = 0.12, 0.01, 0.07
            nav.publish(command)
            states['state'].publish(Bool(data=inputs['manual']))
            states['manual_state'].publish(Bool(data=inputs['manual']))
            states['auto_state'].publish(Bool(data=False))
            states['auto_reason'].publish(String(data='NONE'))
            for key, publisher in scans.items():
                if inputs['paused_source'] == key:
                    continue
                scan = LaserScan()
                scan.header.stamp = io.get_clock().now().to_msg()
                scan.header.frame_id = 'chain_test_' + key
                scan.angle_min, scan.angle_max = -math.pi, math.pi
                scan.angle_increment = math.pi / 180.0
                scan.range_min, scan.range_max = 0.1, 8.0
                scan.ranges = [4.0] * 361
                if inputs['obstacle'] == key:
                    # Nine returns inside the unchanged stop polygon, including
                    # the camera's 0.10 m mounting translation.
                    for index in range(176, 185):
                        scan.ranges[index] = inputs['obstacle_range']
                scan_stamps[key] = time.monotonic()
                publisher.publish(scan)

        io.create_timer(0.025, publish_inputs)
        with log_path.open('w') as output:
            process = subprocess.Popen(
                [str(executable), '--ros-args', '--params-file', str(parameters)],
                stdout=output, stderr=subprocess.STDOUT, env=os.environ.copy())
        lifecycle = io.create_client(ChangeState, '/collision_monitor/change_state')
        until(lifecycle.service_is_ready, 'Collision Monitor lifecycle discovery')
        for transition in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
            request = ChangeState.Request()
            request.transition.id = transition
            future = lifecycle.call_async(request)
            until(future.done, f'Collision Monitor transition {transition}')
            assert future.result().success, diagnostics()

        until(lambda: (node.gate.state == 'NORMAL' and detour_messages and final_messages
                       and moving(detour_messages[-1]) and moving(final_messages[-1])),
              'Nav2 -> Supervisor -> Collision Monitor pass-through')

        def assert_passthrough_for(seconds, label):
            final_messages.clear()
            until_time = time.monotonic() + seconds
            until(lambda: time.monotonic() >= until_time, label)
            assert final_messages and all(moving(message) for message in final_messages)
            assert node.gate.state == 'NORMAL'

        inputs['obstacle'] = 'lidar'
        assert_passthrough_for(0.4, 'LiDAR-only obstacle leaves normal navigation unchanged')
        inputs['obstacle'] = 'realsense'
        inputs['obstacle_range'] = 1.1
        assert_passthrough_for(0.4, 'no independent distant camera slowdown')

        # Only CM sees the near camera obstacle. The supervisor still emits the
        # original Nav2 command, proving the close stop region remains effective.
        inputs['obstacle_range'] = 0.60
        final_messages.clear()
        until(lambda: (final_messages and final_messages[-1] == Twist()
                       and moving(detour_messages[-1])), 'Collision Monitor camera stop')
        assert node.gate.state == 'NORMAL'
        inputs['obstacle'] = None
        final_messages.clear()
        until(lambda: final_messages and moving(final_messages[-1]), 'camera stop clearing')

        # LiDAR remains a maneuver clearance source, but losing it during normal
        # camera-clear navigation must not create a fault hold.
        inputs['paused_source'] = 'lidar'
        assert_passthrough_for(node.config.scan_timeout_sec + 0.3, 'normal LiDAR outage')
        inputs['paused_source'] = None

        # Humble CM ignores stale sources. The independent camera watchdog must
        # therefore stop the complete chain even when CM has no current returns.
        inputs['paused_source'] = 'realsense'
        until(lambda: (node.gate.state == 'FAULT_HOLD'
                       and detour_messages[-1] == Twist()
                       and final_messages[-1] == Twist()), 'camera scan watchdog')
        assert 'RealSense scan stale/invalid' in node.gate.reason
        inputs['paused_source'] = None
        until(lambda: node.gate.state == 'NORMAL' and moving(final_messages[-1]),
              'camera scan recovery')

        inputs['manual'] = True
        detour_messages.clear()
        final_messages.clear()
        until(lambda: (node.gate.state == 'HOLD_ESTOP' and detour_messages and final_messages
                       and detour_messages[-1] == Twist() and final_messages[-1] == Twist()),
              'Manual EStop zero through both velocity boundaries')
        detour_messages.clear()
        final_messages.clear()
        hold_until = time.monotonic() + 0.25
        until(lambda: time.monotonic() >= hold_until, 'sustained manual hold')
        assert detour_messages and final_messages
        assert all(message == Twist() for message in detour_messages)
        assert all(message == Twist() for message in final_messages)
        assert node.estop['manual'] and node.gate.state == 'HOLD_ESTOP'
        assert 'Failed to get' not in diagnostics()
        assert 'extrapolation' not in diagnostics()
    finally:
        node.destroy_node()
        io.destroy_node()
        executor.shutdown(timeout_sec=0.0)
        rclpy.shutdown()
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2.0)
