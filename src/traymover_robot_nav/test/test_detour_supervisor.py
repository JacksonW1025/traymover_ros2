"""Synthetic ROS boundary tests; no chassis or real sensor is launched."""
import math
import os
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402
from tf2_ros import LookupException  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import detour_supervisor as supervisor  # noqa: E402


class Proxy:
    def __init__(self, *args, **kwargs):
        self.has_active_goal = True
        self.motion_authorized = True
        self.handbacks = 0

    def tick(self):
        pass

    def notify_handback(self):
        self.handbacks += 1

    def destroy(self):
        pass


@pytest.fixture
def node(monkeypatch):
    monkeypatch.setattr(supervisor, 'GoalProxy', Proxy)
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    rclpy.init(args=[], domain_id=160 + os.getpid() % 60)
    instance = supervisor.DetourSupervisor()
    yield instance
    instance.destroy_node()
    rclpy.shutdown()


def scan(node, age=0.0):
    msg = LaserScan()
    msg.header.frame_id = 'laser'
    msg.header.stamp = rclpy.time.Time(
        nanoseconds=node.get_clock().now().nanoseconds - int(age * 1e9)).to_msg()
    msg.angle_min = -math.pi
    msg.angle_increment = math.pi / 180
    msg.range_min, msg.range_max = 0.2, 8.0
    msg.ranges = [4.0] * 361
    return msg


def transform(node):
    msg = TransformStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.transform.rotation.w = 1.0
    return msg


class TransformBuffer:
    def __init__(self, node):
        self.node = node

    def lookup_transform(self, *args):
        return transform(self.node)


def healthy(node):
    node.tf = TransformBuffer(node)
    for key in ('lidar', 'realsense', 'perimeter'):
        node.on_scan(key, scan(node))
    loc = PoseWithCovarianceStamped()
    loc.header.stamp = node.get_clock().now().to_msg()
    loc.header.frame_id = 'map'
    loc.pose.pose.orientation.w = 1.0
    node.on_localization(loc)
    for key in ('aggregate', 'manual', 'auto'):
        node.on_estop(key, Bool(data=False))
    node.on_estop('reason', SimpleNamespace(data='NONE'))


def capture(node):
    messages = []
    node.output = SimpleNamespace(publish=messages.append)
    return messages


def test_scan_transforms_origins_and_does_not_make_nan_free(node):
    msg = scan(node)
    msg.angle_min, msg.angle_increment = 0.0, math.pi / 2
    msg.ranges = [1.0, float('nan'), float('inf'), float('-inf'), 0.1, 9.0]
    tf = transform(node).transform
    tf.translation.x = 0.3
    tf.rotation.z, tf.rotation.w = math.sin(math.pi / 4), math.cos(math.pi / 4)
    result = supervisor.scan_to_rays(msg, tf, 10.0, allow_infinite=True)
    assert result.stamp == 10.0
    assert len(result.rays) == 2
    assert result.rays[0].origin_x == 0.3
    assert result.rays[0].angle == pytest.approx(math.pi / 2)
    assert result.rays[0].hit
    assert result.rays[1].distance == 8.0
    assert not result.rays[1].hit


def test_bad_scan_transform_is_not_accepted(node):
    tf = transform(node).transform
    tf.rotation.w = 0.0
    with pytest.raises(ValueError):
        supervisor.scan_to_rays(scan(node), tf, time.monotonic())


def test_delayed_scans_retain_acquisition_age_and_duplicates_fail_closed(node):
    node.tf = TransformBuffer(node)
    msg = scan(node, age=2.0)
    node.on_scan('lidar', msg)
    assert node.scans['lidar'] is None
    msg = scan(node)
    node.on_scan('lidar', msg)
    assert node.scans['lidar'] is not None
    node.on_scan('lidar', msg)
    assert node.scans['lidar'] is None
    assert not node.scan_tf_ok['lidar']


def test_tf_failure_and_localization_staleness_remain_distinct(node):
    healthy(node)
    obs = node.observation(time.monotonic())
    assert obs.tf_healthy and obs.localization_healthy and obs.estop_fresh
    node.localization_at = time.monotonic() - 6
    obs = node.observation(time.monotonic())
    assert obs.tf_healthy and not obs.localization_healthy

    class Missing:
        def lookup_transform(self, *args):
            raise LookupException('synthetic missing TF')

    node.tf = Missing()
    assert not node.observation(time.monotonic()).tf_healthy
    out = capture(node)
    node.on_tick()
    assert out[-1] == Twist()
    assert node.gate.state == 'FAULT_HOLD'


def test_manual_estop_callback_asserts_zero_immediately(node):
    healthy(node)
    out = capture(node)
    node.on_estop('manual', Bool(data=True))
    assert out[-1] == Twist()
    node.on_tick()
    assert out[-1] == Twist()
    assert node.gate.state == 'HOLD_ESTOP'
    assert node.estop['manual']  # Supervisor never releases an EStop.


def test_stale_estop_and_invalid_nav_command_assert_zero(node):
    healthy(node)
    out = capture(node)
    msg = Twist()
    msg.linear.x = float('nan')
    node.on_cmd(msg)
    assert node.cmd_at == float('-inf')
    assert out[-1] == Twist()
    node.estop_at['aggregate'] -= 2.0
    node.on_tick()
    assert out[-1] == Twist()
    assert node.gate.state == 'FAULT_HOLD'


def test_proxy_cancel_blocks_all_output_and_goal_change_discards_command(node):
    healthy(node)
    out = capture(node)
    node.cmd.linear.x = 0.2
    node.cmd_at = time.monotonic()
    node.proxy.motion_authorized = False
    node.on_tick()
    assert out[-1] == Twist()
    node.on_goal_changed()
    assert node.cmd_at == float('-inf')
    assert out[-1] == Twist()


def test_driver_and_estop_request_topics_have_no_supervisor_publishers(node):
    publishers = dict(node.get_publisher_names_and_types_by_node(node.get_name(), ''))
    assert '/cmd_vel_detour' in publishers
    assert '/cmd_vel' not in publishers
    assert '/traymover_estop/auto_request' not in publishers
    assert '/traymover_estop/state' not in publishers


def test_projected_lidar_infinite_beams_are_unknown(node):
    msg = scan(node)
    msg.ranges = [float('inf')] * len(msg.ranges)
    assert not supervisor.scan_to_rays(msg, transform(node).transform, 10.0).rays


def test_brief_estop_between_timer_ticks_still_enters_hold(node):
    healthy(node)
    out = capture(node)
    node.on_estop('manual', Bool(data=True))
    node.on_estop('manual', Bool(data=False))
    node.on_tick()
    assert node.gate.state == 'HOLD_ESTOP'
    assert out[-1] == Twist()
    node.on_tick()
    assert node.gate.state == 'HOLD_ESTOP'
    assert out[-1] == Twist()


def test_auto_off_latched_free_state_needs_no_driver_heartbeat(node):
    from detour_gate import Config, DetourGate
    healthy(node)
    node.config = Config(enable_auto_estop=False)
    node.gate = DetourGate(node.config)
    out = capture(node)
    for key in ('manual', 'auto', 'aggregate'):
        node.estop_at[key] -= 100.0
    node.scans = {key: None for key in node.scans}
    node.localization_at = float('-inf')
    cmd = Twist()
    cmd.linear.x, cmd.linear.y, cmd.angular.z = 0.14, 0.01, 0.2
    node.on_cmd(cmd)
    node.on_tick()
    assert out[-1] == cmd
    assert node.gate.state == 'NORMAL'


def test_axial_depth_metric_keeps_camera_offset_out_of_guard(node):
    msg = scan(node)
    msg.angle_min, msg.angle_increment = math.pi / 4, 0.01
    msg.ranges = [1.2]
    tf = transform(node).transform
    tf.translation.x = 0.45
    result = supervisor.scan_to_rays(msg, tf, 10.0, allow_infinite=True)
    assert result.rays[0].safety_distance_m == pytest.approx(1.2 / math.sqrt(2))
    assert result.rays[0].origin_x == 0.45


def test_cached_depth_clearance_accounts_for_motion_since_acquisition(node):
    healthy(node)
    node.scan_acquisition_poses['realsense'] = (supervisor.Pose(-0.08, 0.0, -0.2), 0.45)
    saved = node.scans['realsense'].rays[180]
    now_ray = node.observation(time.monotonic()).realsense.rays[180]
    assert now_ray.safety_distance_m == pytest.approx(
        saved.safety_distance_m - 0.08 - 0.45 * 0.2)


def test_scan_waits_for_exact_time_tf_without_refreshing_acquisition(node):
    class Delayed:
        def lookup_transform(self, *args):
            raise LookupException('TF arrives later than scan')
    node.tf = Delayed()
    msg = scan(node, age=0.2)
    node.on_scan('lidar', msg)
    assert node.scans['lidar'] is None
    assert len(node.pending_scans['lidar']) == 1
    node.tf = TransformBuffer(node)
    node.consume_scans('lidar')
    assert node.scans['lidar'] is not None
    assert time.monotonic() - node.scans['lidar'].stamp >= 0.19
    assert not node.pending_scans['lidar']


def test_only_map_transform_failure_still_invalidates_tf(node):
    healthy(node)

    class MissingMap:
        def lookup_transform(self, target, *args):
            if target == 'map':
                raise LookupException('map to odom unavailable')
            return transform(node)

    node.tf = MissingMap()
    assert not node.observation(time.monotonic()).tf_healthy
    out = capture(node)
    node.on_tick()
    assert out[-1] == Twist()
    assert node.gate.state == 'FAULT_HOLD'
