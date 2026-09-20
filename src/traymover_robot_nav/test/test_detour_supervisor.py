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
        self.forced_handbacks = []

    def tick(self):
        pass

    def notify_handback(self, force_replan=False):
        self.handbacks += 1
        self.forced_handbacks.append(force_replan)
        if force_replan:
            self.motion_authorized = False

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


def simple_mode(node):
    node.config = supervisor.replace(node.config, simple_detour=True)
    node.gate = supervisor.DetourGate(node.config)


def test_simple_navigation_accepts_local_scans_without_odom_or_map_tf(node):
    simple_mode(node)
    healthy(node)

    class LocalOnly:
        def lookup_transform(self, target, *args):
            if target != 'base_link':
                raise LookupException('dynamic TF not available at scan stamp')
            return transform(node)

    node.tf = LocalOnly()
    for key in node.scans:
        node.on_scan(key, scan(node))
        assert node.scans[key] is not None
        assert not node.pending_scans[key]
    node.localization_at = float('-inf')
    obs = node.observation(time.monotonic())
    assert obs.lidar is not None and obs.realsense is not None
    assert not obs.tf_healthy
    cmd = Twist()
    cmd.linear.x, cmd.linear.y, cmd.angular.z = 0.15, 0.02, 0.12
    node.on_cmd(cmd)
    out = capture(node)
    node.on_tick()
    assert node.gate.state == 'NORMAL'
    assert out[-1] == cmd


def test_simple_detour_motion_still_requires_current_odom(node):
    simple_mode(node)
    healthy(node)
    node.gate.state = 'TURN_1'

    class LocalOnly:
        def lookup_transform(self, target, *args):
            if target != 'base_link':
                raise LookupException('odom unavailable')
            return transform(node)

    node.tf = LocalOnly()
    out = capture(node)
    node.on_tick()
    assert out[-1] == Twist()
    assert node.gate.state == 'FAULT_HOLD'


def test_simple_scans_keep_acquisition_age_and_missing_sensor_stops(node):
    simple_mode(node)
    healthy(node)
    msg = scan(node, age=0.3)
    node.scan_headers.pop('lidar')
    node.on_scan('lidar', msg)
    assert time.monotonic() - node.scans['lidar'].stamp >= 0.29
    node.scans['realsense'] = supervisor.replace(
        node.scans['realsense'], stamp=time.monotonic() - 2.0)
    out = capture(node)
    node.on_tick()
    assert out[-1] == Twist()
    assert node.gate.state == 'FAULT_HOLD'
    assert 'stale' in node.scan_problems['realsense']


def test_scan_diagnostics_distinguish_invalid_stamp_frame_tf_and_empty(node):
    simple_mode(node)
    healthy(node)
    msg = scan(node)
    msg.header.frame_id = ''
    node.on_scan('lidar', msg)
    assert 'frame' in node.scan_problems['lidar']
    assert node.scans['lidar'] is None
    msg = scan(node)
    msg.header.stamp.sec = 0
    msg.header.stamp.nanosec = 0
    node.on_scan('lidar', msg)
    assert 'stamp' in node.scan_problems['lidar']
    msg = scan(node)
    msg.ranges = [float('nan')] * len(msg.ranges)
    node.on_scan('lidar', msg)
    assert 'empty' in node.scan_problems['lidar']
    assert node.scans['lidar'] is None

    class Missing:
        def lookup_transform(self, *args):
            raise LookupException('sensor extrinsics absent')

    node.tf = Missing()
    node.on_scan('lidar', scan(node))
    assert 'TF' in node.scan_problems['lidar']


def test_simple_mode_current_odom_health_does_not_require_map_tf(node):
    simple_mode(node)
    healthy(node)

    class LatestOdomOnly:
        def lookup_transform(self, target, source, stamp):
            if target == 'map' or (target == 'odom' and stamp.nanoseconds != 0):
                raise LookupException('no scan-time or map transform')
            return transform(node)

    node.tf = LatestOdomOnly()
    for key in node.scans:
        node.on_scan(key, scan(node))
        assert key not in node.scan_acquisition_poses
    node.localization_at = float('-inf')
    obs = node.observation(time.monotonic())
    assert obs.tf_healthy
    assert not obs.localization_healthy
    assert obs.lidar.rays


def test_simple_mode_compensates_cached_local_rays_when_scan_pose_available(node):
    simple_mode(node)
    healthy(node)
    node.scan_acquisition_poses['realsense'] = (supervisor.Pose(-0.08, 0.0, 0.0), 0.45)
    saved = node.scans['realsense'].rays[180]
    ray = node.observation(time.monotonic()).realsense.rays[180]
    assert ray.origin_x == pytest.approx(saved.origin_x - 0.08)
    assert ray.safety_distance_m == pytest.approx(saved.safety_distance_m - 0.08)


def test_scan_diagnostics_are_throttled_per_sensor(node, monkeypatch):
    messages = []
    monkeypatch.setattr(node, 'get_logger', lambda: SimpleNamespace(warning=messages.append))
    node.scan_problem('lidar', 'TF absent')
    node.scan_problem('lidar', 'TF still absent')
    node.scan_problem('realsense', 'empty scan')
    assert len(messages) == 2
    assert node.scan_problems['lidar'] == 'TF still absent'


def test_simple_mode_does_not_interrupt_nav_for_separate_ndt_sample(node):
    simple_mode(node)
    healthy(node)
    out = capture(node)
    node.on_localization(PoseWithCovarianceStamped())
    assert not out
    assert node.localization_at == float('-inf')


@pytest.mark.parametrize('key', ['perimeter', 'lidar', 'realsense'])
@pytest.mark.parametrize('fault', ['empty', 'stamp'])
def test_simple_optional_sensor_fault_cannot_inject_stop_pulses(node, key, fault):
    simple_mode(node)
    healthy(node)
    out = capture(node)
    message = scan(node)
    if fault == 'empty':
        message.ranges = [float('nan')] * len(message.ranges)
    else:
        message.header.stamp.sec = 0
        message.header.stamp.nanosec = 0
    node.on_scan(key, message)
    assert node.scans[key] is None
    assert key in node.scan_problems
    if key in ('perimeter', 'lidar'):
        assert out == []
        command = Twist()
        command.linear.x = 0.35
        node.on_cmd(command)
        node.on_tick()
        assert out[-1] == command
    else:
        assert out == [Twist()]


@pytest.mark.parametrize('key', ['lidar', 'realsense'])
@pytest.mark.parametrize('fault', ['empty', 'stamp'])
def test_simple_maneuver_camera_fault_stops_lidar_is_optional(node, key, fault):
    from detour_core import Candidate, Scene

    simple_mode(node)
    healthy(node)
    now = time.monotonic()
    obs = node.observation(now)
    node.gate._candidate = Candidate('RIGHT', -math.pi / 4, 2.2, 4.0)
    node.gate._start_phase('TURN_1', now, obs, Scene(obs, node.config, now))
    out = capture(node)
    message = scan(node)
    if fault == 'empty':
        message.ranges = [float('nan')] * len(message.ranges)
    else:
        message.header.stamp.sec = 0
        message.header.stamp.nanosec = 0
    node.on_scan(key, message)
    if key == 'lidar':
        assert out == []
        assert not node.scan_required(key)
        node.on_tick()
        assert node.gate.state == 'TURN_1'
        assert out[-1].linear.x == 0.0 and out[-1].angular.z < 0.0
        return
    assert out == [Twist()]
    node.on_tick()
    assert out[-1] == Twist()
    assert node.gate.state == 'FAULT_HOLD'


def test_simple_stale_lidar_preserves_nav_command_with_healthy_camera(node):
    simple_mode(node)
    healthy(node)
    node.scans['lidar'] = supervisor.replace(
        node.scans['lidar'], stamp=time.monotonic() - 3.0)
    out = capture(node)
    command = Twist()
    command.linear.x, command.angular.z = 0.25, 0.15
    node.on_cmd(command)
    node.on_tick()
    assert node.gate.state == 'NORMAL'
    assert out[-1] == command
    assert 'stale' in node.scan_problems['lidar']


def test_strict_malformed_lidar_still_stops_immediately(node):
    healthy(node)
    out = capture(node)
    message = scan(node)
    message.header.frame_id = ''
    node.on_scan('lidar', message)
    assert out == [Twist()]


def test_simple_low_valid_reason_waits_for_estop_confirmation(node):
    simple_mode(node)
    healthy(node)
    out = capture(node)
    command = Twist()
    command.linear.x = 0.25
    node.on_cmd(command)
    node.on_estop('reason', SimpleNamespace(data='LOW_VALID_DEPTH'))
    assert out == []
    assert node.pending_depth_fault is None
    node.on_tick()
    assert node.gate.state == 'NORMAL'
    assert out[-1] == command
    node.on_estop('auto', Bool(data=True))
    assert out[-1] == Twist()
    node.on_tick()
    assert out[-1] == Twist()
    assert node.gate.state != 'NORMAL'


@pytest.mark.parametrize('simple, reason', [
    (False, 'LOW_VALID_DEPTH'), (True, 'INVALID_FRAME'), (True, 'DEPTH_TIMEOUT')])
def test_depth_fault_reasons_still_stop_immediately(node, simple, reason):
    if simple:
        simple_mode(node)
    healthy(node)
    out = capture(node)
    node.on_estop('reason', SimpleNamespace(data=reason))
    assert out == [Twist()]
    assert node.pending_depth_fault == reason
    node.on_tick()
    assert node.gate.state == 'FAULT_HOLD'
    assert out[-1] == Twist()


def sampled_odom(node, age=0.6):
    simple_mode(node)
    healthy(node)
    sample = transform(node)
    sample.header.stamp = rclpy.time.Time(
        nanoseconds=node.get_clock().now().nanoseconds - int(age * 1e9)).to_msg()
    node.tf = SimpleNamespace(lookup_transform=lambda *args: sample)
    return sample


def advance_odom(sample, seconds, x=0.0, angle=0.0):
    sample.header.stamp = rclpy.time.Time(
        nanoseconds=rclpy.time.Time.from_msg(sample.header.stamp).nanoseconds
        + int(seconds * 1e9)).to_msg()
    sample.transform.translation.x = x
    sample.transform.rotation.z = math.sin(angle / 2)
    sample.transform.rotation.w = math.cos(angle / 2)


def test_repeated_tf_polls_use_actual_sample_interval_for_motion(node):
    sample = sampled_odom(node)
    assert node.observation(time.monotonic()).tf_healthy
    baseline = node.last_pose_sample
    for _ in range(6):
        assert node.observation(time.monotonic()).tf_healthy
        assert node.last_pose_sample == baseline
    advance_odom(sample, 0.3, x=0.20, angle=0.30)
    assert node.observation(time.monotonic()).tf_healthy
    assert node.last_pose_sample != baseline


@pytest.mark.parametrize('x, angle', [(0.8, 0.0), (0.0, 1.0)])
def test_actual_odom_jump_still_faults(node, x, angle):
    sample = sampled_odom(node)
    assert node.observation(time.monotonic()).tf_healthy
    advance_odom(sample, 0.3, x=x, angle=angle)
    assert not node.observation(time.monotonic()).tf_healthy
    assert node.pose_fault_until > time.monotonic()


@pytest.mark.parametrize('seconds, x', [(-0.1, 0.0), (0.0, 0.01)])
def test_backward_or_inconsistent_tf_stamp_cannot_update_baseline(node, seconds, x):
    sample = sampled_odom(node)
    assert node.observation(time.monotonic()).tf_healthy
    baseline = node.last_pose_sample
    advance_odom(sample, seconds, x=x)
    assert not node.observation(time.monotonic()).tf_healthy
    assert node.last_pose_sample == baseline


def test_repeated_stale_tf_stops_maneuver(node):
    sampled_odom(node, age=2.0)
    node.gate.state = 'TURN_1'
    out = capture(node)
    node.on_tick()
    assert node.gate.state == 'FAULT_HOLD'
    assert out[-1] == Twist()


def test_completed_detour_replans_once_and_requires_new_nav_command(node, monkeypatch):
    simple_mode(node)
    healthy(node)
    node.gate.state = 'RECOVER_TO_NAV'
    node.gate.recovery_requested = True
    node.gate.force_replan_requested = True
    monkeypatch.setattr(node.gate, 'tick', lambda now, obs:
                        obs.nav_cmd if node.gate.state == 'NORMAL' else supervisor.Velocity())
    out = capture(node)
    command = Twist()
    command.linear.x = 0.2
    node.on_cmd(command)
    node.on_tick()
    assert node.proxy.forced_handbacks == [True]
    assert node.gate.state == 'RECOVER_TO_NAV'
    assert out[-1] == Twist()
    node.on_cmd(command)  # Old backend can still publish while cancellation finishes.
    node.on_tick()
    assert node.proxy.handbacks == 1
    assert out[-1] == Twist()
    node.proxy.motion_authorized = True  # Replacement goal accepted.
    node.on_tick()
    assert node.gate.state == 'NORMAL'
    assert out[-1] == Twist()
    node.on_tick()
    assert out[-1] == Twist()
    node.on_cmd(command)
    node.on_tick()
    assert out[-1] == command


def test_public_cancel_during_replan_discards_pending_recovery(node, monkeypatch):
    simple_mode(node)
    healthy(node)
    node.gate.state = 'RECOVER_TO_NAV'
    node.gate.recovery_requested = True
    node.gate.force_replan_requested = True
    monkeypatch.setattr(node.gate, 'tick', lambda now, obs: supervisor.Velocity())
    out = capture(node)
    node.on_tick()
    assert node.handback_pending
    node.proxy.has_active_goal = False
    node.on_goal_changed()
    node.on_tick()
    assert not node.handback_pending
    assert not node.gate.force_replan_requested
    assert node.proxy.handbacks == 1
    assert out[-1] == Twist()


def test_fault_recovery_does_not_force_restart_navigation(node, monkeypatch):
    simple_mode(node)
    healthy(node)
    node.gate.state = 'RECOVER_TO_NAV'
    node.gate.recovery_requested = True
    monkeypatch.setattr(node.gate, 'tick', lambda now, obs: supervisor.Velocity())
    node.on_tick()
    assert node.proxy.forced_handbacks == [False]
    assert node.gate.state == 'NORMAL'
