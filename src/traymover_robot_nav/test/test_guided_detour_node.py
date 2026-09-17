"""Synthetic ROS adapter tests. No real chassis, sensors or motion are started."""
import math
import os
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import TransformStamped, Twist  # noqa: E402
from rcl_interfaces.msg import ParameterType, ParameterValue  # noqa: E402
from sensor_msgs.msg import BatteryState, Imu, LaserScan  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402
from tf2_ros import LookupException  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import guided_detour_node as guided  # noqa: E402


@pytest.fixture
def node(monkeypatch):
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    rclpy.init(args=[], domain_id=80 + os.getpid() % 70)
    instance = guided.GuidedDetourNode()
    yield instance
    instance.destroy_node()
    rclpy.shutdown()


def stamp(node, age=0.0):
    return rclpy.time.Time(nanoseconds=(
        node.get_clock().now().nanoseconds - int(age * 1e9))).to_msg()


def imu(node, angle=0.0, age=0.0):
    msg = Imu()
    msg.header.stamp = stamp(node, age)
    msg.header.frame_id = 'imu_link'
    msg.orientation.z = math.sin(angle / 2)
    msg.orientation.w = math.cos(angle / 2)
    return msg


def scan(node, age=0.0):
    msg = LaserScan()
    msg.header.stamp = stamp(node, age)
    msg.header.frame_id = 'laser'
    msg.angle_min, msg.angle_increment = -math.pi, math.pi / 180
    msg.range_min, msg.range_max = 0.2, 8.0
    msg.ranges = [4.0] * 361
    return msg


class TransformBuffer:
    def __init__(self):
        self.calls = []

    def lookup_transform(self, *args):
        self.calls.append(args)
        msg = TransformStamped()
        msg.transform.rotation.w = 1.0
        return msg


def capture(node):
    messages = []
    node.output = SimpleNamespace(publish=messages.append)
    return messages


def healthy(node):
    node.tf = TransformBuffer()
    node.on_imu(imu(node))
    for key in node.scans:
        node.on_scan(key, scan(node))
    battery = BatteryState()
    battery.header.stamp = stamp(node)
    node.on_battery(battery)
    for key in ('aggregate', 'manual', 'auto', 'request'):
        node.on_estop(key, Bool(data=False))
    node.on_estop('reason', String(data='NONE'))
    now = time.monotonic()
    node.driver_parameter_at = node.cm_state_at = node.graph_at = now
    node.driver_auto_enabled = node.cm_active = True
    node.graph_fault = ''
    node.on_output(Twist())
    node.manual_service = SimpleNamespace(service_is_ready=lambda: True)
    return node.observation()


def test_preflight_fails_without_critical_inputs_and_never_moves(node):
    output = capture(node)
    checks = node.preflight()
    assert checks['IMU']['status'] == 'FAIL'
    assert checks['Auto EStop']['status'] == 'FAIL'
    assert checks['Collision Monitor']['status'] == 'FAIL'
    node.on_tick()
    assert all(guided.twist_is_zero(msg) for msg in output)
    assert not node.core.arm(time.monotonic())


def test_preflight_requires_serial_parameter_output_and_services(node):
    obs = healthy(node)
    assert all(check['status'] == 'PASS' for check in node.preflight().values())
    assert obs.pose is None and not obs.localization_healthy and not obs.nav_goal_active
    assert obs.tf_healthy and obs.driver_fresh and obs.collision_monitor_fresh
    node.driver_auto_enabled = False
    assert not node.observation().driver_fresh
    assert node.preflight()['Driver Auto EStop parameter']['status'] == 'FAIL'
    node.driver_auto_enabled = True
    node.cm_output_at -= 2.0
    assert not node.observation().collision_monitor_fresh
    node.battery_at -= 2.0
    assert not node.observation().driver_fresh


def test_sensor_tf_uses_acquisition_time_and_base_only(node):
    healthy(node)
    for target, source, requested in node.tf.calls:
        assert target == 'base_link'
        assert source == 'laser'
        assert requested.nanoseconds > 0


def test_missing_tf_or_nearby_imu_keeps_scan_unknown(node):
    node.tf = TransformBuffer()
    node.on_scan('lidar', scan(node))
    assert node.scans['lidar'] is None
    node.on_imu(imu(node))

    class Missing:
        def lookup_transform(self, *args):
            raise LookupException('synthetic missing sensor frame')

    node.tf = Missing()
    node.consume_scans('lidar')
    assert node.scans['lidar'] is None
    assert not node.observation().tf_healthy


def test_stale_and_replayed_headers_are_not_refreshed(node):
    healthy(node)
    output = capture(node)
    msg = scan(node)
    node.on_scan('lidar', msg)
    node.on_scan('lidar', msg)
    assert node.scans['lidar'] is None
    msg = imu(node)
    node.on_imu(msg)
    node.on_imu(msg)
    assert node.imu_yaw is None
    assert guided.twist_is_zero(output[-1])


def test_imu_invalid_during_motion_latches_fault_and_immediate_zero(node):
    healthy(node)
    output = capture(node)
    node.last_command = guided.Velocity(0.0, 0.15)
    invalid = imu(node)
    invalid.orientation_covariance[0] = -1.0
    node.on_imu(invalid)
    assert node.adapter_fault == 'invalid IMU sample during motion'
    assert guided.twist_is_zero(output[-1])
    node.on_imu(imu(node))
    assert node.observation().health_fault  # A good sample cannot resume it.


def test_retained_scan_rotates_and_axial_depth_loses_motion_margin():
    saved = guided.Scan(10.0, (guided.Ray(0.4, 0.0, 0.0, 2.0, True, 1.8),))
    adjusted = guided.adjusted_scan(saved, 0.0, math.pi / 2, 0.05)
    ray = adjusted.rays[0]
    assert adjusted.stamp == 10.0
    assert ray.origin_x == pytest.approx(0.0, abs=1e-9)
    assert ray.origin_y == pytest.approx(-0.4)
    assert ray.angle == pytest.approx(-math.pi / 2)
    assert ray.distance == pytest.approx(1.95)
    assert ray.safety_distance_m == pytest.approx(1.8 - 0.05 - 0.4 * math.pi / 2)


def test_brief_estop_pulse_is_latched_between_ticks(node):
    healthy(node)
    output = capture(node)
    node.on_estop('auto', Bool(data=True))
    node.on_estop('auto', Bool(data=False))
    node.on_tick()
    assert node.core.state == 'HOLD_ESTOP'
    assert guided.twist_is_zero(output[-1])


def test_brief_auto_pulse_retains_automatic_source_at_core_boundary(node):
    healthy(node)
    observations = []
    node.core.tick = lambda now, obs: observations.append(obs) or guided.Velocity()
    node.on_estop('request', Bool(data=True))
    node.on_estop('request', Bool(data=False))
    node.on_tick()
    assert observations[-1].auto_estop
    assert not observations[-1].manual_estop
    node.on_tick()
    assert not observations[-1].auto_estop


def test_auto_request_heartbeat_is_required(node):
    healthy(node)
    node.estop_at['request'] -= 2.0
    assert not node.observation().estop_fresh
    assert node.preflight()['Auto request heartbeat']['status'] == 'FAIL'


def test_estop_software_timestamps_do_not_reuse_an_old_zero(node):
    healthy(node)
    assert node.current_estop_event is None
    node.last_command = guided.Velocity(0.02, 0.0)
    node.on_estop('request', Bool(data=True))
    event = node.current_estop_event
    assert event['command_was_nonzero']
    assert event['auto_command_was_nonzero']
    assert event['auto_request_command_was_nonzero']
    assert 't_output_zero' not in event
    node.on_output(Twist())
    assert 't_first_output_zero' in event
    assert 't_output_zero' not in event  # Still no post-aggregate proof.
    node.on_estop('auto', Bool(data=True))
    assert event['auto_command_was_nonzero']
    assert not event['auto_state_command_was_nonzero']
    node.on_estop('aggregate', Bool(data=True))
    node.on_output(Twist())
    assert event['t_output_zero'] >= event['t_aggregate_estop'] >= event['t_auto_request']
    assert node.snapshot()['event_times'] == event


def test_manual_aggregate_then_later_auto_cannot_prove_active_auto_interruption(node):
    healthy(node)
    node.last_command = guided.Velocity(0.02, 0.0)
    # Aggregate can arrive before the manual-state callback on separate topics.
    node.on_estop('aggregate', Bool(data=True))
    event = node.current_estop_event
    assert event['command_was_nonzero']
    assert event['aggregate_estop_command_was_nonzero']
    assert not event['auto_command_was_nonzero']
    node.on_estop('manual', Bool(data=True))
    node.on_estop('request', Bool(data=True))
    node.on_estop('auto', Bool(data=True))
    assert not event['auto_command_was_nonzero']
    assert not event['auto_request_command_was_nonzero']
    assert not event['auto_state_command_was_nonzero']


def test_auto_state_before_request_preserves_active_auto_interruption(node):
    healthy(node)
    node.last_command = guided.Velocity(0.02, 0.0)
    node.on_estop('auto', Bool(data=True))
    event = node.current_estop_event
    node.on_estop('request', Bool(data=True))
    node.on_estop('aggregate', Bool(data=True))
    assert event['auto_command_was_nonzero']
    assert event['auto_state_command_was_nonzero']
    assert not event['auto_request_command_was_nonzero']


def test_nonzero_final_command_during_estop_is_a_failure(node):
    healthy(node)
    node.on_estop('aggregate', Bool(data=True))
    msg = Twist()
    msg.linear.x = 0.02
    node.on_output(msg)
    assert node.adapter_fault
    assert 't_nonzero_after_aggregate' in node.current_estop_event
    assert node.observation().health_fault


def test_competing_publisher_or_missing_monitor_blocks_motion(node):
    healthy(node)

    def publishers(topic):
        names = ['collision_monitor', 'teleop'] if topic == '/cmd_vel' else ['guided_detour_test']
        return [SimpleNamespace(node_namespace='/', node_name=name) for name in names]

    node.get_publishers_info_by_topic = publishers
    node.refresh_graph(time.monotonic())
    assert 'teleop' in node.observation().health_fault
    assert node.preflight()['Velocity ownership']['status'] == 'FAIL'


def test_health_services_verify_boolean_enabled_and_active(node):
    healthy(node)
    now = time.monotonic()
    parameter = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=False)
    future = SimpleNamespace(done=lambda: True, result=lambda: SimpleNamespace(values=[parameter]))
    node.service_futures['driver'] = (future, now)
    node.poll_services()
    assert not node.driver_auto_enabled


def test_manual_service_requests_stop_only_and_abort_repeats_zero(node):
    healthy(node)
    output = capture(node)
    requests = []
    node.manual_service = SimpleNamespace(service_is_ready=lambda: True,
                                          call_async=lambda request: requests.append(request))
    node.assert_manual_estop()
    assert len(requests) == 1 and requests[0].data
    node.close_zero(0.06)
    assert len(output) >= 3
    assert all(guided.twist_is_zero(msg) for msg in output)
    assert len(requests) == 1  # Shutdown never releases EStop.


def test_guided_node_cannot_bypass_collision_monitor(node):
    publishers = dict(node.get_publisher_names_and_types_by_node(node.get_name(), ''))
    assert '/cmd_vel_detour_test' in publishers
    assert '/cmd_vel' not in publishers
    assert '/traymover_estop/auto_request' not in publishers
