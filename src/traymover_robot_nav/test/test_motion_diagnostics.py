"""Passive motion logging tests with a fake node; never start a robot or publish."""

import csv
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace

import pytest


pytest.importorskip('rclpy')
from geometry_msgs.msg import TransformStamped, Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy  # noqa: E402
from std_msgs.msg import Bool, UInt8  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402


ROOT = Path(__file__).resolve().parents[3]


class FakeNode:
    """Only subscription and existing read-only lifecycle query APIs are available."""

    def __init__(self):
        self.subscriptions = {}

    def create_subscription(self, msg_type, topic, callback, qos):
        self.subscriptions[topic] = (msg_type, callback, qos)

    def create_client(self, *args):
        return SimpleNamespace(service_is_ready=lambda: False)

    def create_timer(self, period, callback, **kwargs):
        return SimpleNamespace(period=period, callback=callback)

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=0))

    def destroy_node(self):
        pass


@pytest.fixture
def monitor(tmp_path, monkeypatch):
    spec = importlib.util.spec_from_file_location(
        'motion_test_diagnostics', ROOT / 'scripts/ndt_diagnostics.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    node = FakeNode()
    steady_time = [100.0]
    monkeypatch.setattr(module.rclpy, 'create_node', lambda *args, **kwargs: node)
    monkeypatch.setattr(module.time, 'monotonic', lambda: steady_time[0])
    instance = module.NdtDiagnostics(SimpleNamespace(
        output_dir=str(tmp_path), interval_sec=5.0,
        use_sim_time='false', cloud_topic='/point_cloud_localization'))
    yield instance, node, steady_time, tmp_path
    instance.close()


def motion_records(directory):
    return [dict(field.split('=', 1) for field in line.split()[2:])
            for line in (directory / 'ndt_events.log').read_text().splitlines()
            if 'event=MOTION ' in line]


def test_missing_motion_topics_are_unknown_and_csv_schema_stays_stable(monitor):
    instance, _, _, directory = monitor
    instance.report()
    record, = motion_records(directory)
    for key in ('cmd_vel', 'odom_twist', 'estop', 'estop_auto', 'estop_manual', 'motor_enabled'):
        assert record[key + '_value'] == 'unknown'
        assert record[key + '_count'] == '0'
        assert record[key + '_age_sec'] == '-'
        assert record[key + '_last_nonzero_age_sec'] == '-'
    with (directory / 'ndt_metrics.csv').open() as stream:
        header, row = list(csv.reader(stream))
    assert header == [
        'wall_time', 'ros_time', 'elapsed_sec', 'event',
        'cloud_count', 'cloud_age_sec', 'cloud_points',
        'odom_count', 'odom_age_sec', 'pcl_pose_count',
        'pcl_pose_age_sec', 'initialpose_count', 'tf_count',
        'map_odom_age_sec', 'odom_base_age_sec', 'lifecycle_state',
    ]
    assert len(row) == len(header)


def test_commands_are_aggregated_and_zero_does_not_erase_last_nonzero(monitor):
    instance, node, steady_time, directory = monitor
    msg_type, callback, _ = node.subscriptions['/cmd_vel']
    assert msg_type is Twist
    before = (directory / 'ndt_events.log').read_text()
    command = Twist()
    command.linear.x = 0.2
    command.angular.z = -0.1
    for _ in range(1000):
        callback(command)
    steady_time[0] = 102.0
    callback(Twist())
    assert (directory / 'ndt_events.log').read_text() == before
    steady_time[0] = 105.0
    instance.report()
    record = motion_records(directory)[-1]
    assert record['cmd_vel_count'] == '1001'
    assert record['cmd_vel_interval_count'] == '1001'
    assert record['cmd_vel_nonzero_count'] == '1000'
    assert record['cmd_vel_interval_nonzero_count'] == '1000'
    assert record['cmd_vel_value'] == ','.join(['0.000000'] * 6)
    assert record['cmd_vel_age_sec'] == '3.000'
    assert record['cmd_vel_last_nonzero_age_sec'] == '5.000'
    steady_time[0] = 110.0
    instance.report()
    record = motion_records(directory)[-1]
    assert record['cmd_vel_count'] == '1001'
    assert record['cmd_vel_interval_count'] == '0'
    assert record['cmd_vel_interval_nonzero_count'] == '0'
    assert record['cmd_vel_age_sec'] == '8.000'


@pytest.mark.parametrize('key,topic', [
    ('estop', '/traymover_estop/state'),
    ('estop_auto', '/traymover_estop/auto_state'),
    ('estop_manual', '/traymover_estop/manual_state'),
])
def test_latched_estop_states_remain_distinct(monitor, key, topic):
    instance, node, _, directory = monitor
    msg_type, callback, qos = node.subscriptions[topic]
    assert msg_type is Bool
    assert qos.depth == 1
    assert qos.reliability == ReliabilityPolicy.RELIABLE
    assert qos.durability == DurabilityPolicy.TRANSIENT_LOCAL
    callback(Bool(data=True))
    instance.report()
    record = motion_records(directory)[-1]
    assert record[key + '_value'] == '1'
    assert record[key + '_nonzero_count'] == '1'
    for other in {'estop', 'estop_auto', 'estop_manual'} - {key}:
        assert record[other + '_value'] == 'unknown'
    callback(Bool(data=False))
    instance.report()
    record = motion_records(directory)[-1]
    assert record[key + '_value'] == '0'
    assert record[key + '_count'] == '2'
    assert record[key + '_interval_nonzero_count'] == '0'


def test_motor_enabled_preserves_raw_byte_without_claiming_measured_velocity(monitor):
    instance, node, _, directory = monitor
    msg_type, callback, _ = node.subscriptions['/motor_enabled']
    assert msg_type is UInt8
    callback(UInt8(data=3))
    instance.report()
    record = motion_records(directory)[-1]
    assert record['motor_enabled_value'] == '3'
    # These names contain speed but carry PWM, and are not measured wheel velocity.
    assert '/motor_status_speed_x' not in node.subscriptions
    assert '/motor_status_speed_th' not in node.subscriptions


def test_stm32_report_pairs_measured_twist_and_footprint_tf(monitor):
    instance, node, _, directory = monitor
    command = Twist()
    command.angular.z = 0.04
    node.subscriptions['/cmd_vel'][1](command)

    odom = Odometry()
    odom.header.frame_id, odom.child_frame_id = 'odom', 'base_footprint'
    odom.pose.pose.position.x = .34
    odom.pose.pose.orientation.z = math.sin(.3 / 2)
    odom.pose.pose.orientation.w = math.cos(.3 / 2)
    node.subscriptions['/odom'][1](odom)
    transform = TransformStamped()
    transform.header.frame_id, transform.child_frame_id = 'odom', 'base_footprint'
    node.subscriptions['/tf'][1](TFMessage(transforms=[transform]))

    instance.report()
    record, = motion_records(directory)
    assert record['cmd_vel_nonzero_count'] == '1'
    assert record['odom_twist_value'] == '0.000000,0.000000'
    assert record['odom_twist_count'] == '1'
    assert record['odom_twist_nonzero_count'] == '0'
    assert 'odom_pose_xyzyaw=0.340,0.000,0.000,0.300' in (
        directory / 'ndt_events.log').read_text()
    with (directory / 'ndt_metrics.csv').open() as stream:
        row, = csv.DictReader(stream)
    assert float(row['odom_base_age_sec']) >= 0.0
