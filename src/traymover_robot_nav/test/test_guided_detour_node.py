"""Synthetic ROS tests for Option 19's minimal depth/odom adapter."""
import math
import os
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip('rclpy')
from nav_msgs.msg import Odometry  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import guided_detour_node as option19  # noqa: E402


@pytest.fixture
def node(monkeypatch):
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    rclpy.init(args=[], domain_id=80 + os.getpid() % 70)
    instance = option19.LineReturnDetourNode()
    yield instance
    instance.destroy_node()
    rclpy.shutdown()


def stamp(node, age=0.0):
    nanoseconds = node.get_clock().now().nanoseconds - int(age * 1e9)
    return rclpy.time.Time(nanoseconds=nanoseconds).to_msg()


def odom(node, x=0.0, y=0.0, angle=0.0, age=0.0):
    message = Odometry()
    message.header.stamp = stamp(node, age)
    message.pose.pose.position.x = x
    message.pose.pose.position.y = y
    message.pose.pose.orientation.z = math.sin(angle / 2.0)
    message.pose.pose.orientation.w = math.cos(angle / 2.0)
    return message


def scan(node, obstacle=False, age=0.0):
    message = LaserScan()
    message.header.stamp = stamp(node, age)
    message.header.frame_id = 'base_link'
    message.angle_min = -math.pi
    message.angle_increment = math.pi / 180.0
    message.range_min, message.range_max = 0.3, 8.0
    message.ranges = [math.inf] * 361
    if obstacle:
        for index in range(176, 185):
            message.ranges[index] = 1.05
    return message


def capture(node):
    messages = []
    node.output = SimpleNamespace(publish=messages.append)
    return messages


def feed(node, depth_obstacle=False, x=0.0, y=0.0, angle=0.0):
    depth = scan(node, depth_obstacle)
    depth.header.frame_id = 'camera_depth_frame'
    node.on_depth_scan(depth)
    node.on_odom(odom(node, x, y, angle))


def test_enter_request_waits_only_for_first_depth_and_odom_samples(node):
    output = capture(node)
    assert node.start_motion()
    node.on_tick()
    assert option19.twist_is_zero(output[-1])
    assert node.core.original_line is None

    feed(node, x=1.0, y=2.0, angle=0.3)
    node.on_tick()
    assert output[-1].linear.x == pytest.approx(node.config.forward_speed)
    assert node.core.original_line == option19.Pose(1.0, 2.0, 0.3)


def test_depth_scan_uses_finite_hits_and_infinite_clear_rays(node):
    depth = scan(node, obstacle=True)
    depth.header.frame_id = 'camera_depth_frame'
    node.on_depth_scan(depth)
    assert node.depth_scan is not None
    hits = [ray for ray in node.depth_scan.rays if ray.hit]
    clear = [ray for ray in node.depth_scan.rays if not ray.hit]
    assert len(hits) == 9
    assert hits[0].distance == pytest.approx(1.05)
    assert clear and all(ray.distance == 8.0 for ray in clear)


def test_depth_camera_alone_triggers_first_obstacle(node):
    output = capture(node)
    node.start_motion()
    feed(node)
    node.on_tick()
    assert output[-1].linear.x > 0.0
    for unused in range(node.config.obstacle_confirm_frames):
        time.sleep(0.001)
        feed(node, depth_obstacle=True)
        node.on_tick()
    assert option19.twist_is_zero(output[-1])
    assert node.core.state == 'WAITING'


def test_stale_odom_stops_after_start(node):
    output = capture(node)
    node.start_motion()
    feed(node)
    node.on_tick()
    node.odom_at -= node.config.odom_timeout_sec + 0.1
    depth = scan(node)
    depth.header.frame_id = 'camera_depth_frame'
    node.on_depth_scan(depth)
    node.on_tick()
    assert option19.twist_is_zero(output[-1])
    assert node.core.state == 'FAULT_HOLD'


def test_manual_or_aggregate_estop_stops_immediately(node):
    output = capture(node)
    node.start_motion()
    feed(node)
    node.on_tick()
    node.on_aggregate_estop(Bool(data=True))
    assert option19.twist_is_zero(output[-1])
    node.on_tick()
    assert node.core.state == 'HOLD_ESTOP'


def test_operator_manual_stop_only_requests_latched_stop(node):
    requests = []
    node.manual_service = SimpleNamespace(
        service_is_ready=lambda: True,
        call_async=lambda request: requests.append(request) or SimpleNamespace(),
    )
    node.start_motion()
    feed(node)
    node.on_tick()
    node.assert_manual_estop()
    assert requests and requests[0].data is True
    assert node.core.state == 'ABORTED'


def test_close_repeats_zero(node, monkeypatch):
    output = capture(node)
    monkeypatch.setattr(option19.rclpy, 'spin_once', lambda *args, **kwargs: None)
    node.close_zero(duration=0.01)
    assert output and all(option19.twist_is_zero(message) for message in output)
