"""Exercise option 18's real Auto EStop binary with synthetic depth only."""

import importlib.util
import os
from pathlib import Path
import subprocess
import time
import uuid

import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String


PKG = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    'depth_policy_launch',
    PKG / 'launch' / 'traymover_nav_lidar_realsense_detour.launch.py')
LAUNCH = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(LAUNCH)


class DepthPolicyHarness:
    def __init__(self):
        binary = Path(os.environ.get(
            'TRAYMOVER_AUTO_ESTOP_BINARY',
            PKG.parents[1] / 'install/traymover_robot_safety/lib/'
            'traymover_robot_safety/traymover_auto_estop'))
        if not binary.is_file():
            pytest.skip('Build/install traymover_robot_safety before this ROS test')
        self.context = Context()
        domain = 160 + os.getpid() % 50
        rclpy.init(context=self.context, domain_id=domain)
        prefix = '/option18_depth_policy_' + uuid.uuid4().hex
        self.node = rclpy.create_node('depth_policy_test', context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.requests = []
        self.reasons = []
        self.node.create_subscription(
            Bool, prefix + '/request',
            lambda msg: self.requests.append(msg.data), 100)
        self.node.create_subscription(
            String, prefix + '/reason',
            lambda msg: self.reasons.append(msg.data),
            QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.publisher = self.node.create_publisher(Image, prefix + '/depth', 10)
        args = [str(binary), '--ros-args']
        # Import the actual option 18 settings: no copied tuning constants.
        for name, value in LAUNCH.AUTO_ESTOP_OVERRIDES.items():
            args.extend(['-p', f'{name}:={value}'])
        args.extend([
            '-p', 'depth_topic:=' + prefix + '/depth',
            '-r', '/traymover_estop/auto_request:=' + prefix + '/request',
            '-r', '/traymover_estop/auto_reason:=' + prefix + '/reason'])
        self.process = subprocess.Popen(
            args, env={**os.environ, 'ROS_DOMAIN_ID': str(domain)},
            stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
        try:
            self.wait_for(lambda: self.publisher.get_subscription_count() > 0)
            # Acknowledged invalid input avoids relying on discovery timing.
            for _ in range(20):
                self.frame('invalid')
                if self.reasons and self.reasons[-1] == 'INVALID_FRAME':
                    break
            self.expect(True, 'INVALID_FRAME')
        except BaseException:
            self.close()
            raise

    def spin(self, duration=0.10):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            assert self.process.poll() is None, 'Auto EStop exited early'
            self.executor.spin_once(timeout_sec=0.01)

    def wait_for(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            self.spin(0.02)
        assert predicate(), (self.requests[-8:], self.reasons[-8:])

    def frame(self, pattern='clear'):
        image = Image(height=240, width=320, step=640, encoding='16UC1')
        data = bytearray(b'\xd0\x07' * (320 * 240))  # 2 m background
        for y in range(48, 204):
            for x in range(80, 240):
                roi_index = (y - 48) * 160 + x - 80
                near = (
                    (pattern == 'thin' and x == 160)
                    or (pattern == 'small' and 154 <= x < 166 and 120 <= y < 132)
                    or (pattern == 'obstacle' and 140 <= x < 180 and 106 <= y < 146))
                missing = (
                    pattern == 'blank'
                    or (pattern == 'low_valid' and roi_index % 100 >= 12)
                    or (pattern == 'severe' and roi_index % 100 >= 9))
                value = 0 if missing else (700 if near else 2000)
                offset = 2 * (y * 320 + x)
                data[offset] = value & 255
                data[offset + 1] = value >> 8
        image.data = data
        if pattern == 'invalid':
            image.encoding = 'unsupported'
        self.publisher.publish(image)
        self.spin()

    def expect(self, request, reason):
        self.wait_for(lambda: (
            bool(self.requests) and bool(self.reasons)
            and self.requests[-1] is request and self.reasons[-1] == reason))

    def clear(self):
        for _ in range(int(LAUNCH.AUTO_ESTOP_OVERRIDES['clear_frame_count'])):
            self.frame()
        self.expect(False, 'NONE')

    def close(self):
        self.process.terminate()
        try:
            self.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait(timeout=5)
        self.executor.shutdown()
        self.node.destroy_node()
        self.context.shutdown()


@pytest.fixture
def depth_policy():
    harness = DepthPolicyHarness()
    try:
        harness.clear()
        yield harness
    finally:
        harness.close()


def test_noise_ignored_but_persistent_obstacle_stops(depth_policy):
    start = len(depth_policy.requests)
    for pattern in ('thin', 'small'):
        for _ in range(5):
            depth_policy.frame(pattern)
    assert depth_policy.requests[start:] and not any(depth_policy.requests[start:])
    for _ in range(2):
        depth_policy.frame('obstacle')
        depth_policy.expect(False, 'NONE')
    depth_policy.frame('obstacle')
    depth_policy.expect(True, 'NEAR_OBSTACLE')
    for _ in range(11):
        depth_policy.frame()
        depth_policy.expect(True, 'NEAR_OBSTACLE')
    depth_policy.frame()
    depth_policy.expect(False, 'NONE')


def test_transient_low_valid_depth_is_debounced_but_faults_stop(depth_policy):
    start = len(depth_policy.requests)
    for _ in range(3):
        depth_policy.frame('low_valid')
        depth_policy.expect(False, 'LOW_VALID_DEPTH')
    assert depth_policy.requests[start:] and not any(depth_policy.requests[start:])
    depth_policy.frame('low_valid')
    depth_policy.expect(True, 'LOW_VALID_DEPTH')
    for pattern, reason in (
            ('severe', 'LOW_VALID_DEPTH'), ('blank', 'LOW_VALID_DEPTH'),
            ('invalid', 'INVALID_FRAME')):
        depth_policy.clear()
        depth_policy.frame(pattern)
        depth_policy.expect(True, reason)


def test_short_gap_allowed_but_complete_depth_outage_stops(depth_policy):
    start = len(depth_policy.requests)
    depth_policy.spin(1.0)
    assert depth_policy.requests[start:] and not any(depth_policy.requests[start:])
    depth_policy.expect(False, 'NONE')
    depth_policy.wait_for(
        lambda: depth_policy.reasons[-1] == 'DEPTH_TIMEOUT', timeout=1.5)
    depth_policy.expect(True, 'DEPTH_TIMEOUT')
    depth_policy.clear()
