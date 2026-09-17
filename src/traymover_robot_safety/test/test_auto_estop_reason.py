"""Exercise the real safety node with synthetic depth, without any hardware.

Every safety topic is remapped into an isolated test namespace. These checks
also lock the existing Bool confirmation and release behavior while testing
the additive reason topic.
"""

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


class SafetyNodeHarness:
    def __init__(self, heartbeat=20.0, prime_depth=True):
        workspace = Path(__file__).resolve().parents[3]
        binary = Path(os.environ.get(
            'TRAYMOVER_AUTO_ESTOP_BINARY',
            workspace / 'build/traymover_robot_safety/traymover_auto_estop'))
        if not binary.is_file():
            pytest.skip('Build traymover_robot_safety before running its ROS tests')
        self.context = Context()
        self.domain_id = 160 + os.getpid() % 50
        rclpy.init(context=self.context, domain_id=self.domain_id)
        self.prefix = '/reason_test_' + uuid.uuid4().hex
        self.node = rclpy.create_node(
            'reason_test', context=self.context, namespace=self.prefix)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.requests = []
        self.reasons = []
        self.reason_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(
            Bool, self.prefix + '/request',
            lambda msg: self.requests.append(msg.data), 10)
        self.node.create_subscription(
            String, self.prefix + '/reason',
            lambda msg: self.reasons.append(msg.data), self.reason_qos)
        self.depth_pub = self.node.create_publisher(
            Image, self.prefix + '/depth', 10)
        self.process = subprocess.Popen([
            str(binary), '--ros-args',
            '-p', 'depth_topic:=' + self.prefix + '/depth',
            '-p', 'clear_frame_count:=3',
            '-p', 'depth_timeout_sec:=0.8',
            '-p', 'heartbeat_rate_hz:=' + str(heartbeat),
            '-r', '/traymover_estop/auto_request:=' + self.prefix + '/request',
            '-r', '/traymover_estop/auto_reason:=' + self.prefix + '/reason',
        ], env={**os.environ, 'ROS_DOMAIN_ID': str(self.domain_id)},
            stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
        try:
            self.wait_for(lambda: self.depth_pub.get_subscription_count() > 0)
            if prime_depth:
                # Graph discovery may precede remote DDS writer matching. Use
                # an acknowledged frame before checking exact frame counts.
                for _ in range(30):
                    self.frame(invalid=True)
                    if self.reasons and self.reasons[-1] == 'INVALID_FRAME':
                        break
                self.expect(True, 'INVALID_FRAME')
        except Exception:
            self.close()
            raise

    def spin(self, duration=0.12):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            assert self.process.poll() is None, 'Safety test node exited early'
            self.executor.spin_once(timeout_sec=0.02)

    def wait_for(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            self.spin(0.02)
        assert predicate(), (self.requests[-5:], self.reasons[-5:])

    def frame(self, depth_mm=2000, valid_fraction=1.0, invalid=False):
        image = Image()
        image.height = 40
        image.width = 64
        image.step = image.width * 2
        image.encoding = '32FC1' if invalid else '16UC1'
        data = bytearray(image.step * image.height)
        # Distribute invalid pixels evenly, including the evaluator's ROI.
        for pixel in range(image.width * image.height):
            depth = depth_mm if pixel % 10 < int(valid_fraction * 10) else 0
            data[2 * pixel] = depth & 255
            data[2 * pixel + 1] = depth >> 8
        image.data = data
        self.depth_pub.publish(image)
        self.spin()

    def expect(self, requested, reason):
        self.wait_for(lambda: (
            bool(self.requests) and bool(self.reasons)
            and self.requests[-1] is requested and self.reasons[-1] == reason))

    def clear(self):
        for _ in range(3):
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
def safety_node():
    harness = SafetyNodeHarness()
    try:
        yield harness
    finally:
        harness.close()


def test_startup_reason_is_available_to_late_subscriber():
    harness = SafetyNodeHarness(heartbeat=0.1, prime_depth=False)
    try:
        harness.wait_for(lambda: harness.reasons == ['STARTUP'])
        late_reasons = []
        harness.node.create_subscription(
            String, harness.prefix + '/reason',
            lambda msg: late_reasons.append(msg.data), harness.reason_qos)
        harness.wait_for(lambda: late_reasons == ['STARTUP'])
    finally:
        harness.close()


def test_near_reason_survives_heartbeat_and_clear_confirmation(safety_node):
    safety_node.clear()
    safety_node.frame(depth_mm=700)
    safety_node.expect(False, 'NONE')
    safety_node.frame(depth_mm=700)
    safety_node.expect(True, 'NEAR_OBSTACLE')
    start = len(safety_node.reasons)
    safety_node.spin(0.2)
    assert len(safety_node.reasons) > start
    assert set(safety_node.reasons[start:]) == {'NEAR_OBSTACLE'}
    # Hysteresis still uses the unmodified 1.0 m release threshold.
    safety_node.frame(depth_mm=900)
    safety_node.expect(True, 'NEAR_OBSTACLE')
    for _ in range(2):
        safety_node.frame()
        safety_node.expect(True, 'NEAR_OBSTACLE')
    safety_node.frame()
    safety_node.expect(False, 'NONE')
    assert 'HEARTBEAT' not in safety_node.reasons


def test_low_valid_depth_replaces_near_reason_before_confirmation(safety_node):
    safety_node.frame(depth_mm=700)
    safety_node.expect(True, 'NEAR_OBSTACLE')
    safety_node.frame(valid_fraction=0.4)
    safety_node.expect(True, 'LOW_VALID_DEPTH')
    start = len(safety_node.reasons)
    safety_node.spin(0.15)
    assert set(safety_node.reasons[start:]) == {'LOW_VALID_DEPTH'}
    safety_node.frame(invalid=True)
    safety_node.expect(True, 'INVALID_FRAME')


def test_reason_does_not_change_low_valid_bool_confirmation(safety_node):
    safety_node.clear()
    safety_node.frame(valid_fraction=0.4)
    safety_node.expect(False, 'LOW_VALID_DEPTH')
    start = len(safety_node.reasons)
    safety_node.spin(0.15)
    assert set(safety_node.reasons[start:]) == {'LOW_VALID_DEPTH'}
    safety_node.frame(valid_fraction=0.4)
    safety_node.expect(True, 'LOW_VALID_DEPTH')
    safety_node.clear()


@pytest.mark.parametrize('frame_args,reason', [
    ({'invalid': True}, 'INVALID_FRAME'),
    ({'valid_fraction': 0.0}, 'LOW_VALID_DEPTH'),
])
def test_sensor_fault_latched_until_clear_confirmation(
        safety_node, frame_args, reason):
    safety_node.clear()
    safety_node.frame(**frame_args)
    safety_node.expect(True, reason)
    safety_node.spin(0.15)
    safety_node.expect(True, reason)
    for _ in range(2):
        safety_node.frame()
        safety_node.expect(True, reason)
    safety_node.frame()
    safety_node.expect(False, 'NONE')


def test_timeout_overrides_near_and_recovers_with_existing_hysteresis(safety_node):
    safety_node.frame(depth_mm=700)
    safety_node.expect(True, 'NEAR_OBSTACLE')
    safety_node.wait_for(lambda: safety_node.reasons[-1] == 'DEPTH_TIMEOUT')
    safety_node.expect(True, 'DEPTH_TIMEOUT')
    for _ in range(2):
        safety_node.frame()
        safety_node.expect(True, 'DEPTH_TIMEOUT')
    safety_node.frame()
    safety_node.expect(False, 'NONE')
