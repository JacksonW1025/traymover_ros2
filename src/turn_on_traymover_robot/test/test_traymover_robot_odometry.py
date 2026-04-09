#!/usr/bin/env python3
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest
import rclpy

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from turn_on_traymover_robot import traymover_robot as robot


def make_feedback_frame(
    *,
    left_encoder=0,
    right_encoder=0,
    speedx_mm_s=0,
    speedth_millirad_s=0,
    msg_id=robot.MSG_ID_GET_MOTOR_DATA,
    status=robot.STATUS_SUCCESS,
):
    frame = bytearray(robot.FRAME_SIZE)
    frame[0:2] = robot.FRAME_HEADER
    frame[2] = robot.FRAME_LENGTH
    frame[3] = msg_id
    frame[4] = status
    frame[11:13] = int(left_encoder).to_bytes(2, 'big', signed=True)
    frame[13:15] = int(right_encoder).to_bytes(2, 'big', signed=True)
    frame[27] = 1
    frame[28] = 0
    frame[29:31] = int(speedx_mm_s).to_bytes(2, 'big', signed=True)
    frame[31:33] = int(speedth_millirad_s).to_bytes(2, 'big', signed=True)
    frame[33] = 2
    frame[35:37] = (42).to_bytes(2, 'big', signed=False)
    frame[37] = robot.compute_checksum(frame)
    frame[38:40] = robot.FRAME_TAIL
    return bytes(frame)


class FakeSerial:
    def __init__(self, feedback_frame):
        self.feedback_frame = feedback_frame
        self.read_buffer = bytearray()
        self.writes = []
        self.is_open = True

    @property
    def in_waiting(self):
        return len(self.read_buffer)

    def write(self, data):
        self.writes.append(bytes(data))
        self.read_buffer.extend(self.feedback_frame)
        return len(data)

    def read(self, size=1):
        if not self.read_buffer or size <= 0:
            return b''
        size = min(size, len(self.read_buffer))
        data = bytes(self.read_buffer[:size])
        del self.read_buffer[:size]
        return data

    def reset_input_buffer(self):
        self.read_buffer.clear()

    def reset_output_buffer(self):
        return None

    def close(self):
        self.is_open = False


@pytest.fixture(scope='module', autouse=True)
def rclpy_lifecycle():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_extract_next_valid_frame_resynchronizes_after_bad_data():
    valid_frame = make_feedback_frame(left_encoder=10, right_encoder=10)
    bad_frame = bytearray(valid_frame)
    bad_frame[37] ^= 0x01

    buffer = bytearray(b'\x00\x01\x02')
    buffer.extend(bad_frame)
    buffer.extend(valid_frame)

    extracted = robot.extract_next_valid_frame(buffer)

    assert extracted == valid_frame
    assert buffer == bytearray()


def test_parse_motor_feedback_frame_converts_units():
    frame = make_feedback_frame(
        left_encoder=-120,
        right_encoder=240,
        speedx_mm_s=350,
        speedth_millirad_s=-750,
    )

    feedback = robot.parse_motor_feedback_frame(frame)

    assert feedback.left_encoder_ticks == -120
    assert feedback.right_encoder_ticks == 240
    assert feedback.linear_velocity_mps == pytest.approx(0.35)
    assert feedback.angular_velocity_rads == pytest.approx(-0.75)


def test_integrate_diff_drive_step_handles_forward_and_rotation():
    meters_per_tick = 0.01
    x, y, yaw = robot.integrate_diff_drive_step(
        0.0, 0.0, 0.0, 10, 10, meters_per_tick, 0.445
    )
    assert x == pytest.approx(0.1)
    assert y == pytest.approx(0.0)
    assert yaw == pytest.approx(0.0)

    x, y, yaw = robot.integrate_diff_drive_step(
        0.0, 0.0, 0.0, -10, 10, meters_per_tick, 0.445
    )
    assert x == pytest.approx(0.0, abs=1e-6)
    assert y == pytest.approx(0.0, abs=1e-6)
    assert yaw == pytest.approx(20.0 * meters_per_tick / 0.445)


def test_node_publishes_odom_from_fake_feedback(monkeypatch):
    fake_serial = FakeSerial(
        make_feedback_frame(left_encoder=200, right_encoder=200, speedx_mm_s=200)
    )
    monkeypatch.setattr(robot.serial, 'Serial', lambda *args, **kwargs: fake_serial)

    node = robot.TurnOnTraymoverRobot()
    odom_messages = []
    transforms = []
    node.odom_pub = SimpleNamespace(publish=odom_messages.append)
    node.publish_odom_tf = True
    node.odom_tf_broadcaster = SimpleNamespace(sendTransform=transforms.append)

    node.send_frame_callback()

    assert fake_serial.writes
    assert len(odom_messages) == 1
    assert len(transforms) == 1

    odom_msg = odom_messages[0]
    expected_x = 200 * robot.meters_per_tick(0.20, 5600, 10)
    assert odom_msg.header.frame_id == 'odom'
    assert odom_msg.child_frame_id == 'base_footprint'
    assert odom_msg.pose.pose.position.x == pytest.approx(expected_x)
    assert odom_msg.pose.pose.position.y == pytest.approx(0.0)
    assert odom_msg.twist.twist.linear.x == pytest.approx(0.2)
    assert odom_msg.twist.twist.angular.z == pytest.approx(0.0)
    assert transforms[0].header.frame_id == 'odom'
    assert transforms[0].child_frame_id == 'base_footprint'

    node.destroy_node()


def test_node_applies_encoder_sign_parameters(monkeypatch):
    fake_serial = FakeSerial(
        make_feedback_frame(left_encoder=80, right_encoder=80, speedx_mm_s=80)
    )
    monkeypatch.setattr(robot.serial, 'Serial', lambda *args, **kwargs: fake_serial)

    node = robot.TurnOnTraymoverRobot()
    node.left_encoder_sign = -1
    node.right_encoder_sign = -1
    odom_messages = []
    node.odom_pub = SimpleNamespace(publish=odom_messages.append)

    node.send_frame_callback()

    assert len(odom_messages) == 1
    assert odom_messages[0].pose.pose.position.x < 0.0

    node.destroy_node()
