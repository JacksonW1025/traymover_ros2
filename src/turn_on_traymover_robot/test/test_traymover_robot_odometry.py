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
    power=0,
    charge_status=0,
    msg_id=robot.MSG_ID_GET_MOTOR_DATA,
    status=robot.STATUS_SUCCESS,
):
    frame = bytearray(robot.FRAME_SIZE)
    frame[0:2] = robot.FRAME_HEADER
    frame[2] = robot.FRAME_LENGTH
    frame[3] = msg_id
    frame[4] = status
    frame[7] = int(power) & 0xFF
    frame[8] = int(charge_status) & 0xFF
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


def make_status_frame(
    *,
    power_temperature=24,
    power_current=0,
    driver_mode=2,
    left_pwm=0,
    right_pwm=0,
    move_dir=0,
    android_cmd=0,
    left_code=0,
    front_code=0,
    right_code=0,
    tof_switch=0,
    left_scale=0,
    right_scale=0,
    motor_filter=0,
    version=42,
    msg_id=robot.MSG_ID_GET_MOTOR_STATUS,
    status=robot.STATUS_SUCCESS,
):
    frame = bytearray(robot.FRAME_SIZE)
    frame[0:2] = robot.FRAME_HEADER
    frame[2] = robot.FRAME_LENGTH
    frame[3] = msg_id
    frame[4] = status
    frame[5:7] = int(power_temperature).to_bytes(2, 'big', signed=True)
    frame[7:9] = int(power_current).to_bytes(2, 'big', signed=True)
    frame[9] = int(driver_mode) & 0xFF
    frame[10:12] = int(left_pwm).to_bytes(2, 'big', signed=True)
    frame[12:14] = int(right_pwm).to_bytes(2, 'big', signed=True)
    frame[14] = int(move_dir) & 0xFF
    frame[15] = int(android_cmd) & 0xFF
    frame[16] = int(left_code) & 0xFF
    frame[17] = int(front_code) & 0xFF
    frame[18] = int(right_code) & 0xFF
    frame[19] = int(tof_switch) & 0xFF
    frame[20:22] = int(left_scale).to_bytes(2, 'big', signed=False)
    frame[22:24] = int(right_scale).to_bytes(2, 'big', signed=False)
    frame[24] = int(motor_filter) & 0xFF
    frame[25:27] = int(version).to_bytes(2, 'big', signed=False)
    frame[37] = robot.compute_checksum(frame)
    frame[38:40] = robot.FRAME_TAIL
    return bytes(frame)


class FakeSerial:
    def __init__(self, feedback_frame=None, responses=None):
        self.feedback_frame = feedback_frame
        self.responses = {}
        for msg_id, response in (responses or {}).items():
            if isinstance(response, (list, tuple)):
                self.responses[msg_id] = list(response)
            else:
                self.responses[msg_id] = [response]
        self.read_buffer = bytearray()
        self.writes = []
        self.is_open = True

    @property
    def in_waiting(self):
        return len(self.read_buffer)

    def write(self, data):
        self.writes.append(bytes(data))
        msg_id = data[3] if len(data) > 3 else None
        response = None
        if msg_id in self.responses and self.responses[msg_id]:
            if len(self.responses[msg_id]) > 1:
                response = self.responses[msg_id].pop(0)
            else:
                response = self.responses[msg_id][0]
        elif self.feedback_frame is not None:
            response = self.feedback_frame

        if response is not None:
            self.read_buffer.extend(response)
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


def test_parse_motor_status_frame_converts_fields():
    frame = make_status_frame(
        power_temperature=31,
        power_current=12,
        left_pwm=180,
        right_pwm=-90,
        left_scale=3,
        right_scale=4,
        version=442,
    )

    status = robot.parse_motor_status_frame(frame)

    assert status.power_temperature == 31
    assert status.power_current == 12
    assert status.left_pwm == 180
    assert status.right_pwm == -90
    assert status.left_scale == 3
    assert status.right_scale == 4
    assert status.version == 442


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
    node.status_poll_period = None
    odom_messages = []
    transforms = []
    node.odom_pub = SimpleNamespace(publish=odom_messages.append)
    node.publish_odom_tf = True
    node.odom_tf_broadcaster = SimpleNamespace(sendTransform=transforms.append)

    node.send_frame_callback()

    assert fake_serial.writes
    assert len(odom_messages) == 1
    assert len(transforms) == 1

    # 第一次回调 _last_feedback_mono=None,dt = 1/poll_rate_hz。
    # STM32 报告 speedx=0.2 m/s,yaw=0 → x += 0.2 * dt。
    odom_msg = odom_messages[0]
    dt = 1.0 / node.poll_rate_hz
    expected_x = 0.2 * dt
    assert odom_msg.header.frame_id == 'odom'
    assert odom_msg.child_frame_id == 'base_footprint'
    assert odom_msg.pose.pose.position.x == pytest.approx(expected_x)
    assert odom_msg.pose.pose.position.y == pytest.approx(0.0)
    assert odom_msg.twist.twist.linear.x == pytest.approx(0.2)
    assert odom_msg.twist.twist.angular.z == pytest.approx(0.0)
    assert transforms[0].header.frame_id == 'odom'
    assert transforms[0].child_frame_id == 'base_footprint'

    node.destroy_node()


def test_node_ignores_encoder_signs_for_pose(monkeypatch):
    """符号参数现在只用于 stationary 检测,不再影响 pose —— pose 完全来自 STM32 速度。"""
    fake_serial = FakeSerial(
        make_feedback_frame(left_encoder=80, right_encoder=80, speedx_mm_s=80)
    )
    monkeypatch.setattr(robot.serial, 'Serial', lambda *args, **kwargs: fake_serial)

    node = robot.TurnOnTraymoverRobot()
    node.status_poll_period = None
    node.left_encoder_sign = -1
    node.right_encoder_sign = -1
    odom_messages = []
    node.odom_pub = SimpleNamespace(publish=odom_messages.append)

    node.send_frame_callback()

    assert len(odom_messages) == 1
    assert odom_messages[0].pose.pose.position.x > 0.0  # STM32 说前进,pose 就前进

    node.destroy_node()


def test_node_publishes_power_status_topics(monkeypatch):
    fake_serial = FakeSerial(
        responses={
            robot.MSG_ID_GET_MOTOR_DATA: make_feedback_frame(
                left_encoder=0,
                right_encoder=0,
                speedx_mm_s=0,
                speedth_millirad_s=0,
                power=98,
                charge_status=3,
            ),
            robot.MSG_ID_GET_MOTOR_STATUS: make_status_frame(
                power_temperature=24,
                power_current=7,
                version=442,
            ),
        }
    )
    monkeypatch.setattr(robot.serial, 'Serial', lambda *args, **kwargs: fake_serial)

    node = robot.TurnOnTraymoverRobot()
    node.odom_enabled = False
    battery_messages = []
    power_messages = []
    charge_messages = []
    temperature_messages = []
    current_messages = []
    node.battery_state_pub = SimpleNamespace(publish=battery_messages.append)
    node.power_level_pub = SimpleNamespace(publish=power_messages.append)
    node.charge_status_pub = SimpleNamespace(publish=charge_messages.append)
    node.power_temperature_pub = SimpleNamespace(publish=temperature_messages.append)
    node.power_current_pub = SimpleNamespace(publish=current_messages.append)

    node.send_frame_callback()

    assert len(power_messages) == 1
    assert power_messages[0].data == 98
    assert len(charge_messages) == 1
    assert charge_messages[0].data == 3
    assert len(temperature_messages) == 1
    assert temperature_messages[0].data == 24
    assert len(current_messages) == 1
    assert current_messages[0].data == 7
    assert battery_messages
    assert battery_messages[-1].percentage == pytest.approx(0.98)
    assert battery_messages[-1].temperature == pytest.approx(24.0)
    assert battery_messages[-1].current == pytest.approx(7.0)
    assert battery_messages[-1].power_supply_status == robot.BatteryState.POWER_SUPPLY_STATUS_CHARGING

    node.destroy_node()
