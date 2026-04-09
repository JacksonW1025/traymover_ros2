#!/usr/bin/env python3
# coding=utf-8
"""
Traymover Robot Serial Driver Node

This node subscribes to /cmd_vel, sends 40-byte control/query frames to the
STM32 chassis controller, parses the returned motor feedback frame, and
publishes raw wheel odometry on /odom.
"""

import math
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
import serial
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler


FRAME_SIZE = 40
FRAME_HEADER = b'\x7F\x7F'
FRAME_LENGTH = 0x28
FRAME_TAIL = b'\x0D\x0A'

MSG_ID_GET_MOTOR_DATA = 0x02
STATUS_SUCCESS = 0x60

ODOM_POSE_COVARIANCE_MOVING = (
    1e6, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1e6, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1e6,
)
ODOM_POSE_COVARIANCE_STOPPED = (
    1e6, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1e6, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1e-9,
)
ODOM_TWIST_COVARIANCE_MOVING = (
    1e-3, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1e6, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1e-6,
)
ODOM_TWIST_COVARIANCE_STOPPED = (
    1e-6, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1e6, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1e-6,
)


@dataclass(frozen=True)
class MotorFeedback:
    left_encoder_ticks: int
    right_encoder_ticks: int
    linear_velocity_mps: float
    angular_velocity_rads: float
    emergency_status: int
    init_status: int
    power: int
    charge_status: int
    left_alarm: int
    right_alarm: int
    driver_mode: int
    motor_enabled: int
    shutdown: int
    version: int


def clamp_int16(value: int) -> int:
    return max(-32768, min(32767, value))


def compute_checksum(frame, end_index: int = 37) -> int:
    total = sum(frame[0:end_index]) & 0xFF
    return ((~total) + 0x01) & 0xFF


def build_frame(
    vel_x_ms,
    vel_th_rads,
    wheel_diameter_m=0.20,
    wheel_track_m=0.445,
    gear_reduction=5600,
    tick_meter_ratio=10,
    left_motor_scale=1,
    right_motor_scale=1,
    alarm_led=1,
    ir_threshold=20,
):
    """Build the 40-byte control/query frame."""
    frame = bytearray(FRAME_SIZE)
    frame[0:2] = FRAME_HEADER
    frame[2] = FRAME_LENGTH
    frame[3] = MSG_ID_GET_MOTOR_DATA
    frame[4] = 0x00
    frame[5] = 0x00
    frame[6] = 0x00
    frame[7] = 0x00
    frame[8] = 0x00
    frame[9] = 0x00

    vel_x_int = clamp_int16(int(vel_x_ms * 1000))
    frame[10:12] = vel_x_int.to_bytes(2, 'big', signed=True)

    vel_th_int = clamp_int16(int(vel_th_rads * 1000))
    frame[12:14] = vel_th_int.to_bytes(2, 'big', signed=True)

    frame[14:16] = int(left_motor_scale).to_bytes(2, 'big', signed=False)
    frame[16:18] = int(right_motor_scale).to_bytes(2, 'big', signed=False)

    frame[18] = 0x00
    frame[19] = 0x00
    frame[20] = 0x00
    frame[21] = int(alarm_led) & 0xFF
    frame[22] = 0x00
    frame[23] = int(ir_threshold) & 0xFF
    frame[24] = int(ir_threshold) & 0xFF
    frame[25] = int(ir_threshold) & 0xFF

    frame[26] = int(wheel_diameter_m * 1000) & 0xFF

    gear_reduction_int = int(gear_reduction) & 0xFFFFFFFF
    gear_bytes = gear_reduction_int.to_bytes(4, 'big', signed=False)
    frame[27] = gear_bytes[2]
    frame[28] = gear_bytes[3]

    wheel_track_int = max(0, int(wheel_track_m * 1000))
    frame[29:31] = wheel_track_int.to_bytes(2, 'big', signed=False)
    frame[31] = int(tick_meter_ratio) & 0xFF
    frame[32] = 0x00
    frame[33] = 0x00
    frame[34] = gear_bytes[0]
    frame[35] = gear_bytes[1]
    frame[36] = 0x00

    frame[37] = compute_checksum(frame)
    frame[38:40] = FRAME_TAIL
    return bytes(frame)


def validate_frame_basics(frame: bytes) -> None:
    if len(frame) != FRAME_SIZE:
        raise ValueError(f'frame size {len(frame)} != {FRAME_SIZE}')
    if frame[0:2] != FRAME_HEADER:
        raise ValueError('invalid frame header')
    if frame[2] != FRAME_LENGTH:
        raise ValueError(f'invalid frame length byte 0x{frame[2]:02X}')
    if frame[38:40] != FRAME_TAIL:
        raise ValueError('invalid frame tail')
    expected_checksum = compute_checksum(frame)
    if frame[37] != expected_checksum:
        raise ValueError(
            f'invalid checksum 0x{frame[37]:02X}, expected 0x{expected_checksum:02X}'
        )


def extract_next_valid_frame(buffer: bytearray) -> bytes | None:
    """
    Extract the next checksum-valid 40-byte frame from a rolling buffer.

    The buffer is modified in-place. Garbage and malformed candidates are
    discarded until a valid frame is found or more bytes are needed.
    """
    while len(buffer) >= 2:
        start_index = buffer.find(FRAME_HEADER)
        if start_index < 0:
            if len(buffer) > 1:
                del buffer[:-1]
            return None

        if start_index > 0:
            del buffer[:start_index]

        if len(buffer) < FRAME_SIZE:
            return None

        if buffer[2] != FRAME_LENGTH:
            del buffer[0]
            continue

        candidate = bytes(buffer[:FRAME_SIZE])
        try:
            validate_frame_basics(candidate)
        except ValueError:
            del buffer[0]
            continue

        del buffer[:FRAME_SIZE]
        return candidate

    return None


def parse_motor_feedback_frame(frame: bytes) -> MotorFeedback:
    validate_frame_basics(frame)

    if frame[3] != MSG_ID_GET_MOTOR_DATA:
        raise ValueError(f'unexpected msg id 0x{frame[3]:02X}')
    if frame[4] != STATUS_SUCCESS:
        raise ValueError(f'unexpected status 0x{frame[4]:02X}')

    left_encoder_ticks = int.from_bytes(frame[11:13], 'big', signed=True)
    right_encoder_ticks = int.from_bytes(frame[13:15], 'big', signed=True)
    linear_velocity_mps = int.from_bytes(frame[29:31], 'big', signed=True) / 1000.0
    angular_velocity_rads = int.from_bytes(frame[31:33], 'big', signed=True) / 1000.0

    return MotorFeedback(
        left_encoder_ticks=left_encoder_ticks,
        right_encoder_ticks=right_encoder_ticks,
        linear_velocity_mps=linear_velocity_mps,
        angular_velocity_rads=angular_velocity_rads,
        emergency_status=frame[5],
        init_status=frame[6],
        power=frame[7],
        charge_status=frame[8],
        left_alarm=frame[9],
        right_alarm=frame[10],
        driver_mode=frame[33],
        motor_enabled=frame[27],
        shutdown=frame[28],
        version=int.from_bytes(frame[35:37], 'big', signed=False),
    )


def meters_per_tick(
    wheel_diameter_m: float,
    gear_reduction: float,
    tick_meter_ratio: float,
) -> float:
    denominator = float(gear_reduction) * float(tick_meter_ratio)
    if denominator <= 0.0:
        raise ValueError('gear_reduction * tick_meter_ratio must be positive')
    if wheel_diameter_m <= 0.0:
        raise ValueError('wheel_diameter must be positive')
    return math.pi * float(wheel_diameter_m) / denominator


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def integrate_diff_drive_step(
    x: float,
    y: float,
    yaw: float,
    left_ticks: int,
    right_ticks: int,
    meters_per_tick_value: float,
    wheel_track_m: float,
) -> tuple[float, float, float]:
    if wheel_track_m <= 0.0:
        raise ValueError('wheel_track must be positive')

    left_distance = float(left_ticks) * meters_per_tick_value
    right_distance = float(right_ticks) * meters_per_tick_value
    delta_s = (left_distance + right_distance) * 0.5
    delta_yaw = (right_distance - left_distance) / float(wheel_track_m)
    mid_yaw = yaw + (delta_yaw * 0.5)

    x += delta_s * math.cos(mid_yaw)
    y += delta_s * math.sin(mid_yaw)
    yaw = normalize_angle(yaw + delta_yaw)
    return x, y, yaw


class TurnOnTraymoverRobot(Node):
    """Traymover robot serial driver node."""

    def __init__(self):
        super().__init__('turn_on_traymover_robot')

        self.declare_parameter('usart_port_name', '/dev/ttyUSB0')
        self.declare_parameter('serial_baud_rate', 115200)
        self.declare_parameter('wheel_diameter', 0.20)
        self.declare_parameter('wheel_track', 0.445)
        self.declare_parameter('gear_reduction', 5600)
        self.declare_parameter('tick_meter_ratio', 10)
        self.declare_parameter('left_motor_scale', 1)
        self.declare_parameter('right_motor_scale', 1)
        self.declare_parameter('alarm_led', 1)
        self.declare_parameter('ir_threshold', 20)
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('robot_frame_id', 'base_footprint')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('odom_source_mode', 'stm32_feedback')
        self.declare_parameter('publish_odom_tf', False)
        self.declare_parameter('left_encoder_sign', 1)
        self.declare_parameter('right_encoder_sign', 1)
        self.declare_parameter('poll_rate_hz', 20.0)

        self.port_name = self.get_parameter('usart_port_name').value
        self.baud_rate = self.get_parameter('serial_baud_rate').value
        self.wheel_diameter = self.get_parameter('wheel_diameter').value
        self.wheel_track = self.get_parameter('wheel_track').value
        self.gear_reduction = self.get_parameter('gear_reduction').value
        self.tick_meter_ratio = self.get_parameter('tick_meter_ratio').value
        self.left_motor_scale = self.get_parameter('left_motor_scale').value
        self.right_motor_scale = self.get_parameter('right_motor_scale').value
        self.alarm_led = self.get_parameter('alarm_led').value
        self.ir_threshold = self.get_parameter('ir_threshold').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
        self.robot_frame_id = self.get_parameter('robot_frame_id').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.odom_source_mode = self.get_parameter('odom_source_mode').value
        self.publish_odom_tf = self.get_parameter('publish_odom_tf').value
        self.left_encoder_sign = int(self.get_parameter('left_encoder_sign').value)
        self.right_encoder_sign = int(self.get_parameter('right_encoder_sign').value)
        self.poll_rate_hz = float(self.get_parameter('poll_rate_hz').value)

        if self.left_encoder_sign == 0:
            self.left_encoder_sign = 1
        if self.right_encoder_sign == 0:
            self.right_encoder_sign = 1
        if self.poll_rate_hz <= 0.0:
            self.get_logger().warn('poll_rate_hz must be positive; falling back to 20.0 Hz.')
            self.poll_rate_hz = 20.0

        self.odom_enabled = self.odom_source_mode == 'stm32_feedback'
        if self.odom_source_mode not in ('none', 'stm32_feedback'):
            self.get_logger().warn(
                f'Unsupported odom_source_mode={self.odom_source_mode!r}; '
                'falling back to command-only mode.'
            )
            self.odom_enabled = False

        self.vel_x = 0.0
        self.vel_th = 0.0
        self.last_cmd_vel_time = self.get_clock().now()

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.receive_buffer = bytearray()
        self.serial_read_timeout = min(0.1, max(0.02, 1.0 / self.poll_rate_hz))
        self._last_warning_times = {}
        self._feedback_active_logged = False
        self._last_feedback_mono = None

        try:
            self.meters_per_tick = meters_per_tick(
                self.wheel_diameter,
                self.gear_reduction,
                self.tick_meter_ratio,
            )
        except ValueError as exc:
            self.get_logger().error(f'Invalid odometry parameters: {exc}')
            self.meters_per_tick = 0.0
            self.odom_enabled = False

        self.serial_port = None
        try:
            self.serial_port = serial.Serial(
                port=self.port_name,
                baudrate=self.baud_rate,
                bytesize=serial.EIGHTBITS,
                stopbits=serial.STOPBITS_ONE,
                parity=serial.PARITY_NONE,
                timeout=0.0,
                write_timeout=0.1,
            )
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            self.get_logger().info(
                f'Serial port opened: {self.port_name} @ {self.baud_rate} bps'
            )
        except serial.SerialException as exc:
            self.get_logger().error(f'Failed to open serial port {self.port_name}: {exc}')
            self.get_logger().warn('Node will continue without serial (for testing).')

        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 2
        )
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.odom_tf_broadcaster = TransformBroadcaster(self) if self.publish_odom_tf else None
        self.poll_timer = self.create_timer(1.0 / self.poll_rate_hz, self.send_frame_callback)

        self.log_odometry_capability()
        self.get_logger().info('Traymover robot driver initialized')

    def throttled_warn(self, key: str, message: str, interval: float = 2.0) -> None:
        now = time.monotonic()
        last_time = self._last_warning_times.get(key, float('-inf'))
        if now - last_time >= interval:
            self._last_warning_times[key] = now
            self.get_logger().warn(message)

    def log_odometry_capability(self):
        if not self.odom_enabled:
            self.get_logger().warn(
                'STM32 odometry feedback is disabled; this node will not publish /odom.'
            )
            return

        self.get_logger().info(
            'STM32 odometry feedback enabled: publishing /odom from wheel encoder deltas '
            f'with wheel_track={self.wheel_track:.3f} m, wheel_diameter={self.wheel_diameter:.3f} m, '
            f'publish_odom_tf={self.publish_odom_tf}.'
        )

    def cmd_vel_callback(self, twist: Twist):
        self.vel_x = twist.linear.x
        self.vel_th = twist.angular.z
        self.last_cmd_vel_time = self.get_clock().now()

    def send_frame_callback(self):
        now = self.get_clock().now()
        dt = (now - self.last_cmd_vel_time).nanoseconds / 1e9
        if dt > self.cmd_vel_timeout:
            self.vel_x = 0.0
            self.vel_th = 0.0

        frame = build_frame(
            vel_x_ms=self.vel_x,
            vel_th_rads=self.vel_th,
            wheel_diameter_m=self.wheel_diameter,
            wheel_track_m=self.wheel_track,
            gear_reduction=self.gear_reduction,
            tick_meter_ratio=self.tick_meter_ratio,
            left_motor_scale=self.left_motor_scale,
            right_motor_scale=self.right_motor_scale,
            alarm_led=self.alarm_led,
            ir_threshold=self.ir_threshold,
        )

        if self.serial_port is None or not self.serial_port.is_open:
            return

        try:
            self.serial_port.write(frame)
        except serial.SerialException as exc:
            self.throttled_warn('serial_write', f'Serial write failed: {exc}')
            return

        if not self.odom_enabled:
            return

        feedback = self.read_motor_feedback()
        if feedback is None:
            self.throttled_warn(
                'missing_feedback',
                'No valid STM32 motor feedback frame received; /odom was not updated.',
            )
            return

        self.handle_motor_feedback(feedback, now.to_msg())

    def read_motor_feedback(self) -> MotorFeedback | None:
        if self.serial_port is None or not self.serial_port.is_open:
            return None

        deadline = time.monotonic() + self.serial_read_timeout
        while time.monotonic() < deadline:
            try:
                available = self.serial_port.in_waiting
            except serial.SerialException as exc:
                self.throttled_warn('serial_read_status', f'Serial read failed: {exc}')
                return None

            try:
                if available > 0:
                    chunk = self.serial_port.read(available)
                else:
                    chunk = self.serial_port.read(1)
            except serial.SerialException as exc:
                self.throttled_warn('serial_read', f'Serial read failed: {exc}')
                return None

            if chunk:
                self.receive_buffer.extend(chunk)
            else:
                time.sleep(0.001)

            while True:
                frame = extract_next_valid_frame(self.receive_buffer)
                if frame is None:
                    break

                try:
                    feedback = parse_motor_feedback_frame(frame)
                except ValueError as exc:
                    self.throttled_warn('feedback_parse', f'Discarding STM32 frame: {exc}')
                    continue

                return feedback

        return None

    def handle_motor_feedback(self, feedback: MotorFeedback, stamp):
        left_ticks = self.left_encoder_sign * feedback.left_encoder_ticks
        right_ticks = self.right_encoder_sign * feedback.right_encoder_ticks

        self.odom_x, self.odom_y, self.odom_yaw = integrate_diff_drive_step(
            self.odom_x,
            self.odom_y,
            self.odom_yaw,
            left_ticks,
            right_ticks,
            self.meters_per_tick,
            self.wheel_track,
        )

        stationary = (
            left_ticks == 0
            and right_ticks == 0
            and abs(feedback.linear_velocity_mps) < 1e-6
            and abs(feedback.angular_velocity_rads) < 1e-6
        )

        odom_msg = self.build_odom_message(feedback, left_ticks, right_ticks, stationary, stamp)
        self.odom_pub.publish(odom_msg)

        if self.publish_odom_tf:
            self.publish_odom_transform(stamp)

        self._last_feedback_mono = time.monotonic()
        if not self._feedback_active_logged:
            self._feedback_active_logged = True
            self.get_logger().info('STM32 odom feedback active.')

    def build_odom_message(
        self,
        feedback: MotorFeedback,
        left_ticks: int,
        right_ticks: int,
        stationary: bool,
        stamp,
    ) -> Odometry:
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.robot_frame_id

        quat = quaternion_from_euler(0.0, 0.0, self.odom_yaw)
        odom_msg.pose.pose.position.x = self.odom_x
        odom_msg.pose.pose.position.y = self.odom_y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        odom_msg.twist.twist.linear.x = feedback.linear_velocity_mps
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = feedback.angular_velocity_rads

        if stationary:
            odom_msg.pose.covariance = list(ODOM_POSE_COVARIANCE_STOPPED)
            odom_msg.twist.covariance = list(ODOM_TWIST_COVARIANCE_STOPPED)
        else:
            odom_msg.pose.covariance = list(ODOM_POSE_COVARIANCE_MOVING)
            odom_msg.twist.covariance = list(ODOM_TWIST_COVARIANCE_MOVING)

        return odom_msg

    def publish_odom_transform(self, stamp) -> None:
        if self.odom_tf_broadcaster is None:
            return

        quat = quaternion_from_euler(0.0, 0.0, self.odom_yaw)
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame_id
        transform.child_frame_id = self.robot_frame_id
        transform.transform.translation.x = self.odom_x
        transform.transform.translation.y = self.odom_y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]
        self.odom_tf_broadcaster.sendTransform(transform)

    def destroy_node(self):
        if self.serial_port is not None and self.serial_port.is_open:
            stop_frame = build_frame(
                vel_x_ms=0.0,
                vel_th_rads=0.0,
                wheel_diameter_m=self.wheel_diameter,
                wheel_track_m=self.wheel_track,
                gear_reduction=self.gear_reduction,
                tick_meter_ratio=self.tick_meter_ratio,
                left_motor_scale=self.left_motor_scale,
                right_motor_scale=self.right_motor_scale,
                alarm_led=self.alarm_led,
                ir_threshold=self.ir_threshold,
            )
            try:
                self.serial_port.write(stop_frame)
            except serial.SerialException:
                pass

            self.serial_port.close()
            self.get_logger().info('Serial port closed')

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TurnOnTraymoverRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
