#!/usr/bin/env python3
# coding=utf-8
"""
Traymover Robot Serial Driver Node

Subscribes to /cmd_vel (geometry_msgs/Twist) and sends 40-byte control frames
to the STM32 chassis controller via serial port.

Protocol: 40-byte fixed-length frame
  - Header: 0x7F 0x7F
  - Tail: 0x0D 0x0A
  - Checksum: two's complement of sum(bytes[0:37])
  - Velocity encoding: velocity * 1000 as int16 big-endian

Reference: SerialDriver.md for full protocol specification.
"""

import struct
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import serial


# 协议常量
FRAME_SIZE = 40
FRAME_HEADER = [0x7F, 0x7F]
FRAME_LENGTH = 0x28  # 40
FRAME_TAIL = [0x0D, 0x0A]

# Serial ID Cmd
MSG_ID_GET_MOTOR_DATA = 0x02


def build_frame(vel_x_ms, vel_th_rads,
                wheel_diameter_m=0.17,
                wheel_track_m=0.455,
                gear_reduction=5600,
                tick_meter_ratio=10,
                left_motor_scale=1,
                right_motor_scale=1,
                alarm_led=1,
                ir_threshold=20):
    """
    构建40字节控制帧。

    Args:
        vel_x_ms: X轴线速度 (m/s)
        vel_th_rads: Z轴角速度 (rad/s)
        其余参数: 底盘机械/传感器配置

    Returns:
        bytes: 40字节帧数据
    """
    frame = bytearray(FRAME_SIZE)

    # [0-1] Header
    frame[0] = 0x7F
    frame[1] = 0x7F

    # [2] Length
    frame[2] = FRAME_LENGTH

    # [3] Serial ID Cmd
    frame[3] = MSG_ID_GET_MOTOR_DATA

    # [4] Move Cmd: 0x00 (Forward) — 速度控制模式下固定
    frame[4] = 0x00

    # [5-6] Distance: 0x0000 (无距离限制)
    frame[5] = 0x00
    frame[6] = 0x00

    # [7] System Mode: 0x00
    frame[7] = 0x00

    # [8] TOF Switch: 0x00
    frame[8] = 0x00

    # [9] Reserved
    frame[9] = 0x00

    # [10-11] Vel_x: linear.x * 1000, int16 big-endian
    vel_x_int = int(vel_x_ms * 1000)
    vel_x_int = max(-32768, min(32767, vel_x_int))  # clamp to int16
    frame[10] = (vel_x_int >> 8) & 0xFF
    frame[11] = vel_x_int & 0xFF

    # [12-13] Vel_th: angular.z * 1000, int16 big-endian
    vel_th_int = int(vel_th_rads * 1000)
    vel_th_int = max(-32768, min(32767, vel_th_int))
    frame[12] = (vel_th_int >> 8) & 0xFF
    frame[13] = vel_th_int & 0xFF

    # [14-15] Left Motor Scale, uint16 big-endian
    frame[14] = (left_motor_scale >> 8) & 0xFF
    frame[15] = left_motor_scale & 0xFF

    # [16-17] Right Motor Scale, uint16 big-endian
    frame[16] = (right_motor_scale >> 8) & 0xFF
    frame[17] = right_motor_scale & 0xFF

    # [18] Update Motor Chip: 0x00
    frame[18] = 0x00

    # [19-20] Motor Filter: 0x0000 (初始值0, 由memset初始化)
    frame[19] = 0x00
    frame[20] = 0x00

    # [21] Alarm LED
    frame[21] = alarm_led & 0xFF

    # [22] IR Switch: 0x00
    frame[22] = 0x00

    # [23-25] IR Thresholds
    frame[23] = ir_threshold & 0xFF
    frame[24] = ir_threshold & 0xFF
    frame[25] = ir_threshold & 0xFF

    # [26] Wheel Diameter: wheel_diameter_m * 1000, uint8
    frame[26] = int(wheel_diameter_m * 1000) & 0xFF

    # [27-28] Gear Reduction 低16位 (big-endian)
    gear_low = gear_reduction & 0xFFFF
    frame[27] = (gear_low >> 8) & 0xFF
    frame[28] = gear_low & 0xFF

    # [29-30] Wheel Track: wheel_track_m * 1000, uint16 big-endian
    wheel_track_int = int(wheel_track_m * 1000)
    frame[29] = (wheel_track_int >> 8) & 0xFF
    frame[30] = wheel_track_int & 0xFF

    # [31] Tick Meter Ratio
    frame[31] = tick_meter_ratio & 0xFF

    # [32] Buzzer: 0 (静音)
    frame[32] = 0x00

    # [33] Arrival Signal: 0 (未到达)
    frame[33] = 0x00

    # [34-35] Gear Reduction 高16位 (big-endian)
    gear_high = (gear_reduction >> 16) & 0xFFFF
    frame[34] = (gear_high >> 8) & 0xFF
    frame[35] = gear_high & 0xFF

    # [36] Jack Status: 0 (无操作)
    frame[36] = 0x00

    # [37] Checksum: (~sum(bytes[0:37]) + 0x01) & 0xFF
    total = sum(frame[0:37]) & 0xFF
    frame[37] = ((~total) + 0x01) & 0xFF

    # [38-39] Tail
    frame[38] = 0x0D
    frame[39] = 0x0A

    return bytes(frame)


class TurnOnTraymoverRobot(Node):
    """Traymover机器人串口驱动节点"""

    def __init__(self):
        super().__init__('turn_on_traymover_robot')

        # 声明参数
        self.declare_parameter('usart_port_name', '/dev/ttyUSB0')
        self.declare_parameter('serial_baud_rate', 115200)
        self.declare_parameter('wheel_diameter', 0.17)
        self.declare_parameter('wheel_track', 0.455)
        self.declare_parameter('gear_reduction', 5600)
        self.declare_parameter('tick_meter_ratio', 10)
        self.declare_parameter('left_motor_scale', 1)
        self.declare_parameter('right_motor_scale', 1)
        self.declare_parameter('alarm_led', 1)
        self.declare_parameter('ir_threshold', 20)
        self.declare_parameter('cmd_vel_timeout', 0.5)

        # 读取参数
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

        # 速度状态
        self.vel_x = 0.0
        self.vel_th = 0.0
        self.last_cmd_vel_time = self.get_clock().now()

        # 打开串口
        self.serial_port = None
        try:
            self.serial_port = serial.Serial(
                port=self.port_name,
                baudrate=self.baud_rate,
                bytesize=serial.EIGHTBITS,
                stopbits=serial.STOPBITS_ONE,
                parity=serial.PARITY_NONE,
                timeout=0.1
            )
            self.get_logger().info(
                f'Serial port opened: {self.port_name} @ {self.baud_rate} bps')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port {self.port_name}: {e}')
            self.get_logger().warn('Node will continue without serial (for testing)')

        # 订阅 /cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 2)

        # 定时发送帧 (10Hz)
        self.send_timer = self.create_timer(0.1, self.send_frame_callback)

        self.get_logger().info('Traymover robot driver initialized')

    def cmd_vel_callback(self, twist):
        """接收速度指令"""
        self.vel_x = twist.linear.x
        self.vel_th = twist.angular.z
        self.last_cmd_vel_time = self.get_clock().now()

    def send_frame_callback(self):
        """定时构建并发送40字节控制帧"""
        # 超时保护：如果超过cmd_vel_timeout未收到指令，速度归零
        now = self.get_clock().now()
        dt = (now - self.last_cmd_vel_time).nanoseconds / 1e9
        if dt > self.cmd_vel_timeout:
            self.vel_x = 0.0
            self.vel_th = 0.0

        # 构建帧
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

        # 发送
        if self.serial_port is not None and self.serial_port.is_open:
            try:
                self.serial_port.write(frame)
            except serial.SerialException as e:
                self.get_logger().error(f'Serial write failed: {e}')

    def destroy_node(self):
        """关闭时发送零速度帧并关闭串口"""
        # 发送停车帧
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
