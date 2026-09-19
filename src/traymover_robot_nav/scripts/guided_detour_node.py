#!/usr/bin/env python3
"""Minimal ROS boundary for Option 19: front depth, wheel odometry, direct velocity."""
from dataclasses import fields
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool

from detour_core import Pose, Ray, Scan, Velocity
from detour_motion import LineReturnConfig, LineReturnCore, LineReturnObservation
from detour_obstacle import stamp_seconds, yaw


def twist_is_zero(message):
    return all(value == 0.0 for value in (
        message.linear.x, message.linear.y, message.linear.z,
        message.angular.x, message.angular.y, message.angular.z))


class LineReturnDetourNode(Node):
    def __init__(self):
        super().__init__('option19_line_return')
        defaults = LineReturnConfig()
        self.config = LineReturnConfig(**{
            field.name: self.declare_parameter(field.name, getattr(defaults, field.name)).value
            for field in fields(defaults)
        })
        self.core = LineReturnCore(self.config)
        self.depth_scan = None
        self.pose = None
        self.odom_at = float('-inf')
        self.manual_estop = False
        self.aggregate_estop = False
        self.start_requested = False
        self.closing = False
        self.last_state = None
        self.output = self.create_publisher(Twist, '/cmd_vel', 1)
        status_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status = self.create_publisher(String, '/option19/state', status_qos)
        self.create_subscription(
            LaserScan, '/scan_realsense', self.on_depth_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(
            Bool, '/traymover_estop/state', self.on_aggregate_estop, status_qos)
        self.create_subscription(
            Bool, '/traymover_estop/manual_state', self.on_manual_estop, status_qos)
        self.manual_service = self.create_client(SetBool, '/traymover_estop/set')
        self.timer = self.create_timer(0.05, self.on_tick)
        self.publish_zero()

    def message_time(self, stamp):
        header = stamp_seconds(stamp)
        age = self.get_clock().now().nanoseconds * 1e-9 - header
        if header <= 0.0 or not math.isfinite(age) or age < -0.05:
            return None
        return time.monotonic() - max(0.0, age)

    def scan_from_message(self, message, origin_x=0.0, depth=False):
        acquired = self.message_time(message.header.stamp)
        if acquired is None or message.angle_increment <= 0.0:
            return None
        rays = []
        for index, value in enumerate(message.ranges):
            angle = message.angle_min + index * message.angle_increment
            if math.isfinite(value) and message.range_min <= value <= message.range_max:
                axial = max(0.0, value * math.cos(angle)) if depth else None
                rays.append(Ray(origin_x, 0.0, angle, value, True, axial))
            elif math.isinf(value) and value > 0.0 and message.range_max > 0.0:
                rays.append(Ray(origin_x, 0.0, angle, message.range_max, False))
        return Scan(acquired, tuple(rays)) if rays else None

    def on_depth_scan(self, message):
        # The RealSense is fixed facing forward at x=0.45 m. Treat the
        # depthimage_to_laserscan angles as base-forward angles directly.
        self.depth_scan = self.scan_from_message(message, origin_x=0.45, depth=True)

    def on_odom(self, message):
        acquired = self.message_time(message.header.stamp)
        position = message.pose.pose.position
        heading = yaw(message.pose.pose.orientation)
        if (acquired is None
                or not all(math.isfinite(value) for value in (position.x, position.y, heading))):
            self.pose = None
            self.odom_at = float('-inf')
            return
        self.pose = Pose(position.x, position.y, heading)
        self.odom_at = acquired

    def on_aggregate_estop(self, message):
        self.aggregate_estop = bool(message.data)
        if self.aggregate_estop:
            self.publish_zero()

    def on_manual_estop(self, message):
        self.manual_estop = bool(message.data)
        if self.manual_estop:
            self.publish_zero()

    def observation(self, now=None):
        now = time.monotonic() if now is None else now
        odom_fresh = (self.pose is not None
                      and 0.0 <= now - self.odom_at <= self.config.odom_timeout_sec)
        return LineReturnObservation(
            pose=self.pose, lidar=self.depth_scan, realsense=self.depth_scan,
            perimeter=self.depth_scan,
            aggregate_estop=self.aggregate_estop, manual_estop=self.manual_estop,
            auto_estop=False, auto_reason='NONE', estop_fresh=True,
            tf_healthy=True, localization_healthy=odom_fresh, odom_stamp=self.odom_at,
        )

    def start_motion(self):
        self.start_requested = True
        return True

    def on_tick(self):
        if self.closing:
            self.publish_zero()
            return
        now = time.monotonic()
        observation = self.observation(now)
        if self.start_requested and self.core.original_line is None:
            self.core.start(now, observation)
        velocity = (self.core.tick(now, observation)
                    if self.core.original_line is not None else Velocity())
        message = Twist()
        message.linear.x = velocity.linear_x
        message.angular.z = velocity.angular_z
        self.output.publish(message)
        if self.core.state != self.last_state:
            self.status.publish(String(data=f'{self.core.state} | {self.core.reason}'))
            self.last_state = self.core.state

    def publish_zero(self):
        self.output.publish(Twist())

    def assert_manual_estop(self):
        self.publish_zero()
        self.core.stop(time.monotonic(), 'manual EStop requested by operator')
        if self.manual_service.service_is_ready():
            return self.manual_service.call_async(SetBool.Request(data=True))
        return None

    def close_zero(self, duration=0.3):
        self.closing = True
        self.core.stop(time.monotonic(), 'option 19 shutdown')
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish_zero()
            rclpy.spin_once(self, timeout_sec=0.02)
        self.publish_zero()

    def destroy_node(self):
        self.publish_zero()
        return super().destroy_node()
