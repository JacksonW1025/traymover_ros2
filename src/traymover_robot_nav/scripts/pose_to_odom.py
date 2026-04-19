#!/usr/bin/env python3
"""Bridge lidar_localization_ros2's /pcl_pose (PoseStamped) into a
nav_msgs/Odometry topic on /odom.

Nav2 controller_server and bt_navigator require an Odometry topic to read
current twist for trajectory prediction. lidar_localization_ros2 only
publishes PoseStamped, so we numerically differentiate position and yaw
and express the twist in the child (base_link) frame.
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


class PoseToOdom(Node):
    def __init__(self):
        super().__init__('pose_to_odom')
        self.declare_parameter('pose_topic', '/pcl_pose')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('base_frame', 'base_link')

        pose_topic = self.get_parameter('pose_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        self.base_frame = self.get_parameter('base_frame').value

        self.last_pose = None
        self.last_t = None

        self.sub = self.create_subscription(
            PoseStamped, pose_topic, self.on_pose, 10)
        self.pub = self.create_publisher(Odometry, odom_topic, 10)
        self.get_logger().info(
            f'bridging {pose_topic} (PoseStamped) -> {odom_topic} (Odometry)')

    def on_pose(self, msg: PoseStamped):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        odom = Odometry()
        odom.header = msg.header
        odom.child_frame_id = self.base_frame
        # Copy pose but clamp z/roll/pitch to zero so downstream Nav2 does not
        # see the NDT 6DOF distortion (same reason as tf_flattener).
        odom.pose.pose.position.x = msg.pose.position.x
        odom.pose.pose.position.y = msg.pose.position.y
        odom.pose.pose.position.z = 0.0
        yaw = yaw_from_quat(msg.pose.orientation)
        qx, qy, qz, qw = quat_from_yaw(yaw)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        if self.last_pose is not None and self.last_t is not None:
            dt = t - self.last_t
            if dt > 1e-4:
                dx = msg.pose.position.x - self.last_pose.position.x
                dy = msg.pose.position.y - self.last_pose.position.y
                yaw = yaw_from_quat(msg.pose.orientation)
                last_yaw = yaw_from_quat(self.last_pose.orientation)
                # rotate world-frame delta into body frame (base_link)
                cos_y, sin_y = math.cos(-yaw), math.sin(-yaw)
                vx = (dx * cos_y - dy * sin_y) / dt
                vy = (dx * sin_y + dy * cos_y) / dt
                dyaw = wrap_pi(yaw - last_yaw)
                wz = dyaw / dt
                odom.twist.twist.linear.x = vx
                odom.twist.twist.linear.y = vy
                odom.twist.twist.angular.z = wz

        self.last_pose = msg.pose
        self.last_t = t
        self.pub.publish(odom)


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quat_from_yaw(yaw):
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def main():
    rclpy.init()
    try:
        rclpy.spin(PoseToOdom())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
