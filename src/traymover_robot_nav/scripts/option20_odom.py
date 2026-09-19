#!/usr/bin/env python3
"""Timestamp-preserving planar FAST-LIO odometry with latched reset detection."""

import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


def rotation_matrix(q):
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(q).all() or norm < 0.5 or norm > 1.5:
        raise ValueError('invalid_quaternion')
    x, y, z, w = q / norm
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def planar_pose(position, quaternion, imu_height=0.12):
    """IMU is below base_link: imu_height is the positive body-to-base offset.

    URDF imu_joint is z=-0.12; therefore p_base=p_body+R_body*[0,0,0.12].
    The launch adds camera_init's 0.08 m ground offset and base_link is 0.20 m
    above base_footprint. Those vertical offsets disappear in planar xy/yaw.
    """
    rotation = rotation_matrix(quaternion)
    base = np.asarray(position, dtype=float) + rotation @ [0., 0., imu_height]
    if not np.isfinite(base).all():
        raise ValueError('invalid_position')
    return np.array([base[0], base[1], math.atan2(rotation[1, 0], rotation[0, 0])])


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


class PlanarTracker:
    """Reject discontinuities permanently; never silently reconnect two origins."""

    def __init__(self, max_age=0.5, max_gap=0.75, max_speed=1.5,
                 max_yaw_rate=2.5, filter_tau=0.10):
        self.max_age, self.max_gap = max_age, max_gap
        self.max_speed, self.max_yaw_rate = max_speed, max_yaw_rate
        self.filter_tau = filter_tau
        self.previous = None
        self.twist = np.zeros(3)
        self.fault = ''

    def update(self, stamp, now, pose):
        if self.fault:
            raise ValueError(self.fault)
        if not np.isfinite([stamp, now, *pose]).all() or stamp <= 0:
            raise ValueError('invalid_stamp_or_pose')
        if now-stamp > self.max_age or stamp-now > 0.10:
            raise ValueError('stale_or_future_odometry')
        if self.previous is not None:
            old_stamp, old_pose = self.previous
            dt = stamp-old_stamp
            delta = pose-old_pose
            delta[2] = wrap_angle(delta[2])
            if dt <= 0:
                self.fault = 'odometry_timestamp_reset'
            elif dt > self.max_gap:
                self.fault = 'odometry_gap'
            elif (np.linalg.norm(delta[:2])/dt > self.max_speed
                  or abs(delta[2])/dt > self.max_yaw_rate):
                self.fault = 'odometry_jump'
            if self.fault:
                raise ValueError(self.fault)
            c, s = math.cos(pose[2]), math.sin(pose[2])
            velocity = np.array([c*delta[0]+s*delta[1],
                                 -s*delta[0]+c*delta[1], delta[2]]) / dt
            alpha = dt/(self.filter_tau+dt)
            self.twist += alpha*(velocity-self.twist)
        self.previous = stamp, pose.copy()
        return self.twist.copy()


class Option20Odom(Node):
    def __init__(self):
        super().__init__('option20_odom')
        defaults = {'input_topic': '/option20/lio_odom', 'imu_height': 0.12,
                    'max_age': 0.5, 'max_gap': 0.75, 'max_speed': 1.5,
                    'max_yaw_rate': 2.5, 'filter_tau': 0.10}
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.tracker = PlanarTracker(**{k: self.get_parameter(k).value for k in
                                     ('max_age', 'max_gap', 'max_speed',
                                      'max_yaw_rate', 'filter_tau')})
        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.status_pub = self.create_publisher(String, '/option20/odom_status', 10)
        self.tf_pub = TransformBroadcaster(self)
        self.last_reported_reason = ''
        self.create_subscription(Odometry, self.get_parameter('input_topic').value,
                                 self.on_odom, qos_profile_sensor_data)

    def on_odom(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        status = {'stamp': stamp, 'stamp_sec': stamp, 'healthy': False,
                  'valid': False, 'reason': ''}
        try:
            if msg.header.frame_id != 'camera_init' or msg.child_frame_id != 'body':
                self.tracker.fault = 'unexpected_odometry_frames'
                raise ValueError(self.tracker.fault)
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            pose = planar_pose([p.x, p.y, p.z], [q.x, q.y, q.z, q.w],
                               self.get_parameter('imu_height').value)
            twist = self.tracker.update(stamp, self.get_clock().now().nanoseconds*1e-9,
                                        pose)
            odom = Odometry()
            odom.header.stamp = msg.header.stamp
            odom.header.frame_id, odom.child_frame_id = 'odom', 'base_footprint'
            odom.pose.pose.position.x, odom.pose.pose.position.y = map(float, pose[:2])
            odom.pose.pose.orientation.z = math.sin(pose[2]/2)
            odom.pose.pose.orientation.w = math.cos(pose[2]/2)
            odom.twist.twist.linear.x, odom.twist.twist.linear.y = map(float, twist[:2])
            odom.twist.twist.angular.z = float(twist[2])
            # Planar unobserved components are explicitly uncertain.
            for index, variance in zip((0, 7, 14, 21, 28, 35),
                                       (0.04, 0.04, 1e6, 1e6, 1e6, 0.02)):
                odom.pose.covariance[index] = variance
                odom.twist.covariance[index] = variance
            transform = TransformStamped()
            transform.header = odom.header
            transform.child_frame_id = odom.child_frame_id
            transform.transform.translation.x = float(pose[0])
            transform.transform.translation.y = float(pose[1])
            transform.transform.rotation = odom.pose.pose.orientation
            self.pub.publish(odom)
            self.tf_pub.sendTransform(transform)
            status.update(healthy=True, valid=True)
        except ValueError as exc:
            status['reason'] = str(exc)
        reason = status['reason']
        if reason != self.last_reported_reason:
            if reason:
                self.get_logger().warning(
                    f'Planar odometry rejected: {reason}; '
                    f'latched={bool(self.tracker.fault)}, stamp={stamp:.6f}. '
                    'See /option20/odom_status; navigation remains stopped.')
            else:
                self.get_logger().info('Planar odometry input is current again.')
            self.last_reported_reason = reason
        self.status_pub.publish(String(data=json.dumps(status)))


def main():
    rclpy.init()
    node = Option20Odom()
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
