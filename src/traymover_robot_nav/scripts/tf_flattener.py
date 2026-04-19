#!/usr/bin/env python3
"""Listen to `map -> base_link` (6DOF from lidar_localization_ros2) and
republish `map -> base_link_2d` with z, roll, pitch clamped to zero so
Nav2 sees only the physically meaningful x, y, yaw components.

Motivation: indoor flat-floor NDT occasionally converges to a 3D local
minimum (we observed z=2.95 m, roll/pitch ~17 deg) while the projected
(x, y, yaw) remains plausible. Nav2 planning and control only need the
2D components; clamping here prevents bad 3D components from propagating
to costmap rolling, footprint placement, and MPPI prediction.
"""
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


class TfFlattener(Node):
    def __init__(self):
        super().__init__('tf_flattener')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('source_frame', 'base_link')
        self.declare_parameter('target_frame', 'base_link_2d')
        self.declare_parameter('rate_hz', 30.0)

        self.global_frame = self.get_parameter('global_frame').value
        self.source_frame = self.get_parameter('source_frame').value
        self.target_frame = self.get_parameter('target_frame').value
        rate = float(self.get_parameter('rate_hz').value)

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            f'flattening {self.global_frame} -> {self.source_frame} into '
            f'{self.global_frame} -> {self.target_frame} at {rate:.1f} Hz')

    def tick(self):
        try:
            t = self.buffer.lookup_transform(
                self.global_frame, self.source_frame, rclpy.time.Time())
        except Exception:
            return

        yaw = yaw_from_quat(t.transform.rotation)
        out = TransformStamped()
        out.header.stamp = t.header.stamp
        out.header.frame_id = self.global_frame
        out.child_frame_id = self.target_frame
        out.transform.translation.x = t.transform.translation.x
        out.transform.translation.y = t.transform.translation.y
        out.transform.translation.z = 0.0
        qx, qy, qz, qw = quat_from_yaw(yaw)
        out.transform.rotation.x = qx
        out.transform.rotation.y = qy
        out.transform.rotation.z = qz
        out.transform.rotation.w = qw
        self.broadcaster.sendTransform(out)


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quat_from_yaw(yaw):
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def main():
    rclpy.init()
    try:
        rclpy.spin(TfFlattener())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
