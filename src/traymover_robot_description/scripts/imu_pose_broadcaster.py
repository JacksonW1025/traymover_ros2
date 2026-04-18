#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


class ImuPoseBroadcaster(Node):
    """Broadcast base_footprint -> base_link using the latest IMU quaternion."""

    def __init__(self):
        super().__init__('imu_pose_broadcaster')

        self.declare_parameter('imu_topic', '/imu/data_raw')
        self.declare_parameter('parent_frame', 'base_footprint')
        self.declare_parameter('child_frame', 'base_link')
        self.declare_parameter('translation_x', 0.0)
        self.declare_parameter('translation_y', 0.0)
        self.declare_parameter('translation_z', 0.20)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('yaw_only', True)

        self.imu_topic = self.get_parameter('imu_topic').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        self.translation_x = float(self.get_parameter('translation_x').value)
        self.translation_y = float(self.get_parameter('translation_y').value)
        self.translation_z = float(self.get_parameter('translation_z').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.yaw_only = bool(self.get_parameter('yaw_only').value)

        if self.publish_rate <= 0.0:
            self.get_logger().warn('publish_rate must be positive, defaulting to 30.0 Hz')
            self.publish_rate = 30.0

        self.latest_quaternion = (0.0, 0.0, 0.0, 1.0)
        self.has_received_imu = False
        self.last_invalid_warn_time = None

        self.tf_broadcaster = TransformBroadcaster(self)
        self.imu_sub = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            20,
        )
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_transform)

        self.get_logger().info(
            f'Broadcasting {self.parent_frame} -> {self.child_frame} from {self.imu_topic}'
        )

    def imu_callback(self, msg: Imu):
        quaternion = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        normalized = self.normalize_quaternion(quaternion)
        if normalized is None:
            self.warn_invalid_quaternion(quaternion)
            return

        if self.yaw_only:
            normalized = self.extract_yaw_quaternion(normalized)

        if not self.has_received_imu:
            self.get_logger().info('Received first valid IMU orientation')
        self.latest_quaternion = normalized
        self.has_received_imu = True

    def normalize_quaternion(self, quaternion):
        if any(not math.isfinite(component) for component in quaternion):
            return None

        norm = math.sqrt(sum(component * component for component in quaternion))
        if norm <= 1e-6:
            return None

        return tuple(component / norm for component in quaternion)

    def warn_invalid_quaternion(self, quaternion):
        now = self.get_clock().now()
        if self.last_invalid_warn_time is not None:
            if (now - self.last_invalid_warn_time).nanoseconds < int(5.0 * 1e9):
                return

        self.last_invalid_warn_time = now
        self.get_logger().warn(
            'Ignoring invalid IMU quaternion '
            f'({quaternion[0]:.4f}, {quaternion[1]:.4f}, '
            f'{quaternion[2]:.4f}, {quaternion[3]:.4f})'
        )

    def extract_yaw_quaternion(self, quaternion):
        x, y, z, w = quaternion

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        half_yaw = yaw * 0.5
        return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))

    def publish_transform(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = self.child_frame
        transform.transform.translation.x = self.translation_x
        transform.transform.translation.y = self.translation_y
        transform.transform.translation.z = self.translation_z
        transform.transform.rotation.x = self.latest_quaternion[0]
        transform.transform.rotation.y = self.latest_quaternion[1]
        transform.transform.rotation.z = self.latest_quaternion[2]
        transform.transform.rotation.w = self.latest_quaternion[3]
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = ImuPoseBroadcaster()
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
