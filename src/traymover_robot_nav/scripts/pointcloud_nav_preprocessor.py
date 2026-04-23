#!/usr/bin/env python3
"""Prepare navigation-specific point clouds from the raw LiDAR stream."""

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener, TransformException


FIELDS_XYZI = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
]


def quaternion_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ], dtype=np.float32)


class PointcloudNavPreprocessor(Node):
    def __init__(self) -> None:
        super().__init__('pointcloud_nav_preprocessor')
        self.declare_parameter('input_topic', '/point_cloud_raw')
        self.declare_parameter('localization_topic', '/point_cloud_localization')
        self.declare_parameter('nav_topic', '/point_cloud_nav')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('rear_filter_min_x', -1.25)
        self.declare_parameter('rear_filter_max_x', -0.35)
        self.declare_parameter('rear_filter_half_width', 0.90)
        self.declare_parameter('rear_filter_min_z', -0.50)
        self.declare_parameter('rear_filter_max_z', 2.00)

        self.input_topic = self.get_parameter('input_topic').value
        self.localization_topic = self.get_parameter('localization_topic').value
        self.nav_topic = self.get_parameter('nav_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        self.rear_filter_min_x = float(self.get_parameter('rear_filter_min_x').value)
        self.rear_filter_max_x = float(self.get_parameter('rear_filter_max_x').value)
        self.rear_filter_half_width = float(self.get_parameter('rear_filter_half_width').value)
        self.rear_filter_min_z = float(self.get_parameter('rear_filter_min_z').value)
        self.rear_filter_max_z = float(self.get_parameter('rear_filter_max_z').value)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.localization_pub = self.create_publisher(PointCloud2, self.localization_topic, 10)
        self.nav_pub = self.create_publisher(PointCloud2, self.nav_topic, 10)
        self.sub = self.create_subscription(PointCloud2, self.input_topic, self.on_cloud, qos)
        self._last_tf_warn_ns = 0

        self.get_logger().info(
            f'preprocessing {self.input_topic} -> '
            f'{self.localization_topic}, {self.nav_topic} in {self.target_frame}'
        )

    def on_cloud(self, msg: PointCloud2) -> None:
        xyz_iter = point_cloud2.read_points(
            msg,
            field_names=['x', 'y', 'z'],
            skip_nans=True,
        )
        xyz_struct = np.asarray(list(xyz_iter))
        xyz = np.column_stack((
            xyz_struct['x'],
            xyz_struct['y'],
            xyz_struct['z'],
        )).astype(np.float32, copy=False)
        if xyz.size == 0:
            return
        if xyz.ndim == 1:
            xyz = xyz.reshape(1, 3)

        transformed_xyz = self.transform_xyz(msg, xyz)
        if transformed_xyz is None or transformed_xyz.size == 0:
            return

        rear_mask = (
            (transformed_xyz[:, 0] >= self.rear_filter_min_x) &
            (transformed_xyz[:, 0] <= self.rear_filter_max_x) &
            (np.abs(transformed_xyz[:, 1]) <= self.rear_filter_half_width) &
            (transformed_xyz[:, 2] >= self.rear_filter_min_z) &
            (transformed_xyz[:, 2] <= self.rear_filter_max_z)
        )
        localization_xyz = transformed_xyz[~rear_mask]
        if localization_xyz.size == 0:
            localization_xyz = transformed_xyz

        header = Header(stamp=msg.header.stamp, frame_id=self.target_frame)
        self.nav_pub.publish(self.to_cloud(header, transformed_xyz))
        self.localization_pub.publish(self.to_cloud(header, localization_xyz))

    def transform_xyz(self, msg: PointCloud2, xyz: np.ndarray) -> np.ndarray | None:
        if msg.header.frame_id == self.target_frame:
            return xyz.astype(np.float32, copy=False)

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                msg.header.stamp,
                timeout=Duration(seconds=0.1),
            )
        except TransformException as exc:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_tf_warn_ns > int(2e9):
                self._last_tf_warn_ns = now_ns
                self.get_logger().warn(
                    f'cannot transform {msg.header.frame_id} -> {self.target_frame}: {exc}'
                )
            return None

        q = transform.transform.rotation
        t = transform.transform.translation
        rotation = quaternion_to_matrix(q.x, q.y, q.z, q.w)
        translation = np.array([t.x, t.y, t.z], dtype=np.float32)
        return (xyz.astype(np.float32, copy=False) @ rotation.T) + translation

    def to_cloud(self, header, xyz: np.ndarray) -> PointCloud2:
        cloud_xyzi = np.empty((xyz.shape[0], 4), dtype=np.float32)
        cloud_xyzi[:, :3] = xyz
        cloud_xyzi[:, 3] = 0.0
        return point_cloud2.create_cloud(header, FIELDS_XYZI, cloud_xyzi)


def main() -> None:
    rclpy.init()
    node = PointcloudNavPreprocessor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
