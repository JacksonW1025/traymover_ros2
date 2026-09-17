#!/usr/bin/env python3
"""Filter PointCloud2 by XY angle while preserving each raw point record."""

import math
from typing import Tuple

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


def field_by_name(msg: PointCloud2, name: str) -> PointField:
    for field in msg.fields:
        if field.name == name:
            return field
    raise KeyError(f'PointCloud2 missing field {name!r}')


def xy_dtype(msg: PointCloud2) -> np.dtype:
    x_field = field_by_name(msg, 'x')
    y_field = field_by_name(msg, 'y')
    for field in (x_field, y_field):
        if field.datatype != PointField.FLOAT32 or field.count != 1:
            raise ValueError(f'field {field.name!r} must be FLOAT32[1]')
    endian = '>' if msg.is_bigendian else '<'
    return np.dtype({
        'names': ['x', 'y'],
        'formats': [f'{endian}f4', f'{endian}f4'],
        'offsets': [x_field.offset, y_field.offset],
        'itemsize': msg.point_step,
    })


def point_records(msg: PointCloud2) -> np.ndarray:
    point_count = int(msg.width) * int(msg.height)
    if point_count == 0:
        return np.empty((0, int(msg.point_step)), dtype=np.uint8)

    expected_row_step = int(msg.width) * int(msg.point_step)
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    usable_size = int(msg.row_step) * int(msg.height)
    if raw.size < usable_size:
        raise ValueError(
            f'PointCloud2 data too short: {raw.size} bytes for row_step={msg.row_step}, '
            f'height={msg.height}'
        )

    if int(msg.row_step) == expected_row_step:
        return raw[:usable_size].reshape(point_count, int(msg.point_step))

    rows = raw[:usable_size].reshape(int(msg.height), int(msg.row_step))
    compact = rows[:, :expected_row_step].reshape(point_count, int(msg.point_step))
    return np.ascontiguousarray(compact)


def angle_mask(xy: np.ndarray, angle_min_rad: float, angle_max_rad: float) -> np.ndarray:
    x = xy['x']
    y = xy['y']
    finite = np.isfinite(x) & np.isfinite(y)
    angles = np.arctan2(y, x)
    if angle_min_rad <= angle_max_rad:
        return finite & (angles >= angle_min_rad) & (angles <= angle_max_rad)
    return finite & ((angles >= angle_min_rad) | (angles <= angle_max_rad))


def filter_cloud(msg: PointCloud2, angle_min_rad: float, angle_max_rad: float) -> Tuple[PointCloud2, int]:
    records = point_records(msg)
    xy = records.view(xy_dtype(msg)).reshape(-1)
    keep = angle_mask(xy, angle_min_rad, angle_max_rad)
    filtered = records[keep]

    out = PointCloud2()
    out.header = msg.header
    out.height = 1
    out.width = int(filtered.shape[0])
    out.fields = msg.fields
    out.is_bigendian = msg.is_bigendian
    out.point_step = msg.point_step
    out.row_step = int(out.point_step) * int(out.width)
    out.is_dense = bool(msg.is_dense)
    out.data = filtered.tobytes()
    return out, int(keep.size - keep.sum())


class PointcloudAngularFilter(Node):
    def __init__(self) -> None:
        super().__init__('pointcloud_angular_filter')
        self.declare_parameter('input_topic', '/point_cloud_raw')
        self.declare_parameter('output_topic', '/point_cloud_raw_front')
        self.declare_parameter('angle_min_rad', -math.pi / 2.0)
        self.declare_parameter('angle_max_rad', math.pi / 2.0)
        self.declare_parameter('log_every_n', 50)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.angle_min_rad = float(self.get_parameter('angle_min_rad').value)
        self.angle_max_rad = float(self.get_parameter('angle_max_rad').value)
        self.log_every_n = max(0, int(self.get_parameter('log_every_n').value))
        self.frame_count = 0
        self._warned_invalid_cloud = False

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.pub = self.create_publisher(PointCloud2, self.output_topic, qos)
        self.sub = self.create_subscription(PointCloud2, self.input_topic, self.on_cloud, qos)
        self.get_logger().info(
            f'filtering {self.input_topic} -> {self.output_topic}, '
            f'angle=[{self.angle_min_rad:.3f}, {self.angle_max_rad:.3f}] rad'
        )

    def on_cloud(self, msg: PointCloud2) -> None:
        try:
            filtered, removed = filter_cloud(msg, self.angle_min_rad, self.angle_max_rad)
        except (KeyError, ValueError) as exc:
            if not self._warned_invalid_cloud:
                self._warned_invalid_cloud = True
                self.get_logger().error(f'cannot filter cloud: {exc}')
            return

        self.pub.publish(filtered)
        self.frame_count += 1
        if self.log_every_n and self.frame_count % self.log_every_n == 0:
            total = int(msg.width) * int(msg.height)
            self.get_logger().info(
                f'frame {self.frame_count}: kept {filtered.width}/{total}, removed {removed}'
            )


def main() -> None:
    rclpy.init()
    node = PointcloudAngularFilter()
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
