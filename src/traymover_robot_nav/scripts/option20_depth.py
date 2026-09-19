#!/usr/bin/env python3
"""Project the complete rectified depth ROI; invalid depth never means clear space."""

import json

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from option20_clouds import (clearing_points, fresh_stamp, obstacle_points, stamp_seconds,
                             transform_points, xyz_cloud)


def depth_xyz(image, info, stride=4, min_range=0.2, max_range=5.0):
    """Decode endian/row-stride aware depth and project using rectified intrinsics."""
    if stride < 1:
        raise ValueError('invalid_stride')
    encodings = {'16UC1': ('u2', 0.001), '32FC1': ('f4', 1.)}
    if image.encoding not in encodings:
        raise ValueError('unsupported_depth_encoding')
    if image.width != info.width or image.height != info.height:
        raise ValueError('camera_info_dimensions_mismatch')
    if image.header.frame_id != info.header.frame_id:
        raise ValueError('camera_info_frame_mismatch')
    # image_rect_raw uses P. K is the fallback for a calibration lacking P.
    fx, fy, cx, cy = info.p[0], info.p[5], info.p[2], info.p[6]
    if fx <= 0 or fy <= 0:
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    if not np.isfinite([fx, fy, cx, cy]).all() or fx <= 0 or fy <= 0:
        raise ValueError('invalid_camera_intrinsics')
    code, scale = encodings[image.encoding]
    dtype = np.dtype(('>' if image.is_bigendian else '<')+code)
    if image.step < image.width*dtype.itemsize or len(image.data) < image.step*image.height:
        raise ValueError('invalid_depth_stride_or_length')
    if image.width == 0 or image.height == 0:
        raise ValueError('empty_depth_image')
    depth = np.ndarray((image.height, image.width), dtype=dtype, buffer=image.data,
                       strides=(image.step, dtype.itemsize))[::stride, ::stride].astype(float)*scale
    v, u = np.mgrid[0:image.height:stride, 0:image.width:stride]
    valid = np.isfinite(depth) & (depth >= min_range) & (depth <= max_range)
    z = depth[valid]
    xyz = np.column_stack(((u[valid]-cx)*z/fx, (v[valid]-cy)*z/fy, z))
    return xyz, float(valid.sum()/valid.size), int(valid.size)


class Option20Depth(Node):
    def __init__(self):
        super().__init__('option20_depth')
        defaults = {'image_topic': '/camera/camera/depth/image_rect_raw',
                    'info_topic': '/camera/camera/depth/camera_info',
                    'stride': 4, 'min_range': 0.2, 'max_range': 5.0,
                    'max_age': 0.5, 'max_info_age': 2.0,
                    'min_valid_fraction': 0.05, 'min_valid_points': 30,
                    'obstacle_min_height': 0.12, 'obstacle_max_height': 2.3}
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {k: self.get_parameter(k).value for k in defaults}
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.info, self.pending, self.last = None, None, 0.
        self.pub = self.create_publisher(PointCloud2, '/option20/depth_obstacles', 10)
        self.clearing_pub = self.create_publisher(PointCloud2, '/option20/depth_clearing', 10)
        self.status_pub = self.create_publisher(String, '/option20/depth_status', 10)
        self.create_subscription(CameraInfo, self.p['info_topic'], self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, self.p['image_topic'], self.on_image, qos_profile_sensor_data)
        self.create_timer(0.02, self.flush)

    def on_info(self, msg):
        self.info = msg

    def on_image(self, msg):
        self.pending = msg

    def flush(self):
        msg = self.pending
        if msg is None:
            return
        stamp = stamp_seconds(msg.header.stamp)
        status = {'stamp': stamp, 'stamp_sec': stamp, 'healthy': False,
                  'valid': False, 'valid_fraction': 0., 'valid_ratio': 0., 'reason': ''}
        try:
            now = self.get_clock().now().nanoseconds*1e-9
            if not fresh_stamp(stamp, now, self.p['max_age']):
                raise ValueError('stale_or_future_depth')
            if stamp <= self.last:
                raise ValueError('out_of_order_depth')
            if self.info is None:
                raise ValueError('missing_camera_info')
            if abs(stamp-stamp_seconds(self.info.header.stamp)) > self.p['max_info_age']:
                raise ValueError('stale_camera_info')
            points, ratio, samples = depth_xyz(
                msg, self.info, self.p['stride'], self.p['min_range'], self.p['max_range'])
            status.update(valid_fraction=ratio, valid_ratio=ratio,
                          points=len(points), samples=samples)
            if ratio < self.p['min_valid_fraction'] or len(points) < self.p['min_valid_points']:
                raise ValueError('insufficient_valid_depth')
            points = transform_points(self.buffer, points, msg.header.frame_id, msg.header.stamp)
            obstacles = obstacle_points(points, self.p['obstacle_min_height'],
                                        self.p['obstacle_max_height'])
            self.pub.publish(xyz_cloud(obstacles, msg.header.stamp))
            ground = clearing_points(points, self.p['max_range'])
            self.clearing_pub.publish(xyz_cloud(ground, msg.header.stamp))
            status.update(healthy=True, valid=True, obstacle_points=len(obstacles),
                          clearing_points=len(ground))
            self.last = stamp
        except TransformException:
            return
        except (ValueError, TypeError) as exc:
            status['reason'] = str(exc)
        self.status_pub.publish(String(data=json.dumps(status)))
        self.pending = None


def main():
    rclpy.init()
    node = Option20Depth()
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
