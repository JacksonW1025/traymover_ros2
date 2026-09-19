#!/usr/bin/env python3
"""Separate structure-only virtual scan from full collision-height lidar points."""

import json
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformException, TransformListener

from option20_odom import rotation_matrix


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec*1e-9


def fresh_stamp(stamp, now, max_age):
    return stamp > 0 and -0.10 <= now-stamp <= max_age


def cloud_xyz(msg):
    """Read padded, organized, big/little endian clouds without Python point loops."""
    formats = {PointField.FLOAT32: 'f4', PointField.FLOAT64: 'f8'}
    fields = {f.name: f for f in msg.fields}
    if not all(name in fields and fields[name].datatype in formats for name in ('x', 'y', 'z')):
        raise ValueError('missing_float_xyz')
    if msg.point_step <= 0 or msg.row_step < msg.width*msg.point_step:
        raise ValueError('invalid_cloud_stride')
    if len(msg.data) < msg.height*msg.row_step:
        raise ValueError('truncated_cloud')
    endian = '>' if msg.is_bigendian else '<'
    columns = []
    for name in ('x', 'y', 'z'):
        field = fields[name]
        dtype = np.dtype(endian+formats[field.datatype])
        if field.offset < 0 or field.offset+dtype.itemsize > msg.point_step:
            raise ValueError('invalid_field_offset')
        values = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                            offset=field.offset, strides=(msg.row_step, msg.point_step))
        columns.append(values.reshape(-1))
    return np.column_stack(columns).astype(np.float32)


def xyz_cloud(points, stamp, frame='base_footprint'):
    points = np.asarray(points, dtype='<f4').reshape(-1, 3)
    msg = PointCloud2()
    msg.header = Header(stamp=stamp, frame_id=frame)
    msg.height, msg.width = 1, len(points)
    msg.fields = [PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1)
                  for i, name in enumerate(('x', 'y', 'z'))]
    msg.is_bigendian, msg.is_dense = False, True
    msg.point_step, msg.row_step = 12, 12*len(points)
    msg.data = points.tobytes()
    return msg


def self_mask(points):
    """Identify the URDF body's actual volumes with 2 cm reflection padding.

    Coordinates are in base_footprint (base_link z=0.20). In particular the
    elevated sensor overhang must not erase low obstacles ahead of the chassis.
    """
    points = np.asarray(points).reshape(-1, 3)
    x, y, z = points.T
    # Chassis box: center z=.19, size .80 x .55 x .24.
    own_body = ((np.abs(x) <= 0.42) & (np.abs(y) <= 0.295)
                & (z >= 0.05) & (z <= 0.33))
    # Mast: center (.375, 0, .91), size .05 x .155 x 1.20.
    own_body |= ((x >= 0.33) & (x <= 0.42) & (np.abs(y) <= 0.0975)
                 & (z >= 0.29) & (z <= 1.53))
    # LiDAR cylinder: center (.435, 0, 1.565), radius .06, height .11.
    own_body |= ((np.hypot(x-0.435, y) <= 0.08) & (z >= 1.49) & (z <= 1.64))
    # Camera shelf and camera casing, both narrowly bounded in height.
    own_body |= ((x >= 0.38) & (x <= 0.52) & (np.abs(y) <= 0.06)
                 & (z >= 0.92) & (z <= 0.97))
    own_body |= ((x >= 0.4175) & (x <= 0.4825) & (np.abs(y) <= 0.07)
                 & (z >= 0.9275) & (z <= 0.9925))
    return own_body


def obstacle_points(points, min_height=0.12, max_height=2.3):
    points = np.asarray(points).reshape(-1, 3)
    keep = (np.isfinite(points).all(axis=1) & ~self_mask(points)
            & (points[:, 2] >= min_height) & (points[:, 2] <= max_height))
    return points[keep]


def clearing_points(points, max_range=50.0):
    """Only measured near-ground returns may clear; never invent free-space rays.

    Exclude the obstacle band's lower boundary so a 12 cm return cannot both
    mark and clear a cell. High wall/ceiling returns do not prove floor clearance.
    """
    points = np.asarray(points).reshape(-1, 3)
    distance = np.linalg.norm(points[:, :2], axis=1)
    keep = (np.isfinite(points).all(axis=1) & ~self_mask(points)
            & (points[:, 2] >= -0.10) & (points[:, 2] < 0.12)
            & (distance >= 0.2) & (distance <= max_range))
    return points[keep]


def virtual_scan(points, bands, angle_increment=math.pi/360, range_min=0.5,
                 range_max=50., origin_x=0.435):
    """Union of height bands, nearest real return per azimuth; absent bins stay inf."""
    points = np.asarray(points).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    x, y = points[:, 0]-origin_x, points[:, 1]
    distance = np.hypot(x, y)
    count = int(round(2*math.pi/angle_increment))
    increment = 2*math.pi/count
    bins = np.floor((np.arctan2(y, x)+math.pi)/increment+0.5).astype(np.int64) % count
    valid = np.isfinite(points).all(axis=1) & (distance >= range_min) & (distance <= range_max)
    selected = np.zeros(len(points), dtype=bool)
    layer_bins = []
    for lower, upper in np.asarray(bands).reshape(-1, 2):
        layer = valid & (points[:, 2] >= lower) & (points[:, 2] <= upper)
        selected |= layer
        layer_bins.append(int(len(np.unique(bins[layer]))))
    ranges = np.full(count, np.inf, dtype=np.float32)
    np.minimum.at(ranges, bins[selected], distance[selected])
    stats = {'points': int(selected.sum()), 'bins': int(np.isfinite(ranges).sum()),
             'count': count, 'band_bins': layer_bins}
    return ranges, increment, stats


def transform_points(buffer, points, source, stamp, target='base_footprint'):
    if not source:
        raise ValueError('empty_frame')
    if source == target:
        return points
    # Exact sensor time only. Never replace an unavailable transform with latest TF.
    transform = buffer.lookup_transform(target, source, Time.from_msg(stamp)).transform
    q, t = transform.rotation, transform.translation
    return points @ rotation_matrix([q.x, q.y, q.z, q.w]).T + [t.x, t.y, t.z]


class Option20Clouds(Node):
    def __init__(self):
        super().__init__('option20_clouds')
        defaults = {'scan_input': '/cloud_registered', 'lidar_input': '/point_cloud_raw',
                    'height_bands': [1.1, 1.5, 1.5, 1.9, 1.9, 2.3],
                    'angle_increment': math.pi/360, 'range_min': 0.5, 'range_max': 50.,
                    'origin_x': 0.435, 'max_age': 0.5, 'min_scan_bins': 30,
                    'obstacle_min_height': 0.12, 'obstacle_max_height': 2.3}
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {k: self.get_parameter(k).value for k in defaults}
        bands = np.asarray(self.p['height_bands'])
        if bands.size == 0 or bands.size % 2 or not np.isfinite(bands).all():
            raise ValueError('height_bands must contain finite lower/upper pairs')
        if np.any(bands.reshape(-1, 2)[:, 0] >= bands.reshape(-1, 2)[:, 1]):
            raise ValueError('height band lower bound must be below upper bound')
        if not 0 < self.p['angle_increment'] <= math.pi or not 0 < self.p['range_min'] < self.p['range_max']:
            raise ValueError('invalid scan angles or ranges')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.pending = {}
        self.last = {}
        self.scan_pub = self.create_publisher(LaserScan, '/scan_localization', 10)
        self.lidar_pub = self.create_publisher(PointCloud2, '/option20/lidar_obstacles', 10)
        self.clearing_pub = self.create_publisher(PointCloud2, '/option20/lidar_clearing', 10)
        self.status_pubs = {kind: self.create_publisher(String, '/option20/'+kind+'_status', 10)
                            for kind in ('scan', 'lidar')}
        for kind, key in (('scan', 'scan_input'), ('lidar', 'lidar_input')):
            self.create_subscription(PointCloud2, self.p[key],
                                     lambda msg, kind=kind: self.receive(kind, msg),
                                     qos_profile_sensor_data)
        self.create_timer(0.02, self.flush)

    def receive(self, kind, msg):
        self.pending[kind] = msg

    def flush(self):
        now = self.get_clock().now().nanoseconds*1e-9
        for kind, msg in list(self.pending.items()):
            stamp = stamp_seconds(msg.header.stamp)
            status = {'stamp': stamp, 'stamp_sec': stamp, 'healthy': False,
                      'valid': False, 'reason': ''}
            try:
                if not fresh_stamp(stamp, now, self.p['max_age']):
                    raise ValueError('stale_or_future_cloud')
                if stamp <= self.last.get(kind, 0.):
                    raise ValueError('out_of_order_cloud')
                points = cloud_xyz(msg)
                points = points[np.isfinite(points).all(axis=1)]
                points = transform_points(self.buffer, points, msg.header.frame_id, msg.header.stamp)
                if kind == 'scan':
                    ranges, increment, stats = virtual_scan(
                        points, self.p['height_bands'], self.p['angle_increment'],
                        self.p['range_min'], self.p['range_max'], self.p['origin_x'])
                    scan = LaserScan()
                    scan.header = Header(stamp=msg.header.stamp, frame_id='option20_scan')
                    scan.angle_min, scan.angle_increment = -math.pi, increment
                    scan.angle_max = scan.angle_min+(len(ranges)-1)*increment
                    scan.range_min, scan.range_max = self.p['range_min'], self.p['range_max']
                    scan.scan_time = float(stamp-self.last[kind]) if kind in self.last else 0.1
                    scan.ranges = ranges.tolist()
                    self.scan_pub.publish(scan)
                    status.update(stats)
                    healthy = stats['bins'] >= self.p['min_scan_bins']
                else:
                    obstacles = obstacle_points(points, self.p['obstacle_min_height'],
                                                self.p['obstacle_max_height'])
                    self.lidar_pub.publish(xyz_cloud(obstacles, msg.header.stamp))
                    ground = clearing_points(points, self.p['range_max'])
                    self.clearing_pub.publish(xyz_cloud(ground, msg.header.stamp))
                    status.update(points=len(points), obstacle_points=len(obstacles),
                                  clearing_points=len(ground))
                    healthy = len(points) >= 30
                self.last[kind] = stamp
                status.update(healthy=healthy, valid=healthy,
                              reason='' if healthy else 'insufficient_returns')
            except TransformException:
                # Let the executor receive odometry TF, retry until original data expires.
                continue
            except (ValueError, TypeError) as exc:
                status['reason'] = str(exc)
            self.status_pubs[kind].publish(String(data=json.dumps(status)))
            del self.pending[kind]


def main():
    rclpy.init()
    node = Option20Clouds()
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
