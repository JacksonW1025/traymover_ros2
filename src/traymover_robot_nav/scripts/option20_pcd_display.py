#!/usr/bin/env python3
"""Publish display-only PCD in precisely the option 20 occupancy-map frame."""
import json
from pathlib import Path

import numpy as np
import open3d as o3d
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


class PcdDisplay(Node):
    def __init__(self):
        super().__init__('option20_pcd_display')
        defaults = {'pcd_path': '', 'map_manifest': '', 'floor_z': 0.0,
                    'voxel_size': .20, 'max_points': 200000,
                    'topic': '/option20/pcd_map', 'frame_id': 'map'}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        params = {name: self.get_parameter(name).value for name in defaults}
        transform = np.eye(4)
        transform[2, 3] = -params['floor_z']
        if params['map_manifest']:
            manifest = json.loads(Path(params['map_manifest']).read_text())
            manifest_path = Path(manifest['pcd_path']).resolve(strict=True)
            if params['pcd_path'] and Path(params['pcd_path']).resolve() != manifest_path:
                raise ValueError('display PCD differs from prepared-map manifest')
            params['pcd_path'] = str(manifest_path)
            stat = manifest_path.stat()
            if (stat.st_size != manifest['pcd_size']
                    or stat.st_mtime_ns != manifest['pcd_mtime_ns']):
                raise ValueError('PCD changed after map preparation')
            transform = np.asarray(manifest['original_to_map'], dtype=float)
        if (transform.shape != (4, 4) or not np.isfinite(transform).all()
                or not np.isfinite(params['voxel_size']) or params['voxel_size'] <= 0
                or params['max_points'] < 1):
            raise ValueError('invalid display transform or point sampling parameters')
        cloud = o3d.io.read_point_cloud(params['pcd_path'])
        cloud.transform(transform)
        cloud = cloud.voxel_down_sample(params['voxel_size'])
        points = np.asarray(cloud.points)
        if not len(points) or not np.isfinite(points).all():
            raise ValueError('display PCD has no valid points')
        stride = max(1, int(np.ceil(len(points) / params['max_points'])))
        points = np.asarray(points[::stride], dtype='<f4')
        self.message = PointCloud2()
        self.message.header.frame_id = params['frame_id']
        self.message.height = 1
        self.message.width = len(points)
        self.message.fields = [PointField(name=name, offset=i * 4,
                                          datatype=PointField.FLOAT32, count=1)
                               for i, name in enumerate(('x', 'y', 'z'))]
        self.message.is_bigendian = False
        self.message.point_step = 12
        self.message.row_step = len(points) * 12
        self.message.is_dense = True
        self.message.data = points.tobytes()
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(PointCloud2, params['topic'], qos)
        self.timer = self.create_timer(5., self.publish_cloud)
        self.get_logger().info(f'Display map loaded: {len(points)} points')
        self.publish_cloud()

    def publish_cloud(self):
        self.message.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.message)


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = SingleThreadedExecutor()
    try:
        node = PcdDisplay()
        executor.add_node(node)
        if rclpy.ok():
            executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
