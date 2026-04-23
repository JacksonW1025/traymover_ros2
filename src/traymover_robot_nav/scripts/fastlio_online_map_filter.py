#!/usr/bin/env python3
"""Online filtered map accumulator for FAST-LIO.

This node keeps FAST-LIO's odometry / scan matching untouched. Instead it
subscribes to the registered world-frame scan output and builds a second,
filtered map for saving to PCD:

- points inside a configurable body-fixed rear exclusion box are dropped to
  avoid recording a trailing operator
- only voxels observed repeatedly across multiple scans are promoted into the
  saved map, which suppresses transient pedestrians moving through the scene

The node publishes the stable filtered map as PointCloud2 and exposes a
Trigger service that writes the current filtered map to a fixed staging path.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import sys
from typing import Dict, Tuple

import numpy as np
try:
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.duration import Duration
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2, PointField
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header
    from std_srvs.srv import Trigger

    ROS_AVAILABLE = True
except ModuleNotFoundError:
    rclpy = None
    Odometry = None
    Duration = None
    ExternalShutdownException = Exception
    Node = object
    PointCloud2 = None
    PointField = None
    point_cloud2 = None
    Header = None
    Trigger = None
    ROS_AVAILABLE = False


if ROS_AVAILABLE:
    XYZI_FIELDS = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]

    ROS_TO_NUMPY_DTYPES = {
        PointField.INT8: np.int8,
        PointField.UINT8: np.uint8,
        PointField.INT16: np.int16,
        PointField.UINT16: np.uint16,
        PointField.INT32: np.int32,
        PointField.UINT32: np.uint32,
        PointField.FLOAT32: np.float32,
        PointField.FLOAT64: np.float64,
    }
else:
    XYZI_FIELDS = []
    ROS_TO_NUMPY_DTYPES = {}


@dataclass
class PoseSample:
    stamp_sec: float
    translation: np.ndarray
    rotation_body_to_world: np.ndarray


@dataclass
class VoxelStats:
    hits: int
    first_seen_sec: float
    last_seen_sec: float
    sum_x: float
    sum_y: float
    sum_z: float
    sum_intensity: float


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


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


def build_structured_dtype(msg: PointCloud2, field_names: Tuple[str, ...]) -> np.dtype:
    names = []
    formats = []
    offsets = []
    field_by_name = {field.name: field for field in msg.fields}

    for name in field_names:
        if name not in field_by_name:
            raise KeyError(f'PointCloud2 missing required field {name!r}')
        field = field_by_name[name]
        if field.datatype not in ROS_TO_NUMPY_DTYPES:
            raise KeyError(f'unsupported PointCloud2 datatype {field.datatype} for field {name!r}')
        numpy_dtype = ROS_TO_NUMPY_DTYPES[field.datatype]
        names.append(name)
        offsets.append(field.offset)
        if field.count == 1:
            formats.append(numpy_dtype)
        else:
            formats.append((numpy_dtype, field.count))

    return np.dtype({
        'names': names,
        'formats': formats,
        'offsets': offsets,
        'itemsize': msg.point_step,
    })


def pointcloud2_to_xyzi(msg: PointCloud2) -> np.ndarray:
    dtype = build_structured_dtype(msg, ('x', 'y', 'z', 'intensity'))
    cloud = np.frombuffer(msg.data, dtype=dtype, count=msg.width * msg.height)

    if msg.is_bigendian != (sys.byteorder == 'big'):
        cloud = cloud.byteswap().newbyteorder()

    xyz = np.column_stack((
        cloud['x'].astype(np.float32, copy=False),
        cloud['y'].astype(np.float32, copy=False),
        cloud['z'].astype(np.float32, copy=False),
        cloud['intensity'].astype(np.float32, copy=False),
    ))
    finite_mask = np.isfinite(xyz).all(axis=1)
    return xyz[finite_mask]


def apply_body_box_exclusion(
    points_xyzi_world: np.ndarray,
    pose: PoseSample | None,
    pose_timeout_sec: float,
    stamp_sec: float,
    min_x: float,
    max_x: float,
    half_width: float,
    min_z: float,
    max_z: float,
) -> np.ndarray:
    if pose is None:
        return points_xyzi_world
    if pose_timeout_sec > 0.0 and abs(stamp_sec - pose.stamp_sec) > pose_timeout_sec:
        return points_xyzi_world

    points_world = points_xyzi_world[:, :3]
    points_body = (points_world - pose.translation) @ pose.rotation_body_to_world
    inside_mask = (
        (points_body[:, 0] >= min_x) &
        (points_body[:, 0] <= max_x) &
        (np.abs(points_body[:, 1]) <= half_width) &
        (points_body[:, 2] >= min_z) &
        (points_body[:, 2] <= max_z)
    )
    return points_xyzi_world[~inside_mask]


class StableVoxelMap:
    def __init__(
        self,
        voxel_size: float,
        min_hits: int,
        min_observation_span_sec: float,
        candidate_ttl_sec: float,
    ) -> None:
        self.voxel_size = float(voxel_size)
        self.min_hits = int(min_hits)
        self.min_observation_span_sec = float(min_observation_span_sec)
        self.candidate_ttl_sec = float(candidate_ttl_sec)
        self._voxels: Dict[Tuple[int, int, int], VoxelStats] = {}

    def update_scan(self, points_xyzi: np.ndarray, stamp_sec: float) -> int:
        if points_xyzi.size == 0:
            self.prune(stamp_sec)
            return 0

        points = points_xyzi[:, :3]
        intensity = points_xyzi[:, 3]
        voxel_coords = np.floor(points / self.voxel_size).astype(np.int32)
        unique_coords, inverse = np.unique(voxel_coords, axis=0, return_inverse=True)

        counts = np.bincount(inverse)
        sum_x = np.bincount(inverse, weights=points[:, 0])
        sum_y = np.bincount(inverse, weights=points[:, 1])
        sum_z = np.bincount(inverse, weights=points[:, 2])
        sum_intensity = np.bincount(inverse, weights=intensity)

        centroids = np.column_stack((
            sum_x / counts,
            sum_y / counts,
            sum_z / counts,
            sum_intensity / counts,
        )).astype(np.float32, copy=False)

        for idx, coord in enumerate(unique_coords):
            key = (int(coord[0]), int(coord[1]), int(coord[2]))
            stats = self._voxels.get(key)
            if stats is None:
                self._voxels[key] = VoxelStats(
                    hits=1,
                    first_seen_sec=stamp_sec,
                    last_seen_sec=stamp_sec,
                    sum_x=float(centroids[idx, 0]),
                    sum_y=float(centroids[idx, 1]),
                    sum_z=float(centroids[idx, 2]),
                    sum_intensity=float(centroids[idx, 3]),
                )
                continue

            stats.hits += 1
            stats.last_seen_sec = stamp_sec
            stats.sum_x += float(centroids[idx, 0])
            stats.sum_y += float(centroids[idx, 1])
            stats.sum_z += float(centroids[idx, 2])
            stats.sum_intensity += float(centroids[idx, 3])

        self.prune(stamp_sec)
        return int(unique_coords.shape[0])

    def prune(self, now_sec: float) -> None:
        if self.candidate_ttl_sec <= 0.0:
            return

        stale_keys = [
            key
            for key, stats in self._voxels.items()
            if not self._is_stable(stats)
            and now_sec - stats.last_seen_sec > self.candidate_ttl_sec
        ]
        for key in stale_keys:
            del self._voxels[key]

    def stable_cloud(self) -> np.ndarray:
        stable_items = [stats for stats in self._voxels.values() if self._is_stable(stats)]
        if not stable_items:
            return np.empty((0, 4), dtype=np.float32)

        cloud = np.empty((len(stable_items), 4), dtype=np.float32)
        for idx, stats in enumerate(stable_items):
            inv_hits = 1.0 / float(stats.hits)
            cloud[idx, 0] = stats.sum_x * inv_hits
            cloud[idx, 1] = stats.sum_y * inv_hits
            cloud[idx, 2] = stats.sum_z * inv_hits
            cloud[idx, 3] = stats.sum_intensity * inv_hits
        return cloud

    def total_voxels(self) -> int:
        return len(self._voxels)

    def stable_voxels(self) -> int:
        return sum(1 for stats in self._voxels.values() if self._is_stable(stats))

    def _is_stable(self, stats: VoxelStats) -> bool:
        return (
            stats.hits >= self.min_hits
            and stats.last_seen_sec - stats.first_seen_sec >= self.min_observation_span_sec
        )


def write_binary_pcd(path: Path, cloud_xyzi: np.ndarray) -> None:
    cloud_xyzi = np.asarray(cloud_xyzi, dtype=np.float32)
    if cloud_xyzi.ndim != 2 or cloud_xyzi.shape[1] != 4:
        raise ValueError(f'expected Nx4 float32 array, got shape {cloud_xyzi.shape!r}')

    path.parent.mkdir(parents=True, exist_ok=True)
    point_count = int(cloud_xyzi.shape[0])
    header = (
        '# .PCD v0.7 - Point Cloud Data file format\n'
        'VERSION 0.7\n'
        'FIELDS x y z intensity\n'
        'SIZE 4 4 4 4\n'
        'TYPE F F F F\n'
        'COUNT 1 1 1 1\n'
        f'WIDTH {point_count}\n'
        'HEIGHT 1\n'
        'VIEWPOINT 0 0 0 1 0 0 0\n'
        f'POINTS {point_count}\n'
        'DATA binary\n'
    )

    with path.open('wb') as handle:
        handle.write(header.encode('ascii'))
        handle.write(cloud_xyzi.astype(np.float32, copy=False).tobytes())


if ROS_AVAILABLE:
    class FastlioOnlineMapFilter(Node):
        def __init__(self) -> None:
            super().__init__('fastlio_online_map_filter')

            self.declare_parameter('input_topic', '/cloud_registered')
            self.declare_parameter('odom_topic', '/odom')
            self.declare_parameter('output_topic', '/filtered_map')
            self.declare_parameter('map_file_path', '/tmp/traymover_filtered.pcd')
            self.declare_parameter('voxel_size', 0.15)
            self.declare_parameter('min_observations', 3)
            self.declare_parameter('min_observation_span_sec', 0.25)
            self.declare_parameter('candidate_ttl_sec', 2.0)
            self.declare_parameter('publish_period_sec', 1.0)
            self.declare_parameter('pose_timeout_sec', 0.30)
            self.declare_parameter('rear_filter_enabled', True)
            self.declare_parameter('rear_filter_min_x', -1.25)
            self.declare_parameter('rear_filter_max_x', -0.35)
            self.declare_parameter('rear_filter_half_width', 0.90)
            self.declare_parameter('rear_filter_min_z', -0.50)
            self.declare_parameter('rear_filter_max_z', 2.00)

            self.input_topic = str(self.get_parameter('input_topic').value)
            self.odom_topic = str(self.get_parameter('odom_topic').value)
            self.output_topic = str(self.get_parameter('output_topic').value)
            self.map_file_path = Path(str(self.get_parameter('map_file_path').value))
            self.pose_timeout_sec = float(self.get_parameter('pose_timeout_sec').value)
            self.rear_filter_enabled = bool(self.get_parameter('rear_filter_enabled').value)
            self.rear_filter_min_x = float(self.get_parameter('rear_filter_min_x').value)
            self.rear_filter_max_x = float(self.get_parameter('rear_filter_max_x').value)
            self.rear_filter_half_width = float(self.get_parameter('rear_filter_half_width').value)
            self.rear_filter_min_z = float(self.get_parameter('rear_filter_min_z').value)
            self.rear_filter_max_z = float(self.get_parameter('rear_filter_max_z').value)

            self.voxel_map = StableVoxelMap(
                voxel_size=float(self.get_parameter('voxel_size').value),
                min_hits=int(self.get_parameter('min_observations').value),
                min_observation_span_sec=float(self.get_parameter('min_observation_span_sec').value),
                candidate_ttl_sec=float(self.get_parameter('candidate_ttl_sec').value),
            )

            sensor_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            )
            latched_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )

            self.latest_pose: PoseSample | None = None
            self.last_frame_id = 'camera_init'
            self._scans_seen = 0
            self._last_missing_pose_warn = self.get_clock().now() - Duration(seconds=10.0)

            self.create_subscription(Odometry, self.odom_topic, self.on_odom, sensor_qos)
            self.create_subscription(PointCloud2, self.input_topic, self.on_cloud, sensor_qos)
            self.map_pub = self.create_publisher(PointCloud2, self.output_topic, latched_qos)
            self.create_service(Trigger, 'save_filtered_map', self.on_save_map)

            publish_period_sec = float(self.get_parameter('publish_period_sec').value)
            self.create_timer(publish_period_sec, self.publish_map)

            self.get_logger().info(
                f'filtering {self.input_topic} with voxel_size={self.voxel_map.voxel_size:.2f}, '
                f'min_observations={self.voxel_map.min_hits}, '
                f'min_span={self.voxel_map.min_observation_span_sec:.2f}s, '
                f'candidate_ttl={self.voxel_map.candidate_ttl_sec:.2f}s, '
                f'rear_filter={"on" if self.rear_filter_enabled else "off"} -> '
                f'{self.map_file_path.as_posix()}'
            )

        def on_odom(self, msg: Odometry) -> None:
            pose = msg.pose.pose
            rotation = quaternion_to_matrix(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )
            self.latest_pose = PoseSample(
                stamp_sec=stamp_to_sec(msg.header.stamp),
                translation=np.array([
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                ], dtype=np.float32),
                rotation_body_to_world=rotation,
            )

        def on_cloud(self, msg: PointCloud2) -> None:
            try:
                points_xyzi = pointcloud2_to_xyzi(msg)
            except KeyError as exc:
                self.get_logger().error(str(exc))
                return

            stamp_sec = stamp_to_sec(msg.header.stamp)
            self.last_frame_id = msg.header.frame_id or self.last_frame_id

            if self.rear_filter_enabled:
                if self.latest_pose is None:
                    now = self.get_clock().now()
                    if now - self._last_missing_pose_warn > Duration(seconds=2.0):
                        self._last_missing_pose_warn = now
                        self.get_logger().warn('no /odom yet; rear exclusion box disabled until a pose arrives')
                points_xyzi = apply_body_box_exclusion(
                    points_xyzi_world=points_xyzi,
                    pose=self.latest_pose,
                    pose_timeout_sec=self.pose_timeout_sec,
                    stamp_sec=stamp_sec,
                    min_x=self.rear_filter_min_x,
                    max_x=self.rear_filter_max_x,
                    half_width=self.rear_filter_half_width,
                    min_z=self.rear_filter_min_z,
                    max_z=self.rear_filter_max_z,
                )

            per_scan_voxels = self.voxel_map.update_scan(points_xyzi, stamp_sec)
            self._scans_seen += 1

            if self._scans_seen % 20 == 0:
                self.get_logger().info(
                    f'processed {self._scans_seen} scans, '
                    f'scan_voxels={per_scan_voxels}, '
                    f'candidate_voxels={self.voxel_map.total_voxels()}, '
                    f'stable_voxels={self.voxel_map.stable_voxels()}'
                )

        def publish_map(self) -> None:
            stable_cloud = self.voxel_map.stable_cloud()
            if stable_cloud.size == 0:
                return

            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = self.last_frame_id
            self.map_pub.publish(point_cloud2.create_cloud(header, XYZI_FIELDS, stable_cloud))

        def on_save_map(self, _request, response):
            stable_cloud = self.voxel_map.stable_cloud()
            if stable_cloud.size == 0:
                response.success = False
                response.message = 'filtered map is empty; move the robot and try again'
                return response

            write_binary_pcd(self.map_file_path, stable_cloud)
            response.success = True
            response.message = (
                f'wrote {self.map_file_path} with {stable_cloud.shape[0]} stable voxels '
                f'(candidate_voxels={self.voxel_map.total_voxels()})'
            )
            self.get_logger().info(response.message)
            return response


def main() -> None:
    if not ROS_AVAILABLE:
        raise RuntimeError('ROS 2 Python dependencies are unavailable; source the ROS workspace before running')
    rclpy.init()
    node = FastlioOnlineMapFilter()
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
