#!/usr/bin/env python3
"""Confirm obstacle geometry once for local planning and chassis motion checks."""
from collections import deque
from dataclasses import dataclass
import math
import time

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from scipy.ndimage import distance_transform_edt
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker

from option20_clouds import (
    cloud_xyz, fresh_stamp, self_mask, stamp_seconds, transform_points, xyz_cloud,
)
from option20_depth import depth_xyz
from option20_odom import rotation_matrix


HALF_LENGTH = 0.40
HALF_WIDTH = 0.275
SENSOR_FRONT = 0.50
SENSOR_HALF_WIDTH = 0.11
LIDAR_FORWARD_OFFSET = 0.435
SENSOR_TIMEOUT = 0.40
AUTO_ESTOP_TIMEOUT = 0.30
ODOM_TIMEOUT = 0.30
COMMAND_TIMEOUT = 0.25
PLANNER_TIMEOUT = 2.0
CONTROL_PERIOD = 0.05
ZERO = (0.0, 0.0)
EMPTY = np.empty((0, 3))


def clearances(points, x=0.0, y=0.0, yaw=0.0):
    dx, dy = points[:, 0] - x, points[:, 1] - y
    c, s = math.cos(yaw), math.sin(yaw)
    forward, lateral = c * dx + s * dy, np.abs(c * dy - s * dx)
    chassis = np.hypot(np.maximum(np.abs(forward) - HALF_LENGTH, 0.0),
                       np.maximum(lateral - HALF_WIDTH, 0.0))
    sensor = np.hypot(np.maximum(np.maximum(HALF_LENGTH - forward, forward - SENSOR_FRONT), 0),
                      np.maximum(lateral - SENSOR_HALF_WIDTH, 0.0))
    return np.minimum(chassis, sensor)


def robot_returns(points):
    """Include a small allowance for reflections around the deck and sensor mounts."""
    x, y, z = points.T
    deck = ((np.abs(x) <= 0.43) & (np.abs(y) <= 0.30) & (z <= 0.35))
    mounts = ((x >= 0.33) & (x <= 0.53) & (np.abs(y) <= 0.13)
              & (z >= 0.92) & (z <= 1.66))
    return self_mask(points) | deck | mounts


def voxel_points(points, size):
    x, y = np.floor(points[:, :2] / size).astype(np.int64).T
    keys = (x << 32) | (y & 0xffffffff)
    _, index = np.unique(keys, return_index=True)
    return points[np.sort(index)]


@dataclass(frozen=True)
class ClusterFilter:
    cell: float
    radius: float
    minimum: int
    span: float

    def apply(self, points):
        """Keep connected clusters with a dense core and measurable XY extent."""
        if len(points) < self.minimum:
            return EMPTY
        points = voxel_points(points, self.cell)
        if len(points) < self.minimum:
            return EMPTY
        xy = points[:, :2]
        pairs = cKDTree(xy).query_pairs(self.radius, output_type='ndarray')
        if len(pairs) < self.minimum - 1:
            return EMPTY
        graph = coo_matrix((np.ones(len(pairs), dtype=np.uint8), pairs.T),
                           shape=(len(xy), len(xy))).tocsr()
        count, labels = connected_components(graph, directed=False)
        neighbors = np.bincount(pairs.ravel(), minlength=len(xy)) + 1
        dense = np.zeros(count, dtype=bool)
        dense[labels[neighbors >= self.minimum]] = True
        lower, upper = np.full((count, 2), np.inf), np.full((count, 2), -np.inf)
        np.minimum.at(lower, labels, xy)
        np.maximum.at(upper, labels, xy)
        extent = np.linalg.norm(upper - lower, axis=1)
        return points[(dense & (extent >= self.span))[labels]]


class Confirmation:
    """Compare successive observations in odom and briefly retain confirmed hits."""
    def __init__(self, cluster_filter, radius, hold):
        self.cluster_filter = cluster_filter
        self.radius = radius
        self.hold = hold
        self.previous = EMPTY
        self.stamp = -math.inf
        self.recent = deque()

    def update(self, stamp, points, usable=True):
        confirmed = EMPTY
        if 0 < stamp - self.stamp <= SENSOR_TIMEOUT and len(self.previous) and len(points):
            distance, _ = cKDTree(self.previous[:, :2]).query(
                points[:, :2], distance_upper_bound=self.radius)
            matched = np.isfinite(distance)
            # Partial matches need enough support of their own. One shared
            # point must not confirm an otherwise new cluster.
            confirmed = points if matched.all() else self.cluster_filter.apply(points[matched])
        if usable:
            self.previous, self.stamp = points, stamp
            if len(confirmed):
                self.recent.append((stamp, confirmed))
        self.expire(stamp)
        return confirmed

    def expire(self, stamp):
        while self.recent and stamp - self.recent[0][0] > self.hold:
            self.recent.popleft()

    def snapshot(self, stamp):
        self.expire(stamp)
        return (np.concatenate([points for _, points in reversed(self.recent)])
                if self.recent else EMPTY)


def front_returns(points, fov_deg=None):
    x, y = points[:, :2].T
    width = (HALF_WIDTH if fov_deg is None else
             (x - LIDAR_FORWARD_OFFSET) * math.tan(math.radians(fov_deg / 2)))
    return (x >= HALF_LENGTH) & (np.abs(y) <= width)


def detour_trigger(front, distance, threshold):
    return front & (distance <= threshold)


def planner_cloud(points, trigger, stamp):
    """Keep trigger eligibility attached to each confirmed point."""
    msg = xyz_cloud(EMPTY, stamp, 'map')
    msg.fields.append(PointField(
        name='detour_trigger', offset=12, datatype=PointField.FLOAT32, count=1))
    msg.width, msg.point_step = len(points), 16
    msg.row_step = msg.width * msg.point_step
    msg.data = np.column_stack((points, trigger)).astype('<f4').tobytes()
    return msg


def ramp_command(previous, target, elapsed, linear_rate, angular_rate):
    if target == ZERO:
        return ZERO
    if elapsed > COMMAND_TIMEOUT:
        previous = ZERO
    dt = min(elapsed, CONTROL_PERIOD)
    if dt <= 0.0:
        return previous
    dv, dw = target[0] - previous[0], target[1] - previous[1]
    # Advance both components together to keep the requested curvature from rest.
    fraction = 1.0 / max(1.0, abs(dv) / (linear_rate * dt), abs(dw) / (angular_rate * dt))
    if fraction == 1.0:
        return target
    return previous[0] + fraction * dv, previous[1] + fraction * dw


def approach_speed_limit(clearance, cfg):
    distance = clearance - cfg['detour_approach_distance_m']
    fraction = min(1.0, max(0.0, distance / cfg['detour_approach_ramp_m']))
    slow, cruise = cfg['detour_approach_speed_mps'], cfg['cruise_linear_speed_mps']
    return slow + fraction * (cruise - slow)


def motion_is_clear(points, velocity, minimum):
    """Project the footprint during reaction and braking, including turns."""
    linear, angular = velocity
    if velocity == ZERO or not len(points):
        return True
    horizon = 0.25 + max(abs(linear) / 0.50, abs(angular) / 0.65)
    corner_speed = abs(linear) + 0.52 * abs(angular)
    steps = max(1, math.ceil(horizon * corner_speed / 0.025))
    margin = 0.5 * horizon * corner_speed / steps
    initial = clearances(points)
    # A point just inside the soft margin must not prevent moving away.
    limit = np.minimum(minimum + margin, initial - 1e-4)
    limit[initial <= 0.0] = 0.0
    for step in range(1, steps + 1):
        t = horizon * step / steps
        yaw = angular * t
        if abs(angular) < 1e-6:
            x, y = linear * t, 0.0
        else:
            x = linear / angular * math.sin(yaw)
            y = linear / angular * (1.0 - math.cos(yaw))
        if np.any(clearances(points, x, y, yaw) <= limit):
            return False
    return True


class MapEnvelope:
    def __init__(self, msg, wall_match=0.0):
        info = msg.info
        self.resolution = info.resolution
        grid = np.asarray(msg.data, dtype=np.int8).reshape(info.height, info.width)
        self.blocked = grid != 0
        occupied = grid >= 65
        self.wall_match = wall_match
        self.nearest_occupied = (distance_transform_edt(
            ~occupied, return_distances=False, return_indices=True)
            if wall_match > 0 and occupied.any() else None)
        p, q = info.origin.position, info.origin.orientation
        self.origin = np.array([p.x, p.y])
        self.rotation = rotation_matrix([q.x, q.y, q.z, q.w])[:2, :2]

    def matches_wall(self, points):
        if self.nearest_occupied is None:
            return np.zeros(len(points), dtype=bool)
        coordinates = (points[:, :2] - self.origin) @ self.rotation / self.resolution
        x, y = np.floor(coordinates).astype(int).T
        height, width = self.blocked.shape
        inside = (x >= 0) & (x < width) & (y >= 0) & (y < height)
        nearest = self.nearest_occupied[::-1, y[inside], x[inside]].T
        delta = coordinates[inside] - (nearest + 0.5)
        inside[inside] = np.sum(delta**2, axis=1) <= (self.wall_match / self.resolution)**2
        return inside

    def nearby(self, position, body_rotation):
        """Use occupied/unknown cell centers, including cells beyond the map."""
        center = (position - self.origin) @ self.rotation / self.resolution
        x, y = np.floor(center).astype(int)
        radius = math.ceil(1.4 / self.resolution)
        columns = np.arange(x - radius, x + radius + 1)
        rows = np.arange(y - radius, y + radius + 1)
        width, height = self.blocked.shape[1], self.blocked.shape[0]
        inside = ((rows[:, None] >= 0) & (rows[:, None] < height)
                  & (columns >= 0) & (columns < width))
        blocked = self.blocked[np.clip(rows, 0, height - 1)[:, None],
                               np.clip(columns, 0, width - 1)] | ~inside
        iy, ix = np.nonzero(blocked)
        offset = (np.column_stack((columns[ix], rows[iy])) + 0.5 - center) * self.resolution
        return offset @ self.rotation.T @ body_rotation


@dataclass
class Sample:
    stamp: float
    received: float
    data: object

    def fresh(self, ros_now, now, timeout):
        return fresh_stamp(self.stamp, ros_now, timeout) and now - self.received <= timeout


@dataclass(frozen=True)
class Decision:
    state: str
    velocity: tuple = ZERO
    estop: bool = False


def navigation_marker(stamp, estop, state):
    marker = Marker()
    marker.header.stamp = stamp
    marker.header.frame_id = 'base_link'
    marker.ns = 'option22_navigation_status'
    marker.type = Marker.TEXT_VIEW_FACING
    marker.frame_locked = True
    marker.pose.position.z = 1.70
    marker.pose.orientation.w = 1.0
    marker.scale.z = 0.25
    if estop:
        marker.text = 'E-STOP'
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1., .10, .10, 1.
    elif state in ('NO_LOCAL_DETOUR', 'DETOUR_BLOCKED'):
        marker.text = 'NO CLEAR DETOUR'
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1., .68, .10, 1.
    else:
        marker.action = Marker.DELETE
    return marker


class DetourTurn:
    def __init__(self):
        self.entry = self.completed = None

    def decision(self, entry, heading_error, measured, speed):
        if self.entry != entry:
            if abs(measured[0]) > 0.025 or abs(measured[1]) > 0.05:
                return Decision('WAITING_TO_STOP')
            self.entry = entry
        if self.completed == entry:
            return None
        if abs(heading_error) > 0.13:
            return Decision('DETOUR_TURNING', (0.0, math.copysign(speed, heading_error)))
        if abs(measured[1]) > 0.05:
            return Decision('WAITING_TO_STOP')
        self.completed = entry
        return None


CONFIG = dict(
    detour_trigger_m=0.80, lidar_detour_fov_deg=45.0, detour_approach_distance_m=0.95,
    detour_approach_speed_mps=0.16, detour_approach_ramp_m=0.25,
    mapped_wall_match_m=0.14,
    lidar_range_m=5.0, obstacle_min_height_m=0.08,
    obstacle_max_height_m=1.70, cluster_cell_m=0.035, cluster_radius_m=0.14,
    lidar_cluster_cells=4, lidar_cluster_span_m=0.10, confirmation_radius_m=0.15,
    obstacle_hold_sec=0.30, depth_min_valid_fraction=0.05, motion_clearance_m=0.08,
    cruise_linear_speed_mps=0.50,
    detour_linear_speed_mps=0.18, angular_limit_radps=0.25,
    in_place_angular_limit_radps=0.20,
    linear_accel_limit_mps2=0.55, angular_accel_limit_radps2=1.0,
)


class ObstacleGateNode(Node):
    def __init__(self):
        super().__init__('option22_obstacle_gate')
        self.cfg = {key: self.declare_parameter(key, value).value for key, value in CONFIG.items()}
        for key in ('linear_accel_limit_mps2', 'angular_accel_limit_radps2',
                    'detour_approach_ramp_m'):
            if not math.isfinite(self.cfg[key]) or self.cfg[key] <= 0.0:
                raise ValueError(f'{key} must be finite and positive')
        self.camera_estop = None
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        self.pending = {source: deque(maxlen=4) for source in ('lidar', 'depth')}
        lidar_filter = ClusterFilter(
            self.cfg['cluster_cell_m'], self.cfg['cluster_radius_m'],
            self.cfg['lidar_cluster_cells'], self.cfg['lidar_cluster_span_m'])
        self.lidar_track = Confirmation(
            lidar_filter, self.cfg['confirmation_radius_m'], self.cfg['obstacle_hold_sec'])
        self.lidar_wall_track = Confirmation(
            lidar_filter, self.cfg['confirmation_radius_m'], self.cfg['obstacle_hold_sec'])
        self.samples = {}
        self.processed_stamps = {}
        self.info = self.odom = self.command = self.manual = self.planner = None
        self.map = self.state = None
        self.detour_turn = DetourTurn()
        self.detour_heading = None
        self.chassis_estop = self.chassis_auto_estop = False
        self.latest_plan = None
        self.plan_hidden_since = 0.0
        self.displayed_plan = None
        self.displayed_status = None
        self.close_obstacle = False
        self.last_output = (time.monotonic(), ZERO)
        self.obstacle_pub = self.create_publisher(PointCloud2, '/option22/obstacles', 1)
        self.velocity_pub = self.create_publisher(Twist, '/option22/cmd_vel_safe', 1)
        self.state_pub = self.create_publisher(String, '/option22/state', 1)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.display_estop_pub = self.create_publisher(Bool, '/option22/display_estop', latched)
        self.display_auto_pub = self.create_publisher(Bool, '/option22/display_auto_estop', latched)
        self.marker_pub = self.create_publisher(Marker, '/option22/navigation_marker', latched)
        self.display_plan_pub = self.create_publisher(Path, '/option22/display_plan', latched)
        depth = self.declare_parameter('depth_topic', '/camera/camera/depth/image_rect_raw').value
        info = self.declare_parameter('camera_info_topic', '/camera/camera/depth/camera_info').value
        latest_frame = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PointCloud2, '/point_cloud_raw',
                                 self.pending['lidar'].append, latest_frame)
        self.create_subscription(Image, depth, self.pending['depth'].append, latest_frame)
        self.create_subscription(CameraInfo, info, self.on_info, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(String, '/option22/planner_state', self.on_planner, 1)
        self.create_subscription(PoseStamped, '/option22/detour_entry', self.on_detour_heading, 1)
        self.create_subscription(Twist, '/option22/cmd_vel_nav', self.on_command, 1)
        self.create_subscription(Bool, '/option22/auto_request', self.on_camera_estop, 1)
        self.create_subscription(Bool, '/traymover_estop/manual_state', self.on_manual, 1)
        self.create_subscription(Bool, '/traymover_estop/state', self.on_chassis_estop, latched)
        self.create_subscription(Bool, '/traymover_estop/auto_state', self.on_chassis_auto, latched)
        self.create_subscription(Path, '/option22/tracking_path', self.on_plan, 1)
        self.create_subscription(OccupancyGrid, '/map', self.on_map,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(CONTROL_PERIOD, self.update, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_info(self, msg):
        self.info = msg

    def on_odom(self, msg):
        self.odom = Sample(stamp_seconds(msg.header.stamp), time.monotonic(), msg)

    def on_planner(self, msg):
        if self.planner is not None and self.planner[0] == 'DETOURING' and msg.data != 'DETOURING':
            self.detour_heading = None
        self.planner = (msg.data, time.monotonic())

    def on_detour_heading(self, msg):
        if msg.header.frame_id == 'map':
            q = msg.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))
            self.detour_heading = Sample(stamp_seconds(msg.header.stamp), time.monotonic(), yaw)

    def on_command(self, msg):
        self.command = ((msg.linear.x, msg.angular.z), time.monotonic())

    def on_camera_estop(self, msg):
        self.camera_estop = (msg.data, time.monotonic())

    def on_manual(self, msg):
        self.manual = (msg.data, time.monotonic())

    def on_chassis_estop(self, msg):
        self.chassis_estop = msg.data

    def on_chassis_auto(self, msg):
        self.chassis_auto_estop = msg.data

    def on_plan(self, msg):
        self.latest_plan = Sample(stamp_seconds(msg.header.stamp), time.monotonic(), msg)

    def on_map(self, msg):
        if (msg.header.frame_id == 'map' and msg.info.resolution > 0
                and msg.info.width and msg.info.height):
            self.map = MapEnvelope(msg, self.cfg['mapped_wall_match_m'])

    def map_pose(self, ros_now):
        try:
            odom_in_map = self.buffer.lookup_transform('map', 'odom', Time())
        except TransformException:
            return None
        stamp = stamp_seconds(odom_in_map.header.stamp)
        if stamp and not fresh_stamp(stamp, ros_now, ODOM_TIMEOUT):
            return None
        q, p = odom_in_map.transform.rotation, odom_in_map.transform.translation
        return rotation_matrix([q.x, q.y, q.z, q.w]), np.array([p.x, p.y, p.z])

    def body_returns(self, msg, points):
        body = transform_points(self.buffer, points, msg.header.frame_id, msg.header.stamp)
        use = (~robot_returns(body) & (body[:, 2] >= self.cfg['obstacle_min_height_m'])
               & (body[:, 2] <= self.cfg['obstacle_max_height_m']))
        return body[use]

    def observe_depth(self, msg, stamp):
        if self.info is None or abs(stamp - stamp_seconds(self.info.header.stamp)) > 2.0:
            raise ValueError('missing or stale camera calibration')
        points, fraction, _ = depth_xyz(msg, self.info, min_range=0.001, max_range=5.0)
        # Keep navigation's depth/TF readiness check separate from automatic EStop.
        transform_points(self.buffer, EMPTY, msg.header.frame_id, msg.header.stamp)
        return fraction >= self.cfg['depth_min_valid_fraction'] and len(points) >= 30

    def observe_lidar(self, msg, stamp, map_pose):
        points = cloud_xyz(msg)
        points = points[np.isfinite(points).all(axis=1)]
        healthy = len(points) >= 30
        body = self.body_returns(msg, points)
        body = body[np.linalg.norm(body[:, :2], axis=1) <= self.cfg['lidar_range_m']]
        # Compare frames at their acquisition pose, even when the chassis turns.
        pose = transform_points(self.buffer, np.eye(4, 3),
                                'base_footprint', msg.header.stamp, target='odom')
        rotation, translation = (pose[:3] - pose[3]).T, pose[3]
        wall = EMPTY
        if self.map is not None and map_pose is not None:
            map_rotation, map_origin = map_pose
            world = (body @ rotation.T + translation) @ map_rotation.T + map_origin
            on_wall = self.map.matches_wall(world)
            # Keep mapped walls out of the dynamic clustering and trigger stream.
            wall = self.lidar_wall_track.cluster_filter.apply(body[on_wall])
            body = body[~on_wall]
        points = self.lidar_track.cluster_filter.apply(body)
        self.lidar_track.update(stamp, points @ rotation.T + translation, usable=healthy)
        self.lidar_wall_track.update(stamp, wall @ rotation.T + translation, usable=healthy)
        return healthy

    def refresh_sensor(self, source, ros_now, now, map_pose=None):
        queue = self.pending[source]
        previous = self.processed_stamps.get(source, 0.0)
        for index in range(len(queue) - 1, -1, -1):
            msg = queue[index]
            stamp = stamp_seconds(msg.header.stamp)
            if not fresh_stamp(stamp, ros_now, SENSOR_TIMEOUT) or stamp <= previous:
                del queue[index]
                continue
            try:
                healthy = (self.observe_depth(msg, stamp) if source == 'depth'
                           else self.observe_lidar(msg, stamp, map_pose))
            except TransformException:
                # Give odometry time to publish TF for this acquisition stamp.
                continue
            except (ValueError, TypeError) as error:
                healthy = False
                self.get_logger().warning(f'{source}: {error}', throttle_duration_sec=2.0)
            # Tolerate an isolated bad frame while the previous usable one
            # is fresh. Invalid samples never extend the sensor deadline.
            self.processed_stamps[source] = stamp
            if healthy:
                self.samples[source] = Sample(stamp, now, None)
            for _ in range(index + 1):
                queue.popleft()
            return

    def decision(self, ros_now, now):
        self.close_obstacle = False
        if self.camera_estop is None or now - self.camera_estop[1] > AUTO_ESTOP_TIMEOUT:
            return Decision('AUTO_ESTOP_UNAVAILABLE', estop=True)
        if self.camera_estop[0]:
            return Decision('CAMERA_ESTOP', estop=True)
        self.refresh_sensor('depth', ros_now, now)
        depth = self.samples.get('depth')
        if depth is None or not depth.fresh(ros_now, now, SENSOR_TIMEOUT):
            return Decision('DEPTH_UNAVAILABLE')

        map_pose = self.map_pose(ros_now)
        self.refresh_sensor('lidar', ros_now, now, map_pose)
        lidar = self.samples.get('lidar')
        if lidar is None or not lidar.fresh(ros_now, now, SENSOR_TIMEOUT):
            return Decision('LIDAR_UNAVAILABLE')
        if self.odom is None or not self.odom.fresh(ros_now, now, ODOM_TIMEOUT):
            return Decision('ODOM_UNAVAILABLE')

        odom = self.odom.data
        pose, twist = odom.pose.pose, odom.twist.twist
        position = np.array([pose.position.x, pose.position.y, pose.position.z])
        q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        measured = (twist.linear.x, twist.angular.z)
        if (odom.header.frame_id != 'odom' or odom.child_frame_id != 'base_footprint'
                or not np.isfinite([*position, *q, *measured]).all()
                or not 0.98 < np.linalg.norm(q) < 1.02):
            return Decision('ODOM_INVALID')

        if map_pose is None:
            return Decision('MAP_TF_UNAVAILABLE')
        map_rotation, map_origin = map_pose
        rotation = rotation_matrix(q)
        points = voxel_points(self.lidar_track.snapshot(ros_now), self.cfg['cluster_cell_m'])
        walls = self.lidar_wall_track.snapshot(ros_now)
        guarded = voxel_points(np.concatenate((points, walls)), self.cfg['cluster_cell_m'])
        body = (guarded - position) @ rotation
        dynamic = (points - position) @ rotation
        front = front_returns(dynamic, self.cfg['lidar_detour_fov_deg'])
        distance = clearances(dynamic)
        trigger = detour_trigger(front, distance, self.cfg['detour_trigger_m'])
        self.close_obstacle = bool(trigger.any())
        world = points @ map_rotation.T + map_origin
        self.obstacle_pub.publish(planner_cloud(world, trigger, odom.header.stamp))
        if self.manual is None or now - self.manual[1] > ODOM_TIMEOUT:
            return Decision('ESTOP_STATE_UNAVAILABLE')
        if self.manual[0]:
            return Decision('MANUAL_STOP')
        if self.chassis_estop:
            return Decision('CHASSIS_ESTOP')
        if self.map is None:
            return Decision('MAP_UNAVAILABLE')
        fresh_command = self.command is not None and now - self.command[1] <= COMMAND_TIMEOUT
        if self.planner is None or now - self.planner[1] > PLANNER_TIMEOUT:
            return Decision('PLANNER_UNAVAILABLE' if fresh_command and self.command[0] != ZERO
                            else 'IDLE')
        state = self.planner[0]
        if state not in ('FOLLOWING_ROUTE', 'DETOURING'):
            return Decision(state)
        if not fresh_command:
            return Decision('CONTROLLER_NO_COMMAND')
        if self.command[0] == ZERO:
            return Decision('CONTROLLER_ZERO_COMMAND')
        linear, angular = self.command[0]
        if not (math.isfinite(linear) and math.isfinite(angular)) or linear < 0.0:
            return Decision('INVALID_COMMAND')
        map_body_rotation = map_rotation @ rotation
        turn_limit = self.cfg['in_place_angular_limit_radps']
        turn = None
        if state == 'DETOURING':
            heading = self.detour_heading
            if heading is None or now - heading.received > PLANNER_TIMEOUT:
                return Decision('WAITING_FOR_DETOUR_HEADING')
            yaw = math.atan2(map_body_rotation[1, 0], map_body_rotation[0, 0])
            error = math.remainder(heading.data - yaw, 2 * math.pi)
            turn = self.detour_turn.decision(heading.stamp, error, measured, turn_limit)
            velocity_limit = self.cfg['detour_linear_speed_mps']
        else:
            velocity_limit = approach_speed_limit(distance[front].min(initial=math.inf), self.cfg)
        angular_limit = self.cfg['angular_limit_radps'] if linear > 0.0 else turn_limit
        scale = min(1.0, velocity_limit / max(linear, 1e-6),
                    angular_limit / max(abs(angular), 1e-6))
        command = turn.velocity if turn is not None else (linear * scale, angular * scale)
        last_time, previous = self.last_output
        if command[0] == 0.0:
            previous = (0.0, math.copysign(min(abs(previous[1]), turn_limit), previous[1]))
        elif previous[0] > velocity_limit:
            # Brake to the active speed cap while retaining the current turning radius.
            previous = (velocity_limit, previous[1] * velocity_limit / previous[0])
        if turn is not None:
            # The checked in-place turn needs the STM32's usable angular rate.
            output = command
        else:
            output = ramp_command(previous, command, now - last_time,
                                  self.cfg['linear_accel_limit_mps2'],
                                  self.cfg['angular_accel_limit_radps2'])
        motions = dict.fromkeys((measured, output))
        if not all(motion_is_clear(body, v, self.cfg['motion_clearance_m']) for v in motions):
            return Decision('TRAJECTORY_BLOCKED')
        map_position = position @ map_rotation.T + map_origin
        map_points = self.map.nearby(map_position[:2], map_body_rotation[:2, :2])
        margin = 0.72 * self.map.resolution
        if not all(motion_is_clear(map_points, v, margin) for v in motions):
            return Decision('MAP_BLOCKED')
        return Decision(turn.state if turn is not None else state, output)

    def update_display(self, stamp, ros_now, now, decision):
        auto = decision.estop or self.chassis_auto_estop
        estop = auto or self.chassis_estop or (self.manual is not None and self.manual[0])
        status = (estop, auto, decision.state)
        if status != self.displayed_status:
            self.display_estop_pub.publish(Bool(data=estop))
            self.display_auto_pub.publish(Bool(data=auto))
            self.marker_pub.publish(navigation_marker(stamp, estop, decision.state))
            self.displayed_status = status

        plan = self.latest_plan
        follows_route = decision.state == 'FOLLOWING_ROUTE' and not self.close_obstacle
        follows_detour = (self.planner is not None and self.planner[0] == 'DETOURING'
                          and now - self.planner[1] <= PLANNER_TIMEOUT
                          and self.detour_heading is not None and plan is not None
                          and plan.stamp >= self.detour_heading.stamp)
        if estop or not (follows_route or follows_detour):
            self.plan_hidden_since = ros_now
        visible = (not estop and (follows_route or follows_detour) and plan is not None
                   and plan.stamp > self.plan_hidden_since
                   and plan.fresh(ros_now, now, PLANNER_TIMEOUT))
        displayed = plan.stamp if visible else 0.0
        if displayed != self.displayed_plan:
            cleared = Path()
            cleared.header.frame_id, cleared.header.stamp = 'map', stamp
            self.display_plan_pub.publish(plan.data if visible else cleared)
            self.displayed_plan = displayed

    def update(self):
        stamp = self.get_clock().now().to_msg()
        ros_now = stamp_seconds(stamp)
        now = time.monotonic()
        decision = self.decision(ros_now, now)
        output = Twist()
        output.linear.x, output.angular.z = decision.velocity
        self.velocity_pub.publish(output)
        self.last_output = (now, decision.velocity)
        self.state_pub.publish(String(data=decision.state))
        self.update_display(stamp, ros_now, now, decision)
        if self.state != decision.state:
            planner = self.planner[0] if self.planner is not None else 'NONE'
            requested = (self.command[0] if self.command is not None
                         and now - self.command[1] <= COMMAND_TIMEOUT else None)
            ages = ', '.join(f'{source}_age={ros_now - self.samples[source].stamp:.2f}s'
                             if source in self.samples else f'{source}_age=unknown'
                             for source in self.pending)
            self.get_logger().info(
                f'{decision.state} planner={planner} nav={requested} '
                f'output={decision.velocity} estop={decision.estop} {ages}')
            self.state = decision.state


def main():
    rclpy.init()
    node = ObstacleGateNode()
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
