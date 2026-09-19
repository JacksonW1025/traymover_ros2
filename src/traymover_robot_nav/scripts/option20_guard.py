#!/usr/bin/env python3
"""Fail-closed option 20 command gate; pure policy is testable without ROS."""
import json
import math
import time
from dataclasses import dataclass

import numpy as np


def angle_delta(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


@dataclass
class Config:
    sensor_timeout: float = 0.6
    odom_timeout: float = 0.4
    localization_timeout: float = 2.5
    command_timeout: float = 0.3
    auto_estop_timeout: float = 0.3
    future_tolerance: float = 0.1
    release_time: float = 0.8
    localization_samples: int = 3
    xy_std: float = 0.4
    yaw_std: float = 0.25
    correction_jump: float = 0.35
    correction_yaw_jump: float = 0.25
    min_finite_beams: int = 30
    min_map_score: float = 0.25
    map_hit_distance: float = 0.3
    max_linear: float = 0.14
    max_angular: float = 0.35
    linear_accel: float = 0.12
    angular_accel: float = 0.4
    stopping_decel: float = 0.2
    reaction_time: float = 0.6
    padding: float = 0.15
    slow_distance: float = 0.35
    half_length: float = 0.4
    half_width: float = 0.275
    min_depth_fraction: float = 0.1

    def __post_init__(self):
        if any(not math.isfinite(float(v)) or v <= 0 for v in vars(self).values()):
            raise ValueError('Guard parameters must be positive and finite')
        if self.padding < 0.15 or self.max_linear > 0.14 or self.max_angular > 0.35:
            raise ValueError('Option 20 safety limits cannot be weakened')


class Freshness:
    def __init__(self):
        self.samples = {}
        self.invalid = {}

    def receive(self, name, stamp, ros_now, mono_now, future_tolerance=0.1):
        previous = self.samples.get(name)
        if (not all(math.isfinite(v) for v in (stamp, ros_now, mono_now))
                or stamp <= 0 or stamp > ros_now + future_tolerance
                or (previous is not None and stamp <= previous[0])):
            self.invalid[name] = 'invalid_or_nonincreasing_stamp'
            return False
        self.samples[name] = (stamp, mono_now)
        self.invalid.pop(name, None)
        return True

    def reason(self, name, ros_now, mono_now, timeout, tolerance):
        if name in self.invalid:
            return name + ':' + self.invalid[name]
        if name not in self.samples:
            return name + ':missing'
        stamp, received = self.samples[name]
        age = ros_now - stamp
        if age < -tolerance or age > timeout or mono_now - received > timeout:
            return name + ':stale_or_clock_jump'
        return ''


def correction_from_pose(global_pose, odom_pose):
    yaw = angle_delta(global_pose[2], odom_pose[2])
    c, s = math.cos(yaw), math.sin(yaw)
    return (global_pose[0] - c * odom_pose[0] + s * odom_pose[1],
            global_pose[1] - s * odom_pose[0] - c * odom_pose[1], yaw)


class MapMatcher:
    """Pre-dilate occupied endpoints, treating unknown/outside as no match."""
    def __init__(self, data, width, height, resolution, origin, hit_distance):
        self.resolution = resolution
        self.origin = origin
        if resolution <= 0 or width <= 0 or height <= 0:
            raise ValueError('Invalid map dimensions')
        occupied = np.asarray(data).reshape(height, width) >= 65
        self.hits = np.zeros_like(occupied)
        radius = int(math.ceil(hit_distance / resolution))
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if math.hypot(dx, dy) * resolution > hit_distance:
                    continue
                if abs(dx) >= width or abs(dy) >= height:
                    continue
                ys, yd = slice(max(0, -dy), min(height, height - dy)), slice(max(0, dy), min(height, height + dy))
                xs, xd = slice(max(0, -dx), min(width, width - dx)), slice(max(0, dx), min(width, width + dx))
                self.hits[yd, xd] |= occupied[ys, xs]

    def score(self, ranges, angle_min, angle_increment, range_min, range_max, scan_pose):
        ranges = np.asarray(ranges, dtype=float)
        finite = np.isfinite(ranges) & (ranges >= range_min) & (ranges < range_max)
        n = int(np.count_nonzero(finite))
        if not n:
            return 0.0, 0
        angles = angle_min + np.flatnonzero(finite) * angle_increment + scan_pose[2]
        x = scan_pose[0] + ranges[finite] * np.cos(angles) - self.origin[0]
        y = scan_pose[1] + ranges[finite] * np.sin(angles) - self.origin[1]
        c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
        ix = np.floor((c * x + s * y) / self.resolution).astype(int)
        iy = np.floor((-s * x + c * y) / self.resolution).astype(int)
        inside = (ix >= 0) & (iy >= 0) & (ix < self.hits.shape[1]) & (iy < self.hits.shape[0])
        return float(np.count_nonzero(self.hits[iy[inside], ix[inside]])) / n, n


class Guard:
    def __init__(self, config=None):
        self.cfg = config or Config()
        self.freshness = Freshness()
        self.command = (0.0, 0.0)
        self.command_time = None
        self.output = (0.0, 0.0)
        self.last_tick = None
        self.ready_since = None
        self.localization_count = 0
        self.last_correction = None
        self.localization_error = 'not_initialized'
        self.scan_score = 0.0
        self.finite_beams = 0
        self.depth_healthy = False
        self.points = {}
        self.measured_speed = 0.0
        self.measured_angular = 0.0
        self.auxiliary = {}
        self.estop = {'aggregate': (True, None), 'manual': (True, None)}
        self.auto_request = (True, None)
        self.publisher_exclusive = True

    def reset_localization(self):
        self.localization_count = 0
        self.last_correction = None
        self.localization_error = 'initializing'
        self.ready_since = None

    def localization(self, correction, covariance):
        cfg = self.cfg
        if (len(covariance) != 36 or not np.isfinite(covariance).all()
                or not np.isfinite(correction).all()):
            error = 'invalid_pose_or_covariance'
        elif any(covariance[i] < 0 for i in (0, 7, 35)):
            error = 'negative_covariance'
        elif (max(covariance[0], covariance[7]) > cfg.xy_std ** 2
              or covariance[35] > cfg.yaw_std ** 2):
            error = 'uncertain_pose'
        elif self.last_correction is not None and (
                math.hypot(correction[0] - self.last_correction[0], correction[1] - self.last_correction[1]) > cfg.correction_jump
                or abs(angle_delta(correction[2], self.last_correction[2])) > cfg.correction_yaw_jump):
            error = 'global_correction_jump'
        else:
            error = ''
        if np.isfinite(correction).all():
            self.last_correction = correction
        self.localization_error = error
        self.localization_count = 0 if error else self.localization_count + 1
        if error:
            self.ready_since = None

    def obstacle_scale(self, v, w):
        cfg = self.cfg
        arrays = list(self.points.values())
        points = np.concatenate(arrays) if arrays else np.empty((0, 2))
        if not len(points):
            return 1.0
        x, y = points[:, 0], points[:, 1]
        speed = max(abs(v), abs(self.measured_speed))
        angular = max(abs(w), abs(self.measured_angular))
        radius = math.hypot(cfg.half_length, cfg.half_width)
        travel = speed * cfg.reaction_time + speed ** 2 / (2 * cfg.stopping_decel)
        rotational_travel = angular * radius * cfg.reaction_time + (angular * radius) ** 2 / (2 * cfg.stopping_decel)
        if angular > 0.02:
            clearance = np.hypot(x, y) - radius - cfg.padding - travel - rotational_travel
        else:
            dx = np.maximum(np.abs(x) - cfg.half_length - cfg.padding, 0.0)
            dy = np.maximum(np.abs(y) - cfg.half_width - cfg.padding, 0.0)
            clearance = np.hypot(dx, dy)
            # Inflate in both longitudinal directions: stops residual backward drift too.
            longitudinal = (np.abs(y) <= cfg.half_width + cfg.padding)
            clearance[longitudinal] = np.maximum(0, np.abs(x[longitudinal]) - cfg.half_length - cfg.padding - travel)
        nearest = float(np.min(clearance))
        return min(1.0, max(0.0, nearest / cfg.slow_distance))

    def tick(self, ros_now, mono_now):
        cfg = self.cfg
        reasons = []
        for name, timeout in [('odom', cfg.odom_timeout), ('scan', cfg.sensor_timeout),
                              ('lidar', cfg.sensor_timeout), ('depth', cfg.sensor_timeout),
                              ('depth_status', cfg.sensor_timeout), ('odom_status', cfg.odom_timeout),
                              ('scan_status', cfg.sensor_timeout), ('lidar_status', cfg.sensor_timeout), ('amcl', cfg.localization_timeout)]:
            reason = self.freshness.reason(name, ros_now, mono_now, timeout, cfg.future_tolerance)
            if reason:
                reasons.append(reason)
        for name in ('odom_status', 'scan_status', 'lidar_status'):
            if self.auxiliary.get(name) is not True:
                reasons.append(name + ':unhealthy')
        for name, (active, received) in self.estop.items():
            if active or received is None:
                reasons.append('estop:' + name + ':active_or_missing')
        active, received = self.auto_request
        if active or received is None or mono_now - received > cfg.auto_estop_timeout:
            reasons.append('estop:auto_request:active_or_stale')
        if not self.publisher_exclusive:
            reasons.append('command:multiple_safe_publishers')
        if self.localization_error:
            reasons.append('localization:' + self.localization_error)
        if self.localization_count < cfg.localization_samples:
            reasons.append('localization:stabilizing')
        if self.finite_beams < cfg.min_finite_beams or self.scan_score < cfg.min_map_score:
            reasons.append('scan:insufficient_map_agreement')
        if not self.depth_healthy:
            reasons.append('depth:insufficient_valid_pixels')
        command_reasons = []
        if self.command_time is None or mono_now - self.command_time > cfg.command_timeout:
            command_reasons.append('command:stale')
        if not np.isfinite(self.command).all() or self.command[0] < -1e-6:
            command_reasons.append('command:invalid_or_reverse')
        v = min(cfg.max_linear, max(0.0, self.command[0]))
        w = min(cfg.max_angular, max(-cfg.max_angular, self.command[1]))
        scale = self.obstacle_scale(v, w)
        if scale <= 0:
            reasons.append('obstacle:stop_envelope')
        dt = min(0.1, max(0.0, mono_now - self.last_tick)) if self.last_tick is not None else 0.0
        self.last_tick = mono_now
        sensors_ready = not reasons
        if not sensors_ready:
            self.ready_since = None
        elif self.ready_since is None:
            self.ready_since = mono_now
        ready_for_goal = (sensors_ready and mono_now - self.ready_since >= cfg.release_time)
        if sensors_ready and not ready_for_goal:
            reasons.append('release:stabilizing')
        if (ready_for_goal and self.command_time is not None
                and self.command_time < self.ready_since + cfg.release_time):
            command_reasons.append('command:awaiting_post_release_command')
        reasons.extend(command_reasons)
        if reasons:
            self.output = (0.0, 0.0)
        else:
            def ramp(old, target, accel):
                # Reduced limits take effect immediately; acceleration is bounded.
                if abs(target) <= abs(old) or old * target < 0:
                    return target if old * target >= 0 else 0.0
                return old + max(-accel * dt, min(accel * dt, target - old))
            self.output = (ramp(self.output[0], v * scale, cfg.linear_accel),
                           ramp(self.output[1], w * scale, cfg.angular_accel))
        return self.output, {'ready': not reasons, 'sensors_ready': sensors_ready,
                             'ready_for_goal': ready_for_goal, 'reasons': reasons,
                             'map_score': self.scan_score, 'finite_beams': self.finite_beams,
                             'localization_samples': self.localization_count, 'obstacle_scale': scale}



def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from rclpy.duration import Duration
    from rclpy.time import Time
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
    from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry, OccupancyGrid
    from sensor_msgs.msg import LaserScan, PointCloud2
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import String, Bool
    from std_srvs.srv import Empty
    from tf2_ros import Buffer, TransformListener

    def stamp_seconds(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def yaw(q):
        values = (q.x, q.y, q.z, q.w)
        if not np.isfinite(values).all() or abs(sum(v * v for v in values) - 1.0) > 0.05:
            raise ValueError('Invalid quaternion')
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def pose_tuple(p):
        value = (p.position.x, p.position.y, yaw(p.orientation))
        if not np.isfinite(value).all():
            raise ValueError('Nonfinite pose')
        return value

    def transform_tuple(t):
        value = (t.translation.x, t.translation.y, yaw(t.rotation))
        if not np.isfinite(value).all():
            raise ValueError('Nonfinite transform')
        return value

    class GuardNode(Node):
        def __init__(self):
            super().__init__('option20_guard')
            values = {}
            for key, value in vars(Config()).items():
                values[key] = self.declare_parameter(key, value).value
            self.guard = Guard(Config(**values))
            self.matcher = None
            self.initialpose_stamp = 0.0
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, self)
            self.publisher = self.create_publisher(Twist, '/option20/cmd_vel_safe', 10)
            self.health = self.create_publisher(String, '/option20/health', 10)
            self.create_subscription(Twist, '/option20/cmd_vel_nav', self.command, 10)
            self.create_subscription(Odometry, '/odom', self.odom, qos_profile_sensor_data)
            self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl, 10)
            self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initialpose, 10)
            self.create_subscription(LaserScan, '/scan_localization', self.scan, qos_profile_sensor_data)
            self.create_subscription(String, '/option20/depth_status', self.depth_status, 10)
            for name in ('odom_status', 'scan_status', 'lidar_status'):
                self.create_subscription(String, '/option20/' + name,
                                         lambda msg, key=name: self.status(key, msg), 10)
            estop_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                   reliability=ReliabilityPolicy.RELIABLE)
            for name, topic in [('aggregate', '/traymover_estop/state'),
                                ('manual', '/traymover_estop/manual_state')]:
                self.create_subscription(Bool, topic,
                    lambda msg, key=name: self.guard.estop.__setitem__(key, (bool(msg.data), time.monotonic())), estop_qos)
            self.create_subscription(Bool, '/traymover_estop/auto_request',
                lambda msg: setattr(self.guard, 'auto_request', (bool(msg.data), time.monotonic())), 10)
            for name in ('lidar', 'depth'):
                self.create_subscription(PointCloud2, '/option20/' + name + '_obstacles',
                                         lambda msg, key=name: self.cloud(key, msg), qos_profile_sensor_data)
            map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(OccupancyGrid, '/localization_map', self.map, map_qos)
            self.nomotion = self.create_client(Empty, '/request_nomotion_update')
            self.nomotion_future = None
            self.create_timer(1.0, self.request_update)
            self.create_timer(0.05, self.tick)
            self.last_health = 0.0

        def now(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def accept(self, name, stamp):
            return self.guard.freshness.receive(name, stamp, self.now(), time.monotonic(), self.guard.cfg.future_tolerance)

        def invalidate(self, name, error):
            self.guard.freshness.invalid[name] = str(error)

        def command(self, msg):
            values = (msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z)
            if not np.isfinite(values).all() or any(abs(v) > 1e-6 for v in values[1:5]):
                self.guard.command = (math.nan, math.nan)
            else:
                self.guard.command = (msg.linear.x, msg.angular.z)
            self.guard.command_time = time.monotonic()

        def odom(self, msg):
            try:
                if msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_footprint':
                    raise ValueError('unexpected_frames')
                pose_tuple(msg.pose.pose)
                values = (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z)
                if not np.isfinite(values).all():
                    raise ValueError('invalid_velocity')
                if self.accept('odom', stamp_seconds(msg.header.stamp)):
                    self.guard.measured_speed = math.hypot(values[0], values[1])
                    self.guard.measured_angular = values[2]
            except Exception as exc:
                self.invalidate('odom', exc)

        def initialpose(self, msg):
            self.initialpose_stamp = self.now()
            self.guard.reset_localization()
            self.guard.freshness.samples.pop('amcl', None)

        def amcl(self, msg):
            try:
                if msg.header.frame_id != 'map':
                    raise ValueError('unexpected_frame')
                if stamp_seconds(msg.header.stamp) < self.initialpose_stamp:
                    raise ValueError('pose_predates_initialization')
                # Compare global corrections at the SAME timestamp, not raw robot displacement.
                tf = self.buffer.lookup_transform('odom', 'base_footprint', Time.from_msg(msg.header.stamp), timeout=Duration(seconds=0.0))
                correction = correction_from_pose(pose_tuple(msg.pose.pose), transform_tuple(tf.transform))
                if self.accept('amcl', stamp_seconds(msg.header.stamp)):
                    self.guard.localization(correction, list(msg.pose.covariance))
            except Exception as exc:
                self.invalidate('amcl', exc)
                self.guard.localization_count = 0

        def map(self, msg):
            try:
                if msg.header.frame_id != 'map':
                    raise ValueError('unexpected_map_frame')
                self.matcher = MapMatcher(msg.data, msg.info.width, msg.info.height,
                                          msg.info.resolution, pose_tuple(msg.info.origin), self.guard.cfg.map_hit_distance)
            except Exception as exc:
                self.matcher = None
                self.get_logger().error('Map unavailable: ' + str(exc))

        def scan(self, msg):
            try:
                if self.matcher is None:
                    raise ValueError('map_missing')
                if not np.isfinite((msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max)).all() or msg.angle_increment <= 0 or msg.range_max <= msg.range_min:
                    raise ValueError('invalid_scan_metadata')
                if np.isnan(np.asarray(msg.ranges, dtype=float)).any():
                    raise ValueError('nan_scan_ranges')
                tf = self.buffer.lookup_transform('map', msg.header.frame_id, Time.from_msg(msg.header.stamp), timeout=Duration(seconds=0.0))
                score, count = self.matcher.score(msg.ranges, msg.angle_min, msg.angle_increment,
                                                  msg.range_min, msg.range_max, transform_tuple(tf.transform))
                if self.accept('scan', stamp_seconds(msg.header.stamp)):
                    self.guard.scan_score, self.guard.finite_beams = score, count
            except Exception as exc:
                self.invalidate('scan', exc)

        def cloud(self, name, msg):
            try:
                if msg.header.frame_id != 'base_footprint':
                    raise ValueError('unexpected_frame')
                points = np.asarray(point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=False), dtype=float).reshape(-1, 3)
                if not np.isfinite(points).all():
                    raise ValueError('nonfinite_points')
                if self.accept(name, stamp_seconds(msg.header.stamp)):
                    self.guard.points[name] = points[:, :2]
            except Exception as exc:
                self.invalidate(name, exc)

        def depth_status(self, msg):
            try:
                data = json.loads(msg.data)
                fraction = float(data['valid_ratio'])
                if not math.isfinite(fraction) or not 0 <= fraction <= 1:
                    raise ValueError('invalid_valid_fraction')
                if self.accept('depth_status', float(data['stamp_sec'])):
                    self.guard.depth_healthy = data.get('valid') is True and fraction >= self.guard.cfg.min_depth_fraction
            except Exception as exc:
                self.guard.depth_healthy = False
                self.invalidate('depth_status', exc)

        def status(self, name, msg):
            try:
                data = json.loads(msg.data)
                if self.accept(name, float(data['stamp_sec'])):
                    self.guard.auxiliary[name] = data.get('valid') is True
                    if not self.guard.auxiliary[name]:
                        self.invalidate(name, data.get('reason', 'invalid'))
            except Exception as exc:
                self.guard.auxiliary[name] = False
                self.invalidate(name, exc)

        def request_update(self):
            if self.nomotion_future is not None and not self.nomotion_future.done():
                return
            if self.nomotion.service_is_ready():
                self.nomotion_future = self.nomotion.call_async(Empty.Request())

        def tick(self):
            mono = time.monotonic()
            self.guard.publisher_exclusive = self.count_publishers('/option20/cmd_vel_safe') == 1
            output, health = self.guard.tick(self.now(), mono)
            msg = Twist()
            msg.linear.x, msg.angular.z = output
            self.publisher.publish(msg)
            if mono - self.last_health >= 0.2:
                health['stamp'] = self.now()
                health['command'] = list(output)
                self.health.publish(String(data=json.dumps(health)))
                self.last_health = mono

    rclpy.init(args=args)
    node = GuardNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
