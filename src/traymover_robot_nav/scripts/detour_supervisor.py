#!/usr/bin/env python3
"""Option 18 ROS boundary: no EStop writes, no hardware feedback assumptions.

All local deadlines use a steady clock. Header timestamps must also be recent
and advance, so queued/replayed messages cannot make a dead sensor look fresh.
Phase displacement comes from FAST-LIO's odom TF, never wheel feedback.
"""
from collections import deque
import copy
from dataclasses import fields, replace
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from detour_gate import Config, DetourGate, Observation, Pose, Ray, Scan, Velocity
from detour_goal_proxy import GoalProxy
from detour_obstacle import scan_to_rays, stamp_seconds, yaw
from detour_obstacle import valid_quaternion  # noqa: F401 (compatibility export)


class DetourSupervisor(Node):
    def __init__(self):
        super().__init__('traymover_detour_supervisor')
        defaults = Config()
        config = {}
        for field in fields(defaults):
            config[field.name] = self.declare_parameter(
                field.name, getattr(defaults, field.name)).value
        self.config = Config(**config)
        self.gate = DetourGate(self.config)
        self.extra = {}
        for name, default in {
                'pose_frame': 'odom', 'base_frame': 'base_link', 'map_frame': 'map',
                'localization_topic': '/pcl_pose', 'tf_timeout_sec': 1.0,
                'localization_timeout_sec': 5.0, 'estop_timeout_sec': 1.0,
                'supervisor_rate_hz': 20.0}.items():
            # Allow future gate Config to share an existing parameter.
            self.extra[name] = (self.get_parameter(name).value if self.has_parameter(name)
                                else self.declare_parameter(name, default).value)
        for name in ('tf_timeout_sec', 'localization_timeout_sec',
                     'estop_timeout_sec', 'supervisor_rate_hz'):
            if not math.isfinite(self.extra[name]) or self.extra[name] <= 0:
                raise ValueError(f'{name} must be finite and positive')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.scans = {'lidar': None, 'realsense': None, 'perimeter': None}
        self.scan_headers = {}
        self.scan_problems = {}
        self.scan_diagnostic_at = {}
        self.pending_scans = {key: deque(maxlen=10) for key in self.scans}
        self.scan_acquisition_poses = {}
        self.scan_tf_ok = {key: False for key in self.scans}
        self.localization_at = float('-inf')
        self.localization_header = float('-inf')
        self.cmd = Twist()
        self.cmd_at = float('-inf')
        self.estop = {'aggregate': True, 'manual': False, 'auto': True,
                      'reason': 'STARTUP'}
        self.estop_at = {name: float('-inf') for name in self.estop}
        self.pending_manual = False
        self.pending_auto = False
        self.pending_depth_fault = None
        self.normal_since = time.monotonic()
        self.last_status = None
        self.next_status_at = 0.0
        self.handback_pending = False
        self.last_pose_sample = None
        self.pose_fault_until = 0.0
        self.output = self.create_publisher(Twist, '/cmd_vel_detour', 1)
        status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status = {
            key: self.create_publisher(
                Bool if key == 'active' else String, f'/traymover_detour/{key}', status_qos)
            for key in ('state', 'active', 'reason', 'phase')}
        self.create_subscription(Twist, '/cmd_vel_nav', self.on_cmd, 1)
        for key, topic in (('lidar', '/scan'), ('realsense', '/scan_realsense'),
                           ('perimeter', '/scan_detour_perimeter')):
            self.create_subscription(LaserScan, topic,
                                     lambda msg, key=key: self.on_scan(key, msg),
                                     qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped,
                                 self.extra['localization_topic'], self.on_localization,
                                 qos_profile_sensor_data)
        for key, suffix in (('aggregate', 'state'), ('manual', 'manual_state'),
                            ('auto', 'auto_state'), ('reason', 'auto_reason')):
            self.create_subscription(
                String if key == 'reason' else Bool, f'/traymover_estop/{suffix}',
                lambda msg, key=key: self.on_estop(key, msg), status_qos)
        self.proxy = GoalProxy(
            self, is_paused=lambda: self.gate.state != 'NORMAL',
            on_goal_changed=self.on_goal_changed)
        # A paused ROS clock must not suspend zero output or sensor watchdogs.
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(
            1.0 / self.extra['supervisor_rate_hz'], self.on_tick, clock=self.steady_clock)
        self.get_logger().info(
            '[detour] Initial tuning values; zero command asserted until inputs are healthy')
        self.get_logger().info(
            f'[detour] simple_mode={self.config.simple_detour} '
            f'trigger={self.config.detour_trigger_distance_m:.2f}m '
            f'right_turn={self.config.detour_turn_angle_deg:.1f}deg '
            f'forward_target={self.config.detour_first_leg_m:.2f}m '
            f'camera_half_width={self.config.simple_obstacle_half_width_m:.2f}m '
            f'confirm_frames={self.config.obstacle_confirm_frames} '
            f'min_points={self.config.min_obstacle_points} '
            f'camera_timeout={self.config.realsense_timeout_sec:.2f}s '
            f'lidar_timeout={self.config.scan_timeout_sec:.2f}s')

    def acquisition_time(self, stamp, now):
        stamped = stamp_seconds(stamp)
        age = self.get_clock().now().nanoseconds * 1e-9 - stamped
        if stamped <= 0 or not math.isfinite(age) or age < -0.05:
            raise ValueError('missing or future header stamp')
        return now - max(0.0, age), stamped

    def on_cmd(self, message):
        values = (message.linear.x, message.linear.y, message.linear.z,
                  message.angular.x, message.angular.y, message.angular.z)
        if not all(math.isfinite(value) for value in values):
            self.cmd_at = float('-inf')
            self.output.publish(Twist())
            return
        self.cmd, self.cmd_at = message, time.monotonic()

    def on_estop(self, key, message):
        self.estop[key], self.estop_at[key] = message.data, time.monotonic()
        if key == 'manual' and message.data:
            self.pending_manual = True
        if key in ('auto', 'aggregate') and message.data:
            self.pending_auto = True
        # LOW_VALID_DEPTH is also a pre-confirmation diagnostic: the depth
        # guard publishes it before its consecutive-frame EStop threshold.
        unconfirmed_low_valid = (
            self.config.simple_detour and key == 'reason' and
            message.data == 'LOW_VALID_DEPTH' and
            not (self.estop['auto'] or self.estop['aggregate'] or self.pending_auto))
        depth_fault = (key == 'reason' and
                       message.data not in ('NONE', 'NEAR_OBSTACLE') and
                       not unconfirmed_low_valid)
        if depth_fault:
            self.pending_depth_fault = message.data
        if (key != 'reason' and message.data) or depth_fault:
            self.output.publish(Twist())

    def scan_problem(self, key, reason):
        self.scan_problems[key] = reason
        now = time.monotonic()
        if now >= self.scan_diagnostic_at.get(key, 0.0):
            self.get_logger().warning(f'[detour] {key} scan: {reason}')
            self.scan_diagnostic_at[key] = now + 5.0

    def scan_required(self, key):
        """The fixed simple cycle requires camera data; strict mode requires all scans."""
        if not self.config.enable_detour:
            return False
        if not self.config.simple_detour:
            return True
        return key == 'realsense'

    def on_scan(self, key, message):
        try:
            acquired, header = self.acquisition_time(message.header.stamp, time.monotonic())
            if not message.header.frame_id:
                raise ValueError('missing scan frame_id')
            if header <= self.scan_headers.get(key, float('-inf')):
                raise ValueError('non-advancing scan stamp')
            self.scan_headers[key] = header
            self.pending_scans[key].append((message, acquired))
            self.consume_scans(key)
        except ValueError as error:
            self.scan_problem(key, str(error))
            self.pending_scans[key].clear()
            self.scans[key], self.scan_tf_ok[key] = None, False
            if self.scan_required(key):
                self.output.publish(Twist())

    def consume_scans(self, key):
        # Static sensor extrinsics are enough to observe the local scene.
        # Strict mode retains its scan-time odom requirement. Simple mode uses
        # scan-time odom only when available for cached-ray compensation.
        pending = self.pending_scans[key]
        while pending:
            message, acquired = pending[0]
            limit = (self.config.realsense_timeout_sec if key == 'realsense'
                     else self.config.scan_timeout_sec)
            age = time.monotonic() - acquired
            if age > limit:
                self.scan_problem(key, f'stale acquisition: age={age:.2f}s limit={limit:.2f}s')
                pending.popleft()
                continue
            target = (self.extra['base_frame'] if self.config.simple_detour
                      else self.extra['pose_frame'])
            try:
                transform = self.tf.lookup_transform(
                    target, message.header.frame_id, Time.from_msg(message.header.stamp))
            except TransformException as error:
                self.scan_problem(key, f'TF {target} <- {message.header.frame_id}: {error}')
                break
            base_at_scan = None
            try:
                base_at_scan = self.tf.lookup_transform(
                    self.extra['pose_frame'], self.extra['base_frame'],
                    Time.from_msg(message.header.stamp)).transform
            except TransformException as error:
                if not self.config.simple_detour:
                    self.scan_problem(key, f'TF odom pose at acquisition: {error}')
                    break
            pending.popleft()
            try:
                sensor_t = transform.transform.translation
                self.scan_acquisition_poses.pop(key, None)
                if base_at_scan is not None:
                    base_t = base_at_scan.translation
                    if not all(math.isfinite(v) for v in (base_t.x, base_t.y, base_t.z)):
                        raise ValueError('invalid scan acquisition pose')
                    sensor_radius = (math.hypot(sensor_t.x, sensor_t.y)
                                     if self.config.simple_detour else
                                     math.hypot(sensor_t.x - base_t.x, sensor_t.y - base_t.y))
                    self.scan_acquisition_poses[key] = (
                        Pose(base_t.x, base_t.y, yaw(base_at_scan.rotation)), sensor_radius)
                saved = scan_to_rays(
                    message, transform.transform, acquired, allow_infinite=(key == 'realsense'))
                if not saved.rays:
                    raise ValueError('empty scan: no valid observed rays')
                self.scans[key], self.scan_tf_ok[key] = saved, True
                self.scan_problems.pop(key, None)
            except ValueError as error:
                self.scan_problem(key, str(error))
                self.scans[key], self.scan_tf_ok[key] = None, False
                if self.scan_required(key):
                    self.output.publish(Twist())

    def on_localization(self, message):
        try:
            acquired, header = self.acquisition_time(message.header.stamp, time.monotonic())
            position = message.pose.pose.position
            if (message.header.frame_id != self.extra['map_frame'] or
                    header <= self.localization_header or
                    not all(math.isfinite(v) for v in (position.x, position.y, position.z))):
                raise ValueError('invalid localization sample')
            yaw(message.pose.pose.orientation)
            self.localization_at, self.localization_header = acquired, header
        except ValueError:
            self.localization_at = float('-inf')
            if self.config.enable_detour and not self.config.simple_detour:
                self.output.publish(Twist())

    def on_goal_changed(self):
        self.gate.reset(time.monotonic(), 'navigation goal changed; old maneuver discarded')
        self.cmd_at = float('-inf')
        self.handback_pending = False
        self.output.publish(Twist())

    def observation(self, now):
        pose, tf_ok = None, False
        try:
            transform = self.tf.lookup_transform(
                self.extra['pose_frame'], self.extra['base_frame'], Time())
            acquired, pose_header = self.acquisition_time(transform.header.stamp, now)
            t = transform.transform.translation
            pose = Pose(t.x, t.y, yaw(transform.transform.rotation))
            tf_ok = (now - acquired <= self.extra['tf_timeout_sec'] and
                     all(math.isfinite(v) for v in (t.x, t.y, t.z)))
            if not self.config.simple_detour:
                map_transform = self.tf.lookup_transform(
                    self.extra['map_frame'], self.extra['pose_frame'], Time())
                map_acquired, _ = self.acquisition_time(map_transform.header.stamp, now)
                tf_ok = tf_ok and now - map_acquired <= self.extra['tf_timeout_sec']
        except (TransformException, ValueError):
            tf_ok = False
        if pose is not None and tf_ok:
            if self.last_pose_sample is not None:
                old_header, old_pose = self.last_pose_sample
                dt = pose_header - old_header
                distance = math.hypot(pose.x - old_pose.x, pose.y - old_pose.y)
                angle = abs(math.atan2(math.sin(pose.yaw - old_pose.yaw),
                                       math.cos(pose.yaw - old_pose.yaw)))
                # Compare actual TF samples, not timer polls of the same sample.
                # Repeated headers cannot refresh the baseline or hide a jump.
                inconsistent_stamp = dt < 0 or (dt == 0 and (
                    distance > 1e-9 or angle > 1e-9))
                if (inconsistent_stamp or (dt > 0 and (
                        distance > 0.10 + 0.5 * dt or angle > 0.15 + 1.0 * dt))):
                    self.pose_fault_until = now + self.config.clear_confirm_sec
                if dt > 0:
                    self.last_pose_sample = (pose_header, pose)
            else:
                self.last_pose_sample = (pose_header, pose)
        tf_ok = tf_ok and now >= self.pose_fault_until
        estop_keys = ('aggregate', 'manual', 'auto')
        if self.config.enable_auto_estop:
            estop_keys += ('reason',)
        # With Auto EStop OFF the unchanged driver publishes latched states
        # only on change. Silence then is not a missing heartbeat.
        estop_fresh = all(
            (now - self.estop_at[key] <= self.extra['estop_timeout_sec']
             if self.config.enable_auto_estop else math.isfinite(self.estop_at[key]))
            for key in estop_keys)
        # Compensate cached scans using their exact acquisition pose whenever
        # available. Simple mode can also use fresh local scans without waiting
        # for dynamic TF; their original acquisition age still expires.

        def current_scan(key):
            saved = self.scans[key]
            if saved is None:
                if key not in self.scan_problems:
                    self.scan_problem(key, 'no scan received')
                return None
            limit = (self.config.realsense_timeout_sec if key == 'realsense'
                     else self.config.scan_timeout_sec)
            if now - saved.stamp > limit:
                self.scan_problem(key, f'stale cached scan: age={now - saved.stamp:.2f}s '
                                  f'limit={limit:.2f}s')
            anchor = self.scan_acquisition_poses.get(key)
            if self.config.simple_detour and (pose is None or anchor is None):
                return saved
            if pose is None or anchor is None:
                return None
            c, s = math.cos(pose.yaw), math.sin(pose.yaw)
            acquired_pose, sensor_radius = anchor
            if self.config.simple_detour:
                ca, sa = math.cos(acquired_pose.yaw), math.sin(acquired_pose.yaw)
                saved = Scan(saved.stamp, tuple(Ray(
                    acquired_pose.x + ca * ray.origin_x - sa * ray.origin_y,
                    acquired_pose.y + sa * ray.origin_x + ca * ray.origin_y,
                    acquired_pose.yaw + ray.angle, ray.distance, ray.hit,
                    ray.safety_distance_m) for ray in saved.rays))
            rotation = abs(math.atan2(math.sin(pose.yaw - acquired_pose.yaw),
                                      math.cos(pose.yaw - acquired_pose.yaw)))
            # Reduce camera-depth clearance for any possible approach since
            # acquisition, including camera motion during an in-place turn.
            movement_bound = (math.hypot(pose.x - acquired_pose.x, pose.y - acquired_pose.y)
                              + sensor_radius * rotation)
            rays = tuple(Ray(
                c * (ray.origin_x - pose.x) + s * (ray.origin_y - pose.y),
                -s * (ray.origin_x - pose.x) + c * (ray.origin_y - pose.y),
                ray.angle - pose.yaw, ray.distance, ray.hit,
                None if ray.safety_distance_m is None else
                max(0.0, ray.safety_distance_m - movement_bound)) for ray in saved.rays)
            return Scan(saved.stamp, rays)

        return Observation(
            pose=pose, lidar=current_scan('lidar'), realsense=current_scan('realsense'),
            perimeter=current_scan('perimeter'),
            nav_cmd=Velocity(self.cmd.linear.x, self.cmd.angular.z),
            nav_cmd_stamp=self.cmd_at, nav_goal_active=self.proxy.has_active_goal,
            aggregate_estop=self.estop['aggregate'], manual_estop=self.estop['manual'],
            auto_estop=self.estop['auto'], auto_reason=self.estop['reason'],
            estop_fresh=estop_fresh,
            tf_healthy=tf_ok and (self.config.simple_detour or all(self.scan_tf_ok.values())),
            localization_healthy=(
                now - self.localization_at <= self.extra['localization_timeout_sec']))

    def on_tick(self):
        self.proxy.tick()
        for key in self.scans:
            self.consume_scans(key)
        now = time.monotonic()
        previous = self.gate.state
        observation = self.observation(now)
        observation = replace(
            observation,
            manual_estop=observation.manual_estop or self.pending_manual,
            aggregate_estop=observation.aggregate_estop or self.pending_auto,
            auto_reason=self.pending_depth_fault or observation.auto_reason)
        velocity = self.gate.tick(now, observation)
        self.pending_manual = False
        self.pending_auto = False
        self.pending_depth_fault = None
        if self.gate.state == 'RECOVER_TO_NAV' and self.gate.recovery_requested:
            if not self.handback_pending:
                self.handback_pending = True
                self.proxy.notify_handback(
                    force_replan=self.gate.force_replan_requested)
            if self.proxy.motion_authorized:
                self.gate.acknowledge_recovery()
                self.handback_pending = False
                self.normal_since = now
        elif self.gate.state != 'RECOVER_TO_NAV':
            self.handback_pending = False
        if self.gate.state == 'NORMAL' and previous != 'NORMAL':
            self.normal_since = now
            if previous != 'RECOVER_TO_NAV':
                self.proxy.notify_handback()
        message = Twist()
        if self.proxy.motion_authorized:
            if self.gate.state == 'NORMAL':
                if (velocity is observation.nav_cmd and self.cmd_at >= self.normal_since and
                        now - self.cmd_at <= self.config.cmd_vel_timeout_sec and
                        observation.estop_fresh and not any((
                            observation.aggregate_estop, observation.manual_estop,
                            observation.auto_estop))):
                    message = copy.deepcopy(self.cmd)
            else:
                message.linear.x = velocity.linear_x
                message.angular.z = velocity.angular_z
        self.output.publish(message)
        signature = (self.gate.state, self.gate.phase, self.gate.reason)
        if signature != self.last_status:
            self.get_logger().info(
                f'[detour] {previous} -> {self.gate.state}: {self.gate.reason} '
                f'phase={self.gate.phase}')
        if signature != self.last_status or now >= self.next_status_at:
            for key, publisher in self.status.items():
                publisher.publish(Bool(data=self.gate.active) if key == 'active' else
                                  String(data=getattr(self.gate, key)))
            self.next_status_at = now + 1.0
        self.last_status = signature

    def destroy_node(self):
        self.output.publish(Twist())
        self.proxy.destroy()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DetourSupervisor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
