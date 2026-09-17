#!/usr/bin/env python3
"""Map-free ROS adapter for the shared detour core.

The foreground runner spins this node while waiting for terminal input. All
sensor callbacks, core requests and timer ticks therefore share one executor
thread. No odometry or map pose is consumed. Software timestamps below describe
observed ROS messages, never physical braking or measured translation.
"""
from collections import deque
from dataclasses import fields, replace
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import GetState
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import BatteryState, Imu, LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener

from detour_gate import Ray, Scan, Velocity, angle_difference
from detour_motion import GuidedConfig, GuidedDetourCore, GuidedObservation
from detour_obstacle import scan_to_rays, stamp_seconds, yaw


def twist_is_zero(message):
    return all(value == 0.0 for value in (
        message.linear.x, message.linear.y, message.linear.z,
        message.angular.x, message.angular.y, message.angular.z))


def adjusted_scan(saved, acquired_yaw, current_yaw, translation_margin):
    """Rotate retained rays by IMU delta and shorten their clearance evidence.

    The margin is a conservative *command-based* allowance; it is not measured
    distance. Optical-axis depth is additionally reduced by the camera's rotation
    arc, preserving the production guard's camera-depth safety metric.
    """
    delta = angle_difference(current_yaw, acquired_yaw)
    c, s = math.cos(delta), math.sin(delta)
    rays = []
    for ray in saved.rays:
        motion_margin = translation_margin + math.hypot(ray.origin_x, ray.origin_y) * abs(delta)
        rays.append(Ray(
            c * ray.origin_x + s * ray.origin_y,
            -s * ray.origin_x + c * ray.origin_y,
            ray.angle - delta, max(0.000001, ray.distance - translation_margin), ray.hit,
            None if ray.safety_distance_m is None else
            max(0.0, ray.safety_distance_m - motion_margin)))
    return Scan(saved.stamp, tuple(rays))


class GuidedDetourNode(Node):
    def __init__(self):
        super().__init__('guided_detour_test')
        defaults = GuidedConfig()
        self.config = GuidedConfig(**{
            field.name: self.declare_parameter(field.name, getattr(defaults, field.name)).value
            for field in fields(defaults)})
        self.core = GuidedDetourCore(self.config)
        self.extra = {
            name: self.declare_parameter(name, default).value for name, default in {
                'base_frame': 'base_link', 'imu_topic': '/imu/data_raw',
                'driver_node': '/turn_on_traymover_robot',
                'collision_monitor_node': '/collision_monitor',
                'estop_timeout_sec': 1.0, 'driver_timeout_sec': 1.0,
                'collision_monitor_timeout_sec': 1.0, 'service_timeout_sec': 1.5,
                'supervisor_rate_hz': 20.0}.items()}
        for key, value in self.extra.items():
            if key.endswith('_sec') or key.endswith('_hz'):
                if not math.isfinite(value) or value <= 0:
                    raise ValueError(f'{key} must be finite and positive')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.scans = {key: None for key in ('lidar', 'realsense', 'perimeter')}
        self.scan_headers = {}
        self.scan_yaws = {}
        self.scan_tf_ok = {key: False for key in self.scans}
        self.pending_scans = {key: deque(maxlen=10) for key in self.scans}
        self.imu_history = deque(maxlen=400)
        self.imu_yaw = None
        self.imu_at = self.imu_header = float('-inf')
        self.battery_at = self.battery_header = float('-inf')
        self.estop = {'aggregate': True, 'manual': False, 'auto': True,
                      'reason': 'STARTUP', 'request': True}
        self.estop_at = {key: float('-inf') for key in self.estop}
        self.pending_manual = self.pending_auto = False
        self.pending_automatic_source = False
        self.pending_reason = None
        self.driver_auto_enabled = False
        self.driver_parameter_at = float('-inf')
        self.cm_active = False
        self.cm_state_at = float('-inf')
        self.cm_output = Twist()
        self.cm_output_at = float('-inf')
        self.cm_zero_seen = False
        self.last_command = Velocity()
        self.graph_at = float('-inf')
        self.graph_fault = 'velocity publisher ownership not checked'
        self.adapter_fault = ''
        self.faults = []
        self.safety_events = []
        self.current_estop_event = None
        self.closing = False
        self.last_signature = None
        self.output = self.create_publisher(Twist, '/cmd_vel_detour_test', 1)
        status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status = self.create_publisher(String, '/guided_detour_test/state', status_qos)
        for key, topic in (('lidar', '/scan'), ('realsense', '/scan_realsense'),
                           ('perimeter', '/scan_detour_perimeter')):
            self.create_subscription(LaserScan, topic,
                                     lambda msg, key=key: self.on_scan(key, msg),
                                     qos_profile_sensor_data)
        self.create_subscription(Imu, self.extra['imu_topic'], self.on_imu,
                                 qos_profile_sensor_data)
        self.create_subscription(BatteryState, '/battery_state', self.on_battery,
                                 qos_profile_sensor_data)
        for key, suffix in (('aggregate', 'state'), ('manual', 'manual_state'),
                            ('auto', 'auto_state'), ('reason', 'auto_reason')):
            self.create_subscription(
                String if key == 'reason' else Bool, f'/traymover_estop/{suffix}',
                lambda msg, key=key: self.on_estop(key, msg), status_qos)
        self.create_subscription(Bool, '/traymover_estop/auto_request',
                                 lambda msg: self.on_estop('request', msg), 1)
        self.create_subscription(Twist, '/cmd_vel', self.on_output, 1)
        self.driver_parameters = self.create_client(
            GetParameters, self.extra['driver_node'].rstrip('/') + '/get_parameters')
        self.cm_lifecycle = self.create_client(
            GetState, self.extra['collision_monitor_node'].rstrip('/') + '/get_state')
        self.manual_service = self.create_client(SetBool, '/traymover_estop/set')
        self.service_futures = {}
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(
            1.0 / self.extra['supervisor_rate_hz'], self.on_tick, clock=self.steady_clock)
        self.service_timer = self.create_timer(0.5, self.poll_services, clock=self.steady_clock)
        self.publish_zero()

    def acquisition_time(self, stamp, now):
        header = stamp_seconds(stamp)
        age = self.get_clock().now().nanoseconds * 1e-9 - header
        if header <= 0 or not math.isfinite(age) or age < -0.05:
            raise ValueError('missing or future acquisition stamp')
        return now - max(0.0, age), header

    def on_imu(self, message):
        try:
            now = time.monotonic()
            acquired, header = self.acquisition_time(message.header.stamp, now)
            heading = yaw(message.orientation)
            if header <= self.imu_header or message.orientation_covariance[0] == -1:
                raise ValueError('non-advancing or unavailable IMU orientation')
            if self.imu_yaw is not None:
                dt = acquired - self.imu_at
                if dt <= 0 or abs(angle_difference(heading, self.imu_yaw)) > 0.15 + dt:
                    raise ValueError('IMU orientation discontinuity')
            self.imu_yaw, self.imu_at, self.imu_header = heading, acquired, header
            self.imu_history.append((acquired, heading))
        except ValueError:
            self.imu_yaw, self.imu_at = None, float('-inf')
            self.imu_history.clear()
            self.input_failure('invalid IMU sample during motion')

    def yaw_at(self, acquired):
        """Require a nearby IMU sample; interpolate across wrapped yaw."""
        samples = list(self.imu_history)
        for before, after in zip(samples, samples[1:]):
            if before[0] <= acquired <= after[0] and after[0] - before[0] <= 0.1:
                fraction = (acquired - before[0]) / (after[0] - before[0])
                return before[1] + fraction * angle_difference(after[1], before[1])
        if samples:
            sample = min(samples, key=lambda item: abs(item[0] - acquired))
            if abs(sample[0] - acquired) <= 0.05:
                return sample[1]
        return None

    def on_battery(self, message):
        try:
            acquired, header = self.acquisition_time(message.header.stamp, time.monotonic())
            if header <= self.battery_header:
                raise ValueError('non-advancing chassis serial stamp')
            self.battery_at, self.battery_header = acquired, header
        except ValueError:
            self.battery_at = float('-inf')
            self.input_failure('invalid chassis serial sample during motion')

    def on_scan(self, key, message):
        try:
            acquired, header = self.acquisition_time(message.header.stamp, time.monotonic())
            if header <= self.scan_headers.get(key, float('-inf')):
                raise ValueError('non-advancing scan stamp')
            self.scan_headers[key] = header
            self.pending_scans[key].append((message, acquired))
            self.consume_scans(key)
        except ValueError:
            self.pending_scans[key].clear()
            self.scans[key], self.scan_tf_ok[key] = None, False
            self.input_failure(f'invalid {key} scan during motion')

    def consume_scans(self, key):
        pending = self.pending_scans[key]
        while pending:
            message, acquired = pending[0]
            timeout = (self.config.realsense_timeout_sec if key == 'realsense'
                       else self.config.scan_timeout_sec)
            if time.monotonic() - acquired > timeout:
                pending.popleft()
                continue
            acquired_yaw = self.yaw_at(acquired)
            if acquired_yaw is None:
                break
            try:
                transform = self.tf.lookup_transform(
                    self.extra['base_frame'], message.header.frame_id,
                    Time.from_msg(message.header.stamp))
            except TransformException:
                break
            pending.popleft()
            try:
                self.scans[key] = scan_to_rays(
                    message, transform.transform, acquired, allow_infinite=(key == 'realsense'))
                self.scan_yaws[key] = acquired_yaw
                self.scan_tf_ok[key] = True
            except ValueError:
                self.scans[key], self.scan_tf_ok[key] = None, False
                self.input_failure(f'invalid {key} transform during motion')

    def on_estop(self, key, message):
        now = time.monotonic()
        previous = self.estop[key]
        was_received = math.isfinite(self.estop_at[key])
        self.estop[key], self.estop_at[key] = message.data, now
        if key == 'manual' and message.data:
            self.pending_manual = True
        if key in ('aggregate', 'auto', 'request') and message.data:
            self.pending_auto = True
        if key in ('auto', 'request') and message.data:
            self.pending_automatic_source = True
        if key == 'reason' and message.data not in ('NONE', 'NEAR_OBSTACLE'):
            self.pending_reason = message.data
        if (key in ('request', 'auto', 'aggregate') and message.data
                and (not previous or not was_received)):
            field = {'request': 't_auto_request', 'auto': 't_auto_state',
                     'aggregate': 't_aggregate_estop'}[key]
            event = self.current_estop_event
            if event is None or field in event or now - event['started_at'] > 5.0:
                event = {'started_at': now, 'clock': 'monotonic',
                         'command_was_nonzero': self.last_command != Velocity(),
                         'auto_command_was_nonzero': False}
                self.safety_events.append(event)
                self.current_estop_event = event
            event[field] = now
            commanded_motion = self.last_command != Velocity()
            source = {'request': 'auto_request', 'auto': 'auto_state',
                      'aggregate': 'aggregate_estop'}[key]
            event[source + '_command_was_nonzero'] = commanded_motion
            # Aggregate may come from the manual EStop. Only an actual Auto
            # request/state observed while commanding motion proves that Auto
            # interrupted it. Preserve an earlier true across callback order.
            if key in ('request', 'auto') and commanded_motion:
                event['auto_command_was_nonzero'] = True
        if ((key != 'reason' and message.data) or
                (key == 'reason' and message.data not in ('NONE', 'NEAR_OBSTACLE'))):
            self.publish_zero()

    def on_output(self, message):
        now = time.monotonic()
        values = (message.linear.x, message.linear.y, message.linear.z,
                  message.angular.x, message.angular.y, message.angular.z)
        if not all(math.isfinite(value) for value in values):
            self.record_fault('Collision Monitor emitted invalid velocity')
            return
        self.cm_output, self.cm_output_at = message, now
        zero = twist_is_zero(message)
        self.cm_zero_seen = self.cm_zero_seen or zero
        event = self.current_estop_event
        if event is not None and zero and now >= event['started_at']:
            event.setdefault('t_first_output_zero', now)
            # Driver state can arrive after the first zero from auto_request.
            # Keep a separate post-aggregate observation for the recovery proof.
            if 't_aggregate_estop' in event and now >= event['t_aggregate_estop']:
                event.setdefault('t_output_zero', now)
        if (event is not None and not zero and self.estop['aggregate']
                and 't_aggregate_estop' in event and now > event['t_aggregate_estop']):
            event.setdefault('t_nonzero_after_aggregate', now)
            self.record_fault('nonzero Collision Monitor output observed during aggregate EStop')

    def record_fault(self, reason):
        if reason != self.adapter_fault:
            self.faults.append({'time': time.monotonic(), 'reason': reason})
        self.adapter_fault = reason
        self.publish_zero()

    def input_failure(self, reason):
        if self.last_command != Velocity():
            self.record_fault(reason)
        else:
            self.publish_zero()

    def poll_services(self):
        now = time.monotonic()
        for name, client, request in (
                ('driver', self.driver_parameters,
                 GetParameters.Request(names=['auto_estop_enabled'])),
                ('collision', self.cm_lifecycle, GetState.Request())):
            old = self.service_futures.get(name)
            if old is not None:
                future, requested_at = old
                if future.done():
                    try:
                        result = future.result()
                        if name == 'driver':
                            self.driver_auto_enabled = (
                                len(result.values) == 1
                                and result.values[0].type == ParameterType.PARAMETER_BOOL
                                and result.values[0].bool_value)
                            self.driver_parameter_at = now
                        else:
                            self.cm_active = result.current_state.id == 3
                            self.cm_state_at = now
                    except Exception as error:
                        self.get_logger().warning(f'{name} health request failed: {error}')
                    self.service_futures.pop(name, None)
                elif now - requested_at >= self.extra['service_timeout_sec']:
                    future.cancel()
                    self.service_futures.pop(name, None)
                else:
                    continue
            if client.service_is_ready():
                self.service_futures[name] = (client.call_async(request), now)
        self.refresh_graph(now)

    def refresh_graph(self, now):
        def publishers(topic):
            return [(info.node_namespace.rstrip('/') + '/' + info.node_name)
                    for info in self.get_publishers_info_by_topic(topic)]
        final = publishers('/cmd_vel')
        incoming = publishers('/cmd_vel_detour_test')
        expected = self.extra['collision_monitor_node']
        own = self.get_namespace().rstrip('/') + '/' + self.get_name()
        problems = []
        if final != [expected]:
            problems.append(f'/cmd_vel publishers {final!r}; require only {expected}')
        if incoming != [own]:
            problems.append(f'/cmd_vel_detour_test publishers {incoming!r}; require only {own}')
        self.graph_fault = '; '.join(problems)
        self.graph_at = now

    def observation(self, now=None):
        now = time.monotonic() if now is None else now

        def current_scan(key):
            saved = self.scans[key]
            if saved is None or self.imu_yaw is None:
                return None
            # A full maximum-command allowance avoids relying on wheel distance
            # or exact command delivery timing. It does not prove actual travel.
            maximum_speed = max(self.config.test_straight_speed, self.config.estop_test_speed)
            margin = max(0.0, now - saved.stamp) * maximum_speed
            return adjusted_scan(saved, self.scan_yaws[key], self.imu_yaw, margin)

        services_fresh = now - self.driver_parameter_at <= self.extra['service_timeout_sec']
        driver_fresh = (now - self.battery_at <= self.extra['driver_timeout_sec']
                        and services_fresh and self.driver_auto_enabled)
        collision_fresh = (
            self.cm_active and self.cm_zero_seen
            and now - self.cm_state_at <= self.extra['service_timeout_sec']
            and now - self.cm_output_at <= self.extra['collision_monitor_timeout_sec'])
        fault = self.adapter_fault or self.graph_fault
        if now - self.graph_at > self.extra['service_timeout_sec']:
            fault = fault or 'velocity ownership information stale'
        if not self.manual_service.service_is_ready():
            fault = fault or 'manual EStop service unavailable'
        return GuidedObservation(
            lidar=current_scan('lidar'), realsense=current_scan('realsense'),
            perimeter=current_scan('perimeter'),
            imu_yaw=self.imu_yaw, imu_stamp=self.imu_at,
            aggregate_estop=self.estop['aggregate'], manual_estop=self.estop['manual'],
            auto_estop=self.estop['auto'] or self.estop['request'],
            auto_reason=self.estop['reason'],
            estop_fresh=all(now - value <= self.extra['estop_timeout_sec']
                            for value in self.estop_at.values()),
            tf_healthy=all(self.scan_tf_ok.values()),
            collision_monitor_fresh=collision_fresh, driver_fresh=driver_fresh,
            health_fault=fault)

    def preflight(self):
        now = time.monotonic()
        obs = self.observation(now)
        checks = {}

        def check(name, okay, stamp=None, detail=''):
            age = (max(0.0, now - stamp) if stamp is not None
                   and math.isfinite(stamp) else None)
            checks[name] = {'status': 'PASS' if okay else 'FAIL',
                            'age_sec': age, 'detail': detail}

        check('IMU', self.imu_yaw is not None
              and now - self.imu_at <= self.config.imu_timeout_sec, self.imu_at)
        for key, label in (('lidar', 'LiDAR'), ('realsense', 'RealSense scan'),
                           ('perimeter', 'Perimeter scan')):
            saved = self.scans[key]
            timeout = (self.config.realsense_timeout_sec if key == 'realsense'
                       else self.config.scan_timeout_sec)
            stamp = saved.stamp if saved else None
            check(label, bool(saved and saved.rays and now - stamp <= timeout), stamp)
        check('Driver serial', now - self.battery_at <= self.extra['driver_timeout_sec'],
              self.battery_at)
        check('Driver Auto EStop parameter', self.driver_auto_enabled
              and now - self.driver_parameter_at <= self.extra['service_timeout_sec'],
              self.driver_parameter_at,
              'ENABLED' if self.driver_auto_enabled else 'UNVERIFIED / DISABLED')
        for key, label in (('aggregate', 'Aggregate EStop'), ('manual', 'Manual EStop'),
                           ('auto', 'Auto EStop'), ('request', 'Auto request heartbeat')):
            check(label, not self.estop[key]
                  and now - self.estop_at[key] <= self.extra['estop_timeout_sec'],
                  self.estop_at[key], 'ACTIVE' if self.estop[key] else 'FREE')
        check('Auto reason', self.estop['reason'] == 'NONE'
              and now - self.estop_at['reason'] <= self.extra['estop_timeout_sec'],
              self.estop_at['reason'], self.estop['reason'])
        check('Collision Monitor', obs.collision_monitor_fresh, self.cm_output_at,
              'ACTIVE + zero output observed' if obs.collision_monitor_fresh
              else 'missing active/output proof')
        check('Velocity ownership', not self.graph_fault
              and now - self.graph_at <= self.extra['service_timeout_sec'],
              self.graph_at, self.graph_fault)
        check('Manual EStop service', self.manual_service.service_is_ready())
        check('Sensor transforms', obs.tf_healthy,
              detail='exact-time sensor -> base_link; no map/odom')
        if self.adapter_fault:
            check('Latched adapter fault', False, detail=self.adapter_fault)
        return checks

    def on_tick(self):
        if self.closing:
            self.publish_zero()
            return
        for key in self.scans:
            self.consume_scans(key)
        now = time.monotonic()
        obs = self.observation(now)
        obs = replace(obs, manual_estop=obs.manual_estop or self.pending_manual,
                      aggregate_estop=obs.aggregate_estop or self.pending_auto,
                      auto_estop=obs.auto_estop or self.pending_automatic_source,
                      auto_reason=self.pending_reason or obs.auto_reason)
        velocity = self.core.tick(now, obs)
        self.pending_manual = self.pending_auto = False
        self.pending_automatic_source = False
        self.pending_reason = None
        if (velocity.linear_x < 0 or (velocity.linear_x != 0 and velocity.angular_z != 0)
                or not all(math.isfinite(value) for value
                           in (velocity.linear_x, velocity.angular_z))):
            self.record_fault('invalid, reverse or simultaneous turn/straight command rejected')
            self.core.abort(now, self.adapter_fault)
            velocity = Velocity()
        message = Twist()
        message.linear.x, message.angular.z = velocity.linear_x, velocity.angular_z
        self.last_command = velocity
        self.output.publish(message)
        signature = (self.core.state, self.core.phase, self.core.reason)
        if signature != self.last_signature:
            self.status.publish(String(data=' | '.join(signature)))
            self.last_signature = signature

    def publish_zero(self):
        self.last_command = Velocity()
        self.output.publish(Twist())

    def assert_manual_estop(self):
        """Manual stop only: this adapter never sends a release request."""
        self.publish_zero()
        self.core.abort(time.monotonic(), 'manual EStop requested by operator')
        if self.manual_service.service_is_ready():
            return self.manual_service.call_async(SetBool.Request(data=True))
        self.record_fault('manual EStop service unavailable during operator stop')
        return None

    def snapshot(self):
        return {
            'state': self.core.state, 'phase': self.core.phase, 'reason': self.core.reason,
            'command': {'linear_x': self.last_command.linear_x,
                        'angular_z': self.last_command.angular_z},
            'output': {'linear_x': self.cm_output.linear.x, 'angular_z': self.cm_output.angular.z},
            'final_output': {'linear_x': self.cm_output.linear.x,
                             'angular_z': self.cm_output.angular.z, 'stamp': self.cm_output_at},
            'output_zero': twist_is_zero(self.cm_output),
            'estop': dict(self.estop),
            'event_times': dict(self.current_estop_event or {}),
            'safety_events': [dict(event) for event in self.safety_events],
            'faults': list(self.faults), 'preflight': self.preflight()}

    def close_zero(self, duration=0.3):
        self.closing = True
        self.core.abort(time.monotonic(), 'guided runner shutdown')
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish_zero()
            rclpy.spin_once(self, timeout_sec=0.02)
        self.publish_zero()

    def destroy_node(self):
        self.publish_zero()
        return super().destroy_node()
