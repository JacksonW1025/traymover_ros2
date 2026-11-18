#!/usr/bin/env python3
"""Closed-loop synthetic-room experiment with real STM32 driver and real NDT.

No real device is opened: the MCU endpoint is always a newly created PTY.
The controller is a small waypoint tracker, not Nav2's planner/controller.
Requires the ROS 2 workspace environment. Results include CSV, rosbag and plots.
"""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import pty
import select
import signal
import struct
import subprocess
import sys
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped, Twist
from lifecycle_msgs.srv import ChangeState
from nav_msgs.msg import Odometry, Path as RosPath
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.serialization import serialize_message
from rosgraph_msgs.msg import Clock
import rosbag2_py
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/turn_on_traymover_robot'))
from turn_on_traymover_robot import traymover_robot as driver


def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def yaw_of(q):
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


def plant_step(pose, vx, wz, dt):
    """Independent midpoint plant with small substeps (not driver integration)."""
    x, y, yaw = pose
    for _ in range(5):
        step = dt / 5
        x += vx * math.cos(yaw + wz*step/2) * step
        y += vx * math.sin(yaw + wz*step/2) * step
        yaw += wz * step
    return np.array([x, y, yaw])


ROOM_PILLARS = ((4.5, 3.8, .7, 2.7), (-3.5, -2.5, .9, 2.3),
                (4.2, -3.1, 1.2, 1.8))


def room_points():
    """Asymmetric 16 x 12 m room, floor, walls and three differently sized pillars."""
    surfaces = []
    def rectangle(axis, fixed, a, b):
        aa, bb = np.meshgrid(np.arange(*a, 0.12), np.arange(*b, 0.12))
        points = np.empty((aa.size, 3))
        points[:, axis] = fixed
        other = [i for i in range(3) if i != axis]
        points[:, other[0]], points[:, other[1]] = aa.ravel(), bb.ravel()
        surfaces.append(points)
    rectangle(2, 0, (-8, 8.01), (-6, 6.01))
    for x in (-8, 8):
        rectangle(0, x, (-6, 6.01), (0, 3.01))
    for y in (-6, 6):
        rectangle(1, y, (-8, 8.01), (0, 3.01))
    for x, y, w, h in ROOM_PILLARS:
        for xx in (x-w/2, x+w/2):
            rectangle(0, xx, (y-w/2, y+w/2+.01), (0, h))
        for yy in (y-w/2, y+w/2):
            rectangle(1, yy, (x-w/2, x+w/2+.01), (0, h))
    return np.concatenate(surfaces).astype(np.float32)


def nearest_angular_indices(bins, distance):
    """Same stable nearest-per-bin result as lexsort, in linear time."""
    if len(bins) == 0:
        return np.array([], dtype=int)
    minima = np.full(int(bins.max())+1, np.inf)
    np.minimum.at(minima, bins, distance)
    candidates = np.flatnonzero(distance == minima[bins])
    first = np.full(len(minima), len(distance), dtype=int)
    np.minimum.at(first, bins[candidates], candidates)
    return first[first < len(distance)]


def make_scan(world, pose, rng, noise=0.01, max_range=18.):
    """Nearest surface sample per angular bin; approximate occlusion, not a ray tracer."""
    delta = world.astype(float) - [pose[0], pose[1], 0.2]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    # Avoid a BLAS thread pool for a three-column rotation: otherwise the
    # sensor generator can consume all Jetson cores and distort runtime tests.
    base = np.empty_like(delta)
    base[:, 0] = c*delta[:, 0] + s*delta[:, 1]
    base[:, 1] = -s*delta[:, 0] + c*delta[:, 1]
    base[:, 2] = delta[:, 2]
    sensor = base - [.435, 0, 1.365]
    distance = np.linalg.norm(sensor, axis=1)
    elevation = np.arctan2(sensor[:, 2], np.linalg.norm(sensor[:, :2], axis=1))
    keep = (distance > .6) & (distance < max_range) & (elevation > -.50) & (elevation < .45)
    base, sensor, distance, elevation = base[keep], sensor[keep], distance[keep], elevation[keep]
    azimuth = np.arctan2(sensor[:, 1], sensor[:, 0])
    bins = np.floor((azimuth + math.pi) / (2*math.pi) * 720).astype(int) * 32
    bins += np.clip(((elevation + .5) / .95 * 32).astype(int), 0, 31)
    points = base[nearest_angular_indices(bins, distance)]
    points = points[rng.random(len(points)) > .15]
    return (points + rng.normal(0, noise, points.shape)).astype(np.float32)


def cloud(points, stamp, frame):
    data = np.column_stack([points, np.ones(len(points), dtype=np.float32)]).astype('<f4')
    msg = PointCloud2(header=Header(stamp=stamp, frame_id=frame), height=1, width=len(data),
                      is_bigendian=False, point_step=16, row_step=len(data)*16,
                      is_dense=True, data=data.tobytes())
    msg.fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
                  for i, name in enumerate(('x', 'y', 'z', 'intensity'))]
    return msg


def write_pcd(path, points):
    with open(path, 'w') as out:
        out.write('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\n'
                  'TYPE F F F F\nCOUNT 1 1 1 1\n'
                  f'WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
                  f'POINTS {len(points)}\nDATA ascii\n')
        np.savetxt(out, np.column_stack([points, np.ones(len(points))]), fmt='%.4f')


class Bag:
    def __init__(self, output):
        self.lock = threading.RLock()
        self.writer = rosbag2_py.SequentialWriter()
        self.writer.open(rosbag2_py.StorageOptions(uri=str(output), storage_id='sqlite3'),
                         rosbag2_py.ConverterOptions('', ''))
        self.topics = set()

    def add(self, topic, msg, stamp):
        with self.lock:
            self._add(topic, msg, stamp)

    def _add(self, topic, msg, stamp):
        if topic not in self.topics:
            name = type(msg).__module__.split('.')[0] + '/msg/' + type(msg).__name__
            self.writer.create_topic(rosbag2_py.TopicMetadata(
                name=topic, type=name, serialization_format='cdr'))
            self.topics.add(topic)
        self.writer.write(topic, serialize_message(msg), int(stamp * 1e9))

    def close(self):
        with self.lock:
            self.writer = None


class VirtualMCU:
    def __init__(self, master, scenario, output):
        self.master, self.scenario = master, scenario
        self.lock = threading.Lock()
        self.command = (0., 0.)
        self.velocity = (0., 0.)
        self.elapsed = 0.
        self.stop = threading.Event()
        self.output = open(output, 'w')
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        buffer = bytearray()
        while not self.stop.is_set():
            if not select.select([self.master], [], [], .02)[0]:
                continue
            try:
                buffer.extend(os.read(self.master, 4096))
            except OSError:
                continue
            while True:
                request = driver.extract_next_valid_frame(buffer)
                if request is None:
                    break
                with self.lock:
                    if request[3] == 2:
                        self.command = tuple(int.from_bytes(request[k:k+2], 'big', signed=True)/1000
                                             for k in (10, 12))
                    vx, wz = self.velocity
                    elapsed = self.elapsed
                if self.scenario == 'dropout' and 25 <= elapsed < 26:
                    continue
                scale = getattr(self, 'velocity_gain', 1.05 if self.scenario == 'bias' else 1.)
                frame = bytearray(40)
                frame[:5] = bytes.fromhex('7f 7f 28 02 60')
                frame[6:8] = bytes([1, 71])
                frame[29:31] = round(vx*scale*1000).to_bytes(2, 'big', signed=True)
                # The option-21 STM32 returns yaw with the opposite sign to ROS +Z.
                frame[31:33] = round(-wz*scale*1000).to_bytes(2, 'big', signed=True)
                frame[37] = driver.compute_checksum(frame)
                frame[38:] = b'\r\n'
                self.output.write(json.dumps({'t': elapsed, 'tx': request.hex(), 'rx': frame.hex()})+'\n')
                os.write(self.master, frame[:13])
                os.write(self.master, frame[13:])

    def close(self):
        self.stop.set()
        self.thread.join(timeout=2)
        self.output.close()


def run_scenario(args, scenario, model=None):
    output = args.output / scenario
    output.mkdir(parents=True, exist_ok=False)
    world = room_points() if model is None else model.prior_map
    source = getattr(args, 'odometry_source', 'wheel')
    write_pcd(output / 'room.pcd', world)
    production = yaml.safe_load((ROOT / 'src/traymover_robot_nav/config/localization.yaml').read_text())
    params = production['/**']['ros__parameters']
    params.update(use_sim_time=True, map_path=str(output / 'room.pcd'),
                  auto_correction_enabled=True, height_boost_enabled=False,
                  max_map_odom_update_translation=.50, max_map_odom_update_rotation=.25)
    (output / 'ndt.yaml').write_text(yaml.safe_dump(production))
    bag = Bag(output / 'bag')
    environment = dict(os.environ, ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY='1',
                       ROS_LOG_DIR=str(output / 'ros_logs'))
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY='1')
    rclpy.init(domain_id=args.domain)
    master, slave = pty.openpty()
    mcu = VirtualMCU(master, scenario, output / 'serial.jsonl')
    if model is not None:
        mcu.velocity_gain = model.wheel_gain
    node = rclpy.create_node('option21_simulation')
    profile = yaml.safe_load((ROOT / 'src/turn_on_traymover_robot/config/stm32_navigation.yaml').read_text())
    overrides = profile['turn_on_traymover_robot']['ros__parameters']
    overrides.update(usart_port_name=os.ttyname(slave), use_sim_time=True, status_poll_rate_hz=0.)
    if source == 'fastlio':
        overrides.update(odom_source_mode='none', publish_odom_tf=False)
    chassis = driver.TurnOnTraymoverRobot(parameter_overrides=[
        Parameter(name, value=value) for name, value in overrides.items()])
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(chassis)
    clock_pub = node.create_publisher(Clock, '/clock', 10)
    cmd_pub = node.create_publisher(Twist, '/cmd_vel', 10)
    scan_pub = node.create_publisher(PointCloud2, '/point_cloud_localization', 5)
    raw_pub = node.create_publisher(PointCloud2, '/point_cloud_raw', 5) if model else None
    if model:
        from sensor_msgs.msg import Imu
        imu_pub = node.create_publisher(Imu, '/imu/data_raw', 100)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    map_pub = node.create_publisher(PointCloud2, '/simulation/map', latched)
    init_pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    path_pubs = {name: node.create_publisher(RosPath, '/simulation/'+name, latched)
                 for name in ('truth_path', 'wheel_path', 'localized_path', 'goals')}
    paths = {name: RosPath(header=Header(frame_id='map')) for name in path_pubs}
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    static = StaticTransformBroadcaster(node)
    static_msg = TransformStamped()
    static_msg.header.frame_id, static_msg.child_frame_id = 'base_footprint', 'base_link'
    static_msg.transform.translation.z = .20
    static_msg.transform.rotation.w = 1.
    static_messages = [static_msg]
    if source == 'fastlio':
        static_messages = []
        for parent, child, z in [('odom', 'camera_init', 0.), ('body', 'base_link', .12),
                                 ('base_link', 'base_footprint', -.20)]:
            transform = TransformStamped()
            transform.header.frame_id, transform.child_frame_id = parent, child
            transform.transform.translation.z = z
            transform.transform.rotation.w = 1.
            static_messages.append(transform)
    static.sendTransform(static_messages)
    sim_time = 10.
    odoms, aligned = [], []
    def record(topic, msg):
        bag.add(topic, msg, sim_time)
    node.create_subscription(Odometry, '/odom',
                             lambda msg: (odoms.append(msg), record('/odom', msg)), 100)
    node.create_subscription(PoseWithCovarianceStamped, '/pcl_pose',
                             lambda msg: (aligned.append(msg), record('/pcl_pose', msg)), 100)
    node.create_subscription(TFMessage, '/tf', lambda msg: record('/tf', msg), 100)
    # Write the exact static transform too: recorder starts after its publication.
    record('/tf_static', TFMessage(transforms=static_messages))
    fastlio_process = None
    fastlio_log = None
    if source == 'fastlio':
        fastlio_params = yaml.safe_load((ROOT / 'src/traymover_robot_slam/FAST_LIO/config/traymover.yaml').read_text())
        fastlio_params['/**']['ros__parameters'].update({
            'use_sim_time': True, 'pcd_save.pcd_save_en': False, 'publish.map_en': False,
            'publish.path_en': False, 'runtime_pos_log_enable': False})
        (output / 'fastlio.yaml').write_text(yaml.safe_dump(fastlio_params))
        fastlio_log = open(output / 'fastlio.log', 'w')
        fastlio_process = subprocess.Popen([
            'ros2', 'run', 'fast_lio', 'fastlio_mapping', '--ros-args',
            '--params-file', str(output / 'fastlio.yaml'), '-r', '/Odometry:=/odom'],
            env=environment, stdout=fastlio_log, stderr=subprocess.STDOUT, start_new_session=True)
    log = open(output / 'ndt.log', 'w')
    process = subprocess.Popen([
        'ros2', 'run', 'lidar_localization_ros2', 'lidar_localization_node',
        '--ros-args', '--params-file', str(output / 'ndt.yaml'),
        '-r', '/cloud:=/point_cloud_localization'],
        env=environment, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    background_thread = None
    def spin(seconds):
        if background_thread is not None:
            time.sleep(max(0., seconds))
            return
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=min(.002, max(0., end-time.monotonic())))
    def publish_clock():
        clock = Clock()
        clock.clock.sec = int(sim_time)
        clock.clock.nanosec = round((sim_time-int(sim_time))*1e9)
        clock_pub.publish(clock)
    def change(transition):
        client = node.create_client(ChangeState, '/lidar_localization/change_state')
        end = time.monotonic()+20
        while not client.service_is_ready() and time.monotonic() < end:
            publish_clock()
            spin(.05)
            if process.poll() is not None:
                raise RuntimeError('NDT process exited; see ndt.log')
        request = ChangeState.Request()
        request.transition.id = transition
        future = client.call_async(request)
        while not future.done() and time.monotonic() < end:
            publish_clock()
            spin(.05)
        if not future.done() or not future.result().success:
            raise RuntimeError('NDT lifecycle transition failed; see ndt.log')
        node.destroy_client(client)
    truth = np.zeros(3)
    history = [[sim_time, *truth]]
    rows = []
    waypoints = (np.array([[3., 0.], [3., 2.], [0., 2.], [0., 0.]]) if model is None
                 else np.array([[15., 0.], [15., 8.], [15., 0.], [0., 0.]]))
    for x, y in waypoints:
        goal = PoseStamped(header=Header(frame_id='map'))
        goal.pose.position.x, goal.pose.position.y = float(x), float(y)
        goal.pose.orientation.w = 1.
        paths['goals'].poses.append(goal)
    goal_index, completed_at = 0, None
    rng = np.random.default_rng(getattr(args, 'seed', 20261117))
    input_hash = hashlib.sha256()
    try:
        if model is not None:
            # Large synthetic point-cloud generation must not starve the ROS
            # observer and make queued TF look like an estimator's latency.
            def background_spin():
                try:
                    executor.spin()
                except rclpy.executors.ExternalShutdownException:
                    pass
            background_thread = threading.Thread(target=background_spin, daemon=True)
            background_thread.start()
        publish_clock()
        spin(.3)
        change(1)
        change(3)
        initial = PoseWithCovarianceStamped()
        initial.header.frame_id = 'map'
        initial_error = (.3, -.2, .1) if model is None else model.initial_error
        initial.pose.pose.position.x, initial.pose.pose.position.y = initial_error[:2]
        initial.pose.pose.position.z = .2
        initial.pose.pose.orientation.z = math.sin(initial_error[2]/2)
        initial.pose.pose.orientation.w = math.cos(initial_error[2]/2)
        initial.pose.covariance[0] = initial.pose.covariance[7] = .25
        initial.pose.covariance[35] = .1
        if model is None:
            init_pub.publish(initial)
            record('/initialpose', initial)
        map_msg = cloud(world, rclpy.time.Time(seconds=10).to_msg(), 'map')
        map_pub.publish(map_msg)
        record('/simulation/map', map_msg)
        localized = None
        for step in range(round(args.duration/.05)):
            tick_start = time.monotonic()
            elapsed = step*.05
            sim_time = 10.+elapsed
            # The plant follows the last decoded MCU command, never odom/TF truth.
            with mcu.lock:
                command = mcu.command
                previous_velocity = np.array(mcu.velocity)
            actual = previous_velocity + .35*(np.array(command)-previous_velocity)
            if model is None:
                truth = plant_step(truth, *actual, .05)
            else:
                truth, actual = model.motion(step)
            history.append([sim_time, *truth])
            with mcu.lock:
                mcu.velocity = tuple(actual)
                mcu.elapsed = elapsed
            cmd = Twist()
            if model is not None:
                command = model.command(elapsed)
                cmd.linear.x, cmd.angular.z = map(float, command)
                for imu in model.imu_messages(step):
                    imu_pub.publish(imu)
                    # Canonical sensor values, excluding transport/CDR padding.
                    q, a, g = imu.orientation, imu.linear_acceleration, imu.angular_velocity
                    input_hash.update(struct.pack('<II10d', imu.header.stamp.sec,
                        imu.header.stamp.nanosec, q.x, q.y, q.z, q.w,
                        a.x, a.y, a.z, g.x, g.y, g.z))
                if elapsed >= model.motion_end:
                    completed_at = model.motion_end
            elif localized is not None and elapsed > 2. and completed_at is None:
                distance = np.linalg.norm(waypoints[goal_index]-localized[:2])
                if distance < .10:
                    goal_index += 1
                    if goal_index == len(waypoints):
                        completed_at = elapsed
                        goal_index -= 1
                if completed_at is None:
                    dx, dy = waypoints[goal_index]-localized[:2]
                    angle = wrap(math.atan2(dy, dx)-localized[2])
                    cmd.linear.x = min(.4, .7*math.hypot(dx, dy))*max(0., math.cos(angle))
                    cmd.angular.z = float(np.clip(1.8*angle, -.55, .55))
            cmd_pub.publish(cmd)
            record('/cmd_vel', cmd)
            # Let the command arrive before advancing the driver's ROS timer.
            spin(.002)
            publish_clock()
            spin(.004)
            stamp = Clock().clock
            stamp.sec = int(sim_time)
            stamp.nanosec = round((sim_time-int(sim_time))*1e9)
            # For paired tracking tests, both frontends warm up before the
            # SAME initial pose and first NDT cloud. Startup is tested separately.
            if model is not None and step == 100:
                initial.header.stamp = stamp
                init_pub.publish(initial)
                record('/initialpose', initial)
            if (step % (2 if model is None else 1) == 0
                    and not (scenario == 'dropout' and 25 <= elapsed < 28)):
                if model is not None and model.scan_cache is not None:
                    points = model.scan_cache[step]
                else:
                    live_world = world if model is None else model.live_points(elapsed)
                    points = make_scan(live_world, truth, rng, noise=.01 if model is None else .02,
                                       max_range=18. if model is None else model.max_range)
                scan = cloud(points, stamp, 'base_link')
                if model is None or step >= 104:
                    scan_pub.publish(scan)
                    record('/point_cloud_localization', scan)
                if model:
                    raw = model.raw_cloud(points, stamp)
                    input_hash.update(struct.pack('<II', raw.header.stamp.sec, raw.header.stamp.nanosec))
                    input_hash.update(raw.data)
                    raw_pub.publish(raw)
                    if step % 10 == 0:
                        record('/point_cloud_raw', raw)
            spin(max(.001, .05/args.rate-(time.monotonic()-tick_start)))
            localized, localized_stamp = None, float('nan')
            try:
                transform = buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
                p, q = transform.transform.translation, transform.transform.rotation
                localized = np.array([p.x, p.y, yaw_of(q)])
                localized_stamp = transform.header.stamp.sec+transform.header.stamp.nanosec/1e9
            except TransformException:
                pass
            if step % 2 == 0:
                odom = odoms[-1] if odoms else None
                op = odom.pose.pose if odom else None
                raw = np.array([op.position.x, op.position.y, yaw_of(op.orientation)]) if op else np.full(3, np.nan)
                ostamp = odom.header.stamp.sec+odom.header.stamp.nanosec/1e9 if odom else float('nan')
                if source == 'fastlio':
                    try:
                        raw_tf = buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
                        p, q = raw_tf.transform.translation, raw_tf.transform.rotation
                        raw = np.array([p.x, p.y, yaw_of(q)])
                        ostamp = raw_tf.header.stamp.sec+raw_tf.header.stamp.nanosec/1e9
                    except TransformException:
                        raw = np.full(3, np.nan)
                h = np.array(history)
                truth_at = lambda t: np.array([np.interp(t, h[:, 0], h[:, i]) for i in (1, 2, 3)])
                raw_truth = truth_at(ostamp)
                loc_truth = truth_at(localized_stamp)
                loc = localized if localized is not None else np.full(3, np.nan)
                rows.append([elapsed, *truth, *raw, *loc,
                             np.linalg.norm(raw[:2]-raw_truth[:2]), abs(wrap(raw[2]-raw_truth[2])),
                             np.linalg.norm(loc[:2]-loc_truth[:2]), abs(wrap(loc[2]-loc_truth[2])),
                             sim_time-ostamp, sim_time-localized_stamp, goal_index])
                for name, value in [('truth_path', truth), ('wheel_path', raw), ('localized_path', loc)]:
                    if not np.isfinite(value).all():
                        continue
                    pose = PoseStamped(header=Header(frame_id='map', stamp=stamp))
                    pose.pose.position.x, pose.pose.position.y = map(float, value[:2])
                    pose.pose.orientation.z, pose.pose.orientation.w = math.sin(value[2]/2), math.cos(value[2]/2)
                    paths[name].poses.append(pose)
                if step % 20 == 0:
                    for name, path in paths.items():
                        path.header.stamp = stamp
                        path_pubs[name].publish(path)
                        record('/simulation/'+name, path)
            if step % 200 == 0:
                print(f'{scenario}: t={elapsed:.0f}s goal={goal_index+1}/4 NDT={len(aligned)} '
                      f'truth={truth[:2].round(2)} tf_age={sim_time-localized_stamp:.3f}s', flush=True)
            if completed_at is not None and elapsed-completed_at >= 5:
                break
        spin(.2)
    finally:
        # SIGINT can invalidate the ROS context before this cleanup executes.
        # Always reap estimator subprocesses even when publishing is impossible.
        if rclpy.ok():
            cmd_pub.publish(Twist())
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        log.close()
        if fastlio_process is not None:
            if fastlio_process.poll() is None:
                os.killpg(fastlio_process.pid, signal.SIGINT)
                try:
                    fastlio_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(fastlio_process.pid, signal.SIGKILL)
                    fastlio_process.wait(timeout=5)
            fastlio_log.close()
        executor.shutdown()
        if background_thread is not None:
            background_thread.join(timeout=2)
        chassis.destroy_node()
        mcu.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        os.close(master)
        os.close(slave)
        bag.close()
    names = ['t', 'truth_x', 'truth_y', 'truth_yaw', 'odom_x', 'odom_y', 'odom_yaw',
             'localized_x', 'localized_y', 'localized_yaw', 'odom_error_m', 'odom_yaw_error_rad',
             'localized_error_m', 'localized_yaw_error_rad', 'odom_age_s', 'localized_age_s', 'goal_index']
    with open(output / 'metrics.csv', 'w') as out:
        writer = csv.writer(out)
        writer.writerow(names)
        writer.writerows(rows)
    array = np.asarray(rows)
    valid = np.isfinite(array[:, 12]) & (array[:, 0] > 4.)
    results = {'scenario': scenario, 'simulated_seconds': float(array[-1, 0]),
               'waypoints_completed': 4 if completed_at is not None else goal_index,
               'ndt_accepted': len(aligned), 'samples': len(array),
               'localized_valid_fraction': float(np.mean(valid[array[:, 0] > 4.])),
               'final_true_goal_error_m': float(np.linalg.norm(truth[:2]-waypoints[-1])),
               'odom_rmse_m': float(np.sqrt(np.mean(array[:, 10]**2))),
               'localized_rmse_m': float(np.sqrt(np.mean(array[valid, 12]**2))) if valid.any() else None,
               'localized_max_error_m': float(np.max(array[valid, 12])) if valid.any() else None,
               'localized_yaw_rmse_deg': float(np.degrees(np.sqrt(np.mean(array[valid, 13]**2)))) if valid.any() else None,
               'assumed_velocity_gain': 1.05 if scenario == 'bias' else 1.,
               'cloud_noise_std_m': .01, 'seed': 20261117,
               'note': 'Synthetic static geometry and assumed plant; not hardware accuracy evidence.'}
    results['acceptance_checks'] = {
        'four_waypoints_completed': results['waypoints_completed'] == 4,
        'ndt_updates_received': len(aligned) >= 10,
        'global_position_rmse_below_20cm': results['localized_rmse_m'] is not None
            and results['localized_rmse_m'] < .20,
        'global_yaw_rmse_below_3deg': results['localized_yaw_rmse_deg'] is not None
            and results['localized_yaw_rmse_deg'] < 3.,
        'true_destination_error_below_25cm': results['final_true_goal_error_m'] < .25,
    }
    results['passed'] = all(results['acceptance_checks'].values())
    if model is not None:
        results.update(odometry_source=source, input_sensor_sha256=input_hash.hexdigest(),
                       experiment=model.description, seed=getattr(args, 'seed', 20261117),
                       assumed_velocity_gain=model.wheel_gain, prescribed_trajectory=True)
        results.pop('waypoints_completed')
        results.pop('acceptance_checks')
        results.pop('passed')
    (output / 'results.json').write_text(json.dumps(results, indent=2)+'\n')
    return results


def plot_results(output, results):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(len(results), 2, figsize=(12, 4*len(results)), squeeze=False)
    for row, result in enumerate(results):
        a = np.genfromtxt(output / result['scenario'] / 'metrics.csv', delimiter=',', names=True)
        ax, error = axes[row]
        for prefix, label, color in [('truth', 'Ground truth', 'green'), ('odom', 'Wheel odometry', 'darkorange'),
                                     ('localized', 'NDT + wheel odometry', 'royalblue')]:
            ax.plot(a[prefix+'_x'], a[prefix+'_y'], label=label, color=color)
        ax.set(title=result['scenario'], xlabel='x (m)', ylabel='y (m)', aspect='equal')
        ax.grid(True)
        ax.legend()
        error.plot(a['t'], a['odom_error_m'], label='Wheel odometry', color='darkorange')
        error.plot(a['t'], a['localized_error_m'], label='NDT + wheel odometry', color='royalblue')
        current_error = np.hypot(a['localized_x']-a['truth_x'], a['localized_y']-a['truth_y'])
        error.plot(a['t'], current_error, '--', color='gray', alpha=.7,
                   label='NDT current-time error (includes stale TF)')
        error.set(xlabel='Simulation time (s)', ylabel='Position error (m)')
        error.grid(True)
        error.legend()
    fig.suptitle('Option 21: synthetic room / real driver / real NDT (not hardware calibration)')
    fig.tight_layout()
    fig.savefig(output / 'comparison.png', dpi=160)
    plt.close(fig)


def augment_results(output, result):
    """Also assess stale poses against NOW, not just their old measurement time."""
    directory = output / result['scenario']
    a = np.genfromtxt(directory / 'metrics.csv', delimiter=',', names=True)
    after_bootstrap = a['t'] > 4.
    valid = after_bootstrap & np.isfinite(a['localized_x'])
    error = np.hypot(a['localized_x']-a['truth_x'], a['localized_y']-a['truth_y'])
    result.update(
        localized_current_rmse_m=float(np.sqrt(np.mean(error[valid]**2))) if valid.any() else None,
        localized_current_max_error_m=float(np.max(error[valid])) if valid.any() else None,
        localized_fresh_fraction=float(np.mean(
            np.isfinite(a['localized_age_s'][after_bootstrap]) &
            (a['localized_age_s'][after_bootstrap] <= .3))),
        max_odom_age_s=float(np.nanmax(a['odom_age_s'])),
        max_localized_age_s=float(np.nanmax(a['localized_age_s'])),
        first_localization_s=float(a['t'][np.isfinite(a['localized_x'])][0]) if valid.any() else None,
    )
    result['acceptance_checks']['current_position_rmse_below_20cm'] = (
        result['localized_current_rmse_m'] is not None and result['localized_current_rmse_m'] < .20)
    result['passed'] = all(result['acceptance_checks'].values())
    (directory / 'results.json').write_text(json.dumps(result, indent=2)+'\n')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--scenario', choices=['ideal', 'bias', 'dropout', 'all'], default='all')
    parser.add_argument('--duration', type=float, default=90.)
    parser.add_argument('--rate', type=float, default=2.)
    parser.add_argument('--domain', type=int, default=167)
    parser.add_argument('--analyze-only', action='store_true', help='Recompute summaries from existing CSV')
    args = parser.parse_args()
    if args.rate <= 0 or args.duration < 5 or not 0 <= args.domain <= 232:
        parser.error('Require positive rate, duration >= 5 and ROS domain 0..232')
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    scenarios = ['ideal', 'bias', 'dropout'] if args.scenario == 'all' else [args.scenario]
    results = []
    for scenario in scenarios:
        result = (json.loads((args.output / scenario / 'results.json').read_text())
                  if args.analyze_only else run_scenario(args, scenario))
        results.append(augment_results(args.output, result))
        print(json.dumps(results[-1]), flush=True)
    (args.output / 'results.json').write_text(json.dumps(results, indent=2)+'\n')
    write_playback_qos(args.output)
    plot_results(args.output, results)
    print(f'Results and RViz bags: {args.output}', flush=True)


def write_playback_qos(output):
    (output / 'playback_qos.yaml').write_text(yaml.safe_dump({
        name: {'reliability': 'reliable', 'durability': 'transient_local', 'history': 'keep_last', 'depth': 1}
        for name in ['/tf_static', '/simulation/map', '/simulation/truth_path',
                     '/simulation/wheel_path', '/simulation/localized_path', '/simulation/goals']}))


if __name__ == '__main__':
    main()
