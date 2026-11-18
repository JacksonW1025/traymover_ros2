#!/usr/bin/env python3
"""Run real option-21 Nav2/NDT and chassis driver against a PTY-based robot.

Only Nav2 controls the plant. Ground truth supplies synthetic sensor readings
and evaluation, never a localization TF or an Odometry message.
"""
import argparse
import collections
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import pty
import shutil
import signal
import subprocess
import threading
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

import option21_simulation as sim
import option21_navigation_scene as scene


def raw_scan(points, stamp):
    """Convert base_link points to the production URDF's laser coordinates."""
    return sim.cloud(points - np.array([.435, 0., 1.365]), stamp, 'laser')


def test_goals(obstacles=False):
    if obstacles:
        return [(6., -4.4, 0.)]
    return [(2., 0., 0.), (2., 1.5, math.pi / 2)]


def acceptance(counts, goals, errors, ages, expected_goals=2):
    checks = {
        'nav2_goals_succeeded': len(goals) == expected_goals and all(
            g['status'] == GoalStatus.STATUS_SUCCEEDED for g in goals),
        'planner_produced_path': counts['plan'] > 0,
        'controller_produced_velocity': counts['nonzero_cmd'] > 0,
        'driver_produced_odometry': counts['odom'] > 20,
        'preprocessor_produced_cloud': counts['cloud'] > 20,
        'ndt_produced_pose': counts['ndt'] > 2,
        'true_goal_errors_below_30cm': bool(goals) and all(
            g['true_distance_m'] < .30 for g in goals),
        'localization_rmse_below_20cm': bool(errors) and float(
            np.sqrt(np.mean(np.square(errors)))) < .20,
        'localization_tf_fresh_95pct': bool(ages) and float(
            np.mean(np.asarray(ages) <= .6)) >= .95,
    }
    return checks


def stop_group(child):
    """Reap launch children even when their group leader has already failed."""
    try:
        os.killpg(child.pid, signal.SIGINT)
    except ProcessLookupError:
        pass
    try:
        child.wait(timeout=5)
    except subprocess.TimeoutExpired:
        pass
    for _ in range(20):
        try:
            os.killpg(child.pid, 0)
        except ProcessLookupError:
            break
        time.sleep(.1)
    else:
        try:
            os.killpg(child.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    child.wait(timeout=5)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--domain', type=int, default=172)
    parser.add_argument('--self-test', action='store_true')
    parser.add_argument('--keep-running', action='store_true')
    parser.add_argument('--rviz', action='store_true')
    parser.add_argument('--obstacle-demo', action='store_true',
                        help='Test a goal whose straight path crosses pillar 3')
    parser.add_argument('--duration', type=float, default=1800.)
    args = parser.parse_args()
    if not 0 <= args.domain <= 232 or args.duration <= 0:
        parser.error('Require ROS domain 0..232 and positive duration')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    for name in ('processes', 'ros', 'maps', 'parameters', 'tmp'):
        (output / name).mkdir()
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY='1',
                      ROS_LOG_DIR=str(output / 'ros'), TMPDIR=str(output / 'tmp'),
                      PATH=str(Path.home() / '.local/bin') + ':' + os.environ['PATH'])
    environment = dict(os.environ)
    world = sim.room_points()
    pcd = output / 'maps/room.pcd'
    sim.write_pcd(pcd, world)
    with (output / 'processes/map_conversion.log').open('w') as log:
        subprocess.run([
            'python3', str(sim.ROOT / 'src/traymover_robot_nav/scripts/pcd2pgm.py'),
            '--pcd', str(pcd), '--out', str(output / 'maps/room_2d'),
            '--z-reference', 'floor', '--z-min', '.17', '--z-max', '2.20',
            '--min-points', '1', '--dilate', '2', '--min-region-cells', '1'],
            env=environment, stdout=log, stderr=subprocess.STDOUT, check=True)
    map_validation = scene.repair_simulation_map(output / 'maps/room_2d')
    nav_config = scene.full_navigation_parameters(yaml.safe_load(
        (sim.ROOT / 'src/traymover_robot_nav/config/nav2_params.yaml').read_text()))
    nav_parameters = output / 'parameters/full_navigation.yaml'
    nav_parameters.write_text(yaml.safe_dump(nav_config))
    rviz = yaml.safe_load((sim.ROOT / 'src/traymover_robot_nav/rviz/traymover_nav.rviz').read_text())
    rviz['Panels'] = [p for p in rviz['Panels'] if p['Class'] != 'traymover_robot_safety/EstopPanel']
    rviz['Panels'].append({'Class': 'nav2_rviz_plugins/Navigation 2', 'Name': 'Navigation 2'})
    manager = rviz['Visualization Manager']
    manager['Views']['Current'].update(Distance=23., Pitch=1.45)
    manager['Views']['Current']['Focal Point'] = {'X': 0., 'Y': 0., 'Z': 0.}
    manager['Global Options']['Frame Rate'] = 20
    scene.decorate_rviz(manager)
    manager['Displays'].append({
        'Class': 'rviz_default_plugins/Pose', 'Name': 'Simulation Ground Truth (green)',
        'Enabled': True, 'Topic': {'Value': '/simulation/truth_pose'},
        'Color': '30; 240; 70', 'Shape': 'Arrow', 'Shaft Length': .5,
        'Head Length': .2, 'Shaft Radius': .05, 'Head Radius': .12})
    rviz_path = output / 'navigation.rviz'
    rviz_path.write_text(yaml.safe_dump(rviz, sort_keys=False))
    rclpy.init(domain_id=args.domain)
    master, slave = pty.openpty()
    mcu = sim.VirtualMCU(master, 'ideal', output / 'serial.jsonl')
    profile = yaml.safe_load((sim.ROOT / 'src/turn_on_traymover_robot/config/stm32_navigation.yaml').read_text())
    parameters = profile['turn_on_traymover_robot']['ros__parameters']
    # Hard override: the simulator can only open this newly allocated PTY.
    parameters.update(usart_port_name=os.ttyname(slave), use_sim_time=True,
                      status_poll_rate_hz=0.)
    chassis = sim.driver.TurnOnTraymoverRobot(parameter_overrides=[
        Parameter(key, value=value) for key, value in parameters.items()])
    node = rclpy.create_node('option21_full_chain', parameter_overrides=[
        Parameter('use_sim_time', value=True)])
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(chassis)
    def spin():
        try:
            executor.spin()
        except ExternalShutdownException:
            pass
    thread = threading.Thread(target=spin, daemon=True)
    thread.start()
    bag = sim.Bag(output / 'observations')
    counts = collections.Counter()
    elapsed, sim_time = 0., 10.
    stopping = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stopping.set())
    signal.signal(signal.SIGTERM, lambda *_: stopping.set())
    def record(topic, msg):
        bag.add(topic, msg, sim_time)
    def observe(name, topic, kind, save=True, qos=100):
        def callback(msg):
            counts[name] += 1
            if name == 'cmd' and (abs(msg.linear.x) > .001 or abs(msg.angular.z) > .001):
                counts['nonzero_cmd'] += 1
            if save:
                record(topic, msg)
        node.create_subscription(kind, topic, callback, qos)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for name, topic, kind in (
            ('odom', '/odom', Odometry), ('ndt', '/pcl_pose', PoseWithCovarianceStamped),
            ('cmd', '/cmd_vel', Twist), ('plan', '/plan', NavPath),
            ('local_plan', '/local_plan', NavPath), ('tf', '/tf', TFMessage),
            ('initialpose', '/initialpose', PoseWithCovarianceStamped),
            ('goal_pose', '/goal_pose', PoseStamped),
            ('action_status', '/navigate_to_pose/_action/status', GoalStatusArray)):
        observe(name, topic, kind)
    observe('tf_static', '/tf_static', TFMessage, qos=latched)
    observe('cloud', '/point_cloud_localization', PointCloud2, save=False, qos=5)
    observe('aligned_cloud', '/ndt_aligned_scan', PointCloud2, save=False, qos=latched)
    observe('map_cloud', '/initial_map', PointCloud2, save=False, qos=latched)
    observe('global_costmap', '/global_costmap/costmap', OccupancyGrid, save=False, qos=latched)
    observe('local_costmap', '/local_costmap/costmap', OccupancyGrid, save=False, qos=latched)
    first_plan = []
    def capture_plan(msg):
        if not first_plan and len(msg.poses) > 1:
            first_plan.extend([[p.pose.position.x, p.pose.position.y] for p in msg.poses])
            (output / 'first_global_plan.json').write_text(json.dumps(first_plan))
    node.create_subscription(NavPath, '/plan', capture_plan, 10)
    clock_pub = node.create_publisher(Clock, '/clock', 10)
    raw_pub = node.create_publisher(PointCloud2, '/point_cloud_raw', 5)
    truth_pub = node.create_publisher(PoseStamped, '/simulation/truth_pose', 5)
    initial_pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    obstacles_pub = node.create_publisher(MarkerArray, '/simulation/obstacles', latched)
    trail_pub = node.create_publisher(NavPath, '/simulation/truth_path', 1)
    trail = NavPath()
    trail.header.frame_id = 'map'
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node)
    client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    process_log = (output / 'processes/navigation.log').open('w')
    command = [
        'ros2', 'launch', 'traymover_robot_nav', 'traymover_nav_stm32.launch.py',
        'use_sim_time:=true', 'bringup_hardware:=false', 'launch_rviz:=false',
        f'pcd_path:={pcd}', f'map:={output / "maps/room_2d.yaml"}',
        f'params_file:={nav_parameters}',
        'bt_xml:=/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml',
        'through_poses_bt_xml:=/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml']
    process = subprocess.Popen(command, env=environment, stdout=process_log,
                               stderr=subprocess.STDOUT, start_new_session=True)
    children = [(process, process_log)]
    for name, cmd in (
            ('behaviors', ['ros2', 'run', 'nav2_behaviors', 'behavior_server',
                           '--ros-args', '--params-file', str(nav_parameters)]),
            ('behavior_lifecycle', ['ros2', 'run', 'nav2_lifecycle_manager', 'lifecycle_manager',
                                   '--ros-args', '-r', '__node:=simulation_behavior_manager',
                                   '-p', 'use_sim_time:=true', '-p', 'autostart:=true',
                                   '-p', 'node_names:=[behavior_server]'])):
        log = (output / f'processes/{name}.log').open('w')
        children.append((subprocess.Popen(cmd, env=environment, stdout=log,
                        stderr=subprocess.STDOUT, start_new_session=True), log))
    for name, script, folder in (
            ('ndt', 'ndt_diagnostics.py', 'ndt'),
            ('odometry', 'fastlio_diagnostics.py', 'odometry')):
        log = (output / f'processes/{name}_diagnostics.log').open('w')
        cmd = ['python3', str(sim.ROOT / 'scripts' / script), '--output-dir',
               str(output / folder), '--use-sim-time', 'true', '--interval-sec', '5']
        if name == 'odometry':
            cmd += ['--odometry-source', 'stm32']
        children.append((subprocess.Popen(cmd, env=environment, stdout=log,
                        stderr=subprocess.STDOUT, start_new_session=True), log))
    rviz_process = None
    if args.rviz:
        log = (output / 'processes/rviz.log').open('w')
        rviz_process = subprocess.Popen([
            'rviz2', '-d', str(rviz_path), '--ros-args', '-p', 'use_sim_time:=true'],
            env=environment, stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True)
        children.append((rviz_process, log))
    manifest = {'mode': 'full option21 Nav2 closed loop with virtual STM32',
                'ros_domain_id': args.domain, 'hardware_port': os.ttyname(slave),
                'navigation_command': command, 'map_points': len(world),
                'self_test': args.self_test, 'model_dt_s': .05,
                'map_validation': map_validation, 'controller': 'DWBLocalPlanner',
                'local_and_global_obstacle_layers': True, 'recovery_behaviors': True,
                'lidar_hz': 10, 'feedback_velocity_gain': 1.,
                'ground_truth_not_supplied_to_localizer': True,
                'raw_cloud_recorded': False, 'output': str(output),
                'limitations': ['Synthetic ideal wheel feedback; no physical unit calibration',
                                'No physical slip, actuator delay identification or hardware safety test']}
    manifest['source_sha256'] = {str(path.relative_to(sim.ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in [Path(__file__).resolve(),
                     sim.ROOT / 'scripts/option21_navigation_scene.py',
                     sim.ROOT / 'scripts/option21_simulation.py',
                     sim.ROOT / 'src/turn_on_traymover_robot/turn_on_traymover_robot/traymover_robot.py',
                     sim.ROOT / 'src/traymover_robot_nav/config/nav2_params.yaml',
                     sim.ROOT / 'src/traymover_robot_nav/config/localization.yaml']}
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2))
    metrics = (output / 'metrics.csv').open('w', newline='')
    writer = csv.writer(metrics)
    writer.writerow(['t', 'truth_x', 'truth_y', 'truth_yaw', 'localized_x',
                     'localized_y', 'localized_yaw', 'current_error_m', 'tf_age_s'])
    truth, velocity = np.zeros(3), np.zeros(2)
    rng = np.random.default_rng(20261117)
    initialized = finished = False
    response = result = None
    goals, errors, ages = [], [], []
    snapshot = {}
    ready_for_goal_at = 0.
    route = test_goals(args.obstacle_demo)
    min_clearance, collision_blocks = float('inf'), 0
    try:
        for step in range(round(args.duration / .05)):
            if stopping.is_set():
                break
            if rviz_process is not None and rviz_process.poll() == 0:
                break
            started = time.monotonic()
            elapsed, sim_time = step * .05, 10. + step * .05
            stamp = rclpy.time.Time(seconds=sim_time).to_msg()
            with mcu.lock:
                command_velocity = np.array(mcu.command)
            velocity += .35 * (command_velocity - velocity)
            candidate = sim.plant_step(truth, *velocity, .05)
            if scene.collides(candidate):
                velocity[:] = 0.
                collision_blocks += 1
            else:
                truth = candidate
            min_clearance = min(min_clearance, scene.clearance(truth))
            with mcu.lock:
                mcu.velocity, mcu.elapsed = tuple(velocity), elapsed
            clock_pub.publish(Clock(clock=stamp))
            if step % 2 == 0:
                raw_pub.publish(raw_scan(sim.make_scan(world, truth, rng), stamp))
                pose = PoseStamped()
                pose.header.frame_id, pose.header.stamp = 'map', stamp
                pose.pose.position.x, pose.pose.position.y = map(float, truth[:2])
                pose.pose.position.z = .2
                pose.pose.orientation.z, pose.pose.orientation.w = math.sin(truth[2]/2), math.cos(truth[2]/2)
                truth_pub.publish(pose)
                record('/simulation/truth_pose', pose)
                trail.header.stamp = stamp
                trail.poses.append(pose)
                if len(trail.poses) > 3000:
                    trail.poses.pop(0)
                if step % 10 == 0:
                    trail_pub.publish(trail)
                loc, age, error = np.full(3, np.nan), float('nan'), float('nan')
                try:
                    tf = buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                    p, q = tf.transform.translation, tf.transform.rotation
                    loc = np.array([p.x, p.y, sim.yaw_of(q)])
                    age = sim_time - tf.header.stamp.sec - tf.header.stamp.nanosec/1e9
                    error = float(np.linalg.norm(loc[:2] - truth[:2]))
                    if elapsed > 8 and not finished:
                        errors.append(error)
                        ages.append(age)
                except TransformException:
                    pass
                writer.writerow([elapsed, *truth, *loc, error, age])
            if args.self_test and not initialized and elapsed > 5 and counts['map_cloud']:
                initial = PoseWithCovarianceStamped()
                initial.header.frame_id, initial.header.stamp = 'map', stamp
                initial.pose.pose.position.x, initial.pose.pose.position.y = .3, -.2
                initial.pose.pose.position.z = .2
                initial.pose.pose.orientation.z, initial.pose.pose.orientation.w = math.sin(.05), math.cos(.05)
                initial.pose.covariance[0] = initial.pose.covariance[7] = .25
                initial.pose.covariance[35] = .1
                initial_pub.publish(initial)
                initialized = True
            if args.self_test and not finished and counts['ndt'] and client.server_is_ready():
                if response is None and result is None and elapsed >= ready_for_goal_at:
                    x, y, yaw = route[len(goals)]
                    goal = NavigateToPose.Goal()
                    goal.pose.header.frame_id, goal.pose.header.stamp = 'map', stamp
                    goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
                    goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
                    response = client.send_goal_async(goal)
                    print(f'NAV2_SEND goal={len(goals)+1} pose={(x, y, yaw)}', flush=True)
                if response is not None and response.done():
                    handle = response.result()
                    response = None
                    if handle.accepted:
                        result = handle.get_result_async()
                    else:
                        ready_for_goal_at = elapsed + 2
                if result is not None and result.done():
                    target = route[len(goals)]
                    goal_result = {'goal': target, 'status': result.result().status,
                                   't': elapsed, 'true_pose': truth.tolist(),
                                   'true_distance_m': float(np.linalg.norm(truth[:2] - target[:2]))}
                    goals.append(goal_result)
                    result = None
                    ready_for_goal_at = elapsed + 2
                    print('NAV2_RESULT ' + json.dumps(goal_result), flush=True)
                    if goal_result['status'] != GoalStatus.STATUS_SUCCEEDED or len(goals) == len(route):
                        finished = True
                        checks = acceptance(counts, goals, errors, ages, len(route))
                        plan_result = scene.path_metrics(first_plan)
                        checks.update(local_planner_produced_trajectory=counts['local_plan'] > 0,
                                      both_costmaps_published=counts['global_costmap'] > 0 and counts['local_costmap'] > 0,
                                      physical_model_no_collision=collision_blocks == 0)
                        if args.obstacle_demo:
                            checks['plan_detours_around_pillar'] = (
                                bool(plan_result) and not plan_result['enters_solid_pillar']
                                and plan_result['max_deviation_from_straight_m'] > .45)
                        snapshot = {'passed': all(checks.values()), 'checks': checks,
                                    'goals': goals, 'counts': dict(counts),
                                    'first_global_plan': plan_result,
                                    'minimum_true_footprint_clearance_m': min_clearance,
                                    'physical_collision_blocks': collision_blocks,
                                    'localization_current_rmse_m': float(np.sqrt(np.mean(np.square(errors)))) if errors else None,
                                    'max_tf_age_s': max(ages) if ages else None}
                        (output / 'self_test_results.json').write_text(json.dumps(snapshot, indent=2))
                        print('SELF_TEST ' + json.dumps(snapshot), flush=True)
                        if not args.keep_running:
                            break
            if step % 100 == 0:
                obstacles_pub.publish(scene.markers(stamp))
                metrics.flush()
                mcu.output.flush()
                state = {'t': elapsed, 'truth': truth.tolist(), 'counts': dict(counts),
                         'self_test_finished': finished, 'self_test': snapshot,
                         'minimum_true_footprint_clearance_m': min_clearance,
                         'physical_collision_blocks': collision_blocks,
                         'output_free_gb': shutil.disk_usage(output).free / 1e9,
                         'cloud_publishers': len(node.get_publishers_info_by_topic('/point_cloud_localization'))}
                (output / 'live_status.json').write_text(json.dumps(state, indent=2))
                print('LIVE ' + json.dumps(state), flush=True)
                if state['cloud_publishers'] > 1:
                    raise RuntimeError('Duplicate localization cloud publishers; use a clean ROS domain')
                if any(p.poll() is not None for p, _ in children):
                    raise RuntimeError('An experiment process exited; inspect processes/')
                if shutil.disk_usage(output).free < 1_000_000_000:
                    raise RuntimeError('Output disk free space below 1 GB')
            time.sleep(max(0., .05 - (time.monotonic() - started)))
    finally:
        for child, log in reversed(children):
            stop_group(child)
            log.close()
        executor.shutdown(timeout_sec=2)
        thread.join(timeout=2)
        client.destroy()
        chassis.destroy_node()
        node.destroy_node()
        mcu.close()
        os.close(master)
        os.close(slave)
        metrics.close()
        bag.close()
        rclpy.try_shutdown()
    if args.self_test and not snapshot:
        (output / 'self_test_results.json').write_text(json.dumps({
            'passed': False, 'reason': 'Interrupted or timed out before all goals finished',
            'goals': goals, 'counts': dict(counts)}, indent=2))
    if args.self_test and not snapshot.get('passed', False):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
