"""Exercise the option 22 menu, policy, and closed-loop Nav2 with synthetic sensors."""
import importlib.util
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from test_nav_simple_config import _run_option10_prompt


PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG / 'scripts'))
from option22_obstacle_gate import (
    CONFIG, ClusterFilter, Confirmation, Decision, DetourTurn,
    MapEnvelope, ObstacleGateNode, Sample, ZERO, approach_speed_limit, clearances,
    detour_trigger, front_returns, motion_is_clear, ramp_command,
    robot_returns, xyz_cloud,
)
from pcd2pgm import FREE_VAL, OCC_VAL, UNKNOWN_VAL, main as convert_pcd, observed_ground


def test_menu_starts_one_guarded_stm32_stack(tmp_path):
    result, _, hardware = _run_option10_prompt(tmp_path, '1\nn\nn\n\n', option=22)
    assert result.returncode == 0, result.stdout + result.stderr
    assert hardware
    commands = (tmp_path / 'hardware').read_text()
    assert commands.count('ros2 launch') == 1
    assert 'option22.launch.py' in commands and 'bringup_hardware:=true' in commands
    assert 'TRAYMOVER_ODOMETRY_SOURCE=stm32' in commands
    assert 'example_option22_2d.yaml' in commands
    assert '--observed-ground-only' in (tmp_path / 'arguments').read_text()
    assert '22) action_start_nav 22 ;;' in (PKG.parents[1] / 'scripts/traymover.sh').read_text()


@pytest.mark.parametrize('bringup_hardware', [True, False])
def test_chassis_and_controller_have_separate_scoped_transports(monkeypatch, bringup_hardware):
    import ament_index_python.packages
    from launch import LaunchDescription, LaunchService
    from launch.actions import IncludeLaunchDescription, SetLaunchConfiguration
    from launch.utilities import perform_substitutions
    from launch_ros.actions import Node

    monkeypatch.setattr(ament_index_python.packages, 'get_package_share_directory',
                        lambda name: str(PKG.parent / name))
    spec = importlib.util.spec_from_file_location('option22_launch_graph',
                                                 PKG / 'launch/option22.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    original = IncludeLaunchDescription.execute
    included = {'base_serial.launch.py', 'traymover_auto_estop.launch.py',
                'traymover_nav_stm32.launch.py', 'traymover_nav.launch.py'}

    def select_includes(action, context):
        path = action.launch_description_source._LaunchDescriptionSource__location
        if Path(perform_substitutions(context, path)).name in included:
            return original(action, context)
        return []

    remaps = {}
    estop_arguments = {}

    def capture_node(action, context):
        action._perform_substitutions(context)
        remaps[action.node_name] = dict(action.expanded_remapping_rules or [])
        if action.node_name == '/traymover_auto_estop':
            estop_arguments.update({key: context.launch_configurations[key] for key in (
                'start_realsense_driver', 'depth_topic', 'stop_distance_m', 'release_distance_m')})
        return []

    monkeypatch.setattr(IncludeLaunchDescription, 'execute', select_includes)
    monkeypatch.setattr(Node, 'execute', capture_node)
    service = LaunchService()
    service.include_launch_description(LaunchDescription([
        SetLaunchConfiguration('ros_namespace', '/'),
        SetLaunchConfiguration('pcd_path', '/tmp/simulated.pcd'),
        SetLaunchConfiguration('bringup_hardware', str(bringup_hardware).lower()),
        SetLaunchConfiguration('map', '/tmp/simulated.yaml'),
        SetLaunchConfiguration('launch_rviz', 'true'),
        SetLaunchConfiguration('start_realsense_driver', 'false'),
        module.generate_launch_description(),
    ]))
    assert service.run() == 0
    if bringup_hardware:
        assert remaps['/turn_on_traymover_robot'] == {
            '/cmd_vel': '/option22/cmd_vel_safe',
            '/traymover_estop/auto_request': '/option22/auto_request',
        }
        assert remaps['/rviz2'] == {}
    else:
        assert '/turn_on_traymover_robot' not in remaps
        assert remaps['/rviz2'] == {
            '/traymover_estop/state': '/option22/display_estop',
            '/traymover_estop/auto_state': '/option22/display_auto_estop',
        }
    assert remaps['/controller_server'] == {'/cmd_vel': '/option22/cmd_vel_nav'}
    assert remaps['/traymover_auto_estop'] == {
        '/traymover_estop/auto_request': '/option22/auto_request',
    }
    assert estop_arguments == {
        'start_realsense_driver': 'false', 'depth_topic': '/camera/camera/depth/image_rect_raw',
        'stop_distance_m': '0.20', 'release_distance_m': '0.25',
    }
    assert remaps['/option22_obstacle_gate'] == {}
    profile = module.rviz_profile(PKG)
    displays = {item['Name']: item for item in profile['Visualization Manager']['Displays']}
    assert displays['GlobalPlan']['Topic']['Value'] == '/option22/display_plan'
    assert displays['Navigation status']['Topic']['Value'] == '/option22/navigation_marker'
    assert not displays['LocalPlan']['Enabled']

    config = yaml.safe_load((PKG / 'config' / 'option22_obstacles.yaml').read_text())
    assert config['gate'] == CONFIG
    follow = module.navigation_parameters(PKG, **config)['controller_server']['ros__parameters'][
        'FollowPath']
    assert follow['use_velocity_scaled_lookahead_dist'] and follow['use_interpolation']
    assert (follow['min_lookahead_dist'], follow['max_lookahead_dist']) == (.45, .65)
    assert follow['lookahead_time'] == 1.85
    assert follow['use_rotate_to_heading'] and follow['rotate_to_heading_min_angle'] == .65
    assert follow['desired_linear_vel'] == .50
    assert follow['rotate_to_heading_angular_vel'] == .20
    assert follow['use_collision_detection']


def test_spatial_support_and_successive_observations_filter_near_lidar_speckles():
    spatial_filter = ClusterFilter(.035, .14, 4, .10)
    cluster = np.array([[0.57, y, 0.25] for y in [-0.04, 0.0, 0.04, 0.08]])
    speckles = np.repeat([[0.8, -0.2, 0.4]], 20, axis=0)
    compact = np.array([[x, y, .40] for x in (.659, .671) for y in (-.006, .006)])
    assert len(spatial_filter.apply(compact)) == 0
    filtered = spatial_filter.apply(np.vstack((cluster, speckles, compact + [.4, 0., 0.])))
    np.testing.assert_array_equal(filtered, cluster)
    tracking = Confirmation(spatial_filter, .15, .30)
    assert len(tracking.update(1.0, filtered)) == 0
    assert len(tracking.update(1.1, filtered + [.04, 0, 0])) == len(cluster)
    assert len(tracking.update(1.2, np.empty((0, 3)))) == 0
    assert len(tracking.snapshot(1.29)) == len(cluster)
    assert len(tracking.snapshot(1.41)) == 0
    assert len(tracking.update(1.8, filtered)) == 0


def test_temporal_confirmation_requires_support_in_overlap():
    spatial_filter = ClusterFilter(.035, .14, 4, .10)
    cluster = np.array([[.60, y, .35] for y in (-.04, 0., .04, .08)])
    tracking = Confirmation(spatial_filter, .15, .30)
    assert len(tracking.update(1.0, cluster)) == 0
    # Each frame contains a substantial cluster, but the overlap has one hit.
    shifted = cluster + [0., .24, 0.]
    assert len(tracking.update(1.1, shifted)) == 0
    assert len(tracking.update(1.2, shifted, usable=False)) == len(cluster)
    assert len(tracking.snapshot(1.2)) == 0
    assert len(tracking.update(1.3, shifted)) == len(cluster)


def test_self_filter_masks_deck_and_mounts_while_retaining_nearby_objects():
    points = np.array([[.42, .27, .34], [.50, -.11, 1.40], [.40, 0., .66],
                       [.50, -.11, .55], [.54, 0., 1.40], [.0, 0., .65]])
    assert robot_returns(points).tolist() == [True, True, True, False, False, False]


@pytest.mark.parametrize('camera_request, state', [
    (None, 'AUTO_ESTOP_UNAVAILABLE'),
    ((False, 3.0), 'AUTO_ESTOP_UNAVAILABLE'),
    ((True, 4.0), 'CAMERA_ESTOP'),
])
def test_independent_estop_takes_priority_over_navigation_inputs(camera_request, state):
    node = SimpleNamespace(camera_estop=camera_request, close_obstacle=True)
    assert ObstacleGateNode.decision(node, 4.1, 4.1) == Decision(state, estop=True)


def test_detour_requires_forward_lidar_cone_and_80_cm_clearance():
    points = np.array([[1.29, 0., .40], [1.31, 0., .40], [.68, .11, .40],
                       [.68, .09, .40], [.45, .32, .40], [-.70, 0., .40]])
    front = front_returns(points, 45.)
    distance = clearances(points)
    assert detour_trigger(front, distance, .80).tolist() == [
        True, False, False, True, False, False]
    assert detour_trigger(front_returns(points), distance, .80)[2]


def test_rviz_estop_clears_old_route_and_waits_for_a_new_plan():
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path as NavPath

    updates = {}

    def publisher(name):
        updates[name] = []
        return SimpleNamespace(publish=updates[name].append)

    plan = NavPath(poses=[PoseStamped()])
    node = SimpleNamespace(
        chassis_auto_estop=False, chassis_estop=False, manual=(False, 4.0),
        displayed_status=None, displayed_plan=None, close_obstacle=False, planner=None,
        detour_heading=None, plan_hidden_since=0.0, latest_plan=Sample(4., 4., plan),
        display_estop_pub=publisher('stop'), display_auto_pub=publisher('auto'),
        display_plan_pub=publisher('path'), marker_pub=publisher('marker'))
    stamp = Time(sec=4)
    ObstacleGateNode.update_display(node, stamp, 4.1, 4.1, Decision('FOLLOWING_ROUTE'))
    assert updates['path'][-1].poses
    ObstacleGateNode.update_display(node, stamp, 4.2, 4.2, Decision('CAMERA_ESTOP', estop=True))
    marker = updates['marker'][-1]
    assert updates['stop'][-1].data and updates['auto'][-1].data
    assert marker.text == 'E-STOP' and marker.color.r == 1.0 and marker.color.g < .2
    assert not updates['path'][-1].poses
    ObstacleGateNode.update_display(node, stamp, 4.3, 4.3, Decision('FOLLOWING_ROUTE'))
    assert not updates['stop'][-1].data and not updates['auto'][-1].data
    assert not updates['path'][-1].poses
    node.latest_plan = Sample(4.35, 4.35, plan)
    ObstacleGateNode.update_display(node, stamp, 4.4, 4.4, Decision('FOLLOWING_ROUTE'))
    assert updates['path'][-1].poses
    ObstacleGateNode.update_display(node, stamp, 4.5, 4.5, Decision('NO_LOCAL_DETOUR'))
    assert not updates['stop'][-1].data and not updates['path'][-1].poses
    assert updates['marker'][-1].text == 'NO CLEAR DETOUR'


def test_detour_stops_then_turns_without_linear_motion():
    turn = DetourTurn()
    assert turn.decision(1., -.55, (.16, 0.), .25).velocity == ZERO
    decision = turn.decision(1., -.55, ZERO, .25)
    assert decision.state == 'DETOUR_TURNING' and decision.velocity == (0., -.25)
    assert turn.decision(1., -.12, (0., -.20), .25).velocity == ZERO
    assert turn.decision(1., -.12, ZERO, .25) is None
    assert turn.decision(2., .45, ZERO, .25).velocity == (0., .25)


def test_command_ramp_tracks_curve_and_bounds_speed_changes():
    target = (.35, .25)
    first = ramp_command(ZERO, target, .05, .55, 1.0)
    assert first == pytest.approx((.0275, .0275 * target[1] / target[0]))

    reversed_turn = ramp_command(target, (.16, -.25), .05, .55, 1.0)
    assert reversed_turn == pytest.approx((.331, .20))
    assert ramp_command((.35, -.21), (.34, -.25), .05, .55, 1.0) == (.34, -.25)


def test_command_ramp_stops_immediately_and_limits_the_first_step_after_a_gap():
    assert ramp_command((.35, .25), ZERO, .05, .55, 1.0) == ZERO
    assert ramp_command((.35, .25), (.35, 0.), .35, .55, 1.0) == pytest.approx((.0275, 0.))
    assert ramp_command(ZERO, (0., -.25), .15, .55, 1.0) == pytest.approx((0., -.05))


def test_approach_speed_transitions_to_the_existing_near_obstacle_limit():
    assert approach_speed_limit(float('inf'), CONFIG) == pytest.approx(.50)
    assert approach_speed_limit(1.20, CONFIG) == pytest.approx(.50)
    assert approach_speed_limit(1.075, CONFIG) == pytest.approx(.33)
    assert approach_speed_limit(.95, CONFIG) == pytest.approx(.16)
    assert approach_speed_limit(.80, CONFIG) == pytest.approx(.16)


def test_swept_braking_checks_turns_and_allows_moving_away_from_soft_margin():
    point = np.array([[.39, .39, .30]])
    assert not motion_is_clear(point, (0., .25), .08)
    assert motion_is_clear(point, (0., -.25), .08)
    assert not motion_is_clear(np.array([[.83, 0., .30]]), (.35, 0.), .08)


def test_ground_support_leaves_unobserved_gaps_and_isolated_points_unknown():
    ground = np.array([[x, y, 0.] for x in np.arange(.3, .8, .05)
                       for y in np.arange(.3, .8, .05)] +
                      [[x, y, 0.] for x in np.arange(2., 2.5, .05)
                       for y in np.arange(.3, .8, .05)] + [[1.4, .5, 0.]])
    mask = observed_ground(ground[:, :2], (40, 60), [0., 0.], .05)
    assert mask[10, 10] and mask[10, 45]
    assert not mask[10, 28] and not mask[39, 10]


def test_option22_converter_uses_nonzero_floor_even_without_occupied_returns(tmp_path, monkeypatch):
    import open3d as o3d
    from PIL import Image

    points = np.array([[x, y, -0.35] for x in np.arange(0., 1., .08)
                       for y in np.arange(0., 1., .08)])
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    pcd, output = tmp_path / 'floor.pcd', tmp_path / 'map'
    assert o3d.io.write_point_cloud(str(pcd), cloud)
    monkeypatch.setattr(sys, 'argv', [
        'pcd2pgm.py', '--pcd', str(pcd), '--out', str(output), '--observed-ground-only',
    ])
    convert_pcd()
    grid = np.asarray(Image.open(output.with_suffix('.pgm')))
    meta = yaml.safe_load(output.with_suffix('.yaml').read_text())
    assert grid[grid.shape[0] // 2, grid.shape[1] // 2] == FREE_VAL
    assert grid[0, 0] == UNKNOWN_VAL
    assert meta['free_thresh'] < 1 - UNKNOWN_VAL / 255 < meta['occupied_thresh']


def test_map_envelope_checks_unknown_cells_and_outside_map():
    from nav_msgs.msg import OccupancyGrid

    grid = OccupancyGrid()
    grid.info.resolution, grid.info.width, grid.info.height = .05, 40, 40
    grid.info.origin.orientation.w = 1.
    cells = np.zeros((40, 40), dtype=np.int8)
    cells[:, 30:] = -1
    cells[:, 0] = 100
    grid.data = cells.ravel().tolist()
    envelope = MapEnvelope(grid, wall_match=.14)
    assert envelope.matches_wall(np.array([[.10, 1.], [1.6, 1.], [-.02, 1.]])).tolist() == [
        True, False, False]
    points = envelope.nearby(np.array([.9, 1.]), np.eye(2))
    assert not motion_is_clear(points, (.35, 0.), .036)
    points = envelope.nearby(np.array([1., .30]), np.array([[0., 1.], [-1., 0.]]))
    assert not motion_is_clear(points, (.35, 0.), .036)


def _start_navigation(map_path):
    """Run the production launch with recorded/synthetic odom and localization inputs."""
    from launch import LaunchDescription, LaunchService
    from launch.actions import IncludeLaunchDescription, SetLaunchConfiguration
    from launch.utilities import perform_substitutions

    original = IncludeLaunchDescription.execute

    def with_simulated_inputs(action, context):
        path = action.launch_description_source._LaunchDescriptionSource__location
        name = Path(perform_substitutions(context, path)).name
        if name in ('lidar_localization.launch.py', 'navigation_pointcloud.launch.py'):
            return []
        return original(action, context)

    IncludeLaunchDescription.execute = with_simulated_inputs
    spec = importlib.util.spec_from_file_location('option22_launch', PKG / 'launch/option22.launch.py')
    launch = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch)
    service = LaunchService()
    service.include_launch_description(LaunchDescription([
        SetLaunchConfiguration('pcd_path', '/tmp/simulated.pcd'),
        SetLaunchConfiguration('map', map_path),
        SetLaunchConfiguration('start_realsense_driver', 'false'),
        launch.generate_launch_description(),
    ]))
    return service.run()


def _ranges(x, y, angles, obstacles, front_wall=55.8, half_width=1.6):
    """Intersect rays with the corridor and all nearby cylinders."""
    dx, dy = np.cos(angles), np.sin(angles)
    with np.errstate(divide='ignore', invalid='ignore'):
        ranges = np.minimum((np.where(dx > 0, front_wall, -5.8) - x) / dx,
                            (np.copysign(half_width, dy) - y) / dy)
    for ox, oy, radius in obstacles:
        along = (ox - x) * dx + (oy - y) * dy
        cross = (ox - x) * dy - (oy - y) * dx
        hit = (along > 0) & (np.abs(cross) < radius)
        ranges[hit] = np.minimum(ranges[hit], along[hit] - np.sqrt(radius**2 - cross[hit]**2))
    return ranges


@pytest.mark.parametrize('scene', ['close_planner', 'corridor', 'unknown_boundary', 'mapped_wall'])
def test_nav2_bounded_detour(tmp_path, monkeypatch, scene):
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import TransformStamped, Twist
    from lifecycle_msgs.msg import State
    from lifecycle_msgs.srv import GetState
    from nav2_msgs.action import ComputePathToPose, NavigateToPose
    from nav_msgs.msg import Odometry, Path as NavPath
    from rclpy.action import ActionClient
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from sensor_msgs.msg import CameraInfo, Image, PointCloud2
    from std_msgs.msg import Bool, String
    from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
    from visualization_msgs.msg import Marker

    narrow, facing_wall = scene == 'unknown_boundary', scene == 'mapped_wall'
    camera_wall = 2.05 if facing_wall else 55.8
    half_width = 1.15 if scene == 'close_planner' else 1.60
    grid = np.full((70, 1240), FREE_VAL, dtype=np.uint8)
    grid[:3, :] = grid[-3:, :] = OCC_VAL
    if scene == 'close_planner':
        grid[:12, :] = grid[-12:, :] = OCC_VAL
    if narrow:
        grid[3:24, :] = grid[46:-3, :] = UNKNOWN_VAL
    if facing_wall:
        # Static wall starts at x=2.05. LiDAR below sees a return at x=1.95.
        grid[:, 161:] = OCC_VAL
    (tmp_path / 'map.pgm').write_bytes(b'P5\n1240 70\n255\n' + grid.tobytes())
    map_path = tmp_path / 'map.yaml'
    map_path.write_text(yaml.safe_dump(dict(
        image='map.pgm', resolution=0.05, origin=[-6., -1.75, 0.],
        negate=0, occupied_thresh=0.65, free_thresh=0.25)))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    domain = 130 + os.getpid() % 20
    context = Context()
    context.init(args=[], domain_id=domain)
    node = Node('option22_simulation', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    static_tf = StaticTransformBroadcaster(node)
    dynamic_tf = TransformBroadcaster(node)

    def transform(parent, child, **translation):
        msg = TransformStamped()
        msg.header.frame_id, msg.child_frame_id = parent, child
        msg.transform.rotation.w = 1.0
        for key, value in translation.items():
            setattr(msg.transform.translation, key, value)
        return msg

    camera_tf = transform('base_footprint', 'synthetic_optical', x=.45, z=.96)
    (camera_tf.transform.rotation.x, camera_tf.transform.rotation.y,
     camera_tf.transform.rotation.z, camera_tf.transform.rotation.w) = (-.5, .5, -.5, .5)
    static_tf.sendTransform([transform('map', 'odom'), camera_tf,
                             transform('base_footprint', 'base_link', z=.20)])
    odometry = node.create_publisher(Odometry, '/odom', 2)
    lidar = node.create_publisher(PointCloud2, '/point_cloud_raw', 2)
    depth = node.create_publisher(Image, '/camera/camera/depth/image_rect_raw', 2)
    info = node.create_publisher(CameraInfo, '/camera/camera/depth/camera_info', 2)
    manual = node.create_publisher(Bool, '/traymover_estop/manual_state', 1)
    pose = [0., 0., 0.]
    command = [0., 0.]
    states, plans, requests, positions, obstacles = [], [], [], [], []
    depth_enabled = True
    occluder_depth_mm = None
    beams = np.linspace(-np.pi, np.pi, 720, endpoint=False)
    columns = np.arctan2(80. - np.arange(160), 90.)
    floor = np.full(96, np.inf)
    floor[49:] = .96 * 90. / (np.arange(49, 96) - 48)
    last_pose = time.monotonic()
    lidar_frames = 0
    noise = [[x, y, .40] for x in (.659, .671) for y in (-.006, .006)]
    noise += [[.50, y, 1.40] for y in (-.10, -.05, .03, .10)]
    noise += [[.42, y, .34] for y in (-.10, -.05, .03, .10)]
    noise += [[1.45, y, 3.95] for y in (-.09, -.04, 0., .04, .09)]

    def publish():
        nonlocal last_pose, lidar_frames
        now = time.monotonic()
        dt = min(.08, now - last_pose)
        last_pose = now
        vx, wz = command if requests and not requests[-1] else ZERO
        pose[0] += vx * math.cos(pose[2]) * dt
        pose[1] += vx * math.sin(pose[2]) * dt
        pose[2] += wz * dt
        positions.append(pose.copy())
        stamp = node.get_clock().now().to_msg()
        tf = transform('odom', 'base_footprint', x=pose[0], y=pose[1])
        tf.header.stamp = stamp
        tf.transform.rotation.z, tf.transform.rotation.w = (
            math.sin(pose[2] / 2), math.cos(pose[2] / 2))
        dynamic_tf.sendTransform(tf)
        odom = Odometry(header=tf.header, child_frame_id=tf.child_frame_id)
        odom.pose.pose.position.x, odom.pose.pose.position.y = pose[:2]
        odom.pose.pose.orientation = tf.transform.rotation
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = vx, wz
        odometry.publish(odom)
        manual.publish(Bool(data=False))
        distances = _ranges(*pose[:2], pose[2] + beams, obstacles,
                            front_wall=1.95 if facing_wall else 55.8, half_width=half_width)
        points = np.column_stack((distances * np.cos(beams), distances * np.sin(beams),
                                  np.full_like(beams, .8)))
        lidar_frames += 1
        if pose[0] < .25 and lidar_frames % 4 == 0:
            # A malformed sparse frame must not suppress a recent healthy scan.
            points = points[:1]
        lidar.publish(xyz_cloud(np.vstack((points, noise)), stamp))
        if depth_enabled:
            cx = pose[0] + .45 * math.cos(pose[2])
            cy = pose[1] + .45 * math.sin(pose[2])
            # The narrow-corridor case must find the obstacle from LiDAR alone.
            depth_obstacles = [] if narrow else obstacles
            depth_at_column = _ranges(cx, cy, pose[2] + columns, depth_obstacles,
                                      front_wall=camera_wall, half_width=half_width) * np.cos(columns)
            values = np.minimum(np.minimum(depth_at_column[None, :], floor[:, None]), 4.0)
            mm = np.where((values >= .1) & (values <= 5.), values * 1000, 0).astype('<u2')
            mm[48, 80] = 75  # One short-range pixel, including across consecutive frames.
            if occluder_depth_mm is not None:
                mm[25:75, 45:115] = occluder_depth_mm
            image = Image(height=96, width=160, encoding='16UC1', step=320, data=mm.tobytes())
            image.header.stamp, image.header.frame_id = stamp, 'synthetic_optical'
            calibration = CameraInfo(header=image.header, height=96, width=160)
            calibration.p = [90., 0., 80., 0., 0., 90., 48., 0., 0., 0., 1., 0.]
            info.publish(calibration)
            depth.publish(image)

    def receive_velocity(msg):
        command[:] = [msg.linear.x, msg.angular.z]
        assert 0. <= msg.linear.x <= .50 + 1e-9
        if msg.linear.x == 0.:
            assert abs(msg.angular.z) <= .20 + 1e-9

    node.create_timer(.05, publish)
    node.create_subscription(Twist, '/option22/cmd_vel_safe', receive_velocity, 1)
    node.create_subscription(Bool, '/option22/auto_request', lambda msg: requests.append(msg.data), 1)
    node.create_subscription(String, '/option22/state', lambda msg: states.append(msg.data), 1)
    node.create_subscription(NavPath, '/plan', plans.append, 1)
    if scene == 'close_planner':
        displayed_stop, markers, displayed_paths = [], [], []
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(Bool, '/option22/display_estop',
                                 lambda msg: displayed_stop.append(msg.data), latched)
        node.create_subscription(Marker, '/option22/navigation_marker', markers.append, latched)
        node.create_subscription(NavPath, '/option22/display_plan', displayed_paths.append, latched)
    client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    planner = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
    navigator_state = node.create_client(GetState, '/bt_navigator/get_state')

    log = tmp_path / 'option22.log'
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOG_DIR=str(tmp_path / 'ros'))
    with log.open('w') as stream:
        process = subprocess.Popen(
            [sys.executable, __file__, '--launch-simulation', str(map_path)],
            env=env, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)

        def until(predicate, timeout=15.0):
            deadline = time.monotonic() + timeout
            while not predicate() and time.monotonic() < deadline:
                assert process.poll() is None, log.read_text()
                executor.spin_once(timeout_sec=.02)
            assert predicate(), (positions[-160::20], command, states[-20:],
                                 log.read_text()[-10000:])

        try:
            until(navigator_state.service_is_ready)
            active = False
            deadline = time.monotonic() + 15.0
            while not active and time.monotonic() < deadline:
                current = navigator_state.call_async(GetState.Request())
                until(current.done)
                active = current.result().current_state.id == State.PRIMARY_STATE_ACTIVE
                executor.spin_once(timeout_sec=.05)
            assert active, log.read_text()
            until(client.server_is_ready)
            until(lambda: requests and not requests[-1])
            startup_clear_index = requests.index(False)
            if scene in ('close_planner', 'corridor'):
                until(planner.server_is_ready)
                until(lambda: states and states[-1] == 'IDLE')
                # Lifecycle activation can precede DDS matching of the obstacle feed.
                startup_goal = ComputePathToPose.Goal(planner_id='GridBased')
                startup_goal.goal.header.frame_id = 'map'
                startup_goal.goal.pose.position.x, startup_goal.goal.pose.orientation.w = .6, 1.
                deadline = time.monotonic() + 5.0
                while True:
                    accepted = planner.send_goal_async(startup_goal)
                    until(accepted.done)
                    startup_result = accepted.result().get_result_async()
                    until(startup_result.done)
                    if startup_result.result().status == GoalStatus.STATUS_SUCCEEDED:
                        break
                    assert time.monotonic() < deadline, log.read_text()
            if scene == 'close_planner':
                goal = ComputePathToPose.Goal(planner_id='GridBased')
                goal.goal.header.frame_id = 'map'
                goal.goal.pose.position.x, goal.goal.pose.orientation.w = 5.75, 1.

                def compute_path():
                    accepted = planner.send_goal_async(goal)
                    until(accepted.done)
                    result = accepted.result().get_result_async()
                    until(result.done)
                    response = result.result()
                    assert response.status == GoalStatus.STATUS_SUCCEEDED, log.read_text()
                    return response.result.path

                for obstacle_x, expect_bypass in [(1.64, False), (1.44, True)]:
                    obstacles[:] = [(obstacle_x, 0., .24)]
                    observed_at = time.monotonic() + (0.55 if expect_bypass else 1.50)
                    until(lambda: time.monotonic() >= observed_at)
                    poses = compute_path().poses
                    offset = max(abs(p.pose.position.y) for p in poses)
                    assert (offset > .45) if expect_bypass else (offset < .10)
                    if expect_bypass:
                        heading = poses[0].pose.orientation
                        assert abs(2 * math.atan2(heading.z, heading.w)) > .25

                # Simulate the original obstacle leaving both sensors' view
                # during the turn, including a gap longer than the old reset.
                pose[2] = 2 * math.atan2(heading.z, heading.w)
                obstacles.clear()
                after_turn = time.monotonic() + 3.2
                until(lambda: time.monotonic() >= after_turn)
                kept = compute_path().poses
                assert [(p.pose.position.x, p.pose.position.y) for p in kept] == [
                    (p.pose.position.x, p.pose.position.y) for p in poses]
                assert kept[0].pose.orientation == heading
                pose[2] = 0.
                occluder_depth_mm = 180
                until(lambda: requests and requests[-1] and displayed_stop and displayed_stop[-1]
                      and markers and markers[-1].text == 'E-STOP'
                      and displayed_paths and not displayed_paths[-1].poses)
                accepted = planner.send_goal_async(goal)
                until(accepted.done)
                result = accepted.result().get_result_async()
                until(result.done)
                assert result.result().status == GoalStatus.STATUS_ABORTED
                assert pose[:2] == [0., 0.] and command == [0., 0.]
                occluder_depth_mm = None
                until(lambda: not requests[-1] and not displayed_stop[-1]
                      and markers[-1].action == Marker.DELETE)
                occluder_depth_mm = 0  # A covered lens can return invalid depth.
                until(lambda: requests[-1] and displayed_stop[-1])
                return

            if scene == 'corridor':
                corridor_goal = ComputePathToPose.Goal(planner_id='GridBased')
                corridor_goal.goal.header.frame_id = 'map'
                corridor_goal.goal.pose.position.x = 52.
                corridor_goal.goal.pose.orientation.w = 1.
                long_goal = planner.send_goal_async(corridor_goal)
                until(long_goal.done)
                long_result = long_goal.result().get_result_async()
                until(long_result.done)
                assert long_result.result().status == GoalStatus.STATUS_SUCCEEDED, log.read_text()
                long_plan = long_result.result().result.path.poses
                assert long_plan[-1].pose.position.x == pytest.approx(52.)
                assert max(abs(p.pose.position.y) for p in long_plan) < .10
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal_x = 1.44 if facing_wall else 5.75
            goal.pose.pose.position.x, goal.pose.pose.orientation.w = goal_x, 1.
            accepted = client.send_goal_async(goal)
            until(accepted.done)
            handle = accepted.result()
            assert handle.accepted
            result = handle.get_result_async()
            until(lambda: pose[0] > .25)
            assert max(abs(p.pose.position.y) for p in plans[-1].poses) < .10
            assert not any(requests[startup_clear_index:])
            assert not {'WAITING', 'TRAJECTORY_BLOCKED', 'NO_LOCAL_DETOUR'}.intersection(states)
            moving_index = states.index('FOLLOWING_ROUTE')
            assert 'LIDAR_UNAVAILABLE' not in states[moving_index:]
            if facing_wall:
                until(lambda: result.done() or 'WAITING' in states)
                assert 'WAITING' not in states, log.read_text()
                assert result.result().status == GoalStatus.STATUS_SUCCEEDED, log.read_text()
                assert not {'WAITING', 'DETOURING', 'NO_LOCAL_DETOUR'}.intersection(states)
                assert not any(requests[startup_clear_index:])
                assert abs(pose[0] - goal_x) < .22
                occluder_depth_mm = 180
                until(lambda: states[-1] == 'CAMERA_ESTOP' and requests[-1])
                return

            if scene == 'corridor':
                until(lambda: command[0] >= .49)
            obstacles.append((pose[0] + 1.65, 0., .24))
            if narrow:
                until(lambda: states[-1] == 'NO_LOCAL_DETOUR')
                stopped = pose.copy()
                assert command == [0., 0.]
                assert pose == stopped and not requests[-1]
                assert all(abs(p.pose.position.y) < .10 for plan in plans for p in plan.poses)
                obstacles.clear()
                until(lambda: pose[0] > stopped[0] + .15)
                accepted = handle.cancel_goal_async()
                until(accepted.done)
                until(result.done)
                assert result.result().status == GoalStatus.STATUS_CANCELED
            else:
                until(lambda: states[-1] == 'DETOUR_TURNING' and len(plans)
                      and max(abs(p.pose.position.y) for p in plans[-1].poses) > .45)
                assert not requests[-1] and command[0] == 0.
                until(lambda: abs(command[1]) == pytest.approx(.20))
                stopped = pose.copy()
                bypass = plans[-1].poses
                assert max(abs(p.pose.position.y) for p in bypass) <= .76
                assert all(a.pose.position.x <= b.pose.position.x + .005
                           for a, b in zip(bypass, bypass[1:]))
                suffix = [p for p in bypass if p.pose.position.x > stopped[0] + 3.82]
                assert suffix and all(abs(p.pose.position.y) < .07 for p in suffix)
                for pose_on_path in bypass:
                    q = pose_on_path.pose.orientation
                    yaw = 2.0 * math.atan2(q.z, q.w)
                    s, c = abs(math.sin(yaw)), abs(math.cos(yaw))
                    footprint_half_width = max(.40 * s + .275 * c, .50 * s + .11 * c)
                    assert abs(pose_on_path.pose.position.y) + footprint_half_width < 1.54
                until(lambda: abs(pose[1]) > .35)
                occluder_depth_mm = 180
                until(lambda: states[-1] == 'CAMERA_ESTOP' and requests[-1]
                      and command == [0., 0.])
                stopped = pose.copy()
                delay = time.monotonic() + .5
                until(lambda: time.monotonic() >= delay)
                assert pose == stopped and requests[-1]
                occluder_depth_mm = None
                until(result.done, timeout=52.)
                assert result.result().status == GoalStatus.STATUS_SUCCEEDED, log.read_text()
                assert max(abs(p[1]) for p in positions) > .45
                assert abs(pose[0] - goal_x) < .22 and abs(pose[1]) < .22
            depth_enabled = False
            until(lambda: states[-1] == 'CAMERA_ESTOP' and requests[-1]
                  and command == [0., 0.])
        finally:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            executor.remove_node(node)
            node.destroy_node()
            context.shutdown()


if __name__ == '__main__' and sys.argv[1:2] == ['--launch-simulation']:
    sys.exit(_start_navigation(sys.argv[2]))
