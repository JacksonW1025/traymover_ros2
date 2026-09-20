"""Option 18: option 10 navigation with EStop-safe obstacle detouring.

Initial tuning values only; no hardware validation is implied. Collision Monitor
remains downstream and the chassis driver's EStop gate remains authoritative.
"""
import math
import os
from pathlib import Path
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.actions import LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue


# Shared with the pure gate. Numeric values are deliberately exposed for tuning.
FLOAT_DEFAULTS = {
    'wait_before_detour_sec': '1.0',
    'detour_trigger_distance_m': '1.00',
    'detour_guard_distance_m': '0.90',
    'detour_max_linear_speed': '0.08',
    'detour_max_angular_speed': '0.25',
    'clear_confirm_sec': '0.4',
    'stop_settle_time_sec': '0.25',
    'detour_target_clearance_m': '1.10',
    'detour_turn_angle_deg': '45.0',
    'simple_obstacle_half_width_m': '0.35',
    'detour_first_leg_m': '2.2',
    'detour_forward_leg_m': '1.0',
    'scan_timeout_sec': '2.0',
    'realsense_timeout_sec': '1.5',
    'cmd_vel_timeout_sec': '0.5',
    'turn_phase_timeout_sec': '12.0',
    'straight_phase_timeout_sec': '60.0',
    'tf_timeout_sec': '1.5',
    # NDT aligns every 2 seconds; TF alone is not localization evidence.
    'localization_timeout_sec': '5.0',
    'estop_timeout_sec': '1.0',
    'supervisor_rate_hz': '20.0',
}
INT_DEFAULTS = {'obstacle_confirm_frames': '3', 'min_obstacle_points': '8'}

# Scoped to option 18; leave standalone Auto EStop / other menu modes unchanged.
AUTO_ESTOP_OVERRIDES = {
    'depth_timeout_sec': '1.5',
    'min_near_pixel_fraction': '0.02',
    'min_near_tile_count': '4',
    'obstacle_frame_count': '3',
    'min_valid_fraction': '0.25',
    'severe_invalid_fraction': '0.10',
    'low_valid_frame_count': '4',
    'clear_frame_count': '12',
}


def _navigation_remappings():
    # This Humble installation remaps action transports, not action basenames.
    # All three services and both topics must agree on the private backend.
    remaps = [
        (f'/{action}/_action/{endpoint}', f'/{action}_detour_backend/_action/{endpoint}')
        for action in ('navigate_to_pose', 'navigate_through_poses')
        for endpoint in ('send_goal', 'get_result', 'cancel_goal', 'feedback', 'status')
    ]
    return remaps + [('/goal_pose', '/goal_pose_detour_backend_unused'),
                     ('/cmd_vel', '/cmd_vel_nav')]


def _enabled(context, name):
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value not in ('true', 'false', '1', '0', 'yes', 'no', 'on', 'off'):
        raise ValueError(f'{name} must be a boolean, got {value!r}')
    return value in ('true', '1', 'yes', 'on')


def _validate_numeric_arguments(context):
    # Validate before returning any launch action, including camera/chassis.
    values = {
        name: float(LaunchConfiguration(name).perform(context))
        for name in (*FLOAT_DEFAULTS, 'dynamic_obstacle_persistence_sec')
    }
    for name, value in values.items():
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be finite and positive')
    for name in (*INT_DEFAULTS, 'realsense_scan_height'):
        value = float(LaunchConfiguration(name).perform(context))
        if not math.isfinite(value) or value < 1 or value != int(value):
            raise ValueError(f'{name} must be a positive integer')
    if not (values['detour_trigger_distance_m'] > values['detour_guard_distance_m'] > 0.8):
        raise ValueError('require detour trigger > guard > Auto EStop stop distance (0.8 m)')
    if values['detour_target_clearance_m'] <= values['detour_guard_distance_m']:
        raise ValueError('detour target clearance must exceed guard distance')
    if values['detour_max_linear_speed'] > 0.10 or values['detour_max_angular_speed'] > 0.30:
        raise ValueError('detour speed must not exceed 0.10 m/s or 0.30 rad/s')
    if not 5.0 <= values['detour_turn_angle_deg'] <= 75.0:
        raise ValueError('detour turn angle must be within 5 to 75 degrees')


def _navigation_params(context):
    """Default to exactly option 10; sensor costmap layers are an explicit opt-in."""
    if not _enabled(context, 'enable_dynamic_costmaps'):
        return LaunchConfiguration('params_file').perform(context)
    with open(LaunchConfiguration('params_file').perform(context), encoding='utf-8') as stream:
        params = yaml.safe_load(stream)
    with open(LaunchConfiguration('obstacle_params_file').perform(context),
              encoding='utf-8') as stream:
        obstacles = yaml.safe_load(stream)
    persistence = float(LaunchConfiguration('dynamic_obstacle_persistence_sec').perform(context))
    for name in ('local_costmap', 'global_costmap'):
        costmap = params[name][name]['ros__parameters']
        layer = obstacles[name][name]['ros__parameters']['obstacle_layer']
        for source in layer['observation_sources'].split():
            if 'observation_persistence' in layer[source]:
                layer[source]['observation_persistence'] = persistence
        costmap['obstacle_layer'] = layer
        if 'obstacle_layer' not in costmap['plugins']:
            index = (costmap['plugins'].index('inflation_layer')
                     if 'inflation_layer' in costmap['plugins'] else len(costmap['plugins']))
            costmap['plugins'].insert(index, 'obstacle_layer')
    # Like Nav2's RewrittenYaml, materialize parameters for the included launch.
    with tempfile.NamedTemporaryFile(mode='w', prefix='traymover_detour_',
                                     suffix='.yaml', delete=False) as stream:
        yaml.safe_dump(params, stream, sort_keys=False)
        return stream.name


def _launch_nodes(context):
    _validate_numeric_arguments(context)
    configured_path = _navigation_params(context)
    nav_share = get_package_share_directory('traymover_robot_nav')
    bringup_share = get_package_share_directory('turn_on_traymover_robot')
    safety_share = get_package_share_directory('traymover_robot_safety')
    auto_enabled = _enabled(context, 'enable_auto_estop')
    requested_detour = _enabled(context, 'enable_detour')
    detour_enabled = requested_detour and auto_enabled
    actions = []
    if requested_detour and not auto_enabled:
        actions.append(LogInfo(msg='[traymover] Wait-and-detour requires automatic EStop.'))
        actions.append(LogInfo(msg='[traymover] Detour has been forced OFF.'))

    def include(share, filename, arguments, **kwargs):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', filename)),
            launch_arguments=arguments.items(), **kwargs)

    # Main launch owns the only RealSense driver. The included navigation and
    # automatic EStop launches are explicitly forbidden from starting another.
    if _enabled(context, 'start_realsense_driver'):
        realsense_share = get_package_share_directory('realsense2_camera')
        actions.append(GroupAction(
            scoped=True, forwarding=False,
            launch_configurations={
                'enable_depth': 'true', 'enable_color': 'false',
                'depth_module.depth_profile': '640x480x15',
                'align_depth.enable': 'false', 'pointcloud.enable': 'false',
                'enable_sync': 'false',
            },
            actions=[include(realsense_share, 'rs_launch.py', {})],
        ))
    if auto_enabled:
        actions.append(GroupAction(scoped=True, actions=[include(
            safety_share, 'traymover_auto_estop.launch.py', {
                **AUTO_ESTOP_OVERRIDES,
                'start_realsense_driver': 'false',
                'depth_topic': LaunchConfiguration('realsense_depth_topic'),
            })]))

    actions.extend([
        include(bringup_share, 'base_serial.launch.py', {
            'odom_source_mode': 'none',
            'auto_estop_enabled': 'true' if auto_enabled else 'false',
        }, condition=IfCondition(LaunchConfiguration('bringup_hardware'))),
        include(bringup_share, 'traymover_lidar.launch.py', {
            'enable_scan_bridge': 'false',
        }, condition=IfCondition(LaunchConfiguration('bringup_hardware'))),
    ])

    if detour_enabled:
        # The navigation scan covers 270 degrees. Rotation checks
        # need independent, observed evidence for the missing rear sector.
        # Unknown/no-return bins never certify free space in the supervisor.
        actions.append(Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='traymover_detour_perimeter_scan', output='screen',
            remappings=[
                ('cloud_in', '/point_cloud_raw'),
                ('scan', '/scan_detour_perimeter'),
            ],
            parameters=[{
                'target_frame': 'base_link', 'transform_tolerance': 0.05,
                'min_height': -0.25, 'max_height': 1.05,
                'angle_min': -math.pi, 'angle_max': math.pi,
                'angle_increment': 0.0087, 'scan_time': 0.1,
                'range_min': 0.30, 'range_max': 60.0,
                'use_inf': True, 'inf_epsilon': 1.0,
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
            }],
        ))

    navigation_args = {
        name: LaunchConfiguration(name) for name in (
            'use_sim_time', 'autostart', 'map', 'pcd_path', 'launch_rviz',
            'rviz_config', 'bt_xml', 'through_poses_bt_xml',
            'height_boost_enabled', 'height_boost_min_z', 'auto_correction_enabled',
        )
    }
    navigation_args.update({
        'params_file': configured_path,
        'bringup_hardware': 'false',
        'publish_scan': 'true',
        'lifecycle_manager_package': 'traymover_robot_nav',
        'lifecycle_manager_executable': 'option18_lifecycle_manager',
    })
    # Keep RViz on the public proxy action; SetRemap affects ROS Nodes in the
    # nested stack, so launch RViz separately outside this group.
    navigation_args['launch_rviz'] = 'false'
    actions.append(GroupAction(scoped=True, actions=[
        *(SetRemap(src=src, dst=dst) for src, dst in _navigation_remappings()),
        include(nav_share, 'traymover_nav.launch.py', navigation_args),
    ]))
    actions.append(Node(
        package='depthimage_to_laserscan', executable='depthimage_to_laserscan_node',
        name='traymover_realsense_depth_to_laserscan', output='screen',
        remappings=[
            ('depth', LaunchConfiguration('realsense_depth_topic')),
            ('depth_camera_info', LaunchConfiguration('realsense_info_topic')),
            ('scan', '/scan_realsense'),
        ],
        parameters=[{
            'scan_height': ParameterValue(
                LaunchConfiguration('realsense_scan_height'), value_type=int),
            'range_min': 0.20, 'range_max': 3.0,
            'output_frame': LaunchConfiguration('realsense_output_frame'),
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
        }],
    ))
    actions.append(include(nav_share, 'collision_monitor.launch.py', {
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'params_file': LaunchConfiguration('collision_params_file'),
        'lifecycle_manager_package': 'traymover_robot_nav',
        'lifecycle_manager_executable': 'option18_lifecycle_manager',
    }))
    actions.append(Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
    ))
    supervisor_params = {
        name: ParameterValue(LaunchConfiguration(name), value_type=float)
        for name in FLOAT_DEFAULTS
    }
    supervisor_params.update({
        name: ParameterValue(LaunchConfiguration(name), value_type=int)
        for name in INT_DEFAULTS
    })
    supervisor_params.update({
        'enable_auto_estop': auto_enabled,
        'enable_detour': detour_enabled,
        'simple_detour': _enabled(context, 'simple_detour'),
        'allow_straight_motion': _enabled(context, 'allow_straight_motion'),
        'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
        'pose_frame': LaunchConfiguration('pose_frame'),
        'localization_topic': LaunchConfiguration('localization_topic'),
    })
    actions.append(Node(
        package='traymover_robot_nav', executable='detour_supervisor.py',
        name='traymover_detour_supervisor', output='screen', parameters=[supervisor_params],
    ))
    return actions


def generate_launch_description():
    nav_share = get_package_share_directory('traymover_robot_nav')
    pcd_dir = Path('/home/wheeltec/traymover_ros2/src/traymover_robot_slam/FAST_LIO/PCD')
    candidates = sorted(pcd_dir.glob('*.pcd'), key=lambda p: p.stat().st_mtime, reverse=True)
    default_pcd = str(candidates[0] if candidates else pcd_dir / 'traymover.pcd')
    defaults = {
        'enable_auto_estop': 'true', 'enable_detour': 'true',
        'simple_detour': 'true', 'enable_dynamic_costmaps': 'false',
        'allow_straight_motion': 'true',
        'use_sim_time': 'false', 'autostart': 'true',
        'bringup_hardware': 'false', 'launch_rviz': 'false',
        'start_realsense_driver': 'true',
        'dynamic_obstacle_persistence_sec': '5.0',
        'params_file': os.path.join(nav_share, 'config', 'nav2_params.yaml'),
        'obstacle_params_file': os.path.join(
            nav_share, 'config', 'nav2_params_lidar_realsense_detour.yaml'),
        'collision_params_file': os.path.join(
            nav_share, 'config', 'collision_monitor_lidar_realsense_detour.yaml'),
        'map': os.path.join(nav_share, 'map', 'traymover_2d.yaml'),
        'pcd_path': default_pcd,
        'height_boost_enabled': 'false',
        'height_boost_min_z': '2.0',
        'auto_correction_enabled': 'false',
        'rviz_config': os.path.join(nav_share, 'rviz', 'traymover_nav.rviz'),
        'bt_xml': os.path.join(nav_share, 'behavior_trees', 'simple_navigate_to_pose.xml'),
        'through_poses_bt_xml': os.path.join(
            nav_share, 'behavior_trees', 'simple_navigate_through_poses.xml'),
        'realsense_depth_topic': '/camera/camera/depth/image_rect_raw',
        'realsense_info_topic': '/camera/camera/depth/camera_info',
        'realsense_output_frame': 'camera_depth_frame',
        'realsense_scan_height': '320',
        'pose_frame': 'odom', 'localization_topic': '/pcl_pose',
        **FLOAT_DEFAULTS, **INT_DEFAULTS,
    }
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()),
        OpaqueFunction(function=_launch_nodes),
    ])
