"""STM32/NDT navigation with fused dynamic obstacles and guarded chassis output."""
import math
from pathlib import Path
from tempfile import NamedTemporaryFile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue


def navigation_parameters(share, planner, gate):
    params = yaml.safe_load((share / 'config' / 'nav2_params.yaml').read_text())
    params['bt_navigator']['ros__parameters']['default_server_timeout'] = 100
    local = params['local_costmap']['local_costmap']['ros__parameters']
    global_map = params['global_costmap']['global_costmap']['ros__parameters']
    local['static_layer'] = global_map['static_layer'].copy()
    for costmap in (local, global_map):
        costmap['update_frequency'] = 5.0
        costmap['track_unknown_space'] = True
        costmap['footprint'] = (
            '[[-0.40, -0.275], [-0.40, 0.275], [0.40, 0.275], [0.40, 0.11], '
            '[0.50, 0.11], [0.50, -0.11], [0.40, -0.11], [0.40, -0.275]]')
        costmap['plugins'] = ['static_layer', 'inflation_layer']
    params['planner_server']['ros__parameters']['GridBased'].update(
        planner, plugin='traymover_robot_nav::Option22LocalDetour',
        tracking_clearance_m=gate['motion_clearance_m'],
        allow_unknown=False, tolerance=0.0)
    follow = params['controller_server']['ros__parameters']['FollowPath']
    follow.update(max_angular_accel=4.4, rotate_to_heading_angular_vel=gate['angular_limit_radps'],
                  desired_linear_vel=gate['cruise_linear_speed_mps'],
                  min_lookahead_dist=0.30, max_lookahead_dist=0.48,
                  use_collision_detection=True)
    return params


def rviz_profile(share):
    profile = yaml.safe_load((share / 'rviz' / 'traymover_nav.rviz').read_text())
    displays = profile['Visualization Manager']['Displays']
    for display in displays:
        if display.get('Name') == 'GlobalPlan':
            display['Topic'].update({
                'Value': '/option22/display_plan', 'Durability Policy': 'Transient Local'})
        elif display.get('Name') == 'LocalPlan':
            display['Enabled'] = display['Value'] = False
    displays.append({
        'Class': 'rviz_default_plugins/Marker',
        'Name': 'Navigation status',
        'Enabled': True,
        'Value': True,
        'Topic': {
            'Value': '/option22/navigation_marker', 'Depth': 1,
            'Durability Policy': 'Transient Local', 'History Policy': 'Keep Last',
            'Reliability Policy': 'Reliable',
        },
    })
    return profile


def launch_nodes(context, nav, base):
    config = yaml.safe_load((nav / 'config' / 'option22_obstacles.yaml').read_text())
    wait = float(LaunchConfiguration('wait_before_detour_sec').perform(context))
    if not math.isfinite(wait) or wait < 0:
        raise ValueError('Option 22 requires a nonnegative wait')
    config['planner']['wait_before_detour_sec'] = wait

    def include(share, filename, **arguments):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch' / filename)),
            launch_arguments=arguments.items(),
        )

    use_sim_time = LaunchConfiguration('use_sim_time')
    start_camera = IfCondition(LaunchConfiguration('start_realsense_driver'))
    actions = []
    if start_camera.evaluate(context):
        camera = Path(get_package_share_directory('realsense2_camera'))
        actions.append(GroupAction(scoped=True, forwarding=False,
                                   launch_configurations={
                                       'enable_depth': 'true', 'enable_color': 'false',
                                       'depth_module.depth_profile': '640x480x15',
                                       'align_depth.enable': 'false',
                                       'pointcloud.enable': 'false', 'enable_sync': 'false',
                                   }, actions=[include(camera, 'rs_launch.py')]))

    with NamedTemporaryFile(mode='w', prefix='traymover_option22_', suffix='.yaml',
                            delete=False) as stream:
        yaml.safe_dump(navigation_parameters(nav, **config), stream, sort_keys=False)
    params = Path(stream.name)
    actions.extend([
        RegisterEventHandler(OnShutdown(on_shutdown=lambda *_: params.unlink(missing_ok=True))),
        GroupAction(scoped=True, condition=IfCondition(LaunchConfiguration('bringup_hardware')),
                    actions=[
                        SetRemap(src='/cmd_vel', dst='/option22/cmd_vel_safe'),
                        SetRemap(src='/traymover_estop/auto_request', dst='/option22/auto_request'),
                        include(base, 'base_serial.launch.py',
                                use_imu='false', odom_source_mode='stm32_velocity',
                                robot_config=LaunchConfiguration('robot_config'),
                                auto_estop_enabled='true'),
                        include(base, 'traymover_lidar.launch.py', enable_scan_bridge='false'),
                    ]),
        Node(package='traymover_robot_nav', executable='option22_obstacle_gate.py',
             name='option22_obstacle_gate', output='screen',
             additional_env={'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1'}, parameters=[{
                 **config['gate'], 'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                 'depth_topic': LaunchConfiguration('depth_topic'),
                 'camera_info_topic': LaunchConfiguration('camera_info_topic'),
             }]),
        GroupAction(scoped=True, actions=[
            SetRemap(src='/cmd_vel', dst='/option22/cmd_vel_nav'),
            include(nav, 'traymover_nav_stm32.launch.py',
                    pcd_path=LaunchConfiguration('pcd_path'), map=LaunchConfiguration('map'),
                    use_sim_time=use_sim_time, bringup_hardware='false',
                    launch_rviz='false',
                    height_boost_enabled=LaunchConfiguration('height_boost_enabled'),
                    height_boost_min_z=LaunchConfiguration('height_boost_min_z'),
                    auto_correction_enabled=LaunchConfiguration('auto_correction_enabled'),
                    lifecycle_manager_package='traymover_robot_nav',
                    lifecycle_manager_executable='option18_lifecycle_manager',
                    params_file=str(params),
                    bt_xml=str(nav / 'behavior_trees' / 'option22_navigate_to_pose.xml'),
                    through_poses_bt_xml=str(
                        nav / 'behavior_trees' / 'option22_navigate_through_poses.xml')),
        ]),
    ])
    if IfCondition(LaunchConfiguration('launch_rviz')).evaluate(context):
        with NamedTemporaryFile(mode='w', prefix='traymover_option22_', suffix='.rviz',
                                delete=False) as stream:
            yaml.safe_dump(rviz_profile(nav), stream, sort_keys=False)
        profile = Path(stream.name)
        actions.extend([
            RegisterEventHandler(OnShutdown(on_shutdown=lambda *_: profile.unlink(missing_ok=True))),
            Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
                 arguments=['-d', str(profile)],
                 parameters=[{'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
                 remappings=[('/traymover_estop/state', '/option22/display_estop'),
                             ('/traymover_estop/auto_state', '/option22/display_auto_estop')]),
        ])
    return actions


def generate_launch_description():
    nav = Path(get_package_share_directory('traymover_robot_nav'))
    base = Path(get_package_share_directory('turn_on_traymover_robot'))
    defaults = {
        'robot_config': str(base / 'config' / 'stm32_navigation.yaml'),
        'bringup_hardware': 'false', 'launch_rviz': 'false', 'use_sim_time': 'false',
        'start_realsense_driver': 'true',
        'height_boost_enabled': 'false', 'height_boost_min_z': '2.0',
        'auto_correction_enabled': 'true',
        'depth_topic': '/camera/camera/depth/image_rect_raw',
        'camera_info_topic': '/camera/camera/depth/camera_info',
        'wait_before_detour_sec': str(
            yaml.safe_load((nav / 'config' / 'option22_obstacles.yaml').read_text())
            ['planner']['wait_before_detour_sec']),
    }
    return LaunchDescription([
        DeclareLaunchArgument('pcd_path', description='Existing prior PCD map (absolute path).'),
        DeclareLaunchArgument('map', description='2D occupancy YAML with observed free space.'),
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        OpaqueFunction(function=launch_nodes, args=[nav, base]),
    ])
