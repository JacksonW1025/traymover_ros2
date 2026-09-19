"""AMCL fallback navigation. No NDT; hardware is opt-in for replay/testing."""
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue


def launch_nodes(context):
    share = Path(get_package_share_directory('traymover_robot_nav'))
    assets = Path(LaunchConfiguration('map_directory').perform(context)).resolve()
    manifest_path = assets / 'manifest.json'
    manifest = json.loads(manifest_path.read_text())
    for filename in ('localization.yaml', 'navigation.yaml'):
        if not (assets / filename).is_file():
            raise ValueError(f'Option 20 map missing: {assets / filename}')
    pcd = Path(manifest['pcd_path'])
    if not pcd.is_file():
        raise ValueError(f'Option 20 source PCD missing: {pcd}')
    source_stat = pcd.stat()
    if (source_stat.st_size, source_stat.st_mtime_ns) != (
            manifest['pcd_size'], manifest['pcd_mtime_ns']):
        raise ValueError('Source PCD changed after map preparation; regenerate option 20 maps')
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    common = {'use_sim_time': use_sim_time}
    params = str(share / 'config' / 'nav2_option20.yaml')
    bringup = Path(get_package_share_directory('turn_on_traymover_robot'))
    safety = Path(get_package_share_directory('traymover_robot_safety'))
    camera = Path(get_package_share_directory('realsense2_camera'))
    description = Path(get_package_share_directory('traymover_robot_description'))
    fastlio = Path(get_package_share_directory('fast_lio'))

    def include(root, filename, arguments=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(root / 'launch' / filename)),
            launch_arguments=(arguments or {}).items())

    def own(executable, parameters=None):
        return Node(package='traymover_robot_nav', executable=executable,
                    name=executable.removesuffix('.py'), output='screen',
                    parameters=[common, parameters or {}])

    def static(name, parent, child, xyz):
        return Node(package='tf2_ros', executable='static_transform_publisher',
                    name=name, parameters=[common],
                    arguments=[str(v) for v in xyz] + ['0', '0', '0', parent, child])

    model = ParameterValue(Command([
        'xacro ', str(description / 'urdf' / 'traymover.urdf.xacro')]), value_type=str)
    nodes = [
        # Scope the chassis command remap so legacy /cmd_vel publishers cannot
        # bypass the final guard. Existing manual/automatic EStop stays active.
        GroupAction(condition=IfCondition(LaunchConfiguration('bringup_hardware')),
                    actions=[
                        SetRemap(src='/cmd_vel', dst='/option20/cmd_vel_safe'),
                        include(bringup, 'base_serial.launch.py', {
                            'odom_source_mode': 'none', 'use_imu': 'true',
                            'auto_estop_enabled': 'true'}),
                        include(bringup, 'traymover_lidar.launch.py', {
                            'enable_scan_bridge': 'false'}),
                    ]),
        GroupAction(condition=IfCondition(LaunchConfiguration('bringup_hardware')),
                    scoped=True, forwarding=False,
                    launch_configurations={
                        'enable_depth': 'true', 'enable_color': 'false',
                        'depth_module.depth_profile': '640x480x15',
                        'align_depth.enable': 'false', 'pointcloud.enable': 'false',
                        'enable_sync': 'false'},
                    actions=[include(camera, 'rs_launch.py')]),
        GroupAction(condition=IfCondition(LaunchConfiguration('bringup_hardware')),
                    actions=[include(safety, 'traymover_auto_estop.launch.py', {
                        'start_realsense_driver': 'false'})]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='option20_robot_state_publisher', parameters=[common, {
                 'robot_description': model}], output='screen'),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             name='option20_joint_state_publisher', parameters=[common, {
                 'robot_description': model}], output='screen'),
        # Two branches: raw inertial body for deskewed point transforms, and
        # planar base_footprint for AMCL/Nav2. Never attach base_link to body.
        static('option20_raw_origin', 'odom', 'camera_init', [0, 0, .08]),
        static('option20_base_height', 'base_footprint', 'base_link', [0, 0, .20]),
        static('option20_scan_origin', 'base_footprint', 'option20_scan', [.435, 0, 0]),
        Node(package='fast_lio', executable='fastlio_mapping', name='fastlio_mapping',
             output='screen', parameters=[
                 str(fastlio / 'config' / 'traymover.yaml'),
                 common, {'pcd_save.pcd_save_en': False, 'publish.map_en': False,
                          'publish.path_en': False, 'runtime_pos_log_enable': False,
                          'publish.scan_publish_en': True}],
             remappings=[('/Odometry', '/option20/lio_odom'),
                         ('/path', '/option20/lio_path')]),
        own('option20_odom.py'),
        own('option20_clouds.py', {
            'height_bands': [float(value) for band in manifest['bands'] for value in band]}),
        own('option20_depth.py'),
        own('option20_guard.py'),
        own('option20_pcd_display.py', {
            'pcd_path': str(pcd), 'map_manifest': str(manifest_path)}),
    ]
    for name, filename, topic in (
            ('map_server', 'navigation.yaml', '/map'),
            ('localization_map_server', 'localization.yaml', '/localization_map')):
        nodes.append(Node(package='nav2_map_server', executable='map_server',
                          name=name, output='screen', parameters=[common, {
                              'yaml_filename': str(assets / filename),
                              'topic_name': topic, 'frame_id': 'map'}],
                          remappings=[('map', topic)]))
    nodes.append(Node(package='nav2_amcl', executable='amcl', name='amcl',
                      output='screen', parameters=[params, common],
                      remappings=[('map', '/localization_map'),
                                  ('scan', '/scan_localization')]))
    for package, executable in (
            ('nav2_planner', 'planner_server'),
            ('nav2_controller', 'controller_server'),
            ('nav2_bt_navigator', 'bt_navigator'),
            ('nav2_behaviors', 'behavior_server')):
        extra = {}
        if executable == 'bt_navigator':
            extra = {
                'default_nav_to_pose_bt_xml': str(
                    share / 'behavior_trees' / 'option20_navigate_to_pose.xml'),
                'default_nav_through_poses_bt_xml': str(
                    share / 'behavior_trees' / 'option20_navigate_through_poses.xml')}
        nodes.append(Node(package=package, executable=executable, name=executable,
                          output='screen', parameters=[params, common, extra],
                          remappings=[('cmd_vel', '/option20/cmd_vel_nav')]))
    # Separate managers allow localization to become active before the navigation
    # servers wait for map->odom, which intentionally needs an initial pose.
    for name, lifecycle_nodes in (
            ('localization', ['map_server', 'localization_map_server', 'amcl']),
            ('navigation', ['planner_server', 'controller_server',
                            'behavior_server', 'bt_navigator'])):
        nodes.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                          name=f'option20_lifecycle_{name}', output='screen',
                          parameters=[common, {'autostart': True,
                                               # This installed Humble bond/SMC
                                               # implementation intermittently
                                               # throws during bond formation.
                                               # Data/command watchdogs and the
                                               # chassis EStop enforce stopping.
                                               'bond_timeout': 0.0,
                                               'node_names': lifecycle_nodes}]))
    nodes.append(Node(package='rviz2', executable='rviz2', name='option20_rviz',
                      arguments=['-d', str(share / 'rviz' / 'option20.rviz')],
                      parameters=[common],
                      condition=IfCondition(LaunchConfiguration('launch_rviz'))))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_directory', description=(
            'Directory generated by scripts/option20_prepare_map.py.')),
        DeclareLaunchArgument('bringup_hardware', default_value='false'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=launch_nodes),
    ])
