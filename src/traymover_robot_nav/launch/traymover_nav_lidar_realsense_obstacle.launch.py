"""Launch: option 16 LiDAR + RealSense dynamic obstacle navigation.

This is an additive variant of option 10. It keeps the same FAST-LIO ->
lidar_localization -> PCD-derived 2D map -> Nav2 baseline, then adds only the
thin safety inputs needed for dynamic obstacle handling:

  * high-mounted LiDAR point cloud -> /scan for farther / upper-body obstacles
  * waist-height RealSense depth -> /scan_realsense for nearby low obstacles
  * Nav2 local_costmap obstacle_layer consumes both scans
  * nav2_collision_monitor filters /cmd_vel_nav into the final /cmd_vel

The LiDAR is useful for walls and adult upper bodies, but it is too high to be
the only obstacle source for boxes, stools, cables, children, or low carts.
RealSense depth complements that blind spot at short range. This launch does
not do recognition, clustering, tracking, prediction, or sensor fusion; the
scans are only obstacle sources for the local costmap and collision monitor.
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


NAV2_LIFECYCLE_NODES = [
    'map_server',
    'planner_server',
    'controller_server',
    'bt_navigator',
]

DEFAULT_PCD = ('/home/wheeltec/traymover_ros2/src/traymover_robot_slam/'
               'FAST_LIO/PCD/traymover.pcd')


def find_default_pcd() -> str:
    pcd_dir = Path('/home/wheeltec/traymover_ros2/src/traymover_robot_slam/FAST_LIO/PCD')
    candidates = sorted(pcd_dir.glob('*.pcd'), key=lambda p: p.stat().st_mtime, reverse=True)
    if candidates:
        return str(candidates[0])
    return DEFAULT_PCD


def generate_launch_description():
    nav_share = get_package_share_directory('traymover_robot_nav')
    bringup_share = get_package_share_directory('turn_on_traymover_robot')

    default_params = os.path.join(
        nav_share, 'config', 'nav2_params_lidar_realsense_obstacle.yaml')
    default_collision_params = os.path.join(
        nav_share, 'config', 'collision_monitor_lidar_realsense.yaml')
    default_map = os.path.join(nav_share, 'map', 'traymover_2d.yaml')
    default_rviz = os.path.join(nav_share, 'rviz', 'traymover_nav.rviz')
    default_bt = os.path.join(nav_share, 'behavior_trees', 'simple_navigate_to_pose.xml')
    default_through_poses_bt = os.path.join(
        nav_share, 'behavior_trees', 'simple_navigate_through_poses.xml')
    pointcloud_launch = os.path.join(nav_share, 'launch', 'navigation_pointcloud.launch.py')
    collision_launch = os.path.join(nav_share, 'launch', 'collision_monitor.launch.py')

    params_file = LaunchConfiguration('params_file')
    collision_params_file = LaunchConfiguration('collision_params_file')
    map_yaml = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    bringup_hardware = LaunchConfiguration('bringup_hardware')
    launch_rviz = LaunchConfiguration('launch_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    pcd_path = LaunchConfiguration('pcd_path')
    bt_xml = LaunchConfiguration('bt_xml')
    through_poses_bt_xml = LaunchConfiguration('through_poses_bt_xml')
    use_realsense = LaunchConfiguration('use_realsense')
    start_realsense_driver = LaunchConfiguration('start_realsense_driver')
    realsense_depth_topic = LaunchConfiguration('realsense_depth_topic')
    realsense_info_topic = LaunchConfiguration('realsense_info_topic')
    realsense_scan_topic = LaunchConfiguration('realsense_scan_topic')
    realsense_output_frame = LaunchConfiguration('realsense_output_frame')

    def maybe_start_realsense_driver(context, *args, **kwargs):
        enabled = start_realsense_driver.perform(context).lower() in (
            'true', '1', 'yes', 'on')
        if not enabled:
            return []

        realsense_share = get_package_share_directory('realsense2_camera')
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(realsense_share, 'launch', 'rs_launch.py')),
                launch_arguments={
                    'enable_depth': 'true',
                    'enable_color': 'false',
                    'align_depth.enable': 'false',
                    'pointcloud.enable': 'false',
                }.items(),
            )
        ]

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites={
                'yaml_filename': map_yaml,
                'use_sim_time': use_sim_time,
                'default_nav_to_pose_bt_xml': bt_xml,
                'default_nav_through_poses_bt_xml': through_poses_bt_xml,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    hw_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'base_serial.launch.py')),
        condition=IfCondition(bringup_hardware),
        launch_arguments={
            'odom_source_mode': 'none',
        }.items(),
    )
    hw_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'traymover_lidar.launch.py')),
        condition=IfCondition(bringup_hardware),
        launch_arguments={
            'enable_scan_bridge': 'false',
        }.items(),
    )
    pointcloud_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pointcloud_launch),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'input_cloud_topic': '/point_cloud_raw',
            'localization_cloud_topic': '/point_cloud_localization',
            'nav_cloud_topic': '/point_cloud_nav',
            'scan_topic': '/scan',
            'target_frame': 'base_link',
            'publish_scan': 'true',
        }.items(),
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, 'launch', 'lidar_localization.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'pcd_path': pcd_path,
            'cloud_topic': '/point_cloud_localization',
            'max_map_odom_update_translation': '0.50',
            'max_map_odom_update_rotation': '0.25',
        }.items(),
    )

    realsense_scan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='traymover_realsense_depth_to_laserscan',
        remappings=[
            ('depth', realsense_depth_topic),
            ('depth_camera_info', realsense_info_topic),
            ('scan', realsense_scan_topic),
        ],
        parameters=[{
            'scan_height': 10,
            'range_min': 0.20,
            'range_max': 3.0,
            # Default follows the depthimage_to_laserscan package convention.
            # Override only after confirming the RealSense TF tree on the robot.
            'output_frame': realsense_output_frame,
            'use_sim_time': use_sim_time,
        }],
        condition=IfCondition(use_realsense),
        output='screen',
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[configured_params],
    )
    planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_params],
    )
    controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[configured_params],
        remappings=[
            ('cmd_vel', '/cmd_vel_nav'),
        ],
    )
    bt = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_params],
    )
    lifecycle_mgr = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': NAV2_LIFECYCLE_NODES,
        }],
    )

    collision_monitor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(collision_launch),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': collision_params_file,
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('collision_params_file', default_value=default_collision_params),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument(
            'bringup_hardware', default_value='false',
            description='Also start base_serial + traymover_lidar.'),
        DeclareLaunchArgument(
            'launch_rviz', default_value='false',
            description='Also start RViz with the navigation profile.'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),
        DeclareLaunchArgument(
            'bt_xml', default_value=default_bt,
            description='Behavior tree used for option-16 point-to-point navigation.'),
        DeclareLaunchArgument(
            'through_poses_bt_xml', default_value=default_through_poses_bt,
            description='Recovery-free through-poses tree loaded by bt_navigator.'),
        DeclareLaunchArgument(
            'pcd_path', default_value=find_default_pcd(),
            description='Prior PCD map loaded by lidar_localization_ros2.'),
        DeclareLaunchArgument(
            'use_realsense', default_value='true',
            description='Start depthimage_to_laserscan for RealSense depth obstacle input.'),
        DeclareLaunchArgument(
            'start_realsense_driver', default_value='false',
            description='Optionally include realsense2_camera rs_launch.py if installed.'),
        DeclareLaunchArgument(
            'realsense_depth_topic',
            default_value='/camera/camera/depth/image_rect_raw',
            description='RealSense depth image topic consumed by depthimage_to_laserscan.'),
        DeclareLaunchArgument(
            'realsense_info_topic',
            default_value='/camera/camera/depth/camera_info',
            description='RealSense depth camera_info topic consumed by depthimage_to_laserscan.'),
        DeclareLaunchArgument(
            'realsense_scan_topic',
            default_value='/scan_realsense',
            description='LaserScan topic published from RealSense depth.'),
        DeclareLaunchArgument(
            'realsense_output_frame',
            default_value='camera_depth_frame',
            description='LaserScan frame; override after checking RealSense TF on the robot.'),
        hw_base,
        hw_lidar,
        pointcloud_pipeline,
        localization,
        OpaqueFunction(function=maybe_start_realsense_driver),
        realsense_scan,
        map_server,
        planner,
        controller,
        bt,
        lifecycle_mgr,
        collision_monitor,
        rviz,
    ])
