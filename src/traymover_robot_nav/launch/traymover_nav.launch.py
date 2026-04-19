"""Launch: Phase 2 full traymover autonomous navigation.

Brings up the full runtime chain:
  0. (optional) hardware bringup — base_serial + traymover_lidar
     (set bringup_hardware:=true for one-command start; default false).
  1. lidar_localization.launch.py — robot_state_publisher +
     lidar_localization_ros2 (NDT_OMP, loads prebuilt PCD, publishes map -> base_link).
  2. pose_to_odom bridge — converts /pcl_pose (PoseStamped) into /odom (Odometry)
     so Nav2 controller_server can read current twist.
  3. nav2 navigation stack — map_server, planner_server, controller_server,
     bt_navigator, behavior_server, waypoint_follower, lifecycle_manager.
     controller + behavior cmd_vel is remapped to /cmd_vel_nav.
  4. collision_monitor.launch.py — filters /cmd_vel_nav -> /cmd_vel as the
     obstacle-stop safety net.
  5. (optional) RViz — set launch_rviz:=true.

Common launch invocations:
  # Preferred for real-hardware test (single command, one window):
  ros2 launch traymover_robot_nav traymover_nav.launch.py \\
       bringup_hardware:=true launch_rviz:=true

  # Multi-terminal for debugging (this launch only; run hardware separately):
  # T1: ros2 launch turn_on_traymover_robot base_serial.launch.py
  # T2: ros2 launch turn_on_traymover_robot traymover_lidar.launch.py
  # T3: ros2 launch traymover_robot_nav traymover_nav.launch.py launch_rviz:=true

After startup, use RViz "2D Pose Estimate" to set initial pose, then "2D Goal
Pose" to send a navigation goal.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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
    'behavior_server',
    'bt_navigator',
    'waypoint_follower',
]

# Keep in sync with default_pcd in lidar_localization.launch.py and
# map_path in config/localization.yaml.
DEFAULT_PCD = ('/home/wheeltec/traymover_ros2/src/traymover_robot_slam/'
               'FAST_LIO/PCD/traymover_20260418_183556.pcd')


def generate_launch_description():
    nav_share = get_package_share_directory('traymover_robot_nav')
    bringup_share = get_package_share_directory('turn_on_traymover_robot')

    default_params = os.path.join(nav_share, 'config', 'nav2_params.yaml')
    collision_params = os.path.join(nav_share, 'config', 'collision_monitor.yaml')
    default_map = os.path.join(nav_share, 'map', 'traymover_2d.yaml')
    default_rviz = os.path.join(nav_share, 'rviz', 'traymover_nav.rviz')

    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    bringup_hardware = LaunchConfiguration('bringup_hardware')
    launch_rviz = LaunchConfiguration('launch_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    pcd_path = LaunchConfiguration('pcd_path')

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites={
                'yaml_filename': map_yaml,
                'use_sim_time': use_sim_time,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    # 0) Optional hardware bringup (base_serial + traymover_lidar)
    hw_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'base_serial.launch.py')),
        condition=IfCondition(bringup_hardware),
    )
    hw_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'traymover_lidar.launch.py')),
        condition=IfCondition(bringup_hardware),
    )

    # 1) Localization chain (RSP + lidar_localization_ros2)
    #    /scan is expected from the hardware bringup's pointcloud_to_laserscan.
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, 'launch', 'lidar_localization.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'pcd_path': pcd_path,
        }.items(),
    )

    # 2) Pose -> Odom bridge
    pose_to_odom = Node(
        package='traymover_robot_nav',
        executable='pose_to_odom.py',
        name='pose_to_odom',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # 3) Nav2 lifecycle nodes. controller_server cmd_vel -> /cmd_vel_nav
    #    so collision_monitor can filter before reaching the serial driver.
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
        remappings=[('cmd_vel', 'cmd_vel_nav')],
    )
    behaviors = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[configured_params],
        remappings=[('cmd_vel', 'cmd_vel_nav')],
    )
    bt = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_params],
    )
    waypoint = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
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

    # 4) Obstacle-stop safety net.
    # Explicitly pin params_file so the outer launch's same-named
    # LaunchConfiguration doesn't leak nav2_params.yaml into this sub-launch.
    collision_monitor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, 'launch', 'collision_monitor.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': collision_params,
        }.items(),
    )

    # 5) Optional RViz
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
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument(
            'bringup_hardware', default_value='false',
            description='Also start base_serial + traymover_lidar (true for one-command real-hardware test).'),
        DeclareLaunchArgument(
            'launch_rviz', default_value='false',
            description='Also start RViz with the navigation profile.'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),
        DeclareLaunchArgument(
            'pcd_path', default_value=DEFAULT_PCD,
            description='Prior PCD map loaded by lidar_localization_ros2.'),
        hw_base,
        hw_lidar,
        localization,
        pose_to_odom,
        map_server,
        planner,
        controller,
        behaviors,
        bt,
        waypoint,
        lifecycle_mgr,
        collision_monitor,
        rviz,
    ])
