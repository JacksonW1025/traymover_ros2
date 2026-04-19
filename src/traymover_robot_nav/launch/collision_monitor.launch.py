"""Launch: Phase 1 obstacle-stop safety net.

Wraps nav2_collision_monitor. Expects:
  - /scan already published (from pointcloud_to_laserscan, started by
    lidar_localization.launch.py)
  - A cmd_vel_nav publisher upstream (e.g. Nav2 controller_server with
    its controller_frequency output remapped to /cmd_vel_nav)

Output:
  - /cmd_vel (consumed by turn_on_traymover_robot serial driver)
  - /polygon_stop, /polygon_slowdown (for RViz visualization)

Note: keyboard teleop is NOT routed through this monitor. It publishes
/cmd_vel directly. The monitor only filters Nav2 autonomous output.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('traymover_robot_nav')
    default_params = os.path.join(pkg_share, 'config', 'collision_monitor.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    lifecycle_mgr = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_collision_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['collision_monitor'],
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        collision_monitor,
        lifecycle_mgr,
    ])
