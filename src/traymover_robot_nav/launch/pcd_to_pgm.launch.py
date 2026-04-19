"""Offline launch: convert a FAST_LIO PCD into Nav2 2D occupancy grid.

Invokes scripts/pcd2pgm.py (installed under lib/traymover_robot_nav/).

Example:
    ros2 launch traymover_robot_nav pcd_to_pgm.launch.py \\
        pcd:=/abs/path/to/input.pcd out:=/abs/path/prefix
"""
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

DEFAULT_PCD = ('/home/wheeltec/traymover_ros2/src/traymover_robot_slam/'
               'FAST_LIO/PCD/traymover_20260418_183556.pcd')


def generate_launch_description():
    pkg_prefix = get_package_prefix('traymover_robot_nav')
    script = os.path.join(pkg_prefix, 'lib', 'traymover_robot_nav', 'pcd2pgm.py')

    pkg_share = get_package_share_directory('traymover_robot_nav')
    default_out = os.path.join(pkg_share, 'map', 'traymover_2d')

    args = [
        DeclareLaunchArgument('pcd', default_value=DEFAULT_PCD),
        DeclareLaunchArgument('out', default_value=default_out),
        DeclareLaunchArgument('resolution', default_value='0.05'),
        DeclareLaunchArgument('z_min', default_value='0.15'),
        DeclareLaunchArgument('z_max', default_value='1.5'),
        DeclareLaunchArgument('dilate', default_value='1'),
    ]

    proc = ExecuteProcess(
        cmd=[
            'python3', script,
            '--pcd', LaunchConfiguration('pcd'),
            '--out', LaunchConfiguration('out'),
            '--resolution', LaunchConfiguration('resolution'),
            '--z-min', LaunchConfiguration('z_min'),
            '--z-max', LaunchConfiguration('z_max'),
            '--dilate', LaunchConfiguration('dilate'),
        ],
        output='screen',
    )

    return LaunchDescription(args + [proc])
