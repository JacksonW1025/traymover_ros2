from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = Path(get_package_share_directory('traymover_nav2')).resolve()
    workspace_dir = share_dir.parents[3]

    install_map_prefix = share_dir / 'map' / 'TRAYMOVER'
    source_map_prefix = workspace_dir / 'src' / 'traymover_robot_nav2' / 'map' / 'TRAYMOVER'

    use_sim_time = LaunchConfiguration('use_sim_time')

    common_parameters = [
        {'use_sim_time': use_sim_time},
        {'save_map_timeout': 20.0},
        {'free_thresh_default': 0.25},
        {'occupied_thresh_default': 0.65},
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true.',
        ),
        Node(
            package='nav2_map_server',
            executable='map_saver_cli',
            name='install_map_saver',
            output='screen',
            arguments=['-f', str(install_map_prefix)],
            parameters=common_parameters,
        ),
        Node(
            package='nav2_map_server',
            executable='map_saver_cli',
            name='source_map_saver',
            output='screen',
            arguments=['-f', str(source_map_prefix)],
            parameters=common_parameters,
        ),
    ])
