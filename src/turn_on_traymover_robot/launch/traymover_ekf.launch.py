from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = Path(
        get_package_share_directory('turn_on_traymover_robot'),
        'config',
        'ekf.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'ekf_config',
            default_value=str(default_config),
            description='EKF configuration file.',
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='traymover_ekf_filter_node',
            parameters=[LaunchConfiguration('ekf_config')],
            remappings=[('/odometry/filtered', 'odom_combined')],
            output='screen',
        ),
    ])
