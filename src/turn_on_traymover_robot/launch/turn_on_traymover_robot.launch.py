import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('turn_on_traymover_robot'), 'config')
    config_file = os.path.join(config_dir, 'traymover_robot.yaml')

    return LaunchDescription([
        Node(
            package='turn_on_traymover_robot',
            executable='traymover_robot_node',
            name='turn_on_traymover_robot',
            parameters=[config_file],
            output='screen',
        ),
    ])
