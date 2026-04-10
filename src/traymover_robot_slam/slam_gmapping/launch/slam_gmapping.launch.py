import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_robot_config(config_path: str) -> dict:
    return yaml.safe_load(Path(config_path).read_text())


def _validate_gmapping_preconditions(context, *args, **kwargs):
    config_path = LaunchConfiguration('robot_config_file').perform(context)
    if not os.path.exists(config_path):
        raise RuntimeError(
            f'gmapping precondition failed: robot config file does not exist: {config_path}'
        )

    config = _load_robot_config(config_path)
    params = config.get('turn_on_traymover_robot', {}).get('ros__parameters', {})
    odom_source_mode = params.get('odom_source_mode', 'none')
    if odom_source_mode != 'stm32_feedback':
        raise RuntimeError(
            'gmapping requires STM32 odom feedback and /odom_combined. '
            f'Current odom_source_mode is {odom_source_mode!r}, so N300pro IMU-only odom '
            'is not supported. Validate the STM32 feedback path first, then set '
            'odom_source_mode to "stm32_feedback".'
        )
    return []

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true.',
        ),
        DeclareLaunchArgument(
            'robot_config_file',
            default_value=os.path.join(
                get_package_share_directory('turn_on_traymover_robot'),
                'config',
                'traymover_robot.yaml',
            ),
            description='Traymover base-driver configuration used to validate gmapping odom support.',
        ),
        OpaqueFunction(function=_validate_gmapping_preconditions),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        Node(
            package='slam_gmapping',
            executable='slam_gmapping',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
    ])
