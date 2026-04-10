import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


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
            'Traymover gmapping bringup requires STM32 odom feedback and /odom_combined. '
            f'Current odom_source_mode is {odom_source_mode!r}; N300pro IMU-only odom is not '
            'supported. Validate the STM32 feedback path first, then set odom_source_mode '
            'to "stm32_feedback".'
        )
    return []


def generate_launch_description():
    bringup_dir = get_package_share_directory('turn_on_traymover_robot')
    slam_dir = get_package_share_directory('slam_gmapping')

    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_config_file = LaunchConfiguration('robot_config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true.',
        ),
        DeclareLaunchArgument(
            'robot_config_file',
            default_value=os.path.join(bringup_dir, 'config', 'traymover_robot.yaml'),
            description='Traymover base-driver configuration used to validate gmapping odom support.',
        ),
        OpaqueFunction(function=_validate_gmapping_preconditions),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'turn_on_traymover_robot.launch.py')),
            launch_arguments={
                'use_description': 'true',
                'use_rviz': 'false',
                'use_imu': 'true',
                'use_lidar': 'true',
                'use_ekf': 'true',
                'use_sim_time': use_sim_time,
                'robot_config': robot_config_file,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_dir, 'launch', 'slam_gmapping.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'robot_config_file': robot_config_file,
            }.items(),
        ),
    ])
