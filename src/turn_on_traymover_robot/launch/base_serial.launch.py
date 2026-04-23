import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_yaml(file_path: str) -> dict:
    return yaml.safe_load(Path(file_path).read_text())


def include_base_launch(context, *args, **kwargs):
    cfg_path = LaunchConfiguration('traymover_param_yaml').perform(context)
    cfg = load_yaml(cfg_path)
    imu_mode = LaunchConfiguration('imu_mode').perform(context) or cfg['imu_mode']
    odom_source_mode = LaunchConfiguration('odom_source_mode').perform(context)
    use_imu = LaunchConfiguration('use_imu').perform(context).lower() == 'true'
    robot_params = [LaunchConfiguration('robot_config')]
    if odom_source_mode:
        robot_params.append({'odom_source_mode': odom_source_mode})
    nodes = [
        Node(
            package='turn_on_traymover_robot',
            executable='traymover_robot_node',
            name='turn_on_traymover_robot',
            output='screen',
            parameters=robot_params,
        )
    ]

    if use_imu and imu_mode == 'N300_pro':
        nodes.append(
            Node(
                package='hipnuc_imu',
                executable='talker',
                name='traymover_imu_publisher',
                output='screen',
                parameters=[{
                    'serial_port': LaunchConfiguration('imu_serial_port'),
                    'baud_rate': LaunchConfiguration('imu_baud_rate'),
                    'frame_id': LaunchConfiguration('imu_frame_id'),
                    'imu_topic': LaunchConfiguration('imu_topic'),
                }],
            )
        )
    elif use_imu and imu_mode != 'stm32':
        raise ValueError(f'Unsupported imu_mode: {imu_mode}')

    return nodes


def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('turn_on_traymover_robot'),
        'config',
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'traymover_param_yaml',
            default_value=os.path.join(config_dir, 'traymover_param.yaml'),
            description='Shared Traymover bringup parameter file.',
        ),
        DeclareLaunchArgument(
            'robot_config',
            default_value=os.path.join(config_dir, 'traymover_robot.yaml'),
            description='Traymover robot driver parameter file.',
        ),
        DeclareLaunchArgument(
            'use_imu',
            default_value='true',
            description='Launch the external N300 Pro IMU driver.',
        ),
        DeclareLaunchArgument(
            'imu_mode',
            default_value='',
            description='Override imu_mode from traymover_param.yaml.',
        ),
        DeclareLaunchArgument(
            'odom_source_mode',
            default_value='',
            description='Override odom_source_mode from traymover_robot.yaml.',
        ),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/imu/data_raw',
            description='Topic used by the external IMU driver.',
        ),
        DeclareLaunchArgument(
            'imu_serial_port',
            default_value='/dev/ttyCH343USB0',
            description='Serial port used by the N300 Pro IMU.',
        ),
        DeclareLaunchArgument(
            'imu_baud_rate',
            default_value='115200',
            description='Serial baud rate used by the N300 Pro IMU.',
        ),
        DeclareLaunchArgument(
            'imu_frame_id',
            default_value='imu_link',
            description='Frame ID published in the IMU message header.',
        ),
        OpaqueFunction(function=include_base_launch),
    ])
