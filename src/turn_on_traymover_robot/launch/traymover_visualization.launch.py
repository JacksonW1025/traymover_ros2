import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_dir = get_package_share_directory('turn_on_traymover_robot')
    description_dir = get_package_share_directory('traymover_robot_description')
    default_model_path = os.path.join(description_dir, 'urdf', 'traymover.urdf.xacro')
    default_rviz_path = os.path.join(description_dir, 'rviz', 'traymover.rviz')
    default_param_path = os.path.join(bringup_dir, 'config', 'traymover_param.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    model = LaunchConfiguration('model')
    rviz_config = LaunchConfiguration('rviz_config')
    traymover_param_yaml = LaunchConfiguration('traymover_param_yaml')
    imu_topic = LaunchConfiguration('imu_topic')
    imu_mode = LaunchConfiguration('imu_mode')
    imu_serial_port = LaunchConfiguration('imu_serial_port')
    imu_baud_rate = LaunchConfiguration('imu_baud_rate')
    imu_frame_id = LaunchConfiguration('imu_frame_id')
    lidar_type = LaunchConfiguration('lidar_type')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true.',
        ),
        DeclareLaunchArgument(
            'model',
            default_value=default_model_path,
            description='Absolute path to the Traymover xacro model.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_path,
            description='Absolute path to the RViz configuration file.',
        ),
        DeclareLaunchArgument(
            'traymover_param_yaml',
            default_value=default_param_path,
            description='Shared Traymover bringup parameter file.',
        ),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/imu/data_raw',
            description='IMU topic used by both the driver and RViz display.',
        ),
        DeclareLaunchArgument(
            'imu_mode',
            default_value='',
            description='Override imu_mode from traymover_param.yaml.',
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
        DeclareLaunchArgument(
            'lidar_type',
            default_value='',
            description='Override lidar_type from traymover_param.yaml.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'traymover_imu.launch.py')
            ),
            launch_arguments={
                'traymover_param_yaml': traymover_param_yaml,
                'imu_mode': imu_mode,
                'imu_topic': imu_topic,
                'imu_serial_port': imu_serial_port,
                'imu_baud_rate': imu_baud_rate,
                'imu_frame_id': imu_frame_id,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'traymover_lidar.launch.py')
            ),
            launch_arguments={
                'traymover_param_yaml': traymover_param_yaml,
                'lidar_type': lidar_type,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(description_dir, 'launch', 'display.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'model': model,
                'rviz_config': rviz_config,
                'imu_topic': imu_topic,
            }.items(),
        ),
    ])
