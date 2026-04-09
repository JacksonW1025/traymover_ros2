import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('turn_on_traymover_robot')
    config_dir = os.path.join(bringup_dir, 'config')
    description_launch_dir = os.path.join(
        get_package_share_directory('traymover_robot_description'), 'launch')
    bringup_launch_dir = os.path.join(bringup_dir, 'launch')

    use_description = LaunchConfiguration('use_description')
    use_rviz = LaunchConfiguration('use_rviz')
    use_imu = LaunchConfiguration('use_imu')
    use_lidar = LaunchConfiguration('use_lidar')
    use_ekf = LaunchConfiguration('use_ekf')
    use_sim_time = LaunchConfiguration('use_sim_time')
    imu_topic = LaunchConfiguration('imu_topic')
    imu_serial_port = LaunchConfiguration('imu_serial_port')
    imu_baud_rate = LaunchConfiguration('imu_baud_rate')
    imu_frame_id = LaunchConfiguration('imu_frame_id')
    imu_mode = LaunchConfiguration('imu_mode')
    lidar_type = LaunchConfiguration('lidar_type')
    traymover_param_yaml = LaunchConfiguration('traymover_param_yaml')
    robot_config = LaunchConfiguration('robot_config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_description',
            default_value='true',
            description='Launch robot_state_publisher and joint_state_publisher.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz with the Traymover model configuration.',
        ),
        DeclareLaunchArgument(
            'use_imu',
            default_value='true',
            description='Launch the N300 Pro IMU driver.',
        ),
        DeclareLaunchArgument(
            'use_lidar',
            default_value='false',
            description='Launch the lidar driver for raw point cloud output.',
        ),
        DeclareLaunchArgument(
            'use_ekf',
            default_value='false',
            description='Launch robot_localization EKF for /odom_combined.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true.',
        ),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/imu/data_raw',
            description='IMU topic used to orient the Traymover base in RViz.',
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
            'imu_mode',
            default_value='',
            description='Override IMU mode from traymover_param.yaml.',
        ),
        DeclareLaunchArgument(
            'lidar_type',
            default_value='',
            description='Override lidar type from traymover_param.yaml.',
        ),
        DeclareLaunchArgument(
            'traymover_param_yaml',
            default_value=os.path.join(config_dir, 'traymover_param.yaml'),
            description='Traymover shared bringup parameter file.',
        ),
        DeclareLaunchArgument(
            'robot_config',
            default_value=os.path.join(config_dir, 'traymover_robot.yaml'),
            description='Traymover base driver parameter file.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(description_launch_dir, 'description.launch.py')),
            condition=IfCondition(use_description),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'imu_topic': imu_topic,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(description_launch_dir, 'rviz.launch.py')),
            condition=IfCondition(use_rviz),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_launch_dir, 'base_serial.launch.py')),
            launch_arguments={
                'use_imu': use_imu,
                'imu_mode': imu_mode,
                'imu_topic': imu_topic,
                'imu_serial_port': imu_serial_port,
                'imu_baud_rate': imu_baud_rate,
                'imu_frame_id': imu_frame_id,
                'traymover_param_yaml': traymover_param_yaml,
                'robot_config': robot_config,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_launch_dir, 'traymover_lidar.launch.py')),
            condition=IfCondition(use_lidar),
            launch_arguments={
                'traymover_param_yaml': traymover_param_yaml,
                'lidar_type': lidar_type,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_launch_dir, 'traymover_ekf.launch.py')),
            condition=IfCondition(use_ekf),
        ),
    ])
