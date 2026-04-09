import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_dir = get_package_share_directory('traymover_robot_description')
    launch_dir = os.path.join(package_dir, 'launch')
    default_model_path = os.path.join(package_dir, 'urdf', 'traymover.urdf.xacro')
    default_rviz_path = os.path.join(package_dir, 'rviz', 'traymover.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    model = LaunchConfiguration('model')
    rviz_config = LaunchConfiguration('rviz_config')
    imu_topic = LaunchConfiguration('imu_topic')

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
            'imu_topic',
            default_value='/imu/data_raw',
            description='IMU topic used to orient the base_link frame.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'description.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'model': model,
                'imu_topic': imu_topic,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'rviz.launch.py')
            ),
            launch_arguments={
                'rviz_config': rviz_config,
            }.items(),
        ),
    ])
