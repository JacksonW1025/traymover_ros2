import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_dir = get_package_share_directory('traymover_robot_description')
    default_model_path = os.path.join(package_dir, 'urdf', 'traymover.urdf.xacro')

    use_sim_time = LaunchConfiguration('use_sim_time')
    model = LaunchConfiguration('model')
    imu_topic = LaunchConfiguration('imu_topic')

    robot_description = ParameterValue(
        Command(['xacro', ' ', model]),
        value_type=str,
    )

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
            'imu_topic',
            default_value='/imu/data_raw',
            description='IMU topic used to orient the base_link frame.',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='traymover_robot_state_publisher',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'robot_description': robot_description},
            ],
            remappings=[
                ('joint_states', '/traymover/joint_states'),
                ('robot_description', '/traymover/robot_description'),
            ],
        ),
        Node(
            package='traymover_robot_description',
            executable='imu_pose_broadcaster.py',
            name='imu_pose_broadcaster',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'imu_topic': imu_topic},
                {'parent_frame': 'base_footprint'},
                {'child_frame': 'base_link'},
                {'translation_x': 0.0},
                {'translation_y': 0.0},
                {'translation_z': 0.20},
                {'publish_rate': 30.0},
            ],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='traymover_joint_state_publisher',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'robot_description': robot_description},
            ],
            remappings=[
                ('joint_states', '/traymover/joint_states'),
                ('robot_description', '/traymover/robot_description'),
            ],
        ),
    ])
