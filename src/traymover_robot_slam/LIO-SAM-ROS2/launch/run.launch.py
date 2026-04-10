import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('lio_sam')
    parameter_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    common_parameters = [parameter_file, {'use_sim_time': use_sim_time}]

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(share_dir, 'config', 'traymover_params.yaml'),
            description='Path to the LIO-SAM parameter file.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true.',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments='0.0 0.0 0.0 0.0 0.0 0.0 1.0 map odom'.split(' '),
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_imuPreintegration',
            name='lio_sam_imuPreintegration',
            parameters=common_parameters,
            output='screen',
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_imageProjection',
            name='lio_sam_imageProjection',
            parameters=common_parameters,
            output='screen',
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_featureExtraction',
            name='lio_sam_featureExtraction',
            parameters=common_parameters,
            output='screen',
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_mapOptimization',
            name='lio_sam_mapOptimization',
            parameters=common_parameters,
            output='screen',
        ),
    ])
