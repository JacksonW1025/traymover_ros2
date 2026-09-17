"""Option 19 sensors and mandatory safety layers; the guided runner stays in a TTY.

No localization or navigation stack is included. All scan transforms are relative
to the existing robot model's base_link, also used as the Collision Monitor fixed
frame because this test does not have measured odometry. Initial test values have
not been validated on hardware.
"""
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    nav_share = get_package_share_directory('traymover_robot_nav')
    bringup_share = get_package_share_directory('turn_on_traymover_robot')
    safety_share = get_package_share_directory('traymover_robot_safety')
    description_share = get_package_share_directory('traymover_robot_description')
    camera_share = get_package_share_directory('realsense2_camera')

    def include(share, filename, arguments):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', filename)),
            launch_arguments=arguments.items())

    robot_description = ParameterValue(Command([
        'xacro', ' ', os.path.join(description_share, 'urdf', 'traymover.urdf.xacro'),
    ]), value_type=str)
    model_remappings = [
        ('joint_states', '/traymover/joint_states'),
        ('robot_description', '/traymover/robot_description'),
    ]

    actions = [
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        LogInfo(msg='Automatic EStop: REQUIRED / ENABLED'),
        LogInfo(msg='Collision Monitor: REQUIRED / ENABLED'),
        LogInfo(msg='Run guided_detour_test.py in the foreground; no motion is armed by launch.'),
        include(bringup_share, 'base_serial.launch.py', {
            'use_imu': 'true', 'odom_source_mode': 'none', 'auto_estop_enabled': 'true',
        }),
        include(bringup_share, 'traymover_lidar.launch.py', {
            'enable_scan_bridge': 'false',
        }),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='traymover_robot_state_publisher', output='screen',
            parameters=[{'robot_description': robot_description}],
            remappings=model_remappings,
        ),
        Node(
            package='joint_state_publisher', executable='joint_state_publisher',
            name='traymover_joint_state_publisher', output='screen',
            parameters=[{'robot_description': robot_description}],
            remappings=model_remappings,
        ),
        GroupAction(
            scoped=True, forwarding=False,
            launch_configurations={
                'enable_depth': 'true', 'enable_color': 'false',
                'depth_module.depth_profile': '640x480x15',
                'align_depth.enable': 'false', 'pointcloud.enable': 'false',
                'enable_sync': 'false',
            },
            actions=[include(camera_share, 'rs_launch.py', {})],
        ),
        GroupAction(scoped=True, actions=[include(
            safety_share, 'traymover_auto_estop.launch.py', {
                'start_realsense_driver': 'false',
                'depth_topic': '/camera/camera/depth/image_rect_raw',
            })]),
        Node(
            package='depthimage_to_laserscan', executable='depthimage_to_laserscan_node',
            name='traymover_realsense_depth_to_laserscan', output='screen',
            remappings=[
                ('depth', '/camera/camera/depth/image_rect_raw'),
                ('depth_camera_info', '/camera/camera/depth/camera_info'),
                ('scan', '/scan_realsense'),
            ],
            parameters=[{
                'scan_height': 10, 'range_min': 0.20, 'range_max': 3.0,
                'output_frame': 'camera_depth_frame',
            }],
        ),
        include(nav_share, 'collision_monitor.launch.py', {
            'use_sim_time': 'false',
            'params_file': os.path.join(
                nav_share, 'config', 'collision_monitor_guided_detour_test.yaml'),
        }),
        Node(
            package='rviz2', executable='rviz2', name='guided_detour_rviz',
            arguments=['-d', os.path.join(nav_share, 'rviz', 'guided_detour_test.rviz')],
            condition=IfCondition(LaunchConfiguration('launch_rviz')),
        ),
    ]
    # Match the production full-perimeter projection. Finite observed returns,
    # never unknown/no-return bins, certify rotation space in the shared core.
    # Keep both public topics so the production and test adapters use one contract.
    for name, topic in [('traymover_guided_scan', '/scan'),
                        ('traymover_detour_perimeter_scan', '/scan_detour_perimeter')]:
        actions.append(Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            name=name, output='screen',
            remappings=[('cloud_in', '/point_cloud_raw'), ('scan', topic)],
            parameters=[{
                'target_frame': 'base_link', 'transform_tolerance': 0.05,
                'min_height': -0.25, 'max_height': 1.05,
                'angle_min': -math.pi, 'angle_max': math.pi,
                'angle_increment': 0.0087, 'scan_time': 0.1,
                'range_min': 0.30, 'range_max': 60.0,
                'use_inf': True, 'inf_epsilon': 1.0,
            }],
        ))
    return LaunchDescription(actions)
