"""Option 19 hardware: chassis odometry and one forward RealSense depth scan."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('turn_on_traymover_robot')
    camera_share = get_package_share_directory('realsense2_camera')

    def include(share, filename, arguments):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', filename)),
            launch_arguments=arguments.items())

    return LaunchDescription([
        LogInfo(msg='Option 19: fixed right detour from RealSense depth.'),
        include(bringup_share, 'base_serial.launch.py', {
            'use_imu': 'false', 'odom_source_mode': 'stm32_feedback',
            'auto_estop_enabled': 'false',
        }),
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
        Node(
            package='depthimage_to_laserscan', executable='depthimage_to_laserscan_node',
            name='traymover_option19_depth_scan', output='screen',
            remappings=[
                ('depth', '/camera/camera/depth/image_rect_raw'),
                ('depth_camera_info', '/camera/camera/depth/camera_info'),
                ('scan', '/scan_realsense'),
            ],
            parameters=[{
                'scan_height': 20, 'range_min': 0.20, 'range_max': 10.0,
                'output_frame': 'camera_depth_frame',
            }],
        ),
    ])
