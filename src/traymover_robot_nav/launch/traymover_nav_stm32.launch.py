"""Option 21: option-10 NDT/Nav2 with STM32 velocity odometry, no FAST-LIO."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    nav = get_package_share_directory('traymover_robot_nav')
    base = get_package_share_directory('turn_on_traymover_robot')
    defaults = {
        'use_sim_time': 'false', 'bringup_hardware': 'false', 'launch_rviz': 'false',
        'use_imu': 'false',
        'map': os.path.join(nav, 'map', 'traymover_2d.yaml'),
        'params_file': os.path.join(nav, 'config', 'nav2_params.yaml'),
        'height_boost_enabled': 'false', 'height_boost_min_z': '2.0',
        'auto_correction_enabled': 'true',
        'robot_config': os.path.join(base, 'config', 'stm32_navigation.yaml'),
        'imu_serial_port': '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A6C071391-if00',
    }
    # The PCD can come from any compatible mapper (including LVI-SAM).
    arguments = {key: LaunchConfiguration(key) for key in defaults}
    arguments['pcd_path'] = LaunchConfiguration('pcd_path')
    arguments['odometry_source'] = 'stm32'
    return LaunchDescription([
        DeclareLaunchArgument('pcd_path', description='Existing prior PCD map (absolute path).'),
        *[DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()],
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(nav, 'launch', 'traymover_nav.launch.py')),
            launch_arguments=arguments.items()),
    ])
