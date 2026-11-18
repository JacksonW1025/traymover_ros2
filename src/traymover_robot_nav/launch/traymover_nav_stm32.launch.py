"""Option 21: option-10 NDT/Nav2 with STM32 velocity odometry, no FAST-LIO."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    nav = get_package_share_directory('traymover_robot_nav')
    base = get_package_share_directory('turn_on_traymover_robot')
    # At 20 Hz, RPP's first turn command from rest is acceleration / 20.
    # The chassis sometimes stalled at 0.16 rad/s. Start at 0.22 rad/s
    # while capping the nominal in-place turn at 0.25 rad/s.
    follow_path = 'controller_server.ros__parameters.FollowPath'
    stm32_params = RewrittenYaml(
        source_file=os.path.join(nav, 'config', 'nav2_params.yaml'),
        param_rewrites={
            f'{follow_path}.rotate_to_heading_angular_vel': '0.25',
            f'{follow_path}.max_angular_accel': '4.4',
        },
        convert_types=True,
    )
    # The field map was tracking at fitness 0.02–0.24, then accepted
    # mismatches above 1.0. Correct twice as often with half the yaw step so
    # each correction moves the controller's reference heading less.
    stm32_localization_params = RewrittenYaml(
        source_file=os.path.join(nav, 'config', 'localization.yaml'),
        param_rewrites={
            'score_threshold': '0.5',
            'ndt_align_interval_s': '1.0',
            'auto_correction_max_step_rotation': '0.04',
        },
        convert_types=True,
    )
    defaults = {
        'use_sim_time': 'false', 'bringup_hardware': 'false', 'launch_rviz': 'false',
        'use_imu': 'false',
        'map': os.path.join(nav, 'map', 'traymover_2d.yaml'),
        'params_file': stm32_params,
        'localization_params': stm32_localization_params,
        'bt_xml': os.path.join(nav, 'behavior_trees', 'simple_navigate_to_pose.xml'),
        'through_poses_bt_xml': os.path.join(nav, 'behavior_trees', 'simple_navigate_through_poses.xml'),
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
