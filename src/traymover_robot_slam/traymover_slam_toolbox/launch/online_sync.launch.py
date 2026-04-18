"""Launch slam_toolbox (sync) for the Traymover chassis.

Same bringup path as online_async.launch.py but runs the synchronous solver,
which blocks each scan update until processing completes. Slower but produces
crisper maps; prefer for offline refinement or low-speed mapping runs.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    slam_pkg = get_package_share_directory('traymover_slam_toolbox')
    bringup_launch_dir = os.path.join(
        get_package_share_directory('turn_on_traymover_robot'), 'launch')

    default_params_file = os.path.join(
        slam_pkg, 'config', 'mapper_params_online_sync.yaml')
    default_rviz_file = os.path.join(
        slam_pkg, 'rviz', 'traymover_slam.rviz')

    slam_params_file = LaunchConfiguration('slam_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    use_ekf = LaunchConfiguration('use_ekf')
    lidar_type = LaunchConfiguration('lidar_type')
    rviz_config = LaunchConfiguration('rviz_config')

    # Wrapped in GroupAction so `use_rviz: false` passed to the bringup does not
    # leak into our parent scope and disable our SLAM RViz node.
    traymover_robot = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_launch_dir, 'turn_on_traymover_robot.launch.py')),
            launch_arguments={
                'use_lidar': 'true',
                'use_ekf': use_ekf,
                'use_rviz': 'false',
                'use_sim_time': use_sim_time,
                'lidar_type': lidar_type,
            }.items(),
        ),
    ])

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        emulate_tty=True,
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=default_params_file,
            description='slam_toolbox parameter file for the Traymover.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz with the SLAM view.',
        ),
        DeclareLaunchArgument(
            'use_ekf',
            default_value='true',
            description='Run robot_localization EKF so SLAM uses odom_combined.',
        ),
        DeclareLaunchArgument(
            'lidar_type',
            default_value='',
            description='Override lidar type (lscx / ls_*); empty uses traymover_param.yaml.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_file,
            description='RViz config tuned for live SLAM (map + /scan on map frame).',
        ),
        traymover_robot,
        slam_toolbox_node,
        rviz_node,
    ])
