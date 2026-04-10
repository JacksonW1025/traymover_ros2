import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _create_nav_actions(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    map_path = LaunchConfiguration('map').perform(context)
    params = LaunchConfiguration('params').perform(context)
    traymover_param_yaml = LaunchConfiguration('traymover_param_yaml').perform(context)
    robot_config = LaunchConfiguration('robot_config').perform(context)

    nav_dir = get_package_share_directory('traymover_nav2')
    nav_launch_dir = os.path.join(nav_dir, 'launch')

    bringup_dir = get_package_share_directory('turn_on_traymover_robot')
    bringup_launch_dir = os.path.join(bringup_dir, 'launch')

    default_map_path = os.path.join(
        nav_dir,
        'map',
        'TRAYMOVER.yaml',
    )

    if not os.path.exists(map_path):
        if map_path == default_map_path:
            raise RuntimeError(
                'Traymover Nav2 default map does not exist yet. Run '
                '"ros2 launch traymover_nav2 save_map.launch.py" after gmapping, or '
                'pass a custom map:=/absolute/path/to/map.yaml.'
            )

        raise RuntimeError(f'Traymover Nav2 map file does not exist: {map_path}')

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_launch_dir, 'turn_on_traymover_robot.launch.py')
            ),
            launch_arguments={
                'use_description': 'true',
                'use_rviz': 'false',
                'use_imu': 'true',
                'use_lidar': 'true',
                'use_ekf': 'true',
                'use_sim_time': use_sim_time,
                'traymover_param_yaml': traymover_param_yaml,
                'robot_config': robot_config,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_launch_dir, 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_path,
                'use_sim_time': use_sim_time,
                'params_file': params,
            }.items(),
        ),
    ]


def generate_launch_description():
    nav_dir = get_package_share_directory('traymover_nav2')
    bringup_dir = get_package_share_directory('turn_on_traymover_robot')
    bringup_config_dir = os.path.join(bringup_dir, 'config')

    default_map = os.path.join(nav_dir, 'map', 'TRAYMOVER.yaml')
    default_params = os.path.join(
        nav_dir,
        'param',
        'traymover_params',
        'param_traymover_diff.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true.',
        ),
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Full path to the map yaml file to load.',
        ),
        DeclareLaunchArgument(
            'params',
            default_value=default_params,
            description='Full path to the Nav2 parameter file.',
        ),
        DeclareLaunchArgument(
            'traymover_param_yaml',
            default_value=os.path.join(bringup_config_dir, 'traymover_param.yaml'),
            description='Traymover shared bringup parameter file.',
        ),
        DeclareLaunchArgument(
            'robot_config',
            default_value=os.path.join(bringup_config_dir, 'traymover_robot.yaml'),
            description='Traymover base-driver configuration file.',
        ),
        OpaqueFunction(function=_create_nav_actions),
    ])
