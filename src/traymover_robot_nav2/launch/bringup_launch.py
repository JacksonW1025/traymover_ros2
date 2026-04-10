import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file,
    }

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    bringup_cmd_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace,
        ),
        Node(
            condition=IfCondition(use_composition),
            name='nav2_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[configured_params, {'autostart': autostart}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'slam_launch.py')),
            condition=IfCondition(slam),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'use_respawn': use_respawn,
                'params_file': params_file,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'localization_launch.py')),
            condition=IfCondition(PythonExpression(['not ', slam])),
            launch_arguments={
                'namespace': namespace,
                'map': map_yaml_file,
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'params_file': params_file,
                'use_composition': use_composition,
                'use_respawn': use_respawn,
                'container_name': 'nav2_container',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch.py')),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'params_file': params_file,
                'use_composition': use_composition,
                'use_respawn': use_respawn,
                'container_name': 'nav2_container',
            }.items(),
        ),
    ])

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace.',
        ),
        DeclareLaunchArgument(
            'use_namespace',
            default_value='false',
            description='Whether to apply a namespace to the navigation stack.',
        ),
        DeclareLaunchArgument(
            'slam',
            default_value='False',
            description='Whether to run SLAM instead of localization.',
        ),
        DeclareLaunchArgument(
            'map',
            description='Full path to the map yaml file to load.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true.',
        ),
        DeclareLaunchArgument(
            'params_file',
            description='Full path to the Nav2 parameter file.',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the Nav2 stack.',
        ),
        DeclareLaunchArgument(
            'use_composition',
            default_value='True',
            description='Whether to use composed bringup.',
        ),
        DeclareLaunchArgument(
            'use_respawn',
            default_value='False',
            description='Whether to respawn nodes when composition is disabled.',
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Log level.',
        ),
        bringup_cmd_group,
    ])
