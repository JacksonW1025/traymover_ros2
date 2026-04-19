import os
import yaml
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def load_yaml(path: Path) -> dict:
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def include_lidar_launch(context, *args, **kwargs):
    yaml_path = LaunchConfiguration('traymover_param_yaml').perform(context)
    cfg = load_yaml(Path(yaml_path))
    lidar_type = LaunchConfiguration('lidar_type').perform(context) or cfg['lidar_type']
    launch_actions = []
    if lidar_type == 'lscx':
        template_yaml = Path(
            get_package_share_directory('lslidar_driver'),
            'config', 'lslidar_cx.yaml')
        cx_cfg = yaml.safe_load(template_yaml.read_text())['cx']['lslidar_driver_node']['ros__parameters']
        lscx_cfg = cfg.get('lscx', {})
        if 'device_ip' in lscx_cfg:
            cx_cfg['device_ip'] = lscx_cfg['device_ip']
        if 'pointcloud_topic' in lscx_cfg:
            cx_cfg['pointcloud_topic'] = lscx_cfg['pointcloud_topic']
        if 'frame_id' in lscx_cfg:
            cx_cfg['frame_id'] = lscx_cfg['frame_id']
        if lscx_cfg.get('angle_disable_min') and lscx_cfg.get('angle_disable_max'):
            cx_cfg['angle_disable_min'] = lscx_cfg['angle_disable_min']
            cx_cfg['angle_disable_max'] = lscx_cfg['angle_disable_max']

        lidar_launch = Node(
            package='lslidar_driver',
            executable='lslidar_driver_node',
            name='lslidar_driver_node',
            namespace='cx',
            parameters=[cx_cfg],
            output='screen',
        )
        pointcloud_topic = lscx_cfg.get('pointcloud_topic', cx_cfg.get('pointcloud_topic', '/point_cloud_raw'))
        frame_id = lscx_cfg.get('frame_id', cx_cfg.get('frame_id', 'laser'))
        scan_bridge = Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='traymover_pointcloud_to_laserscan',
            remappings=[
                ('cloud_in', pointcloud_topic),
                ('scan', '/scan'),
            ],
            parameters=[{
                'target_frame': frame_id,
                'transform_tolerance': 0.05,
                'min_height': -0.2,
                'max_height': 2.0,
                # /scan limited to front 180 deg (-pi/2 to +pi/2 in laser
                # frame) — Nav2 obstacle_layer and collision_monitor only
                # see points in front of the robot. LiDAR driver still
                # publishes the full 360 deg on /point_cloud_raw so NDT
                # localization has full geometric constraints.
                'angle_min': -1.5707963,
                'angle_max': 1.5707963,
                'angle_increment': 0.0087,
                'scan_time': 0.1,
                'range_min': 0.3,
                'range_max': cx_cfg.get('max_range', 200.0),
                'use_inf': True,
                'inf_epsilon': 1.0,
            }],
            output='screen',
        )
        launch_actions.extend([lidar_launch, scan_bridge])
    elif lidar_type.startswith('ls_'):
        template_yaml = Path(
            get_package_share_directory('lslidar_driver'),
            'config', 'lslidar_x10.yaml')
        x10_cfg = yaml.safe_load(template_yaml.read_text())['x10']['lslidar_driver_node']['ros__parameters']
        x10_overrides = cfg.get('x10', {})
        if 'lidar_port' in x10_overrides:
            x10_cfg['serial_port'] = x10_overrides['lidar_port']
        if x10_overrides.get('angle_disable_min') and x10_overrides.get('angle_disable_max'):
            x10_cfg['angle_disable_min'] = x10_overrides['angle_disable_min']
            x10_cfg['angle_disable_max'] = x10_overrides['angle_disable_max']

        lidar_launch = Node(
            package='lslidar_driver',
            executable='lslidar_driver_node',
            name='lslidar_driver_node',
            namespace='x10',
            parameters=[x10_cfg],
            output='screen',
        )
        launch_actions.append(lidar_launch)
    else:
        raise ValueError(
            f'Unsupported lidar_type {lidar_type!r}. This Traymover workspace currently '
            'supports the Leishen C32/CX path and lslidar X10-family drivers only.')

    return launch_actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'traymover_param_yaml',
            default_value=os.path.join(
                get_package_share_directory('turn_on_traymover_robot'),
                'config', 'traymover_param.yaml'),
            description='Path to traymover_param.yaml'),
        DeclareLaunchArgument(
            'lidar_type',
            default_value='',
            description='Which lidar model to launch'),
        OpaqueFunction(function=include_lidar_launch),
    ])
