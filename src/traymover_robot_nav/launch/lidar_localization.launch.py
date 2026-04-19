"""Launch: global LiDAR localization against a prebuilt FAST_LIO PCD map.

Runs two pieces:
  1. lidar_localization_ros2 (NDT_OMP) Lifecycle node, auto-configured and activated.
     Publishes map -> base_link TF. Listens to /initialpose (RViz 2D Pose Estimate).
  2. robot_state_publisher: publishes URDF static TFs (base_link -> laser / imu_link / ...).

Notes:
  - /scan is expected to be published by turn_on_traymover_robot's
    traymover_lidar.launch.py (which runs pointcloud_to_laserscan). We do NOT
    start a duplicate pc2scan here to avoid node-name collision.
  - imu_pose_broadcaster from traymover_robot_description is intentionally NOT started,
    to avoid TF conflict on base_link (lidar_localization becomes its sole parent publisher).
  - Assumes LiDAR + IMU drivers (/point_cloud_raw, /imu/data_raw) are already running
    from base_serial.launch.py / traymover_lidar.launch.py.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent,
                            RegisterEventHandler)
from launch.events import matches_action
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    nav_share = get_package_share_directory('traymover_robot_nav')
    desc_share = get_package_share_directory('traymover_robot_description')

    default_params = os.path.join(nav_share, 'config', 'localization.yaml')
    default_urdf = os.path.join(desc_share, 'urdf', 'traymover.urdf.xacro')

    localization_params = LaunchConfiguration('localization_params')
    urdf_model = LaunchConfiguration('urdf_model')
    use_sim_time = LaunchConfiguration('use_sim_time')
    # Default PCD matches what's hardcoded in config/localization.yaml.
    # Pass a different path via `pcd_path:=<abs>` to override at runtime.
    default_pcd = ('/home/wheeltec/traymover_ros2/src/traymover_robot_slam/'
                   'FAST_LIO/PCD/traymover_20260418_183556.pcd')

    robot_description = ParameterValue(
        Command(['xacro', ' ', urdf_model]), value_type=str)

    lidar_loc = LifecycleNode(
        package='lidar_localization_ros2',
        executable='lidar_localization_node',
        name='lidar_localization',
        namespace='',
        parameters=[
            localization_params,
            # Overrides. pcd_path defaults to an empty string; when empty
            # the node falls back to the map_path inside the YAML, so passing
            # nothing keeps old behavior.
            {'use_sim_time': use_sim_time},
            {'map_path': LaunchConfiguration('pcd_path')},
        ],
        remappings=[
            ('/cloud', '/point_cloud_raw'),
            ('/imu', '/imu/data_raw'),
        ],
        output='screen',
    )

    to_configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(lidar_loc),
        transition_id=Transition.TRANSITION_CONFIGURE,
    ))

    on_inactive = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=lidar_loc,
        goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(lidar_loc),
            transition_id=Transition.TRANSITION_ACTIVATE,
        ))],
    ))

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='traymover_robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
        }],
        output='screen',
    )

    # tf_flattener kept as a no-op safety net: publishes map -> base_link_2d
    # mirroring x/y/yaw of the raw 6DOF TF. Nav2 does NOT use base_link_2d
    # now (NDT after tuning is already effectively 2D), but flattener stays
    # available for diagnostics (ros2 run tf2_ros tf2_echo map base_link_2d)
    # and as fallback if NDT 6DOF divergence recurs.
    tf_flattener = Node(
        package='traymover_robot_nav',
        executable='tf_flattener.py',
        name='tf_flattener',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'global_frame': 'map',
            'source_frame': 'base_link',
            'target_frame': 'base_link_2d',
            'rate_hz': 30.0,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('localization_params', default_value=default_params),
        DeclareLaunchArgument('urdf_model', default_value=default_urdf),
        DeclareLaunchArgument(
            'pcd_path', default_value=default_pcd,
            description='Absolute path to the PCD used as prior map.'),
        rsp,
        lidar_loc,
        tf_flattener,
        to_configure,
        on_inactive,
    ])
