"""Launch: global LiDAR localization against a prebuilt FAST_LIO PCD map,
with FAST_LIO running online as the fast odometry source.

Rationale for the two-stage stack (FAST_LIO + NDT):
  Pure NDT-to-map in open airport space fails — NDT's convergence basin is
  ~ndt_resolution/2 (= 0.25 m with our 0.5 m voxel), and with no motion
  prior the init_guess = previous_pose, so any motion larger than that
  freezes the pose or lets yaw oscillate.  FAST_LIO provides 20 Hz IMU+LiDAR
  tight-coupled odometry (odom->body), giving NDT a good init_guess every
  frame; NDT only corrects the slow FAST_LIO drift against the prebuilt PCD.

TF chain at runtime:
    map -> odom            (from lidar_localization, enable_map_odom_tf=true)
    odom -> camera_init    (static identity, this launch)
    camera_init -> body    (from fastlio_mapping, ~20 Hz)
    body -> base_link      (static, (0, 0, +0.12), this launch — inverse of
                            URDF imu_joint origin [0 0 -0.12])
    base_link -> {imu_link, laser, wheels, ...}   (URDF via RSP)

Required upstream (started separately by base_serial + traymover_lidar
or by traymover_nav.launch.py bringup_hardware:=true):
    /point_cloud_raw   (LSLiDAR CX32)
    /imu/data_raw      (hipnuc N300 Pro)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent,
                            RegisterEventHandler)
from launch.events import matches_action
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    nav_share = get_package_share_directory('traymover_robot_nav')
    desc_share = get_package_share_directory('traymover_robot_description')
    fastlio_share = get_package_share_directory('fast_lio')

    default_params = os.path.join(nav_share, 'config', 'localization.yaml')
    default_urdf = os.path.join(desc_share, 'urdf', 'traymover.urdf.xacro')
    fastlio_config = os.path.join(fastlio_share, 'config', 'traymover.yaml')

    localization_params = LaunchConfiguration('localization_params')
    urdf_model = LaunchConfiguration('urdf_model')
    use_sim_time = LaunchConfiguration('use_sim_time')
    # Default PCD matches what's hardcoded in config/localization.yaml.
    # Pass a different path via `pcd_path:=<abs>` to override at runtime.
    default_pcd = ('/home/wheeltec/traymover_ros2/src/traymover_robot_slam/'
                   'FAST_LIO/PCD/traymover_20260418_183556.pcd')

    robot_description = ParameterValue(
        Command(['xacro', ' ', urdf_model]), value_type=str)

    # ------------------------------------------------------------------
    # 1) FAST_LIO online — publishes /Odometry (camera_init -> body) and
    #    TF camera_init->body at ~20 Hz. Remapped to /odom so Nav2 and
    #    lidar_localization both read it from a single topic.
    #    pcd_save.pcd_save_en:=false so shutdown does not overwrite the
    #    staging PCD used by the mapping flow.
    # ------------------------------------------------------------------
    fast_lio = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fastlio_mapping',
        parameters=[
            fastlio_config,
            {
                'use_sim_time': use_sim_time,
                'pcd_save.pcd_save_en': False,
            },
        ],
        remappings=[
            ('/Odometry', '/odom'),
        ],
        output='screen',
    )

    # ------------------------------------------------------------------
    # 2) Static TFs that stitch FAST_LIO's frames into our URDF tree.
    #    odom  ==  camera_init         (name bridge, identity)
    #    body  +0.12m z -> base_link   (inverse of URDF imu_joint origin
    #                                   [0 0 -0.12]; imu_joint rpy is zero
    #                                   so rotation is identity)
    # ------------------------------------------------------------------
    static_odom_to_cam_init = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_odom_to_camera_init',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'camera_init'],
        output='screen',
    )
    static_body_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_body_to_base_link',
        # xyz in body frame where base_link sits. Body (IMU) is at
        # base_link + (0, 0, -0.12), so base_link is at body + (0, 0, +0.12).
        arguments=['0', '0', '0.12', '0', '0', '0', 'body', 'base_link'],
        output='screen',
    )

    # ------------------------------------------------------------------
    # 3) lidar_localization (NDT_OMP) — map -> odom correction lifecycle node
    # ------------------------------------------------------------------
    lidar_loc = LifecycleNode(
        package='lidar_localization_ros2',
        executable='lidar_localization_node',
        name='lidar_localization',
        namespace='',
        parameters=[
            localization_params,
            {'use_sim_time': use_sim_time},
            {'map_path': LaunchConfiguration('pcd_path')},
        ],
        remappings=[
            ('/cloud', '/point_cloud_raw'),
            ('/imu', '/imu/data_raw'),
            # /odom is the FAST_LIO-provided twist; lidar_localization
            # integrates it between NDT scans. Kept explicit so breaking the
            # remap in FAST_LIO doesn't silently disable motion prediction.
            ('/odom', '/odom'),
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

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('localization_params', default_value=default_params),
        DeclareLaunchArgument('urdf_model', default_value=default_urdf),
        DeclareLaunchArgument(
            'pcd_path', default_value=default_pcd,
            description='Absolute path to the PCD used as prior map.'),
        rsp,
        fast_lio,
        static_odom_to_cam_init,
        static_body_to_base,
        lidar_loc,
        to_configure,
        on_inactive,
    ])
