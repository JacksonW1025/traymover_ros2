import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.actions import LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def start_realsense(_context):
    try:
        realsense_share = get_package_share_directory('realsense2_camera')
    except PackageNotFoundError:
        return [LogInfo(msg=(
            'Automatic EStop requires a RealSense depth publisher. '
            'Source the RealSense ROS overlay or install ros-humble-realsense2-camera. '
            'Automatic EStop will stay engaged while depth is unavailable.'))]

    return [GroupAction(
        scoped=True,
        forwarding=False,
        launch_configurations={
            'enable_depth': 'true',
            'enable_color': 'false',
            'depth_module.depth_profile': '640x480x15',
            'align_depth.enable': 'false',
            'pointcloud.enable': 'false',
            'enable_sync': 'false',
        },
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(realsense_share, 'launch', 'rs_launch.py')
                )
            )
        ],
    )]


def generate_launch_description():
    auto_estop = Node(
        package='traymover_robot_safety',
        executable='traymover_auto_estop',
        name='traymover_auto_estop',
        output='screen',
        parameters=[{
            'depth_topic': LaunchConfiguration('depth_topic'),
            'depth_timeout_sec': ParameterValue(
                LaunchConfiguration('depth_timeout_sec'), value_type=float),
            'heartbeat_rate_hz': ParameterValue(
                LaunchConfiguration('heartbeat_rate_hz'), value_type=float),
            'depth_scale_m': ParameterValue(
                LaunchConfiguration('depth_scale_m'), value_type=float),
            'stop_distance_m': ParameterValue(
                LaunchConfiguration('stop_distance_m'), value_type=float),
            'release_distance_m': ParameterValue(
                LaunchConfiguration('release_distance_m'), value_type=float),
            'roi_x_min': ParameterValue(
                LaunchConfiguration('roi_x_min'), value_type=float),
            'roi_x_max': ParameterValue(
                LaunchConfiguration('roi_x_max'), value_type=float),
            'roi_y_min': ParameterValue(
                LaunchConfiguration('roi_y_min'), value_type=float),
            'roi_y_max': ParameterValue(
                LaunchConfiguration('roi_y_max'), value_type=float),
            'min_valid_fraction': ParameterValue(
                LaunchConfiguration('min_valid_fraction'), value_type=float),
            'severe_invalid_fraction': ParameterValue(
                LaunchConfiguration('severe_invalid_fraction'), value_type=float),
            'grid_columns': ParameterValue(
                LaunchConfiguration('grid_columns'), value_type=int),
            'grid_rows': ParameterValue(
                LaunchConfiguration('grid_rows'), value_type=int),
            'near_tile_fraction': ParameterValue(
                LaunchConfiguration('near_tile_fraction'), value_type=float),
            'min_near_tile_count': ParameterValue(
                LaunchConfiguration('min_near_tile_count'), value_type=int),
            'min_near_pixel_fraction': ParameterValue(
                LaunchConfiguration('min_near_pixel_fraction'), value_type=float),
            'obstacle_frame_count': ParameterValue(
                LaunchConfiguration('obstacle_frame_count'), value_type=int),
            'low_valid_frame_count': ParameterValue(
                LaunchConfiguration('low_valid_frame_count'), value_type=int),
            'clear_frame_count': ParameterValue(
                LaunchConfiguration('clear_frame_count'), value_type=int),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('start_realsense_driver', default_value='true'),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/camera/camera/depth/image_rect_raw'),
        DeclareLaunchArgument('depth_timeout_sec', default_value='0.80'),
        DeclareLaunchArgument('heartbeat_rate_hz', default_value='10.0'),
        DeclareLaunchArgument('depth_scale_m', default_value='0.001'),
        DeclareLaunchArgument('stop_distance_m', default_value='0.8'),
        DeclareLaunchArgument('release_distance_m', default_value='1.0'),
        DeclareLaunchArgument('roi_x_min', default_value='0.25'),
        DeclareLaunchArgument('roi_x_max', default_value='0.75'),
        DeclareLaunchArgument('roi_y_min', default_value='0.20'),
        DeclareLaunchArgument('roi_y_max', default_value='0.85'),
        DeclareLaunchArgument('min_valid_fraction', default_value='0.50'),
        DeclareLaunchArgument('severe_invalid_fraction', default_value='0.20'),
        DeclareLaunchArgument('grid_columns', default_value='16'),
        DeclareLaunchArgument('grid_rows', default_value='12'),
        DeclareLaunchArgument('near_tile_fraction', default_value='0.20'),
        DeclareLaunchArgument('min_near_tile_count', default_value='2'),
        DeclareLaunchArgument('min_near_pixel_fraction', default_value='0.005'),
        DeclareLaunchArgument('obstacle_frame_count', default_value='2'),
        DeclareLaunchArgument('low_valid_frame_count', default_value='2'),
        DeclareLaunchArgument('clear_frame_count', default_value='8'),
        OpaqueFunction(
            function=start_realsense,
            condition=IfCondition(LaunchConfiguration('start_realsense_driver'))),
        auto_estop,
    ])
