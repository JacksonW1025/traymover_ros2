"""Gazebo/Nav2 bringup for the Traymover dynamic-obstacle detour demo."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


NAV2_LIFECYCLE_NODES = [
    "map_server",
    "amcl",
    "planner_server",
    "controller_server",
    "bt_navigator",
]


def generate_launch_description():
    sim_share = get_package_share_directory("traymover_robot_sim")
    description_share = get_package_share_directory("traymover_robot_description")

    default_world = os.path.join(sim_share, "worlds", "traymover_detour.sdf")
    default_map = os.path.join(sim_share, "maps", "detour_demo.yaml")
    default_params = os.path.join(sim_share, "config", "nav2_params_detour_sim.yaml")
    default_collision_params = os.path.join(
        sim_share, "config", "collision_monitor_detour_sim.yaml"
    )
    default_bridge = os.path.join(sim_share, "config", "bridge.yaml")
    default_bt = os.path.join(sim_share, "behavior_trees", "navigate_detour.xml")
    default_rviz = os.path.join(sim_share, "rviz", "traymover_detour.rviz")
    default_xacro = os.path.join(
        description_share, "urdf", "traymover_sim.urdf.xacro"
    )
    default_box = os.path.join(sim_share, "models", "dynamic_box", "model.sdf")

    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_detour = LaunchConfiguration("enable_detour")
    spawn_dynamic_obstacle = LaunchConfiguration("spawn_dynamic_obstacle")
    obstacle_spawn_delay = LaunchConfiguration("obstacle_spawn_delay")
    obstacle_lifetime_sec = LaunchConfiguration("obstacle_lifetime_sec")
    auto_send_goal = LaunchConfiguration("auto_send_goal")
    launch_rviz = LaunchConfiguration("launch_rviz")
    map_yaml = LaunchConfiguration("map")
    world = LaunchConfiguration("world")
    goal_x = LaunchConfiguration("goal_x")
    goal_y = LaunchConfiguration("goal_y")
    goal_yaw = LaunchConfiguration("goal_yaw")

    robot_description = Command(["xacro ", default_xacro])
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=default_params,
            param_rewrites={
                "yaml_filename": map_yaml,
                "use_sim_time": use_sim_time,
                "default_nav_to_pose_bt_xml": default_bt,
                "default_nav_through_poses_bt_xml": default_bt,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    gazebo = Node(
        package="ros_gz_sim",
        executable="gz_sim",
        name="gz_sim",
        output="screen",
        arguments=["-r", world],
    )
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
    )
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_traymover",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "traymover",
            "-x",
            "1.0",
            "-y",
            "0.0",
            "-z",
            "0.0",
        ],
    )
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        arguments=["--ros-args", "-p", "config_file:=" + default_bridge],
    )
    odom_tf = Node(
        package="traymover_robot_sim",
        executable="sim_odom_tf",
        name="sim_odom_tf",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[configured_params],
    )
    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[configured_params],
    )
    planner = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[configured_params],
    )
    controller = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[configured_params],
        remappings=[("cmd_vel", "/cmd_vel_nav")],
    )
    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[configured_params],
    )
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": NAV2_LIFECYCLE_NODES,
            }
        ],
    )
    collision_monitor = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        output="screen",
        parameters=[ParameterFile(default_collision_params, allow_substs=True)],
    )
    detour_supervisor = Node(
        package="traymover_robot_sim",
        executable="detour_supervisor",
        name="detour_supervisor",
        output="screen",
        condition=IfCondition(enable_detour),
        parameters=[{"use_sim_time": use_sim_time, "enable_detour": enable_detour}],
    )

    spawn_box = TimerAction(
        period=obstacle_spawn_delay,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                name="spawn_dynamic_box",
                output="screen",
                condition=IfCondition(spawn_dynamic_obstacle),
                arguments=[
                    "-file",
                    default_box,
                    "-name",
                    "dynamic_box",
                    "-x",
                    "4.0",
                    "-y",
                    "0.0",
                    "-z",
                    "0.45",
                ],
            )
        ],
    )
    delete_box = TimerAction(
        period=PythonExpression(
            ["float('", obstacle_spawn_delay, "') + float('", obstacle_lifetime_sec, "')"]
        ),
        actions=[
            ExecuteProcess(
                cmd=["ros2", "run", "ros_gz_sim", "delete_entity", "--name", "dynamic_box"],
                output="screen",
                condition=IfCondition(
                    PythonExpression(["float('", obstacle_lifetime_sec, "') > 0.0"])
                ),
            )
        ],
    )
    goal_sender = Node(
        package="traymover_robot_sim",
        executable="demo_goal_sender",
        name="demo_goal_sender",
        output="screen",
        condition=IfCondition(auto_send_goal),
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "goal_x": goal_x,
                "goal_y": goal_y,
                "goal_yaw": goal_yaw,
            }
        ],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(launch_rviz),
        arguments=["-d", default_rviz],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("enable_detour", default_value="true"),
            DeclareLaunchArgument("spawn_dynamic_obstacle", default_value="true"),
            DeclareLaunchArgument("obstacle_spawn_delay", default_value="12.0"),
            DeclareLaunchArgument("obstacle_lifetime_sec", default_value="0.0"),
            DeclareLaunchArgument("auto_send_goal", default_value="true"),
            DeclareLaunchArgument("goal_x", default_value="7.0"),
            DeclareLaunchArgument("goal_y", default_value="0.0"),
            DeclareLaunchArgument("goal_yaw", default_value="0.0"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("map", default_value=default_map),
            DeclareLaunchArgument("world", default_value=default_world),
            gazebo,
            state_publisher,
            spawn_robot,
            bridge,
            odom_tf,
            map_server,
            amcl,
            planner,
            controller,
            bt_navigator,
            lifecycle_manager,
            collision_monitor,
            detour_supervisor,
            spawn_box,
            delete_box,
            goal_sender,
            rviz,
        ]
    )
