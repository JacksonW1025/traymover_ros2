"""Expand real nested launch scopes without starting a ROS or hardware process."""

import importlib.util
from pathlib import Path

import pytest


PKG = Path(__file__).resolve().parents[1]


def test_scoped_remaps_reach_nav2_and_leave_public_endpoints_unmodified(monkeypatch):
    pytest.importorskip('launch_ros')
    import ament_index_python.packages
    from launch import LaunchDescription, LaunchService
    from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetLaunchConfiguration
    from launch.utilities import perform_substitutions
    from launch_ros.actions import Node

    monkeypatch.setattr(
        ament_index_python.packages, 'get_package_share_directory',
        lambda name: str(PKG if name == 'traymover_robot_nav' else PKG.parent / name))
    path = PKG / 'launch' / 'traymover_nav_lidar_realsense_detour.launch.py'
    spec = importlib.util.spec_from_file_location('option18_launch_graph', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    # Keep the real baseline and collision-monitor include expansion. Unrelated
    # hardware/localization includes have their own tests; exclude them before
    # importing their launch files or evaluating xacro/lifecycle actions.
    execute_include = IncludeLaunchDescription.execute
    expanded_includes = []

    def include_without_hardware(action, context):
        location = action.launch_description_source._LaunchDescriptionSource__location
        name = Path(perform_substitutions(context, location)).name
        if name not in ('traymover_nav.launch.py', 'collision_monitor.launch.py',
                        'navigation_pointcloud.launch.py'):
            return []
        expanded_includes.append(name)
        return execute_include(action, context)

    monkeypatch.setattr(IncludeLaunchDescription, 'execute', include_without_hardware)
    captured = {}

    def capture_node(action, context):
        # Use launch_ros' actual parameter/remap substitution, then intercept
        # execution before ExecuteProcess can spawn anything.
        action._perform_substitutions(context)
        name = '/' + action.node_name.lstrip('/')
        assert name not in captured
        captured[name] = dict(action.expanded_remapping_rules or [])
        return []

    def reject_process(_action, _context):
        pytest.fail('launch expansion attempted to start a process')

    monkeypatch.setattr(Node, 'execute', capture_node)
    monkeypatch.setattr(ExecuteProcess, 'execute', reject_process)
    service = LaunchService()
    service.include_launch_description(LaunchDescription([
        SetLaunchConfiguration('ros_namespace', '/'),
        SetLaunchConfiguration('launch_rviz', 'true'),
        module.generate_launch_description(),
    ]))
    assert service.run() == 0
    assert 'traymover_nav.launch.py' in expanded_includes
    assert 'collision_monitor.launch.py' in expanded_includes

    expected = {
        f'/{action}/_action/{endpoint}': f'/{action}_detour_backend/_action/{endpoint}'
        for action in ('navigate_to_pose', 'navigate_through_poses')
        for endpoint in ('send_goal', 'get_result', 'cancel_goal', 'feedback', 'status')
    }
    expected.update({'/goal_pose': '/goal_pose_detour_backend_unused',
                     '/cmd_vel': '/cmd_vel_nav'})
    for name in ('/bt_navigator', '/controller_server', '/planner_server'):
        assert name in captured, captured
        assert captured[name] == expected
    for name in ('/traymover_detour_supervisor', '/collision_monitor', '/rviz2',
                 '/lifecycle_manager_collision_monitor'):
        assert captured[name] == {}
    # The separate depth projector retains its own sensor remaps, with no
    # navigation scope leaking into it after the nested baseline returns.
    assert captured['/traymover_realsense_depth_to_laserscan'] == {
        'depth': '/camera/camera/depth/image_rect_raw',
        'depth_camera_info': '/camera/camera/depth/camera_info',
        'scan': '/scan_realsense',
    }
