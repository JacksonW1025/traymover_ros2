#!/usr/bin/env python3
import importlib.util
from pathlib import Path

from ament_index_python.packages import PackageNotFoundError
from launch import LaunchContext
from launch.actions import LogInfo, OpaqueFunction


PKG_ROOT = Path(__file__).resolve().parents[1]


def test_rviz_plugin_exports_estop_panel():
    plugin_text = (PKG_ROOT / 'plugin_description.xml').read_text(encoding='utf-8')

    assert 'traymover_robot_safety/EstopPanel' in plugin_text
    assert 'traymover_robot_safety::EstopPanel' in plugin_text
    assert 'rviz_common::Panel' in plugin_text


def test_keyboard_uses_shared_estop_interface():
    script_text = (
        PKG_ROOT / 'scripts' / 'traymover_estop_keyboard'
    ).read_text(encoding='utf-8')

    assert "/traymover_estop/set" in script_text
    assert "/traymover_estop/state" in script_text
    assert "SetBool" in script_text
    assert "Bool" in script_text


def test_auto_estop_launch_uses_fail_safe_depth_defaults():
    launch_text = (
        PKG_ROOT / 'launch' / 'traymover_auto_estop.launch.py'
    ).read_text(encoding='utf-8')

    assert "'depth_module.depth_profile': '640x480x15'" in launch_text
    assert "'enable_color': 'false'" in launch_text
    assert "DeclareLaunchArgument('depth_timeout_sec', default_value='0.80')" in launch_text
    assert "DeclareLaunchArgument('stop_distance_m', default_value='0.8')" in launch_text
    assert "DeclareLaunchArgument('release_distance_m', default_value='1.0')" in launch_text
    assert "DeclareLaunchArgument('min_valid_fraction', default_value='0.50')" in launch_text
    assert "DeclareLaunchArgument('severe_invalid_fraction', default_value='0.20')" in launch_text
    assert "DeclareLaunchArgument('near_tile_fraction', default_value='0.20')" in launch_text
    assert "DeclareLaunchArgument('min_near_tile_count', default_value='2')" in launch_text
    assert "DeclareLaunchArgument('min_near_pixel_fraction', default_value='0.005')" in launch_text
    assert "DeclareLaunchArgument('obstacle_frame_count', default_value='2')" in launch_text
    assert "DeclareLaunchArgument('low_valid_frame_count', default_value='2')" in launch_text
    assert "DeclareLaunchArgument('clear_frame_count', default_value='8')" in launch_text


def test_realsense_dependency_only_resolved_when_launching_its_driver(monkeypatch):
    path = PKG_ROOT / 'launch' / 'traymover_auto_estop.launch.py'
    spec = importlib.util.spec_from_file_location('auto_estop_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    lookups = []

    def missing_camera(name):
        lookups.append(name)
        raise PackageNotFoundError(name)

    monkeypatch.setattr(module, 'get_package_share_directory', missing_camera)
    entities = module.generate_launch_description().entities
    camera = next(action for action in entities if isinstance(action, OpaqueFunction))
    context = LaunchContext()
    context.launch_configurations['start_realsense_driver'] = 'false'
    assert not camera.condition.evaluate(context)
    assert not lookups

    context.launch_configurations['start_realsense_driver'] = 'true'
    assert camera.condition.evaluate(context)
    assert isinstance(camera.execute(context)[0], LogInfo)
    assert lookups == ['realsense2_camera']


def test_auto_estop_and_panel_use_shared_aggregate_interface():
    node_text = (PKG_ROOT / 'src' / 'auto_estop_node.cpp').read_text(
        encoding='utf-8'
    )
    panel_text = (PKG_ROOT / 'src' / 'estop_panel.cpp').read_text(
        encoding='utf-8'
    )

    assert '/traymover_estop/auto_request' in node_text
    assert '/traymover_estop/state' in panel_text
    assert '/traymover_estop/manual_state' in panel_text
    assert '/traymover_estop/auto_state' in panel_text
    assert 'EStop active: Automatic' in panel_text
    assert 'setUiState(estop_);' in panel_text
