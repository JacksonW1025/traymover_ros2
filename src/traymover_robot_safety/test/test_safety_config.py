#!/usr/bin/env python3
from pathlib import Path


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
    assert "DeclareLaunchArgument('depth_timeout_sec', default_value='0.50')" in launch_text
    assert "DeclareLaunchArgument('stop_distance_m', default_value='0.8')" in launch_text
    assert "DeclareLaunchArgument('release_distance_m', default_value='1.0')" in launch_text
    assert "DeclareLaunchArgument('min_valid_fraction', default_value='0.50')" in launch_text
    assert "DeclareLaunchArgument('clear_frame_count', default_value='8')" in launch_text


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
