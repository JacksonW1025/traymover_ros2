"""No-hardware contracts for Option 19's minimal launch and menu."""
import importlib.util
from pathlib import Path
import subprocess

import pytest
import yaml


PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE.parent.parent


def test_option19_parameters_describe_one_complete_detour():
    data = yaml.safe_load((PACKAGE / 'config' / 'guided_detour_test.yaml').read_text())
    params = data['guided_detour_test']['ros__parameters']
    assert params['forward_speed'] == 0.05
    assert params['wait_before_detour_sec'] == 0.20
    assert params['detour_trigger_distance_m'] == 2.00
    assert params['detour_guard_distance_m'] == 0.95
    assert params['detour_turn_angle_deg'] == 45.0
    assert params['detour_first_leg_m'] == 1.20
    assert params['detour_forward_leg_m'] == 1.00
    assert params['return_forward_m'] == 0.80
    assert params['line_tolerance_m'] == 0.10
    assert params['heading_tolerance_deg'] == 5.0
    assert params['odom_timeout_sec'] == 0.5
    assert params['scan_timeout_sec'] == 1.0


def test_menu_has_no_option19_question_or_extra_mode():
    menu = (WORKSPACE / 'scripts' / 'traymover.sh').read_text()
    assert '19) Single-button Straight-line Detour Test (No Map / No Nav2)' in menu
    assert '19) action_start_guided_detour_test ;;' in menu
    body = menu.split('action_start_guided_detour_test() {', 1)[1].split('\n}', 1)[0]
    assert 'read -r -p' not in body
    assert 'ARM' not in body
    assert 'launch_rviz' not in body
    assert 'traymover_auto_estop' not in body
    assert 'collision_monitor' not in body.lower()
    assert 'realsense' not in body.lower()
    assert 'guided_detour_test.launch.py' in body
    assert 'guided_detour_test.py --ros-args' in body
    assert 'trap cleanup_guided_session EXIT' in body
    subprocess.run(['bash', '-n', str(WORKSPACE / 'scripts' / 'traymover.sh')], check=True)


def test_launch_uses_only_chassis_and_realsense_depth(monkeypatch):
    pytest.importorskip('launch_ros')
    from launch import LaunchContext
    from launch.actions import GroupAction, IncludeLaunchDescription
    from launch.utilities import perform_substitutions

    path = PACKAGE / 'launch' / 'guided_detour_test.launch.py'
    spec = importlib.util.spec_from_file_location('option19_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory',
                        lambda name: str(PACKAGE.parent / name))
    nodes = []
    monkeypatch.setattr(module, 'Node', lambda **kwargs: nodes.append(kwargs) or kwargs)
    actions = module.generate_launch_description().entities
    context = LaunchContext()
    includes = {}
    nested = []
    for action in actions:
        nested.extend(action.get_sub_entities() if isinstance(action, GroupAction) else [action])
    for item in nested:
        if isinstance(item, IncludeLaunchDescription):
            location = item.launch_description_source._LaunchDescriptionSource__location
            includes[Path(perform_substitutions(context, location)).name] = dict(
                item.launch_arguments)

    assert set(includes) == {
        'base_serial.launch.py', 'rs_launch.py',
    }
    assert includes['base_serial.launch.py'] == {
        'use_imu': 'false', 'odom_source_mode': 'stm32_feedback',
        'auto_estop_enabled': 'false',
    }
    assert {node['package'] for node in nodes} == {
        'depthimage_to_laserscan',
    }
    depth = next(node for node in nodes if node['package'] == 'depthimage_to_laserscan')
    assert dict(depth['remappings'])['scan'] == '/scan_realsense'
    assert depth['parameters'][0]['scan_height'] == 20
