"""Exercise menu 21 and launch conditions without starting any hardware."""
import importlib.util
from pathlib import Path

import pytest
import yaml

from test_nav_simple_config import _run_option10_prompt


PKG = Path(__file__).resolve().parents[1]
ROOT = PKG.parents[1]


def load_launch(name, monkeypatch):
    spec = importlib.util.spec_from_file_location(name, PKG / 'launch' / f'{name}.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(PKG))
    return module


def context_for(description, **overrides):
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    context = LaunchContext()
    context.launch_configurations.update(overrides)
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    return context


def test_option21_menu_keeps_baseline_and_selects_feedback(tmp_path):
    result, _, hardware = _run_option10_prompt(tmp_path, '1\nn\nn\n\n', option=21)
    assert result.returncode == 0, result.stdout + result.stderr
    assert hardware
    commands = (tmp_path / 'hardware').read_text()
    assert 'odom_source_mode:=stm32_velocity' in commands
    assert 'use_imu:=false' in commands
    assert 'stm32_navigation.yaml' in commands
    assert 'traymover_nav_stm32.launch.py' in commands
    assert 'TRAYMOVER_ODOMETRY_SOURCE=stm32' in commands
    assert 'auto_correction_enabled:=true' in commands
    assert 'odom_source_mode:=none' not in commands
    info, = (tmp_path / 'nav_logs').glob('*/run_info.txt')
    assert 'mode=live_option21' in info.read_text()
    assert 'odometry_source=stm32_velocity' in info.read_text()
    launcher = (ROOT / 'scripts/traymover.sh').read_text()
    assert '21) action_start_nav 21 ;;' in launcher


@pytest.mark.parametrize('source,expected', [('fastlio', 'none'), ('stm32', 'stm32_velocity')])
def test_hardware_and_localization_receive_matching_sources(monkeypatch, source, expected):
    from launch.actions import IncludeLaunchDescription
    module = load_launch('traymover_nav', monkeypatch)
    description = module.generate_launch_description()
    context = context_for(description, odometry_source=source)
    includes = [dict(a.launch_arguments) for a in description.entities
                if isinstance(a, IncludeLaunchDescription)]
    hardware, = [args for args in includes if 'odom_source_mode' in args]
    localization, = [args for args in includes if 'odometry_source' in args]
    assert hardware['odom_source_mode'].perform(context) == expected
    assert localization['odometry_source'].perform(context) == source


@pytest.mark.parametrize('source', ['fastlio', 'stm32'])
def test_only_one_motion_tf_branch_is_active(monkeypatch, source):
    module = load_launch('lidar_localization', monkeypatch)
    real_node = module.Node
    captured = []
    def capture(**kwargs):
        captured.append(kwargs)
        return real_node(**kwargs)
    monkeypatch.setattr(module, 'Node', capture)
    description = module.generate_launch_description()
    context = context_for(description, odometry_source=source)
    active = {item.get('name') for item in captured
              if 'condition' not in item or item['condition'].evaluate(context)}
    if source == 'stm32':
        assert 'stm32_base_height' in active
        assert not active.intersection({'fastlio_mapping', 'static_odom_to_camera_init',
                                        'static_body_to_base_link'})
    else:
        assert 'fastlio_mapping' in active
        assert 'stm32_base_height' not in active


def test_option21_wrapper_and_si_profile(monkeypatch):
    from launch.actions import IncludeLaunchDescription
    module = load_launch('traymover_nav_stm32', monkeypatch)
    description = module.generate_launch_description()
    context = context_for(description, pcd_path='/tmp/lvisam.pcd')
    include, = [a for a in description.entities if isinstance(a, IncludeLaunchDescription)]
    args = dict(include.launch_arguments)
    assert args['odometry_source'] == 'stm32'
    assert args['pcd_path'].perform(context) == '/tmp/lvisam.pcd'
    assert args['auto_correction_enabled'].perform(context) == 'true'
    assert args['use_imu'].perform(context) == 'false'
    profile = yaml.safe_load((ROOT / 'src/turn_on_traymover_robot/config/'
                             'stm32_navigation.yaml').read_text())
    params = profile['turn_on_traymover_robot']['ros__parameters']
    assert params['odom_source_mode'] == 'stm32_velocity'
    assert params['publish_odom_tf'] is True
    assert params['robot_frame_id'] == 'base_footprint'
    assert params['odom_frame_id'] == 'odom'
