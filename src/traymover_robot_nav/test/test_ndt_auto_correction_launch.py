"""Verify option-10 opt-in and typed NDT correction launch propagation."""

import importlib.util
from pathlib import Path

import pytest

from test_nav_simple_config import _run_option10_prompt


PKG = Path(__file__).resolve().parents[1]


def test_option10_enables_and_records_automatic_correction(tmp_path):
    result, _, hardware = _run_option10_prompt(tmp_path, '1\nn\nn\n\n')
    assert result.returncode == 0, result.stdout + result.stderr
    assert hardware
    commands = (tmp_path / 'hardware').read_text().splitlines()
    nav_command, = [line for line in commands if 'traymover_nav.launch.py' in line]
    assert 'auto_correction_enabled:=true' in nav_command
    info, = (tmp_path / 'nav_logs').glob('*/run_info.txt')
    settings = dict(line.split('=', 1) for line in info.read_text().splitlines())
    assert settings['auto_correction_enabled'] == 'true'


def _load_launch(name, monkeypatch):
    spec = importlib.util.spec_from_file_location(
        name, PKG / 'launch' / f'{name}.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda package: str(PKG))
    return module


@pytest.mark.parametrize('enabled', [None, 'true', 'false'])
def test_nested_launch_propagates_typed_opt_in(monkeypatch, enabled):
    pytest.importorskip('launch_ros')
    pytest.importorskip('nav2_common')
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
    from launch_ros.utilities import evaluate_parameters, normalize_parameters

    outer = _load_launch('traymover_nav', monkeypatch).generate_launch_description()
    declaration, = [action for action in outer.entities
                    if isinstance(action, DeclareLaunchArgument)
                    and action.name == 'auto_correction_enabled']
    context = LaunchContext()
    declaration.execute(context)
    assert context.launch_configurations['auto_correction_enabled'] == 'false'
    if enabled is not None:
        context.launch_configurations['auto_correction_enabled'] = enabled
    arguments, = [dict(action.launch_arguments) for action in outer.entities
                  if isinstance(action, IncludeLaunchDescription)
                  and 'auto_correction_enabled' in dict(action.launch_arguments)]
    inner_context = LaunchContext()
    inner_context.launch_configurations['auto_correction_enabled'] = (
        arguments['auto_correction_enabled'].perform(context))

    inner = _load_launch('lidar_localization', monkeypatch)
    real_lifecycle_node = inner.LifecycleNode
    captured = {}

    def capture_node(**kwargs):
        captured.update(kwargs)
        return real_lifecycle_node(**kwargs)

    monkeypatch.setattr(inner, 'LifecycleNode', capture_node)
    description = inner.generate_launch_description()
    declaration, = [action for action in description.entities
                    if isinstance(action, DeclareLaunchArgument)
                    and action.name == 'auto_correction_enabled']
    default_context = LaunchContext()
    declaration.execute(default_context)
    assert default_context.launch_configurations['auto_correction_enabled'] == 'false'
    parameters, = [p for p in captured['parameters']
                   if isinstance(p, dict) and 'auto_correction_enabled' in p]
    evaluated, = evaluate_parameters(inner_context, normalize_parameters([parameters]))
    assert evaluated['auto_correction_enabled'] is (enabled == 'true')
