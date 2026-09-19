"""Exercise height-weighting menu and launch wiring without starting ROS nodes."""

import importlib.util
from pathlib import Path
import subprocess

import pytest
import yaml

from test_nav_simple_config import _run_option10_prompt


PKG = Path(__file__).resolve().parents[1]
ROOT = PKG.parents[1]


def _prompt(answers, repeats=1):
    launcher = (ROOT / 'scripts/traymover.sh').read_text()
    body = launcher.split('prompt_ndt_height_boost() {', 1)[1].split('\n}', 1)[0]
    script = 'prompt_ndt_height_boost() {' + body + '\n}\n'
    script += ('prompt_ndt_height_boost || exit $?\n'
               'printf "RESULT=%s,%s\\n" "$NDT_HEIGHT_BOOST_ENABLED" '
               '"$NDT_HEIGHT_BOOST_MIN_Z"\n') * repeats
    return subprocess.run(['bash', '-u', '-c', script], input=answers,
                          text=True, capture_output=True, timeout=10)


@pytest.mark.parametrize('answer,canonical', [
    ('0', '0.0'), ('2', '2.0'), ('02.500', '2.5'), ('.75', '0.75'),
    (' 2.25 ', '2.25'),
])
def test_prompt_accepts_nonnegative_decimal(answer, canonical):
    result = _prompt(answer + '\n')
    assert result.returncode == 0, result.stderr
    assert f'RESULT=true,{canonical}' in result.stdout
    assert 'base_link' in result.stdout


@pytest.mark.parametrize('answer', ['-1', 'nan', 'inf', '1e309', '1e2',
                                    '9' * 400, '1; echo unsafe', '$(echo unsafe)'])
def test_prompt_rejects_invalid_input_and_can_disable(answer):
    result = _prompt(answer + '\n\n')
    assert result.returncode == 0, result.stderr
    assert 'Enter a finite nonnegative decimal' in result.stdout
    assert 'RESULT=false,2.0' in result.stdout


def test_prompt_eof_fails_and_disabled_choice_resets_previous_run():
    result = _prompt('')
    assert result.returncode != 0
    assert 'RESULT=' not in result.stdout
    result = _prompt('2.5\n\n', repeats=2)
    assert result.returncode == 0, result.stderr
    assert 'RESULT=true,2.5' in result.stdout
    assert result.stdout.endswith('RESULT=false,2.0\n')


@pytest.mark.parametrize('answer,enabled,threshold', [
    ('2.50', 'true', '2.5'), ('', 'false', '2.0'),
])
def test_option10_records_and_passes_settings(tmp_path, answer, enabled, threshold):
    result, _, hardware = _run_option10_prompt(tmp_path, f'1\nn\nn\n{answer}\n')
    assert result.returncode == 0, result.stdout + result.stderr
    assert hardware
    commands = (tmp_path / 'hardware').read_text()
    assert f'height_boost_enabled:={enabled}' in commands
    assert f'height_boost_min_z:={threshold}' in commands
    info, = (tmp_path / 'nav_logs').glob('*/run_info.txt')
    settings = dict(line.split('=', 1) for line in info.read_text().splitlines())
    assert settings['height_boost_enabled'] == enabled
    assert settings['height_boost_min_z'] == threshold
    assert settings['height_boost_frame'] == 'base_link'
    assert settings['height_boost_extra_copies'] == '1'


def test_option10_height_prompt_eof_aborts_before_hardware(tmp_path):
    result, _, hardware = _run_option10_prompt(tmp_path, '1\nn\nn\n')
    assert result.returncode != 0
    assert not hardware
    assert not (tmp_path / 'nav_logs').exists()


def test_default_configuration_keeps_height_weighting_disabled():
    params = yaml.safe_load((PKG / 'config/localization.yaml').read_text())
    params = params['/**']['ros__parameters']
    assert params['height_boost_enabled'] is False
    assert params['height_boost_min_z'] == 2.0


def _load_launch(name, monkeypatch):
    spec = importlib.util.spec_from_file_location(name, PKG / 'launch' / f'{name}.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda package: str(PKG))
    return module


@pytest.mark.parametrize('enabled,threshold', [('true', '2.75'), ('false', '2.0')])
def test_launch_propagates_typed_runtime_parameters(monkeypatch, enabled, threshold):
    pytest.importorskip('launch_ros')
    pytest.importorskip('nav2_common')
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
    from launch.substitutions import LaunchConfiguration
    from launch_ros.utilities import evaluate_parameters, normalize_parameters

    outer = _load_launch('traymover_nav', monkeypatch).generate_launch_description()
    defaults = {action.name: action.default_value for action in outer.entities
                if isinstance(action, DeclareLaunchArgument)}
    context = LaunchContext()
    for name in ('height_boost_enabled', 'height_boost_min_z'):
        context.launch_configurations[name] = ''.join(
            substitution.perform(context) for substitution in defaults[name])
    assert LaunchConfiguration('height_boost_enabled').perform(context) == 'false'
    assert LaunchConfiguration('height_boost_min_z').perform(context) == '2.0'
    context.launch_configurations.update(
        height_boost_enabled=enabled, height_boost_min_z=threshold)
    includes = [dict(action.launch_arguments) for action in outer.entities
                if isinstance(action, IncludeLaunchDescription)]
    arguments, = [args for args in includes if 'height_boost_enabled' in args]
    inner_context = LaunchContext()
    for name in ('height_boost_enabled', 'height_boost_min_z'):
        inner_context.launch_configurations[name] = arguments[name].perform(context)

    inner = _load_launch('lidar_localization', monkeypatch)
    real_lifecycle_node = inner.LifecycleNode
    captured = {}

    def capture_node(**kwargs):
        captured.update(kwargs)
        return real_lifecycle_node(**kwargs)

    monkeypatch.setattr(inner, 'LifecycleNode', capture_node)
    inner.generate_launch_description()
    height_params, = [p for p in captured['parameters']
                      if isinstance(p, dict) and 'height_boost_enabled' in p]
    evaluated, = evaluate_parameters(inner_context, normalize_parameters([height_params]))
    assert evaluated['height_boost_enabled'] is (enabled == 'true')
    assert type(evaluated['height_boost_min_z']) is float
    assert evaluated['height_boost_min_z'] == float(threshold)
