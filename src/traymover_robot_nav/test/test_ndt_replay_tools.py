"""Exercise replay controls and logging without starting robot hardware."""

import importlib.util
from pathlib import Path
import subprocess
import sys
import time
from types import SimpleNamespace

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[3]
REPLAY = ROOT / 'scripts/test_nav_replay.sh'


def test_help_does_not_require_maps_or_ros():
    result = subprocess.run(
        ['bash', str(REPLAY), '--help'], capture_output=True, text=True, timeout=5)
    assert result.returncode == 0
    assert '--mode static|continuous' in result.stdout


@pytest.mark.parametrize('arguments', [
    ['--mode', 'invalid', 'map', 'bag'],
    ['map', 'bag', '-1'],
    ['map', 'bag', 'nan'],
])
def test_invalid_arguments_fail_before_starting_ros(arguments):
    result = subprocess.run(
        ['bash', str(REPLAY), *arguments], capture_output=True, text=True, timeout=5)
    assert result.returncode == 2
    assert 'Run logs:' not in result.stdout


def test_stop_resumes_and_reaps_a_paused_process_group():
    script = REPLAY.read_text()
    stop_function = script.split('stop_process() {', 1)[1].split('\n}', 1)[0]
    # This is a real stopped process, independent of ROS or attached hardware.
    body = '''
set -eu
stop_process() {''' + stop_function + '''
}
setsid sleep 60 &
pid=$!
trap 'kill -KILL -- "-$pid" 2>/dev/null || true' EXIT
sleep 0.1
kill -STOP -- "-$pid"
stop_process "$pid"
! kill -0 "$pid" 2>/dev/null
'''
    result = subprocess.run(['bash', '-c', body], capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stderr


def test_candidate_parameters_are_inert():
    candidate = ROOT / 'src/traymover_robot_nav/config/ndt_replay_candidates.yaml'
    text = candidate.read_text()
    assert yaml.safe_load(text) is None
    assert all(not line.strip() or line.lstrip().startswith('#') for line in text.splitlines())


@pytest.mark.parametrize('module_name', ['ndt_diagnostics', 'fastlio_diagnostics'])
@pytest.mark.parametrize('clock_arguments,expected', [
    ([], 'true'),
    (['--use-sim-time', 'true'], 'true'),
    (['--use-sim-time', 'false'], 'false'),
])
def test_diagnostics_clock_arguments(monkeypatch, module_name, clock_arguments, expected):
    pytest.importorskip('rclpy')
    spec = importlib.util.spec_from_file_location(module_name, ROOT / 'scripts' / f'{module_name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(sys, 'argv', [module_name, '--output-dir', '/tmp', *clock_arguments])
    assert module.parse_args().use_sim_time == expected


@pytest.mark.parametrize('module_name', ['ndt_diagnostics', 'fastlio_diagnostics'])
def test_diagnostics_reject_invalid_clock_argument(monkeypatch, module_name):
    pytest.importorskip('rclpy')
    spec = importlib.util.spec_from_file_location(module_name, ROOT / 'scripts' / f'{module_name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(
        sys, 'argv', [module_name, '--output-dir', '/tmp', '--use-sim-time', 'invalid'])
    with pytest.raises(SystemExit) as error:
        module.parse_args()
    assert error.value.code == 2


@pytest.mark.parametrize('use_sim_time', ['true', 'false'])
@pytest.mark.parametrize('module_name,class_name', [
    ('ndt_diagnostics', 'NdtDiagnostics'),
    ('fastlio_diagnostics', 'FastlioDiagnostics'),
])
def test_diagnostics_report_without_sim_clock(
        tmp_path, monkeypatch, module_name, class_name, use_sim_time):
    rclpy = pytest.importorskip('rclpy')
    from rclpy.clock import ClockType

    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros'))
    spec = importlib.util.spec_from_file_location(module_name, ROOT / 'scripts' / f'{module_name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    args = SimpleNamespace(
        output_dir=str(tmp_path), interval_sec=0.5, use_sim_time=use_sim_time,
        cloud_topic='/test/ndt_cloud', lidar_topic='/test/lidar',
        imu_topic='/test/imu', odom_topic='/test/odom', lidar_qos='sensor_data')
    rclpy.init()
    monitor = None
    try:
        monitor = getattr(module, class_name)(args)
        assert monitor.timer.clock.clock_type == ClockType.STEADY_TIME
        deadline = time.monotonic() + 3.0
        prefix = 'ndt' if module_name == 'ndt_diagnostics' else 'fastlio'
        events = tmp_path / f'{prefix}_events.log'
        while 'event=REPORT' not in events.read_text() and time.monotonic() < deadline:
            rclpy.spin_once(monitor.node, timeout_sec=0.1)
        assert 'event=REPORT' in events.read_text()
        assert monitor.node.get_parameter('use_sim_time').value is (use_sim_time == 'true')
        if use_sim_time == 'false':
            assert monitor.ros_time() > 0.0
        monitor.close()
        monitor.close()  # Shutdown must be idempotent.
        assert events.read_text().count('event=STOP') == 1
    finally:
        if monitor is not None:
            monitor.close()
        if rclpy.ok():
            rclpy.shutdown()
