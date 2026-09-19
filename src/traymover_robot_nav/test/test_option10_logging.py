"""Run the live logging supervisor against fake sensors/navigation only."""

import os
from pathlib import Path
import shutil
import signal
import subprocess
import time

import pytest


ROOT = Path(__file__).resolve().parents[3]


def test_option0_includes_collectors_in_cleanup():
    launcher = (ROOT / 'scripts/traymover.sh').read_text()
    patterns = launcher.split('KILL_PATTERNS=(', 1)[1].split('\n)', 1)[0]
    cleanup = launcher.split('kill_previous() {', 1)[1].split('\n}', 1)[0]
    script = '''
pkill() { printf '%s\\n' "$*"; }
sleep() { :; }
stop_rosbag_recorders() { :; }
reindex_missing_rosbag_metadata() { :; }
KILL_PATTERNS=(''' + patterns + '\n)\nkill_previous() {' + cleanup + '\n}\nkill_previous\n'
    result = subprocess.run(['bash', '-u', '-c', script],
                            capture_output=True, text=True, timeout=5)
    assert result.returncode == 0, result.stderr
    for name in ('run_nav_logged.sh', 'ndt_diagnostics.py', 'fastlio_diagnostics.py'):
        assert f'-f scripts/{name}' in result.stdout


@pytest.mark.parametrize('entrypoint', [
    'turn_on_traymover_robot/traymover_robot_node',
    'hipnuc_imu/talker',
])
def test_option0_matches_installed_hardware_entrypoint_only(entrypoint):
    """The console-script process survives parent launch exit; match its actual argv."""
    launcher = (ROOT / 'scripts/traymover.sh').read_text()
    patterns = launcher.split('KILL_PATTERNS=(', 1)[1].split('\n)', 1)[0]
    # Ask the same POSIX regex engine used by pkill, without signalling any process.
    script = 'KILL_PATTERNS=(' + patterns + '\n)\n'
    script += '''
for pattern in "${KILL_PATTERNS[@]}"; do
    case "$pattern" in
        *"lib/$ENTRYPOINT"*)
            printf '%s\\n' "$CHASSIS_ARGV" | grep -Eq "$pattern" || exit 1
            if printf '%s\\n' "$UNRELATED_ARGV" | grep -Eq "$pattern"; then exit 2; fi
            exit 0 ;;
    esac
done
exit 3
'''
    executable = str(ROOT / 'install' / entrypoint.split('/')[0] / 'lib' / entrypoint)
    env = dict(os.environ,
               ENTRYPOINT=entrypoint,
               CHASSIS_ARGV=f'/usr/bin/python3 {executable} --ros-args',
               UNRELATED_ARGV=f'/usr/bin/python3 {executable}_diagnostic')
    result = subprocess.run(['bash', '-u'], input=script, env=env,
                            capture_output=True, text=True, timeout=5)
    assert result.returncode == 0, result.stderr


def test_terminal_wrapper_preserves_quoted_log_directory(tmp_path):
    launcher = (ROOT / 'scripts/traymover.sh').read_text()
    body = launcher.split('spawn_in_terminal() {', 1)[1].split('\n}', 1)[0]
    setup = tmp_path / 'setup.bash'
    setup.write_text(':\n')
    log_dir = tmp_path / "logs ' quote"
    env = dict(os.environ, ROS_LOG_DIR_DEFAULT=str(log_dir),
               ROS_DISTRO_SETUP=str(setup), WS_SETUP=str(setup))
    script = 'pick_terminal() { :; }\nspawn_in_terminal() {' + body + '\n}\n'
    script += '''spawn_in_terminal test 'printf "%s\\n" "$ROS_LOG_DIR"'\n'''
    result = subprocess.run(['bash', '-u', '-c', script], env=env, input='\n',
                            capture_output=True, text=True, timeout=5)
    assert result.returncode == 0, result.stderr
    assert log_dir.is_dir()
    assert str(log_dir) in result.stdout


@pytest.mark.parametrize('stop_signal', [None, signal.SIGTERM, signal.SIGHUP])
def test_logged_navigation_lifetime_and_clock(tmp_path, stop_signal):
    scripts = tmp_path / 'scripts'
    scripts.mkdir()
    wrapper = scripts / 'run_nav_logged.sh'
    shutil.copyfile(ROOT / 'scripts/run_nav_logged.sh', wrapper)
    monitor = '''
import os, pathlib, signal, sys, time
out = pathlib.Path(sys.argv[sys.argv.index('--output-dir') + 1])
name = out.name
(out / (name + '_events.log')).write_text('START')
(out / 'arguments.txt').write_text(repr(sys.argv))
(out / 'pid').write_text(str(os.getpid()))
def stop(*args):
    (out / 'stopped').touch()
    sys.exit(0)
signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)
while True:
    time.sleep(0.1)
'''
    for name in ('ndt', 'fastlio'):
        (scripts / f'{name}_diagnostics.py').write_text(monitor)
    # The snapshot tool is deliberately fake; no ROS graph is queried here.
    ros2 = scripts / 'ros2'
    ros2.write_text('#!/bin/sh\nprintf "%s:\\n  ros__parameters: {}\\n" "$3"\n')
    ros2.chmod(0o755)
    run_dir = tmp_path / "logs with space and 'quote"
    nav = scripts / 'fake_nav.py'
    nav.write_text('''
import os, pathlib, sys, time
out = pathlib.Path(sys.argv[1])
(out / 'nav_pid').write_text(str(os.getpid()))
print('NAV_STDOUT_MARKER', flush=True)
print('NAV_STDERR_MARKER', file=sys.stderr, flush=True)
time.sleep(float(sys.argv[2]))
''')
    env = dict(os.environ, PATH=str(scripts) + os.pathsep + os.environ['PATH'])
    process = subprocess.Popen(
        ['bash', str(wrapper), str(run_dir), 'python3', str(nav), str(run_dir),
         '1' if stop_signal is None else '60'], env=env,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        deadline = time.monotonic() + 5
        snapshot = run_dir / 'parameters/pointcloud_nav_preprocessor.yaml'
        while not snapshot.exists() and time.monotonic() < deadline:
            time.sleep(0.05)
        assert snapshot.exists()
        if stop_signal is not None:
            process.send_signal(stop_signal)
        stdout, stderr = process.communicate(timeout=12)
        expected = 0 if stop_signal is None else 128 + stop_signal
        assert process.returncode == expected, stdout + stderr
        assert 'NAV_STDOUT_MARKER' in stdout
        log = (run_dir / 'processes/nav_stack.log').read_text()
        assert 'NAV_STDOUT_MARKER' in log and 'NAV_STDERR_MARKER' in log
        for name in ('ndt', 'fastlio'):
            assert "'--use-sim-time', 'false'" in (run_dir / name / 'arguments.txt').read_text()
            assert (run_dir / name / 'stopped').exists()
            pid = int((run_dir / name / 'pid').read_text())
            with pytest.raises(ProcessLookupError):
                os.kill(pid, 0)
    finally:
        if process.poll() is None:
            process.terminate()
            process.communicate(timeout=12)
