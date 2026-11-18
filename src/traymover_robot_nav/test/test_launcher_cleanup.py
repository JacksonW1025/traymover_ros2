"""Process cleanup tests without a ROS graph or hardware."""

import os
from pathlib import Path
import signal
import subprocess
import sys
import time


STOP = Path(__file__).resolve().parents[3] / 'scripts/traymover_stop_processes.py'


def test_cleanup_requires_launcher_patterns():
    result = subprocess.run([sys.executable, str(STOP)], capture_output=True, timeout=5)
    assert result.returncode == 2


def test_cleanup_stops_detached_worker_and_preserves_unrelated_shell(tmp_path):
    worker = tmp_path / 'cleanup_fixture_worker.py'
    child_pid = tmp_path / 'child.pid'
    worker.write_text(
        'import os, pathlib, signal, subprocess, sys, time\n'
        'signal.signal(signal.SIGINT, signal.SIG_IGN)\n'
        'signal.signal(signal.SIGTERM, signal.SIG_IGN)\n'
        'child = subprocess.Popen(["sleep", "90"], start_new_session=True)\n'
        'pathlib.Path(sys.argv[1]).write_text(str(child.pid))\n'
        'time.sleep(90)\n')
    parent = subprocess.Popen([sys.executable, str(worker), str(child_pid)])
    unrelated = subprocess.Popen(['bash', '-c', f': {worker.name}; sleep 90; :'],
                                 start_new_session=True)
    child = None
    try:
        deadline = time.monotonic() + 5
        while not child_pid.exists() and time.monotonic() < deadline:
            time.sleep(.02)
        child = int(child_pid.read_text())
        command = ('import runpy, sys; '
                   'main = runpy.run_path(sys.argv[1])["main"]; '
                   'sys.exit(main(sys.argv[2:], marker=b"TRAYMOVER_CLEANUP_TEST=1"))')
        result = subprocess.run([sys.executable, '-c', command, str(STOP),
                                 r'cleanup_fixture_worker\.py'], capture_output=True,
                                text=True, timeout=10)
        assert result.returncode == 0, result.stderr or result.stdout
        assert '(2 before cleanup)' in result.stdout
        assert parent.wait(timeout=3) == -signal.SIGKILL
        assert unrelated.poll() is None
        stat = Path(f'/proc/{child}/stat')
        if stat.exists():
            try:
                assert stat.read_text().rsplit(')', 1)[1].split()[0] == 'Z'
            except FileNotFoundError:
                pass
    finally:
        if parent.poll() is None:
            parent.kill()
        parent.wait(timeout=3)
        if unrelated.poll() is None:
            os.killpg(unrelated.pid, signal.SIGKILL)
        unrelated.wait(timeout=3)
        if child is not None:
            try:
                os.kill(child, signal.SIGKILL)
            except ProcessLookupError:
                pass
