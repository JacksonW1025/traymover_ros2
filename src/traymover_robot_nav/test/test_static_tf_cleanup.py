"""Prevent old navigation TF publishers from reparenting the next run's base."""

import importlib.util
import os
from pathlib import Path
import re
import signal
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location('cleanup_static_tf', ROOT / 'scripts/cleanup_static_tf.py')
cleanup = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cleanup)


def process(proc, pid, node, parent=1, domain='0', executable='static_transform_publisher'):
    entry = proc / str(pid)
    entry.mkdir()
    args = [f'/opt/ros/humble/lib/tf2_ros/{executable}', '--ros-args', '-r', f'__node:={node}']
    (entry / 'cmdline').write_bytes(('\0'.join(args) + '\0').encode())
    (entry / 'environ').write_bytes(f'ROS_DOMAIN_ID={domain}\0'.encode())
    fields = ['S', str(parent)] + ['0'] * 17 + ['12345']
    (entry / 'stat').write_text(f'{pid} (static TF) ' + ' '.join(fields))
    (proc / str(parent)).mkdir(exist_ok=True)
    (proc / str(parent) / 'comm').write_text('systemd' if parent == 1 else 'python3')


def test_orphan_selection_is_scoped_by_node_executable_domain_and_parent(tmp_path):
    process(tmp_path, 10, 'option20_base_height')
    process(tmp_path, 11, 'static_body_to_base_link')
    process(tmp_path, 12, 'unrelated_tf')
    process(tmp_path, 13, 'option20_base_height', domain='9')
    process(tmp_path, 14, 'option20_base_height', parent=2)
    process(tmp_path, 15, 'option20_base_height', executable='bash')
    selected = {pid: orphan for pid, _, orphan, _ in cleanup.find_publishers(tmp_path, '0')}
    assert selected == {10: True, 11: True, 14: False}
    assert cleanup.find_publishers(tmp_path, '0', uid=os.getuid() + 1) == []
    assert cleanup.same_process(10, '12345', tmp_path)
    assert not cleanup.same_process(10, 'different-start-time', tmp_path)


def test_active_navigation_is_not_killed(monkeypatch):
    monkeypatch.setattr(cleanup, 'find_publishers', lambda: [
        (10, 'option20_base_height', True, '1'),
        (11, 'static_body_to_base_link', False, '2'),
    ])
    stopped = []
    monkeypatch.setattr(cleanup, 'stop_orphan', lambda *args: stopped.append(args))
    assert cleanup.main() == 1
    assert stopped == []


def test_cleanup_stops_only_selected_orphans_and_handles_failure(monkeypatch):
    monkeypatch.setattr(cleanup, 'find_publishers', lambda: [
        (10, 'option20_base_height', True, '1'),
    ])
    stopped = []
    monkeypatch.setattr(cleanup, 'stop_orphan', lambda *args: stopped.append(args) or True)
    assert cleanup.main() == 0
    assert stopped == [(10, '1')]
    monkeypatch.setattr(cleanup, 'stop_orphan', lambda *args: False)
    assert cleanup.main() == 1


def test_menu_cleanup_matches_managed_static_nodes_but_not_unrelated_nodes():
    launcher = (ROOT / 'scripts/traymover.sh').read_text()
    patterns = launcher.split('KILL_PATTERNS=(', 1)[1].split('\n)', 1)[0]
    # Let bash expand exactly the array used by the real cleanup path.
    result = subprocess.run(['bash', '-c', 'p=(' + patterns + '\n)\nprintf "%s\\n" "${p[@]}"'],
                            check=True, text=True, capture_output=True)
    pattern, = [p for p in result.stdout.splitlines() if 'static_transform_publisher' in p]
    pattern = pattern.replace('[[:space:]]', r'\s')
    for node in cleanup.MANAGED_NODES:
        command = f'/opt/ros/humble/lib/tf2_ros/static_transform_publisher 0 0 0 --ros-args -r __node:={node}'
        assert re.search(pattern, command)
        assert not re.search(pattern, command + '_unrelated')
    action = launcher.split('action_start_nav() {', 1)[1].split('\n}', 1)[0]
    assert action.index('cleanup_static_tf.py') < action.index('Starting chassis + IMU')
    assert action.index('cleanup_static_tf.py') < action.index('Generating runtime 2D nav map')


def test_active_tf_prevents_map_rewrite_and_hardware_start(tmp_path):
    from test_nav_simple_config import _run_option10_prompt
    result, modes, hardware = _run_option10_prompt(tmp_path, '1\nn\n', cleanup_status=1)
    assert result.returncode != 0
    assert modes == []
    assert not hardware
    assert not list((tmp_path / 'maps').glob('*.yaml'))


def test_pidfd_stops_only_the_verified_process(monkeypatch):
    sent, closed = [], []
    monkeypatch.setattr(cleanup.os, 'pidfd_open', lambda pid: 77)
    monkeypatch.setattr(cleanup.os, 'close', closed.append)
    monkeypatch.setattr(cleanup.signal, 'pidfd_send_signal', lambda *args: sent.append(args))
    monkeypatch.setattr(cleanup, 'same_process', lambda *args: False)
    assert cleanup.stop_orphan(10, '12345')
    assert not sent and closed == [77]  # Reused PID: close handle without signalling.
    alive = iter([True, False, False, False])
    monkeypatch.setattr(cleanup, 'same_process', lambda *args: next(alive))
    assert cleanup.stop_orphan(10, '12345')
    assert sent == [(77, signal.SIGTERM)]
    assert closed == [77, 77]


def test_conflicting_base_parent_reproduces_the_disconnected_tree():
    tf2_ros = pytest.importorskip('tf2_ros')
    from geometry_msgs.msg import TransformStamped
    from rclpy.time import Time

    def put(buffer, parent, child, z=0.0):
        transform = TransformStamped()
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.transform.rotation.w = 1.0
        transform.transform.translation.z = z
        buffer.set_transform_static(transform, 'test')

    buffer = tf2_ros.Buffer()
    put(buffer, 'odom', 'camera_init')
    put(buffer, 'camera_init', 'body')
    put(buffer, 'body', 'base_link', 0.12)
    assert buffer.can_transform('odom', 'base_link', Time())
    put(buffer, 'base_footprint', 'base_link', 0.20)
    assert not buffer.can_transform('odom', 'base_link', Time())
    # A fresh session after orphan cleanup receives only the intended parent.
    buffer = tf2_ros.Buffer()
    put(buffer, 'odom', 'camera_init')
    put(buffer, 'camera_init', 'body')
    put(buffer, 'body', 'base_link', 0.12)
    transform = buffer.lookup_transform('odom', 'base_link', Time())
    assert transform.transform.translation.z == pytest.approx(0.12)
