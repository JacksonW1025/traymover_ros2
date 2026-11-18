"""Check sensor frames and acceptance gates for the full Nav2 experiment."""
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'scripts'))
SPEC = importlib.util.spec_from_file_location('option21_full_chain',
                                             ROOT / 'scripts/option21_full_chain.py')
CHAIN = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CHAIN)


def test_raw_scan_matches_actual_robot_sensor_mount():
    import xacro
    tree = ET.fromstring(xacro.process_file(str(
        ROOT / 'src/traymover_robot_description/urdf/traymover.urdf.xacro')).toxml())
    offset = np.zeros(3)
    child = 'laser'
    while child != 'base_link':
        joint, = [j for j in tree.findall('joint') if j.find('child').get('link') == child]
        assert joint.get('type') == 'fixed'
        origin = joint.find('origin')
        assert np.fromstring(origin.get('rpy', '0 0 0'), sep=' ') == pytest.approx([0, 0, 0])
        offset += np.fromstring(origin.get('xyz'), sep=' ')
        child = joint.find('parent').get('link')
    points = np.array([[1., 2., 3.], [-4., -.5, .1]], dtype=np.float32)
    msg = CHAIN.raw_scan(points, Time(sec=10))
    laser_points = np.frombuffer(bytes(msg.data), dtype='<f4').reshape(-1, 4)[:, :3]
    assert msg.header.frame_id == 'laser'
    assert laser_points + offset == pytest.approx(points, abs=1e-6)


def good_counts():
    return {'plan': 10, 'nonzero_cmd': 100, 'odom': 300, 'cloud': 100, 'ndt': 5}


def good_goals():
    return [{'status': GoalStatus.STATUS_SUCCEEDED, 'true_distance_m': .1} for _ in range(2)]


@pytest.mark.parametrize('failure', ['plan', 'nonzero_cmd', 'odom', 'cloud', 'ndt'])
def test_successful_action_alone_does_not_prove_full_chain(failure):
    counts = good_counts()
    counts[failure] = 0
    assert not all(CHAIN.acceptance(counts, good_goals(), [.05], [.05]).values())


@pytest.mark.parametrize('errors,ages', [([.5], [.05]), ([.05], [2.])])
def test_inaccurate_or_stale_localization_fails_acceptance(errors, ages):
    assert not all(CHAIN.acceptance(good_counts(), good_goals(), errors, ages).values())


def test_healthy_closed_loop_meets_acceptance():
    assert all(CHAIN.acceptance(good_counts(), good_goals(), [.05], [.05]).values())


def test_failed_launch_leader_still_stops_surviving_children(monkeypatch):
    calls = []
    def kill_group(pid, signum):
        calls.append((pid, signum))
        if signum == 0:
            raise ProcessLookupError()
    monkeypatch.setattr(CHAIN.os, 'killpg', kill_group)
    # A launch error already reaped the leader; it must still signal the group.
    leader = SimpleNamespace(pid=12345, poll=lambda: 1, wait=lambda timeout: 1)
    CHAIN.stop_group(leader)
    assert (12345, CHAIN.signal.SIGINT) in calls
