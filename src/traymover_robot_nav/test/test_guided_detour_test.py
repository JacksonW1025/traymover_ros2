"""Offline tests for Option 19's single-button terminal runner."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'guided_detour_test.py'
SPEC = importlib.util.spec_from_file_location('option19_runner', SCRIPT)
runner = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(runner)


class Clock:
    def __init__(self):
        self.value = 1.0

    def __call__(self):
        return self.value

    def advance(self, duration=0.05):
        self.value += duration


class FakeNode:
    def __init__(self, healthy=True, finish=True):
        self.healthy = healthy
        self.finish = finish
        self.core = SimpleNamespace(state='NORMAL', reason='waiting')
        self.starts = 0
        self.estops = 0

    def preflight(self):
        status = 'PASS' if self.healthy else 'FAIL'
        return {'Safety inputs': {'status': status}}

    def start_motion(self):
        self.starts += 1
        if not self.healthy:
            return False
        self.core.state = 'NORMAL'
        self.core.reason = 'forward motion started'
        return True

    def assert_manual_estop(self):
        self.estops += 1
        self.core.state = 'ABORTED'
        self.core.reason = 'manual EStop'


def make_session(node, answers):
    clock = Clock()
    answers = iter(answers)
    spins = []

    def spin(unused_node, timeout_sec):
        spins.append(timeout_sec)
        clock.advance(timeout_sec)
        if node.finish and node.starts and len(spins) >= 3:
            node.core.state = 'COMPLETE'
            node.core.reason = 'original straight line restored'

    session = runner.SingleButtonSession(
        node, spin, read_line=lambda: next(answers), clock=clock)
    return session, spins


def test_one_enter_starts_and_runs_to_verified_completion(capsys):
    node = FakeNode()
    session, spins = make_session(node, ['\n'])

    session.run()

    assert node.starts == 1
    assert spins
    output = capsys.readouterr().out
    assert '回到原直线路径' in output


def test_nonempty_input_never_authorizes_motion_until_enter(capsys):
    node = FakeNode()
    session, unused = make_session(node, ['on\n', '\n'])

    session.run()

    assert node.starts == 1
    assert '只需按 Enter' in capsys.readouterr().out


def test_enter_requests_start_without_preflight_workflow():
    node = FakeNode()
    session, unused = make_session(node, ['\n'])
    session.wait_for_button()
    node.start_motion()
    assert node.starts == 1


def test_fault_stops_session_instead_of_claiming_line_return():
    node = FakeNode(finish=False)
    session, unused = make_session(node, ['\n'])
    session.wait_for_button()
    node.start_motion()
    node.core.state = 'FAULT_HOLD'
    node.core.reason = 'line tolerance exceeded'

    with pytest.raises(runner.SessionStopped, match='line tolerance exceeded'):
        session.run_to_completion()


def test_runner_source_has_no_arm_or_staged_report_workflow():
    source = SCRIPT.read_text()
    assert 'Type ARM' not in source
    assert 'SessionReport' not in source
    assert 'begin_stage' not in source
    assert '按 Enter' in source
