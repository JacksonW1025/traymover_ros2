"""Offline operator/session regressions; no ROS graph or chassis is started."""
import importlib.util
import io
import json
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import pytest


SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))
SPEC = importlib.util.spec_from_file_location(
    'guided_session_under_test', SCRIPTS / 'guided_detour_test.py')
runner = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(runner)
from detour_core import Ray, Scan, Velocity  # noqa: E402
from detour_motion import GuidedConfig, GuidedDetourCore, GuidedObservation  # noqa: E402


class Clock:
    def __init__(self):
        self.now = 1.0

    def __call__(self):
        return self.now

    def advance(self, duration=0.05):
        self.now = round(self.now + duration, 8)


class FakeCore:
    def __init__(self):
        self.state = 'READY'
        self.reason = 'ready'
        self.last_motion = {}
        self.events = []
        self.armed = False
        self.arm_calls = []
        self.requests = []
        self.total_forward_command_sec = 0.0
        self.total_rotation_deg = 0.0
        self.old_primitive_discarded = True

    def arm(self, now):
        self.arm_calls.append(now)
        self.armed = True
        return True

    def abort(self, now, reason):
        self.state, self.reason, self.armed = 'ABORTED', reason, False
        self.events.append({'stamp': now, 'event': 'abort', 'reason': reason})

    def obstacle_details(self, observation):
        return {'detected': True, 'confirmation_frames': 3,
                'front_clearance_m': 2.0, 'nearest_range_m': 1.06,
                'supporting_points': 18}

    def choose_side(self, observation):
        return SimpleNamespace(side='LEFT')

    def request_turn(self, now, observation, side):
        self.requests.append(('TURN', side))
        self.state = 'TURN_1'
        return True

    def request_straight(self, now, observation, speed, duration, kind='SHORT_STRAIGHT'):
        self.requests.append((kind, speed, duration))
        self.state = 'STRAIGHT_1'
        return True


class FakeNode:
    def __init__(self, directory, clock=None):
        self.config = GuidedConfig()
        self.core = FakeCore()
        self.clock = clock or Clock()
        self.directory = directory
        self.healthy = True
        self.zeros = 0
        self.close_count = 0
        self.manual_estops = 0
        self.destroyed = False
        self.auto_estop = False
        self.aggregate_estop = False
        self.safety_events = []

    def preflight(self):
        return {'IMU': {'status': 'PASS' if self.healthy else 'FAIL', 'age_sec': 0.03}}

    def observation(self, now):
        return SimpleNamespace(auto_estop=self.auto_estop,
                               aggregate_estop=self.aggregate_estop)

    def publish_zero(self):
        self.zeros += 1

    def close_zero(self):
        self.close_count += 1
        self.publish_zero()

    def assert_manual_estop(self):
        self.manual_estops += 1

    def snapshot(self):
        return {'safety_events': self.safety_events}

    def declare_parameter(self, name, default):
        value = str(self.directory) if name == 'report_directory' else default
        return SimpleNamespace(value=value)

    def destroy_node(self):
        self.destroyed = True


def session_fixture(tmp_path, answers=()):
    clock = Clock()
    node = FakeNode(tmp_path, clock)
    answers = iter(answers)
    report = runner.SessionReport(tmp_path, 'test-commit', {'no_hardware': True})

    def spin(unused_node, timeout_sec):
        clock.advance(timeout_sec)
        if node.core.state in ('TURN_1', 'STRAIGHT_1'):
            node.core.state = 'READY'
            node.core.last_motion = {'completed': True, 'yaw_error_deg': 1.0,
                                     'command_duration_sec': 0.05}

    session = runner.GuidedSession(node, report, spin,
                                   read_line=lambda: next(answers), clock=clock)
    return session, node, report


@pytest.mark.parametrize('statuses,expected', [
    (['PASS', 'PASS'], 'PASS'),
    (['PASS', 'WARN'], 'PASS WITH WARNING'),
    (['PASS', 'INCOMPLETE'], 'INCOMPLETE'),
    (['WARN', 'INCOMPLETE'], 'INCOMPLETE'),
    (['FAIL', 'WARN', 'INCOMPLETE'], 'FAIL'),
])
def test_overall_safety_failure_dominates_partial_and_warning(statuses, expected):
    assert runner.overall_result({str(i): {'status': status}
                                  for i, status in enumerate(statuses)}) == expected


def test_report_is_created_before_arm_and_uses_strict_json(tmp_path):
    session, node, report = session_fixture(tmp_path)
    assert report.path.exists()
    assert not node.core.armed
    report.check('Front obstacle detection', 'WARN', nearest=math.inf,
                 samples=[math.nan, -math.inf, 1.06])
    payload = report.path.read_text()
    decoded = json.loads(payload, parse_constant=lambda value: pytest.fail(value))
    assert decoded['checks']['Front obstacle detection']['nearest'] is None
    assert decoded['checks']['Front obstacle detection']['samples'] == [None, None, 1.06]
    assert decoded['software_commit_sha'] == 'test-commit'
    assert decoded['preliminary'] is True
    assert not list(tmp_path.glob('*.tmp'))
    second = runner.SessionReport(tmp_path, 'second', {})
    assert second.path != report.path
    assert report.path.exists() and second.path.exists()


@pytest.mark.parametrize('answer', ['', 'arm', 'Arm', ' ARM', 'ARM ', 'yes'])
def test_only_exact_arm_can_authorize_session(tmp_path, answer):
    session, node, unused = session_fixture(tmp_path, [answer])
    with pytest.raises(runner.TestAbort, match='ARM not supplied'):
        session.preflight()
    assert node.core.arm_calls == []
    assert node.core.requests == []


def test_exact_arm_is_explicit_and_session_local(tmp_path):
    session, node, unused = session_fixture(tmp_path, ['ARM'])
    session.preflight()
    assert node.core.armed and len(node.core.arm_calls) == 1
    second, new_node, unused = session_fixture(tmp_path)
    assert not new_node.core.armed
    assert node.core.requests == []


def test_preflight_failure_does_not_even_offer_arm(tmp_path):
    session, node, report = session_fixture(tmp_path)
    node.healthy = False
    session.read_line = lambda: pytest.fail('ARM prompt must not appear after failed preflight')
    with pytest.raises(runner.TestAbort, match='PRECHECK FAILED'):
        session.preflight()
    assert report.data['checks']['Preflight sensors']['status'] == 'FAIL'
    assert node.core.arm_calls == [] and node.core.requests == []


def test_health_is_rechecked_after_operator_arm_input(tmp_path):
    session, node, report = session_fixture(tmp_path)

    def arm_after_sensor_disappears():
        node.healthy = False
        return 'ARM'

    session.read_line = arm_after_sensor_disappears
    with pytest.raises(runner.TestAbort, match='PRECHECK FAILED at ARM'):
        session.preflight()
    assert not node.core.armed
    assert report.data['checks']['Preflight sensors']['status'] == 'FAIL'


@pytest.mark.parametrize('answer,reason,manual_count', [
    ('q', 'operator aborted', 0), ('Q', 'operator aborted', 0),
    ('!', 'manual EStop requested', 1), (None, 'input closed', 0),
])
def test_prompt_abort_first_publishes_zero(tmp_path, answer, reason, manual_count):
    session, node, unused = session_fixture(tmp_path, [answer])
    with pytest.raises(runner.TestAbort, match=reason):
        session.prompt('Continue?')
    assert node.zeros == 1
    assert node.manual_estops == manual_count
    assert node.core.requests == []


def test_each_motion_has_its_own_enter_and_observe_is_not_permission(tmp_path):
    session, node, report = session_fixture(tmp_path)
    session.selected_side = 'LEFT'
    calls = []

    def answer():
        calls.append(list(node.core.requests))
        return ''

    session.read_line = answer
    session.turn()
    assert calls == [[]]
    assert node.core.requests == [('TURN', 'LEFT')]
    session.straight()
    assert calls == [[], [('TURN', 'LEFT')], [('TURN', 'LEFT')]]
    assert node.core.requests == [('TURN', 'LEFT'), ('SHORT_STRAIGHT', 0.03, 1.5)]
    assert report.data['checks']['Turn-only primitive']['status'] == 'PASS'
    assert report.data['checks']['Short straight primitive']['status'] == 'PASS'


def test_aborting_next_motion_confirmation_never_sends_straight(tmp_path):
    session, node, unused = session_fixture(tmp_path, ['', '', 'q'])
    session.selected_side = 'LEFT'
    session.turn()
    with pytest.raises(runner.TestAbort, match='operator aborted'):
        session.straight()
    assert node.core.requests == [('TURN', 'LEFT')]


def test_stuck_primitive_has_runner_deadline_and_zero(tmp_path):
    session, node, report = session_fixture(tmp_path, [''])
    session.selected_side = 'LEFT'
    session.spin = lambda unused_node, timeout_sec: node.clock.advance(timeout_sec)
    with pytest.raises(runner.TestAbort, match='primitive failed to terminate'):
        session.turn()
    assert node.core.state == 'ABORTED'
    assert node.zeros >= 2
    assert report.data['checks']['Turn-only primitive']['status'] == 'FAIL'
    assert not session.motion_input


def test_failed_obstacle_detection_retries_are_bounded_without_motion(tmp_path):
    session, node, report = session_fixture(tmp_path, ['', '', ''])
    node.core.obstacle_details = lambda unused: {'detected': False, 'confirmation_frames': 0}
    with pytest.raises(runner.TestAbort, match='could not be confirmed'):
        session.detection()
    assert report.data['checks']['Front obstacle detection']['status'] == 'FAIL'
    assert node.core.requests == []
    assert node.clock() < 14.0


def test_side_rejection_is_bounded_and_never_requests_motion(tmp_path):
    session, node, report = session_fixture(tmp_path, ['', 'n', '', 'n'])
    with pytest.raises(runner.TestAbort, match='side selection not confirmed'):
        session.side_selection()
    assert report.data['checks']['Left/right side selection']['status'] == 'WARN'
    assert node.core.requests == []


def test_side_confirmed_only_on_second_attempt_preserves_warning(tmp_path):
    session, node, report = session_fixture(tmp_path, ['', 'n', '', 'y'])
    session.side_selection()
    assert session.selected_side == 'LEFT'
    assert report.data['checks']['Left/right side selection']['status'] == 'WARN'
    assert node.core.requests == []


def test_side_confirmation_waits_for_ready_without_moving(tmp_path):
    session, node, report = session_fixture(tmp_path, ['', 'y'])
    node.core.state = 'WAITING'

    def plan(unused_node, timeout_sec):
        node.clock.advance(timeout_sec)
        if node.clock() >= 3.0:
            node.core.state = 'READY'

    session.spin = plan
    session.side_selection()
    assert node.clock() >= 3.0
    assert node.core.requests == []
    assert report.data['checks']['Left/right side selection']['status'] == 'PASS'


@pytest.mark.parametrize('event,expected', [('guard', 'PASS'), ('estop', 'WARN')])
def test_guard_race_records_auto_estop_intervention_as_warning(tmp_path, event, expected):
    session, node, report = session_fixture(tmp_path, ['', ''])

    def safety_stop(unused_node, timeout_sec):
        node.clock.advance(timeout_sec)
        if node.core.state == 'STRAIGHT_1':
            node.core.state = 'READY'
            node.core.last_motion = {event: True, 'completed': False,
                                     'command_duration_sec': 0.05}

    session.spin = safety_stop
    session.guard()
    assert report.data['checks']['Detour guard']['status'] == expected
    assert node.core.requests == [('GUARD_TEST', 0.02, 0.5)]


def test_guard_already_blocked_prevents_motion_and_passes(tmp_path):
    session, node, report = session_fixture(tmp_path, ['', ''])

    def refuse_guarded_forward(*args, **kwargs):
        node.core.reason = 'STRAIGHT TEST BLOCKED: detour guard'
        return False

    node.core.request_straight = refuse_guarded_forward
    session.guard()
    assert report.data['checks']['Detour guard']['status'] == 'PASS'
    assert node.core.requests == []


def test_active_estop_checks_use_only_remaining_forward_budget(tmp_path):
    session, node, report = session_fixture(tmp_path, [''])
    node.core.total_forward_command_sec = 2.0

    def interrupt_forward(unused_node, timeout_sec):
        node.clock.advance(timeout_sec)
        if node.core.state == 'STRAIGHT_1':
            node.core.state = 'HOLD_ESTOP'
            node.auto_estop = node.aggregate_estop = True
            node.core.last_motion = {
                'estop': True, 'estop_source': 'AUTO', 'command_duration_sec': 0.05,
                'started_at': session.estop_started, 'ended_at': node.clock(),
                'completed': False,
            }
            node.safety_events.append({'started_at': node.clock(),
                                       'auto_command_was_nonzero': True,
                                       't_auto_state': node.clock(),
                                       't_aggregate_estop': node.clock(),
                                       't_output_zero': node.clock()})

    session.spin = interrupt_forward
    session.estop()
    assert node.core.requests == [('ESTOP_TRIGGER_TEST', 0.02, 3.0)]
    assert report.data['checks']['Mid-detour Auto EStop']['status'] == 'PASS'
    assert report.data['checks']['EStop -> zero software path']['status'] == 'PASS'
    assert node.zeros >= 2


def test_stationary_estop_after_guard_is_incomplete_active_interruption(tmp_path):
    session, node, report = session_fixture(tmp_path)
    prompts = []

    def operator_answer():
        prompts.append(True)
        if len(prompts) == 2:
            node.auto_estop = node.aggregate_estop = True
            node.safety_events.append({'started_at': node.clock(),
                                       't_auto_state': node.clock(),
                                       't_aggregate_estop': node.clock(),
                                       't_output_zero': node.clock()})
        return ''

    def guard_first(unused_node, timeout_sec):
        node.clock.advance(timeout_sec)
        if node.core.state == 'STRAIGHT_1':
            node.core.state = 'READY'
            node.core.last_motion = {'guard': True, 'estop': False,
                                     'command_duration_sec': 0.05, 'completed': False}

    session.read_line, session.spin = operator_answer, guard_first
    session.estop()
    assert report.data['checks']['Mid-detour Auto EStop']['status'] == 'INCOMPLETE'
    assert report.data['checks']['EStop -> zero software path']['status'] == 'PASS'
    assert len(node.core.requests) == 1


def test_manual_stop_then_stationary_auto_cannot_pass_active_auto_or_discard(tmp_path):
    session, node, report = session_fixture(tmp_path)
    prompts = []

    def operator_answer():
        prompts.append(True)
        if len(prompts) == 2:
            node.clock.advance(0.5)
            node.auto_estop = node.aggregate_estop = True
            node.safety_events.append({
                'started_at': node.clock(), 't_auto_state': node.clock(),
                't_aggregate_estop': node.clock(), 't_output_zero': node.clock(),
                'auto_command_was_nonzero': False,
            })
        if len(prompts) == 3:
            node.auto_estop = node.aggregate_estop = False
            node.core.state = 'READY'
        return ''

    def manual_stop(unused_node, timeout_sec):
        node.clock.advance(timeout_sec)
        if node.core.state == 'STRAIGHT_1':
            node.core.state = 'HOLD_ESTOP'
            node.aggregate_estop = True
            node.core.last_motion = {
                'estop': True, 'estop_source': 'MANUAL', 'completed': False,
                'command_duration_sec': 0.05,
                'started_at': session.estop_started, 'ended_at': node.clock(),
            }

    session.read_line, session.spin = operator_answer, manual_stop
    session.estop()
    session.recovery()
    assert report.data['checks']['Mid-detour Auto EStop']['status'] == 'INCOMPLETE'
    assert report.data['checks']['Old primitive discarded']['status'] == 'INCOMPLETE'
    assert report.data['checks']['Post-EStop re-evaluation']['status'] == 'PASS'
    assert len(node.core.requests) == 1


@pytest.mark.parametrize('missing', [
    'estop', 'estop_source', 'command_duration_sec', 'started_at', 'ended_at',
])
def test_active_auto_requires_complete_motion_evidence(tmp_path, missing):
    session, node, unused = session_fixture(tmp_path)
    session.estop_started = 1.0
    session.estop_motion = {
        'estop': True, 'estop_source': 'AUTO', 'command_duration_sec': 0.2,
        'started_at': 1.0, 'ended_at': 2.0,
    }
    node.safety_events = [{'started_at': 1.5, 't_auto_state': 1.5,
                           'auto_command_was_nonzero': True}]
    assert session.active_auto_interruption()
    del session.estop_motion[missing]
    assert not session.active_auto_interruption()


@pytest.mark.parametrize('event', [
    None,
    {'started_at': 1.5, 't_auto_state': 1.5},
    {'started_at': 1.5, 't_auto_state': 1.5, 'auto_command_was_nonzero': False},
    {'started_at': 1.5, 'auto_command_was_nonzero': True},
    {'started_at': 0.5, 't_auto_state': 1.5, 'auto_command_was_nonzero': True},
    {'started_at': 1.5, 't_auto_state': 2.1, 'auto_command_was_nonzero': True},
    {'started_at': 1.5, 't_auto_state': 0.9, 'auto_command_was_nonzero': True},
])
def test_active_auto_rejects_missing_or_unrelated_sensor_event(tmp_path, event):
    session, node, unused = session_fixture(tmp_path)
    session.estop_started = 1.0
    session.estop_motion = {
        'estop': True, 'estop_source': 'AUTO', 'command_duration_sec': 0.2,
        'started_at': 1.0, 'ended_at': 2.0,
    }
    node.safety_events = [] if event is None else [event]
    assert not session.active_auto_interruption()


@pytest.mark.parametrize('timestamp_key', ['t_auto_request', 't_auto_state'])
def test_active_auto_accepts_request_or_state_during_motion(tmp_path, timestamp_key):
    session, node, unused = session_fixture(tmp_path)
    session.estop_started = 1.0
    session.estop_motion = {
        'estop': True, 'estop_source': 'AUTO', 'command_duration_sec': 0.2,
        'started_at': 1.0, 'ended_at': 2.0,
    }
    node.safety_events = [{'started_at': 1.5, timestamp_key: 1.5,
                           'auto_command_was_nonzero': True}]
    assert session.active_auto_interruption()


def test_recovery_waits_at_zero_without_requesting_old_motion(tmp_path):
    session, node, report = session_fixture(tmp_path, [''])
    node.core.state = 'HOLD_ESTOP'
    session.estop_started = 0.0
    session.estop_motion = {'estop': True, 'estop_source': 'AUTO',
                            'command_duration_sec': 0.5, 'started_at': 0.0, 'ended_at': 0.9}
    node.safety_events = [{'started_at': 0.8, 't_auto_state': 0.8,
                           'auto_command_was_nonzero': True}]

    def release(unused_node, timeout_sec):
        node.clock.advance(timeout_sec)
        node.core.state = 'READY'

    session.spin = release
    session.recovery()
    assert node.core.requests == []
    assert report.data['checks']['Old primitive discarded']['status'] == 'PASS'
    assert report.data['checks']['Post-EStop re-evaluation']['status'] == 'PASS'


def test_recovery_detects_unrequested_old_motion_resumption(tmp_path):
    session, node, report = session_fixture(tmp_path, [''])
    session.estop_motion = {'estop': True}

    def bad_release(unused_node, timeout_sec):
        node.clock.advance(timeout_sec)
        node.core.total_forward_command_sec += timeout_sec
        node.core.state = 'READY'

    session.spin = bad_release
    with pytest.raises(runner.TestAbort, match='unsafe recovery'):
        session.recovery()
    assert report.data['checks']['Old primitive discarded']['status'] == 'FAIL'
    assert node.core.state == 'ABORTED'


def test_earlier_estop_zero_is_not_proof_for_current_estop(tmp_path):
    session, node, unused = session_fixture(tmp_path)
    session.estop_started = 4.0
    node.safety_events = [
        {'started_at': 1.0, 't_auto_state': 1.0,
         't_aggregate_estop': 1.1, 't_output_zero': 1.2},
        {'started_at': 5.0, 't_auto_state': 5.0, 't_aggregate_estop': 5.1},
    ]
    assert not session.software_zero_observed()
    node.safety_events[-1]['t_output_zero'] = 5.2
    assert session.software_zero_observed()


@pytest.mark.parametrize('bad_event', [
    {'t_output_zero': 4.9},
    {'t_output_zero': 5.2, 't_nonzero_after_aggregate': 5.3},
])
def test_estop_zero_proof_rejects_wrong_order_or_later_nonzero(tmp_path, bad_event):
    session, node, unused = session_fixture(tmp_path)
    session.estop_started = 4.0
    node.safety_events = [{'started_at': 5.0, 't_auto_state': 5.0,
                           't_aggregate_estop': 5.1, **bad_event}]
    assert not session.software_zero_observed()


@pytest.mark.parametrize('answer', ['q\n', '!\n', ''])
def test_operator_can_interrupt_during_motion(tmp_path, monkeypatch, answer):
    session, node, unused = session_fixture(tmp_path)
    session.read_line = None
    session.motion_input = True
    monkeypatch.setattr(runner.sys, 'stdin', io.StringIO(answer))
    monkeypatch.setattr(runner.select, 'select', lambda *args: ([runner.sys.stdin], [], []))
    with pytest.raises(runner.TestAbort, match='manual EStop requested'):
        session.pump()
    assert node.manual_estops == 1


def test_enter_during_motion_cannot_authorize_a_future_primitive(tmp_path, monkeypatch):
    session, node, unused = session_fixture(tmp_path)
    session.read_line = None
    session.motion_input = True
    stdin = io.StringIO('\n')
    monkeypatch.setattr(runner.sys, 'stdin', stdin)
    monkeypatch.setattr(runner.select, 'select', lambda *args: ([stdin], [], []))
    session.pump()
    assert stdin.read() == ''
    assert node.core.requests == []


def test_report_write_failure_cannot_prevent_shutdown_zero(tmp_path, monkeypatch):
    session, node, report = session_fixture(tmp_path)
    node.snapshot = lambda: {'faults': ['nonzero output after aggregate EStop']}

    def disk_failed():
        raise OSError('report volume became unavailable')

    monkeypatch.setattr(report, 'save', disk_failed)
    with pytest.raises(OSError, match='report volume'):
        session.finish('sensor fault')
    assert node.close_count == 1 and node.zeros > 0
    assert node.core.state == 'ABORTED'


def test_finish_preserves_boundaries_and_flushes_zero(tmp_path, capsys):
    session, node, report = session_fixture(tmp_path)
    session.finish('operator aborted')
    output = capsys.readouterr().out
    assert node.close_count == 1 and node.zeros > 0
    assert node.core.state == 'ABORTED'
    assert 'Overall preliminary result         INCOMPLETE' in output
    unsupported = output.split('This test DOES NOT validate:')[1]
    for limitation in runner.DOES_NOT_VALIDATE:
        assert limitation in unsupported
        assert limitation in report.data['does_not_validate']
    assert 'physical stopping distance' not in report.data['validates']
    assert 'map-based detour' not in report.data['validates']
    assert 'Only completed PASS checks' in output


@pytest.mark.parametrize('error', [
    KeyboardInterrupt(), EOFError(), RuntimeError('test exception'),
])
def test_main_interrupt_or_exception_always_flushes_zero(tmp_path, monkeypatch, error):
    node = FakeNode(tmp_path)
    rclpy = ModuleType('rclpy')
    rclpy.init = lambda **kwargs: None
    rclpy.spin_once = lambda *args, **kwargs: None
    rclpy.ok = lambda: True
    shutdowns = []
    rclpy.shutdown = lambda: shutdowns.append(True)
    signals = ModuleType('rclpy.signals')
    signals.SignalHandlerOptions = SimpleNamespace(NO=0)
    adapter = ModuleType('guided_detour_node')
    adapter.GuidedDetourNode = lambda: node
    monkeypatch.setitem(sys.modules, 'rclpy', rclpy)
    monkeypatch.setitem(sys.modules, 'rclpy.signals', signals)
    monkeypatch.setitem(sys.modules, 'guided_detour_node', adapter)
    monkeypatch.setattr(runner.signal, 'signal', lambda *args: runner.signal.SIG_DFL)

    def interrupted_session(self):
        raise error

    monkeypatch.setattr(runner.GuidedSession, 'run', interrupted_session)
    assert runner.main([]) == 1
    assert node.close_count == 1 and node.zeros >= 1
    assert node.destroyed and shutdowns == [True]
    assert node.core.state == 'ABORTED'
    result = json.loads(next(tmp_path.glob('detour_test_*.json')).read_text())
    assert result['overall'] == ('FAIL' if isinstance(error, RuntimeError) else 'INCOMPLETE')


def test_real_core_wait_clear_remains_zero_despite_human_prompt_delay(tmp_path):
    """Waiting at the removal prompt may let WAITING advance into planning."""
    session, node, report = session_fixture(tmp_path)
    node.core = GuidedDetourCore(node.config)
    session.core = node.core
    front = [1.06]
    commands = []

    def observation(now):
        rays = tuple(Ray(0.0, 0.0, math.radians(degrees),
                         front[0] if abs(degrees) <= 6 else 5.0)
                     for degrees in range(-180, 180, 2))
        return GuidedObservation(
            lidar=Scan(now, rays), realsense=Scan(now, rays),
            imu_yaw=0.0, imu_stamp=now, tf_healthy=True,
            collision_monitor_fresh=True, driver_fresh=True,
            estop_fresh=True, auto_reason='NONE')

    node.observation = observation

    def spin(unused_node, timeout_sec):
        node.clock.advance(timeout_sec)
        commands.append(node.core.tick(node.clock(), observation(node.clock())))

    session.spin = spin
    for unused in range(12):
        spin(node, 0.05)
    prompt_number = [0]

    def operator_input():
        prompt_number[0] += 1
        if prompt_number[0] == 2:
            # Operator spends several seconds reading the removal instruction.
            for unused in range(100):
                spin(node, 0.05)
            front[0] = 5.0
        return ''

    session.read_line = operator_input
    session.wait_clear()
    assert report.data['checks']['WAIT transition']['status'] == 'PASS'
    assert report.data['checks']['Obstacle clear recovery']['status'] == 'PASS'
    assert set(commands) == {Velocity()}
    assert not node.core.armed
