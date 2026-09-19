#!/usr/bin/env python3
"""Foreground, human-paced Option 19 session; hardware lives in a separate launch.

The UI never publishes a nonzero command. All motion requests pass through the
same production detour core, with guided limits and explicit phase permission.
"""
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import select
import signal
import sys
import time


VALIDATES = (
    'local obstacle detection', 'wait/clear state handling',
    'simple detour side selection', 'low-speed turn primitive',
    'low-speed short forward primitive', 'detour guard behavior',
    'Auto EStop interruption', 'post-EStop re-evaluation',
)
DOES_NOT_VALIDATE = (
    'global Nav2 replanning', 'map-based detour', 'full obstacle bypass',
    'original navigation goal recovery', 'airport-scale operation',
    'physical stopping distance',
)
CHECKS = (
    'Preflight sensors', 'Front obstacle detection', 'WAIT transition',
    'Obstacle clear recovery', 'Left/right side selection', 'Turn-only primitive',
    'Short straight primitive', 'Detour guard', 'Mid-detour Auto EStop',
    'EStop -> zero software path', 'Old primitive discarded',
    'Post-EStop re-evaluation',
)


class TestAbort(Exception):
    """An operator abort leaves untested checks INCOMPLETE."""

    __test__ = False


def overall_result(checks):
    statuses = [item['status'] for item in checks.values()]
    if 'FAIL' in statuses:
        return 'FAIL'
    if any(status not in ('PASS', 'WARN') for status in statuses):
        return 'INCOMPLETE'
    return 'PASS WITH WARNING' if 'WARN' in statuses else 'PASS'


def json_safe(value):
    """Strict JSON: unknown/inf ranges are null, never nonstandard Infinity."""
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


class SessionReport:
    def __init__(self, directory, commit, parameters):
        now = datetime.now(timezone.utc)
        self.path = Path(directory).expanduser() / (
            'detour_test_' + now.strftime('%Y%m%d_%H%M%S_%f') + '.json')
        self.data = {
            'timestamp': now.isoformat(), 'software_commit_sha': commit,
            'parameters': parameters, 'preliminary': True,
            'time_basis': 'event times are local monotonic seconds; not physical braking time',
            'checks': {name: {'status': 'INCOMPLETE'} for name in CHECKS},
            'stages': {f'stage_{index}': {'status': 'INCOMPLETE'} for index in range(10)},
            'faults': [], 'validates': VALIDATES,
            'does_not_validate': DOES_NOT_VALIDATE,
        }
        source_dir = Path(__file__).resolve().parent
        self.data['software_source_sha256'] = {
            path.name: hashlib.sha256(path.read_bytes()).hexdigest()
            for pattern in ('detour_*.py', 'guided_detour_*.py')
            for path in sorted(source_dir.glob(pattern))
        }
        # Verify report storage before ARM; failed storage must never enable motion.
        self.save()

    def check(self, name, status, detail='', **metrics):
        self.data['checks'][name] = {'status': status, 'detail': detail, **metrics}
        self.save()

    def save(self):
        self.data['overall'] = overall_result(self.data['checks'])
        for stage in self.data['stages'].values():
            if stage.get('results'):
                stage['status'] = overall_result({
                    name: {'status': status} for name, status in stage['results'].items()})
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix('.json.tmp')
        temporary.write_text(json.dumps(json_safe(self.data), indent=2,
                                        ensure_ascii=False, allow_nan=False) + '\n')
        temporary.replace(self.path)


class GuidedSession:
    """Small UI adapter; spin/input functions are injectable for offline tests."""

    def __init__(self, node, report, spin, read_line=None, clock=time.monotonic):
        self.node, self.core, self.config = node, node.core, node.config
        self.report, self.spin, self.clock = report, spin, clock
        self.read_line = read_line
        self.selected_side = None
        self.stage = 0
        self.estop_motion = None
        self.estop_started = math.inf
        self.motion_input = False
        self.waiting_for_arm = False

    def pump(self):
        self.spin(self.node, timeout_sec=0.05)
        if (self.motion_input or self.waiting_for_arm) and self.read_line is None:
            ready, _, _ = select.select([sys.stdin], [], [], 0)
            if ready:
                answer = sys.stdin.readline()
                if not answer or answer.strip().lower() in ('q', '!'):
                    if self.motion_input or answer.strip() == '!':
                        self.node.assert_manual_estop()
                        raise TestAbort('operator stop during motion; manual EStop requested'
                                        if self.motion_input else 'manual EStop during ARM wait')
                    self.node.publish_zero()
                    raise TestAbort('operator aborted during ARM verification')
                # Discard other input during motion. It cannot authorize a later phase.
        if self.core.state in ('ABORTED', 'LIMIT_REACHED'):
            raise TestAbort(self.core.reason)

    def wait(self, predicate, timeout):
        deadline = self.clock() + timeout
        next_check = self.clock()
        while self.clock() < deadline:
            self.pump()
            # Drain ready ROS callbacks without rebuilding scene/preflight data
            # for every IMU or EStop heartbeat. The core still ticks at 20 Hz.
            now = self.clock()
            if now >= next_check:
                if predicate():
                    return True
                next_check = now + 0.05
        return bool(predicate())

    def prompt(self, text, exact_enter=False):
        self.node.publish_zero()
        print(text, flush=True)
        print('[q + Enter: abort; ! + Enter: assert manual EStop; Ctrl+C: stop]', flush=True)
        while True:
            if self.read_line is None:
                self.pump()
                # spin_once already waits when no callback is ready. A second
                # 50 ms sleep here caps ROS processing at ~20 callbacks/sec.
                ready, _, _ = select.select([sys.stdin], [], [], 0)
                if not ready:
                    continue
                answer = sys.stdin.readline()
                if not answer:
                    raise TestAbort('input closed')
                answer = answer.rstrip('\r\n')
            else:
                answer = self.read_line()
                if answer is None:
                    raise TestAbort('input closed')
            if answer.lower() == 'q':
                raise TestAbort('operator aborted')
            if answer == '!':
                self.node.assert_manual_estop()
                raise TestAbort('manual EStop requested; it will NOT be automatically cleared')
            if exact_enter and answer != '':
                print('Press Enter to continue, or q to abort.', flush=True)
                continue
            return answer

    def enter(self, text):
        self.prompt(text, exact_enter=True)

    def begin_stage(self, number, title, instructions):
        self.stage = number
        print(f'\n[Stage {number}/8] {title}\n{instructions}', flush=True)
        self.report.data['stages'][f'stage_{number}'] = {
            'title': title, 'status': 'INCOMPLETE', 'started_monotonic': self.clock()}
        self.report.save()

    def result(self, name, status, detail='', **metrics):
        print(f'{name}: {status}' + (f' — {detail}' if detail else ''), flush=True)
        stage = self.report.data['stages'].get(f'stage_{self.stage}')
        if stage is not None:
            stage.setdefault('results', {})[name] = status
        self.report.check(name, status, detail, **metrics)

    def scene(self):
        return self.core.obstacle_details(self.node.observation(self.clock()))

    def show_scene(self):
        metrics = self.scene()
        for key, value in metrics.items():
            print(f'  {key}: {value}', flush=True)
        self.report.data['stages'][f'stage_{self.stage}']['scene'] = metrics
        self.report.save()
        return metrics

    def preflight(self):
        print('''\n====================================================
Traymover Guided EStop-safe Detour Test
No Map / No Nav2 / Small Indoor Motion
====================================================
This test will move the robot.

Required:
 - automatic EStop enabled
 - manual EStop available
 - operator remains next to the robot, outside its footprint
 - clear floor around robot; no stairs / ramps / drop-offs
 - use a cardboard box or chair as obstacle first

Automatic EStop: REQUIRED / ENABLED (verification follows)
Collision Monitor: REQUIRED / ENABLED (verification follows)
Reverse motion: DISABLED
These initial test values have NOT been validated on the physical robot.
This test does NOT validate Nav2 global planning, map-based navigation,
full airport detour, or physical stopping distance.
''', flush=True)
        self.begin_stage(0, 'PREFLIGHT', 'Robot motion: NONE. Checking live safety inputs.')
        healthy = self.wait(lambda: all(
            row['status'] == 'PASS' for row in self.node.preflight().values()), 20.0)
        checks = self.node.preflight()
        healthy = healthy and all(row['status'] == 'PASS' for row in checks.values())
        for label, row in checks.items():
            age = row.get('age', row.get('age_sec'))
            age_text = '' if age is None else f' age={age:.3f} s'
            print(f"{label:28s} {row['status']}{age_text} {row.get('detail', '')}")
        self.report.data['sensor_preflight'] = checks
        failures = '; '.join(label + ': ' + (row.get('detail') or 'input unavailable')
                             for label, row in checks.items() if row['status'] != 'PASS')
        self.result('Preflight sensors', 'PASS' if healthy else 'FAIL', failures)
        if not healthy:
            raise TestAbort('PRECHECK FAILED. Motion tests disabled. TEST ABORT.')
        answer = self.prompt('''The robot has NOT moved yet.
Place the robot on a flat floor.
Keep at least approximately 1 m of free space around the test side.
Remain next to the robot with one hand ready for the manual EStop.
Type ARM to allow low-speed motion tests. Anything else aborts.''')
        if answer != 'ARM':
            raise TestAbort('ARM not supplied; motion disabled')
        self.verify_arm()

    def verify_arm(self):
        """Keep zero while an explicit ARM request awaits stable fresh inputs."""
        self.node.publish_zero()
        self.report.data['arm_preflight_initial'] = self.node.preflight()
        self.report.data['arm_health_events'] = []
        stable_since = None
        signature = None
        had_failure = any(row['status'] != 'PASS'
                          for row in self.report.data['arm_preflight_initial'].values())
        print('ARM requested. Robot remains STOPPED. Waiting up to 10 seconds for\n'
              f'all checks to stay PASS for {self.config.clear_confirm_sec:g} seconds.\n'
              'q + Enter cancels; ! + Enter requests manual EStop.', flush=True)

        def stable():
            nonlocal stable_since, signature, had_failure
            now = self.clock()
            checks = self.node.preflight()
            self.report.data['arm_preflight'] = checks
            failures = {name: row.get('detail') or 'input unavailable'
                        for name, row in checks.items() if row['status'] != 'PASS'}
            current = tuple(failures.items())
            if current != signature:
                self.report.data['arm_health_events'].append({
                    'stamp': now, 'failures': failures})
                if failures:
                    print('ARM waiting — ' + '; '.join(
                        name + ': ' + detail for name, detail in failures.items()), flush=True)
                signature = current
            if failures:
                had_failure = True
                stable_since = None
                return False
            if stable_since is None:
                stable_since = now
            return now - stable_since >= self.config.clear_confirm_sec

        self.waiting_for_arm = True
        try:
            healthy = self.wait(stable, 10.0)
        finally:
            self.waiting_for_arm = False
        # Recheck at the decision point, without accepting the earlier startup
        # snapshot or resetting the shared Core out of HOLD/FAULT.
        checks = self.node.preflight()
        self.report.data['arm_preflight'] = checks
        self.report.save()
        if not healthy or not all(row['status'] == 'PASS' for row in checks.values()):
            detail = '; '.join(name + ': ' + (row.get('detail') or 'input unavailable')
                               for name, row in checks.items() if row['status'] != 'PASS')
            self.result('Preflight sensors', 'FAIL', detail or 'inputs did not stay stable')
            raise TestAbort('PRECHECK FAILED at ARM: inputs did not remain stable within 10 s')
        if not self.core.arm(self.clock()):
            self.result('ARM readiness', 'FAIL', self.core.reason)
            raise TestAbort('core refused ARM after input verification')
        self.result('ARM readiness', 'WARN' if had_failure else 'PASS',
                    'inputs recovered and remained stable; no motion requested' if had_failure
                    else 'fresh inputs remained stable; no motion requested')

    def detection(self):
        self.begin_stage(1, 'FRONT OBSTACLE DETECTION', '''Robot motion: NONE.
Place a cardboard box approximately 1.0–1.1 m in front of the robot.
Do not place it closer than necessary. Stay to the side of the robot.''')
        for attempt in range(3):
            self.enter('Press Enter when ready.' if attempt == 0 else
                       'Adjust the box once more; robot remains stopped. Press Enter to retry.')
            found = self.wait(lambda: self.scene().get('detected', False) and
                              self.scene().get('confirmation_frames', 0) >=
                              self.config.obstacle_confirm_frames, 4.0)
            metrics = self.show_scene()
            if found:
                self.result('Front obstacle detection', 'PASS', **metrics)
                return
            print(f'Obstacle detection attempt {attempt + 1}/3: FAIL', flush=True)
        self.result('Front obstacle detection', 'FAIL', 'bounded retries exhausted')
        raise TestAbort('front obstacle could not be confirmed')

    def wait_clear(self):
        self.begin_stage(2, 'WAIT / CLEAR RECOVERY', '''Leave the obstacle in place.
Robot motion: NONE. Testing NORMAL -> WAITING.
TEST wait = 2.0 sec; PRODUCTION default = 5.0 sec.''')
        self.enter('Press Enter to observe WAITING.')
        if not self.core.restart_observation(self.clock(), self.node.observation(self.clock())):
            self.result('WAIT transition', 'FAIL', self.core.reason)
            raise TestAbort('unable to restart observation safely')
        waiting = self.wait(lambda: self.core.state == 'WAITING', 4.0)
        self.result('WAIT transition', 'PASS' if waiting else 'FAIL', self.core.reason)
        if not waiting:
            raise TestAbort('WAITING was not observed')
        self.wait(lambda: self.core.state in ('READY', 'BLOCKED', 'PLAN_DETOUR'),
                  self.config.wait_before_detour_sec + 3.0)
        event_start = len(self.core.events)
        self.enter('Now remove the obstacle. Robot motion remains disabled.\n'
                   'Press Enter after the area is clear.')
        seen = set()

        def cleared():
            seen.add(self.core.state)
            seen.update(event.get('state') for event in self.core.events[event_start:]
                        if 'state' in event)
            return self.core.state == 'NORMAL' and 'CLEARING' in seen

        clear = self.wait(cleared, 8.0)
        self.result('Obstacle clear recovery', 'PASS' if clear else 'FAIL',
                    ' -> '.join(sorted(seen)), observed_states=sorted(seen))
        if not clear:
            raise TestAbort('clear confirmation did not return to NORMAL')

    def side_selection(self):
        self.begin_stage(3, 'DETOUR SIDE SELECTION', '''Robot motion: NONE.
Place the obstacle in front again, with noticeably more free space on ONE side.
Example: box slightly toward the right -> left side should be more open.
A centered box may block both 25-degree corridors; reposition it if needed.''')
        for attempt in range(2):
            self.enter('Press Enter when the box and free side are ready.')
            self.wait(lambda: self.core.choose_side(
                self.node.observation(self.clock())) is not None, 4.0)
            details = self.show_scene()
            candidate = self.core.choose_side(self.node.observation(self.clock()))
            side = candidate.side if candidate else 'UNKNOWN'
            print(f'Selected detour side: {side}', flush=True)
            answer = self.prompt('Does this direction look physically reasonable? [y/N]')
            if candidate is not None and answer.lower() == 'y':
                self.selected_side = side
                ready = self.wait(lambda: self.core.state == 'READY',
                                  self.config.wait_before_detour_sec + 4.0)
                if not ready:
                    self.result('Left/right side selection', 'WARN',
                                'safe plan did not reach READY; motion remains disabled')
                    raise TestAbort('selected side was not ready for motion')
                self.result('Left/right side selection', 'PASS' if attempt == 0 else 'WARN',
                            'operator confirmed current shared-core selection', **details)
                return
            self.result('Left/right side selection', 'WARN', 'SIDE_SELECTION_WARN', **details)
            print('Motion remains disabled. Reposition the box; one retry is available.'
                  if attempt == 0 else
                  'No confirmed safe side. Stop and reposition robot manually.')
        raise TestAbort('side selection not confirmed')

    def execute_motion(self, label, request, maximum):
        # Only callers immediately following their own Enter prompt reach here.
        before = self.clock()
        if not request():
            self.result(label, 'WARN', 'motion refused: ' + self.core.reason)
            return None
        moving_states = ('TURN_1', 'TURN_2', 'STRAIGHT_1', 'STRAIGHT_2',
                         'TURN', 'STRAIGHT', 'SETTLING', 'PHASE_SETTLE')
        self.motion_input = True
        try:
            stopped = self.wait(lambda: self.core.state not in moving_states,
                                maximum + self.config.stop_settle_time_sec + 3.0)
        finally:
            self.motion_input = False
        self.node.publish_zero()
        if not stopped:
            self.core.abort(self.clock(), 'runner primitive timeout')
            self.result(label, 'FAIL', 'runner timeout; zero asserted')
            raise TestAbort('primitive failed to terminate')
        metrics = dict(self.core.last_motion or {})
        metrics['runner_elapsed_sec'] = self.clock() - before
        metrics['sensor_freshness_after_motion'] = self.node.preflight()
        self.report.data['stages'][f'stage_{self.stage}']['motion'] = metrics
        self.report.save()
        for key, value in metrics.items():
            print(f'  {key}: {value}', flush=True)
        if metrics.get('timeout') or metrics.get('fault') or self.core.state == 'FAULT_HOLD':
            self.result(label, 'FAIL', self.core.reason, **metrics)
            raise TestAbort('motion fault; manual repositioning may be required')
        return metrics

    def turn(self, optional=False):
        if not optional:
            self.begin_stage(4, 'TURN ONLY', 'This is the FIRST robot movement.')
        self.enter(f'''Robot will rotate approximately {self.config.detour_turn_angle_deg:g} degrees
toward {self.selected_side}. Linear motion: NONE.
Angular speed limit: {self.config.detour_max_angular_speed:g} rad/s.
Hard timeout: {self.config.turn_phase_timeout_sec:g} sec.
The robot stops after TURN, settles, and observes. Keep clear of its footprint.
Press Enter to execute TURN.''')
        metrics = self.execute_motion('Turn-only primitive', lambda: self.core.request_turn(
            self.clock(), self.node.observation(self.clock()), self.selected_side),
            self.config.turn_phase_timeout_sec)
        if metrics is None:
            raise TestAbort('TURN blocked; manually reposition robot')
        measured_error = metrics.get('yaw_error_deg')
        error = abs(measured_error) if measured_error is not None else math.inf
        tolerance = math.degrees(self.config.yaw_tolerance_rad)
        warn_tolerance = getattr(self.config, 'turn_warn_error_deg', 10.0)
        status = ('FAIL' if error > warn_tolerance or metrics.get('estop') else
                  'WARN' if error > tolerance or metrics.get('guard') else 'PASS')
        self.result('Turn-only primitive', status, **metrics)
        if status == 'FAIL' or not metrics.get('completed'):
            raise TestAbort('TURN did not complete safely; no automatic next motion')

    def straight(self):
        self.begin_stage(5, 'VERY SHORT STRAIGHT MOTION',
                         'Robot is stopped after TURN. Rechecking the CURRENT heading.')
        self.enter('Press Enter to inspect current front clearance (robot will remain stopped).')
        self.wait(lambda: self.core.state == 'READY', 3.0)
        details = self.show_scene()
        # Actual request performs a fresh shared guard check; the displayed value
        # is only guidance, never an authorization to bypass the core.
        if details.get('front_clearance_m', math.inf) <= self.config.detour_guard_distance_m:
            print('STRAIGHT TEST BLOCKED. Reposition the obstacle or choose skip.')
            if self.prompt('Recheck after repositioning? [y/N]').lower() != 'y':
                return
            self.enter('Press Enter after the space is clear. Robot remains stopped.')
            self.wait(lambda: self.core.state == 'READY', 3.0)
        speed = self.config.test_straight_speed
        duration = self.config.test_straight_duration
        self.enter(f'''The robot will move forward VERY slowly.
Speed = {speed:g} m/s; command duration = {duration:g} sec.
This is intentionally a very small movement; distance is not measured.
Stand to the side, with the manual EStop ready.
Press Enter to execute this one STRAIGHT primitive.''')
        metrics = self.execute_motion('Short straight primitive',
                                      lambda: self.core.request_straight(
                                          self.clock(), self.node.observation(self.clock()),
                                          speed, duration), duration)
        if metrics is not None:
            status = 'PASS' if metrics.get('completed') else 'WARN'
            self.result('Short straight primitive', status,
                        'commanded motion duration; no measured travel distance', **metrics)

    def guard(self):
        self.begin_stage(6, 'DETOUR GUARD', '''Robot motion: a very short, very slow forward command.
Use a cardboard box closer than the previous free-space test, without pushing
it against the robot. Stay to the side; keep the manual EStop ready.
Initial thresholds: trigger 1.10 m; guard 0.95 m; Auto EStop 0.80 m (unchanged).
Expected: Guard stops before Auto EStop whenever sensor geometry allows it.
The core may correctly refuse to start if the guard is already blocked.''')
        self.enter('Press Enter to test the guard with at most 0.5 sec at 0.02 m/s.')
        obs = self.node.observation(self.clock())
        metrics = self.execute_motion('Detour guard', lambda: self.core.request_straight(
            self.clock(), obs, 0.02, 0.5, kind='GUARD_TEST'), 0.5)
        if metrics and metrics.get('guard'):
            self.result('Detour guard', 'PASS', 'guard interrupted active motion', **metrics)
        elif (metrics and metrics.get('estop')) or obs.auto_estop or obs.aggregate_estop:
            self.result('Detour guard', 'WARN',
                        'Auto EStop intervened before detour guard. '
                        'Review guard margin / geometry.')
        elif 'guard' in self.core.reason.lower():
            self.result('Detour guard', 'PASS', 'guard prevented motion from starting')
        else:
            self.result('Detour guard', 'WARN', 'guard intervention not observed')
        self.enter('Robot remains stopped. Remove the box and press Enter '
                   'after the area is clear.')
        self.wait(lambda: self.core.state in ('READY', 'NORMAL'), 8.0)

    def estop(self):
        self.begin_stage(7, 'AUTO ESTOP DURING DETOUR', '''Goal: Auto EStop interrupts active detour motion; the old primitive is discarded.
During the extremely slow forward motion, move a cardboard box into the front
safety area FROM THE SIDE. Do not reach under or over the moving robot.
A person is NOT required as the obstacle. Operator stays beside the robot.
Expected: Auto EStop ACTIVE -> command zero -> HOLD_ESTOP.
Guard stays enabled. If it stops first, active Auto EStop interruption remains
unvalidated; the report will say INCOMPLETE, not fabricate a PASS.''')
        remaining = max(0.0, self.config.max_total_forward_command_sec -
                        self.core.total_forward_command_sec)
        duration = min(self.config.estop_test_max_duration, remaining)
        if duration < 0.1:
            raise TestAbort('TEST MOTION LIMIT REACHED; no forward budget remains')
        self.enter(f'Available forward command budget: {duration:.2f} sec; '
                   f'speed {self.config.estop_test_speed:g} m/s.\n'
                   'Robot automatically stops at the deadline. Press Enter to begin.')
        self.estop_started = self.clock()
        self.estop_motion = self.execute_motion(
            'Mid-detour Auto EStop', lambda: self.core.request_straight(
                self.clock(), self.node.observation(self.clock()),
                self.config.estop_test_speed, duration, kind='ESTOP_TRIGGER_TEST'), duration)
        obs = self.node.observation(self.clock())
        active_interruption = self.active_auto_interruption()
        if not active_interruption:
            self.result('Mid-detour Auto EStop', 'INCOMPLETE',
                        'no Auto EStop interruption of an active primitive was proven')
            self.enter('Robot has stopped. From the SIDE place the box in the safety area\n'
                       'to observe stationary Auto EStop/recovery, or q to end. Press Enter.')
            self.wait(lambda: self.node.observation(self.clock()).auto_estop, 5.0)
            obs = self.node.observation(self.clock())
        else:
            self.wait(lambda: self.node.observation(self.clock()).auto_estop and
                      self.node.observation(self.clock()).aggregate_estop, 1.0)
            obs = self.node.observation(self.clock())
        if obs.auto_estop and obs.aggregate_estop:
            if active_interruption:
                self.result('Mid-detour Auto EStop', 'PASS', **self.estop_motion)
            # Adapter provides observed final CM output and timestamped safety events.
            self.wait(lambda: self.software_zero_observed(), 1.0)
            zero = self.software_zero_observed()
            self.result('EStop -> zero software path', 'PASS' if zero else 'FAIL',
                        'software topic observation only; NOT physical stopping time',
                        snapshot=self.node.snapshot())
            if not zero:
                raise TestAbort('EStop output zero was not observed')
        else:
            self.result('Mid-detour Auto EStop', 'INCOMPLETE', 'Auto/aggregate EStop not observed')
            raise TestAbort('no EStop event available for release/recovery test')

    def software_zero_observed(self):
        events = self.node.snapshot().get('safety_events', [])
        return any(event.get('started_at', -math.inf) >= self.estop_started and
                   event.get('t_auto_state') is not None and
                   event.get('t_aggregate_estop') is not None and
                   event.get('t_output_zero') is not None and
                   event['t_output_zero'] >= event['t_aggregate_estop'] and
                   event.get('t_nonzero_after_aggregate') is None for event in events)

    def active_auto_interruption(self):
        motion = self.estop_motion or {}
        if (not motion.get('estop') or motion.get('estop_source') != 'AUTO'
                or motion.get('command_duration_sec', 0) <= 0):
            return False
        start = motion.get('started_at', math.inf)
        end = motion.get('ended_at', -math.inf)
        return any(
            event.get('started_at', -math.inf) >= self.estop_started
            and event.get('auto_command_was_nonzero', False)
            and any(start <= event.get(key, -math.inf) <= end
                    for key in ('t_auto_request', 't_auto_state'))
            for event in self.node.snapshot().get('safety_events', []))

    def recovery(self):
        self.begin_stage(8, 'ESTOP RELEASE / REPLAN TEST', '''Robot is stopped by EStop.
Remove the obstacle. The old straight command must NOT automatically continue.
Manual EStop, if latched, must be released through its normal manual interface.
This runner never clears EStop. No movement will be requested in this stage.''')
        before = self.core.total_forward_command_sec
        self.enter('Press Enter after the obstacle is removed.')
        recovered = self.wait(lambda: self.core.state in ('READY', 'NORMAL') and
                              not self.node.observation(self.clock()).auto_estop and
                              not self.node.observation(self.clock()).aggregate_estop, 12.0)
        resumed = self.core.total_forward_command_sec > before + 1e-6
        if resumed:
            self.result('Old primitive discarded', 'FAIL', 'old motion resumed without permission')
            self.core.abort(self.clock(), 'unexpected resumed primitive')
            raise TestAbort('unsafe recovery')
        discarded = self.core.old_primitive_discarded
        active_test = self.active_auto_interruption()
        self.result('Old primitive discarded',
                    'PASS' if discarded and active_test else 'INCOMPLETE',
                    'OLD MOTION PRIMITIVE: DISCARDED' if discarded else
                    'no active primitive interruption to prove discard')
        self.result('Post-EStop re-evaluation', 'PASS' if recovered else 'FAIL', self.core.reason)
        print('CURRENT SENSOR STATE: RE-EVALUATED' if recovered else 'RECOVERY NOT CONFIRMED')
        candidate = (self.core.choose_side(self.node.observation(self.clock()))
                     if recovered else None)
        print('NEXT DETOUR SIDE: ' + (candidate.side if candidate else 'UNKNOWN / clear path'))
        if not recovered:
            raise TestAbort('EStop recovery did not complete')
        if overall_result(self.report.data['checks']) == 'PASS':
            if self.prompt('Run one additional re-plan + turn test? [y/N]').lower() == 'y':
                candidate = self.core.choose_side(self.node.observation(self.clock()))
                if candidate is None:
                    print('No safe current candidate. Optional TURN skipped.')
                else:
                    self.selected_side = candidate.side
                    self.turn(optional=True)

    def run(self):
        for stage in (self.preflight, self.detection, self.wait_clear, self.side_selection,
                      self.turn, self.straight, self.guard, self.estop, self.recovery):
            stage()
        self.enter('Stage 9 — FINAL ASSESSMENT. Robot motion: NONE. Press Enter for report.')
        self.report.data['stages']['stage_9'] = {'status': 'PASS', 'title': 'Final assessment'}

    def finish(self, reason=None):
        # Stop before snapshot/report I/O: full disk must never delay the flush.
        self.core.abort(self.clock(), reason or 'session complete')
        self.node.close_zero()
        snapshot = self.node.snapshot()
        faults = snapshot.get('faults', [])
        if faults:
            self.report.check('Runtime safety faults', 'FAIL', faults=faults)
        if self.core.last_motion.get('fault') or self.core.last_motion.get('timeout'):
            self.report.check('Motion safety faults', 'FAIL', **self.core.last_motion)
        critical_labels = ('IMU', 'LiDAR', 'RealSense scan', 'Perimeter scan', 'Driver serial',
                           'Driver Auto EStop parameter', 'Collision Monitor',
                           'Velocity ownership', 'Manual EStop service', 'Sensor transforms')
        unavailable = {name: row for name, row in snapshot.get('preflight', {}).items()
                       if name in critical_labels and row['status'] == 'FAIL'}
        if unavailable:
            self.report.check('Critical inputs at exit', 'FAIL', checks=unavailable)
        self.report.data.update({
            'finished': datetime.now(timezone.utc).isoformat(),
            'stop_reason': reason or 'session complete',
            'core_events': self.core.events,
            'last_motion': self.core.last_motion,
            'total_forward_command_sec': self.core.total_forward_command_sec,
            'total_rotation_deg': self.core.total_rotation_deg,
            'final_snapshot': snapshot,
        })
        self.report.save()
        print('\n=================================================\n'
              'Guided Detour Test — Preliminary Result\n'
              '=================================================')
        for label, row in self.report.data['checks'].items():
            print(f"{label:34s} {row['status']}")
        print('-------------------------------------------------')
        print(f"Overall preliminary result         {self.report.data['overall']}")
        if reason:
            print('Stopped: ' + reason)
        print('\nThis test provides preliminary validation of:')
        for item in VALIDATES:
            print(' - ' + item)
        print('Only completed PASS checks above provide evidence; unrun checks remain INCOMPLETE.')
        print('\nThis test DOES NOT validate:')
        for item in DOES_NOT_VALIDATE:
            print(' - ' + item)
        print('\nA person-detection test may be performed later, but is not required.')
        print('If space is insufficient, stop and reposition manually; never auto-reverse.')
        # Keep the actual stop cause beside the path; otherwise a short pasted
        # terminal tail contains only the generic safety boundary and exit code.
        if reason:
            print('Stop reason: ' + reason)
        checks = self.report.data.get('arm_preflight',
                                      self.report.data.get('sensor_preflight', {}))
        context = 'ARM' if 'arm_preflight' in self.report.data else 'Preflight'
        for label, row in checks.items():
            if row['status'] != 'PASS':
                print(f"{context} failure — {label}: {row.get('detail') or 'input unavailable'}")
        print(f'Report: {self.report.path}', flush=True)


def main(args=None):
    # ROS imports stay out of pure reporting/UI unit tests.
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from guided_detour_node import GuidedDetourNode

    node = session = None
    reason = None
    previous_signals = {}

    def interrupted(signum, frame):
        raise TestAbort(signal.Signals(signum).name)

    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous_signals[signum] = signal.signal(signum, interrupted)
        node = GuidedDetourNode()
        directory = node.declare_parameter('report_directory', str(Path.cwd() / 'logs')).value
        commit = node.declare_parameter('software_commit', 'unknown').value
        report = SessionReport(directory, commit, asdict(node.config))
        session = GuidedSession(node, report, rclpy.spin_once)
        session.run()
    except (TestAbort, KeyboardInterrupt, EOFError) as error:
        reason = str(error) or type(error).__name__
    except Exception as error:
        reason = f'{type(error).__name__}: {error}'
        if session:
            session.report.data['faults'].append(reason)
            session.report.check('Runtime exception', 'FAIL', reason)
        print('TEST ABORT: ' + reason, file=sys.stderr, flush=True)
    finally:
        # Ignore repeated signals only during the short best-effort zero flush.
        for signum in previous_signals:
            signal.signal(signum, signal.SIG_IGN)
        try:
            if session:
                session.finish(reason)
            elif node:
                node.close_zero()
        finally:
            if node:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            for signum, handler in previous_signals.items():
                signal.signal(signum, handler)
    return 0 if session and session.report.data['overall'] in ('PASS', 'PASS WITH WARNING') else 1


if __name__ == '__main__':
    sys.exit(main())
