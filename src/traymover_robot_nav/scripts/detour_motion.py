#!/usr/bin/env python3
"""Guided policy for the production DetourGate: IMU turns and bounded commands.

No distance or localization measurements are inferred. Pose-shaped values inside
shared angular/stability calculations contain only an IMU yaw reference. Every
primitive needs a fresh explicit request; recovery only returns to zero/READY.
"""
from dataclasses import dataclass
import math
from typing import Optional

from detour_core import (Candidate, Config, DetourGate, Observation, Pose, Scene,
                         Velocity, angle_difference)


@dataclass
class GuidedConfig(Config):
    test_wait_sec: float = 2.0
    test_turn_angle_deg: float = 25.0
    test_turn_speed: float = 0.15
    test_turn_tolerance_deg: float = 5.0
    test_turn_timeout_sec: float = 5.0
    turn_warn_error_deg: float = 10.0
    test_straight_speed: float = 0.03
    test_straight_duration: float = 1.5
    test_straight_timeout_sec: float = 2.0
    estop_test_speed: float = 0.02
    estop_test_max_duration: float = 5.0
    stop_settle_sec: float = 0.5
    imu_timeout_sec: float = 0.5
    max_total_forward_command_sec: float = 5.0
    max_total_rotation_deg: float = 90.0
    max_test_duration_sec: float = 900.0
    control_timeout_sec: float = 0.25

    def __post_init__(self):
        if not self.enable_auto_estop or not self.enable_detour:
            raise ValueError('guided test requires automatic EStop and detour core')
        self.wait_before_detour_sec = self.test_wait_sec
        self.detour_turn_angle_deg = self.test_turn_angle_deg
        self.detour_max_angular_speed = self.test_turn_speed
        self.yaw_tolerance_rad = math.radians(self.test_turn_tolerance_deg)
        self.turn_phase_timeout_sec = self.test_turn_timeout_sec
        self.detour_max_linear_speed = self.test_straight_speed
        self.straight_phase_timeout_sec = self.test_straight_timeout_sec
        self.stop_settle_time_sec = self.stop_settle_sec
        super().__post_init__()
        caps = {
            'test_turn_angle_deg': 30.0, 'test_turn_speed': 0.15,
            'test_turn_tolerance_deg': 5.0, 'test_turn_timeout_sec': 5.0,
            'turn_warn_error_deg': 10.0,
            'test_straight_speed': 0.04, 'test_straight_duration': 2.0,
            'test_straight_timeout_sec': 2.0, 'estop_test_speed': 0.03,
            'estop_test_max_duration': 5.0, 'max_total_forward_command_sec': 5.0,
            'max_total_rotation_deg': 90.0, 'max_test_duration_sec': 1200.0,
            'imu_timeout_sec': 0.5, 'scan_timeout_sec': 1.0,
            'realsense_timeout_sec': 1.0, 'control_timeout_sec': 0.25,
        }
        for name, limit in caps.items():
            if getattr(self, name) > limit:
                raise ValueError(f'{name} must not exceed {limit}')
        if self.turn_warn_error_deg < self.test_turn_tolerance_deg:
            raise ValueError('turn warning threshold must cover the completion tolerance')
        if self.test_straight_duration > self.test_straight_timeout_sec:
            raise ValueError('straight duration exceeds its hard timeout')
        if self.test_turn_tolerance_deg >= self.test_turn_angle_deg:
            raise ValueError('turn tolerance must be smaller than turn angle')
        if self.stop_settle_sec < 0.5:
            raise ValueError('guided stop settling must be at least 0.5 seconds')


@dataclass(frozen=True)
class GuidedObservation(Observation):
    imu_yaw: Optional[float] = None
    imu_stamp: Optional[float] = None
    collision_monitor_fresh: bool = False
    driver_fresh: bool = False
    health_fault: str = ''


class GuidedDetourCore(DetourGate):
    """The same FSM, obstacle, guard, EStop and recovery checks as production."""

    def __init__(self, config=None):
        super().__init__(config or GuidedConfig())
        self.armed = False
        self.session_started = None
        self.total_forward_command_sec = 0.0
        self.total_rotation_deg = 0.0
        self.events = []
        self.last_motion = {}
        self.old_primitive_discarded = False
        self._last_tick = None
        self._last_command = Velocity()
        self._last_yaw = None
        self._last_observation = None
        self._active_motion = None
        self._straight_duration = None
        self._straight_speed = None
        self._straight_hard_timeout = None
        self._session_fault = None
        self._abort_is_fault = False

    @property
    def expired(self):
        return self._session_fault is not None

    def _pose(self, obs):
        return None if obs.imu_yaw is None else Pose(0.0, 0.0, obs.imu_yaw)

    def _pose_health_fault(self, now, obs):
        if obs.health_fault:
            return obs.health_fault
        if not obs.tf_healthy:
            return 'sensor TF unavailable/stale'
        if (obs.imu_yaw is None or not math.isfinite(obs.imu_yaw)
                or obs.imu_stamp is None or not math.isfinite(obs.imu_stamp)
                or not 0.0 <= now - obs.imu_stamp <= self.config.imu_timeout_sec):
            return 'IMU stale/invalid'
        if not obs.collision_monitor_fresh:
            return 'Collision Monitor unavailable/stale'
        if not obs.driver_fresh:
            return 'chassis driver unavailable/stale'
        return None

    def _motion_authorized(self, obs):
        # Observation stages are active before ARM, but produce only zero.
        return self._session_fault is None

    def _idle_velocity(self, now, obs):
        return Velocity()

    def _transition(self, state, now, reason, settle=True):
        if state == 'RECOVER_TO_NAV':
            state = 'READY'
            reason = 'current sensor state re-evaluated; explicit next-step request required'
        super()._transition(state, now, reason, settle)
        if hasattr(self, 'events'):
            self.events.append({'stamp': now, 'state': state, 'reason': reason})

    def _begin_clear_recovery(self, now):
        self._transition('CLEARING', now, 'clear confirmation complete; remaining stopped')

    def _recover_hold(self, now, obs, front_clear):
        self._discard()
        self._transition('READY', now,
                         'old primitive discarded; current sensor state re-evaluated; '
                         'explicit next-step request required')
        candidate = self.choose_side(obs)
        self.side = candidate.side if candidate is not None else 'UNKNOWN'

    def _candidate_ready(self, now, obs, scene):
        self._transition('READY', now, 'PLAN: selected ' + self.side + '; waiting for operator')

    def _extra_state_tick(self, now, obs, scene):
        if self.state == 'CLEARING':
            if scene.clearance() <= self.config.detour_trigger_distance_m:
                self._transition('WAITING', now, 'obstacle returned during clear confirmation')
            elif self._settled(now, obs):
                self._discard()
                self._transition('NORMAL', now, 'obstacle cleared; no motion authorized')
            return Velocity()
        if self.state == 'SETTLING':
            if self._settled(now, obs):
                self._discard()
                self._transition('READY', now, 'fresh observations after stopped primitive',
                                 settle=False)
            return Velocity()
        if self.state == 'READY':
            if (not self.last_motion and self._confirmed(
                    now, scene.clearance() > self.config.detour_trigger_distance_m
                    and self._settled(now, obs))):
                self._begin_clear_recovery(now)
            return Velocity()
        return None

    def _finish_phase(self, now):
        self._pending_phase = None
        self._transition('SETTLING', now, 'primitive complete; zero, settle, observe')

    def _phase_completed(self, now, turning):
        # Planned time completion is a zero command. A missed control deadline
        # is checked before the shared FSM, independently of this normal stop.
        if not turning and now - self._phase_since >= self._straight_duration:
            self._finish_phase(now)
            return True
        return False

    def _phase_timeout(self, turning):
        if turning:
            return self.config.test_turn_timeout_sec
        return self._straight_hard_timeout

    def _straight_velocity(self, now, obs):
        if abs(angle_difference(obs.imu_yaw, self._phase_start.yaw)) > 0.20:
            self._latched_fault = 'straight phase IMU heading drift'
            return self._hold('FAULT_HOLD', now, obs, self._latched_fault)
        return Velocity(self._straight_speed, 0.0)

    def arm(self, now):
        obs = self._last_observation
        if (self._session_fault or self._latched_fault or obs is None
                or self._health_fault(now, obs) or obs.aggregate_estop
                or obs.auto_estop or obs.manual_estop
                or self.state in ('HOLD_ESTOP', 'FAULT_HOLD', 'ABORTED')):
            return False
        self.armed = True
        self.events.append({'stamp': now, 'event': 'ARM'})
        return True

    def restart_observation(self, now, obs):
        if (self._session_fault or self._latched_fault or self._active_motion
                or self.state in ('HOLD_ESTOP', 'FAULT_HOLD', 'ABORTED', 'SETTLING')
                or self._health_fault(now, obs) or obs.aggregate_estop
                or obs.auto_estop or obs.manual_estop):
            return False
        self._discard()
        self._obstacle_frames = 0
        self._last_frames = (-math.inf, -math.inf)
        self._transition('NORMAL', now, 'new observation stage; no motion request')
        return True

    def choose_side(self, obs):
        if (obs.lidar is None or obs.realsense is None or self._pose(obs) is None
                or not math.isfinite(obs.imu_yaw)):
            return None
        return self._select_candidate(Scene(obs, self.config), obs)

    def obstacle_details(self, obs):
        if obs.lidar is None or obs.realsense is None:
            return {'detected': False, 'nearest_range_m': None, 'supporting_points': 0,
                    'confirmation_frames': self._obstacle_frames, 'selected_side': 'UNKNOWN'}
        scene = Scene(obs, self.config)
        distances = [distance for x, y, distance in scene.primary_hits
                     if x > 0 and abs(y) <= self.config.robot_radius_m
                     and distance <= self.config.detour_trigger_distance_m]
        turn = math.radians(self.config.test_turn_angle_deg)
        half = math.radians(self.config.corridor_half_angle_deg)

        def count(heading):
            return sum(abs(angle_difference(ray.angle, heading)) <= half
                       for ray in scene.lidar_rays)

        candidate = self.choose_side(obs)
        return {
            'detected': len(distances) >= self.config.min_obstacle_points,
            'nearest_range_m': min(distances) if distances else None,
            'supporting_points': len(distances), 'confirmation_frames': self._obstacle_frames,
            'front_clearance_m': scene.clearance(),
            'left_clearance_m': scene.clearance(turn),
            'right_clearance_m': scene.clearance(-turn),
            'left_valid_points': count(turn), 'right_valid_points': count(-turn),
            'center_valid_points': count(0),
            'selected_side': candidate.side if candidate else 'UNKNOWN',
        }

    def _request_allowed(self, now, obs):
        if (not self.armed or self._session_fault or self._latched_fault
                or self._active_motion or self.state not in ('READY', 'NORMAL', 'BLOCKED')
                or self._health_fault(now, obs) or obs.aggregate_estop
                or obs.auto_estop or obs.manual_estop or not self._settled(now, obs)):
            self.reason = ('motion request rejected: ARM, healthy settled sensors '
                           'and READY required')
            return False
        if not Scene(obs, self.config).forward_depth_observed():
            self.reason = 'motion request rejected: central depth coverage unavailable'
            return False
        return True

    def request_turn(self, now, obs, side):
        if not self._request_allowed(now, obs) or side not in ('LEFT', 'RIGHT'):
            return False
        candidate = Scene(obs, self.config).candidate(side, self._pose(obs))
        if candidate is None:
            self.reason = 'no observed safe ' + side + ' turn corridor'
            return False
        if (self.total_rotation_deg + self.config.test_turn_angle_deg
                > self.config.max_total_rotation_deg + 1e-9):
            self.abort(now, 'TEST MOTION LIMIT REACHED: rotation budget')
            return False
        self._candidate = candidate
        self.side = side
        self._origin_yaw = obs.imu_yaw
        self._start_phase('TURN_1', now, obs, Scene(obs, self.config))
        if self.state != 'TURN_1':
            return False
        self._active_motion = {
            'kind': 'TURN', 'side': side, 'start_yaw_rad': obs.imu_yaw,
            'commanded_turn_deg': self.config.test_turn_angle_deg * (1 if side == 'LEFT' else -1),
            'started_at': now, 'command_duration_sec': 0.0,
            'max_commanded_speed': self.config.test_turn_speed,
        }
        self.old_primitive_discarded = False
        return True

    def request_straight(self, now, obs, speed=None, duration=None, kind='SHORT_STRAIGHT'):
        if not self._request_allowed(now, obs):
            return False
        cfg = self.config
        if not cfg.allow_straight_motion:
            self.reason = 'straight motion disabled by configuration'
            return False
        is_estop = kind == 'ESTOP_TRIGGER_TEST'
        if speed is None:
            speed = cfg.estop_test_speed if is_estop else cfg.test_straight_speed
        if duration is None:
            duration = cfg.estop_test_max_duration if is_estop else cfg.test_straight_duration
        limit = cfg.estop_test_max_duration if is_estop else cfg.test_straight_timeout_sec
        max_speed = 0.03 if is_estop else 0.04
        if (kind not in ('SHORT_STRAIGHT', 'GUARD_TEST', 'ESTOP_TRIGGER_TEST')
                or not all(math.isfinite(v) and v > 0 for v in (speed, duration))
                or speed > max_speed or duration > limit):
            raise ValueError('invalid bounded forward primitive')
        if (cfg.max_total_forward_command_sec - self.total_forward_command_sec
                <= 2.0 * cfg.control_timeout_sec):
            self.abort(now, 'TEST MOTION LIMIT REACHED: forward command budget')
            return False
        # Reserve one control-timeout interval inside the remaining session
        # allowance; the final exercise can be shorter than its requested time.
        duration = min(duration, cfg.max_total_forward_command_sec
                       - self.total_forward_command_sec - cfg.control_timeout_sec)
        scene = Scene(obs, cfg)
        if scene.clearance() <= cfg.detour_guard_distance_m:
            self.reason = 'STRAIGHT TEST BLOCKED: detour guard'
            self.events.append({'stamp': now, 'event': 'guard', 'reason': self.reason})
            return False
        self._candidate = Candidate('FORWARD', obs.imu_yaw, 0.0, scene.clearance())
        self.side = 'FORWARD'
        self.phase = 'STRAIGHT_1'
        self._phase_start = self._pose(obs)
        self._phase_since = now
        self._straight_duration = duration
        self._straight_speed = speed
        self._straight_hard_timeout = limit
        self._transition('STRAIGHT_1', now, kind + ' started; duration bounded', settle=False)
        self._active_motion = {
            'kind': kind, 'side': 'FORWARD', 'start_yaw_rad': obs.imu_yaw,
            'commanded_turn_deg': 0.0, 'started_at': now,
            'requested_duration_sec': duration, 'command_duration_sec': 0.0,
            'max_commanded_speed': speed,
        }
        self.old_primitive_discarded = False
        return True

    def _record_motion_end(self, now, obs, completed=False):
        if self._active_motion is None:
            return
        motion = dict(self._active_motion)
        end_yaw = obs.imu_yaw if obs is not None else self._last_yaw
        delta = (math.degrees(angle_difference(end_yaw, motion['start_yaw_rad']))
                 if end_yaw is not None and math.isfinite(end_yaw) else None)
        motion.update(
            end_yaw_rad=end_yaw, measured_delta_deg=delta,
            yaw_error_deg=(motion['commanded_turn_deg'] - delta if delta is not None else None),
            duration_sec=max(0.0, now - motion['started_at']),
            estop=self.state == 'HOLD_ESTOP',
            estop_reason=self.reason if self.state == 'HOLD_ESTOP' else None,
            estop_source=(('MANUAL' if obs.manual_estop else
                           'AUTO' if obs.auto_estop else 'AGGREGATE')
                          if self.state == 'HOLD_ESTOP' and obs is not None else None),
            guard='guard' in self.reason.lower(),
            timeout='timeout' in self.reason.lower() or 'overrun' in self.reason.lower(),
            fault=(self.reason if self.state == 'FAULT_HOLD' or
                   (self.state == 'ABORTED' and self._abort_is_fault) else None),
            interrupted=not completed,
            completed=completed, ended_at=now,
        )
        self.last_motion = motion
        if not completed:
            self.old_primitive_discarded = True
        self.events.append({'stamp': now, 'event': 'motion_end', **motion})
        self._active_motion = None

    def abort(self, now, reason='operator abort'):
        self.armed = False
        normal_interruptions = ('operator', 'sigint', 'sigterm', 'keyboardinterrupt',
                                'session complete', 'guided runner shutdown',
                                'test motion limit reached',
                                'user abort', 'user stop', 'abort requested', 'cancel')
        self._abort_is_fault = not any(word in reason.lower() for word in normal_interruptions)
        self._session_fault = reason
        self._discard()
        self._transition('ABORTED', now, reason)
        self._record_motion_end(now, self._last_observation)
        self._last_command = Velocity()
        return Velocity()

    def tick(self, now, obs):
        if not math.isfinite(now):
            return self.abort(self._last_tick or 0.0, 'non-finite monotonic time')
        if self.session_started is None:
            self.session_started = now
        previous_active = self._active_motion is not None
        if self._last_tick is not None:
            dt = now - self._last_tick
            if dt < 0:
                return self.abort(now, 'monotonic clock moved backward')
            if self._last_command.linear_x > 0:
                self.total_forward_command_sec += dt
                if self._active_motion is not None:
                    self._active_motion['command_duration_sec'] += dt
            if previous_active and self._last_yaw is not None and obs.imu_yaw is not None:
                if math.isfinite(obs.imu_yaw):
                    self.total_rotation_deg += abs(math.degrees(
                        angle_difference(obs.imu_yaw, self._last_yaw)))
            if self._last_command != Velocity() and dt > self.config.control_timeout_sec:
                return self.abort(now, 'control timer overrun; zero command asserted')
        self._last_tick = now
        self._last_observation = obs
        self._last_yaw = obs.imu_yaw
        if self._session_fault:
            return Velocity()
        if now - self.session_started >= self.config.max_test_duration_sec:
            return self.abort(now, 'TEST MOTION LIMIT REACHED: session duration')
        if self.total_forward_command_sec > self.config.max_total_forward_command_sec + 1e-9:
            return self.abort(now, 'TEST MOTION LIMIT REACHED: forward command budget')
        if self.total_rotation_deg >= self.config.max_total_rotation_deg:
            return self.abort(now, 'TEST MOTION LIMIT REACHED: accumulated rotation')
        health_fault = self._health_fault(now, obs)
        if previous_active and health_fault:
            self._latched_fault = health_fault
        velocity = super().tick(now, obs)
        if previous_active and self.state not in self.PHASES:
            self._record_motion_end(now, obs, completed=self.state == 'SETTLING')
        if velocity != Velocity() and not self.armed:
            return self.abort(now, 'unarmed nonzero command rejected')
        if (velocity.linear_x < 0 or (velocity.linear_x != 0 and velocity.angular_z != 0)
                or not all(math.isfinite(v) for v in (velocity.linear_x, velocity.angular_z))):
            return self.abort(now, 'invalid reverse/combined/non-finite command rejected')
        if (velocity.linear_x > 0
                and self.config.max_total_forward_command_sec - self.total_forward_command_sec
                <= self.config.control_timeout_sec):
            return self.abort(now, 'TEST MOTION LIMIT REACHED: forward command budget')
        self._last_command = velocity
        return velocity
