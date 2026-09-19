#!/usr/bin/env python3
"""Option 19's one-button detour and odometry-verified line return."""
from dataclasses import dataclass
import math
from typing import Optional

from detour_core import (Candidate, Config, DetourGate, Observation, Pose,
                         Velocity, angle_difference)


@dataclass
class LineReturnConfig(Config):
    forward_speed: float = 0.05
    return_forward_m: float = 0.80
    line_tolerance_m: float = 0.10
    heading_tolerance_deg: float = 5.0
    odom_timeout_sec: float = 0.50
    max_test_duration_sec: float = 240.0

    def __post_init__(self):
        super().__post_init__()
        values = (
            self.forward_speed, self.return_forward_m, self.line_tolerance_m,
            self.heading_tolerance_deg, self.odom_timeout_sec, self.max_test_duration_sec,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError('line-return parameters must be finite and positive')
        if self.forward_speed > 0.10:
            raise ValueError('forward_speed must not exceed 0.10 m/s')
        if not 0.02 <= self.line_tolerance_m <= 0.20:
            raise ValueError('line_tolerance_m must be within 0.02 to 0.20 m')
        if not 1.0 <= self.heading_tolerance_deg <= 10.0:
            raise ValueError('heading_tolerance_deg must be within 1 to 10 degrees')


@dataclass(frozen=True)
class LineReturnObservation(Observation):
    odom_stamp: Optional[float] = None


class LineReturnCore(DetourGate):
    """Drive forward, bypass one obstacle, and verify return to the start line."""

    def __init__(self, config=None):
        super().__init__(config or LineReturnConfig())
        self.started = False
        self.completed = False
        self.original_line = None
        self.return_target = None
        self.leg = 'FORWARD'
        self.maneuver_started = False
        self.started_at = None

    def _pose_health_fault(self, now, obs):
        fault = super()._pose_health_fault(now, obs)
        if fault:
            return fault
        if (obs.odom_stamp is None or not math.isfinite(obs.odom_stamp)
                or not 0.0 <= now - obs.odom_stamp <= self.config.odom_timeout_sec):
            return 'wheel odometry stale/invalid'
        return None

    def _motion_authorized(self, obs):
        return self.started and not self.completed

    def _idle_velocity(self, now, obs):
        return Velocity(self.config.forward_speed, 0.0) if self.started else Velocity()

    def _forward_coverage_ready(self, scene):
        # Option 19 uses a fixed forward-facing camera and does not require the
        # production supervisor's three-angle depth coverage certificate.
        return True

    def _phase_guard_blocked(self, scene):
        return False

    def _rotation_clear(self, scene):
        return True

    def _settled(self, now, obs):
        return True

    def start(self, now, obs):
        if self.started or self.completed or self.original_line is not None:
            return False
        if (self._health_fault(now, obs) or obs.aggregate_estop
                or obs.auto_estop or obs.manual_estop):
            return False
        self.original_line = obs.pose
        self.started_at = now
        self.started = True
        self.reset(now, 'single-button forward motion started')
        return True

    def line_coordinates(self, pose):
        if self.original_line is None:
            raise ValueError('original line has not been recorded')
        dx = pose.x - self.original_line.x
        dy = pose.y - self.original_line.y
        c = math.cos(self.original_line.yaw)
        s = math.sin(self.original_line.yaw)
        return c * dx + s * dy, -s * dx + c * dy

    def _world_point(self, forward, lateral):
        c = math.cos(self.original_line.yaw)
        s = math.sin(self.original_line.yaw)
        return Pose(
            self.original_line.x + c * forward - s * lateral,
            self.original_line.y + s * forward + c * lateral,
            self.original_line.yaw,
        )

    def _candidate_ready(self, now, obs, scene):
        self.leg = 'OUTBOUND'
        self.maneuver_started = True
        super()._candidate_ready(now, obs, scene)

    def _select_candidate(self, scene, obs):
        target = obs.pose.yaw - math.radians(self.config.detour_turn_angle_deg)
        return Candidate(
            'RIGHT', target, self.config.detour_first_leg_m,
            self.config.detour_target_clearance_m + self.config.detour_first_leg_m)

    def _start_phase(self, phase, now, obs, scene):
        if phase.startswith('STRAIGHT'):
            self._phase_distance = (
                self._candidate.first_leg_m if phase == 'STRAIGHT_1'
                else self.config.detour_forward_leg_m)
        self.phase = phase
        self._phase_start = obs.pose
        self._phase_since = now
        self._transition(phase, now, phase + ' fixed action started', settle=False)

    def _begin_clear_recovery(self, now):
        self._discard()
        self._transition('NORMAL', now, 'obstacle cleared; continuing original line')

    def _recover_hold(self, now, obs, front_clear):
        self._discard()
        state = 'NORMAL' if front_clear and not self.maneuver_started else 'PLAN_DETOUR'
        self._transition(state, now, 'safety hold released; current scene re-evaluated')

    def begin_return(self, now, obs):
        forward, lateral = self.line_coordinates(obs.pose)
        target = self._world_point(forward + self.config.return_forward_m, 0.0)
        dx = target.x - obs.pose.x
        dy = target.y - obs.pose.y
        distance = math.hypot(dx, dy)
        if distance < self.config.min_leg_m:
            distance = self.config.min_leg_m
            target = self._world_point(forward + distance, 0.0)
            dx = target.x - obs.pose.x
            dy = target.y - obs.pose.y
        target_yaw = math.atan2(dy, dx)
        self.return_target = target
        self.leg = 'RETURN'
        self._candidate = Candidate(
            'RETURN', target_yaw, distance, self.config.detour_target_clearance_m + distance)
        self.side = 'RETURN'
        self._pending_phase = 'TURN_1'
        self._transition('PHASE_SETTLE', now, 'obstacle passed; returning to original line')

    def verify_return(self, pose):
        unused_forward, lateral = self.line_coordinates(pose)
        heading_error = abs(angle_difference(pose.yaw, self.original_line.yaw))
        return (abs(lateral) <= self.config.line_tolerance_m
                and heading_error <= math.radians(self.config.heading_tolerance_deg))

    def stop(self, now, reason='operator stop'):
        self.started = False
        self._discard()
        self._transition('ABORTED', now, reason)
        return Velocity()

    def tick(self, now, obs):
        if (self.started_at is not None
                and now - self.started_at > self.config.max_test_duration_sec):
            self._latched_fault = 'option 19 timed out before returning to the original line'
        # Before the first obstacle, keep the reference on the actual straight
        # track instead of rejecting harmless encoder startup drift. Once the
        # obstacle is seen, WAITING freezes this line for the return maneuver.
        if (self.started and not self.maneuver_started and self.state == 'NORMAL'
                and obs.pose is not None and obs.localization_healthy):
            self.original_line = obs.pose
        velocity = super().tick(now, obs)
        if self.state != 'RECOVER_TO_NAV':
            return velocity
        if not self.maneuver_started:
            self._discard()
            self._transition('NORMAL', now, 'path clear; continuing original line')
            return Velocity()
        if self.leg == 'OUTBOUND':
            self.begin_return(now, obs)
            return Velocity()
        if not self.verify_return(obs.pose):
            self._latched_fault = 'measured pose did not return to the original straight line'
            return self._hold('FAULT_HOLD', now, obs, self._latched_fault)
        self.completed = True
        self.started = False
        self._discard()
        self._transition('COMPLETE', now, 'detour complete; original straight line restored')
        return Velocity()
