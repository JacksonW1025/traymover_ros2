#!/usr/bin/env python3
"""Pure, fail-closed wait/detour policy; no ROS or chassis control dependencies.

All distances/speeds are initial tuning values, not hardware-validated limits.
Scan stamps use the same monotonic clock as tick(), at acquisition (not receipt).
Rays are transformed into the current base frame by the ROS adapter.  A missing
ray is unknown space; an explicit finite no-return ray certifies only its range.
"""
from dataclasses import dataclass, field
import math
from typing import Optional, Tuple


# Fixed reference to the current safety stack; never an adjustable node parameter.
AUTO_ESTOP_STOP_DISTANCE_M = 0.80


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Velocity:
    linear_x: float = 0.0
    angular_z: float = 0.0


@dataclass(frozen=True)
class Ray:
    origin_x: float
    origin_y: float
    angle: float
    distance: float
    hit: bool = True
    # Optional sensor-origin axial clearance (e.g. RealSense depth ROI).
    safety_distance_m: Optional[float] = None


@dataclass(frozen=True)
class Scan:
    stamp: float
    rays: Tuple[Ray, ...]


@dataclass
class Config:
    enable_detour: bool = True
    enable_auto_estop: bool = True
    wait_before_detour_sec: float = 5.0
    detour_trigger_distance_m: float = 1.10
    detour_guard_distance_m: float = 0.95
    detour_target_clearance_m: float = 1.15
    detour_max_linear_speed: float = 0.08
    detour_max_angular_speed: float = 0.25
    clear_confirm_sec: float = 1.0
    stop_settle_time_sec: float = 0.5
    detour_turn_angle_deg: float = 45.0
    detour_first_leg_m: float = 0.8
    detour_forward_leg_m: float = 1.0
    scan_timeout_sec: float = 1.0
    realsense_timeout_sec: float = 1.0
    cmd_vel_timeout_sec: float = 0.5
    obstacle_confirm_frames: int = 2
    min_obstacle_points: int = 4
    turn_phase_timeout_sec: float = 12.0
    straight_phase_timeout_sec: float = 25.0
    robot_radius_m: float = 0.50
    corridor_half_angle_deg: float = 20.0
    max_ray_gap_deg: float = 10.0
    min_leg_m: float = 0.10
    yaw_tolerance_rad: float = 0.035
    pose_stability_distance_m: float = 0.02
    pose_stability_yaw_rad: float = 0.035
    pose_stability_time_sec: float = 0.25
    allow_straight_motion: bool = True
    max_straight_lateral_error_m: float = 0.10
    max_straight_reverse_error_m: float = 0.05

    def __post_init__(self):
        # Invalid Auto OFF + Detour ON is never permitted, including direct use.
        self.enable_detour = bool(self.enable_detour and self.enable_auto_estop)
        numeric = [value for value in vars(self).values()
                   if isinstance(value, (int, float)) and not isinstance(value, bool)]
        if not all(math.isfinite(value) and value > 0.0 for value in numeric):
            raise ValueError('detour numeric parameters must be finite and positive')
        if not (self.detour_trigger_distance_m > self.detour_guard_distance_m
                > AUTO_ESTOP_STOP_DISTANCE_M):
            raise ValueError('require trigger > guard > Auto EStop stop distance')
        if self.detour_target_clearance_m <= self.detour_guard_distance_m:
            raise ValueError('target clearance must exceed guard distance')
        if not 5.0 <= self.detour_turn_angle_deg <= 75.0:
            raise ValueError('detour turn angle must be within 5 to 75 degrees')
        if self.detour_max_linear_speed > 0.10 or self.detour_max_angular_speed > 0.30:
            raise ValueError('detour speed must not exceed 0.10 m/s or 0.30 rad/s')
        if self.obstacle_confirm_frames != int(self.obstacle_confirm_frames):
            raise ValueError('obstacle_confirm_frames must be an integer')
        if self.min_obstacle_points != int(self.min_obstacle_points):
            raise ValueError('min_obstacle_points must be an integer')


@dataclass(frozen=True)
class Observation:
    pose: Optional[Pose] = None
    lidar: Optional[Scan] = None
    realsense: Optional[Scan] = None
    nav_cmd: Velocity = field(default_factory=Velocity)
    nav_cmd_stamp: Optional[float] = None
    nav_goal_active: bool = False
    aggregate_estop: bool = False
    manual_estop: bool = False
    auto_estop: bool = False
    auto_reason: str = 'STARTUP'
    estop_fresh: bool = False
    tf_healthy: bool = False
    localization_healthy: bool = False
    perimeter: Optional[Scan] = None


@dataclass(frozen=True)
class Candidate:
    side: str
    target_yaw: float
    first_leg_m: float
    clearance_m: float


def angle_difference(target, current):
    return math.atan2(math.sin(target - current), math.cos(target - current))


def _valid_ray(ray):
    return (all(math.isfinite(value) for value in
                (ray.origin_x, ray.origin_y, ray.angle, ray.distance))
            and ray.distance > 0.0
            and (ray.safety_distance_m is None or
                 (math.isfinite(ray.safety_distance_m) and ray.safety_distance_m >= 0.0)))


def _hit_point(ray):
    x = ray.origin_x + ray.distance * math.cos(ray.angle)
    y = ray.origin_y + ray.distance * math.sin(ray.angle)
    distance = math.hypot(x, y)
    if ray.safety_distance_m is not None:
        distance = min(distance, ray.safety_distance_m)
    return x, y, distance


class Scene:
    """Conservative sampled free-space checks with explicit angular coverage.

    Free-space certificates come from LiDAR (coverage), while hits from BOTH
    sensors can veto them.  This never treats absent RealSense rays as clear.
    The adapter separately requires healthy depth and fresh RealSense scans.
    """

    def __init__(self, observation, config):
        self.config = config
        self.lidar_rays = tuple(ray for ray in observation.lidar.rays if _valid_ray(ray))
        self.realsense_rays = tuple(ray for ray in observation.realsense.rays if _valid_ray(ray))
        self.rays = self.lidar_rays + self.realsense_rays
        self.rotation_rays = (self.lidar_rays if observation.perimeter is None else
                              tuple(ray for ray in observation.perimeter.rays if _valid_ray(ray)))
        self.primary_hits = tuple(_hit_point(ray) for ray in self.rays if ray.hit)
        # The unfiltered perimeter stream supplies rotation coverage; its hits
        # also veto movement.  Do not double count it for obstacle confirmation.
        self.hits = self.primary_hits + tuple(
            _hit_point(ray) for ray in self.rotation_rays if ray.hit)

    def forward_depth_observed(self):
        """Require current central depth coverage before any autonomous motion."""
        gap = math.radians(self.config.max_ray_gap_deg)
        return all(any(abs(angle_difference(ray.angle, math.radians(deg))) <= gap / 2.0
                       for ray in self.realsense_rays) for deg in (-10.0, 0.0, 10.0))

    def clearance(self, heading=0.0):
        """Certified range in a fan, vetoed by footprint-width path obstacles."""
        half = math.radians(self.config.corridor_half_angle_deg)
        gap = math.radians(self.config.max_ray_gap_deg)
        certified = []
        for i in range(9):
            direction = heading - half + 2.0 * half * i / 8.0
            matching = [ray for ray in self.lidar_rays
                        if abs(angle_difference(ray.angle, direction)) <= gap / 2.0]
            if not matching:
                return 0.0
            # Use the nearest angular sample; a distant beam is not evidence
            # through a closer obstacle in a neighbouring ray.
            ray = min(matching, key=lambda item: abs(angle_difference(item.angle, direction)))
            projected_origin = ray.origin_x * math.cos(heading) + ray.origin_y * math.sin(heading)
            certified.append(projected_origin + ray.distance * math.cos(ray.angle - heading))
        result = max(0.0, min(certified))
        for x, y, safety_distance in self.hits:
            along = x * math.cos(heading) + y * math.sin(heading)
            lateral = -x * math.sin(heading) + y * math.cos(heading)
            if along >= 0.0 and abs(lateral) <= self.config.robot_radius_m:
                result = min(result, safety_distance)
        return result

    def obstacle(self):
        distances = [distance for x, y, distance in self.primary_hits
                     if x > 0.0 and abs(y) <= self.config.robot_radius_m
                     and distance <= self.config.detour_trigger_distance_m]
        return (len(distances) >= self.config.min_obstacle_points,
                min(distances) if distances else math.inf)

    def rotation_clear(self):
        """Require observed full perimeter outside the circumscribed footprint."""
        gap = math.radians(self.config.max_ray_gap_deg)
        count = int(math.ceil(2.0 * math.pi / gap))
        for i in range(count):
            direction = -math.pi + 2.0 * math.pi * i / count
            matches = [ray for ray in self.rotation_rays
                       if abs(angle_difference(ray.angle, direction)) <= gap / 2.0]
            if not matches:
                return False
            ray = min(matches, key=lambda item: abs(angle_difference(item.angle, direction)))
            origin_projection = (ray.origin_x * math.cos(direction)
                                 + ray.origin_y * math.sin(direction))
            if origin_projection + ray.distance <= self.config.robot_radius_m:
                return False
        return all(math.hypot(x, y) > self.config.robot_radius_m for x, y, unused in self.hits)

    def candidate(self, side, pose):
        sign = 1.0 if side == 'LEFT' else -1.0
        turn = sign * math.radians(self.config.detour_turn_angle_deg)
        if not self.rotation_clear():
            return None
        # Check headings swept during the turn as well as the resulting heading.
        for i in range(6):
            if self.clearance(turn * i / 5.0) <= self.config.detour_guard_distance_m:
                return None
        clearance = self.clearance(turn)
        if clearance < self.config.detour_target_clearance_m:
            return None
        leg = min(self.config.detour_first_leg_m,
                  clearance - self.config.detour_target_clearance_m)
        if leg < self.config.min_leg_m:
            return None
        return Candidate(side, pose.yaw + turn, leg, clearance)


def turn_command(target_yaw, current_yaw, max_speed, tolerance):
    """Shared bounded angular-only controller; None means target reached."""
    error = angle_difference(target_yaw, current_yaw)
    if abs(error) <= tolerance:
        return None
    return Velocity(0.0, math.copysign(min(max_speed, max(0.05, abs(error))), error))


class DetourGate:
    PHASES = ('TURN_1', 'STRAIGHT_1', 'TURN_2', 'STRAIGHT_2')
    DEPTH_FAULTS = ('LOW_VALID_DEPTH', 'INVALID_FRAME', 'DEPTH_TIMEOUT', 'STARTUP')

    def __init__(self, config=None):
        self.config = config or Config()
        self.saved_interruption = None
        self.reset(0.0, 'startup')

    @property
    def active(self):
        return self.state != 'NORMAL'

    def reset(self, now, reason='navigation goal changed'):
        self.state = 'NORMAL'
        self.reason = reason
        self.phase = 'NONE'
        self.side = 'NONE'
        self.recovery_requested = False
        self._state_since = now
        self._observe_after = now
        self._stable_since = now
        self._stable_pose = None
        self._healthy_since = None
        self._clear_since = None
        self._obstacle_frames = 0
        self._last_frames = (-math.inf, -math.inf)
        self._candidate = None
        self._origin_yaw = None
        self._phase_start = None
        self._phase_since = None
        self._phase_distance = None
        self._pending_phase = None
        self._latched_fault = None
        self._blocked_latched = False

    def acknowledge_recovery(self):
        if self.state != 'RECOVER_TO_NAV' or not self.recovery_requested:
            return False
        self.recovery_requested = False
        self.state = 'NORMAL'
        self.phase = 'NONE'
        self.side = 'NONE'
        self.reason = 'original navigation goal handed back to Nav2'
        self._candidate = None
        self._obstacle_frames = 0
        return True

    def _transition(self, state, now, reason, settle=True):
        if state != 'RECOVER_TO_NAV':
            self.recovery_requested = False
        self.state = state
        self.reason = reason
        self._state_since = now
        self._clear_since = None
        self._healthy_since = None
        if settle:
            self._observe_after = now + self.config.stop_settle_time_sec
            self._stable_since = now
            self._stable_pose = None

    def _discard(self):
        self.recovery_requested = False
        self._candidate = None
        self._phase_start = None
        self._phase_since = None
        self._phase_distance = None
        self._pending_phase = None
        self.phase = 'NONE'
        self.side = 'NONE'

    def _hold(self, state, now, obs, reason):
        if self.state != state:
            if self._candidate is not None:
                self.saved_interruption = {
                    'phase': self.phase, 'pose': self._pose(obs), 'side': self.side,
                    'reason': reason, 'stamp': now,
                }
            self._discard()
            self._transition(state, now, reason)
        else:
            self.reason = reason
        self._healthy_since = None
        # Recovery requires new observations after the LAST active hold sample.
        self._observe_after = now + self.config.stop_settle_time_sec
        self._stable_since = now
        self._stable_pose = None
        return Velocity()

    def _pose(self, obs):
        return obs.pose

    def _motion_authorized(self, obs):
        return obs.nav_goal_active

    def _idle_velocity(self, now, obs):
        return self._nav_velocity(now, obs)

    def _forward_coverage_ready(self, scene):
        return scene.forward_depth_observed()

    def _phase_guard_blocked(self, scene):
        return scene.clearance() <= self.config.detour_guard_distance_m

    def _rotation_clear(self, scene):
        return scene.rotation_clear()

    def _begin_clear_recovery(self, now):
        self._transition('RECOVER_TO_NAV', now, 'temporary obstacle cleared')

    def _recover_hold(self, now, obs, front_clear):
        self._discard()
        if front_clear:
            self._transition('RECOVER_TO_NAV', now,
                             'hold released; old maneuver discarded; current path clear')
        else:
            self._transition('PLAN_DETOUR', now,
                             'hold released; old maneuver discarded; replanning from current pose')

    def _extra_state_tick(self, now, obs, scene):
        return None

    def _select_candidate(self, scene, obs):
        candidates = [candidate for candidate in
                      (scene.candidate('LEFT', self._pose(obs)),
                       scene.candidate('RIGHT', self._pose(obs))) if candidate is not None]
        return (max(candidates, key=lambda candidate: (candidate.first_leg_m,
                                                       candidate.clearance_m))
                if candidates else None)

    def _candidate_ready(self, now, obs, scene):
        self._start_phase('TURN_1', now, obs, scene)
        self.reason = 'PLAN: selected ' + self.side + '; ' + self.reason

    def _pose_health_fault(self, now, obs):
        if not obs.tf_healthy or self._pose(obs) is None:
            return 'TF unavailable/stale'
        if not all(math.isfinite(value) for value in vars(self._pose(obs)).values()):
            return 'pose invalid'
        if not obs.localization_healthy:
            return 'localization stale/unhealthy'
        return None

    def _health_fault(self, now, obs):
        if not obs.estop_fresh:
            return 'EStop state or reason stale/unknown'
        if obs.auto_reason in self.DEPTH_FAULTS:
            return obs.auto_reason
        if obs.auto_reason not in ('NONE', 'NEAR_OBSTACLE'):
            return 'unknown Auto EStop reason'
        scans = [(obs.lidar, self.config.scan_timeout_sec, 'LiDAR'),
                 (obs.realsense, self.config.realsense_timeout_sec, 'RealSense')]
        if obs.perimeter is not None:
            scans.append((obs.perimeter, self.config.scan_timeout_sec, 'perimeter LiDAR'))
        for scan, timeout, label in scans:
            if (scan is None or not math.isfinite(scan.stamp)
                    or not 0.0 <= now - scan.stamp <= timeout
                    or not any(_valid_ray(ray) for ray in scan.rays)):
                return label + ' scan stale/invalid'
        return self._pose_health_fault(now, obs)

    def _update_stability(self, now, pose):
        if pose is None:
            self._stable_pose = None
            self._stable_since = now
            return
        old = self._stable_pose
        if (old is None or math.hypot(pose.x - old.x, pose.y - old.y)
                > self.config.pose_stability_distance_m
                or abs(angle_difference(pose.yaw, old.yaw)) > self.config.pose_stability_yaw_rad):
            self._stable_pose = pose
            self._stable_since = now

    def _settled(self, now, obs):
        return (now >= self._observe_after
                and now - self._stable_since >= self.config.pose_stability_time_sec
                and obs.lidar.stamp > self._observe_after
                and obs.realsense.stamp > self._observe_after
                and (obs.perimeter is None or obs.perimeter.stamp > self._observe_after))

    def _confirmed(self, now, condition, attribute='_clear_since'):
        if not condition:
            setattr(self, attribute, None)
            return False
        since = getattr(self, attribute)
        if since is None:
            setattr(self, attribute, now)
            return False
        return now - since >= self.config.clear_confirm_sec

    def _nav_velocity(self, now, obs):
        if (obs.nav_cmd_stamp is None
                or not 0.0 <= now - obs.nav_cmd_stamp <= self.config.cmd_vel_timeout_sec
                or not math.isfinite(obs.nav_cmd.linear_x)
                or not math.isfinite(obs.nav_cmd.angular_z)):
            self.reason = 'Nav2 command stale/invalid; zero command asserted'
            return Velocity()
        return obs.nav_cmd

    def _start_phase(self, phase, now, obs, scene):
        if phase.startswith('STRAIGHT') and not self.config.allow_straight_motion:
            self._blocked_latched = True
            self._discard()
            self._transition('BLOCKED', now, 'straight motion disabled for staged testing')
            return
        if self._phase_guard_blocked(scene):
            self._discard()
            self._transition('PLAN_DETOUR', now, 'forward guard prevents next phase; re-observing')
            return
        if phase.startswith('TURN') and not self._rotation_clear(scene):
            self._discard()
            self._transition('BLOCKED', now, 'rotation footprint clearance unknown/blocked')
            return
        if phase.startswith('STRAIGHT'):
            requested = (self._candidate.first_leg_m if phase == 'STRAIGHT_1'
                         else self.config.detour_forward_leg_m)
            distance = min(requested, scene.clearance() - self.config.detour_target_clearance_m)
            if distance < self.config.min_leg_m:
                self._discard()
                self._transition('PLAN_DETOUR', now,
                                 'insufficient new-heading clearance; replanning')
                return
            self._phase_distance = distance
        self.phase = phase
        self._phase_start = self._pose(obs)
        self._phase_since = now
        self._transition(phase, now, phase + ' started from current pose', settle=False)

    def _finish_phase(self, now):
        index = self.PHASES.index(self.phase)
        if index == len(self.PHASES) - 1:
            self._transition('RECOVER_TO_NAV', now, 'maneuver complete; zero command asserted')
        else:
            self._pending_phase = self.PHASES[index + 1]
            self._transition('PHASE_SETTLE', now,
                             self.phase + ' complete; validating fresh scans and next phase')

    def tick(self, now, obs):
        if not math.isfinite(now):
            return Velocity()
        cfg = self.config
        if obs.manual_estop:
            return self._hold('HOLD_ESTOP', now, obs, 'manual EStop; zero command asserted')
        # Detour-disabled operation preserves transparent navigation and the
        # driver's existing EStop authority without adding sensor dependencies.
        if not cfg.enable_detour:
            if obs.aggregate_estop or obs.auto_estop:
                return self._hold('HOLD_ESTOP', now, obs, 'EStop active; zero command asserted')
            self.state = 'NORMAL'
            self.phase = 'NONE'
            return self._nav_velocity(now, obs)
        fault = self._latched_fault or self._health_fault(now, obs)
        if fault:
            return self._hold('FAULT_HOLD', now, obs, fault + '; zero command asserted')
        if obs.aggregate_estop or obs.auto_estop:
            return self._hold('HOLD_ESTOP', now, obs,
                              'Auto/aggregate EStop ' + obs.auto_reason
                              + '; zero command asserted')
        # A cancelled/replaced goal can never leave an autonomous phase running.
        if not self._motion_authorized(obs):
            self.reset(now, 'no active navigation goal')
            return Velocity()
        self._update_stability(now, self._pose(obs))
        scene = Scene(obs, cfg)
        if not self._forward_coverage_ready(scene):
            return self._hold('FAULT_HOLD', now, obs, 'current-heading RealSense coverage unknown')
        obstacle, distance = scene.obstacle()
        front_clear = scene.clearance() > cfg.detour_trigger_distance_m
        if self.state in ('HOLD_ESTOP', 'FAULT_HOLD'):
            if self._confirmed(now, self._settled(now, obs), '_healthy_since'):
                self._recover_hold(now, obs, front_clear)
            return Velocity()
        extra_velocity = self._extra_state_tick(now, obs, scene)
        if extra_velocity is not None:
            return extra_velocity
        if self.state == 'NORMAL':
            frames = (obs.lidar.stamp, obs.realsense.stamp)
            if frames[0] > self._last_frames[0] and frames[1] > self._last_frames[1]:
                self._last_frames = frames
                self._obstacle_frames = self._obstacle_frames + 1 if obstacle else 0
            if self._obstacle_frames >= cfg.obstacle_confirm_frames:
                self._transition('WAITING', now,
                                 'obstacle {:.2f} m; zero command asserted'.format(distance))
                return Velocity()
            return self._idle_velocity(now, obs)
        if self.state == 'WAITING':
            if self._confirmed(now, front_clear and self._settled(now, obs)):
                self._begin_clear_recovery(now)
            elif now - self._state_since >= cfg.wait_before_detour_sec and self._settled(now, obs):
                self._transition('PLAN_DETOUR', now,
                                 'obstacle persistent; planning from current pose')
            return Velocity()
        if self.state == 'BLOCKED':
            if self._blocked_latched:
                return Velocity()
            if self._confirmed(now, self._settled(now, obs), '_healthy_since'):
                next_state = 'RECOVER_TO_NAV' if front_clear else 'PLAN_DETOUR'
                self._transition(next_state, now, 'blocked scene re-observed from current pose')
            return Velocity()
        if self.state == 'PLAN_DETOUR':
            if not self._settled(now, obs):
                return Velocity()
            if front_clear:
                if self._confirmed(now, True):
                    self._transition('RECOVER_TO_NAV', now, 'obstacle gone before planning')
                return Velocity()
            self._clear_since = None
            candidate = self._select_candidate(scene, obs)
            if candidate is None:
                self._transition('BLOCKED', now, 'no observed safe left/right maneuver')
                return Velocity()
            self._candidate = candidate
            self.side = self._candidate.side
            self._origin_yaw = self._pose(obs).yaw
            self._candidate_ready(now, obs, scene)
            return Velocity()
        if self.state == 'PHASE_SETTLE':
            if self._settled(now, obs):
                self._start_phase(self._pending_phase, now, obs, scene)
            return Velocity()
        if self.state == 'RECOVER_TO_NAV':
            if self._phase_guard_blocked(scene):
                self._discard()
                self._transition('PLAN_DETOUR', now, 'new obstacle prevents handback')
            elif self._confirmed(now, self._settled(now, obs)):
                self.recovery_requested = True
                self.reason = ('settling period completed; pose appears stable; '
                               'awaiting Nav2 goal handback')
            return Velocity()
        if self.state in self.PHASES:
            if self._phase_guard_blocked(scene):
                self.saved_interruption = {'phase': self.phase, 'pose': self._pose(obs),
                                           'side': self.side, 'reason': 'guard', 'stamp': now}
                self._discard()
                self._transition('PLAN_DETOUR', now,
                                 'guard stop; zero command asserted; replanning')
                return Velocity()
            turning = self.state.startswith('TURN')
            if self._phase_completed(now, turning):
                return Velocity()
            timeout = self._phase_timeout(turning)
            if self._phase_timed_out(now, turning, timeout):
                self._latched_fault = self.state + ' timeout; new goal/reset required'
                return self._hold('FAULT_HOLD', now, obs, self._latched_fault)
            if turning:
                if not self._rotation_clear(scene):
                    self._discard()
                    self._transition('BLOCKED', now, 'rotation footprint clearance lost')
                    return Velocity()
                target = self._candidate.target_yaw if self.state == 'TURN_1' else self._origin_yaw
                command = turn_command(target, self._pose(obs).yaw,
                                       cfg.detour_max_angular_speed, cfg.yaw_tolerance_rad)
                if command is None:
                    self._finish_phase(now)
                    return Velocity()
                return command
            return self._straight_velocity(now, obs)
        return self._hold('FAULT_HOLD', now, obs, 'unknown supervisor state')

    def _phase_completed(self, now, turning):
        return False

    def _phase_timed_out(self, now, turning, timeout):
        return now - self._phase_since >= timeout

    def _phase_timeout(self, turning):
        return (self.config.turn_phase_timeout_sec if turning
                else self.config.straight_phase_timeout_sec)

    def _straight_velocity(self, now, obs):
        cfg = self.config
        yaw_error = abs(angle_difference(self._pose(obs).yaw, self._phase_start.yaw))
        if yaw_error > 0.20:
            self._latched_fault = 'straight phase heading drift; new goal/reset required'
            return self._hold('FAULT_HOLD', now, obs, self._latched_fault)
        dx = self._pose(obs).x - self._phase_start.x
        dy = self._pose(obs).y - self._phase_start.y
        displacement = (dx * math.cos(self._phase_start.yaw)
                        + dy * math.sin(self._phase_start.yaw))
        lateral = -dx * math.sin(self._phase_start.yaw) + dy * math.cos(self._phase_start.yaw)
        if (displacement < -cfg.max_straight_reverse_error_m
                or abs(lateral) > cfg.max_straight_lateral_error_m):
            self._latched_fault = ('straight phase unexpected pose displacement; '
                                   'new goal/reset required')
            return self._hold('FAULT_HOLD', now, obs, self._latched_fault)
        if displacement >= self._phase_distance:
            self._finish_phase(now)
            return Velocity()
        return Velocity(cfg.detour_max_linear_speed, 0.0)
