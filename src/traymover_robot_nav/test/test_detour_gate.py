"""Deterministic policy tests; these are not physical robot validation."""
import importlib.util
import math
from pathlib import Path
import sys
from dataclasses import replace

import pytest


SPEC = importlib.util.spec_from_file_location(
    'detour_gate', Path(__file__).resolve().parents[1] / 'scripts' / 'detour_gate.py')
gate_module = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = gate_module
SPEC.loader.exec_module(gate_module)
Config = gate_module.Config
DetourGate = gate_module.DetourGate
Observation = gate_module.Observation
Pose = gate_module.Pose
Ray = gate_module.Ray
Scan = gate_module.Scan
Scene = gate_module.Scene
Velocity = gate_module.Velocity
ZERO = Velocity()


def scan(now, front=5.0, left=5.0, right=5.0, depth=False):
    rays = []
    for degrees in range(-40 if depth else -180, 42 if depth else 180, 2):
        distance = front if abs(degrees) <= 6 else left if degrees > 0 else right
        rays.append(Ray(0.0, 0.0, math.radians(degrees), distance, True))
    return Scan(now, tuple(rays))


def observation(now, front=5.0, left=5.0, right=5.0, pose=None, **kwargs):
    data = dict(pose=pose or Pose(0.0, 0.0, 0.0),
                lidar=scan(now, front, left, right),
                realsense=scan(now, front, left, right, depth=True),
                nav_cmd=Velocity(0.2, 0.1), nav_cmd_stamp=now,
                nav_goal_active=True, aggregate_estop=False, manual_estop=False,
                auto_estop=False, auto_reason='NONE', estop_fresh=True,
                tf_healthy=True, localization_healthy=True)
    data.update(kwargs)
    return Observation(**data)


class Simulation:
    def __init__(self, **config):
        self.gate = DetourGate(Config(**config))
        self.now = 1.0
        self.pose = Pose(0.0, 0.0, 0.0)
        self.front = 1.06
        self.left = 5.0
        self.right = 3.0

    def tick(self, dt=0.1, **kwargs):
        self.now = round(self.now + dt, 6)
        obs = observation(self.now, self.front, self.left, self.right, self.pose, **kwargs)
        return self.gate.tick(self.now, obs)

    def until(self, state, timeout=15.0, **kwargs):
        end = self.now + timeout
        while self.gate.state != state and self.now < end:
            self.tick(**kwargs)
        assert self.gate.state == state, (self.gate.state, self.gate.reason)

    def turn_one(self):
        self.until('TURN_1')
        assert self.gate.side == 'LEFT'
        assert self.tick().angular_z > 0.0
        self.pose = Pose(0.0, 0.0, math.pi / 4)
        self.front = 5.0
        assert self.tick() == ZERO
        assert self.gate.state == 'PHASE_SETTLE'
        self.until('STRAIGHT_1')
        assert self.tick().linear_x == self.gate.config.detour_max_linear_speed


def test_default_dependency_and_distance_contract():
    cfg = Config()
    assert cfg.enable_detour and cfg.enable_auto_estop
    assert (cfg.detour_trigger_distance_m > cfg.detour_guard_distance_m
            > gate_module.AUTO_ESTOP_STOP_DISTANCE_M)
    assert not Config(enable_auto_estop=False).enable_detour
    assert cfg.robot_radius_m >= math.hypot(0.40, 0.275)
    for values in ({'detour_guard_distance_m': 0.80}, {'detour_trigger_distance_m': 0.90},
                   {'detour_max_linear_speed': 0.3}, {'detour_max_angular_speed': 0.7},
                   {'detour_first_leg_m': math.nan}, {'min_obstacle_points': 1.5}):
        with pytest.raises(ValueError):
            Config(**values)


def test_normal_passthrough_and_nav_command_timeout():
    gate = DetourGate()
    obs = observation(1.0)
    assert gate.tick(1.0, obs) is obs.nav_cmd
    assert gate.state == 'NORMAL'
    assert gate.tick(2.0, replace(observation(2.0), nav_cmd_stamp=1.0)) == ZERO
    assert gate.state == 'NORMAL'


def test_unknown_defaults_fail_closed():
    gate = DetourGate()
    assert gate.tick(1.0, Observation()) == ZERO
    assert gate.state == 'FAULT_HOLD'


def test_repeated_timer_ticks_do_not_count_as_obstacle_frames():
    gate = DetourGate()
    obs = observation(1.0, front=1.06)
    for now in (1.0, 1.1, 1.2):
        assert gate.tick(now, obs) == obs.nav_cmd
    assert gate.state == 'NORMAL'
    assert gate.tick(1.3, observation(1.3, front=1.06)) == ZERO
    assert gate.state == 'WAITING'


def test_temporary_obstacle_waits_clear_confirmation_then_returns_to_nav():
    sim = Simulation()
    sim.until('WAITING')
    sim.front = 5.0
    for unused in range(10):
        assert sim.tick() == ZERO
    assert sim.gate.state == 'WAITING'
    sim.until('RECOVER_TO_NAV')
    while not sim.gate.recovery_requested:
        assert sim.tick() == ZERO
    sim.gate.acknowledge_recovery()
    assert sim.tick() == Velocity(0.2, 0.1)
    assert sim.gate.state == 'NORMAL'


def test_persistent_obstacle_discrete_full_maneuver_and_handback():
    sim = Simulation()
    sim.turn_one()
    leg = sim.gate._phase_distance
    sim.pose = Pose(leg * math.cos(math.pi / 4), leg * math.sin(math.pi / 4), math.pi / 4)
    assert sim.tick() == ZERO
    assert sim.gate.state == 'PHASE_SETTLE'
    sim.until('TURN_2')
    assert sim.tick().linear_x == 0.0
    assert sim.tick().angular_z < 0.0
    sim.pose = Pose(sim.pose.x, sim.pose.y, 0.0)
    assert sim.tick() == ZERO
    sim.until('STRAIGHT_2')
    assert sim.tick() == Velocity(0.08, 0.0)
    sim.pose = Pose(sim.pose.x + sim.gate._phase_distance, sim.pose.y, 0.0)
    assert sim.tick() == ZERO
    assert sim.gate.state == 'RECOVER_TO_NAV'
    for unused in range(30):
        assert sim.tick() == ZERO
    assert sim.gate.recovery_requested
    sim.gate.acknowledge_recovery()
    assert sim.tick() == Velocity(0.2, 0.1)


@pytest.mark.parametrize('left,right,expected', [(5.0, 2.0, 'LEFT'), (2.0, 5.0, 'RIGHT')])
def test_selects_more_open_side(left, right, expected):
    sim = Simulation()
    sim.left, sim.right = left, right
    sim.until('TURN_1')
    assert sim.gate.side == expected
    cmd = sim.tick()
    assert cmd.linear_x == 0.0
    assert (cmd.angular_z > 0.0) == (expected == 'LEFT')


def test_guard_stops_then_replans_without_estop():
    sim = Simulation()
    sim.turn_one()
    sim.front = 0.93
    assert sim.tick() == ZERO
    assert sim.gate.state == 'PLAN_DETOUR'
    assert sim.gate.saved_interruption['phase'] == 'STRAIGHT_1'
    assert sim.gate._candidate is None
    sim.until('BLOCKED')
    assert sim.tick() == ZERO


@pytest.mark.parametrize('manual', [False, True])
def test_mid_detour_estop_discards_old_distance_and_replans_current_pose(manual):
    sim = Simulation()
    sim.turn_one()
    sim.pose = Pose(0.15, 0.15, math.pi / 4)
    assert sim.tick(aggregate_estop=True, manual_estop=manual,
                    auto_estop=not manual,
                    auto_reason='NONE' if manual else 'NEAR_OBSTACLE') == ZERO
    assert sim.gate.state == 'HOLD_ESTOP'
    assert sim.gate.saved_interruption['phase'] == 'STRAIGHT_1'
    assert sim.gate.saved_interruption['pose'] == sim.pose
    assert sim.gate._candidate is None
    # Hold at least a second: release still requires settling AFTER release.
    for unused in range(15):
        assert sim.tick(aggregate_estop=True, manual_estop=manual,
                        auto_estop=not manual,
                        auto_reason='NONE' if manual else 'NEAR_OBSTACLE') == ZERO
    sim.front = 1.06
    sim.pose = Pose(0.20, 0.20, math.pi / 4)
    for unused in range(10):
        assert sim.tick() == ZERO
        assert sim.gate.state == 'HOLD_ESTOP'
    sim.until('PLAN_DETOUR')
    assert sim.gate._phase_distance is None
    sim.until('TURN_1')
    assert sim.gate._phase_start == sim.pose
    assert sim.gate._candidate.target_yaw == pytest.approx(math.pi / 2)
    assert sim.tick().linear_x == 0.0


@pytest.mark.parametrize(
    'reason', ['LOW_VALID_DEPTH', 'INVALID_FRAME', 'DEPTH_TIMEOUT', 'STARTUP', 'ALIEN'])
def test_depth_fault_recovery_requires_healthy_stable_observations(reason):
    sim = Simulation()
    sim.turn_one()
    assert sim.tick(auto_reason=reason, auto_estop=True, aggregate_estop=True) == ZERO
    assert sim.gate.state == 'FAULT_HOLD'
    for unused in range(5):
        assert sim.tick(auto_reason=reason) == ZERO
    assert sim.gate.state == 'FAULT_HOLD'
    sim.front = 1.06
    sim.until('PLAN_DETOUR')
    assert sim.gate._candidate is None


@pytest.mark.parametrize('kwargs', [{'tf_healthy': False}, {'localization_healthy': False},
                                    {'estop_fresh': False}, {'lidar': None}, {'realsense': None}])
def test_stale_pose_sensor_or_estop_health_immediately_zero(kwargs):
    sim = Simulation()
    sim.turn_one()
    assert sim.tick(**kwargs) == ZERO
    assert sim.gate.state == 'FAULT_HOLD'


def test_phase_timeout_holds_without_automatic_blind_retry():
    sim = Simulation()
    sim.until('TURN_1')
    for unused in range(125):
        sim.tick()
    assert sim.gate.state == 'FAULT_HOLD'
    for unused in range(30):
        assert sim.tick() == ZERO
    assert sim.gate.state == 'FAULT_HOLD'


def test_no_safe_sides_blocks_without_motion():
    sim = Simulation()
    sim.left = sim.right = 0.7
    sim.until('BLOCKED')
    for unused in range(40):
        assert sim.tick() == ZERO
        assert sim.gate.state in ('BLOCKED', 'PLAN_DETOUR')


def test_unknown_rear_rotation_space_blocks():
    obs = observation(1.0, front=1.06)
    obs = replace(obs, lidar=replace(obs.lidar, rays=tuple(
        ray for ray in obs.lidar.rays if abs(ray.angle) < math.pi / 2)))
    scene = Scene(obs, Config())
    assert not scene.rotation_clear()
    assert scene.candidate('LEFT', obs.pose) is None


def test_missing_central_depth_is_not_proof_of_clearance():
    gate = DetourGate()
    obs = observation(1.0)
    side_only = Scan(1.0, (Ray(0, 0, math.pi / 4, 5.0),))
    assert gate.tick(1.0, replace(obs, realsense=side_only)) == ZERO
    assert gate.state == 'FAULT_HOLD'


@pytest.mark.parametrize('enable_auto', [False, True])
def test_disabled_detour_transparent_without_sensor_dependencies(enable_auto):
    gate = DetourGate(Config(enable_detour=False, enable_auto_estop=enable_auto))
    obs = Observation(nav_cmd=Velocity(0.2, 0.1), nav_cmd_stamp=1.0)
    assert gate.tick(1.0, obs) is obs.nav_cmd
    assert gate.tick(1.1, replace(obs, manual_estop=True)) == ZERO
    assert gate.tick(1.2, replace(obs, auto_estop=True)) == ZERO
    assert gate.tick(1.3, replace(obs, aggregate_estop=True)) == ZERO
    assert gate.tick(1.4, obs) is obs.nav_cmd
    assert gate.tick(2.0, obs) == ZERO


def test_phase_requires_acquisition_after_settling():
    sim = Simulation()
    sim.until('TURN_1')
    sim.pose = Pose(0.0, 0.0, math.pi / 4)
    sim.front = 5.0
    sim.tick()
    stopped = sim.now
    assert sim.gate.state == 'PHASE_SETTLE'
    for unused in range(8):
        acquisition = min(sim.now, stopped + 0.4)
        assert sim.tick(lidar=scan(acquisition), realsense=scan(acquisition, depth=True)) == ZERO
        assert sim.gate.state == 'PHASE_SETTLE'
    sim.until('STRAIGHT_1')


def test_pose_must_appear_stable_before_next_phase():
    sim = Simulation()
    sim.until('TURN_1')
    sim.front = 5.0
    sim.pose = Pose(0, 0, math.pi / 4)
    sim.tick()
    for index in range(12):
        sim.pose = Pose(index * 0.03, 0, math.pi / 4)
        assert sim.tick() == ZERO
        assert sim.gate.state == 'PHASE_SETTLE'
    sim.until('STRAIGHT_1')


def test_new_heading_blocked_does_not_start_straight():
    sim = Simulation()
    sim.until('TURN_1')
    sim.pose = Pose(0, 0, math.pi / 4)
    sim.front = 1.18  # Above guard, below target + useful leg length.
    sim.tick()
    assert sim.gate.state == 'PHASE_SETTLE'
    sim.until('PLAN_DETOUR')
    assert sim.gate._candidate is None


def test_turn_only_mode_holds_at_first_straight():
    sim = Simulation(allow_straight_motion=False)
    sim.until('TURN_1')
    sim.pose = Pose(0, 0, math.pi / 4)
    sim.front = 5.0
    sim.tick()
    sim.until('BLOCKED')
    for unused in range(30):
        assert sim.tick() == ZERO
    assert sim.gate.state == 'BLOCKED'


@pytest.mark.parametrize('pose', [Pose(-0.2, -0.2, math.pi / 4), Pose(0.2, -0.2, math.pi / 4)])
def test_reverse_or_lateral_movement_never_completes_straight(pose):
    sim = Simulation()
    sim.turn_one()
    sim.pose = pose
    assert sim.tick() == ZERO
    assert sim.gate.state == 'FAULT_HOLD'


def test_goal_cancellation_prevents_autonomous_motion():
    sim = Simulation()
    sim.turn_one()
    assert sim.tick(nav_goal_active=False) == ZERO
    assert sim.gate.state == 'NORMAL'
    assert sim.gate._candidate is None
    assert sim.gate.phase == 'NONE'


def test_reset_discards_goal_maneuver_and_confirmation():
    sim = Simulation()
    sim.turn_one()
    sim.gate.reset(sim.now)
    assert sim.gate.state == 'NORMAL'
    assert sim.gate._candidate is None
    assert sim.gate._obstacle_frames == 0
    assert not sim.gate.recovery_requested


def test_270_degree_navigation_scan_needs_finite_full_perimeter_for_rotation():
    obs = observation(1.0, front=1.06)
    navigation_scan = replace(obs.lidar, rays=tuple(
        ray for ray in obs.lidar.rays if abs(ray.angle) <= math.radians(135)))
    narrow_obs = replace(obs, lidar=navigation_scan)
    assert not Scene(narrow_obs, Config()).rotation_clear()
    perimeter_obs = replace(narrow_obs, perimeter=obs.lidar)
    scene = Scene(perimeter_obs, Config())
    assert scene.rotation_clear()
    assert scene.candidate('LEFT', obs.pose) is not None
    # A declared full angle range with unknown rays is not observed coverage.
    unknown_rear = replace(obs.lidar, rays=tuple(
        replace(ray, distance=math.inf) if abs(ray.angle) > math.radians(135) else ray
        for ray in obs.lidar.rays))
    assert not Scene(replace(narrow_obs, perimeter=unknown_rear), Config()).rotation_clear()


def test_perimeter_close_hit_vetoes_rotation_without_duplicate_trigger_points():
    obs = observation(1.0)
    perimeter = replace(obs.lidar, rays=obs.lidar.rays + (Ray(0, 0, math.pi, 0.4),))
    scene = Scene(replace(obs, perimeter=perimeter), Config())
    assert not scene.rotation_clear()
    assert scene.obstacle() == (False, math.inf)


@pytest.mark.parametrize('perimeter', [Scan(-0.1, (Ray(0, 0, 0, 5),)), Scan(1.0, ())])
def test_required_perimeter_stale_or_empty_fails_closed(perimeter):
    gate = DetourGate()
    assert gate.tick(1.0, replace(observation(1.0), perimeter=perimeter)) == ZERO
    assert gate.state == 'FAULT_HOLD'


def test_phase_settle_requires_fresh_perimeter_acquisition_when_provided():
    sim = Simulation()
    sim.until('TURN_1')
    sim.pose = Pose(0, 0, math.pi / 4)
    sim.front = 5.0
    sim.tick()
    stopped = sim.now
    for unused in range(8):
        acquisition = min(sim.now, stopped + 0.4)
        assert sim.tick(perimeter=scan(acquisition)) == ZERO
        assert sim.gate.state == 'PHASE_SETTLE'
    sim.until('STRAIGHT_1')


@pytest.mark.parametrize('interruption', ['guard', 'manual', 'fault'])
def test_pending_nav_handback_invalidated_by_new_safety_event(interruption):
    sim = Simulation()
    sim.until('WAITING')
    sim.front = 5.0
    sim.until('RECOVER_TO_NAV')
    for unused in range(25):
        assert sim.tick() == ZERO
    assert sim.gate.recovery_requested
    if interruption == 'guard':
        sim.front = 0.93
        expected = 'PLAN_DETOUR'
        assert sim.tick() == ZERO
    elif interruption == 'manual':
        expected = 'HOLD_ESTOP'
        assert sim.tick(manual_estop=True, aggregate_estop=True) == ZERO
    else:
        expected = 'FAULT_HOLD'
        assert sim.tick(auto_reason='DEPTH_TIMEOUT') == ZERO
    assert sim.gate.state == expected
    assert not sim.gate.recovery_requested
    # A delayed Nav2 acceptance callback cannot clear a new safety hold.
    assert sim.gate.acknowledge_recovery() is False
    assert sim.gate.state == expected


def test_nav_handback_acknowledgement_requires_completed_recovery_confirmation():
    gate = DetourGate()
    assert gate.acknowledge_recovery() is False
    sim = Simulation()
    sim.until('WAITING')
    sim.front = 5.0
    sim.until('RECOVER_TO_NAV')
    assert sim.gate.acknowledge_recovery() is False
    for unused in range(25):
        sim.tick()
    assert sim.gate.acknowledge_recovery() is True
    assert sim.gate.state == 'NORMAL'


def camera_scan(now, front):
    # The camera is 0.45 m ahead of base_link; EStop uses its axial depth.
    source = scan(now, front=front, depth=True)
    return replace(source, rays=tuple(
        replace(ray, origin_x=0.45, safety_distance_m=ray.distance * math.cos(ray.angle))
        for ray in source.rays))


def test_camera_origin_offset_does_not_delay_detour_trigger_past_estop_depth():
    gate = DetourGate()
    first = replace(observation(1.0), realsense=camera_scan(1.0, 1.05))
    scene = Scene(first, Config())
    assert scene.obstacle()[0]
    assert scene.obstacle()[1] <= 1.05
    assert scene.clearance() <= 1.05
    assert gate.tick(1.0, first) is first.nav_cmd
    assert gate.tick(1.1, replace(observation(1.1), realsense=camera_scan(1.1, 1.05))) == ZERO
    assert gate.state == 'WAITING'


def test_camera_axial_guard_stops_even_when_base_distance_exceeds_trigger():
    sim = Simulation()
    sim.turn_one()
    assert sim.tick(realsense=camera_scan(sim.now + 0.1, 0.93)) == ZERO
    assert sim.gate.state == 'PLAN_DETOUR'
    assert sim.gate.saved_interruption['reason'] == 'guard'


def test_offaxis_camera_range_uses_axial_safety_distance_in_current_corridor():
    obs = observation(1.0)
    # A scan-frame ray can become forward after camera/base TF rotation.
    axial = 1.2 * math.cos(math.pi / 4)
    ray = Ray(0.45, 0.0, 0.0, 1.2, True, axial)
    obs = replace(obs, realsense=replace(obs.realsense, rays=obs.realsense.rays + (ray,)))
    assert Scene(obs, Config()).clearance() == pytest.approx(axial)
    assert axial < Config().detour_guard_distance_m


@pytest.mark.parametrize('distance', [math.nan, math.inf, -0.1])
def test_invalid_optional_safety_distance_is_not_usable_scan_evidence(distance):
    assert not gate_module._valid_ray(Ray(0, 0, 0, 1.0, True, distance))
    assert gate_module._valid_ray(Ray(0, 0, 0, 1.0, True, 0.0))


def test_actual_auto_estop_distance_reference_cannot_be_lowered_by_config():
    assert gate_module.AUTO_ESTOP_STOP_DISTANCE_M == 0.80
    with pytest.raises(TypeError):
        Config(auto_estop_stop_distance_m=0.1, detour_guard_distance_m=0.2)
    with pytest.raises(ValueError):
        Config(detour_guard_distance_m=0.2)
