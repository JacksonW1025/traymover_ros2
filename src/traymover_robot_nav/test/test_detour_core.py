"""Guided mode simulations exercise the same production FSM and motion guard."""
from dataclasses import replace
import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from detour_core import DetourGate, Ray, Scan, Scene, Velocity  # noqa: E402
from detour_motion import GuidedConfig, GuidedDetourCore, GuidedObservation  # noqa: E402

ZERO = Velocity()


def observation(now, yaw=0.0, box=None, front=4.0, **updates):
    rays = []
    for deg in range(-180, 180, 2):
        distance = front if abs(deg) <= 6 else 4.0
        if box == 'right' and -18 <= deg <= -8:
            distance = 1.06
        if box == 'left' and 8 <= deg <= 18:
            distance = 1.06
        rays.append(Ray(0, 0, math.radians(deg), distance))
    scan = Scan(now, tuple(rays))
    depth = Scan(now, tuple(ray for ray in rays if abs(ray.angle) <= math.radians(40)))
    args = dict(lidar=scan, realsense=depth, perimeter=scan,
                imu_yaw=yaw, imu_stamp=now, tf_healthy=True, estop_fresh=True,
                auto_reason='NONE', collision_monitor_fresh=True, driver_fresh=True)
    args.update(updates)
    return GuidedObservation(**args)


class Sim:
    def __init__(self, **config):
        self.core = GuidedDetourCore(GuidedConfig(**config))
        self.now = 1.0
        self.yaw = 0.0
        self.kwargs = {}
        self.obs = observation(self.now)
        for _ in range(12):
            assert self.tick() == ZERO
        assert self.core.arm(self.now)

    def tick(self, dt=0.1, **updates):
        self.now = round(self.now + dt, 8)
        self.obs = observation(self.now, self.yaw, **dict(self.kwargs, **updates))
        return self.core.tick(self.now, self.obs)

    def until(self, state, maximum=100):
        for _ in range(maximum):
            if self.core.state == state:
                return
            assert self.tick() == ZERO
        assert self.core.state == state, (self.core.state, self.core.reason)

    def turn(self, side='LEFT'):
        self.kwargs['box'] = 'right' if side == 'LEFT' else 'left'
        self.tick()
        self.until('READY')
        for _ in range(8):
            self.tick()
        assert self.core.request_turn(self.now, self.obs, side), self.core.reason
        self.kwargs.clear()

    def straight(self, **kwargs):
        assert self.core.request_straight(self.now, self.obs, **kwargs), self.core.reason
        assert self.tick().linear_x > 0


def test_guided_is_shared_production_core_with_no_nav_or_localization_claim():
    sim = Sim()
    assert isinstance(sim.core, DetourGate)
    assert sim.obs.pose is None and not sim.obs.nav_goal_active
    assert not sim.obs.localization_healthy
    assert sim.core.config.detour_guard_distance_m == 0.95
    assert sim.core.config.test_wait_sec == 2.0
    assert DetourGate().config.wait_before_detour_sec == 5.0


@pytest.mark.parametrize('updates', [
    {'imu_stamp': None}, {'lidar': None}, {'realsense': None},
    {'auto_reason': 'STARTUP'}, {'estop_fresh': False},
    {'collision_monitor_fresh': False}, {'driver_fresh': False},
    {'health_fault': 'unexpected cmd_vel publisher'}, {'tf_healthy': False},
])
def test_preflight_fail_never_arms_or_moves(updates):
    core = GuidedDetourCore()
    obs = observation(1.0, **updates)
    assert core.tick(1.0, obs) == ZERO
    assert not core.arm(1.0)
    assert not core.request_straight(1.0, obs)
    assert core.state == 'FAULT_HOLD'


def test_front_obstacle_frames_wait_and_clear_without_motion():
    sim = Sim()
    sim.kwargs['front'] = 1.06
    assert sim.tick() == ZERO
    details = sim.core.obstacle_details(sim.obs)
    assert details['detected'] and details['supporting_points'] >= 4
    assert details['nearest_range_m'] == pytest.approx(1.06)
    assert sim.tick() == ZERO
    assert sim.core.state == 'WAITING'
    sim.kwargs.clear()
    sim.until('CLEARING')
    sim.until('NORMAL')
    assert sim.core._active_motion is None


@pytest.mark.parametrize('box,expected', [('left', 'RIGHT'), ('right', 'LEFT')])
def test_shared_side_choice_waits_for_explicit_permit(box, expected):
    sim = Sim()
    sim.kwargs['box'] = box
    sim.until('READY')
    assert sim.core.side == expected
    assert sim.core.choose_side(sim.obs).side == expected
    for _ in range(100):
        assert sim.tick() == ZERO
    assert sim.core.state == 'READY'


def test_centered_box_cannot_be_certified_clear_at_25_degree_turn():
    obs = observation(1.0, front=1.06)
    core = GuidedDetourCore()
    assert Scene(obs, core.config).obstacle()[0]
    assert core.choose_side(obs) is None


def test_turn_uses_imu_and_stops_settles_observes_without_next_phase():
    sim = Sim()
    sim.turn()
    for _ in range(45):
        command = sim.tick()
        assert command.linear_x == 0
        assert abs(command.angular_z) <= 0.15
        sim.yaw += command.angular_z * 0.1
        if sim.core.state == 'SETTLING':
            break
    assert sim.core.state == 'SETTLING'
    assert sim.core.last_motion['completed']
    assert abs(sim.core.last_motion['yaw_error_deg']) <= 5
    assert not sim.core.request_straight(sim.now, sim.obs)
    sim.until('READY')
    for _ in range(20):
        assert sim.tick() == ZERO
    assert sim.core.phase == 'NONE'


def test_turn_timeout_is_latched_and_no_new_request_can_clear_it():
    sim = Sim()
    sim.turn()
    for _ in range(55):
        sim.tick()
    assert sim.core.state == 'FAULT_HOLD'
    assert sim.core.last_motion['timeout']
    assert sim.core.old_primitive_discarded
    for _ in range(30):
        assert sim.tick() == ZERO
    assert not sim.core.restart_observation(sim.now, sim.obs)
    assert not sim.core.request_turn(sim.now, sim.obs, 'LEFT')


@pytest.mark.parametrize('updates', [
    {'imu_stamp': -100.0}, {'lidar': None}, {'realsense': None},
    {'estop_fresh': False}, {'collision_monitor_fresh': False},
])
def test_sensor_failure_during_motion_zero_latches_and_discards(updates):
    sim = Sim()
    sim.straight()
    assert sim.tick(**updates) == ZERO
    assert sim.core.state == 'FAULT_HOLD'
    assert sim.core.old_primitive_discarded
    assert sim.core.last_motion['fault']
    for _ in range(40):
        assert sim.tick() == ZERO
    assert not sim.core.request_straight(sim.now, sim.obs)


def test_timed_straight_reports_command_duration_without_distance():
    sim = Sim()
    sim.straight()
    while sim.core.state == 'STRAIGHT_1':
        command = sim.tick()
        assert command.angular_z == 0 and 0 <= command.linear_x <= 0.03
    assert sim.core.state == 'SETTLING'
    result = sim.core.last_motion
    assert result['completed']
    assert result['command_duration_sec'] <= 1.5
    assert result['duration_sec'] == pytest.approx(1.5)
    assert not any('distance' in key for key in result)
    sim.until('READY')
    assert sim.tick() == ZERO


def test_short_straight_exact_hard_deadline_completes_then_zero():
    sim = Sim()
    sim.straight(duration=2.0)
    for _ in range(20):
        sim.tick()
    assert sim.core.last_motion['completed']
    assert sim.core.state == 'SETTLING'
    assert sim.core._last_command == ZERO


def test_straight_hard_timeout_stops_even_if_completion_target_is_corrupted():
    sim = Sim()
    sim.straight(duration=2.0)
    sim.core._straight_duration = 100.0  # Fault injection: cannot bypass hard timeout.
    for _ in range(18):
        sim.tick()
    assert sim.tick(dt=0.1) == ZERO
    assert sim.core.state == 'FAULT_HOLD'
    assert sim.core.last_motion['timeout']


def test_guard_uses_shared_precedence_and_discards_active_command():
    sim = Sim()
    sim.straight(kind='GUARD_TEST', duration=1.0)
    assert sim.tick(front=0.93) == ZERO
    assert sim.core.last_motion['guard']
    assert sim.core.old_primitive_discarded
    assert sim.core._candidate is None
    assert sim.core.state == 'PLAN_DETOUR'


@pytest.mark.parametrize('manual', [False, True])
def test_estop_release_discards_old_primitive_and_recovers_only_ready(manual):
    sim = Sim()
    sim.straight(kind='ESTOP_TRIGGER_TEST', duration=5.0)
    assert sim.tick(aggregate_estop=True, auto_estop=not manual,
                    manual_estop=manual, auto_reason='NEAR_OBSTACLE') == ZERO
    assert sim.core.state == 'HOLD_ESTOP'
    assert sim.core.last_motion['estop']
    assert sim.core.last_motion['estop_source'] == ('MANUAL' if manual else 'AUTO')
    assert 'EStop' in sim.core.last_motion['estop_reason']
    assert sim.core.old_primitive_discarded
    old_duration = sim.core.last_motion['command_duration_sec']
    for _ in range(10):
        assert sim.tick(aggregate_estop=True, auto_estop=True,
                        auto_reason='NEAR_OBSTACLE') == ZERO
    sim.until('READY')
    for _ in range(80):
        assert sim.tick() == ZERO
    assert sim.core._candidate is None or sim.core.phase == 'NONE'
    assert sim.core.last_motion['command_duration_sec'] == old_duration
    assert sim.core.old_primitive_discarded


def test_hold_without_started_primitive_does_not_claim_discard_validation():
    sim = Sim()
    assert sim.tick(auto_estop=True, aggregate_estop=True) == ZERO
    assert not sim.core.old_primitive_discarded


def test_abort_ctrl_c_remains_zero_and_does_not_release_estop():
    sim = Sim()
    sim.straight()
    assert sim.core.abort(sim.now, 'SIGINT') == ZERO
    assert sim.core.state == 'ABORTED'
    assert sim.core.old_primitive_discarded
    assert sim.core.last_motion['interrupted']
    assert sim.core.last_motion['fault'] is None
    for _ in range(20):
        assert sim.tick(auto_estop=True, aggregate_estop=True) == ZERO
    assert not sim.core.armed


def test_forward_session_budget_caps_remaining_estop_duration():
    sim = Sim(max_total_forward_command_sec=2.0)
    sim.straight()
    while sim.core.state == 'STRAIGHT_1':
        sim.tick()
    sim.until('READY')
    remaining = 2.0 - sim.core.total_forward_command_sec
    assert sim.core.request_straight(sim.now, sim.obs, kind='ESTOP_TRIGGER_TEST')
    assert sim.core._straight_duration == pytest.approx(
        remaining - sim.core.config.control_timeout_sec)
    for _ in range(15):
        sim.tick()
    assert sim.core.total_forward_command_sec <= 2.0 + 1e-8
    assert not sim.core.request_straight(sim.now, sim.obs)
    assert sim.core.state == 'ABORTED'


def test_rotation_budget_reserves_requested_turn_and_rejects_extra():
    sim = Sim(max_total_rotation_deg=20.0)
    sim.kwargs['box'] = 'right'
    sim.until('READY')
    for _ in range(8):
        sim.tick()
    assert not sim.core.request_turn(sim.now, sim.obs, 'LEFT')
    assert sim.core.state == 'ABORTED'


def test_total_session_timeout_includes_operator_waits():
    sim = Sim(max_test_duration_sec=2.0)
    for _ in range(20):
        assert sim.tick() == ZERO
    assert sim.core.state == 'ABORTED'
    assert sim.core.expired


def test_control_timer_stall_aborts_active_motion():
    sim = Sim()
    sim.straight()
    assert sim.tick(dt=0.5) == ZERO
    assert sim.core.state == 'ABORTED'
    assert sim.core.last_motion['timeout']


@pytest.mark.parametrize('values', [
    {'enable_auto_estop': False}, {'test_turn_speed': 0.3},
    {'test_turn_timeout_sec': 6.0}, {'max_total_forward_command_sec': 6.0},
    {'max_total_rotation_deg': 91}, {'test_straight_duration': -1.0},
    {'imu_timeout_sec': math.nan}, {'detour_guard_distance_m': 0.4},
    {'stop_settle_sec': 0.1}, {'test_turn_tolerance_deg': 25},
])
def test_conservative_config_cannot_relax_initial_safety_caps(values):
    with pytest.raises(ValueError):
        GuidedConfig(**values)


@pytest.mark.parametrize('speed,duration', [(-0.03, 1), (0.1, 1), (0.03, 3), (math.nan, 1)])
def test_invalid_reverse_or_unbounded_straight_rejected(speed, duration):
    sim = Sim()
    with pytest.raises(ValueError):
        sim.core.request_straight(sim.now, sim.obs, speed=speed, duration=duration)


def test_observation_ready_clears_after_operator_wait_without_blind_motion():
    sim = Sim()
    sim.kwargs['box'] = 'right'
    sim.until('READY')
    for _ in range(40):
        assert sim.tick() == ZERO
    sim.kwargs.clear()
    sim.until('CLEARING')
    sim.until('NORMAL')
    assert sim.core.last_motion == {}


def test_restart_observation_preserves_arm_and_cumulative_limits():
    sim = Sim()
    sim.core.total_forward_command_sec = 1.2
    sim.core.total_rotation_deg = 20.0
    assert sim.core.restart_observation(sim.now, sim.obs)
    assert sim.core.armed
    assert sim.core.total_forward_command_sec == 1.2
    assert sim.core.total_rotation_deg == 20.0


def test_recovery_needs_scans_acquired_after_release_settling():
    sim = Sim()
    sim.straight()
    sim.tick(aggregate_estop=True, auto_estop=True)
    released_at = sim.now
    for _ in range(7):
        stale = observation(released_at)
        assert sim.tick(lidar=stale.lidar, realsense=stale.realsense,
                        perimeter=stale.perimeter) == ZERO
        assert sim.core.state == 'HOLD_ESTOP'
    sim.until('READY')
    assert sim.core.old_primitive_discarded


def test_turn_lost_perimeter_geometry_stops_without_forward_fallback():
    sim = Sim()
    sim.turn()
    sim.tick()
    partial = replace(sim.obs.perimeter, rays=tuple(
        ray for ray in sim.obs.perimeter.rays if abs(ray.angle) < math.pi / 2))
    assert sim.tick(perimeter=replace(partial, stamp=sim.now + 0.1)) == ZERO
    assert sim.core.state == 'BLOCKED'
    assert sim.core.old_primitive_discarded


def test_normal_straight_deadline_with_small_scheduler_jitter_completes():
    sim = Sim()
    sim.straight(duration=2.0)
    for _ in range(18):
        sim.tick()
    assert sim.tick(dt=0.11) == ZERO
    assert sim.core.state == 'SETTLING'
    assert sim.core.last_motion['completed']
    assert not sim.core.last_motion['timeout']


def test_straight_disabled_parameter_is_respected():
    sim = Sim(allow_straight_motion=False)
    assert not sim.core.request_straight(sim.now, sim.obs)
    assert sim.tick() == ZERO
