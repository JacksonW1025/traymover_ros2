"""Option 18 sparse-scan detours and transparent normal navigation."""
from dataclasses import replace
import math
from pathlib import Path
import sys

import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from detour_core import (  # noqa: E402
    Candidate, Config, DetourGate, Observation, Pose, Ray, Scan, Scene, Velocity,
)


def observation(now, pose=None, front=5.0, left=5.0, right=5.0):
    lidar = Scan(now, (
        Ray(0.0, 0.0, -math.pi / 4.0, right),
        Ray(0.0, 0.0, 0.0, front),
        Ray(0.0, 0.0, math.pi / 4.0, left),
    ))
    depth = Scan(now, tuple(Ray(0.0, 0.0, math.radians(angle), front)
                            for angle in (-10., -0.6, -0.2, 0.2, 0.6, 10.)))
    return Observation(
        pose=pose, lidar=lidar, realsense=depth,
        nav_cmd=Velocity(0.35, -0.4), nav_cmd_stamp=now, nav_goal_active=True,
        auto_reason='NONE', estop_fresh=True, tf_healthy=pose is not None,
        localization_healthy=False,
    )


def config(**kwargs):
    return Config(simple_detour=True, obstacle_confirm_frames=1,
                  wait_before_detour_sec=1.0, stop_settle_time_sec=0.25,
                  clear_confirm_sec=0.4, **kwargs)


def test_normal_navigation_preserves_speed_without_odom_map_or_ndt():
    gate = DetourGate(config())
    obs = observation(1.0)
    assert gate.tick(1.0, obs) is obs.nav_cmd
    assert gate.state == 'NORMAL'


def test_optional_stale_perimeter_cannot_block_normal_navigation_or_turns():
    obs = replace(observation(2.0),
                  perimeter=Scan(0.0, (Ray(0.0, 0.0, math.pi, 0.1),)))
    assert DetourGate(config()).tick(2.0, obs) is obs.nav_cmd
    assert Scene(obs, config(), 2.0).rotation_clear()
    assert Scene(replace(obs, perimeter=replace(obs.perimeter, stamp=2.0)),
                 config(), 2.0).rotation_clear()


@pytest.mark.parametrize('field, state', [
    ('realsense', 'NORMAL'), ('realsense', 'TURN_1'),
])
def test_stale_required_sensor_stops_navigation_and_detour(field, state):
    obs = observation(2.0, Pose(0.0, 0.0, 0.0))
    obs = replace(obs, **{field: replace(getattr(obs, field), stamp=0.0)})
    gate = DetourGate(config())
    gate.state = state
    assert gate.tick(2.0, obs) == Velocity()
    assert gate.state == 'FAULT_HOLD'


@pytest.mark.parametrize('field', ['manual_estop', 'auto_estop', 'aggregate_estop'])
@pytest.mark.parametrize('state', ['NORMAL', 'TURN_1'])
def test_estop_always_stops_simple_navigation(field, state):
    gate = DetourGate(config())
    gate.state = state
    obs = observation(1.0, Pose(0.0, 0.0, 0.0))
    assert gate.tick(1.0, replace(obs, **{field: True})) == Velocity()
    assert gate.state == 'HOLD_ESTOP'


def test_unobserved_heading_never_authorizes_autonomous_translation():
    obs = observation(1.0)
    side_scan = Scan(1.0, (Ray(0.0, 0.0, math.pi / 2.0, 5.0),))
    obs = replace(obs, lidar=side_scan, realsense=side_scan)
    gate = DetourGate(config())
    # Nav2 retains its own ordinary planning/velocity behavior.
    assert gate.tick(1.0, obs) is obs.nav_cmd
    scene = Scene(obs, gate.config)
    assert scene.clearance() == 0.0
    assert scene.candidate('LEFT', Pose(0.0, 0.0, 0.0)) is not None
    assert scene.candidate('RIGHT', Pose(0.0, 0.0, 0.0)) is not None
    gate._start_phase('STRAIGHT_2', 1.0, obs, scene)
    assert gate.state == 'PLAN_DETOUR'


def test_camera_can_observe_corridor_when_lidar_has_no_forward_return():
    obs = observation(1.0)
    obs = replace(obs, lidar=Scan(1.0, (Ray(0.0, 0.0, math.pi / 2.0, 5.0),)))
    scene = Scene(obs, config())
    assert scene.clearance() > 4.9
    # A close LiDAR hit still vetoes camera free space.
    obs = replace(obs, lidar=Scan(1.0, (Ray(0.0, 0.0, 0.0, 0.9),)))
    assert Scene(obs, config()).clearance() == pytest.approx(0.9)


def test_close_front_obstacle_allows_safe_rotation_but_blocks_translation():
    gate = DetourGate(config())
    pose = Pose(0.0, 0.0, 0.0)
    for step in range(50):
        now = 1.0 + step * 0.1
        command = gate.tick(now, observation(now, pose, front=0.9))
        assert command.linear_x == 0.0
        if command.angular_z < 0.0:
            break
    assert gate.state == 'TURN_1', gate.reason
    assert command.angular_z < 0.0
    # A still-obstructed new heading cannot start the first straight segment.
    turned = Pose(0.0, 0.0, -math.pi / 4.0)
    for step in range(10):
        now += 0.1
        assert gate.tick(now, observation(now, turned, front=0.9)).linear_x == 0.0
    assert gate.state != 'STRAIGHT_1'


def test_simple_detour_requires_current_odom_once_maneuver_starts():
    gate = DetourGate(config())
    gate.state = 'TURN_1'
    assert gate.tick(1.0, observation(1.0)) == Velocity()
    assert gate.state == 'FAULT_HOLD'
    assert 'TF unavailable/stale' in gate.reason


def test_sensor_fault_can_recover_to_normal_without_odom():
    gate = DetourGate(config())
    gate.tick(1.0, replace(observation(1.0), realsense=None))
    for step in range(1, 50):
        now = 1.0 + step * 0.1
        assert gate.tick(now, observation(now)) == Velocity()
        if gate.recovery_requested:
            break
    assert gate.acknowledge_recovery()
    now += 0.1
    obs = observation(now)
    assert gate.tick(now, obs) is obs.nav_cmd


@pytest.mark.parametrize('side', ['RIGHT'])
def test_complete_simple_detour_uses_measured_pose_and_hands_back(side):
    gate = DetourGate(config(detour_first_leg_m=2.2, straight_phase_timeout_sec=60.))
    pose = Pose(0.0, 0.0, 0.0)
    visited = set()
    saw_turn = saw_straight = False
    now = 1.0
    for step in range(800):
        now += 0.1
        # Camera observes obstacle until the selected fixed turn completes.
        front = 1.05 if abs(pose.yaw) < math.radians(40.) else 5.
        obs = observation(now, pose, front=front,
                          left=5. if side == 'LEFT' else 1.2,
                          right=5. if side == 'RIGHT' else 1.2)
        command = gate.tick(now, obs)
        visited.add(gate.state)
        if gate.state == 'WAITING':
            assert command == Velocity()
        if gate.state == 'TURN_1' and command.angular_z != 0.0:
            saw_turn = True
            assert command.linear_x == 0.0
            assert gate.side == side
        if command.linear_x > 0.0:
            saw_straight = True
            assert command.linear_x <= 0.08
            assert command.angular_z == pytest.approx(0.0)
        # Integrate the actual output; phase completion cannot advance on time alone.
        pose = Pose(pose.x + command.linear_x * math.cos(pose.yaw) * 0.1,
                    pose.y + command.linear_x * math.sin(pose.yaw) * 0.1,
                    pose.yaw + command.angular_z * 0.1)
        if gate.recovery_requested:
            break
    assert gate.state == 'RECOVER_TO_NAV', gate.reason
    assert gate.recovery_requested
    assert {'TURN_1', 'STRAIGHT_1'}.issubset(visited)
    assert not {'TURN_2', 'STRAIGHT_2'}.intersection(visited)
    assert saw_turn and saw_straight
    assert abs(pose.y) > 0.5
    assert abs(abs(pose.yaw) - math.pi / 4.) <= gate.config.yaw_tolerance_rad
    assert gate.acknowledge_recovery()
    obs = observation(now + 0.1, pose)
    assert gate.tick(now + 0.1, obs) is obs.nav_cmd


@pytest.mark.parametrize('lidar', [None, Scan(0.0, (Ray(0., 0., 0., 0.1),))])
def test_normal_navigation_does_not_require_lidar(lidar):
    obs = replace(observation(2.0), lidar=lidar)
    gate = DetourGate(config())
    assert gate.tick(2.0, obs) is obs.nav_cmd
    assert gate.state == 'NORMAL'


@pytest.mark.parametrize('state', ['NORMAL', 'FAULT_HOLD', 'HOLD_ESTOP',
                                   'BLOCKED', 'RECOVER_TO_NAV', 'PLAN_DETOUR'])
def test_lidar_only_obstacle_never_restarts_detour(state):
    gate = DetourGate(config())
    gate.state = state
    for step in range(40):
        now = 1.0 + step * 0.1
        obs = observation(now, Pose(0., 0., 0.))
        obs = replace(obs, lidar=Scan(now, tuple(
            Ray(0., 0., math.radians(angle), 0.85)
            for angle in (-0.6, -0.2, 0.2, 0.6))))
        command = gate.tick(now, obs)
        assert gate.state not in gate.PHASES + ('WAITING',)
        assert command in (Velocity(), obs.nav_cmd)
        if gate.recovery_requested:
            assert gate.acknowledge_recovery()
    assert gate.state == 'NORMAL'


def test_sparse_camera_noise_does_not_trigger_detour():
    gate = DetourGate(config())
    for step in range(20):
        now = 1. + step * 0.1
        obs = observation(now)
        obs = replace(obs, realsense=Scan(now, tuple(
            Ray(0., 0., math.radians(angle), 0.95)
            for angle in (-12., -4., 4., 12.))))
        assert gate.tick(now, obs) is obs.nav_cmd
        assert gate.state == 'NORMAL'


def test_camera_side_cluster_outside_forward_corridor_does_not_trigger():
    gate = DetourGate(config())
    obs = observation(1.)
    obs = replace(obs, realsense=Scan(1., tuple(
        Ray(0., 0., math.radians(angle), 1.)
        for angle in (25., 25.2, 25.4, 25.6))))
    assert gate.tick(1., obs) is obs.nav_cmd
    assert gate.state == 'NORMAL'


def test_camera_confirmation_counts_new_depth_frames_without_new_lidar():
    gate = DetourGate(Config(simple_detour=True, obstacle_confirm_frames=3))
    for stamp, expected in [(1., 'NORMAL'), (1., 'NORMAL'),
                            (1.1, 'NORMAL'), (1.2, 'WAITING')]:
        obs = observation(stamp, front=1.0)
        obs = replace(obs, lidar=replace(obs.lidar, stamp=1.0))
        gate.tick(stamp, obs)
        assert gate.state == expected


@pytest.mark.parametrize('state', ['FAULT_HOLD', 'HOLD_ESTOP', 'BLOCKED', 'RECOVER_TO_NAV'])
def test_recovery_ignores_single_camera_noise_frame(state):
    gate = DetourGate(Config(simple_detour=True, obstacle_confirm_frames=3))
    gate.state = state
    for step in range(100):
        now = 1. + step * 0.1
        obs = observation(now, Pose(0., 0., 0.), front=1.0 if step % 2 else 5.)
        gate.tick(now, obs)
        assert gate.state not in gate.PHASES + ('PLAN_DETOUR', 'WAITING')
        if gate.recovery_requested:
            gate.acknowledge_recovery()
    assert gate.state == 'NORMAL'


def test_pending_low_valid_confirmation_preserves_navigation_with_valid_scan():
    obs = replace(observation(1.), auto_reason='LOW_VALID_DEPTH')
    gate = DetourGate(config())
    assert gate.tick(1., obs) is obs.nav_cmd
    assert gate.state == 'NORMAL'


@pytest.mark.parametrize('change', [
    {'auto_estop': True}, {'aggregate_estop': True},
    {'realsense': None}, {'realsense': Scan(1., ())},
    {'realsense': Scan(-1., (Ray(0., 0., 0., 5.),))},
    {'auto_reason': 'DEPTH_TIMEOUT'}, {'auto_reason': 'INVALID_FRAME'},
])
def test_low_valid_debounce_never_bypasses_stop_or_scan_watchdog(change):
    obs = replace(observation(1.), auto_reason='LOW_VALID_DEPTH')
    gate = DetourGate(config())
    assert gate.tick(1., replace(obs, **change)) == Velocity()
    assert gate.state in ('FAULT_HOLD', 'HOLD_ESTOP')


@pytest.mark.parametrize('stop', [{'auto_estop': True}, {'auto_reason': 'DEPTH_TIMEOUT'}])
def test_hold_restarts_confirmation_and_does_not_count_held_frame(stop):
    gate = DetourGate(Config(simple_detour=True, obstacle_confirm_frames=3))
    for stamp in (1., 1.1):
        gate.tick(stamp, observation(stamp, front=1.))
    assert gate._obstacle_frames == 2
    gate.tick(1.2, replace(observation(1.2, front=1.), **stop))
    assert gate._obstacle_frames == 0
    # The frame observed during the stop cannot count when the stop releases.
    gate.tick(1.2, observation(1.2, front=1.))
    assert gate._obstacle_frames == 0
    for count, stamp in enumerate((1.3, 1.4, 1.5), start=1):
        gate.tick(stamp, observation(stamp, front=1.))
        assert gate._obstacle_frames == count
        if count < 3:
            assert gate.state not in ('WAITING', 'PLAN_DETOUR')


def test_camera_cluster_normalizes_bearings_crossing_full_turn_wrap():
    obs = observation(1.)
    angles = (359.4, 359.8, 0.2, 0.6)
    obs = replace(obs, realsense=Scan(1., tuple(
        Ray(0., 0., math.radians(angle), 1.) for angle in angles)))
    assert Scene(obs, config()).obstacle()[0]
    gate = DetourGate(config())
    assert gate.tick(1., obs) == Velocity()
    assert gate.state == 'WAITING'


def step_pose(pose, command, dt=0.1):
    return Pose(pose.x + command.linear_x * math.cos(pose.yaw) * dt,
                pose.y + command.linear_x * math.sin(pose.yaw) * dt,
                pose.yaw + command.angular_z * dt)


def test_repeated_turns_keep_direction_then_translate_before_recovery():
    gate = DetourGate(config())
    pose = Pose(0., 0., 0.)
    targets = []
    for step in range(1000):
        now = 1. + step * .1
        front = 1. if abs(pose.yaw) < math.radians(80.) else 5.
        command = gate.tick(now, observation(now, pose, front=front))
        if gate.state == 'TURN_1' and command.angular_z:
            assert command.angular_z < 0.
            if not targets or targets[-1] != gate._candidate.target_yaw:
                targets.append(gate._candidate.target_yaw)
        pose = step_pose(pose, command)
        if gate.recovery_requested:
            break
    assert len(targets) == 2
    assert targets[0] == pytest.approx(-math.pi / 4.)
    assert targets[1] == pytest.approx(-math.pi / 2., abs=.035)
    assert gate.recovery_requested
    assert gate.force_replan_requested
    assert math.hypot(pose.x, pose.y) >= .8 - 1e-6
    assert gate.acknowledge_recovery()
    assert not gate.force_replan_requested


def test_fresh_obstacles_allow_more_than_four_fixed_right_turns():
    gate = DetourGate(config())
    pose = Pose(0., 0., 0.)
    for step in range(600):
        now = 1. + step * .1
        command = gate.tick(now, observation(now, pose, front=1.))
        assert command.linear_x == 0.
        assert command.angular_z <= 0.
        pose = step_pose(pose, command)
    assert gate._simple_turn_steps > 4
    assert not gate.recovery_requested


def settled_episode():
    gate = DetourGate(config())
    gate._simple_side = 'RIGHT'
    gate.side = 'RIGHT'
    gate._simple_turn_steps = 1
    gate._transition('PHASE_SETTLE', 1., 'test post-turn')
    return gate


def test_postturn_unknown_center_and_duplicate_camera_cannot_start_forward():
    gate = settled_episode()
    pose = Pose(0., 0., -math.pi / 4.)
    for stamp in (1.1, 1.4, 1.8, 2.2):
        obs = observation(stamp, pose)
        obs = replace(obs, realsense=Scan(stamp, (Ray(0., 0., .8, 5.),)))
        assert gate.tick(stamp, obs) == Velocity()
    assert gate.state == 'PHASE_SETTLE'
    frozen = observation(2.3, pose)
    for now in (2.3, 2.5, 2.8):
        obs = replace(frozen, lidar=replace(frozen.lidar, stamp=now))
        assert gate.tick(now, obs) == Velocity()
    assert gate.state == 'PHASE_SETTLE'
    assert not gate.recovery_requested


def test_forward_guard_reassesses_then_turns_same_direction():
    gate = settled_episode()
    pose = Pose(0., 0., -math.pi / 4.)
    for step in range(20):
        now = 1.1 + step * .1
        gate.tick(now, observation(now, pose))
        if gate.state == 'STRAIGHT_1':
            break
    assert gate.state == 'STRAIGHT_1'
    now += .1
    assert gate.tick(now, observation(now, pose, front=.9)) == Velocity()
    assert gate.state == 'PHASE_SETTLE'
    for step in range(20):
        now += .1
        command = gate.tick(now, observation(now, pose, front=.9))
        if gate.state == 'TURN_1' and command.angular_z:
            break
    assert command.angular_z < 0.
    assert gate._simple_turn_steps == 2
    assert not gate.recovery_requested


def test_forward_leg_uses_full_target_without_future_clearance_certificate():
    gate = settled_episode()
    pose = Pose(0., 0., -math.pi / 4.)
    for step in range(20):
        now = 1.1 + step * .1
        gate.tick(now, observation(now, pose, front=1.6))
        if gate.state == 'STRAIGHT_1':
            break
    assert gate.state == 'STRAIGHT_1'
    assert gate._phase_distance == gate.config.detour_first_leg_m
    for step in range(10):
        now += .1
        command = gate.tick(now, observation(now, pose, front=1.6))
        assert command.linear_x > 0.
        assert not gate.force_replan_requested
        assert not gate.recovery_requested
    progressed = Pose(gate._phase_distance * math.cos(pose.yaw) + 1e-6,
                      gate._phase_distance * math.sin(pose.yaw) + 1e-6, pose.yaw)
    now += .1
    assert gate.tick(now, observation(now, progressed, front=1.6)) == Velocity()
    assert gate.state == 'RECOVER_TO_NAV'
    assert gate.force_replan_requested


def test_simple_candidate_is_always_right_even_if_left_has_more_space():
    gate = DetourGate(config())
    obs = observation(1., Pose(0., 0., 0.), left=8., right=1.4)
    candidate = gate._select_candidate(Scene(obs, gate.config), obs)
    assert candidate.side == 'RIGHT'
    assert candidate.target_yaw == pytest.approx(-math.pi / 4.)


def test_cached_obstacle_after_turn_cannot_repeat_rotation():
    gate = settled_episode()
    gate._simple_obstacle_stamp = 2.
    obs = observation(2., Pose(0., 0., -math.pi / 4.), front=1.)
    for now in (2., 2.1, 2.2, 2.3, 2.4):
        assert gate.tick(now, obs) == Velocity()
        assert gate.state == 'PHASE_SETTLE'


def test_lidar_corridor_hit_does_not_clip_or_stop_simple_forward():
    gate = settled_episode()
    pose = Pose(0., 0., -math.pi / 4.)
    for step in range(30):
        now = 1.1 + step * .1
        obs = observation(now, pose, front=1.4)
        obs = replace(obs, lidar=Scan(now, (Ray(0., 0., 0., .9),)))
        command = gate.tick(now, obs)
        if command.linear_x:
            break
    assert command.linear_x > 0.
    assert gate._phase_distance == gate.config.detour_first_leg_m


def test_simple_rotation_uses_only_camera_not_lidar_or_perimeter():
    obs = observation(2., Pose(0., 0., 0.))
    noise = Scan(2., (Ray(0., 0., math.pi, .1),))
    assert Scene(replace(obs, lidar=noise, perimeter=noise), config(), 2.).rotation_clear()
    camera_close = Scan(2., (Ray(0., 0., 0., .49),))
    assert not Scene(replace(obs, realsense=camera_close), config(), 2.).rotation_clear()


def test_estop_interrupted_turn_finishes_measured_original_angle_before_advance():
    gate = DetourGate(config())
    initial = observation(1., Pose(0., 0., 0.), front=1.)
    scene = Scene(initial, gate.config)
    gate._candidate = gate._select_candidate(scene, initial)
    gate.side = gate._candidate.side
    gate._candidate_ready(1., initial, scene)
    target = gate._candidate.target_yaw
    pose = Pose(0., 0., -.2)
    gate.tick(1.1, replace(observation(1.1, pose), manual_estop=True))
    for step in range(50):
        now = 1.2 + step * .1
        command = gate.tick(now, observation(now, pose))
        assert command.linear_x == 0.
        if command.angular_z:
            break
    assert command.angular_z < 0.
    assert gate._candidate.target_yaw == target


def test_camera_guard_revokes_an_already_armed_nav_handback():
    gate = DetourGate(config())
    gate._simple_side = 'RIGHT'
    gate._transition('RECOVER_TO_NAV', 1.0, 'completed fixed advance')
    gate.force_replan_requested = True
    gate.recovery_requested = True
    obs = observation(2.0, Pose(0.0, 0.0, 0.0))
    # One close beam is not a confirmed detour cluster, but still blocks motion.
    camera = Scan(2.0, (Ray(0., 0., -.2, 4.), Ray(0., 0., 0., .75),
                        Ray(0., 0., .2, 4.)))
    assert gate.tick(2.0, replace(obs, realsense=camera)) == Velocity()
    assert gate.state == 'RECOVER_TO_NAV'
    assert not gate.recovery_requested
    assert not gate.acknowledge_recovery()
    for step in range(1, 20):
        now = 2.0 + step * .1
        assert gate.tick(now, observation(now, obs.pose)) == Velocity()
        if gate.recovery_requested:
            break
    assert gate.acknowledge_recovery()


@pytest.mark.parametrize('x,y', [(.2, .15), (-.06, 0.)])
@pytest.mark.parametrize('simple', [True, False])
def test_fixed_forward_small_pose_error_uses_correction_instead_of_latching(x, y, simple):
    cfg = Config(simple_detour=simple, detour_first_leg_m=2.2,
                 straight_phase_timeout_sec=60.)
    gate = DetourGate(cfg)
    start = observation(1., Pose(0., 0., 0.))
    start = replace(start, lidar=Scan(1., tuple(
        Ray(0., 0., math.radians(degree), 5.) for degree in range(-180, 181))))
    gate._candidate = Candidate('RIGHT', 0., 2.2, 5.)
    gate._start_phase('STRAIGHT_1', 1., start, Scene(start, cfg, 1.))
    assert gate.state == 'STRAIGHT_1'
    command = gate._straight_velocity(1.2, observation(1.2, Pose(x, y, 0.)))
    if simple:
        assert command.linear_x == cfg.detour_max_linear_speed
        assert abs(command.angular_z) <= cfg.detour_max_angular_speed
        assert gate.state == 'STRAIGHT_1'
        assert gate._latched_fault is None
    else:
        assert command == Velocity()
        assert gate.state == 'FAULT_HOLD'
