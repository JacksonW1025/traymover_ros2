"""Option 19 single-button, line-return behavior without ROS hardware."""
from dataclasses import replace
import math
from pathlib import Path
import sys

import pytest


SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))

from detour_core import Pose, Ray, Scan, Velocity  # noqa: E402
from detour_motion import LineReturnConfig, LineReturnCore, LineReturnObservation  # noqa: E402


def scan(stamp, obstacle=False, obstacle_distance=1.05):
    rays = [Ray(0.0, 0.0, math.radians(degree),
                obstacle_distance if obstacle and abs(degree) <= 10 else 5.0,
                obstacle and abs(degree) <= 10,
                obstacle_distance if obstacle and abs(degree) <= 10 else None)
            for degree in range(-180, 181, 5)]
    return Scan(stamp, tuple(rays))


def observation(stamp, pose, obstacle=False, obstacle_distance=1.05):
    lidar = scan(stamp, obstacle, obstacle_distance)
    depth = scan(stamp, obstacle, obstacle_distance)
    return LineReturnObservation(
        pose=pose, lidar=lidar, realsense=depth, perimeter=lidar,
        aggregate_estop=False, manual_estop=False, auto_estop=False,
        auto_reason='NONE', estop_fresh=True, tf_healthy=True,
        localization_healthy=True, odom_stamp=stamp,
    )


def test_start_is_one_shot_and_clear_path_moves_straight():
    config = LineReturnConfig(obstacle_confirm_frames=1)
    core = LineReturnCore(config)
    obs = observation(1.0, Pose(2.0, -1.0, 0.3))

    assert core.tick(1.0, obs) == Velocity()
    assert core.start(1.0, obs)
    assert not core.start(1.0, obs)
    assert core.tick(1.05, observation(1.05, obs.pose)).linear_x == config.forward_speed
    assert core.original_line == obs.pose


def test_option19_does_not_require_production_three_angle_depth_coverage():
    core = LineReturnCore(LineReturnConfig())
    pose = Pose(0.0, 0.0, 0.0)
    obs = observation(1.0, pose)
    narrow_depth = Scan(1.0, (Ray(0.45, 0.0, 0.3, 3.0, True, 2.8),))
    obs = replace(obs, realsense=narrow_depth)

    assert core.start(1.0, obs)
    assert core.tick(1.1, replace(obs, odom_stamp=1.1,
                                  lidar=scan(1.1), perimeter=scan(1.1),
                                  realsense=replace(narrow_depth, stamp=1.1))).linear_x > 0.0


def test_persistent_front_obstacle_stops_before_planning_detour():
    config = LineReturnConfig(obstacle_confirm_frames=1)
    core = LineReturnCore(config)
    start = Pose(0.0, 0.0, 0.0)
    assert core.start(1.0, observation(1.0, start))

    command = core.tick(1.1, observation(1.1, start, obstacle=True))

    assert command == Velocity()
    assert core.state == 'WAITING'
    assert core.original_line == start


def test_option19_always_chooses_fixed_right_detour_without_corridor_check():
    config = LineReturnConfig(detour_turn_angle_deg=45.0, detour_first_leg_m=1.2)
    core = LineReturnCore(config)
    pose = Pose(0.0, 0.0, 0.2)
    obs = observation(1.0, pose, obstacle=True)

    candidate = core._select_candidate(None, obs)

    assert candidate.side == 'RIGHT'
    assert candidate.target_yaw == pytest.approx(0.2 - math.pi / 4.0)
    assert candidate.first_leg_m == 1.2
    assert core._phase_guard_blocked(None) is False
    assert core._rotation_clear(None) is True


def test_forward_motion_tracks_encoder_drift_until_first_obstacle():
    config = LineReturnConfig(line_tolerance_m=0.10)
    core = LineReturnCore(config)
    start = Pose(0.0, 0.0, 0.0)
    assert core.start(1.0, observation(1.0, start))
    current = Pose(0.2, 0.11, math.radians(6.0))

    command = core.tick(1.1, observation(1.1, current))

    assert command.linear_x == config.forward_speed
    assert core.state == 'NORMAL'
    assert core.original_line == current


def test_return_plan_targets_the_recorded_original_line():
    config = LineReturnConfig(return_forward_m=0.8)
    core = LineReturnCore(config)
    origin = Pose(1.0, 2.0, math.radians(30.0))
    assert core.start(1.0, observation(1.0, origin))
    side_pose = Pose(
        origin.x + 1.5 * math.cos(origin.yaw) - 0.7 * math.sin(origin.yaw),
        origin.y + 1.5 * math.sin(origin.yaw) + 0.7 * math.cos(origin.yaw),
        origin.yaw,
    )

    core.begin_return(2.0, observation(2.0, side_pose))
    target = core.return_target
    forward, lateral = core.line_coordinates(target)

    assert forward > 1.5
    assert lateral == pytest.approx(0.0, abs=1e-9)
    assert core.leg == 'RETURN'
    assert core._candidate.first_leg_m > 0.7


def test_completion_requires_measured_line_and_heading_tolerance():
    config = LineReturnConfig(line_tolerance_m=0.10, heading_tolerance_deg=5.0)
    core = LineReturnCore(config)
    origin = Pose(0.0, 0.0, 0.0)
    assert core.start(1.0, observation(1.0, origin))

    assert core.verify_return(Pose(3.0, 0.08, math.radians(4.0)))
    assert not core.verify_return(Pose(3.0, 0.12, 0.0))
    assert not core.verify_return(Pose(3.0, 0.0, math.radians(6.0)))


def test_estop_always_overrides_single_button_motion():
    core = LineReturnCore(LineReturnConfig())
    pose = Pose(0.0, 0.0, 0.0)
    clear = observation(1.0, pose)
    assert core.start(1.0, clear)
    stopped = LineReturnObservation(**{
        **clear.__dict__, 'aggregate_estop': True, 'auto_estop': True,
        'auto_reason': 'NEAR_OBSTACLE',
    })

    assert core.tick(1.1, stopped) == Velocity()
    assert core.state == 'HOLD_ESTOP'


def test_complete_maneuver_returns_to_original_line_before_success():
    config = LineReturnConfig(
        obstacle_confirm_frames=1,
        wait_before_detour_sec=0.05,
        clear_confirm_sec=0.05,
        stop_settle_time_sec=0.05,
        pose_stability_time_sec=0.05,
        detour_first_leg_m=1.2,
        detour_forward_leg_m=1.0,
    )
    core = LineReturnCore(config)
    now = 1.0
    pose = Pose(0.0, 0.0, 0.0)
    assert core.start(now, observation(now, pose))
    now += 0.1
    core.tick(now, observation(now, pose, obstacle=True, obstacle_distance=0.55))

    for unused in range(200):
        now += 0.1
        state = core.state
        obs = observation(now, pose, state in ('WAITING', 'PLAN_DETOUR', 'TURN_1'),
                          obstacle_distance=0.55)

        if state.startswith('TURN'):
            target = (core._candidate.target_yaw if state == 'TURN_1'
                      else core._origin_yaw)
            pose = Pose(pose.x, pose.y, target)
            obs = observation(now, pose, state == 'TURN_1', obstacle_distance=0.55)
        elif state.startswith('STRAIGHT') and core._phase_distance is not None:
            start = core._phase_start
            distance = core._phase_distance
            pose = Pose(
                start.x + distance * math.cos(start.yaw),
                start.y + distance * math.sin(start.yaw),
                start.yaw,
            )
            obs = observation(now, pose)

        core.tick(now, obs)
        if core.state == 'COMPLETE':
            break

    assert core.state == 'COMPLETE', core.reason
    assert core.verify_return(pose)
    unused_forward, lateral = core.line_coordinates(pose)
    assert lateral == pytest.approx(0.0, abs=config.line_tolerance_m)
    assert pose.yaw == pytest.approx(0.0, abs=math.radians(config.heading_tolerance_deg))
