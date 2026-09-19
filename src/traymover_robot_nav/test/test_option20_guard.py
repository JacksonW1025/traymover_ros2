"""Option 20 command safety contracts, independent of ROS/hardware."""
import importlib.util
import math
from pathlib import Path
import sys

import numpy as np
import pytest

SPEC = importlib.util.spec_from_file_location(
    'option20_guard', Path(__file__).resolve().parents[1] / 'scripts' / 'option20_guard.py')
mod = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = mod
SPEC.loader.exec_module(mod)


def healthy(gate, now):
    for name in ('odom', 'scan', 'lidar', 'depth', 'depth_status', 'amcl', 'odom_status', 'scan_status', 'lidar_status'):
        gate.freshness.receive(name, now, now, now)
    gate.localization_error = ''
    gate.localization_count = 3
    gate.scan_score = 0.7
    gate.finite_beams = 120
    gate.depth_healthy = True
    gate.auxiliary = {'odom_status': True, 'scan_status': True, 'lidar_status': True}
    gate.estop = {'aggregate': (False, now), 'manual': (False, now)}
    gate.auto_request = (False, now)
    gate.command = (0.14, 0.0)
    gate.command_time = now
    gate.points = {'lidar': np.empty((0, 2)), 'depth': np.empty((0, 2))}


def moving():
    gate = mod.Guard()
    for now in np.arange(1, 2.21, 0.05):
        healthy(gate, float(now))
        output, _ = gate.tick(float(now), float(now))
    assert output[0] > 0
    return gate, float(now)


def test_missing_everything_fails_closed():
    output, status = mod.Guard().tick(1, 1)
    assert output == (0, 0)
    assert 'odom:missing' in status['reasons']
    assert not status['ready']


@pytest.mark.parametrize('name', ['odom', 'scan', 'lidar', 'depth', 'depth_status', 'amcl', 'odom_status', 'scan_status', 'lidar_status'])
def test_loss_of_any_required_source_stops_immediately(name):
    gate, now = moving()
    healthy(gate, now + 0.05)
    del gate.freshness.samples[name]
    assert gate.tick(now + 0.05, now + 0.05)[0] == (0, 0)


def test_command_timeout_and_reverse_stop_immediately():
    gate, now = moving()
    healthy(gate, now + 0.4)
    gate.command_time = now
    assert gate.tick(now + 0.4, now + 0.4)[0] == (0, 0)
    healthy(gate, now + 0.45)
    gate.command = (-0.01, 0)
    assert gate.tick(now + 0.45, now + 0.45)[0] == (0, 0)


def test_nan_command_does_not_escape_gate():
    gate, now = moving()
    healthy(gate, now + 0.05)
    gate.command = (math.nan, math.inf)
    assert gate.tick(now + 0.05, now + 0.05)[0] == (0, 0)


@pytest.mark.parametrize('stamp', [0, math.nan, math.inf, 2.2, 0.9, 1.0])
def test_invalid_future_and_nonincreasing_stamps_fail_closed(stamp):
    freshness = mod.Freshness()
    assert freshness.receive('scan', 1, 1, 1)
    assert not freshness.receive('scan', stamp, 1.1, 1.1)
    assert freshness.reason('scan', 1.1, 1.1, 0.6, 0.1)
    assert freshness.receive('scan', 1.2, 1.2, 1.2)


def test_receive_timeout_detects_paused_ros_clock():
    freshness = mod.Freshness()
    freshness.receive('scan', 1, 1, 1)
    assert freshness.reason('scan', 1, 2, 0.6, 0.1)


def test_initialpose_does_not_make_localization_ready():
    gate, _ = moving()
    gate.reset_localization()
    assert gate.localization_count == 0
    assert gate.ready_since is None


def test_correction_removes_robot_motion_and_detects_global_jump():
    gate = mod.Guard()
    covariance = [0.0] * 36
    for index in range(4):
        correction = mod.correction_from_pose((index + 2, 1, 0), (index, 0, 0))
        gate.localization(correction, covariance)
    assert gate.localization_count == 4
    gate.localization((3, 1, 0), covariance)
    assert gate.localization_error == 'global_correction_jump'
    assert gate.localization_count == 0


@pytest.mark.parametrize('cov_value', [-1, 1, math.nan])
def test_bad_covariance_resets_readiness(cov_value):
    gate = mod.Guard()
    cov = [0.0] * 36
    cov[0] = cov_value
    gate.localization((0, 0, 0), cov)
    assert gate.localization_count == 0
    assert gate.localization_error


def test_scan_map_match_requires_finite_returns_and_respects_map_origin():
    data = np.zeros((10, 10), dtype=int)
    data[0, 2] = 100
    matcher = mod.MapMatcher(data.ravel(), 10, 10, 1.0, (4, 3, 0), 0.3)
    assert matcher.score([2, math.inf, math.nan], 0, 0.1, 0.1, 10, (4, 3, 0)) == (1, 1)
    assert matcher.score([math.inf] * 30, 0, 0.1, 0.1, 10, (4, 3, 0)) == (0, 0)
    assert matcher.score([2], 0, 0.1, 0.1, 10, (100, 3, 0)) == (0, 1)


def test_map_dilation_does_not_wrap_edges():
    data = np.zeros((10, 10), dtype=int)
    data[0, 0] = 100
    matcher = mod.MapMatcher(data.ravel(), 10, 10, 0.1, (0, 0, 0), 0.3)
    assert matcher.hits[0, 2]
    assert not matcher.hits[-1, -1]


@pytest.mark.parametrize('xy', [(0.5, 0), (0, 0.4), (-0.5, 0)])
def test_full_perimeter_stop_includes_side_and_rear(xy):
    gate, now = moving()
    healthy(gate, now + 0.05)
    gate.points['depth'] = np.asarray([xy], dtype=float)
    assert gate.tick(now + 0.05, now + 0.05)[0] == (0, 0)


def test_rotation_protects_corner_swept_radius():
    gate = mod.Guard()
    gate.points['lidar'] = np.asarray([(0.0, 0.60)])
    assert gate.obstacle_scale(0.0, 0.35) == 0


def test_actual_velocity_increases_stopping_envelope():
    gate = mod.Guard()
    gate.points['lidar'] = np.asarray([(0.65, 0.0)])
    assert gate.obstacle_scale(0.0, 0.0) > 0
    gate.measured_speed = 0.3
    assert gate.obstacle_scale(0.0, 0.0) == 0


def test_release_delay_and_acceleration_are_bounded():
    gate = mod.Guard()
    previous = 0
    for now in np.arange(1, 3, 0.05):
        healthy(gate, float(now))
        output, _ = gate.tick(float(now), float(now))
        if now < 1.8:
            assert output == (0, 0)
        assert output[0] <= gate.cfg.max_linear
        assert output[0] - previous <= gate.cfg.linear_accel * 0.05 + 1e-9
        previous = output[0]


def test_clearing_obstacle_requires_new_stability_period():
    gate, now = moving()
    healthy(gate, now + 0.05)
    gate.points['lidar'] = np.asarray([(0.5, 0)])
    assert gate.tick(now + 0.05, now + 0.05)[0] == (0, 0)
    healthy(gate, now + 0.10)
    assert gate.tick(now + 0.10, now + 0.10)[0] == (0, 0)


@pytest.mark.parametrize('fault', ['estop', 'multiple_publishers', 'odom_status', 'scan_score', 'depth'])
def test_additional_faults_fail_closed(fault):
    gate, now = moving()
    healthy(gate, now + 0.05)
    if fault == 'estop':
        gate.estop['manual'] = (True, now + 0.05)
    elif fault == 'multiple_publishers':
        gate.publisher_exclusive = False
    elif fault == 'odom_status':
        gate.auxiliary['odom_status'] = False
    elif fault == 'scan_score':
        gate.scan_score = 0.1
    elif fault == 'depth':
        gate.depth_healthy = False
    assert gate.tick(now + 0.05, now + 0.05)[0] == (0, 0)


def test_cannot_weaken_hard_safety_limits():
    for params in ({'padding': 0.01}, {'max_linear': 0.5}, {'max_angular': 0.5}):
        with pytest.raises(ValueError):
            mod.Config(**params)


def test_ready_for_goal_without_any_command_never_emits_motion():
    gate = mod.Guard()
    for now in np.arange(1, 2.1, 0.05):
        healthy(gate, float(now))
        gate.command_time = None
        output, health = gate.tick(float(now), float(now))
        assert output == (0, 0)
        assert health['sensors_ready']
    assert health['ready_for_goal']
    assert not health['ready']
    assert health['reasons'] == ['command:stale']


def test_old_command_cannot_resume_after_sensor_release():
    gate = mod.Guard(mod.Config(command_timeout=5.0))
    for now in np.arange(1, 2.1, 0.05):
        healthy(gate, float(now))
        gate.command_time = 1.0
        output, health = gate.tick(float(now), float(now))
        assert output == (0, 0)
    assert health['ready_for_goal']
    assert 'command:awaiting_post_release_command' in health['reasons']
    healthy(gate, 2.15)
    assert gate.tick(2.15, 2.15)[0][0] > 0


def test_empty_cloud_with_invalid_lidar_status_stops():
    gate, now = moving()
    healthy(gate, now + .05)
    gate.auxiliary['lidar_status'] = False
    output, health = gate.tick(now + .05, now + .05)
    assert output == (0, 0)
    assert 'lidar_status:unhealthy' in health['reasons']


def test_latched_estop_events_need_no_heartbeat_when_auto_request_is_fresh():
    gate, now = moving()
    healthy(gate, now + .05)
    gate.estop = {'aggregate': (False, -1000000), 'manual': (False, -1000000)}
    output, health = gate.tick(now + .05, now + .05)
    assert health['ready']
    assert output[0] > 0


@pytest.mark.parametrize('active,offset', [(True, 0.0), (False, -0.31)])
def test_auto_request_active_or_lost_stops_even_with_latched_false(active, offset):
    gate, now = moving()
    healthy(gate, now + .05)
    gate.auto_request = (active, now + .05 + offset)
    output, health = gate.tick(now + .05, now + .05)
    assert output == (0, 0)
    assert 'estop:auto_request:active_or_stale' in health['reasons']
