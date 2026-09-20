"""Interrupted option 18 advances must finish once, then hand back to Nav2."""
from dataclasses import replace

import pytest

from test_simple_detour import config, observation
from detour_core import Candidate, DetourGate, Pose, Scene, Velocity


def start_forward(gate, now=1.0, x=0.0, front=5.0):
    """Enter the first forward segment after an already completed safe turn."""
    obs = observation(now, Pose(x, 0.0, 0.0), front=front)
    gate._simple_side = gate.side = 'RIGHT'
    gate._candidate = Candidate('RIGHT', 0.0, gate.config.detour_first_leg_m, front)
    gate._start_phase('STRAIGHT_1', now, obs, Scene(obs, gate.config, now))
    assert gate.state == 'STRAIGHT_1'
    command = gate.tick(now + 0.01, observation(now + 0.01, obs.pose))
    assert command.linear_x > 0.0
    return now + 0.01


def resume_forward(gate, now, x):
    """Supply fresh stationary scans until the stopped heading is certified."""
    for _ in range(60):
        now += 0.1
        command = gate.tick(now, observation(now, Pose(x, 0.0, 0.0)))
        if gate.state == 'STRAIGHT_1':
            return now
        assert command == Velocity()
    pytest.fail('forward segment did not resume: '
                + gate.state + ': ' + gate.reason)


def await_handback(gate, now, x):
    for _ in range(60):
        now += 0.1
        command = gate.tick(now, observation(now, Pose(x, 0.0, 0.0)))
        assert command == Velocity()
        if gate.recovery_requested:
            assert gate.force_replan_requested
            assert gate.state == 'RECOVER_TO_NAV'
            return now
    pytest.fail('forward completion did not request replan: ' + gate.state)


@pytest.mark.parametrize('interruption', ['sensor', 'estop'])
def test_forward_progress_survives_safe_interruptions(interruption):
    gate = DetourGate(config())
    now = start_forward(gate)
    now += 5.0
    obs = observation(now, Pose(0.4, 0.0, 0.0))
    assert gate.tick(now, obs).linear_x > 0.0
    now += 0.1
    obs = observation(now, obs.pose,
                      front=0.9 if interruption == 'guard' else 5.0)
    if interruption == 'sensor':
        obs = replace(obs, realsense=None)
    elif interruption == 'estop':
        obs = replace(obs, manual_estop=True)
    assert gate.tick(now, obs) == Velocity()
    now = resume_forward(gate, now, 0.4)
    now += 5.0
    assert gate.tick(now, observation(now, Pose(0.8, 0.0, 0.0))) == Velocity()
    await_handback(gate, now, 0.8)


def test_hold_translation_is_not_credited_as_commanded_forward_progress():
    gate = DetourGate(config())
    now = start_forward(gate)
    now += 4.0
    gate.tick(now, observation(now, Pose(0.3, 0.0, 0.0)))
    now += 0.1
    obs = observation(now, Pose(0.3, 0.0, 0.0))
    gate.tick(now, replace(obs, manual_estop=True))
    now += 0.1
    obs = observation(now, Pose(1.3, 0.0, 0.0))
    gate.tick(now, replace(obs, manual_estop=True))
    now = resume_forward(gate, now, 1.3)
    now += 3.0
    assert gate.tick(now, observation(now, Pose(1.6, 0.0, 0.0))).linear_x > 0.0
    assert not gate.force_replan_requested
    now += 3.0
    assert gate.tick(now, observation(now, Pose(1.81, 0.0, 0.0))) == Velocity()
    await_handback(gate, now, 1.81)


def test_forward_timeout_budget_includes_interruption_time():
    gate = DetourGate(config())
    now = start_forward(gate)
    now += 3.0
    gate.tick(now, observation(now, Pose(0.2, 0.0, 0.0)))
    now += 0.1
    gate.tick(now, replace(observation(now, Pose(0.2, 0.0, 0.0)), realsense=None))
    resume_forward(gate, now, 0.2)
    obs = observation(26.1, Pose(0.2, 0.0, 0.0))
    assert gate.tick(26.1, obs) == Velocity()
    assert gate.state == 'FAULT_HOLD'
    assert 'timeout' in gate.reason


@pytest.mark.parametrize('reset_method', ['new_goal', 'acknowledge'])
def test_new_maneuver_starts_with_fresh_distance_and_deadline(reset_method):
    gate = DetourGate(config())
    now = start_forward(gate)
    now += 10.0
    if reset_method == 'new_goal':
        gate.tick(now, observation(now, Pose(0.4, 0.0, 0.0)))
        gate.reset(now)
    else:
        gate.tick(now, observation(now, Pose(0.8, 0.0, 0.0)))
        now = await_handback(gate, now, 0.8)
        assert gate.acknowledge_recovery()
    now = start_forward(gate, now=30.0, x=2.0)
    now += 5.0
    assert gate.tick(now, observation(now, Pose(2.4, 0.0, 0.0))).linear_x > 0.0
    assert not gate.force_replan_requested
    now += 5.0
    assert gate.tick(now, observation(now, Pose(2.81, 0.0, 0.0))) == Velocity()
    await_handback(gate, now, 2.81)


def resume_at_pose(gate, now, pose, front=5.0):
    for _ in range(60):
        now += 0.1
        command = gate.tick(now, observation(now, pose, front=front))
        assert command == Velocity()
        assert gate.state != 'TURN_1', gate.reason
        if gate.state == 'STRAIGHT_1':
            return now
    pytest.fail('safe remaining advance did not resume: ' + gate.reason)


def test_remaining_distance_smaller_than_minimum_leg_can_finish():
    gate = DetourGate(config())
    now = start_forward(gate)
    now += 9.0
    gate.tick(now, observation(now, Pose(0.75, 0.0, 0.0)))
    now += 0.1
    gate.tick(now, replace(observation(now, Pose(0.75, 0.0, 0.0)), realsense=None))
    # At the outer 10-degree ray, projected clearance is about 1.22 m:
    # enough for the final .05 m plus reserve, but less than reserve + .10 m.
    now = resume_at_pose(gate, now, Pose(0.75, 0.0, 0.0), front=1.24)
    now += 1.0
    assert gate.tick(now, observation(now, Pose(0.81, 0.0, 0.0))) == Velocity()
    await_handback(gate, now, 0.81)


def test_forward_backward_pose_jitter_does_not_accumulate_distance():
    gate = DetourGate(config())
    now = start_forward(gate)
    for _ in range(12):
        for x in (0.05, 0.0):
            now += 0.5
            command = gate.tick(now, observation(now, Pose(x, 0.0, 0.0)))
            assert command.linear_x > 0.0
            assert gate.state == 'STRAIGHT_1'
            assert not gate.force_replan_requested
    now += 10.0
    assert gate.tick(now, observation(now, Pose(0.81, 0.0, 0.0))) == Velocity()
    await_handback(gate, now, 0.81)


def test_new_obstacle_restarts_full_distance_after_another_fixed_right_turn():
    import math

    gate = DetourGate(config(detour_first_leg_m=2.2, straight_phase_timeout_sec=60.))
    now = start_forward(gate, front=1.4)
    assert gate._forward_target == pytest.approx(2.2)
    now += 5.
    gate.tick(now, observation(now, Pose(.4, 0., 0.)))
    now += .1
    assert gate.tick(now, observation(now, Pose(.4, 0., 0.), front=1.)) == Velocity()
    assert gate.state == 'PHASE_SETTLE'
    for _ in range(20):
        now += .1
        command = gate.tick(now, observation(now, Pose(.4, 0., 0.), front=1.))
        if command.angular_z:
            break
    assert command.angular_z < 0.
    assert gate._candidate.target_yaw == pytest.approx(-math.pi / 4.)
    assert gate._forward_target is None
    assert gate._forward_since is None
    turned = Pose(.4, 0., -math.pi / 4.)
    now += 3.
    assert gate.tick(now, observation(now, turned)) == Velocity()
    now = resume_at_pose(gate, now, turned, front=1.4)
    assert gate._forward_target == pytest.approx(2.2)
    assert gate._forward_progress == 0.
    now += 10.
    pose = Pose(.4 + 1.8 / math.sqrt(2.), -1.8 / math.sqrt(2.), turned.yaw)
    assert gate.tick(now, observation(now, pose)).linear_x > 0.
    assert not gate.force_replan_requested
    now += 6.
    pose = Pose(.4 + 2.21 / math.sqrt(2.), -2.21 / math.sqrt(2.), turned.yaw)
    assert gate.tick(now, observation(now, pose)) == Velocity()
    assert gate.force_replan_requested
    for _ in range(30):
        now += .1
        assert gate.tick(now, observation(now, pose)) == Velocity()
        if gate.recovery_requested:
            break
    assert gate.recovery_requested


def test_completion_with_new_obstacle_restarts_cycle_instead_of_handback():
    gate = DetourGate(config())
    now = start_forward(gate)
    now += 10.
    assert gate.tick(now, observation(now, Pose(.8, 0., 0.), front=1.)) == Velocity()
    for _ in range(30):
        now += .1
        command = gate.tick(now, observation(now, Pose(.8, 0., 0.), front=1.))
        assert not gate.recovery_requested
        if command.angular_z:
            break
    assert command.angular_z < 0.
    assert gate._forward_target is None
    assert not gate.force_replan_requested
