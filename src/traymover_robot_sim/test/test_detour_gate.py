import pytest

from traymover_robot_sim.detour_supervisor import (
    DetourGate, GateState, SafetyObservation,
)


def sample(t, nav=0.0, output=0.0, obstacle=False, estop=False):
    return SafetyObservation(t, nav, output, obstacle, estop)


def test_blocking_starts_wait_without_forwarding():
    gate = DetourGate(hold_time_sec=8.0)
    decision = gate.update(sample(10.0, nav=0.3, output=0.0, obstacle=True))
    assert decision.state is GateState.STOP_WAITING
    assert not decision.forward_global_scan


def test_blocking_for_eight_seconds_enables_global_scan():
    gate = DetourGate(hold_time_sec=8.0)
    gate.update(sample(10.0, nav=0.3, output=0.0, obstacle=True))
    decision = gate.update(sample(18.0, nav=0.3, output=0.0, obstacle=True))
    assert decision.state is GateState.DETOUR_ACTIVE
    assert decision.forward_global_scan


def test_obstacle_clear_before_timeout_resets_without_detour():
    gate = DetourGate(hold_time_sec=8.0)
    gate.update(sample(10.0, nav=0.3, output=0.0, obstacle=True))
    decision = gate.update(sample(13.0, nav=0.3, output=0.3, obstacle=False))
    assert decision.state is GateState.NORMAL
    assert not decision.forward_global_scan


def test_clear_after_detour_uses_a_clear_window():
    gate = DetourGate(hold_time_sec=8.0, clear_publish_sec=1.5)
    gate.update(sample(0.0, nav=0.3, output=0.0, obstacle=True))
    gate.update(sample(8.0, nav=0.3, output=0.0, obstacle=True))
    clearing = gate.update(sample(9.0, nav=0.3, output=0.3, obstacle=False))
    assert clearing.state is GateState.CLEARING
    assert clearing.forward_global_scan
    normal = gate.update(sample(10.6, nav=0.3, output=0.3, obstacle=False))
    assert normal.state is GateState.NORMAL
    assert not normal.forward_global_scan


def test_estop_alone_without_obstacle_does_not_trigger_detour():
    gate = DetourGate()
    decision = gate.update(sample(1.0, estop=True, obstacle=False))
    assert decision.state is GateState.NORMAL


def test_estop_with_front_obstacle_starts_waiting():
    gate = DetourGate()
    decision = gate.update(sample(1.0, estop=True, obstacle=True))
    assert decision.state is GateState.STOP_WAITING


def test_decreasing_timestamp_is_rejected():
    gate = DetourGate()
    gate.update(sample(2.0))
    with pytest.raises(ValueError, match="timestamps must not decrease"):
        gate.update(sample(1.0))
