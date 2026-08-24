"""Pure time-gated detour state machine for simulation and safety testing."""

from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional


@dataclass(frozen=True)
class SafetyObservation:
    now_sec: float
    nav_speed: float
    output_speed: float
    front_obstacle: bool
    estop_active: bool = False


class GateState(Enum):
    NORMAL = auto()
    STOP_WAITING = auto()
    DETOUR_ACTIVE = auto()
    CLEARING = auto()


@dataclass(frozen=True)
class GateDecision:
    state: GateState
    forward_global_scan: bool


class DetourGate:
    """Gate global-scan forwarding after a sustained local stop condition."""

    def __init__(
        self,
        hold_time_sec: float = 8.0,
        nav_intent_threshold: float = 0.05,
        output_stop_threshold: float = 0.01,
        clear_publish_sec: float = 1.5,
    ) -> None:
        self.hold_time_sec = hold_time_sec
        self.nav_intent_threshold = nav_intent_threshold
        self.output_stop_threshold = output_stop_threshold
        self.clear_publish_sec = clear_publish_sec
        self._state = GateState.NORMAL
        self._blocked_since_sec: Optional[float] = None
        self._clear_started_sec: Optional[float] = None
        self._last_timestamp: Optional[float] = None

    def update(self, observation: SafetyObservation) -> GateDecision:
        now = observation.now_sec
        if self._last_timestamp is not None and now < self._last_timestamp:
            raise ValueError("observation timestamps must not decrease")
        self._last_timestamp = now

        blocking = observation.front_obstacle and (
            (
                observation.nav_speed >= self.nav_intent_threshold
                and observation.output_speed <= self.output_stop_threshold
            )
            or observation.estop_active
        )

        if self._state is GateState.NORMAL:
            if blocking:
                self._state = GateState.STOP_WAITING
                self._blocked_since_sec = now
            return self._decision()

        if self._state is GateState.STOP_WAITING:
            if not blocking:
                self._reset_normal()
            elif now - self._blocked_since_sec >= self.hold_time_sec:
                self._state = GateState.DETOUR_ACTIVE
            return self._decision()

        if self._state is GateState.DETOUR_ACTIVE:
            if not blocking:
                self._state = GateState.CLEARING
                self._clear_started_sec = now
            return self._decision()

        # CLEARING: a renewed block immediately resumes the detour.
        if blocking:
            self._state = GateState.DETOUR_ACTIVE
            self._clear_started_sec = None
        elif now - self._clear_started_sec >= self.clear_publish_sec:
            self._reset_normal()
        return self._decision()

    def _reset_normal(self) -> None:
        self._state = GateState.NORMAL
        self._blocked_since_sec = None
        self._clear_started_sec = None

    def _decision(self) -> GateDecision:
        return GateDecision(
            state=self._state,
            forward_global_scan=self._state
            in (GateState.DETOUR_ACTIVE, GateState.CLEARING),
        )
