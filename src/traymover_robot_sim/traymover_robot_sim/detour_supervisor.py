"""Time-gated detour state machine and its simulated ROS 2 wrapper."""

from dataclasses import dataclass
from enum import Enum, auto
from math import hypot, isfinite
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


def command_speed(twist: Twist) -> float:
    """Return the planar and angular command magnitude used by the gate."""

    return hypot(twist.linear.x, twist.linear.y) + abs(twist.angular.z)


def has_front_obstacle(
    scan: LaserScan, stop_distance: float, front_half_angle: float
) -> bool:
    """Check finite, in-range samples within the requested front sector."""

    for index, distance in enumerate(scan.ranges):
        angle = scan.angle_min + index * scan.angle_increment
        if abs(angle) > front_half_angle:
            continue
        if not isfinite(distance):
            continue
        if distance < scan.range_min or distance > scan.range_max:
            continue
        if distance <= stop_distance:
            return True
    return False


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

        safety_blocking = observation.front_obstacle and (
            (
                observation.nav_speed >= self.nav_intent_threshold
                and observation.output_speed <= self.output_stop_threshold
            )
            or observation.estop_active
        )

        if self._state is GateState.NORMAL:
            if safety_blocking:
                self._state = GateState.STOP_WAITING
                self._blocked_since_sec = now
            return self._decision()

        if self._state is GateState.STOP_WAITING:
            if not safety_blocking:
                self._reset_normal()
            elif now - self._blocked_since_sec >= self.hold_time_sec:
                self._state = GateState.DETOUR_ACTIVE
            return self._decision()

        if self._state is GateState.DETOUR_ACTIVE:
            # Once the detour is active, keep publishing the global scan while
            # the obstacle remains, even as Nav2 turns and the safety output
            # resumes motion. Only the obstacle itself ends the detour.
            if not observation.front_obstacle:
                self._state = GateState.CLEARING
                self._clear_started_sec = now
            return self._decision()

        # CLEARING: a renewed block immediately resumes the detour.
        if observation.front_obstacle:
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


class DetourSupervisor(Node):
    """Apply :class:`DetourGate` to simulated navigation and lidar topics."""

    def __init__(self) -> None:
        super().__init__('traymover_detour_supervisor')

        self.declare_parameter('enable_detour', True)
        self.declare_parameter('hold_time_sec', 8.0)
        self.declare_parameter('stop_distance', 0.8)
        self.declare_parameter('front_half_angle', 0.25)
        self.declare_parameter('clear_publish_sec', 1.5)
        self.declare_parameter('nav_intent_threshold', 0.05)
        self.declare_parameter('output_stop_threshold', 0.01)
        self.declare_parameter('tick_hz', 20.0)

        self.enable_detour = bool(self.get_parameter('enable_detour').value)
        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.front_half_angle = float(self.get_parameter('front_half_angle').value)
        self.tick_hz = float(self.get_parameter('tick_hz').value)
        if self.tick_hz <= 0.0:
            self.get_logger().warning('tick_hz must be positive; using 20 Hz')
            self.tick_hz = 20.0

        self._gate = DetourGate(
            hold_time_sec=float(self.get_parameter('hold_time_sec').value),
            clear_publish_sec=float(self.get_parameter('clear_publish_sec').value),
            nav_intent_threshold=float(
                self.get_parameter('nav_intent_threshold').value
            ),
            output_stop_threshold=float(
                self.get_parameter('output_stop_threshold').value
            ),
        )
        self._latest_scan: Optional[LaserScan] = None
        self._latest_nav_cmd = Twist()
        self._latest_output_cmd = Twist()
        self._estop_active = False

        self.scan_subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.nav_subscription = self.create_subscription(
            Twist, '/cmd_vel_nav', self.nav_cmd_callback, 10
        )
        self.output_subscription = self.create_subscription(
            Twist, '/cmd_vel', self.output_cmd_callback, 10
        )
        self.estop_subscription = self.create_subscription(
            Bool, '/traymover_estop/state', self.estop_callback, 10
        )

        self.scan_global_publisher = self.create_publisher(
            LaserScan, '/scan_global', 10
        )
        self.state_publisher = self.create_publisher(
            String, '/traymover_detour/state', 10
        )
        self.active_publisher = self.create_publisher(
            Bool, '/traymover_detour/active', 10
        )
        self.timer = self.create_timer(1.0 / self.tick_hz, self.tick)

    def scan_callback(self, msg: LaserScan) -> None:
        self._latest_scan = msg

    def nav_cmd_callback(self, msg: Twist) -> None:
        self._latest_nav_cmd = msg

    def output_cmd_callback(self, msg: Twist) -> None:
        self._latest_output_cmd = msg

    def estop_callback(self, msg: Bool) -> None:
        self._estop_active = bool(msg.data)

    def tick(self) -> None:
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if not self.enable_detour:
            decision = GateDecision(GateState.NORMAL, False)
        else:
            front_obstacle = self._latest_scan is not None and has_front_obstacle(
                self._latest_scan, self.stop_distance, self.front_half_angle
            )
            observation = SafetyObservation(
                now_sec=now_sec,
                nav_speed=command_speed(self._latest_nav_cmd),
                output_speed=command_speed(self._latest_output_cmd),
                front_obstacle=front_obstacle,
                estop_active=self._estop_active,
            )
            decision = self._gate.update(observation)

        state_msg = String()
        state_msg.data = decision.state.name
        self.state_publisher.publish(state_msg)
        active_msg = Bool()
        active_msg.data = bool(decision.forward_global_scan)
        self.active_publisher.publish(active_msg)
        if decision.forward_global_scan and self._latest_scan is not None:
            self.scan_global_publisher.publish(self._latest_scan)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DetourSupervisor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
