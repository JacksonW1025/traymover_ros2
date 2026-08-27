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


def limit_detour_angular_speed(twist: Twist, max_abs_angular_speed: float) -> Twist:
    """Copy a command while bounding yaw rate for the prototype detour.

    RPP does not expose a maximum angular velocity parameter. A short, sharp
    path corner can therefore produce a large yaw-rate command even after its
    linear speed has been regulated. The detour mux applies this bound only
    while the sender holds the detour route, leaving the normal navigation and
    safety command paths unchanged.
    """

    if not isfinite(max_abs_angular_speed) or max_abs_angular_speed <= 0.0:
        raise ValueError("max_abs_angular_speed must be finite and positive")
    limited = Twist()
    limited.linear.x = twist.linear.x
    limited.linear.y = twist.linear.y
    limited.linear.z = twist.linear.z
    limited.angular.x = twist.angular.x
    limited.angular.y = twist.angular.y
    limited.angular.z = max(
        -max_abs_angular_speed,
        min(max_abs_angular_speed, twist.angular.z),
    )
    return limited


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


def set_global_scan_frame(scan: LaserScan, frame_id: str) -> LaserScan:
    """Set the frame used by the global replan observation and return it."""

    scan.header.frame_id = frame_id
    return scan


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


def select_output_command(
    decision: GateDecision,
    nav_cmd: Twist,
    safety_cmd: Twist,
    estop_active: bool,
    front_obstacle: bool = False,
    route_hold: bool = False,
) -> Twist:
    """Select the simulated drivetrain command for the current gate state.

    Before the sustained-stop threshold, the collision monitor owns the
    output and can enforce the normal safety stop. Once detour is active, the
    planner must be allowed to rotate and drive around the obstacle; otherwise
    a front stop polygon also suppresses the turning command and the robot is
    deadlocked. A hardware-style e-stop remains an unconditional zero command.
    """

    if estop_active:
        return Twist()
    if (
        front_obstacle
        and decision.state in (GateState.NORMAL, GateState.STOP_WAITING)
        and not route_hold
    ):
        # Do not depend on a collision-monitor polygon's exact boundary for
        # the first stop. The raw lidar gate can stop the prototype as soon
        # as the obstacle enters its configured detection sector. Once the
        # sender has taken route_hold, however, this same obstacle is the one
        # being circumnavigated; stopping again would deadlock the detour.
        return Twist()
    if decision.forward_global_scan or route_hold:
        return nav_cmd
    return safety_cmd


class DetourGate:
    """Gate global-scan forwarding after a sustained local stop condition."""

    def __init__(
        self,
        hold_time_sec: float = 8.0,
        nav_intent_threshold: float = 0.05,
        output_stop_threshold: float = 0.01,
        clear_publish_sec: float = 1.5,
    ) -> None:
        if not isfinite(hold_time_sec) or hold_time_sec <= 0.0:
            raise ValueError("hold_time_sec must be finite and positive")
        if not isfinite(clear_publish_sec) or clear_publish_sec <= 0.0:
            raise ValueError("clear_publish_sec must be finite and positive")
        if not isfinite(nav_intent_threshold) or nav_intent_threshold < 0.0:
            raise ValueError("nav_intent_threshold must be finite and nonnegative")
        if not isfinite(output_stop_threshold) or output_stop_threshold < 0.0:
            raise ValueError(
                "output_stop_threshold must be finite and nonnegative"
            )
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

        # The simulation gate is driven by the raw front-sector observation.
        # CollisionMonitor may be slowing (rather than fully stopping) while
        # the box enters the near field, but it is still a blocking obstacle
        # for the eight-second detour timer.
        safety_blocking = observation.front_obstacle and (
            observation.nav_speed >= self.nav_intent_threshold
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
        self.declare_parameter('front_half_angle', 1.2)
        self.declare_parameter('clear_publish_sec', 1.5)
        self.declare_parameter('nav_intent_threshold', 0.05)
        self.declare_parameter('output_stop_threshold', 0.01)
        self.declare_parameter('detour_max_angular_speed', 0.8)
        self.declare_parameter('tick_hz', 20.0)
        self.declare_parameter('global_scan_frame', 'base_link')

        self.enable_detour = bool(self.get_parameter('enable_detour').value)
        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.front_half_angle = float(self.get_parameter('front_half_angle').value)
        self.global_scan_frame = str(self.get_parameter('global_scan_frame').value)
        self.detour_max_angular_speed = float(
            self.get_parameter('detour_max_angular_speed').value
        )
        if (
            not isfinite(self.detour_max_angular_speed)
            or self.detour_max_angular_speed <= 0.0
        ):
            self.get_logger().warning(
                'detour_max_angular_speed must be positive; using 0.8 rad/s'
            )
            self.detour_max_angular_speed = 0.8
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
        self._latest_safety_cmd = Twist()
        self._estop_active = False
        self._route_hold = False

        self.scan_subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.nav_subscription = self.create_subscription(
            Twist, '/cmd_vel_nav', self.nav_cmd_callback, 10
        )
        self.safety_subscription = self.create_subscription(
            Twist, '/cmd_vel_safety', self.safety_cmd_callback, 10
        )
        self.command_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.estop_subscription = self.create_subscription(
            Bool, '/traymover_estop/state', self.estop_callback, 10
        )
        self.route_hold_subscription = self.create_subscription(
            Bool, '/traymover_detour/route_hold', self.route_hold_callback, 10
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

    def safety_cmd_callback(self, msg: Twist) -> None:
        self._latest_safety_cmd = msg

    def estop_callback(self, msg: Bool) -> None:
        self._estop_active = bool(msg.data)

    def route_hold_callback(self, msg: Bool) -> None:
        self._route_hold = bool(msg.data)

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
                output_speed=command_speed(self._latest_safety_cmd),
                front_obstacle=front_obstacle,
                estop_active=self._estop_active,
            )
            decision = self._gate.update(observation)

        forward_nav = decision.forward_global_scan or self._route_hold
        state_msg = String()
        state_msg.data = decision.state.name
        self.state_publisher.publish(state_msg)
        active_msg = Bool()
        active_msg.data = bool(forward_nav)
        self.active_publisher.publish(active_msg)
        output_command = select_output_command(
            decision,
            self._latest_nav_cmd,
            self._latest_safety_cmd,
            self._estop_active,
            front_obstacle,
            self._route_hold,
        )
        if self._route_hold:
            output_command = limit_detour_angular_speed(
                output_command, self.detour_max_angular_speed
            )
        self.command_publisher.publish(output_command)
        if forward_nav and self._latest_scan is not None:
            # Keep the frame configurable for simulator backends that do not
            # provide a timestamped static laser transform. The default laser
            # frame preserves the sensor origin and measured ranges.
            self.scan_global_publisher.publish(
                set_global_scan_frame(self._latest_scan, self.global_scan_frame)
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DetourSupervisor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
