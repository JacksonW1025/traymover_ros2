"""Send one deterministic NavigateToPose goal for the simulation demo."""

from math import cos, pi, sin
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool


def make_goal_pose(
    goal_x: float,
    goal_y: float,
    goal_yaw: float,
    frame_id: str = "map",
    stamp=None,
) -> PoseStamped:
    """Build a planar goal pose with a quaternion derived from ``goal_yaw``."""

    pose = PoseStamped()
    pose.header.frame_id = frame_id
    if stamp is not None:
        pose.header.stamp = stamp
    pose.pose.position.x = float(goal_x)
    pose.pose.position.y = float(goal_y)
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = sin(float(goal_yaw) / 2.0)
    pose.pose.orientation.w = cos(float(goal_yaw) / 2.0)
    return pose


def choose_detour_waypoint(goal_x: float, goal_y: float) -> tuple[float, float]:
    """Choose a temporary side waypoint for the fixed simulation obstacle.

    The side is selected from the requested destination rather than issuing a
    hard-coded velocity maneuver. Nav2 still plans each leg globally.
    """

    # The box spans y=-0.45..0.45.  A 1.2 m center offset leaves physical
    # clearance for the 0.275 m robot half-width and the 0.35 m inflation
    # radius, while reducing the previous box clearance by roughly 40%.
    side_y = -1.2 if float(goal_y) >= 0.0 else 1.2
    # Start the side leg before the box's west edge so the approach and exit
    # diagonals are similar rather than making a sharp corner at its center.
    entry_x = min(max(float(goal_x) - 3.9, 2.6), 3.1)
    return entry_x, side_y


def choose_detour_waypoint_yaw(goal_y: float) -> float:
    """Return the heading used while entering the selected obstacle side."""

    return -pi / 2.0 if float(goal_y) >= 0.0 else pi / 2.0


def choose_detour_exit_waypoint(goal_x: float, goal_y: float) -> tuple[float, float]:
    """Choose a side waypoint past the fixed box before returning to the goal."""

    side_y = -1.2 if float(goal_y) >= 0.0 else 1.2
    # The showcase box occupies x=3.55..4.45. Keep the exit point beyond its
    # east edge and the global inflation radius, but begin the final diagonal
    # early enough for a smooth join.
    exit_x = min(max(float(goal_x) - 1.4, 5.5), 6.2)
    return exit_x, side_y


class DemoGoalSender(Node):
    """Send the demo goal and optionally replace it with a detour leg."""

    def __init__(self) -> None:
        # Keep the pose helper importable for lightweight unit tests even on
        # systems where the optional Nav2 runtime is not installed.
        from nav2_msgs.action import NavigateToPose

        super().__init__("demo_goal_sender")
        self._action_type = NavigateToPose
        self.declare_parameter("goal_x", 7.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("goal_yaw", 0.0)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("send_delay_sec", 2.0)
        self.declare_parameter("auto_send_goal", True)
        self.declare_parameter("enable_detour_waypoint", False)

        self.goal_x = float(self.get_parameter("goal_x").value)
        self.goal_y = float(self.get_parameter("goal_y").value)
        self.goal_yaw = float(self.get_parameter("goal_yaw").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.send_delay_sec = max(0.0, float(self.get_parameter("send_delay_sec").value))
        self.auto_send_goal = bool(self.get_parameter("auto_send_goal").value)
        self.enable_detour_waypoint = bool(
            self.get_parameter("enable_detour_waypoint").value
        )

        self._client = ActionClient(self, self._action_type, "navigate_to_pose")
        self._timer = None
        if self.auto_send_goal:
            self._timer = self.create_timer(self.send_delay_sec or 0.001, self._send_goal)
        self._goal_handle = None
        self._stage = "direct"
        self._detour_started = False
        self._detour_leg_sent = False
        self._detour_exit_sent = False
        self._goal_in_flight = False
        self._cancel_requested = False
        self._route_hold_publisher = None
        if self.enable_detour_waypoint:
            self._route_hold_publisher = self.create_publisher(
                Bool, "/traymover_detour/route_hold", 10
            )
            self._publish_route_hold(False)
            self.create_subscription(
                Bool, "/traymover_detour/active", self._detour_active_callback, 10
            )

    def _detour_active_callback(self, message: Bool) -> None:
        if message.data:
            self._start_detour()

    def _publish_route_hold(self, active: bool) -> None:
        if self._route_hold_publisher is None:
            return
        message = Bool()
        message.data = bool(active)
        self._route_hold_publisher.publish(message)

    def _start_detour(self) -> None:
        if (
            not self._detour_started
            and self._stage == "direct"
            and self._goal_handle is not None
            and self._goal_in_flight
        ):
            self._detour_started = True
            self._publish_route_hold(True)
            self._cancel_requested = True
            self.get_logger().info(
                "Eight-second stop reached; replacing the direct goal with a "
                "global detour leg"
            )
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._detour_cancel_callback)

    def _detour_cancel_callback(self, future) -> None:
        try:
            future.result()
        except Exception as exc:  # noqa: BLE001 - action transport is asynchronous
            self.get_logger().warning("Could not cancel direct goal: %s", exc)
        self._cancel_requested = False

        # ``cancel_goal_async`` only acknowledges the cancel request; the
        # action server still has to publish the direct goal's terminal result.
        # The waypoint is sent from ``_result_callback`` after that result so
        # Nav2 cannot mistake the new leg for the canceled goal.

    def _send_detour_waypoint(self) -> None:
        if self._detour_leg_sent or not rclpy.ok():
            return
        self._detour_leg_sent = True
        waypoint_x, waypoint_y = choose_detour_waypoint(self.goal_x, self.goal_y)
        self._send_navigation_goal(
            "waypoint",
            waypoint_x,
            waypoint_y,
            choose_detour_waypoint_yaw(self.goal_y),
        )

    def _send_detour_exit(self) -> None:
        if self._detour_exit_sent or not rclpy.ok():
            return
        self._detour_exit_sent = True
        exit_x, exit_y = choose_detour_exit_waypoint(self.goal_x, self.goal_y)
        self._send_navigation_goal("exit", exit_x, exit_y, 0.0)

    def _send_goal(self) -> None:
        if self._timer is None:
            return
        self._timer.cancel()
        if not self._client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warning("navigate_to_pose action server is not ready; retrying")
            self._timer = self.create_timer(1.0, self._send_goal)
            return

        self._send_navigation_goal("direct", self.goal_x, self.goal_y, self.goal_yaw)

    def _send_navigation_goal(
        self, stage: str, goal_x: float, goal_y: float, goal_yaw: float
    ) -> None:
        goal = self._action_type.Goal()
        goal.pose = make_goal_pose(
            goal_x,
            goal_y,
            goal_yaw,
            frame_id=self.frame_id,
            stamp=self.get_clock().now().to_msg(),
        )
        self._stage = stage
        self._goal_in_flight = True
        future = self._client.send_goal_async(goal)
        future.add_done_callback(
            lambda result, requested_stage=stage: self._goal_response_callback(
                result, requested_stage
            )
        )

    def _goal_response_callback(self, future, requested_stage: str) -> None:
        self._goal_handle = future.result()
        if not self._goal_handle or not self._goal_handle.accepted:
            self.get_logger().warning("NavigateToPose goal was rejected; retrying")
            # The action server is advertised while bt_navigator is still
            # inactive during lifecycle bringup.  Keep retrying until the
            # managed Nav2 stack accepts the goal instead of ending the demo
            # before the first route is generated.
            if self.auto_send_goal and rclpy.ok() and requested_stage == "direct":
                self._timer = self.create_timer(1.0, self._send_goal)
            elif requested_stage == "waypoint" and rclpy.ok():
                self._send_navigation_goal(
                    "waypoint",
                    *choose_detour_waypoint(self.goal_x, self.goal_y),
                    choose_detour_waypoint_yaw(self.goal_y),
                )
            elif requested_stage == "exit" and rclpy.ok():
                self._send_navigation_goal(
                    "exit", *choose_detour_exit_waypoint(self.goal_x, self.goal_y), 0.0
                )
            elif rclpy.ok():
                rclpy.shutdown()
            return
        self.get_logger().info("NavigateToPose goal accepted")
        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result, accepted_stage=requested_stage: self._result_callback(
                result, accepted_stage
            )
        )

    def _result_callback(self, future, completed_stage: str) -> None:
        try:
            result = future.result().result
        except Exception as exc:  # noqa: BLE001 - action futures may report transport errors
            self.get_logger().error("NavigateToPose failed: %s", exc)
        else:
            self.get_logger().info(
                f"NavigateToPose ({completed_stage}) completed with result code {result}"
            )
            self._goal_in_flight = False
            if completed_stage == "direct" and self._detour_started:
                self._send_detour_waypoint()
                return
            if completed_stage == "waypoint" and rclpy.ok():
                self.get_logger().info("Detour entry reached; crossing the obstacle side")
                self._send_detour_exit()
            elif completed_stage == "exit" and rclpy.ok():
                self.get_logger().info("Detour exit reached; continuing to the final goal")
                self._send_navigation_goal("final", self.goal_x, self.goal_y, self.goal_yaw)
            elif rclpy.ok():
                # This executable sends one goal (or one detour leg followed
                # by the final goal) and then lets launch tear down cleanly.
                self._publish_route_hold(False)
                rclpy.shutdown()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = DemoGoalSender()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        # Result/rejection callbacks may already have stopped the context.
        # Keep normal shutdown idempotent so the one-shot sender does not
        # raise when spin returns after a callback-driven shutdown.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
