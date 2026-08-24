"""Send one deterministic NavigateToPose goal for the simulation demo."""

from math import cos, sin
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


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


class DemoGoalSender(Node):
    """Wait for Nav2 and send one goal after a configurable startup delay."""

    def __init__(self) -> None:
        super().__init__("demo_goal_sender")
        self.declare_parameter("goal_x", 7.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("goal_yaw", 0.0)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("send_delay_sec", 2.0)

        self.goal_x = float(self.get_parameter("goal_x").value)
        self.goal_y = float(self.get_parameter("goal_y").value)
        self.goal_yaw = float(self.get_parameter("goal_yaw").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.send_delay_sec = max(0.0, float(self.get_parameter("send_delay_sec").value))

        self._client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._timer = self.create_timer(self.send_delay_sec or 0.001, self._send_goal)
        self._goal_handle = None

    def _send_goal(self) -> None:
        self._timer.cancel()
        if not self._client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warning("navigate_to_pose action server is not ready; retrying")
            self._timer = self.create_timer(1.0, self._send_goal)
            return

        goal = NavigateToPose.Goal()
        goal.pose = make_goal_pose(
            self.goal_x,
            self.goal_y,
            self.goal_yaw,
            frame_id=self.frame_id,
            stamp=self.get_clock().now().to_msg(),
        )
        future = self._client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future) -> None:
        self._goal_handle = future.result()
        if not self._goal_handle or not self._goal_handle.accepted:
            self.get_logger().error("NavigateToPose goal was rejected")
            return
        self.get_logger().info("NavigateToPose goal accepted")
        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future) -> None:
        try:
            result = future.result().result
        except Exception as exc:  # noqa: BLE001 - action futures may report transport errors
            self.get_logger().error("NavigateToPose failed: %s", exc)
        else:
            self.get_logger().info("NavigateToPose completed with result code %s", result)
        finally:
            # This executable sends one goal and should not keep the launch
            # alive after Nav2 reports success, rejection, or transport error.
            if rclpy.ok():
                rclpy.shutdown()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = DemoGoalSender()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
