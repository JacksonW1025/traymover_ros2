from math import isclose
from pathlib import Path
import sys


sys.path.insert(0, str(Path(__file__).parents[1]))

from traymover_robot_sim.demo_goal_sender import make_goal_pose


def test_make_goal_pose_uses_map_frame_and_yaw_quaternion():
    pose = make_goal_pose(7.0, 0.0, 0.0, "map")
    assert pose.header.frame_id == "map"
    assert isclose(pose.pose.position.x, 7.0)
    assert isclose(pose.pose.orientation.w, 1.0)
    assert isclose(pose.pose.orientation.z, 0.0)


def test_make_goal_pose_builds_planar_yaw_quaternion():
    pose = make_goal_pose(1.5, -2.0, 1.5707963267948966, "odom")
    assert pose.header.frame_id == "odom"
    assert isclose(pose.pose.position.x, 1.5)
    assert isclose(pose.pose.position.y, -2.0)
    assert isclose(pose.pose.orientation.x, 0.0)
    assert isclose(pose.pose.orientation.y, 0.0)
    assert isclose(pose.pose.orientation.z, 2**-0.5, rel_tol=1e-9)
    assert isclose(pose.pose.orientation.w, 2**-0.5, rel_tol=1e-9)


def test_sender_retains_action_type_for_timer_goal_construction():
    source = (Path(__file__).parents[1] / "traymover_robot_sim" / "demo_goal_sender.py").read_text()
    assert "self._action_type = NavigateToPose" in source
    assert "goal = self._action_type.Goal()" in source


def test_sender_retries_a_goal_rejected_before_nav2_is_active():
    source = (Path(__file__).parents[1] / "traymover_robot_sim" / "demo_goal_sender.py").read_text()
    assert "self._timer = self.create_timer(1.0, self._send_goal)" in source
    assert "NavigateToPose goal was rejected; retrying" in source


def test_sender_formats_result_for_jazzy_rclpy_logger():
    source = (Path(__file__).parents[1] / "traymover_robot_sim" / "demo_goal_sender.py").read_text()
    assert 'self.get_logger().info(f"NavigateToPose completed with result code {result}")' in source
