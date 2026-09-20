#!/usr/bin/env python3
"""Real rclpy action integration tests, isolated from the robot's ROS domain."""

import importlib.util
import os
from pathlib import Path
import sys
import time
from unittest.mock import Mock

import pytest

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.task import Future


MODULE_PATH = Path(__file__).resolve().parents[1] / 'scripts' / 'detour_goal_proxy.py'
SPEC = importlib.util.spec_from_file_location('detour_goal_proxy', MODULE_PATH)
proxy_module = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = proxy_module
SPEC.loader.exec_module(proxy_module)


class Harness:
    def __init__(self, *, create_backend=True, **proxy_options):
        self.context = Context()
        self.context.init(args=[], domain_id=160 + os.getpid() % 60)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.node = Node('detour_proxy_test', context=self.context)
        self.backend_node = Node('fake_nav2_backend', context=self.context)
        self.client_node = Node('fake_navigation_user', context=self.context)
        for node in (self.node, self.backend_node, self.client_node):
            self.executor.add_node(node)
        self.paused = False
        self.changes = []
        self.backend_goals = []
        self.backend_completions = {}
        self.backend_cancels = []
        self.goal_requests = []
        self.acceptance_gate = None
        self.reject_goals = False
        self.proxy = proxy_module.GoalProxy(
            self.node,
            lambda: self.paused,
            lambda: self.changes.append(time.monotonic()),
            **proxy_options,
        )
        self.backend = None
        self.through_backend = None
        if create_backend:
            self.backend = ActionServer(
                self.backend_node, NavigateToPose,
                '/navigate_to_pose_detour_backend',
                execute_callback=self.execute,
                goal_callback=self.goal,
                handle_accepted_callback=self.accepted,
                cancel_callback=self.cancel,
                callback_group=ReentrantCallbackGroup(),
            )
            self.through_backend = ActionServer(
                self.backend_node, NavigateThroughPoses,
                '/navigate_through_poses_detour_backend',
                execute_callback=self.execute,
                goal_callback=self.goal,
                handle_accepted_callback=self.accepted,
                cancel_callback=self.cancel,
                callback_group=ReentrantCallbackGroup(),
            )
        self.client = ActionClient(self.client_node, NavigateToPose, '/navigate_to_pose')
        self.through_client = ActionClient(
            self.client_node, NavigateThroughPoses, '/navigate_through_poses')
        self.rviz_goals = self.client_node.create_publisher(PoseStamped, '/goal_pose', 10)
        self.until(self.client.server_is_ready)
        if create_backend:
            self.until(self.proxy._client.server_is_ready)

    async def goal(self, request):
        self.goal_requests.append(request)
        if self.acceptance_gate is not None:
            await self.acceptance_gate
        return GoalResponse.REJECT if self.reject_goals else GoalResponse.ACCEPT

    def accepted(self, handle):
        self.backend_goals.append(handle)
        self.backend_completions[bytes(handle.goal_id.uuid)] = Future()
        handle.execute()

    async def execute(self, handle):
        return await self.backend_completions[bytes(handle.goal_id.uuid)]

    def cancel(self, handle):
        self.backend_cancels.append(handle)
        return CancelResponse.ACCEPT

    def finish_backend(self, index, status):
        handle = self.backend_goals[index]
        if status == GoalStatus.STATUS_SUCCEEDED:
            handle.succeed()
        elif status == GoalStatus.STATUS_CANCELED:
            handle.canceled()
        else:
            handle.abort()
        action_type = NavigateThroughPoses if hasattr(handle.request, 'poses') else NavigateToPose
        self.backend_completions[bytes(handle.goal_id.uuid)].set_result(action_type.Result())

    def until(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.01)
        assert predicate(), 'ROS action operation did not finish within test timeout'

    def pump(self, duration=0.10):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.01)

    def send(self, x=1.0):
        request = NavigateToPose.Goal()
        request.pose.header.frame_id = 'map'
        request.pose.pose.position.x = x
        request.pose.pose.orientation.w = 1.0
        request.behavior_tree = 'original_tree.xml'
        future = self.client.send_goal_async(request)
        self.until(future.done)
        return future.result()

    def start(self, x=1.0):
        handle = self.send(x)
        assert handle.accepted
        self.until(lambda: self.proxy.motion_authorized)
        return handle

    def send_through(self, positions=(1.0, 2.0, 3.0)):
        self.until(self.through_client.server_is_ready)
        request = NavigateThroughPoses.Goal()
        for x in positions:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.orientation.w = 1.0
            request.poses.append(pose)
        request.behavior_tree = 'through_tree.xml'
        future = self.through_client.send_goal_async(request)
        self.until(future.done)
        return future.result()

    def result(self, handle):
        future = handle.get_result_async()
        self.until(future.done)
        return future.result()

    def close(self):
        self.proxy.destroy()
        self.client.destroy()
        self.through_client.destroy()
        if self.backend is not None:
            self.backend.destroy()
            self.through_backend.destroy()
        self.executor.shutdown(timeout_sec=0.0)
        for node in (self.node, self.backend_node, self.client_node):
            node.destroy_node()
        self.context.shutdown()


@pytest.fixture
def harness(monkeypatch):
    # This context cannot discover the normal robot domain, and DDS remains local.
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    instances = []

    def make(**kwargs):
        instance = Harness(**kwargs)
        instances.append(instance)
        return instance

    yield make
    for instance in reversed(instances):
        instance.close()


@pytest.mark.parametrize('status', [GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED])
def test_normal_backend_terminal_state_is_forwarded(harness, status):
    ros = harness()
    handle = ros.start(4.0)
    assert ros.proxy.saved_request.pose.pose.position.x == 4.0
    assert ros.proxy.saved_request.behavior_tree == 'original_tree.xml'
    saved = ros.proxy.saved_request
    saved.pose.pose.position.x = 9.0
    assert ros.proxy.saved_request.pose.pose.position.x == 4.0
    ros.finish_backend(0, status)
    assert ros.result(handle).status == status
    assert not ros.proxy.has_active_goal
    assert not ros.proxy.motion_authorized
    ros.proxy.notify_handback()
    ros.pump()
    assert len(ros.backend_goals) == 1


def test_paused_abort_preserves_original_goal_and_retries_once_per_handback(harness):
    ros = harness()
    handle = ros.start(7.0)
    external_result = handle.get_result_async()
    for index in range(2):
        ros.paused = True
        ros.finish_backend(index, GoalStatus.STATUS_ABORTED)
        ros.until(lambda: ros.proxy._active.held_abort)
        assert ros.proxy.has_active_goal
        assert ros.proxy.motion_authorized  # Supervisor can complete its safe detour.
        assert not external_result.done()
        ros.paused = False
        assert not ros.proxy.motion_authorized  # No stale nav Twist before reacceptance.
        ros.proxy.notify_handback()
        assert not ros.proxy.motion_authorized
        ros.until(lambda: ros.proxy.motion_authorized)
        ros.proxy.notify_handback()
        ros.proxy.notify_handback()
        ros.pump()
        assert len(ros.backend_goals) == index + 2
        assert ros.backend_goals[-1].request.pose.pose.position.x == 7.0
        assert ros.backend_goals[-1].request.behavior_tree == 'original_tree.xml'
    ros.finish_backend(2, GoalStatus.STATUS_SUCCEEDED)
    ros.until(external_result.done)
    assert external_result.result().status == GoalStatus.STATUS_SUCCEEDED


def test_backend_success_during_detour_revokes_motion_and_clears_goal(harness):
    ros = harness()
    handle = ros.start()
    ros.paused = True
    previous_changes = len(ros.changes)
    ros.finish_backend(0, GoalStatus.STATUS_SUCCEEDED)
    assert ros.result(handle).status == GoalStatus.STATUS_SUCCEEDED
    assert not ros.proxy.motion_authorized
    assert not ros.proxy.has_active_goal
    assert len(ros.changes) == previous_changes + 1


def test_handback_can_retry_while_paused_but_cannot_loop_on_backend_failure(harness):
    ros = harness()
    handle = ros.start()
    ros.paused = True
    ros.finish_backend(0, GoalStatus.STATUS_ABORTED)
    ros.until(lambda: ros.proxy._active.held_abort)
    # RECOVER_TO_NAV remains paused until backend acceptance permits NORMAL.
    ros.proxy.notify_handback()
    assert not ros.proxy.motion_authorized
    ros.until(lambda: ros.proxy.motion_authorized)
    ros.finish_backend(1, GoalStatus.STATUS_ABORTED)
    assert ros.result(handle).status == GoalStatus.STATUS_ABORTED
    ros.proxy.notify_handback()
    ros.proxy.notify_handback()
    ros.pump()
    assert len(ros.backend_goals) == 2
    assert not ros.proxy.motion_authorized


def test_user_cancel_blocks_motion_until_backend_terminal_and_never_reissues(harness):
    ros = harness()
    handle = ros.start()
    cancel = handle.cancel_goal_async()
    ros.until(cancel.done)
    assert cancel.result().goals_canceling
    assert not ros.proxy.motion_authorized
    ros.until(lambda: bool(ros.backend_cancels))
    assert not ros.proxy.motion_authorized
    assert not ros.send(2.0).accepted  # Cancellation acknowledgement is not terminal.
    ros.finish_backend(0, GoalStatus.STATUS_CANCELED)
    assert ros.result(handle).status == GoalStatus.STATUS_CANCELED
    ros.proxy.notify_handback()
    ros.pump()
    assert len(ros.backend_goals) == 1
    assert not ros.proxy.has_active_goal


def test_cancel_preserved_goal_prevents_handback_retry(harness):
    ros = harness()
    handle = ros.start()
    ros.paused = True
    ros.finish_backend(0, GoalStatus.STATUS_ABORTED)
    ros.until(lambda: ros.proxy._active.held_abort)
    cancel = handle.cancel_goal_async()
    ros.until(cancel.done)
    ros.paused = False
    ros.proxy.notify_handback()
    assert ros.result(handle).status == GoalStatus.STATUS_CANCELED
    ros.pump()
    assert len(ros.backend_goals) == 1


def test_preemption_waits_for_old_backend_and_old_callbacks_cannot_clear_new_goal(harness):
    ros = harness()
    first = ros.start(1.0)
    second = ros.send(2.0)
    assert second.accepted
    assert not ros.proxy.motion_authorized
    assert ros.proxy.saved_request.pose.pose.position.x == 2.0
    assert ros.result(first).status == GoalStatus.STATUS_ABORTED
    ros.until(lambda: bool(ros.backend_cancels))
    assert len(ros.backend_goals) == 1
    assert not ros.send(3.0).accepted
    ros.finish_backend(0, GoalStatus.STATUS_CANCELED)
    ros.until(lambda: ros.proxy.motion_authorized)
    assert ros.proxy.saved_request.pose.pose.position.x == 2.0
    assert len(ros.backend_goals) == 2
    ros.finish_backend(1, GoalStatus.STATUS_SUCCEEDED)
    assert ros.result(second).status == GoalStatus.STATUS_SUCCEEDED


def test_late_acceptance_after_send_timeout_is_canceled_and_cannot_authorize_motion(harness):
    ros = harness(send_timeout_sec=0.15)
    ros.acceptance_gate = Future()
    handle = ros.send()
    ros.until(lambda: bool(ros.goal_requests))
    assert ros.result(handle).status == GoalStatus.STATUS_ABORTED
    assert not ros.proxy.motion_authorized
    assert not ros.send(2.0).accepted
    ros.acceptance_gate.set_result(None)
    ros.until(lambda: bool(ros.backend_cancels))
    assert not ros.proxy.motion_authorized
    ros.finish_backend(0, GoalStatus.STATUS_CANCELED)
    ros.until(lambda: not ros.proxy._has_unresolved_cancel())
    ros.acceptance_gate = None
    fresh = ros.start(3.0)
    ros.finish_backend(1, GoalStatus.STATUS_SUCCEEDED)
    assert ros.result(fresh).status == GoalStatus.STATUS_SUCCEEDED


def test_preempting_an_unacknowledged_goal_cancels_its_late_acceptance(harness):
    ros = harness()
    gate = Future()
    ros.acceptance_gate = gate
    first = ros.send(1.0)
    ros.until(lambda: bool(ros.goal_requests))
    second = ros.send(2.0)
    assert second.accepted
    assert ros.proxy.saved_request.pose.pose.position.x == 2.0
    assert not ros.proxy.motion_authorized
    assert ros.result(first).status == GoalStatus.STATUS_ABORTED
    ros.acceptance_gate = None
    gate.set_result(None)
    ros.until(lambda: bool(ros.backend_cancels))
    assert len(ros.backend_goals) == 1
    assert not ros.proxy.motion_authorized
    ros.finish_backend(0, GoalStatus.STATUS_CANCELED)
    ros.until(lambda: ros.proxy.motion_authorized)
    assert ros.proxy.saved_request.pose.pose.position.x == 2.0
    ros.finish_backend(1, GoalStatus.STATUS_SUCCEEDED)
    assert ros.result(second).status == GoalStatus.STATUS_SUCCEEDED


def test_cancel_timeout_finishes_external_goal_but_keeps_safety_interlock(harness):
    ros = harness(cancel_timeout_sec=0.15)
    handle = ros.start()
    cancel = handle.cancel_goal_async()
    ros.until(cancel.done)
    assert ros.result(handle).status == GoalStatus.STATUS_CANCELED
    assert not ros.proxy.motion_authorized
    assert not ros.send(2.0).accepted
    ros.finish_backend(0, GoalStatus.STATUS_CANCELED)
    ros.until(lambda: not ros.proxy._has_unresolved_cancel())
    fresh = ros.start(2.0)
    ros.finish_backend(1, GoalStatus.STATUS_SUCCEEDED)
    assert ros.result(fresh).status == GoalStatus.STATUS_SUCCEEDED


def test_cancel_before_backend_acceptance_cancels_late_goal_without_motion(harness):
    ros = harness()
    ros.acceptance_gate = Future()
    handle = ros.send()
    ros.until(lambda: bool(ros.goal_requests))
    cancel = handle.cancel_goal_async()
    ros.until(cancel.done)
    assert not ros.proxy.motion_authorized
    ros.acceptance_gate.set_result(None)
    ros.until(lambda: bool(ros.backend_cancels))
    assert not ros.proxy.motion_authorized
    ros.finish_backend(0, GoalStatus.STATUS_CANCELED)
    assert ros.result(handle).status == GoalStatus.STATUS_CANCELED
    ros.proxy.notify_handback()
    ros.pump()
    assert len(ros.backend_goals) == 1


def test_unavailable_backend_aborts_with_bounded_wait(harness):
    ros = harness(create_backend=False, server_timeout_sec=0.15)
    handle = ros.send()
    assert handle.accepted
    assert ros.result(handle).status == GoalStatus.STATUS_ABORTED
    assert not ros.proxy.has_active_goal
    assert not ros.proxy.motion_authorized


def test_backend_timeout_remains_bounded_when_ros_time_is_paused(harness):
    ros = harness(create_backend=False, server_timeout_sec=0.15)
    ros.node.set_parameters([Parameter('use_sim_time', value=True)])
    assert ros.node.get_clock().now().nanoseconds == 0
    handle = ros.send()
    assert ros.result(handle).status == GoalStatus.STATUS_ABORTED
    assert not ros.proxy.motion_authorized


def test_backend_rejection_never_authorizes_motion(harness, monkeypatch):
    ros = harness()
    logger = Mock()
    monkeypatch.setattr(ros.node, 'get_logger', lambda: logger)
    ros.reject_goals = True
    handle = ros.send()
    assert ros.result(handle).status == GoalStatus.STATUS_ABORTED
    assert not ros.proxy.motion_authorized
    assert not ros.backend_goals
    logger.error.assert_called_with('[detour] Backend rejected navigation goal; goal aborted')


def test_through_poses_preserves_remaining_route_and_behavior_tree_after_detour(harness):
    ros = harness()
    handle = ros.send_through()
    assert handle.accepted
    ros.until(lambda: ros.proxy.motion_authorized)
    feedback = NavigateThroughPoses.Feedback(number_of_poses_remaining=2)
    ros.backend_goals[0].publish_feedback(feedback)
    ros.until(lambda: ros.proxy._active.completed_poses == 1)
    ros.paused = True
    ros.finish_backend(0, GoalStatus.STATUS_ABORTED)
    ros.until(lambda: ros.proxy._active.held_abort)
    assert len(ros.proxy.saved_request.poses) == 3
    ros.proxy.notify_handback()
    ros.until(lambda: ros.proxy.motion_authorized)
    assert [pose.pose.position.x for pose in ros.backend_goals[1].request.poses] == [2.0, 3.0]
    assert ros.backend_goals[1].request.behavior_tree == 'through_tree.xml'
    # Progress is relative to each backend attempt, not the original route.
    ros.backend_goals[1].publish_feedback(
        NavigateThroughPoses.Feedback(number_of_poses_remaining=1))
    ros.until(lambda: ros.proxy._active.completed_poses == 2)
    ros.paused = False
    ros.proxy.tick()
    ros.paused = True
    ros.finish_backend(1, GoalStatus.STATUS_ABORTED)
    ros.until(lambda: ros.proxy._active.held_abort)
    ros.proxy.notify_handback()
    ros.until(lambda: ros.proxy.motion_authorized)
    assert [pose.pose.position.x for pose in ros.backend_goals[2].request.poses] == [3.0]
    ros.finish_backend(2, GoalStatus.STATUS_SUCCEEDED)
    assert ros.result(handle).status == GoalStatus.STATUS_SUCCEEDED
    assert not ros.proxy.motion_authorized


def test_through_poses_cancel_blocks_motion_and_cannot_resume(harness):
    ros = harness()
    handle = ros.send_through()
    ros.until(lambda: ros.proxy.motion_authorized)
    cancel = handle.cancel_goal_async()
    ros.until(cancel.done)
    assert not ros.proxy.motion_authorized
    assert not ros.send().accepted
    assert not ros.send_through().accepted
    ros.until(lambda: bool(ros.backend_cancels))
    ros.finish_backend(0, GoalStatus.STATUS_CANCELED)
    assert ros.result(handle).status == GoalStatus.STATUS_CANCELED
    ros.proxy.notify_handback()
    ros.pump()
    assert len(ros.backend_goals) == 1


@pytest.mark.parametrize('remaining, expected', [
    (None, [1.0, 2.0, 3.0]), (0, [3.0]), (-1, [1.0, 2.0, 3.0]),
    (4, [1.0, 2.0, 3.0]),
])
def test_through_poses_retry_requires_valid_progress_and_keeps_final_pose(
        harness, remaining, expected):
    ros = harness()
    handle = ros.send_through()
    ros.until(lambda: ros.proxy.motion_authorized)
    if remaining is not None:
        ros.backend_goals[0].publish_feedback(
            NavigateThroughPoses.Feedback(number_of_poses_remaining=remaining))
        ros.pump()
    ros.paused = True
    ros.finish_backend(0, GoalStatus.STATUS_ABORTED)
    ros.until(lambda: ros.proxy._active.held_abort)
    ros.proxy.notify_handback()
    ros.until(lambda: ros.proxy.motion_authorized)
    assert [pose.pose.position.x for pose in ros.backend_goals[1].request.poses] == expected
    ros.finish_backend(1, GoalStatus.STATUS_SUCCEEDED)
    assert ros.result(handle).status == GoalStatus.STATUS_SUCCEEDED


@pytest.mark.parametrize('through_first', [True, False])
def test_cross_type_preemption_waits_for_previous_backend_terminal(harness, through_first):
    ros = harness()
    first = ros.send_through() if through_first else ros.send()
    ros.until(lambda: ros.proxy.motion_authorized)
    second = ros.send() if through_first else ros.send_through()
    assert second.accepted
    assert not ros.proxy.motion_authorized
    assert ros.result(first).status == GoalStatus.STATUS_ABORTED
    ros.until(lambda: bool(ros.backend_cancels))
    assert len(ros.backend_goals) == 1
    assert not ros.send().accepted
    assert not ros.send_through().accepted
    ros.finish_backend(0, GoalStatus.STATUS_CANCELED)
    ros.until(lambda: ros.proxy.motion_authorized)
    assert len(ros.backend_goals) == 2
    assert hasattr(ros.backend_goals[1].request, 'poses') is not through_first
    ros.finish_backend(1, GoalStatus.STATUS_SUCCEEDED)
    assert ros.result(second).status == GoalStatus.STATUS_SUCCEEDED


def test_rviz_goal_pose_topic_uses_public_proxy_and_shared_preemption(harness):
    ros = harness()
    route = ros.send_through()
    ros.until(lambda: ros.proxy.motion_authorized)
    ros.until(lambda: ros.rviz_goals.get_subscription_count() > 0)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = 8.0
    pose.pose.orientation.w = 1.0
    ros.rviz_goals.publish(pose)
    ros.until(lambda: bool(ros.backend_cancels))
    assert not ros.proxy.motion_authorized
    assert ros.result(route).status == GoalStatus.STATUS_ABORTED
    assert len(ros.backend_goals) == 1
    ros.finish_backend(0, GoalStatus.STATUS_CANCELED)
    ros.until(lambda: ros.proxy.motion_authorized)
    assert len(ros.backend_goals) == 2
    assert ros.backend_goals[1].request.pose == pose
    assert ros.backend_goals[1].request.behavior_tree == ''
    ros.finish_backend(1, GoalStatus.STATUS_SUCCEEDED)
    ros.until(lambda: not ros.proxy.has_active_goal)


@pytest.mark.parametrize('status, outcome', [
    (GoalStatus.STATUS_SUCCEEDED, 'succeeded'),
    (GoalStatus.STATUS_ABORTED, 'aborted'),
])
def test_rviz_topic_goal_reports_acceptance_and_terminal_result(
        harness, monkeypatch, status, outcome):
    ros = harness()
    logger = Mock()
    monkeypatch.setattr(ros.node, 'get_logger', lambda: logger)
    ros.until(lambda: ros.rviz_goals.get_subscription_count() > 0)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.orientation.w = 1.0
    ros.rviz_goals.publish(pose)
    ros.until(lambda: ros.proxy.motion_authorized)
    ros.until(lambda: any(
        'RViz goal accepted' in call.args[0] for call in logger.info.call_args_list))
    ros.finish_backend(0, status)
    ros.until(lambda: any(
        f'RViz navigation {outcome}' in call.args[0] for call in logger.method_calls))
    assert not ros.proxy.has_active_goal
    assert not ros.proxy.motion_authorized


def test_force_handback_waits_for_terminal_then_replans_original_goal(harness):
    ros = harness()
    handle = ros.start(7.0)
    result = handle.get_result_async()
    ros.paused = True
    ros.proxy._active.request.pose.header.stamp.sec = 123
    ros.proxy.notify_handback(force_replan=True)
    ros.proxy.notify_handback(force_replan=True)
    assert not ros.proxy.motion_authorized
    ros.until(lambda: len(ros.backend_cancels) == 1)
    ros.pump()
    assert len(ros.backend_goals) == 1
    assert not result.done()
    ros.finish_backend(0, GoalStatus.STATUS_CANCELED)
    ros.until(lambda: ros.proxy.motion_authorized)
    ros.proxy.notify_handback(force_replan=True)
    ros.pump()
    assert len(ros.backend_goals) == 2
    request = ros.backend_goals[-1].request
    assert request.pose.pose.position.x == 7.0
    assert request.pose.header.stamp.sec == 0
    assert request.behavior_tree == 'original_tree.xml'
    assert not result.done()
    ros.finish_backend(1, GoalStatus.STATUS_SUCCEEDED)
    assert ros.result(handle).status == GoalStatus.STATUS_SUCCEEDED


def test_external_cancel_during_force_handback_prevents_restart(harness):
    ros = harness()
    handle = ros.start()
    ros.proxy.notify_handback(force_replan=True)
    cancel = handle.cancel_goal_async()
    ros.until(cancel.done)
    ros.until(lambda: bool(ros.backend_cancels))
    ros.finish_backend(0, GoalStatus.STATUS_CANCELED)
    assert ros.result(handle).status == GoalStatus.STATUS_CANCELED
    ros.pump()
    assert len(ros.backend_goals) == 1


def test_force_handback_cancel_timeout_fails_closed(harness):
    ros = harness(cancel_timeout_sec=0.15)
    handle = ros.start()
    ros.proxy.notify_handback(force_replan=True)
    assert ros.result(handle).status == GoalStatus.STATUS_ABORTED
    assert not ros.proxy.motion_authorized
    assert not ros.send(9.0).accepted
    ros.finish_backend(0, GoalStatus.STATUS_CANCELED)
    ros.pump()
    assert len(ros.backend_goals) == 1


def test_force_handback_replans_only_remaining_route_and_keeps_moving_frame_stamp(harness):
    ros = harness()
    handle = ros.send_through()
    ros.until(lambda: ros.proxy.motion_authorized)
    ros.backend_goals[0].publish_feedback(
        NavigateThroughPoses.Feedback(number_of_poses_remaining=2))
    ros.until(lambda: ros.proxy._active.completed_poses == 1)
    request = ros.proxy._active.request
    request.poses[1].header.stamp.sec = 123
    request.poses[2].header.frame_id = 'odom'
    request.poses[2].header.stamp.sec = 456
    ros.paused = True
    ros.proxy.notify_handback(force_replan=True)
    ros.until(lambda: bool(ros.backend_cancels))
    ros.finish_backend(0, GoalStatus.STATUS_CANCELED)
    ros.until(lambda: ros.proxy.motion_authorized)
    retry = ros.backend_goals[1].request
    assert [pose.pose.position.x for pose in retry.poses] == [2.0, 3.0]
    assert retry.behavior_tree == 'through_tree.xml'
    assert retry.poses[0].header.stamp.sec == 0
    assert retry.poses[1].header.frame_id == 'odom'
    assert retry.poses[1].header.stamp.sec == 456
    ros.finish_backend(1, GoalStatus.STATUS_SUCCEEDED)
    assert ros.result(handle).status == GoalStatus.STATUS_SUCCEEDED


def test_replacement_during_force_handback_wins_without_resurrecting_original(harness):
    ros = harness()
    first = ros.start(1.0)
    ros.proxy.notify_handback(force_replan=True)
    second = ros.send(9.0)
    assert second.accepted
    assert ros.result(first).status == GoalStatus.STATUS_ABORTED
    assert not ros.proxy.motion_authorized
    ros.until(lambda: bool(ros.backend_cancels))
    ros.finish_backend(0, GoalStatus.STATUS_CANCELED)
    ros.until(lambda: ros.proxy.motion_authorized)
    assert len(ros.backend_goals) == 2
    assert ros.backend_goals[1].request.pose.pose.position.x == 9.0
    ros.finish_backend(1, GoalStatus.STATUS_SUCCEEDED)
    assert ros.result(second).status == GoalStatus.STATUS_SUCCEEDED


def test_replan_send_timeout_cancels_late_acceptance_without_motion(harness):
    ros = harness(send_timeout_sec=0.15)
    handle = ros.start()
    ros.paused = True
    ros.proxy.notify_handback(force_replan=True)
    ros.until(lambda: bool(ros.backend_cancels))
    ros.acceptance_gate = Future()
    ros.finish_backend(0, GoalStatus.STATUS_CANCELED)
    ros.until(lambda: len(ros.goal_requests) == 2)
    assert ros.result(handle).status == GoalStatus.STATUS_ABORTED
    ros.acceptance_gate.set_result(None)
    ros.until(lambda: len(ros.backend_cancels) == 2)
    assert not ros.proxy.motion_authorized
    ros.finish_backend(1, GoalStatus.STATUS_CANCELED)
    ros.pump()
    assert len(ros.backend_goals) == 2
