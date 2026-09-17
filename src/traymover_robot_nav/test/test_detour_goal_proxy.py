#!/usr/bin/env python3
"""Real rclpy action integration tests, isolated from the robot's ROS domain."""

import importlib.util
import os
from pathlib import Path
import sys
import time

import pytest

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
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
        self.client = ActionClient(self.client_node, NavigateToPose, '/navigate_to_pose')
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
        self.backend_completions[bytes(handle.goal_id.uuid)].set_result(NavigateToPose.Result())

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

    def result(self, handle):
        future = handle.get_result_async()
        self.until(future.done)
        return future.result()

    def close(self):
        self.proxy.destroy()
        self.client.destroy()
        if self.backend is not None:
            self.backend.destroy()
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


def test_backend_rejection_never_authorizes_motion(harness):
    ros = harness()
    ros.reject_goals = True
    handle = ros.send()
    assert ros.result(handle).status == GoalStatus.STATUS_ABORTED
    assert not ros.proxy.motion_authorized
    assert not ros.backend_goals
