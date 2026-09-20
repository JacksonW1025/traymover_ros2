#!/usr/bin/env python3
"""Preserve Nav2 pose and route ownership across an intentional navigation pause.

The supervisor owns velocity safety. It MUST gate every nonzero output using
``motion_authorized`` and reset its maneuver in ``on_goal_changed``. In particular,
a cancellation acknowledgement is not evidence that the backend has stopped:
authorization stays false until the old backend returns a terminal result.

All action operations are asynchronous. The supplied node can use a single
threaded executor; no callback spins or waits for another callback.
"""

from copy import deepcopy
from dataclasses import dataclass, field
import math
import time
from typing import Callable, Optional

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.clock import Clock, ClockType
from rclpy.task import Future


@dataclass
class _Goal:
    handle: object
    request: object
    action_type: object
    client: object
    completion: Future = field(default_factory=Future)
    backend_handle: object = None
    backend_alive: bool = False
    send_pending: bool = False
    waiting_for_server: bool = True
    accepted_for_motion: bool = False
    held_abort: bool = False
    handback_in_progress: bool = False
    replan_cancel: bool = False
    cancel_requested: bool = False
    preempted: bool = False
    external_finished: bool = False
    cancel_sent: bool = False
    cancel_timed_out: bool = False
    server_deadline: float = 0.0
    send_deadline: float = 0.0
    cancel_deadline: float = 0.0
    attempt: int = 0
    completed_poses: int = 0
    attempt_pose_offset: int = 0


class GoalProxy:
    """Proxy external goals, retaining an aborted backend goal during detour.

    ``notify_handback`` permits exactly one retry of an abort preserved during
    a pause. Call it on a successful transition back to NORMAL. A later, distinct
    pause may preserve another abort. A normal backend abort is forwarded.
    With ``force_replan=True``, a live backend is canceled and restarted after
    its terminal result without terminating the caller's public goal.

    A replacement goal is queued until the preceding backend terminates. Further
    replacements are rejected while that cancellation is outstanding. Timeouts
    terminate external actions within a bounded time, but an unacknowledged send
    or unterminated backend remains a safety interlock, including after timeout.
    A late acceptance is canceled and can never authorize motion.
    """

    def __init__(
        self,
        node,
        is_paused: Callable[[], bool],
        on_goal_changed: Callable[[], None],
        *,
        action_name: str = '/navigate_to_pose',
        backend_name: str = '/navigate_to_pose_detour_backend',
        through_action_name: str = '/navigate_through_poses',
        through_backend_name: str = '/navigate_through_poses_detour_backend',
        server_timeout_sec: float = 5.0,
        send_timeout_sec: float = 5.0,
        cancel_timeout_sec: float = 2.0,
    ):
        for name, value in (
            ('server_timeout_sec', server_timeout_sec),
            ('send_timeout_sec', send_timeout_sec),
            ('cancel_timeout_sec', cancel_timeout_sec),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and positive')
        self._node = node
        self._is_paused = is_paused
        self._on_goal_changed = on_goal_changed
        self._server_timeout = server_timeout_sec
        self._send_timeout = send_timeout_sec
        self._cancel_timeout = cancel_timeout_sec
        self._active: Optional[_Goal] = None
        self._records = []
        self._destroyed = False
        # Both public action types share this one active goal and cancellation
        # interlock. Separate proxies would let their backends drive concurrently.
        self._clients = {}
        self._servers = []
        for action_type, public_name, backend in (
                (NavigateToPose, action_name, backend_name),
                (NavigateThroughPoses, through_action_name, through_backend_name)):
            self._clients[action_type] = ActionClient(node, action_type, backend)
            self._servers.append(ActionServer(
                node, action_type, public_name,
                execute_callback=self._execute,
                goal_callback=self._goal_request,
                handle_accepted_callback=lambda handle, kind=action_type:
                    self._accepted(handle, kind),
                cancel_callback=self._cancel_request,
            ))
        self._client = self._clients[NavigateToPose]
        self._server = self._servers[0]
        # The repository's RViz SetGoal tool publishes PoseStamped. Route it
        # through the public action so it has the same ownership as API goals.
        # Launch disables bt_navigator's original /goal_pose subscription.
        self._topic_client = ActionClient(node, NavigateToPose, action_name)
        self._topic_goal = None
        self._topic_subscription = node.create_subscription(
            PoseStamped, '/goal_pose', self._on_goal_pose, 10)
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._timer = node.create_timer(0.05, self.tick, clock=self._steady_clock)

    @property
    def has_active_goal(self):
        return self._active is not None and not self._active.cancel_requested

    @property
    def saved_request(self):
        """A copy of the original goal, including its behavior_tree selection."""
        return deepcopy(self._active.request) if self.has_active_goal else None

    @property
    def motion_authorized(self):
        active = self._active
        return bool(
            active is not None
            and not active.cancel_requested
            and not active.replan_cancel
            and not active.external_finished
            and active.accepted_for_motion
            and not active.send_pending
            and not active.waiting_for_server
            and (not active.held_abort or self._is_paused())
            and not self._has_unresolved_cancel()
        )

    def _has_unresolved_cancel(self):
        return any(
            record.cancel_requested and (record.send_pending or record.backend_alive)
            for record in self._records
        )

    def _goal_request(self, _request):
        if self._destroyed or self._has_unresolved_cancel():
            self._node.get_logger().warning(
                '[detour] Goal rejected: previous backend cancellation is unresolved')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_goal_pose(self, pose):
        request = NavigateToPose.Goal()
        request.pose = deepcopy(pose)
        self._topic_goal = (request, time.monotonic() + self._server_timeout)

    def _topic_response(self, future):
        if self._destroyed:
            return
        try:
            handle = future.result()
            if not handle.accepted:
                self._node.get_logger().warning('[detour] RViz goal rejected by navigation proxy')
                return
            self._node.get_logger().info('[detour] RViz goal accepted; waiting for navigation')
            handle.get_result_async().add_done_callback(self._topic_result)
        except Exception as error:
            self._node.get_logger().error(f'[detour] RViz goal submission failed: {error}')

    def _topic_result(self, future):
        if self._destroyed:
            return
        try:
            status = future.result().status
        except Exception as error:
            self._node.get_logger().error(f'[detour] RViz navigation result unavailable: {error}')
            return
        outcomes = {
            GoalStatus.STATUS_SUCCEEDED: 'succeeded',
            GoalStatus.STATUS_CANCELED: 'canceled',
            GoalStatus.STATUS_ABORTED: 'aborted',
        }
        outcome = outcomes.get(status, f'ended with status {status}')
        logger = self._node.get_logger()
        if status == GoalStatus.STATUS_SUCCEEDED:
            logger.info(f'[detour] RViz navigation {outcome}')
        else:
            logger.warning(f'[detour] RViz navigation {outcome}')

    def _accepted(self, handle, action_type=NavigateToPose):
        previous = self._active
        if previous is not None:
            self._request_backend_cancel(previous, preempted=True)
            self._finish(previous, GoalStatus.STATUS_ABORTED)
        record = _Goal(
            handle=handle,
            request=deepcopy(handle.request),
            action_type=action_type,
            client=self._clients[action_type],
            server_deadline=time.monotonic() + self._server_timeout,
        )
        self._records.append(record)
        self._active = record
        self._on_goal_changed()

        async def execute_goal(_handle):
            return await record.completion

        handle.execute(execute_callback=execute_goal)

    async def _execute(self, handle):
        record = next(item for item in self._records if item.handle is handle)
        return await record.completion

    def _cancel_request(self, handle):
        record = next((item for item in self._records if item.handle is handle), None)
        if record is None or record.external_finished:
            return CancelResponse.REJECT
        self._request_backend_cancel(record)
        if record is self._active:
            self._on_goal_changed()
        # The server changes the handle to CANCELING after this callback returns.
        # tick/result callbacks finalize it later, never inside this callback.
        return CancelResponse.ACCEPT

    def _request_backend_cancel(self, record, *, preempted=False):
        if not record.cancel_requested:
            record.cancel_requested = True
            record.preempted = preempted
            record.accepted_for_motion = False
            record.waiting_for_server = False
            record.held_abort = False
            record.cancel_deadline = time.monotonic() + self._cancel_timeout
        self._cancel_backend_handle(record)

    def _cancel_backend_handle(self, record):
        if record.backend_alive and record.backend_handle is not None and not record.cancel_sent:
            record.cancel_sent = True
            try:
                future = record.backend_handle.cancel_goal_async()
                future.add_done_callback(lambda result: self._cancel_response(record, result))
            except Exception as error:  # Transport errors must fail closed.
                self._node.get_logger().error(f'[detour] Backend cancel failed: {error}')

    def _cancel_response(self, record, future):
        if self._destroyed:
            return
        try:
            response = future.result()
            if not response.goals_canceling and record.backend_alive:
                self._node.get_logger().warning(
                    '[detour] Backend did not acknowledge cancel; zero command required')
        except Exception as error:
            self._node.get_logger().error(f'[detour] Backend cancel response failed: {error}')
        # Do not clear backend_alive here. Only its terminal result releases it.

    def notify_handback(self, force_replan=False):
        """Retry a held abort, or restart a live backend after forward clearance.

        Internal cancellation retains the public goal. Its terminal result, not
        cancellation acknowledgement, permits submission of a fresh planning BT.
        """
        record = self._active
        if (record is None or record.cancel_requested or record.external_finished
                or record.handback_in_progress):
            return
        if not record.held_abort and not force_replan:
            return
        record.handback_in_progress = True
        record.accepted_for_motion = False
        if force_replan and (record.backend_alive or record.send_pending):
            record.replan_cancel = True
            record.cancel_deadline = time.monotonic() + self._cancel_timeout
            self._cancel_backend_handle(record)
        else:
            self._queue_handback(record)
        self.tick()

    def _queue_handback(self, record):
        record.replan_cancel = False
        record.held_abort = False
        record.accepted_for_motion = False
        record.waiting_for_server = True
        record.server_deadline = time.monotonic() + self._server_timeout
        self._node.get_logger().info('[detour] Reissuing preserved navigation goal after handback')

    def tick(self):
        if self._destroyed:
            return
        now = time.monotonic()
        if self._topic_goal is not None:
            request, deadline = self._topic_goal
            if now >= deadline:
                self._topic_goal = None
                self._node.get_logger().error('[detour] RViz navigation proxy unavailable')
            elif self._topic_client.server_is_ready():
                self._topic_goal = None
                try:
                    self._topic_client.send_goal_async(request).add_done_callback(
                        self._topic_response)
                except Exception as error:
                    self._node.get_logger().error(f'[detour] RViz goal submission failed: {error}')
        if (self._active is not None and not self._is_paused()
                and self._active.accepted_for_motion):
            self._active.handback_in_progress = False
        for record in tuple(self._records):
            if record.cancel_requested:
                self._request_backend_cancel(record)
                unresolved = record.send_pending or record.backend_alive
                if not unresolved or now >= record.cancel_deadline:
                    if unresolved and not record.cancel_timed_out:
                        record.cancel_timed_out = True
                        self._node.get_logger().error(
                            '[detour] Backend cancel timed out; motion remains inhibited')
                    self._finish(record, GoalStatus.STATUS_ABORTED)
            elif record.replan_cancel:
                self._cancel_backend_handle(record)
                if now >= record.cancel_deadline:
                    self._node.get_logger().error(
                        '[detour] Replan cancellation timed out; motion remains inhibited')
                    self._request_backend_cancel(record)
                    self._finish(record, GoalStatus.STATUS_ABORTED)
            elif record.send_pending and now >= record.send_deadline:
                self._node.get_logger().error(
                    '[detour] Backend goal acceptance timed out; motion remains inhibited')
                self._request_backend_cancel(record)
                self._finish(record, GoalStatus.STATUS_ABORTED)
            elif record.waiting_for_server:
                if now >= record.server_deadline:
                    self._node.get_logger().error(
                        '[detour] Backend server unavailable; goal aborted')
                    self._finish(record, GoalStatus.STATUS_ABORTED)
                elif not self._has_unresolved_cancel() and record.client.server_is_ready():
                    self._send(record, now)
        # Keep unresolved sends/backends for their late callbacks; never authorize
        # another goal merely because a timeout expired.
        self._records[:] = [
            record for record in self._records
            if not record.external_finished or record.send_pending or record.backend_alive
        ]

    def _send(self, record, now):
        record.waiting_for_server = False
        record.send_pending = True
        record.accepted_for_motion = False
        record.send_deadline = now + self._send_timeout
        record.attempt += 1
        attempt = record.attempt
        request = deepcopy(record.request)
        record.attempt_pose_offset = record.completed_poses
        if record.action_type is NavigateThroughPoses:
            request.poses = request.poses[record.attempt_pose_offset:]
        if record.attempt > 1:
            poses = request.poses if record.action_type is NavigateThroughPoses else [request.pose]
            for pose in poses:
                # Map goals are fixed world coordinates. Moving-frame targets
                # must retain their original timestamp and frame semantics.
                if pose.header.frame_id == 'map':
                    pose.header.stamp.sec = 0
                    pose.header.stamp.nanosec = 0
        try:
            future = record.client.send_goal_async(
                request,
                feedback_callback=lambda feedback: self._feedback(record, attempt, feedback),
            )
            future.add_done_callback(lambda result: self._send_response(record, attempt, result))
        except Exception as error:
            # The transport may already have sent the request before failing.
            # Keep its unresolved-send interlock until node restart or a response.
            self._node.get_logger().error(f'[detour] Backend goal send failed: {error}')
            self._request_backend_cancel(record)
            self._finish(record, GoalStatus.STATUS_ABORTED)

    def _feedback(self, record, attempt, feedback):
        if (
            not self._destroyed and record is self._active
            and record.attempt == attempt and not record.cancel_requested
            and record.handle.is_active
        ):
            if record.action_type is NavigateThroughPoses:
                remaining = feedback.feedback.number_of_poses_remaining
                count = len(record.request.poses)
                if 0 <= remaining <= count - record.attempt_pose_offset:
                    # Never revisit confirmed passed poses after handback. Keep
                    # the final pose until Nav2 reports terminal success, even
                    # when RemovePassedGoals has removed every intermediate pose.
                    record.completed_poses = max(
                        record.completed_poses, min(count - 1, count - remaining))
            record.handle.publish_feedback(feedback.feedback)

    def _send_response(self, record, attempt, future):
        if self._destroyed or attempt != record.attempt:
            return
        try:
            handle = future.result()
        except Exception as error:
            self._node.get_logger().error(f'[detour] Backend goal response failed: {error}')
            self._request_backend_cancel(record)
            self._finish(record, GoalStatus.STATUS_ABORTED)
            return
        record.send_pending = False
        if not handle.accepted:
            self._node.get_logger().error(
                '[detour] Backend rejected navigation goal; goal aborted')
            self._finish(record, GoalStatus.STATUS_ABORTED)
            return
        record.backend_handle = handle
        record.backend_alive = True
        try:
            handle.get_result_async().add_done_callback(
                lambda result: self._result(record, attempt, result))
        except Exception as error:
            self._node.get_logger().error(f'[detour] Backend result request failed: {error}')
            self._request_backend_cancel(record)
            self._finish(record, GoalStatus.STATUS_ABORTED)
            return
        if record.cancel_requested or record is not self._active or record.external_finished:
            self._request_backend_cancel(record)
        elif record.replan_cancel:
            self._cancel_backend_handle(record)
        else:
            record.accepted_for_motion = True

    def _result(self, record, attempt, future):
        if self._destroyed or attempt != record.attempt:
            return
        try:
            response = future.result()
        except Exception as error:
            self._node.get_logger().error(f'[detour] Backend result unavailable: {error}')
            self._request_backend_cancel(record)
            self._finish(record, GoalStatus.STATUS_ABORTED)
            return
        record.backend_alive = False
        record.backend_handle = None
        record.cancel_sent = False
        if record.cancel_requested:
            self._finish(record, GoalStatus.STATUS_ABORTED, response.result)
        elif record is self._active:
            if record.replan_cancel:
                self._queue_handback(record)
                return
            if (
                response.status == GoalStatus.STATUS_ABORTED
                and self._is_paused()
                and not record.handback_in_progress
            ):
                record.held_abort = True
                self._node.get_logger().info(
                    '[detour] Backend aborted while paused; preserving original navigation goal')
            else:
                self._finish(record, response.status, response.result)

    def _finish(self, record, status, result=None):
        if record.external_finished:
            return
        record.external_finished = True
        record.accepted_for_motion = False
        record.waiting_for_server = False
        if record.handle.is_active:
            if record.handle.is_cancel_requested:
                record.handle.canceled()
            elif status == GoalStatus.STATUS_SUCCEEDED:
                record.handle.succeed()
            else:
                record.handle.abort()
        record.completion.set_result(result if result is not None else record.action_type.Result())
        if record is self._active:
            self._active = None
            self._on_goal_changed()

    def destroy(self):
        """Destroy ROS entities; the supervisor must assert zero before teardown."""
        self._destroyed = True
        self._node.destroy_timer(self._timer)
        self._node.destroy_subscription(self._topic_subscription)
        self._topic_client.destroy()
        for server in self._servers:
            server.destroy()
        for client in self._clients.values():
            client.destroy()
