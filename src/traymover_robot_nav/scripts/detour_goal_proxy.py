#!/usr/bin/env python3
"""Preserve NavigateToPose ownership across an intentional navigation pause.

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
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.clock import Clock, ClockType
from rclpy.task import Future


@dataclass
class _Goal:
    handle: object
    request: object
    completion: Future = field(default_factory=Future)
    backend_handle: object = None
    backend_alive: bool = False
    send_pending: bool = False
    waiting_for_server: bool = True
    accepted_for_motion: bool = False
    held_abort: bool = False
    handback_in_progress: bool = False
    cancel_requested: bool = False
    preempted: bool = False
    external_finished: bool = False
    cancel_sent: bool = False
    cancel_timed_out: bool = False
    server_deadline: float = 0.0
    send_deadline: float = 0.0
    cancel_deadline: float = 0.0
    attempt: int = 0


class GoalProxy:
    """Proxy external goals, retaining an aborted backend goal during detour.

    ``notify_handback`` permits exactly one retry of an abort preserved during
    a pause. Call it on a successful transition back to NORMAL. A later, distinct
    pause may preserve another abort. A normal backend abort is forwarded.

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
        self._client = ActionClient(node, NavigateToPose, backend_name)
        self._server = ActionServer(
            node, NavigateToPose, action_name,
            execute_callback=self._execute,
            goal_callback=self._goal_request,
            handle_accepted_callback=self._accepted,
            cancel_callback=self._cancel_request,
        )
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

    def _accepted(self, handle):
        previous = self._active
        if previous is not None:
            self._request_backend_cancel(previous, preempted=True)
            self._finish(previous, GoalStatus.STATUS_ABORTED)
        record = _Goal(
            handle=handle,
            request=deepcopy(handle.request),
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

    def notify_handback(self):
        """Reissue a preserved goal once, after a successful NORMAL handback."""
        record = self._active
        if record is None or record.cancel_requested or not record.held_abort:
            return
        record.held_abort = False
        record.handback_in_progress = True
        record.accepted_for_motion = False
        record.waiting_for_server = True
        record.server_deadline = time.monotonic() + self._server_timeout
        self._node.get_logger().info('[detour] Reissuing preserved navigation goal after handback')
        self.tick()

    def tick(self):
        if self._destroyed:
            return
        now = time.monotonic()
        if self._active is not None and not self._is_paused():
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
                elif not self._has_unresolved_cancel() and self._client.server_is_ready():
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
        try:
            future = self._client.send_goal_async(
                deepcopy(record.request),
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
        record.completion.set_result(result if result is not None else NavigateToPose.Result())
        if record is self._active:
            self._active = None
            self._on_goal_changed()

    def destroy(self):
        """Destroy ROS entities; the supervisor must assert zero before teardown."""
        self._destroyed = True
        self._node.destroy_timer(self._timer)
        self._server.destroy()
        self._client.destroy()
