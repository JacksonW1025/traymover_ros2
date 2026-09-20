"""Exercise startup recovery and native bond shutdown in an isolated ROS domain.

The lifecycle servers are deterministic test doubles. No navigation, sensors,
velocity publisher, or robot driver is started.
"""

import os
from pathlib import Path
import signal
import subprocess
import threading
import time

from ament_index_python.packages import get_package_prefix
from bond.msg import Status
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
import pytest
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger


class LifecycleDouble(Node):
    def __init__(self, name, *, inactive=False, delayed=False, slow=False, rejected=False):
        super().__init__(name)
        self.state = (State.PRIMARY_STATE_INACTIVE if inactive
                      else State.PRIMARY_STATE_UNCONFIGURED)
        self.delayed = delayed
        self.slow = slow
        self.rejected = rejected
        self.heartbeat = True
        self.transitions = []
        group = MutuallyExclusiveCallbackGroup() if slow else ReentrantCallbackGroup()
        self.create_service(GetState, f'/{name}/get_state', self.get_state,
                            callback_group=group)
        self.create_service(ChangeState, f'/{name}/change_state', self.change_state,
                            callback_group=group)
        self.publisher = self.create_publisher(Status, '/bond', 10)
        self.create_timer(0.05, self.publish_bond, callback_group=group)

    def get_state(self, request, response):
        response.current_state.id = self.state
        return response

    def change_state(self, request, response):
        transition = request.transition.id
        self.transitions.append(transition)
        source, target = {
            Transition.TRANSITION_CONFIGURE: (
                State.PRIMARY_STATE_UNCONFIGURED, State.PRIMARY_STATE_INACTIVE),
            Transition.TRANSITION_ACTIVATE: (
                State.PRIMARY_STATE_INACTIVE, State.PRIMARY_STATE_ACTIVE),
            Transition.TRANSITION_DEACTIVATE: (
                State.PRIMARY_STATE_ACTIVE, State.PRIMARY_STATE_INACTIVE),
            Transition.TRANSITION_CLEANUP: (
                State.PRIMARY_STATE_INACTIVE, State.PRIMARY_STATE_UNCONFIGURED),
        }[transition]
        response.success = self.state == source and not self.rejected
        if response.success:
            if self.slow and transition == Transition.TRANSITION_CONFIGURE:
                self.state = State.TRANSITION_STATE_CONFIGURING
                # Model the map server: configuring blocks GetState too, for
                # longer than two client timeouts (10.6 s vs 5 s in real logs).
                time.sleep(2.6)
            self.state = target
            if self.delayed and transition == Transition.TRANSITION_CONFIGURE:
                # Client loses the response after 1 s, although the transition
                # succeeded. GetState must recover without duplicate configure.
                time.sleep(1.5)
        return response

    def publish_bond(self):
        if self.heartbeat and self.state == State.PRIMARY_STATE_ACTIVE:
            message = Status()
            message.id = self.get_name()
            message.instance_id = 'test-' + self.get_name()
            message.active = True
            message.heartbeat_timeout = 1.0
            message.heartbeat_period = 0.05
            self.publisher.publish(message)


@pytest.mark.parametrize('scenario', ['lost_response', 'slow_transition', 'bounded_failure'])
def test_startup_reconciliation_and_monitoring(monkeypatch, tmp_path, scenario):
    executable = (Path(get_package_prefix('traymover_robot_nav')) / 'lib' /
                  'traymover_robot_nav' / 'option18_lifecycle_manager')
    assert executable.is_file(), 'Build traymover_robot_nav before this integration test'
    domain = 130 + os.getpid() % 20
    monkeypatch.setenv('ROS_DOMAIN_ID', str(domain))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    rclpy.init(args=[], domain_id=domain)
    reject = scenario == 'bounded_failure'
    first = LifecycleDouble('startup_first', delayed=scenario == 'lost_response',
                            slow=scenario == 'slow_transition', rejected=reject)
    second = LifecycleDouble('startup_second', inactive=True)
    probe = Node('startup_test_probe')
    executor = MultiThreadedExecutor(num_threads=4)
    for node in (first, second, probe):
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    process = None
    log = tmp_path / 'manager.log'
    active = probe.create_client(Trigger, '/option18_test_manager/is_active')

    def until(predicate, description, seconds=15):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            assert process.poll() is None, log.read_text()
            if predicate():
                return
            time.sleep(0.03)
        pytest.fail(f'{description}\n{log.read_text()}')

    def manager_active():
        assert active.wait_for_service(timeout_sec=2.0)
        future = active.call_async(Trigger.Request())
        until(future.done, 'manager is_active did not answer')
        return future.result().success

    try:
        with log.open('w') as output:
            process = subprocess.Popen([
                str(executable), '--ros-args', '-r', '__node:=option18_test_manager',
                '-p', 'node_names:=[startup_first,startup_second]',
                '-p', 'autostart:=true', '-p', 'startup_service_timeout_sec:=1',
                '-p', 'startup_attempts:=3', '-p', 'bond_timeout:=1.0',
                '-p', 'attempt_respawn_reconnection:=false',
            ], stdout=output, stderr=subprocess.STDOUT)
            if reject:
                until(lambda: 'startup failed' in log.read_text(), 'failure was not bounded')
                assert not manager_active()
                assert first.transitions == [Transition.TRANSITION_CONFIGURE] * 3
                assert Transition.TRANSITION_ACTIVATE not in second.transitions
            else:
                until(lambda: 'Managed nodes are active' in log.read_text(),
                      'startup did not recover')
                assert manager_active()
                assert first.transitions == [Transition.TRANSITION_CONFIGURE,
                                             Transition.TRANSITION_ACTIVATE]
                assert second.transitions == [Transition.TRANSITION_ACTIVATE]
                # Retain Nav2's bond watchdog: loss of a server stops the stack.
                first.heartbeat = False
                until(lambda: second.state == State.PRIMARY_STATE_UNCONFIGURED,
                      'native bond watchdog did not reset surviving node')
                assert not manager_active()
                assert Transition.TRANSITION_DEACTIVATE in second.transitions
    finally:
        if process is not None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
        executor.shutdown(timeout_sec=3)
        thread.join(timeout=3)
        for node in (first, second, probe):
            node.destroy_node()
        rclpy.shutdown()
