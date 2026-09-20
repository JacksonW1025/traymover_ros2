"""Check production remaps against real, inactive Nav2; never start hardware."""

import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from rclpy.action import ActionClient, get_action_server_names_and_types_by_node
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


PKG = Path(__file__).resolve().parents[1]


def test_real_bt_navigator_exposes_only_private_backend_actions(tmp_path, monkeypatch):
    path = PKG / 'launch' / 'traymover_nav_lidar_realsense_detour.launch.py'
    spec = importlib.util.spec_from_file_location('option18_endpoint_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    executable = (Path(get_package_prefix('nav2_bt_navigator')) /
                  'lib' / 'nav2_bt_navigator' / 'bt_navigator')
    assert executable.is_file(), f'Installed bt_navigator is missing: {executable}'
    # Do not inherit the robot domain. Only a configure transition is sent;
    # no navigation goal, controller, driver, or lifecycle manager is started.
    domain = 130 + os.getpid() % 20
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    environment = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY='1',
                       ROS_LOG_DIR=str(tmp_path / 'ros_logs'))
    command = [str(executable), '--ros-args',
               '--params-file', str(PKG / 'config' / 'nav2_params.yaml')]
    for source, destination in module._navigation_remappings():
        command.extend(['-r', f'{source}:={destination}'])

    context = Context()
    context.init(args=[], domain_id=domain)
    executor = SingleThreadedExecutor(context=context)
    node = Node('option18_endpoint_test', context=context)
    executor.add_node(node)
    clients = []
    process = None
    log_path = tmp_path / 'navigator.log'

    def until(predicate, timeout=15.0):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            assert process.poll() is None, log_path.read_text()
            executor.spin_once(timeout_sec=0.05)
        assert predicate(), log_path.read_text()

    try:
        with log_path.open('w') as output:
            process = subprocess.Popen(command, env=environment, stdout=output,
                                       stderr=subprocess.STDOUT)
            change = node.create_client(ChangeState, '/bt_navigator/change_state')
            state = node.create_client(GetState, '/bt_navigator/get_state')
            until(change.service_is_ready)
            request = ChangeState.Request()
            request.transition.id = Transition.TRANSITION_CONFIGURE
            configured = change.call_async(request)
            until(configured.done)
            assert configured.result().success, log_path.read_text()
            until(state.service_is_ready)
            current = state.call_async(GetState.Request())
            until(current.done)
            assert current.result().current_state.id == State.PRIMARY_STATE_INACTIVE

            expected = set()
            for action_type, public in (
                    (NavigateToPose, '/navigate_to_pose'),
                    (NavigateThroughPoses, '/navigate_through_poses')):
                backend = public + '_detour_backend'
                expected.add(backend)
                client = ActionClient(node, action_type, backend)
                clients.append(client)
                until(client.server_is_ready)
                # server_is_ready checks all three services. Also verify the
                # status/feedback publishers, completing all five endpoints.
                for endpoint in ('feedback', 'status'):
                    until(lambda: node.count_publishers(f'{backend}/_action/{endpoint}') == 1)
                    assert node.count_publishers(f'{public}/_action/{endpoint}') == 0
            actual = {
                name for name, _ in get_action_server_names_and_types_by_node(
                    node, 'bt_navigator', '/')
            }
            assert actual == expected
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5.0)
        for client in clients:
            client.destroy()
        executor.shutdown(timeout_sec=0.0)
        node.destroy_node()
        context.shutdown()
