"""Pure-software ROS 2 validation using the real serial driver and a PTY MCU."""
import math
import os
from pathlib import Path
import pty
import select
import sys
import threading
import time

import pytest
import rclpy
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from turn_on_traymover_robot import traymover_robot as robot


def feedback(vx, wz):
    frame = bytearray.fromhex(
        '7f 7f 28 02 60 00 01 47 00 00 00 00 00 00 00 00 32 00 05 00 '
        '32 00 32 00 df 00 b3 00 00 00 00 00 00 02 00 01 ba 00 0d 0a')
    frame[29:31] = vx.to_bytes(2, 'big', signed=True)
    frame[31:33] = wz.to_bytes(2, 'big', signed=True)
    frame[37] = robot.compute_checksum(frame)
    return bytes(frame)


@pytest.fixture
def ros():
    # Keep the software MCU experiment away from a live robot's ROS graph.
    rclpy.init(domain_id=80 + os.getpid() % 100)
    yield
    rclpy.shutdown()


def test_exact_arc_units_and_reverse():
    assert robot.integrate_velocity_step(0, 0, 0, 0.4, 0, 2) == pytest.approx((0.8, 0, 0))
    assert robot.integrate_velocity_step(0, 0, 0, -0.2, 0, 2) == pytest.approx((-0.4, 0, 0))
    x, y, yaw = robot.integrate_velocity_step(0, 0, 0, 1, 1, math.pi / 2)
    assert (x, y, yaw) == pytest.approx((1, 1, math.pi / 2))
    assert robot.integrate_velocity_step(0, 0, 0, 0, -1, math.pi / 2)[2] == pytest.approx(-math.pi / 2)


def test_ros_time_first_frame_gap_and_reset(ros, monkeypatch):
    # This timing check uses ROS stamps, independent of wall-clock test speed.
    monkeypatch.setattr(robot.serial, 'Serial', lambda **kwargs: (_ for _ in ()).throw(
        robot.serial.SerialException('no hardware in timing test')))
    node = robot.TurnOnTraymoverRobot(parameter_overrides=[
        Parameter('odom_source_mode', value='stm32_velocity')])
    try:
        value = robot.parse_motor_feedback_frame(feedback(400, 0))
        node.handle_motor_feedback(value, Time(sec=10))
        assert node.odom_x == 0
        assert node.left_wheel_angle == 0
        node.handle_motor_feedback(value, Time(sec=10, nanosec=100000000))
        assert node.odom_x == pytest.approx(0.04)
        node.handle_motor_feedback(value, Time(sec=20))
        assert node.odom_x == pytest.approx(0.04)
        assert node.left_wheel_angle == pytest.approx(0.04 / .085)
        node.handle_motor_feedback(value, Time(sec=1))
        assert node.odom_x == pytest.approx(0.04)
        assert node.left_wheel_angle == pytest.approx(0.04 / .085)
        node.handle_motor_feedback(value, Time(sec=1, nanosec=100000000))
        assert node.odom_x == pytest.approx(0.08)
    finally:
        node.destroy_node()


def test_real_pty_to_ros_odom_and_tf(ros):
    master, slave = pty.openpty()
    state = {'vx': 400, 'wz': 0, 'bad': False}
    stop = threading.Event()
    writes = []

    def mcu():
        buffer = bytearray()
        while not stop.is_set():
            if not select.select([master], [], [], 0.02)[0]:
                continue
            try:
                buffer.extend(os.read(master, 4096))
            except OSError:
                continue
            while True:
                request = robot.extract_next_valid_frame(buffer)
                if request is None:
                    break
                writes.append(request)
                response = bytearray(feedback(state['vx'], state['wz']))
                if state['bad']:
                    response[37] ^= 1
                # Real serial fragmentation, not a mock of serial.Serial.
                os.write(master, response[:9])
                os.write(master, response[9:])

    thread = threading.Thread(target=mcu, daemon=True)
    node = observer = executor = None
    try:
        profile_path = Path(__file__).resolve().parents[1] / 'config/stm32_navigation.yaml'
        profile = yaml.safe_load(profile_path.read_text())['turn_on_traymover_robot']['ros__parameters']
        profile.update(usart_port_name=os.ttyname(slave), status_poll_rate_hz=0.0)
        node = robot.TurnOnTraymoverRobot(
            namespace='option21_test',
            cli_args=['--ros-args', '-r', '/cmd_vel:=/option21_test/cmd_vel',
                      '-r', '/tf:=/option21_test/tf'],
            parameter_overrides=[Parameter(name, value=value) for name, value in profile.items()])
        observer = rclpy.create_node('option21_observer')
        messages, transforms, joints = [], [], []
        observer.create_subscription(Odometry, '/option21_test/odom', messages.append, 10)
        observer.create_subscription(JointState, '/option21_test/joint_states', joints.append, 10)
        observer.create_subscription(TFMessage, '/option21_test/tf',
                                     lambda msg: transforms.extend(msg.transforms), 10)
        commands = observer.create_publisher(robot.Twist, '/option21_test/cmd_vel', 10)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(observer)
        thread.start()

        def spin_until(predicate, timeout=5):
            deadline = time.monotonic() + timeout
            while not predicate() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.02)
            assert predicate(), 'Timed out waiting for serial -> ROS feedback'

        def check_wire_command(vx, wz, expected):
            start = len(writes)
            command = robot.Twist()
            command.linear.x = vx
            command.angular.z = wz
            commands.publish(command)
            spin_until(lambda: any(
                tuple(int.from_bytes(frame[offset:offset + 2], 'big', signed=True)
                      for offset in (10, 12)) == expected
                for frame in writes[start:]))

        spin_until(lambda: len(messages) >= 12 and transforms)
        # Feedback says 400 mm/s while every initial command frame says zero.
        assert all(frame[10:14] == b'\0' * 4 for frame in writes)
        first, last = messages[0], messages[-1]
        def stamp(msg):
            return msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        assert last.twist.twist.linear.x == pytest.approx(0.4)
        assert last.pose.pose.position.x - first.pose.pose.position.x == pytest.approx(
            0.4 * (stamp(last) - stamp(first)), abs=1e-5)
        assert (last.header.frame_id, last.child_frame_id) == ('odom', 'base_footprint')
        assert (transforms[-1].header.frame_id, transforms[-1].child_frame_id) == (
            'odom', 'base_footprint')
        spin_until(lambda: any(t.header.stamp == last.header.stamp for t in transforms))
        matching = [t for t in transforms if t.header.stamp == last.header.stamp]
        assert matching
        assert matching[-1].transform.translation.x == pytest.approx(last.pose.pose.position.x)
        spin_until(lambda: any(j.header.stamp == last.header.stamp for j in joints))
        joint = next(j for j in joints if j.header.stamp == last.header.stamp)
        assert joint.name == ['left_wheel_link_joint', 'right_wheel_link_joint']
        assert joint.position == pytest.approx([
            last.pose.pose.position.x / .085, -last.pose.pose.position.x / .085])
        assert joint.velocity == pytest.approx([.4 / .085, -.4 / .085])

        def yaw(orientation):
            return 2.0 * math.atan2(orientation.z, orientation.w)

        check_wire_command(0.3, 0.0, (300, 0))
        check_wire_command(0.3, 0.5, (150, 250))
        check_wire_command(0.3, -0.5, (150, -250))
        check_wire_command(0.0, 0.0, (0, 0))

        for wire_rate, ros_rate in [(-500, 0.5), (500, -0.5)]:
            check_wire_command(0.0, ros_rate, (0, int(math.copysign(250, ros_rate))))

            start = len(messages)
            state.update(vx=0, wz=wire_rate)
            spin_until(lambda: sum(
                m.twist.twist.angular.z == ros_rate for m in messages[start:]) >= 6)
            first_turn = next(m for m in messages[start:] if m.twist.twist.angular.z == ros_rate)
            turn = messages[-1]
            assert turn.twist.twist.linear.x == 0.0
            assert turn.pose.pose.position.x == pytest.approx(first_turn.pose.pose.position.x)
            assert turn.pose.pose.position.y == pytest.approx(first_turn.pose.pose.position.y)
            assert yaw(turn.pose.pose.orientation) - yaw(first_turn.pose.pose.orientation) == (
                pytest.approx(ros_rate * (stamp(turn) - stamp(first_turn)), abs=1e-6))
            spin_until(lambda: any(j.header.stamp == turn.header.stamp for j in joints))
            joint = next(j for j in joints if j.header.stamp == turn.header.stamp)
            assert joint.velocity == pytest.approx([-ros_rate * .455 / 2 / .085] * 2)
            spin_until(lambda: any(t.header.stamp == turn.header.stamp for t in transforms))
            transform = next(t for t in transforms if t.header.stamp == turn.header.stamp)
            assert yaw(transform.transform.rotation) == pytest.approx(yaw(turn.pose.pose.orientation))

        state.update(vx=-200, wz=0)
        spin_until(lambda: messages[-1].twist.twist.linear.x == -.2)

        # Nonzero outgoing commands cannot manufacture odom from bad CRC frames.
        state['bad'] = True
        cmd = robot.Twist()
        cmd.linear.x = 0.1
        commands.publish(cmd)
        before = len(messages)
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert len(messages) <= before + 1  # at most one already queued valid frame
        assert any(int.from_bytes(f[10:12], 'big', signed=True) == 100 for f in writes)
        old_x = messages[-1].pose.pose.position.x
        state.update(bad=False, vx=400, wz=0)
        before = len(messages)
        spin_until(lambda: len(messages) > before)
        assert messages[before].pose.pose.position.x == pytest.approx(old_x)
    finally:
        if executor:
            executor.shutdown()
        if node:
            node.destroy_node()
        stop.set()
        if thread.is_alive():
            thread.join(timeout=1)
        if observer:
            observer.destroy_node()
        os.close(master)
        os.close(slave)


def test_waits_for_port_and_valid_feedback_again_after_reconnect(ros, tmp_path, monkeypatch):
    monkeypatch.setattr(robot, 'SERIAL_RETRY_INTERVAL_SEC', 0.0)
    port = tmp_path / 'stm32'
    node = robot.TurnOnTraymoverRobot(parameter_overrides=[
        Parameter('usart_port_name', value=str(port)),
        Parameter('odom_source_mode', value='stm32_velocity'),
        Parameter('status_poll_rate_hz', value=0.0),
    ])
    command = robot.Twist()
    command.linear.x = .2

    def poll_speed(master):
        node.send_frame_callback()
        assert select.select([master], [], [], .5)[0]
        frame = robot.extract_next_valid_frame(bytearray(os.read(master, 256)))
        assert frame is not None
        return int.from_bytes(frame[10:12], 'big', signed=True)

    first_master, first_slave = pty.openpty()
    second_master, second_slave = pty.openpty()
    try:
        assert node.serial_port is None
        node.cmd_vel_callback(command)
        port.symlink_to(os.ttyname(first_slave))
        assert poll_speed(first_master) == 0
        os.write(first_master, feedback(0, 0))
        assert poll_speed(first_master) == 0
        node.cmd_vel_callback(command)
        assert poll_speed(first_master) == 200

        os.close(first_master)
        first_master = None
        node.send_frame_callback()
        assert node.serial_port is None
        assert node._last_velocity_stamp_ns is None
        assert node._latest_motor_feedback is None
        port.unlink()
        port.symlink_to(os.ttyname(second_slave))
        node.cmd_vel_callback(command)
        assert poll_speed(second_master) == 0
        os.write(second_master, feedback(0, 0))
        assert poll_speed(second_master) == 0
        node.cmd_vel_callback(command)
        assert poll_speed(second_master) == 200
    finally:
        node.destroy_node()
        if first_master is not None:
            os.close(first_master)
        os.close(first_slave)
        os.close(second_master)
        os.close(second_slave)
