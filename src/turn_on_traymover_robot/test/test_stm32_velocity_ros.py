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
from tf2_msgs.msg import TFMessage

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
        node.handle_motor_feedback(value, Time(sec=10, nanosec=100000000))
        assert node.odom_x == pytest.approx(0.04)
        node.handle_motor_feedback(value, Time(sec=20))
        assert node.odom_x == pytest.approx(0.04)
        node.handle_motor_feedback(value, Time(sec=1))
        assert node.odom_x == pytest.approx(0.04)
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
        node = robot.TurnOnTraymoverRobot(
            namespace='option21_test',
            cli_args=['--ros-args', '-r', '/cmd_vel:=/option21_test/cmd_vel',
                      '-r', '/tf:=/option21_test/tf'],
            parameter_overrides=[
                Parameter('usart_port_name', value=os.ttyname(slave)),
                Parameter('odom_source_mode', value='stm32_velocity'),
                Parameter('publish_odom_tf', value=True),
                Parameter('status_poll_rate_hz', value=0.0),
            ])
        observer = rclpy.create_node('option21_observer')
        messages, transforms = [], []
        observer.create_subscription(Odometry, '/option21_test/odom', messages.append, 10)
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

        state.update(vx=-200, wz=-500)
        spin_until(lambda: sum(m.twist.twist.angular.z == -0.5 for m in messages) >= 6)
        assert messages[-1].twist.twist.linear.x == pytest.approx(-0.2)
        assert messages[-1].pose.pose.orientation.z < 0

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
