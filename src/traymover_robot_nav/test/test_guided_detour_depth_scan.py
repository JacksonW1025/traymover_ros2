"""Test installed depth converters using synthetic images, without hardware or motion."""

from array import array
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Transform
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from detour_core import Config, Observation, Ray, Scan, Scene  # noqa: E402
from detour_obstacle import scan_to_rays  # noqa: E402


def test_real_depth_conversion_far_observations_invalid_depth_and_near_guard(
        monkeypatch, tmp_path):
    """Extended range admits measured far depth without hiding near hits or inventing rays."""
    executable = (Path(get_package_prefix('depthimage_to_laserscan')) / 'lib' /
                  'depthimage_to_laserscan' / 'depthimage_to_laserscan_node')
    assert executable.is_file() and os.access(executable, os.X_OK)
    domain = 160 + os.getpid() % 60
    monkeypatch.setenv('ROS_DOMAIN_ID', str(domain))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    context = Context()
    rclpy.init(args=[], context=context, domain_id=domain)
    io = Node('guided_depth_conversion_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(io)
    namespace = f'/guided_depth_test_{os.getpid()}'
    processes, logs = [], []
    received = {3: {}, 10: {}}
    deadline = time.monotonic() + 20.0

    def stamp_key(stamp):
        return stamp.sec, stamp.nanosec

    def record(limit, message):
        received[limit][stamp_key(message.header.stamp)] = message

    def diagnostics():
        return '\n'.join(path.read_text() for path in logs)

    try:
        depth = io.create_publisher(Image, namespace + '/depth', qos_profile_sensor_data)
        calibration = io.create_publisher(CameraInfo, namespace + '/info', qos_profile_sensor_data)
        for limit in (3, 10):
            io.create_subscription(
                LaserScan, namespace + f'/scan_{limit}',
                lambda message, limit=limit: record(limit, message), qos_profile_sensor_data)
            log = tmp_path / f'converter_{limit}.log'
            logs.append(log)
            with log.open('w') as output:
                processes.append(subprocess.Popen([
                    str(executable), '--ros-args', '-r', f'__node:=depth_test_{limit}',
                    '-r', f'depth:={namespace}/depth',
                    '-r', f'depth_camera_info:={namespace}/info',
                    '-r', f'scan:={namespace}/scan_{limit}',
                    '-p', 'scan_height:=10', '-p', 'range_min:=0.2',
                    '-p', f'range_max:={float(limit)}',
                    '-p', 'output_frame:=camera_depth_frame',
                ], stdout=output, stderr=subprocess.STDOUT, env=os.environ.copy()))

        def convert(kind):
            width, height = 64, 20
            info = CameraInfo()
            info.header.stamp = io.get_clock().now().to_msg()
            info.header.frame_id = 'camera_depth_optical_frame'
            info.width, info.height = width, height
            info.distortion_model = 'plumb_bob'
            info.d = [0.0] * 5
            info.k = [55.0, 0.0, 31.5, 0.0, 55.0, 10.0, 0.0, 0.0, 1.0]
            info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            info.p = [55.0, 0.0, 31.5, 0.0, 0.0, 55.0, 10.0, 0.0,
                      0.0, 0.0, 1.0, 0.0]
            image = Image()
            image.header = info.header
            image.width, image.height = width, height
            image.is_bigendian = int(sys.byteorder == 'big')
            image.encoding = '32FC1' if kind == 'nan' else '16UC1'
            image.step = width * (4 if kind == 'nan' else 2)
            pixels = [math.nan if kind == 'nan' else 0 if kind == 'zero' else 4000
                      for unused in range(width * height)]
            if kind == 'near_and_far':
                pixels[10 * width:11 * width] = [800] * width
            image.data = array('f' if kind == 'nan' else 'H', pixels).tobytes()
            key = stamp_key(image.header.stamp)
            while not all(key in received[limit] for limit in (3, 10)):
                assert all(process.poll() is None for process in processes), diagnostics()
                assert time.monotonic() < deadline, f'{kind}: 20 s deadline\n{diagnostics()}'
                calibration.publish(info)
                depth.publish(image)
                executor.spin_once(timeout_sec=0.02)
            return {limit: received[limit][key] for limit in (3, 10)}

        transform = Transform()
        transform.rotation.w = 1.0
        # Camera translation matches the production sensor's forward offset.
        transform.translation.x = 0.45
        lidar = Scan(1.0, tuple(Ray(0.0, 0.0, math.radians(deg), 6.0)
                                for deg in range(-180, 181)))
        config = Config()

        def scene(message):
            converted = scan_to_rays(message, transform, 1.0, allow_infinite=True)
            return Scene(Observation(lidar=lidar, realsense=converted), config)

        far = convert('far')
        assert all(math.isnan(value) for value in far[3].ranges)
        assert not scene(far[3]).forward_depth_observed()
        finite = [value for value in far[10].ranges if math.isfinite(value)]
        assert len(finite) > 50
        assert min(finite) >= 4.0
        assert scene(far[10]).forward_depth_observed()
        assert not scene(far[10]).obstacle()[0]
        assert scene(far[10]).clearance() > config.detour_guard_distance_m

        for kind in ('zero', 'nan'):
            for message in convert(kind).values():
                assert not scan_to_rays(message, transform, 1.0, allow_infinite=True).rays
                assert not scene(message).forward_depth_observed()

        near = convert('near_and_far')
        for message in near.values():
            observed = scene(message)
            assert observed.forward_depth_observed()
            assert observed.obstacle()[0]
            # Angular binning at this deliberately small image resolution adds
            # quantization to reconstruction of the original 0.8 m axial depth.
            assert observed.obstacle()[1] == pytest.approx(0.8, abs=0.01)
            assert observed.clearance() <= config.detour_guard_distance_m
        for old, extended in zip(near[3].ranges, near[10].ranges):
            assert (math.isnan(old) and math.isnan(extended)) or old == extended
    finally:
        io.destroy_node()
        executor.shutdown(timeout_sec=0.0)
        context.shutdown()
        for process in processes:
            for stopping_signal in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
                if process.poll() is not None:
                    break
                process.send_signal(stopping_signal)
                try:
                    process.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    continue
