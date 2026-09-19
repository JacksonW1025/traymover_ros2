"""Sensor regressions: frames, reset rejection, actual timestamps and image layout."""

from pathlib import Path
import math
import sys

import numpy as np
import pytest
from builtin_interfaces.msg import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import LookupException

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from option20_clouds import (clearing_points, cloud_xyz, fresh_stamp, obstacle_points,  # noqa: E402
                             transform_points, virtual_scan, xyz_cloud)
from option20_depth import depth_xyz  # noqa: E402
from option20_odom import PlanarTracker, planar_pose, rotation_matrix  # noqa: E402


def test_planar_pose_rotates_imu_lever_arm():
    angle = 0.3
    pose = planar_pose([1., 2., 0.4], [0., math.sin(angle/2), 0., math.cos(angle/2)])
    assert pose == pytest.approx([1.+0.12*math.sin(angle), 2., 0.])


def test_imu_below_base_lever_arm_roll_and_ground_offsets_agree():
    angle = 0.3
    quaternion = [math.sin(angle/2), 0., 0., math.cos(angle/2)]
    pose = planar_pose([1., 2., 0.], quaternion)
    assert pose == pytest.approx([1., 2.-0.12*math.sin(angle), 0.])
    # At level initialization, the full body->base->footprint chain yields z=0.
    body_in_odom = np.array([0., 0., 0.08])
    body_to_base = rotation_matrix([0., 0., 0., 1.]) @ [0., 0., 0.12]
    footprint = body_in_odom + body_to_base - [0., 0., 0.20]
    assert footprint == pytest.approx([0., 0., 0.])
    # FAST-LIO registered lidar coordinate height equals URDF laser ground height.
    assert 0.08 + 0.12 + 1.365 == pytest.approx(0.20 + 1.365)


def test_twist_is_in_base_frame_and_yaw_wrap_is_continuous():
    tracker = PlanarTracker(filter_tau=0.)
    tracker.update(10., 10., np.array([0., 0., math.pi/2]))
    twist = tracker.update(10.1, 10.1, np.array([0., 0.02, math.pi/2]))
    assert twist == pytest.approx([0.2, 0., 0.], abs=1e-9)
    wrap_tracker = PlanarTracker(filter_tau=0.)
    wrap_tracker.update(20., 20., np.array([0., 0., math.pi-0.01]))
    assert wrap_tracker.update(20.1, 20.1, np.array([0., 0., -math.pi+0.01]))[2] == pytest.approx(0.2)


@pytest.mark.parametrize('stamp,pose,reason', [
    (10., [0., 0., 0.], 'timestamp_reset'),
    (11., [0., 0., 0.], 'gap'),
    (10.1, [1., 0., 0.], 'jump'),
])
def test_odom_faults_latch_without_changing_accepted_pose(stamp, pose, reason):
    tracker = PlanarTracker()
    tracker.update(10., 10., np.zeros(3))
    with pytest.raises(ValueError, match=reason):
        tracker.update(stamp, stamp, np.array(pose))
    assert tracker.previous[0] == 10.
    with pytest.raises(ValueError, match=reason):
        tracker.update(10.2, 10.2, np.zeros(3))


def test_stale_odom_is_not_republished_and_future_is_rejected():
    tracker = PlanarTracker()
    for stamp, now in [(10., 11.), (12., 11.)]:
        with pytest.raises(ValueError, match='stale_or_future'):
            tracker.update(stamp, now, np.zeros(3))
    assert tracker.previous is None
    assert not fresh_stamp(0., 0., 0.5)


def test_three_bands_union_nearest_return_and_lidar_origin():
    points = np.array([[2.435, 0., 1.2], [3.435, 0., 1.6], [4.435, 0., 2.0],
                       [0.435, 2., 1.6], [1.435, 0., 0.1]])
    ranges, increment, stats = virtual_scan(points, [1.1, 1.5, 1.5, 1.9, 1.9, 2.3])
    assert increment == pytest.approx(math.pi/360)
    assert ranges[360] == pytest.approx(2.)
    assert stats == {'points': 4, 'bins': 2, 'count': 720, 'band_bins': [1, 2, 1]}
    assert np.isinf(ranges[0])


def test_far_scan_returns_quantize_to_nearest_angle_and_wrap_pi():
    increment = math.pi/360
    for angle in (0.9*increment, -0.9*increment, math.pi, -math.pi,
                  math.pi-0.1*increment):
        point = [[0.435+49.*math.cos(angle), 49.*math.sin(angle), 1.3]]
        ranges, actual_increment, _ = virtual_scan(point, [1.1, 1.5])
        index = np.flatnonzero(np.isfinite(ranges)).item()
        represented_angle = -math.pi+index*actual_increment
        error = math.atan2(math.sin(represented_angle-angle),
                           math.cos(represented_angle-angle))
        assert abs(error) <= actual_increment/2+1e-12
        if abs(angle) == math.pi:
            assert index == 0


def test_collision_filter_retains_low_obstacle_but_removes_ground_and_self():
    points = np.array([[1., 0., 0.15], [1., 0., 0.05], [0.2, 0., 0.2], [1., 0., 3.]])
    assert obstacle_points(points).tolist() == [[1., 0., 0.15]]


def test_sensor_overhang_filter_preserves_front_low_obstacles():
    obstacles = [[0.435, 0., 0.15], [0.55, 0., 0.20], [0.45, 0., 0.8],
                 [0.30, 0., 0.8], [0.43, 0.15, 1.565]]
    self_points = [[0.435, 0., 1.565], [0.375, 0., 0.8], [0.45, 0., 0.95]]
    assert obstacle_points(np.array(obstacles+self_points)).tolist() == obstacles


def test_person_departure_preserves_ground_returns_for_costmap_clearing():
    person_present = np.array([[1.5, 0., 0.8], [1.6, 0.1, 1.2]])
    person_departed = np.array([[2.5, 0., 0.02], [2.6, 0.1, -0.03]])
    assert len(obstacle_points(person_present)) == 2
    assert len(clearing_points(person_present)) == 0
    assert len(obstacle_points(person_departed)) == 0
    assert np.array_equal(clearing_points(person_departed), person_departed)


def test_clearing_never_uses_invalid_high_far_or_self_returns():
    unsafe = np.array([[np.nan, 0., 0.], [np.inf, 0., 0.], [1., 0., 2.5],
                       [1., 0., 0.12], [0.3, 0., 0.1], [1., 0., -0.11], [51., 0., 0.]])
    assert clearing_points(unsafe).shape == (0, 3)
    assert clearing_points(np.empty((0, 3))).shape == (0, 3)
    assert len(clearing_points(np.array([[5.1, 0., 0.]]), max_range=5.)) == 0


def test_cloud_decoder_big_endian_organized_with_padding():
    points = np.array([[1., 2., 3.], [4., 5., 6.], [7., 8., 9.], [10., 11., 12.]])
    cloud = xyz_cloud(points, Time(sec=5))
    cloud.height, cloud.width, cloud.row_step = 2, 2, 28
    cloud.is_bigendian = True
    data = points.astype('>f4').tobytes()
    cloud.data = data[:24]+b'pad!'+data[24:]+b'pad!'
    assert np.array_equal(cloud_xyz(cloud), points)


def make_depth(encoding='16UC1', endian=False):
    image = Image()
    image.header.frame_id = 'camera_depth_optical_frame'
    image.width, image.height = 3, 2
    image.encoding, image.is_bigendian = encoding, endian
    dtype = np.dtype(('>' if endian else '<')+('u2' if encoding == '16UC1' else 'f4'))
    values = np.array([[1000, 0, 2000], [3000, 4000, 5000]], dtype=float)
    if encoding == '32FC1':
        values /= 1000.
        values[0, 1] = np.nan
    row_bytes = image.width*dtype.itemsize
    image.step = row_bytes+4
    data = values.astype(dtype).tobytes()
    image.data = data[:row_bytes]+b'pad!'+data[row_bytes:]+b'pad!'
    info = CameraInfo()
    info.header.frame_id = image.header.frame_id
    info.width, info.height = image.width, image.height
    info.k = [2., 0., 1., 0., 2., 0., 0., 0., 1.]
    return image, info


@pytest.mark.parametrize('encoding', ['16UC1', '32FC1'])
@pytest.mark.parametrize('endian', [False, True])
def test_depth_projection_handles_padding_endian_and_invalid(encoding, endian):
    points, ratio, samples = depth_xyz(*make_depth(encoding, endian), stride=1)
    assert samples == 6
    assert ratio == pytest.approx(5/6)
    assert points[0] == pytest.approx([-0.5, 0., 1.])
    assert points[-1] == pytest.approx([2.5, 2.5, 5.])


def test_depth_stride_samples_full_image_and_empty_depth_is_invalid():
    image, info = make_depth()
    points, ratio, samples = depth_xyz(image, info, stride=2)
    assert samples == 2 and len(points) == 2 and ratio == 1.
    image.data = bytes(len(image.data))
    points, ratio, samples = depth_xyz(image, info, stride=1)
    assert ratio == 0. and len(points) == 0


def test_exact_tf_failure_is_propagated_no_latest_transform_fallback():
    class MissingTf:
        def lookup_transform(self, target, source, stamp):
            assert stamp.nanoseconds == 5_000_000_000
            raise LookupException('no transform at scan time')
    with pytest.raises(LookupException):
        transform_points(MissingTf(), np.zeros((1, 3)), 'camera_init', Time(sec=5))
