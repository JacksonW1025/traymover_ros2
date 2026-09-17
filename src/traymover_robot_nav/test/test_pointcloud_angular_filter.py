#!/usr/bin/env python3
from pathlib import Path
import importlib.util
import struct
import sys

import math
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField


MODULE_PATH = Path(__file__).resolve().parents[1] / 'scripts' / 'pointcloud_angular_filter.py'
SPEC = importlib.util.spec_from_file_location('pointcloud_angular_filter', MODULE_PATH)
angular_filter = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = angular_filter
SPEC.loader.exec_module(angular_filter)


POINT_STEP = 22
FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name='time', offset=16, datatype=PointField.FLOAT32, count=1),
    PointField(name='ring', offset=20, datatype=PointField.UINT16, count=1),
]


def make_cloud(points):
    data = bytearray()
    for x, y, z, intensity, time, ring in points:
        data.extend(struct.pack('<fffffH', x, y, z, intensity, time, ring))

    msg = PointCloud2()
    msg.header.frame_id = 'laser'
    msg.height = 1
    msg.width = len(points)
    msg.fields = FIELDS
    msg.is_bigendian = False
    msg.point_step = POINT_STEP
    msg.row_step = POINT_STEP * len(points)
    msg.is_dense = True
    msg.data = bytes(data)
    return msg


def unpack_points(msg):
    return [
        struct.unpack_from('<fffffH', msg.data, i * msg.point_step)
        for i in range(msg.width)
    ]


def test_filter_keeps_front_half_and_preserves_raw_fields():
    msg = make_cloud([
        (1.0, 0.0, 0.0, 10.0, 0.01, 1),
        (0.0, 1.0, 0.0, 11.0, 0.02, 2),
        (-1.0, 0.0, 0.0, 12.0, 0.03, 3),
        (0.0, -1.0, 0.0, 13.0, 0.04, 4),
        (1.0, 1.0, 0.0, 14.0, 0.05, 5),
    ])

    out, removed = angular_filter.filter_cloud(msg, -math.pi / 2.0, math.pi / 2.0)

    assert removed == 1
    assert out.height == 1
    assert out.width == 4
    assert out.fields == msg.fields
    assert out.point_step == POINT_STEP
    assert out.row_step == POINT_STEP * 4
    assert [round(point[3]) for point in unpack_points(out)] == [10, 11, 13, 14]
    assert [point[5] for point in unpack_points(out)] == [1, 2, 4, 5]


def test_wrap_angle_range_keeps_rear_half():
    msg = make_cloud([
        (1.0, 0.0, 0.0, 10.0, 0.01, 1),
        (-1.0, 1.0, 0.0, 11.0, 0.02, 2),
        (-1.0, -1.0, 0.0, 12.0, 0.03, 3),
    ])

    out, removed = angular_filter.filter_cloud(msg, 3.0 * math.pi / 4.0, -3.0 * math.pi / 4.0)

    assert removed == 1
    assert out.width == 2
    assert [point[5] for point in unpack_points(out)] == [2, 3]


def test_compacts_organized_cloud_with_row_padding():
    msg = make_cloud([
        (1.0, 0.0, 0.0, 10.0, 0.01, 1),
        (-1.0, 0.0, 0.0, 11.0, 0.02, 2),
        (1.0, 1.0, 0.0, 12.0, 0.03, 3),
        (-1.0, -1.0, 0.0, 13.0, 0.04, 4),
    ])
    row_padding = b'pad!'
    msg.height = 2
    msg.width = 2
    msg.row_step = POINT_STEP * 2 + len(row_padding)
    raw = bytes(msg.data)
    msg.data = (
        raw[:POINT_STEP * 2] + row_padding +
        raw[POINT_STEP * 2:] + row_padding
    )

    out, removed = angular_filter.filter_cloud(msg, -math.pi / 2.0, math.pi / 2.0)

    assert removed == 2
    assert out.width == 2
    assert np.frombuffer(out.data, dtype=np.uint8).size == POINT_STEP * 2
    assert [point[5] for point in unpack_points(out)] == [1, 3]
