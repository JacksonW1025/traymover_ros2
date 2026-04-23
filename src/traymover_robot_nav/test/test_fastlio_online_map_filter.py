#!/usr/bin/env python3
from pathlib import Path
import importlib.util
import sys

import numpy as np


MODULE_PATH = Path(__file__).resolve().parents[1] / 'scripts' / 'fastlio_online_map_filter.py'
SPEC = importlib.util.spec_from_file_location('fastlio_online_map_filter', MODULE_PATH)
map_filter = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = map_filter
SPEC.loader.exec_module(map_filter)


def test_stable_voxel_map_promotes_repeated_points():
    voxel_map = map_filter.StableVoxelMap(
        voxel_size=0.20,
        min_hits=3,
        min_observation_span_sec=0.20,
        candidate_ttl_sec=5.0,
    )

    frame = np.array([
        [1.00, 0.00, 0.00, 10.0],
        [1.05, 0.02, 0.00, 12.0],
    ], dtype=np.float32)
    voxel_map.update_scan(frame, 0.00)
    voxel_map.update_scan(frame, 0.12)
    voxel_map.update_scan(frame, 0.32)

    stable = voxel_map.stable_cloud()

    assert stable.shape == (1, 4)
    assert np.allclose(stable[0, :3], [1.025, 0.01, 0.0], atol=1e-3)
    assert stable[0, 3] > 10.0


def test_stable_voxel_map_prunes_transient_candidates():
    voxel_map = map_filter.StableVoxelMap(
        voxel_size=0.20,
        min_hits=3,
        min_observation_span_sec=0.20,
        candidate_ttl_sec=0.50,
    )

    transient = np.array([[2.0, 0.0, 0.0, 1.0]], dtype=np.float32)
    voxel_map.update_scan(transient, 0.00)
    voxel_map.update_scan(np.empty((0, 4), dtype=np.float32), 1.00)

    assert voxel_map.total_voxels() == 0
    assert voxel_map.stable_cloud().size == 0


def test_body_box_exclusion_removes_points_behind_robot():
    pose = map_filter.PoseSample(
        stamp_sec=1.0,
        translation=np.array([0.0, 0.0, 0.0], dtype=np.float32),
        rotation_body_to_world=np.eye(3, dtype=np.float32),
    )
    cloud = np.array([
        [-0.80, 0.00, 1.00, 1.0],   # inside rear box -> removed
        [1.50, 0.00, 1.00, 2.0],    # in front -> kept
    ], dtype=np.float32)

    filtered = map_filter.apply_body_box_exclusion(
        points_xyzi_world=cloud,
        pose=pose,
        pose_timeout_sec=0.30,
        stamp_sec=1.05,
        min_x=-1.25,
        max_x=-0.35,
        half_width=0.90,
        min_z=-0.50,
        max_z=2.00,
    )

    assert filtered.shape == (1, 4)
    assert np.allclose(filtered[0], cloud[1])


def test_write_binary_pcd_writes_header_and_payload(tmp_path):
    cloud = np.array([
        [1.0, 2.0, 3.0, 4.0],
        [5.0, 6.0, 7.0, 8.0],
    ], dtype=np.float32)
    pcd_path = tmp_path / 'filtered_map.pcd'

    map_filter.write_binary_pcd(pcd_path, cloud)

    data = pcd_path.read_bytes()
    assert b'FIELDS x y z intensity' in data
    assert b'POINTS 2' in data
    assert data.endswith(cloud.tobytes())
