#!/usr/bin/env python3
from pathlib import Path
import argparse
import importlib.util


MODULE_PATH = Path(__file__).resolve().parents[1] / 'scripts' / 'pcd_clean.py'
SPEC = importlib.util.spec_from_file_location('pcd_clean', MODULE_PATH)
pcd_clean = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(pcd_clean)


def make_parser_defaults():
    parser = argparse.ArgumentParser()
    parser.add_argument('--sor-nb', type=int, default=20)
    parser.add_argument('--sor-std', type=float, default=2.0)
    parser.add_argument('--trajectory-corridor', action='store_true')
    parser.add_argument('--trajectory-log', default='')
    parser.add_argument('--trajectory-format', default='auto')
    parser.add_argument('--corridor-width', type=float, default=0.80)
    parser.add_argument('--corridor-xy-res', type=float, default=0.20)
    parser.add_argument('--corridor-sample-step', type=float, default=0.10)
    parser.add_argument('--corridor-min-pts', type=int, default=120)
    parser.add_argument('--floating', action='store_true')
    parser.add_argument('--dbscan', action='store_true')
    parser.add_argument('--person-z-max', type=float, default=1.8)
    parser.add_argument('--person-bbox-max', type=float, default=0.8)
    parser.add_argument('--person-points-max', type=int, default=800)
    parser.add_argument('--isolation-min', type=float, default=0.5)
    parser.add_argument('--light', action='store_true')
    parser.add_argument('--aggressive', action='store_true')
    return parser


def test_light_preset_keeps_cleaning_gentle():
    parser = make_parser_defaults()
    args = parser.parse_args(['--light'])

    pcd_clean.apply_light_preset(args, parser)

    assert args.sor_nb == 12
    assert args.sor_std == 2.8
    assert not args.floating
    assert not args.dbscan


def test_aggressive_preset_still_enables_extra_filters():
    parser = make_parser_defaults()
    args = parser.parse_args(['--aggressive'])

    pcd_clean.apply_aggressive_preset(args, parser)

    assert args.sor_nb == 30
    assert args.sor_std == 1.5
    assert args.floating
    assert args.dbscan


def test_load_trajectory_xy_parses_fastlio_pos_log(tmp_path):
    path = tmp_path / 'pos_log.txt'
    path.write_text(
        '0.0 0 0 0 1.0 2.0 3.0 0 0 0\n'
        '0.1 0 0 0 1.5 2.5 3.0 0 0 0\n'
    )

    xy = pcd_clean.load_trajectory_xy(path, fmt='auto')

    assert xy.shape == (2, 2)
    assert list(xy[0]) == [1.0, 2.0]
    assert list(xy[1]) == [1.5, 2.5]


def test_build_corridor_cells_and_filter_drop_sparse_points(tmp_path):
    import numpy as np
    import open3d as o3d

    traj = tmp_path / 'traj_xy.txt'
    traj.write_text('0.0 0.0\n1.0 0.0\n')

    cloud = o3d.geometry.PointCloud()
    pts = np.array([
        [0.10, 0.05, 0.0],  # sparse speckle inside corridor -> drop
        [0.20, -0.05, 0.0],
        [2.0, 2.0, 0.0],    # outside corridor -> keep
        [2.1, 2.0, 0.0],
        [2.2, 2.0, 0.0],
    ], dtype=float)
    cloud.points = o3d.utility.Vector3dVector(pts)

    class Args:
        trajectory_log = str(traj)
        trajectory_format = 'xy'
        corridor_sample_step = 0.10
        corridor_xy_res = 0.20
        corridor_width = 0.30
        corridor_min_pts = 3

    filtered = pcd_clean.trajectory_corridor_filter(cloud, Args())
    out = np.asarray(filtered.points)

    assert len(out) == 3
    assert np.all(out[:, 0] > 1.5)
