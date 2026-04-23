#!/usr/bin/env python3
from pathlib import Path
import importlib.util

import numpy as np


MODULE_PATH = Path(__file__).resolve().parents[1] / 'scripts' / 'pcd2pgm.py'
SPEC = importlib.util.spec_from_file_location('pcd2pgm', MODULE_PATH)
pcd2pgm = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(pcd2pgm)


def test_remove_small_components_drops_singletons():
    occ = np.zeros((6, 6), dtype=bool)
    occ[0, 0] = True
    occ[2:4, 2:4] = True

    filtered = pcd2pgm.remove_small_components(occ, min_region_cells=2)

    assert not filtered[0, 0]
    assert filtered[2:4, 2:4].all()


def test_occupancy_from_points_requires_multiple_hits_per_cell():
    slab = np.array([
        [0.01, 0.01],
        [0.02, 0.02],
        [0.03, 0.03],
        [0.26, 0.26],
    ], dtype=float)

    occ, *_ = pcd2pgm.occupancy_from_points(
        slab_xy=slab,
        resolution=0.1,
        padding=0.0,
        min_points=3,
    )

    assert occ.sum() == 1


def test_estimate_floor_z_ignores_deep_outliers():
    z = np.concatenate([
        np.full(200, -5.0),   # bad low outliers
        np.full(5000, 0.12),  # real floor
        np.full(1000, 1.80),  # shelves / ceiling clutter
    ])

    floor_z, method = pcd2pgm.estimate_floor_z(
        z_values=z,
        fallback_quantile=0.02,
        low_quantile=0.005,
        high_quantile=0.30,
        bin_size=0.05,
    )

    assert method == 'histogram-mode'
    assert abs(floor_z - 0.12) < 0.03
