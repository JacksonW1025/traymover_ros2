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


def test_resolve_z_bounds_supports_absolute_and_floor_relative_modes():
    assert pcd2pgm.resolve_z_bounds('absolute', 0.5, 7.0) == (0.5, 7.0)
    assert pcd2pgm.resolve_z_bounds('floor', 0.1, 2.2, 0.5) == (0.6, 2.7)


def tilted_cloud():
    rng = np.random.default_rng(19)
    xy = rng.uniform(0.0, 40.0, (20000, 2))
    z = 0.03 * xy[:, 0] - 0.04 * xy[:, 1] - 1.2
    return np.column_stack((xy, z))


def test_plane_rejects_ground_and_keeps_normal_height_obstacles_without_rotation():
    ground = tilted_cloud()
    original = ground.copy()
    plane, diagnostics = pcd2pgm.fit_ground_plane(ground)
    np.testing.assert_allclose(plane, [0.03, -0.04, -1.2], atol=1e-9)
    np.testing.assert_array_equal(ground, original)
    assert diagnostics['inlier_count'] >= 12
    assert np.max(np.abs(pcd2pgm.height_above_plane(ground, plane))) < 1e-9
    obstacle = ground[:100].copy()
    obstacle[:, 2] += np.sqrt(1 + 0.03**2 + 0.04**2)
    np.testing.assert_allclose(pcd2pgm.height_above_plane(obstacle, plane), 1.0)
    again, _ = pcd2pgm.fit_ground_plane(ground)
    np.testing.assert_array_equal(plane, again)


def test_plane_rejects_roof_with_sparse_lower_ground():
    import pytest
    ground = tilted_cloud()
    roof = ground.copy()
    roof[:, 2] += 3.0
    # Lower returns occupy less than the anchor quantile but span the map.
    combined = np.concatenate((roof, ground[::15]))
    with pytest.raises(ValueError, match='lower observations'):
        pcd2pgm.fit_ground_plane(combined)


def test_plane_rejects_invalid_parameters_and_weak_support():
    import pytest
    ground = tilted_cloud()
    for kwargs in ({'cell_size': 0}, {'distance_threshold': np.nan},
                   {'max_tilt_deg': 90}, {'max_samples': 0}, {'iterations': 0}):
        with pytest.raises(ValueError, match='parameters'):
            pcd2pgm.fit_ground_plane(ground, **kwargs)
    for points in (np.empty((0, 3)), np.full((30, 3), np.nan), ground[:20]):
        with pytest.raises(ValueError):
            pcd2pgm.fit_ground_plane(points)
    line = ground.copy()
    line[:, 1] = 0
    with pytest.raises(ValueError):
        pcd2pgm.fit_ground_plane(line)


def test_plane_cli_writes_map_with_original_xy(tmp_path, monkeypatch):
    import sys
    import yaml
    ground = tilted_cloud()
    obstacles = ground[::10].copy()
    obstacles[:, 2] += 1.0
    points = np.concatenate((ground, obstacles))
    cloud = pcd2pgm.o3d.geometry.PointCloud()
    cloud.points = pcd2pgm.o3d.utility.Vector3dVector(points)
    source = tmp_path / 'input.pcd'
    assert pcd2pgm.o3d.io.write_point_cloud(str(source), cloud)
    output = tmp_path / 'map'
    monkeypatch.setattr(sys, 'argv', [
        str(MODULE_PATH), '--pcd', str(source), '--out', str(output),
        '--z-reference', 'plane', '--resolution', '0.5', '--dilate', '0',
        '--min-region-cells', '1',
    ])
    pcd2pgm.main()
    assert output.with_suffix('.pgm').exists()
    metadata = yaml.safe_load(output.with_suffix('.yaml').read_text())
    np.testing.assert_allclose(metadata['origin'][:2], obstacles[:, :2].min(axis=0) - 1,
                               atol=1e-5)


def test_plane_cli_failure_does_not_fallback_or_write(tmp_path, monkeypatch):
    import sys
    import pytest
    cloud = pcd2pgm.o3d.geometry.PointCloud()
    cloud.points = pcd2pgm.o3d.utility.Vector3dVector(tilted_cloud()[:20])
    source = tmp_path / 'input.pcd'
    assert pcd2pgm.o3d.io.write_point_cloud(str(source), cloud)
    output = tmp_path / 'map'
    monkeypatch.setattr(sys, 'argv', [
        str(MODULE_PATH), '--pcd', str(source), '--out', str(output),
        '--z-reference', 'plane',
    ])
    with pytest.raises(SystemExit, match='12 populated'):
        pcd2pgm.main()
    assert not output.with_suffix('.pgm').exists()
    assert not output.with_suffix('.yaml').exists()


def test_plane_handles_large_map_origin():
    ground = tilted_cloud()
    ground[:, :2] += [1e8, -2e8]
    plane, _ = pcd2pgm.fit_ground_plane(ground)
    np.testing.assert_allclose(plane[:2], [0.03, -0.04], atol=1e-8)
    assert np.max(np.abs(pcd2pgm.height_above_plane(ground, plane))) < 1e-7


def test_plane_tolerates_shallow_lower_returns():
    ground = tilted_cloud()
    lower = ground[::15].copy()
    lower[:, 2] -= 0.20
    plane, _ = pcd2pgm.fit_ground_plane(np.concatenate((ground, lower)))
    assert np.max(np.abs(pcd2pgm.height_above_plane(ground, plane))) < 0.05
