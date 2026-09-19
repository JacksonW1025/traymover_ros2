"""Offline paired-map safety and coordinate contract tests."""
import hashlib
import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest
import yaml
from PIL import Image

SCRIPT = Path(__file__).resolve().parents[3] / 'scripts/option20_prepare_map.py'
spec = importlib.util.spec_from_file_location('option20_prepare_map', SCRIPT)
prepare_map = importlib.util.module_from_spec(spec)
spec.loader.exec_module(prepare_map)


def sample_points():
    return np.array([[0.05, .05, 0.], [.15, .05, 0.],
                     [1.05, 1.05, 1.2], [1.05, 1.05, 1.7],
                     [2.05, 2.05, 2.1], [3.05, .05, .5]])


def test_unknown_stays_unknown_and_localization_has_no_inflation():
    loc, nav, _, stats = prepare_map.build_grids(sample_points())
    assert loc.shape == nav.shape
    assert stats['localization_occupied_cells'] == 2
    assert stats['navigation_occupied_cells'] == 3
    assert (nav == prepare_map.UNKNOWN).sum() > (nav == prepare_map.FREE).sum()
    assert np.all(loc[nav == prepare_map.UNKNOWN] == prepare_map.FREE)
    assert stats['localization_band_points'] == [1, 1, 1]


def test_ground_fill_is_bounded_and_does_not_cross_walls():
    ground = np.zeros((9, 9), dtype=bool)
    ground[4, 3] = True
    obstacle = np.zeros_like(ground)
    obstacle[:, 4] = True
    filled = prepare_map.bounded_ground_fill(ground, obstacle, 2)
    assert filled[4, 1]
    assert not filled[4, 0]
    assert not filled[:, 4:].any()
    assert not filled[0].any()


def test_transform_is_rigid_and_levels_plane():
    plane = [.01, -.015, -1.6]
    xy = np.array([[0., 0.], [10., 30.], [-5., 400.]])
    points = np.column_stack((xy, xy @ plane[:2] + plane[2]))
    transform = prepare_map.ground_transform(plane)
    assert np.allclose(transform[:3, :3] @ transform[:3, :3].T, np.eye(3))
    assert np.isclose(np.linalg.det(transform[:3, :3]), 1.)
    levelled = prepare_map.apply_transform(points, transform)
    assert np.allclose(levelled[:, 2], 0.)
    assert np.isclose(np.linalg.norm(points[1] - points[0]),
                      np.linalg.norm(levelled[1] - levelled[0]))


def test_preparation_keeps_source_and_both_map_coordinates(tmp_path):
    o3d = pytest.importorskip('open3d')
    pcd = tmp_path / 'source.pcd'
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(sample_points())
    assert o3d.io.write_point_cloud(str(pcd), cloud)
    before = hashlib.sha256(pcd.read_bytes()).hexdigest()
    output = tmp_path / 'maps'
    manifest = prepare_map.prepare(pcd, output, floor_z=0.)
    assert hashlib.sha256(pcd.read_bytes()).hexdigest() == before
    loc = yaml.safe_load((output / 'localization.yaml').read_text())
    nav = yaml.safe_load((output / 'navigation.yaml').read_text())
    assert loc['origin'] == nav['origin'] == manifest['origin']
    assert loc['resolution'] == nav['resolution'] == .10
    assert Image.open(output / loc['image']).size == Image.open(output / nav['image']).size
    assert np.any(np.asarray(Image.open(output / nav['image'])) == prepare_map.UNKNOWN)
    assert json.loads((output / 'manifest.json').read_text())['pcd_path'] == str(pcd)


@pytest.mark.parametrize('points', [[], [[np.nan, 0, 0]], [[0, 0, 0]]])
def test_invalid_inputs_rejected(points):
    with pytest.raises(ValueError):
        prepare_map.build_grids(points)


@pytest.mark.parametrize('extent', [1.e7, 1.e100])
def test_huge_map_rejected_before_allocation(extent):
    points = sample_points()
    points[-1, 0] = extent
    with pytest.raises(ValueError, match='cell limit'):
        prepare_map.build_grids(points)


def test_automatic_plane_manifest_serializes_and_aligns_maps(tmp_path):
    o3d = pytest.importorskip('open3d')
    x, y = np.meshgrid(np.arange(0., 25., .25), np.arange(0., 25., .25))
    floor = .01 * x - .015 * y - 1.6
    ground = np.column_stack((x.ravel(), y.ravel(), floor.ravel()))
    structure = ground[::20].copy()
    structure[:, 2] += 1.7
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(np.vstack((ground, structure)))
    pcd = tmp_path / 'inclined.pcd'
    assert o3d.io.write_point_cloud(str(pcd), cloud)
    result = prepare_map.prepare(pcd, tmp_path / 'maps')
    assert result['ground_diagnostics']['method'] == 'validated-low-surface-plane'
    assert np.allclose(result['ground_plane'], [.01, -.015, -1.6], atol=.01)
    assert result['statistics']['below_ground_fraction'] == 0.
    assert (tmp_path / 'maps/manifest.json').is_file()
