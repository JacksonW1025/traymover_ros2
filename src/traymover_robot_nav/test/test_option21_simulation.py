"""Check independent synthetic geometry/plant assumptions used by the experiment."""
import importlib.util
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial import cKDTree


ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location('option21_simulation', ROOT / 'scripts/option21_simulation.py')
SIM = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SIM)


def test_plant_has_correct_distance_and_turn_units():
    assert SIM.plant_step(np.zeros(3), .4, 0, 2) == pytest.approx([.8, 0, 0])
    assert SIM.plant_step(np.zeros(3), 0, -.5, 2) == pytest.approx([0, 0, -1])


def test_noise_free_scan_is_on_the_independent_world_surfaces():
    world = SIM.room_points()
    pose = np.array([1.2, -.7, .8])
    scan = SIM.make_scan(world, pose, np.random.default_rng(1), noise=0)
    c, s = np.cos(pose[2]), np.sin(pose[2])
    restored = scan @ np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])
    restored += [pose[0], pose[1], .2]
    distance, _ = cKDTree(world).query(restored)
    assert len(scan) > 1000
    assert distance.max() < 2e-6


def test_stale_tf_counts_as_current_position_error(tmp_path):
    directory = tmp_path / 'dropout'
    directory.mkdir()
    (directory / 'metrics.csv').write_text(
        't,truth_x,truth_y,localized_x,localized_y,localized_age_s,odom_age_s\n'
        '5,0,0,0,0,0,0\n6,1,0,0,0,1,1\n')
    result = SIM.augment_results(tmp_path, {'scenario': 'dropout', 'acceptance_checks': {}})
    assert result['localized_current_max_error_m'] == 1
    assert result['localized_current_rmse_m'] == pytest.approx(np.sqrt(.5))
    assert result['localized_fresh_fraction'] == .5
    assert not result['passed']
