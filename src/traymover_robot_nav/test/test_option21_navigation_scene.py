"""Protect occupied pillars, footprint collision and the obstacle-aware profile."""
from pathlib import Path
import sys

import numpy as np
from PIL import Image
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'scripts'))
import option21_navigation_scene as scene


def test_every_pillar_is_solid_in_the_navigation_map(tmp_path):
    image = tmp_path / 'scene.pgm'
    Image.fromarray(np.full((280, 360), 254, dtype=np.uint8)).save(image)
    prefix = tmp_path / 'scene'
    prefix.with_suffix('.yaml').write_text(yaml.safe_dump({
        'image': image.name, 'resolution': .05, 'origin': [-9., -7., 0.]}))
    result = scene.repair_simulation_map(prefix)
    grid = np.asarray(Image.open(image))
    def value(x, y):
        return grid[279-int((y+7)/.05), int((x+9)/.05)]
    assert result['solid_pillars'] == 3
    assert result['occupied_cells'] > 10000
    assert value(0., 0.) == 254
    assert value(6., -4.4) == 254
    assert value(8.5, 0.) == 0
    for x, y, width, _ in scene.ROOM_PILLARS:
        for dx, dy in ((0, 0), (width/3, width/3), (-width/3, -width/3)):
            assert value(x+dx, y+dy) == 0


def test_collision_uses_rotated_footprint_not_just_robot_center():
    assert scene.collides([3.4, -3.1, 0.])  # Center is outside pillar 3.
    assert scene.collides([3.3, -3.1, np.pi/4])
    assert not scene.collides([3., -3.1, 0.])
    assert scene.collides([0., 5.8, 0.])
    assert not scene.collides([0., 5.4, 0.])


def test_footprint_clearance_matches_simple_axis_aligned_geometry():
    assert scene.clearance([3., -3.1, 0.]) == pytest.approx(.2)
    assert scene.clearance([4.2, -3.1, 0.]) == 0.


def test_direct_goal_line_hits_pillar_but_detour_does_not():
    direct = scene.path_metrics([[0, 0], [6, -4.4]])
    detour = scene.path_metrics([[0, 0], [5.4, -1.8], [6, -4.4]])
    assert direct['enters_solid_pillar']
    assert not detour['enters_solid_pillar']
    assert detour['max_deviation_from_straight_m'] > .45
    assert detour['length_m'] > direct['length_m']


def test_full_profile_isolated_from_field_navigation_defaults():
    baseline = yaml.safe_load((ROOT / 'src/traymover_robot_nav/config/nav2_params.yaml').read_text())
    old = yaml.safe_dump(baseline)
    profile = scene.full_navigation_parameters(baseline)
    assert yaml.safe_dump(baseline) == old
    for name in ('global_costmap', 'local_costmap'):
        params = profile[name][name]['ros__parameters']
        assert params['plugins'] == ['static_layer', 'obstacle_layer', 'inflation_layer']
        assert params['obstacle_layer']['lidar']['marking']
        assert params['obstacle_layer']['lidar']['clearing']
    controller = profile['controller_server']['ros__parameters']['FollowPath']
    assert controller['plugin'] == 'dwb_core::DWBLocalPlanner'
    assert 'ObstacleFootprint' in controller['critics']
    assert controller['publish_local_plan']
    assert 'backup' in profile['behavior_server']['ros__parameters']['behavior_plugins']
