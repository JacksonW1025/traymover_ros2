"""Clearance policy shared by map-based navigation modes (no hardware)."""

import math
from pathlib import Path

import pytest
import yaml


PKG = Path(__file__).resolve().parents[1]
PROFILES = sorted((PKG / 'config').glob('nav2_params*.yaml'))
LEGACY = (PKG.parent / 'traymover_robot_nav2' / 'param' /
          'traymover_params' / 'param_traymover_diff.yaml')


def load(path):
    return yaml.safe_load(path.read_text())


@pytest.mark.parametrize('path', PROFILES + [LEGACY], ids=lambda p: p.name)
def test_cost_field_reaches_corridor_center_without_blocking_narrow_passage(path):
    params = load(path)
    for name in ('local_costmap', 'global_costmap'):
        costmap = params[name][name]['ros__parameters']
        inflation = costmap['inflation_layer']
        footprint = yaml.safe_load(costmap['footprint'])
        inscribed = min(abs(coord) for vertex in footprint for coord in vertex)
        inscribed += costmap.get('footprint_padding', 0.01)

        def cost(distance):
            # Nav2 InflationLayer exponential cost, outside inscribed radius.
            if distance > inflation['inflation_radius']:
                return 0
            if distance <= inscribed:
                return 253
            return int(252 * math.exp(-inflation['cost_scaling_factor'] *
                                      (distance - inscribed)))

        # A 2 m corridor must have a gradient even at its center, not a broad
        # zero-cost interior where the shortest route can hug the inside bend.
        assert 0 < cost(1.0) < cost(0.70) < cost(0.50) < 253
        # Soft inflation must not turn a 0.9 m straight corridor into a wall.
        assert cost(0.45) < 253


@pytest.mark.parametrize('path', PROFILES, ids=lambda p: p.name)
def test_carrot_stays_before_corner_until_robot_is_close(path):
    follow = load(path)['controller_server']['ros__parameters']['FollowPath']
    assert follow['use_velocity_scaled_lookahead_dist']
    assert 0.40 <= follow['min_lookahead_dist'] <= follow['lookahead_dist']
    assert follow['lookahead_dist'] <= follow['max_lookahead_dist'] < 0.70
    for speed in (0.0, follow['desired_linear_vel'] / 2, follow['desired_linear_vel']):
        lookahead = min(follow['max_lookahead_dist'], max(
            follow['min_lookahead_dist'], speed * follow['lookahead_time']))
        # On an L path with a bend 0.7 m ahead, a carrot before the bend has
        # y=0 and RPP curvature=0. The old 1.1 m minimum aimed beyond it.
        assert lookahead < 0.70
    assert follow['use_rotate_to_heading']
    assert 0 < follow['rotate_to_heading_min_angle'] <= math.radians(25)
    assert not follow['allow_reversing']


def test_legacy_mppi_uses_same_distance_conversion_as_inflation_layer():
    params = load(LEGACY)
    critic = params['controller_server']['ros__parameters']['FollowPath']['ObstaclesCritic']
    inflation = params['local_costmap']['local_costmap']['ros__parameters']['inflation_layer']
    for key in ('inflation_radius', 'cost_scaling_factor'):
        assert critic[key] == inflation[key]
