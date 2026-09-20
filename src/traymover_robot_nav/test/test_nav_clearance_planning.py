"""Exercise real NavFn offline; no ROS node, sensor, or robot is started."""

import ast
import math
import os
from pathlib import Path
import shutil
import subprocess

import pytest

from test_nav_clearance_config import load


PKG_ROOT = Path(__file__).resolve().parents[1]
CONFIGS = sorted((PKG_ROOT / 'config').glob('nav2_params*.yaml'))
CONFIGS.append(PKG_ROOT.parent / 'traymover_robot_nav2' / 'param' /
               'traymover_params' / 'param_traymover_diff.yaml')


@pytest.fixture(scope='module')
def navfn_harness(tmp_path_factory):
    prefixes = [Path(p) for p in os.environ.get('AMENT_PREFIX_PATH', '').split(':') if p]
    installation = next(
        ((prefix, include) for prefix in prefixes
         for include in [prefix / 'include', prefix / 'include/nav2_navfn_planner']
         if (include / 'nav2_navfn_planner/navfn.hpp').is_file() and
         (prefix / 'lib/libnav2_navfn_planner.so').is_file()), None)
    if installation is None:
        pytest.skip('Source the ROS overlay containing installed nav2_navfn_planner')
    prefix, include = installation
    compiler = shutil.which('c++')
    if compiler is None:
        pytest.skip('The offline NavFn regression requires a C++ compiler')
    binary = tmp_path_factory.mktemp('navfn') / 'clearance_harness'
    result = subprocess.run(
        [compiler, '-std=c++14', '-O2', '-I' + str(include),
         str(Path(__file__).with_name('navfn_clearance_harness.cpp')),
         str(prefix / 'lib/libnav2_navfn_planner.so'),
         '-Wl,-rpath,' + str(prefix / 'lib'), '-o', str(binary)],
        text=True, capture_output=True, timeout=60)
    assert result.returncode == 0, result.stdout + result.stderr
    return binary


def _configuration(path):
    params = load(path)
    costmap = params['global_costmap']['global_costmap']['ros__parameters']
    planner = params['planner_server']['ros__parameters']['GridBased']
    assert planner['plugin'] == 'nav2_navfn_planner/NavfnPlanner'
    assert planner['use_astar'] is False
    assert costmap['resolution'] == 0.05
    footprint = ast.literal_eval(costmap['footprint'])
    # Nav2 pads each rectangular footprint coordinate before finding inradius.
    padding = costmap.get('footprint_padding', 0.01)
    footprint = [[value + math.copysign(padding, value) for value in point]
                 for point in footprint]
    inscribed = min(abs(a[0] * b[1] - a[1] * b[0]) /
                    math.hypot(b[0] - a[0], b[1] - a[1])
                    for a, b in zip(footprint, footprint[1:] + footprint[:1]))
    return costmap['inflation_layer'], inscribed


def _plan(binary, scenario, inflation, inscribed):
    result = subprocess.run(
        [str(binary), scenario, str(inflation['inflation_radius']),
         str(inflation['cost_scaling_factor']), str(inscribed)],
        text=True, capture_output=True, timeout=10)
    assert result.returncode == 0, result.stdout + result.stderr
    count, clearance, center_error = map(float, result.stdout.split())
    assert count > 2
    return clearance, center_error


@pytest.mark.parametrize('config', CONFIGS, ids=lambda p: p.stem)
@pytest.mark.parametrize('scenario', ['left', 'right'])
def test_corner_path_keeps_more_obstacle_clearance(navfn_harness, config, scenario):
    inflation, inscribed = _configuration(config)
    previous = {'inflation_radius': 0.35, 'cost_scaling_factor': 3.0}
    old_clearance, _ = _plan(navfn_harness, scenario, previous, inscribed)
    clearance, _ = _plan(navfn_harness, scenario, inflation, inscribed)
    assert clearance >= old_clearance + 0.10, (old_clearance, clearance)
    assert clearance >= 0.50


@pytest.mark.parametrize('config', CONFIGS, ids=lambda p: p.stem)
def test_straight_path_moves_towards_corridor_center(navfn_harness, config):
    inflation, inscribed = _configuration(config)
    previous = {'inflation_radius': 0.35, 'cost_scaling_factor': 3.0}
    _, old_error = _plan(navfn_harness, 'straight', previous, inscribed)
    _, error = _plan(navfn_harness, 'straight', inflation, inscribed)
    assert error < old_error * 0.5, (old_error, error)
    assert error <= 0.10


@pytest.mark.parametrize('config', CONFIGS, ids=lambda p: p.stem)
def test_soft_inflation_keeps_narrow_corridor_traversable(navfn_harness, config):
    inflation, inscribed = _configuration(config)
    clearance, error = _plan(navfn_harness, 'narrow', inflation, inscribed)
    assert clearance > inscribed
    assert error <= 0.05
