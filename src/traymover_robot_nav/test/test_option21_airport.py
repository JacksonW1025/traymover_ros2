"""Validate the paired airport experiment's geometry and sensor assumptions."""
import importlib.util
from pathlib import Path

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location('airport_benchmark', ROOT / 'scripts/option21_airport_benchmark.py')
AIRPORT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(AIRPORT)


def test_store_change_is_local_and_major_walls_are_retained():
    prior, live, removed = AIRPORT.hall_maps()
    assert removed / len(prior) < .01
    assert prior[:, 0].min() == -40 and prior[:, 0].max() == 40
    assert prior[:, 1].min() == -25 and prior[:, 1].max() == 25
    assert np.any((live[:, 1] == 28) & (live[:, 0] > 8) & (live[:, 0] < 20))
    cloth = (live[:, 1] == 25) & (live[:, 0] > 8) & (live[:, 0] < 20) & (live[:, 2] < 4)
    assert not cloth.any()
    assert np.array_equal(prior[prior[:, 1] == -25], live[:len(prior)-removed][live[:len(prior)-removed, 1] == -25])


def test_motion_and_imu_are_identical_between_environment_cases():
    a = AIRPORT.AirportModel('static', 101, .15)
    b = AIRPORT.AirportModel('changed', 101, .15)
    for step in range(3):
        assert np.array_equal(a.motion(step)[0], b.motion(step)[0])
        assert a.imu_messages(step) == b.imu_messages(step)
    samples = a.imu_messages(1)
    assert all(9.7 < msg.linear_acceleration.z < 9.9 for msg in samples)
    stamps = [msg.header.stamp.sec+msg.header.stamp.nanosec/1e9 for msg in samples]
    assert np.diff(stamps) == pytest.approx(np.full(9, .005))


def test_xyzirt_snapshot_has_deterministic_padding_and_correct_end_stamp():
    points = np.array([[2., 1., .5], [3., -.4, -.2]], dtype=np.float32)
    stamp = AIRPORT.stamp_at(12.35)
    a = AIRPORT.AirportModel.raw_cloud(points, stamp)
    b = AIRPORT.AirportModel.raw_cloud(points, stamp)
    assert a.data == b.data
    assert a.point_step == 24
    assert set(f.name for f in a.fields) == {'x', 'y', 'z', 'time', 'ring', 'intensity'}
    row = np.frombuffer(a.data, dtype=np.uint8).reshape(-1, 24)
    assert not row[:, 22:].any()
    end = a.header.stamp.sec + a.header.stamp.nanosec/1e9 + 1e-6
    assert end == pytest.approx(12.35, abs=1e-9)


def test_linear_nearest_selection_preserves_the_original_sensor_samples():
    rng=np.random.default_rng(1)
    bins=rng.integers(0,1024,20000)
    # Include equal distances so tie handling must keep the original first point.
    distance=rng.integers(1,500,20000).astype(float)
    order=np.lexsort((distance,bins))
    _,first=np.unique(bins[order],return_index=True)
    assert np.array_equal(AIRPORT.sim.nearest_angular_indices(bins,distance),order[first])


def test_precomputed_scans_preserve_identical_sensor_values():
    model=AIRPORT.AirportModel('changed',101,.1)
    model.prepare_scans()
    rng=np.random.default_rng(101)
    for step,cached in enumerate(model.scan_cache):
        pose,_=model.motion(step)
        uncached=AIRPORT.sim.make_scan(model.live_points(step*.05),pose,rng,
                                      noise=.02,max_range=model.max_range)
        assert np.array_equal(cached,uncached)


def test_unmatched_sensor_input_cannot_be_presented_as_a_fair_comparison(tmp_path):
    data = [dict(experiment={'case': 'changed'}, seed=101, odometry_source=source,
                 input_sensor_sha256=source, global_current_rmse_m=.1)
            for source in ['wheel', 'fastlio']]
    with pytest.raises(RuntimeError, match='Sensor inputs differ'):
        AIRPORT.compare(tmp_path, data)
