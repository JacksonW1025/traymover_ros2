# Offline Localization Experiments

This directory contains offline-only experiments for the Traymover
interference-sensitive localization report. The code and generated data stay
under `tmp/` and do not modify production ROS packages.

## Scope

These experiments do not require the robot, lidar, IMU, serial chassis, or a
running Nav2 stack. They are intended to support report claims about:

- PCD map statistics relevant to NDT sensitivity.
- Synthetic point-cloud interference indicators such as rear-box removal,
  half-FOV masking, far-return weighting, and ghost-point injection scale.
- NDT gate behavior under deterministic synthetic scenarios.

They do not replace real A/B/C/D/E vehicle experiments. Any fitness, drift, or
abort numbers produced here must be described as offline or synthetic data.

## Files

- `scripts/run_offline_experiments.py`: standalone Python 3 runner.
- `data/pcd_interference_stats.csv`: statistics for existing PCD files.
- `data/gate_sweep_attempts.csv`: per-attempt synthetic NDT gate decisions.
- `data/gate_sweep_summary.csv`: accepted/rejected counts by scenario and
  parameter profile.
- `data/experiment_summary.md`: short generated summary for report drafting.
- `data/metadata.json`: run timestamp, input files, and experiment constants.

## Run

From the workspace root:

```bash
python3 tmp/offline_localization_experiments/scripts/run_offline_experiments.py
```

Optional examples:

```bash
python3 tmp/offline_localization_experiments/scripts/run_offline_experiments.py \
  --pcd src/traymover_robot_slam/FAST_LIO/PCD/traymover_20260527_111059.pcd

python3 tmp/offline_localization_experiments/scripts/run_offline_experiments.py \
  --max-points 100000
```

`--max-points` samples each PCD deterministically for faster exploratory runs.
Leave it unset for exact per-file point counts.

## Archived results and portability

The checked-in data records the 2026-06-11 run, not a new vehicle experiment.
Original PCD inputs are not distributed with these results. The synthetic gate
sweeps are deterministic; full PCD statistics require the listed source maps.
Use `--pcd /absolute/path/to/map.pcd --output-dir /tmp/localization-results`
for external maps and to preserve the archived output files.

External PCD paths are recorded as absolute paths in generated metadata; review
them before sharing newly generated reports. Repository-local paths stay relative.

Regression test:

```bash
python3 -m pytest tmp/offline_localization_experiments/test
```
