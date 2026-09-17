# Navigation and localization branch

This branch preserves the Traymover navigation changes reviewed on 2026-09-17.

## Reproducing the source

```bash
git submodule update --init --recursive
```

The modified FAST-LIO and lidar_localization_ros2 histories are stored in this
repository on `deps/fastlio-traymover-20260917` and
`deps/lidar-localization-20260917`. The gitlinks pin exact commits; these branches
are dependency source trees, not complete Traymover workspaces. Do not merge the
dependency branches into the workspace branch. Other submodules retain their
existing pinned revisions and upstream URLs.

## Changes and deployment limits

- Bag replay preserves point fields while selecting the front 180 degrees and
  reports recording duration, size, and LiDAR/IMU counts.
- Option 16 projects absolute map-frame heights 0.5–7.0 m at 0.10 m resolution.
  This is a scene-specific preset, not a floor-relative height setting.
- NDT uses the paired localization implementation, including adaptive target
  resolution, odometry pose prediction, correction gates, and TF updates.
- FAST-LIO avoids saving an empty accumulated cloud.
- `pcd2pgm_compare.py` is an optional diagnostic tool.

The FAST-LIO YAML files retain the robot-specific map output path
`/home/wheeltec/traymover_ros2/src/traymover_robot_slam/FAST_LIO/PCD/traymover.pcd`.
Adjust `map_file_path` when deploying to another workspace. Sensor extrinsics
and option 16 height limits also require deployment-specific verification.

Local Livox message-only simplification, COLCON_IGNORE markers, maps, rosbag
recordings, and RViz window state are excluded. A fresh full-workspace build can
therefore require upstream Livox SDK and simulator dependencies that this
robot's local workspace bypasses. Selected-package tests do not establish a
fresh full-workspace build or validate vehicle behavior.
