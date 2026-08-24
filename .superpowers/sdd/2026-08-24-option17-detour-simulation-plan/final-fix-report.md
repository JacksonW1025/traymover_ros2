# Whole-branch simulation fix report

## Status

DONE. The final review findings are addressed without changing the hardware
URDF or the option 1–17 interfaces.

## Fixes

- Moved the Gazebo Sim DiffDrive system plugin to model scope in the simulation
  xacro. Both simulation wheel joints now use the same positive axis, so a
  forward command produces straight motion. `/cmd_vel` and `/odom` remain the
  configured Gazebo topics.
- Replaced the unsupported sensor `frame_id` SDF extension with Gazebo
  Sensors' `gz_frame_id=laser`. The ROS `robot_state_publisher` publishes the
  static `base_link -> laser` transform, while `sim_odom_tf` publishes
  `odom -> base_link` from bridged odometry.
- Added the ROS-to-Gazebo `/cmd_vel` Twist bridge; retained the existing
  Gazebo-to-ROS `/scan`, `/odom`, and `/clock` bridges.
- Kept `detour_supervisor` running when `enable_detour:=false`, where it
  publishes `NORMAL` and `active=false`. Once a detour is active, the gate
  remains active while the front obstacle remains even if Nav2 is maneuvering
  and the safety output is moving; only obstacle clearance starts the clear
  window.
- Added the direct simulation-description dependency and made option 17 prefer
  Jazzy for its spawned terminal without overriding an explicit
  `ROS_DISTRO_SETUP`.
- Validated `DetourGate` timing parameters as finite and strictly positive and
  navigation/output thresholds as finite and nonnegative; invalid constructor
  values now fail fast with `ValueError`.
- Removed the extra trailing blank line from the simulation design spec.

## Tests and evidence

- `PYTHONPATH=src/traymover_robot_sim:/opt/ros/jazzy/lib/python3.12/site-packages /usr/bin/python3 -m pytest -q src/traymover_robot_sim/test`
  — **32 passed** after the constructor validation regression cases.
- `bash -n scripts/traymover.sh` and `git diff --check` — passed.
- `colcon build --symlink-install --packages-select traymover_robot_description traymover_robot_sim --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3`
  — both packages finished.
- `xacro .../traymover_sim.urdf.xacro` followed by `gz sdf -p` and `gz sdf -k`
  — valid SDF; generated model has one model-level DiffDrive plugin, zero
  link-level plugins, matching wheel axes, and `gz_frame_id=laser`.
- Headless Gazebo Sim smoke (`gz sim -s -r --headless-rendering` plus
  `ros_gz_sim create`) — `/odom`, `/scan`, and `/cmd_vel` Gazebo topics were
  present. A `linear.x=0.4` Gazebo command changed odometry by 0.254 m in the
  sample window with near-zero lateral displacement; the scan message frame
  was `laser` and odometry frames were `odom` / `base_link`.
- ROS bridge smoke with the final `bridge.yaml` — bridge startup logged all
  four mappings; ROS `/scan` reported frame `laser`, ROS `/odom` reported
  frame `odom`, and one ROS `/cmd_vel` publish drove odometry to `x=0.6708 m`.
  `tf2_echo` observed `base_link -> laser` at `[0.250, 0.000, 0.450]` and the
  `odom -> base_link` transform from `sim_odom_tf`.

## Concerns

- `gz sdf` emits a warning that `gz_frame_id` is a non-core SDF sensor child;
  this is the Gazebo Sensors-supported extension and the runtime smoke
  confirmed it is honored (`LaserScan.header.frame_id=laser`).
- `colcon test` currently runs package tests from a package build directory;
  eight pre-existing repository-relative path assertions fail in that mode.
  The same complete 26-test simulation suite passes from the workspace root
  with the command recorded above.
