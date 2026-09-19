# Option 18: EStop-safe Wait Detour

Option 18 adds a conservative, discrete detour supervisor to the existing
LiDAR + RealSense navigation stack. It waits for temporary obstacles, selects
an observed left or right route for persistent obstacles, and returns control
to Nav2. Every distance, speed, timeout, and clearance below is an **initial
tuning value, not a hardware-validated safety limit**. Software checks and
synthetic ROS tests do not establish real-world stopping distance or coverage.

## Starting the mode

From the workspace root, run `bash scripts/traymover.sh` and select:

```text
18) Navigation with LiDAR + RealSense + EStop-safe Wait Detour
```

The prompts follow Option 10: select a PCD, choose whether to fit the ground
plane for the runtime 2D map, and choose whether to start RViz. Option 18 then
asks `Enable automatic EStop? [Y/n]:`. With automatic EStop enabled, the next prompt is
`Enable wait-and-detour? [Y/n]:`. Disabling automatic EStop forces detour off
and skips the detour prompt. Pressing Enter throughout selects Auto EStop ON,
detour ON, a 5.0 s wait, 1.10 m trigger, 0.95 m guard, 0.08 m/s translation,
and 0.25 rad/s rotation. The final NDT high-structure prompt is also identical
to Option 10. Existing menu options 0–17 retain their numbering.

Trigger/guard checks retain a conservative camera axial-depth metric in addition
to transformed obstacle geometry: the camera sits 0.45 m ahead of `base_link`,
so comparing only base-frame range would detect too late relative to Auto EStop's
0.80 m optical-depth stop. Cached depth clearance is reduced by a conservative
bound on robot translation and camera movement during rotation since acquisition.
The supervisor does not change either Auto EStop threshold.

| Auto EStop | Detour | Behavior |
| --- | --- | --- |
| ON | ON | Wait, observe, and perform guarded discrete detours |
| ON | OFF | Nav2 pass-through with the existing EStop and Collision Monitor chain |
| OFF | OFF | Nav2 pass-through with manual/aggregate driver protection |
| OFF | requested ON | Detour is forced OFF, including direct launch use |

The menu uses Option 10's runtime-map projection, static-TF cleanup, NDT height
weighting, bounded automatic NDT correction, and per-run diagnostic logging.
Logs are stored under `logs/nav/<UTC time>_option18.<suffix>/`. It then starts
the chassis and LiDAR and invokes the detour launch. For a direct launch,
replace both example paths with a matching PCD and map YAML:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch traymover_robot_nav traymover_nav_lidar_realsense_detour.launch.py \
  pcd_path:=/absolute/path/map.pcd \
  map:=/absolute/path/map.yaml \
  bringup_hardware:=true launch_rviz:=true \
  auto_correction_enabled:=true \
  enable_auto_estop:=true enable_detour:=true
```

`bringup_hardware` defaults to `false`: the menu already starts the chassis
and LiDAR. When starting the chassis separately, its `auto_estop_enabled`
parameter must match `enable_auto_estop` on this launch. Set it to `true` only when this launch must start those components.
The main detour launch owns one RealSense driver by default. Its nested
navigation and Auto EStop launches receive `start_realsense_driver:=false`.
If supplying an already-running camera, explicitly pass
`start_realsense_driver:=false` to the main launch as well. The same depth
stream serves Auto EStop and depth-to-scan conversion; the resulting scan is
shared by both costmaps, Collision Monitor, and the supervisor.

## Authority and velocity chain

```text
Nav2 /cmd_vel_nav
  -> Detour Supervisor
       NORMAL: fresh Nav2 Twist passes through
       TURN: linear.x = 0, bounded angular.z
       STRAIGHT: bounded linear.x, angular.z = 0
       waiting / hold / fault / blocked: zero Twist
  -> /cmd_vel_detour
  -> Collision Monitor
  -> /cmd_vel
  -> serial driver's aggregate EStop gate
  -> chassis
```

The driver retains its final `manual_estop_active OR auto_estop_active` gate
and command watchdog. The supervisor never writes `/traymover_estop/set` or
`/traymover_estop/auto_request`, clears either EStop source, changes the depth
evaluator, or disables Collision Monitor. An active EStop always inhibits
supervisor motion and independently inhibits the driver.

The supervisor subscribes to `/cmd_vel_nav`, `/scan`, `/scan_realsense`,
the Option 18 full-perimeter `/scan_detour_perimeter`,
`/traymover_estop/state`, `/traymover_estop/manual_state`,
`/traymover_estop/auto_state`, `/traymover_estop/auto_reason`, TF, and
`/pcl_pose`. Its only velocity output is `/cmd_vel_detour`. Nonzero output
also requires an authorized, accepted navigation goal. After handback it
requires a new, fresh Nav2 command instead of replaying a pre-pause command.

The additive `/traymover_estop/auto_reason` is a reliable, transient-local
`std_msgs/String` heartbeat with `STARTUP`, `NONE`, `NEAR_OBSTACLE`,
`LOW_VALID_DEPTH`, `INVALID_FRAME`, or `DEPTH_TIMEOUT`. Heartbeats retain the
actual reason; `HEARTBEAT` is never a reason-topic value. Sensor faults
replace an old obstacle reason, and a latched request retains its cause
through the evaluator's clear-frame confirmation. A low-valid observation
can be reported before its existing Bool confirmation threshold is reached;
the Bool evaluator and its publication flow remain unchanged.

## Observation, motion, and recovery

In `NORMAL`, confirmed front obstacles enter `WAITING` and immediately
assert zero. Clear observations held for `clear_confirm_sec` return through
`RECOVER_TO_NAV` to Nav2. Otherwise the configured wait leads to
`PLAN_DETOUR`, provided EStop, scans, TF, and localization are healthy.

Planning evaluates both left and right candidates, checks the turn sweep,
LiDAR perimeter coverage, footprint clearance, and the proposed new
heading, then favors the candidate with the longer permitted first leg and
greater clearance. It does not always choose left. Missing beams and
unobserved sectors are unknown space, not permission to move. No acceptable
candidate means `BLOCKED` with zero output. The blocked scene can be
re-observed, but there is no blind forward probing.

The maneuver is `TURN_1 -> STRAIGHT_1 -> TURN_2 -> STRAIGHT_2 ->
RECOVER_TO_NAV`. `TURN_2` returns to the heading recorded at the current
planning start. `PHASE_SETTLE` between phases asserts zero, waits, requires
fresh scans acquired after settling, and validates the next phase.
Straight distances are cropped against current observed clearance while
reserving `detour_target_clearance_m`; the configured leg length is a cap.
The planner presently checks scans directly, not local/global costmap cells.
Costmaps continue to constrain Nav2 before and after the detour.

During either motion phase, front clearance at or below the 0.95 m guard
causes zero output and a fresh plan from the current pose. The configured
relationship is `1.10 m trigger > 0.95 m guard > 0.80 m Auto EStop stop`;
the existing Auto EStop release threshold remains 1.0 m. These different
layers use different observations, so the distance ordering cannot
guarantee that the guard will always precede Auto EStop.

`NEAR_OBSTACLE` Auto EStop enters `HOLD_ESTOP`. Manual EStop also enters
`HOLD_ESTOP`. The interrupted phase, pose, and side are retained for
diagnostics, while the old maneuver is discarded. Release requires both
automatic and aggregate EStop to be clear, valid state heartbeats, healthy
sensors/localization/TF, fresh post-settle observations, and a stable clear
confirmation period. The supervisor then replans from the current pose,
returns to Nav2 if the path is clear, or stays blocked. It never resumes an
old straight leg's remaining distance after either kind of EStop.

Depth faults, stale/malformed scans, unknown Auto EStop reasons, stale EStop
heartbeats, invalid TF, and stale localization enter `FAULT_HOLD`. Sensor
recovery must remain healthy through settling and confirmation before a
new evaluation. Phase timeout, excessive straight heading drift, reverse
displacement, or excessive lateral displacement instead **latch a fault**:
cancel/review and submit a new goal to reset it after the cause is resolved.
A fresh sensor message alone does not clear these maneuver faults.

Stopping means repeated zero commands, a settling interval, and auxiliary
pose-stability checks. Suitable log language is `zero command asserted`,
`settling period completed`, and `pose appears stable`. The system does
not establish physical braking completion or motor-stop feedback.

## Pose, timestamps, and sensor coverage

The default `pose_frame:=odom` uses FAST-LIO's `odom -> base_link` TF for
yaw and straight displacement. Wheel encoders and wheel odometry are not
required. The existing `map -> odom` localization correction is also checked.
`/pcl_pose` must carry fresh, advancing `PoseWithCovarianceStamped` samples
in `map`; current TF alone is not sufficient localization evidence.
`localization_timeout_sec:=5.0` accommodates the existing
`ndt_align_interval_s: 2.0` in `config/localization.yaml`. The 5.0 s value is
a freshness budget, not a change to NDT's update interval or a localization
accuracy guarantee.

Scans are transformed at acquisition time into odom and expressed again
relative to the current robot pose. Header times must be valid and advance;
old queued scans do not become fresh merely by arriving late. Deadlines
and zero-output timers use a steady clock, including when ROS time pauses.

Before motion tests, verify measured camera/LiDAR extrinsics, scan frame
IDs, ROS time alignment, positive-forward scan direction, and turn sign.
The inherited depth-to-scan output frame defaults to `camera_depth_frame`;
its TF must represent the published planar scan correctly. Invalid or
missing transforms hold motion. Relabeling a message does not correct a
wrong physical transform.

The existing navigation `/scan` covers 270 degrees and leaves the rear sector
unobserved. Option 18 therefore adds an independent full-360-degree projector
from `/point_cloud_raw` to `/scan_detour_perimeter` when detour is enabled.
This extra stream supplies observed rear/perimeter evidence for rotation;
it does not change the existing navigation scan. Its freshness and TF are
checked alongside the other scans. Missing bins and projected `+inf`
values remain **unknown** in the supervisor: only valid finite observations
are admitted. A blank stream can produce `FAULT_HOLD`, and insufficient
angular coverage can produce `BLOCKED`. Neither case permits a blind turn.

The LiDAR is mounted high and may miss boxes, stools, cables, children,
and low carts. RealSense adds nearby low-obstacle observations but has
limited field of view and can produce invalid depth. Observed free space
in the high LiDAR plane does not prove that the full floor-level swept
volume is empty. The implementation requires central RealSense coverage
and LiDAR perimeter/fan coverage, but this cannot remove the hardware's
blind spots. Unobserved maneuver clearance produces a hold or `BLOCKED` result;
do not weaken those checks merely to make a demonstration proceed.

## Original navigation goal

Clients, including RViz, use the public `/navigate_to_pose` action provided
by the supervisor's goal proxy. Nav2's matching action is remapped to
`/navigate_to_pose_detour_backend`. This mode's goal-preservation contract
covers `NavigateToPose`; multi-goal/waypoint actions are not proxied.

Nav2 keeps running during a velocity pause. The proxy retains the original
request, including its behavior-tree selection. If Nav2 aborts while the
supervisor is paused, the external goal remains pending; successful
handback permits one resend of that preserved request for that pause.
A normal-navigation abort is returned to the client. The proxy does not
retry indefinitely or persist goals across a supervisor process restart.

Cancel and replacement requests immediately remove motion authorization
and discard the maneuver. Replacement waits for the previous backend to
terminate; further replacements are rejected while cancellation remains
unresolved. A cancel acknowledgement alone does not authorize motion.
Backend server/acceptance waits are bounded at 5.0 s and cancellation at
2.0 s. A timed-out external action can finish while the unresolved backend
still inhibits motion. A late acceptance is canceled, never treated as
permission to move. Resolve the backend condition before submitting a new
goal; a timeout is not a safe automatic reset.

## Configuration

Option 18 installs independent
`config/nav2_params_lidar_realsense_detour.yaml` and
`config/collision_monitor_lidar_realsense_detour.yaml`. The Collision Monitor
polygons match Option 16; only its input topic changes to `cmd_vel_detour`.
The output remains `cmd_vel`.

All planner, controller, behavior-tree, static-map, and localization-related
Nav2 parameters in the Option 18 file match Option 10's `nav2_params.yaml`.
Option 18 only adds LiDAR/RealSense obstacle layers to the local and global
costmaps; regression tests enforce that boundary.

The global costmap adds a dynamic obstacle layer for `/scan` and
`/scan_realsense`, with marking and raytrace clearing enabled and a default
5.0 s observation persistence. It does not write dynamic obstacles to the
static map. Persistence is an observation-buffer duration, not a promise
that a marked costmap cell disappears after exactly five seconds: clearing
still needs appropriate free-space observations. Verify obstacle removal
and costmap clearing during staged tests.

These parameters are exposed by the main launch:

| Parameter | Initial default | Purpose |
| --- | --- | --- |
| `wait_before_detour_sec` | 5.0 | Initial persistent-obstacle wait |
| `detour_trigger_distance_m` | 1.10 | Early obstacle trigger |
| `detour_guard_distance_m` | 0.95 | Stop/replan during maneuver |
| `detour_target_clearance_m` | 1.15 | Desired range reserve when cropping legs |
| `detour_max_linear_speed` | 0.08 | Straight-phase m/s; maximum accepted 0.10 |
| `detour_max_angular_speed` | 0.25 | Turn-phase rad/s; maximum accepted 0.30 |
| `detour_turn_angle_deg` | 45.0 | Candidate turn; accepted range 5–75 degrees |
| `detour_first_leg_m` / `detour_forward_leg_m` | 0.8 / 1.0 | Straight-leg caps |
| `clear_confirm_sec` | 1.0 | Clear/health confirmation interval |
| `stop_settle_time_sec` | 0.5 | Zero-command settling interval |
| `scan_timeout_sec` / `realsense_timeout_sec` | 1.0 / 1.0 | Scan acquisition-age limits |
| `cmd_vel_timeout_sec` | 0.5 | Nav2 command freshness |
| `tf_timeout_sec` | 1.0 | Pose/correction TF age limit |
| `localization_timeout_sec` | 5.0 | `/pcl_pose` age limit |
| `estop_timeout_sec` | 1.0 | EStop state/reason heartbeat age limit |
| `turn_phase_timeout_sec` / `straight_phase_timeout_sec` | 12.0 / 25.0 | Latched maneuver timeouts |
| `obstacle_confirm_frames` / `min_obstacle_points` | 2 / 4 | Trigger confirmation |
| `dynamic_obstacle_persistence_sec` | 5.0 | Dynamic costmap observation persistence |
| `allow_straight_motion` | true | Set false for staged turn-only testing |
| `supervisor_rate_hz` | 20.0 | State evaluation/velocity output rate |
| `pose_frame` / `localization_topic` | `odom` / `/pcl_pose` | Pose and localization inputs |

Additional node parameters are declared by `detour_gate.Config`, rather
than exposed as main-launch arguments: `robot_radius_m=0.50`,
`corridor_half_angle_deg=20.0`, `max_ray_gap_deg=10.0`, `min_leg_m=0.10`,
`yaw_tolerance_rad=0.035`, `pose_stability_distance_m=0.02`,
`pose_stability_yaw_rad=0.035`, `pose_stability_time_sec=0.25`,
`max_straight_lateral_error_m=0.10`, and
`max_straight_reverse_error_m=0.05`. The node also declares
`base_frame=base_link` and `map_frame=map`.
`AUTO_ESTOP_STOP_DISTANCE_M=0.80` is a fixed supervisor consistency bound,
not a configurable node parameter. It does not modify Auto EStop thresholds.

## Required staged hardware validation

Perform these stages in order. Preserve the full safety chain throughout;
the parameter overrides below are additions to the direct-launch example.

1. **Wheels off / chassis disabled:** verify the ROS graph, public/backend
   action ownership, state changes, frame alignment, single camera driver,
   and Twist values. Inject faults without enabling chassis motion.
2. **On the ground, detour disabled:** use `enable_auto_estop:=true
   enable_detour:=false`; verify existing navigation, Collision Monitor,
   manual EStop, and Auto EStop behavior before attempting detours.
3. **Detour enabled, WAITING only:** use `wait_before_detour_sec:=3600`.
   Confirm that the obstacle trigger asserts zero throughout the observation.
   This is a bounded test, **not a dedicated WAIT-only interlock**: cancel
   the goal and terminate the test well before the timer expires. A cleared
   obstacle can still return control to Nav2. EStop/fault recovery can replan
   without repeating the initial wait, so do not inject those events during
   this stage; cancel if the state leaves `WAITING` unexpectedly.
4. **TURN only:** use `allow_straight_motion:=false`. A permitted first
   turn can run, but attempting a straight phase enters latched `BLOCKED`.
   Inspect the result and cancel the goal; do not enable straight motion
   to clear this staged-test latch in place.
5. **TURN plus very short STRAIGHT:** allow straight motion, use
   `detour_max_linear_speed:=0.03` to `0.05`, and set both
   `detour_first_leg_m:=0.10` and `detour_forward_leg_m:=0.10`.
   Validate every settle/scan/phase boundary before increasing distance.
6. **Full simple detour:** test the configured legs with
   `detour_max_linear_speed:=0.08`. Verify both left and right selection,
   blocked routes, costmap clearing, and the original-goal handback.
7. **Deliberate mid-detour EStop:** test automatic obstacle EStop and manual
   EStop during a straight phase. Confirm zero, normal EStop release,
   post-release observations, old-maneuver discard, and a new start pose.
8. **Dynamic people entering the path:** verify guard stops and Auto EStop.
   An operator must stand ready to use physical/manual EStop immediately.

Do not proceed to the next stage until the current stage's results are
understood. No stage in this document is claimed to have been performed.

## Inspection and software verification

Final software results and changed-file inventory: [verification report](wait_detour_verification.md).

```bash
ros2 topic echo /traymover_detour/state
ros2 topic echo /traymover_detour/phase
ros2 topic echo /traymover_detour/reason
ros2 topic echo /traymover_estop/auto_reason
ros2 topic info /cmd_vel_detour --verbose
ros2 topic info /cmd_vel --verbose
ros2 action list -t
./scripts/check_localization.sh
```

`/traymover_detour/active` is a Bool indicating any non-`NORMAL` state,
including holds; it is not a signal that motors are moving. The other
detour status topics are Strings, published on change and periodically.

Use separate generated directories when verifying the integration checkout.
The implementation checkout used for this work is
`/tmp/traymover-integrated-20260917-2b46r7et/workspace`; it is separate from
the original IDE workspace. Run commands from the checkout being verified.
The combined build uses `/tmp/traymover-detour-{build,install,log}`:

```bash
source /opt/ros/humble/setup.bash
bash -n scripts/traymover.sh
git diff --check
colcon --log-base /tmp/traymover-detour-log build \
  --build-base /tmp/traymover-detour-build \
  --install-base /tmp/traymover-detour-install \
  --packages-select traymover_robot_nav traymover_robot_safety turn_on_traymover_robot
source /tmp/traymover-detour-install/setup.bash
colcon --log-base /tmp/traymover-detour-log test \
  --build-base /tmp/traymover-detour-build \
  --install-base /tmp/traymover-detour-install \
  --packages-select traymover_robot_nav traymover_robot_safety turn_on_traymover_robot
colcon test-result --test-result-base /tmp/traymover-detour-build --verbose
```

The safety slice has been built separately and verified with 10 existing
depth-evaluator cases, 4 existing safety-configuration cases, and 7 new
real-node synthetic-depth cases. The new cases cover transient-local
STARTUP delivery, reason persistence, obstacle and sensor-fault reasons,
Bool confirmation/hysteresis, and timeout recovery. That evidence does
not by itself verify the complete navigation graph, full combined suite,
or any hardware behavior; retain the combined run's actual logs and
results separately.
