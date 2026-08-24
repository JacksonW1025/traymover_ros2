
# Option 17 Detour Simulation Implementation Plan

> For agentic workers: REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox syntax for tracking.

**Goal:** Add menu option 17 and a minimal Gazebo Sim digital-twin demo in which a LiDAR-detected unmapped obstacle first causes a safety stop and, after eight seconds of continuous blocking, causes Nav2 to generate a genuinely different global path.

**Architecture:** Add an isolated traymover_robot_sim Python package for the Jazzy/Gazebo Sim launch, world, map, Nav2 configuration, fixed-goal sender, odometry TF bridge, and detour_supervisor. The supervisor gates /scan_global: /scan always feeds the local obstacle layer and collision monitor, while /scan_global is published only after the safety-stop state persists for the configured hold time. The simulation uses AMCL and a static map; it does not start hardware, FAST-LIO, PCD localization, NDT, or RealSense.

**Tech Stack:** ROS 2 Jazzy, Gazebo Sim, ros_gz_sim, ros_gz_bridge, Nav2, rclpy, sensor_msgs, geometry_msgs, nav_msgs, std_msgs, nav2_msgs, Python pytest, Xacro, Bash.

**Spec:** docs/superpowers/specs/2026-08-24-option17-detour-simulation-design.md

## Global Constraints

- Keep options 1–16 behavior unchanged; option 17 is simulation-only and does not start /dev/ttyCH*, real LiDAR, RealSense, FAST-LIO, PCD, or NDT.
- Use Gazebo Sim/ROS 2 Jazzy interfaces available on the Thor host; do not assume the source workspace’s Humble installation exists locally.
- Preserve the hardware model src/traymover_robot_description/urdf/traymover.urdf.xacro; add a separate simulation model with the same base_link and laser frame names.
- The safety chain remains /cmd_vel_nav -> collision_monitor -> /cmd_vel; the detour node never publishes velocity.
- Dynamic obstacles reach the global costmap only through /scan_global after continuous blocking for hold_time_sec=8.0 by default.
- Use --packages-select for any build; never build the entire workspace as part of this feature.
- Use /usr/bin/python3 for local ROS tooling when the active Conda interpreter would select an incompatible Python.

---

### Task 1: Create the isolated simulation package and test harness

**Files:**
- Create: src/traymover_robot_sim/package.xml
- Create: src/traymover_robot_sim/setup.py
- Create: src/traymover_robot_sim/setup.cfg
- Create: src/traymover_robot_sim/resource/traymover_robot_sim
- Create: src/traymover_robot_sim/traymover_robot_sim/__init__.py
- Create: src/traymover_robot_sim/test/test_sim_package.py

**Interfaces:**
- Produces an ament_python package named traymover_robot_sim.
- Registers executables detour_supervisor, demo_goal_sender, and sim_odom_tf from the package module paths that later tasks implement.
- Installs launch, config, worlds, models, maps, behavior_trees, and rviz beneath share/traymover_robot_sim.

- [ ] Step 1: Write the failing package-layout test.

~~~
from pathlib import Path


def test_sim_package_declares_runtime_asset_directories():
    root = Path(__file__).parents[1]
    package_xml = (root / "package.xml").read_text()
    assert "<name>traymover_robot_sim</name>" in package_xml
    for directory in ("launch", "config", "worlds", "models", "maps", "behavior_trees", "rviz"):
        assert (root / directory).is_dir(), directory


def test_setup_registers_required_console_scripts():
    setup_text = (Path(__file__).parents[1] / "setup.py").read_text()
    for executable in ("detour_supervisor", "demo_goal_sender", "sim_odom_tf"):
        assert executable in setup_text
~~~

- [ ] Step 2: Run the test and verify it fails because the package does not exist.

Run: /usr/bin/python3 -m pytest -q src/traymover_robot_sim/test/test_sim_package.py

Expected: collection or file-not-found failure.

- [ ] Step 3: Add package metadata and the empty runtime directories.

Declare build/runtime dependencies for ament_python, rclpy, geometry_msgs, sensor_msgs, nav_msgs, std_msgs, nav2_msgs, nav2_map_server, nav2_amcl, nav2_planner, nav2_controller, nav2_bt_navigator, nav2_lifecycle_manager, nav2_collision_monitor, robot_state_publisher, ros_gz_sim, ros_gz_bridge, rviz2, launch, launch_ros, xacro, tf2_ros, and pytest as a test dependency. Register the three console scripts and install all runtime asset directories with glob in setup.py.

- [ ] Step 4: Run the test and verify it passes.

Run: /usr/bin/python3 -m pytest -q src/traymover_robot_sim/test/test_sim_package.py

Expected: PASS.

- [ ] Step 5: Commit the package scaffold.

~~~
git add src/traymover_robot_sim
git commit -m "feat: scaffold traymover simulation package"
~~~

### Task 2: Implement and test the time-gated detour state machine

**Files:**
- Create: src/traymover_robot_sim/traymover_robot_sim/detour_supervisor.py
- Create: src/traymover_robot_sim/test/test_detour_gate.py

**Interfaces:**
- SafetyObservation(now_sec: float, nav_speed: float, output_speed: float, front_obstacle: bool, estop_active: bool = False) is an immutable input sample.
- GateState contains NORMAL, STOP_WAITING, DETOUR_ACTIVE, and CLEARING.
- GateDecision(state: GateState, forward_global_scan: bool) is the state-machine output.
- DetourGate(hold_time_sec: float = 8.0, nav_intent_threshold: float = 0.05, output_stop_threshold: float = 0.01, clear_publish_sec: float = 1.5) exposes update(observation: SafetyObservation) -> GateDecision.
- A blocking sample is front_obstacle and ((nav_speed >= nav_intent_threshold and output_speed <= output_stop_threshold) or estop_active); EStop without a front obstacle never starts detour.

- [ ] Step 1: Write failing tests for all state transitions.

~~~
from traymover_robot_sim.detour_supervisor import (
    DetourGate, GateState, SafetyObservation,
)


def sample(t, nav=0.0, output=0.0, obstacle=False, estop=False):
    return SafetyObservation(t, nav, output, obstacle, estop)


def test_blocking_starts_wait_without_forwarding():
    gate = DetourGate(hold_time_sec=8.0)
    decision = gate.update(sample(10.0, nav=0.3, output=0.0, obstacle=True))
    assert decision.state is GateState.STOP_WAITING
    assert not decision.forward_global_scan


def test_blocking_for_eight_seconds_enables_global_scan():
    gate = DetourGate(hold_time_sec=8.0)
    gate.update(sample(10.0, nav=0.3, output=0.0, obstacle=True))
    decision = gate.update(sample(18.0, nav=0.3, output=0.0, obstacle=True))
    assert decision.state is GateState.DETOUR_ACTIVE
    assert decision.forward_global_scan


def test_obstacle_clear_before_timeout_resets_without_detour():
    gate = DetourGate(hold_time_sec=8.0)
    gate.update(sample(10.0, nav=0.3, output=0.0, obstacle=True))
    decision = gate.update(sample(13.0, nav=0.3, output=0.3, obstacle=False))
    assert decision.state is GateState.NORMAL
    assert not decision.forward_global_scan


def test_clear_after_detour_uses_a_clear_window():
    gate = DetourGate(hold_time_sec=8.0, clear_publish_sec=1.5)
    gate.update(sample(0.0, nav=0.3, output=0.0, obstacle=True))
    gate.update(sample(8.0, nav=0.3, output=0.0, obstacle=True))
    clearing = gate.update(sample(9.0, nav=0.3, output=0.3, obstacle=False))
    assert clearing.state is GateState.CLEARING
    assert clearing.forward_global_scan
    normal = gate.update(sample(10.6, nav=0.3, output=0.3, obstacle=False))
    assert normal.state is GateState.NORMAL
    assert not normal.forward_global_scan


def test_estop_alone_without_obstacle_does_not_trigger_detour():
    gate = DetourGate()
    decision = gate.update(sample(1.0, estop=True, obstacle=False))
    assert decision.state is GateState.NORMAL
~~~

- [ ] Step 2: Run the tests and verify they fail because the state-machine types do not exist.

Run: /usr/bin/python3 -m pytest -q src/traymover_robot_sim/test/test_detour_gate.py

Expected: import failure.

- [ ] Step 3: Implement only the pure state machine.

Use Enum, frozen dataclass inputs/outputs, a nullable blocked_since_sec, and a nullable clear_started_sec. update() must reset waiting when the blocking predicate is false, enter DETOUR_ACTIVE exactly when now_sec - blocked_since_sec >= hold_time_sec, and keep forwarding during CLEARING until the clear window expires. Reject decreasing timestamps with ValueError so a broken simulated clock cannot shorten the safety hold.

- [ ] Step 4: Run the tests and verify they pass.

Run: /usr/bin/python3 -m pytest -q src/traymover_robot_sim/test/test_detour_gate.py

Expected: PASS.

- [ ] Step 5: Commit the state machine.

~~~
git add src/traymover_robot_sim/traymover_robot_sim/detour_supervisor.py src/traymover_robot_sim/test/test_detour_gate.py
git commit -m "feat: add timed detour gate state machine"
~~~

### Task 3: Wrap the gate in a ROS node and test LiDAR gating

**Files:**
- Modify: src/traymover_robot_sim/traymover_robot_sim/detour_supervisor.py
- Create: src/traymover_robot_sim/test/test_scan_helpers.py

**Interfaces:**
- command_speed(twist: geometry_msgs.msg.Twist) -> float returns hypot(twist.linear.x, twist.linear.y) + abs(twist.angular.z).
- has_front_obstacle(scan: sensor_msgs.msg.LaserScan, stop_distance: float, front_half_angle: float) -> bool ignores NaN/inf/out-of-range samples and checks only abs(angle) <= front_half_angle.
- DetourSupervisor(Node) subscribes to /scan, /cmd_vel_nav, /cmd_vel, and /traymover_estop/state; it publishes /scan_global, /traymover_detour/state, and /traymover_detour/active.
- Parameters: enable_detour, hold_time_sec, stop_distance, front_half_angle, clear_publish_sec, nav_intent_threshold, output_stop_threshold, tick_hz.

- [ ] Step 1: Write failing pure helper tests.

~~~
from math import inf, nan
from sensor_msgs.msg import LaserScan
from traymover_robot_sim.detour_supervisor import has_front_obstacle


def scan(ranges, angle_min=-0.6, angle_increment=0.2):
    msg = LaserScan()
    msg.angle_min = angle_min
    msg.angle_increment = angle_increment
    msg.range_min = 0.2
    msg.range_max = 5.0
    msg.ranges = ranges
    return msg


def test_front_obstacle_ignores_invalid_and_side_samples():
    assert has_front_obstacle(scan([inf, 0.7, nan, 2.0]), 0.8, 0.25)
    assert not has_front_obstacle(scan([0.5, inf, inf, inf]), 0.8, 0.25)
~~~

- [ ] Step 2: Run the helper tests and verify the missing helper fails.

Run: /usr/bin/python3 -m pytest -q src/traymover_robot_sim/test/test_scan_helpers.py

Expected: import failure until has_front_obstacle exists.

- [ ] Step 3: Implement the ROS wrapper.

Store the latest scan, navigation command, output command, and optional EStop state. A 20 Hz timer creates a SafetyObservation using ROS clock seconds, calls DetourGate.update(), publishes the state strings and active boolean, and republishes the latest scan to /scan_global only when the decision says so. Preserve the input scan frame, timing, angular limits, and range limits. With enable_detour=false, keep publishing state NORMAL but never publish /scan_global.

- [ ] Step 4: Run the helper and state-machine tests plus the existing navigation config test.

Run: /usr/bin/python3 -m pytest -q src/traymover_robot_sim/test/test_scan_helpers.py src/traymover_robot_sim/test/test_detour_gate.py src/traymover_robot_nav/test/test_nav_simple_config.py

Expected: PASS for the new tests; the existing navigation test must continue to pass.

- [ ] Step 5: Commit the ROS node.

~~~
git add src/traymover_robot_sim/traymover_robot_sim/detour_supervisor.py src/traymover_robot_sim/test/test_scan_helpers.py
git commit -m "feat: gate simulated lidar for delayed detours"
~~~

### Task 4: Add the simulation robot, world, map, and Gazebo bridge

**Files:**
- Create: src/traymover_robot_description/urdf/traymover_sim.urdf.xacro
- Create: src/traymover_robot_sim/worlds/traymover_detour.sdf
- Create: src/traymover_robot_sim/models/dynamic_box/model.sdf
- Create: src/traymover_robot_sim/maps/detour_demo.yaml
- Create: src/traymover_robot_sim/maps/detour_demo.pgm
- Create: src/traymover_robot_sim/config/bridge.yaml
- Create: src/traymover_robot_sim/traymover_robot_sim/sim_odom_tf.py
- Create: src/traymover_robot_sim/test/test_sim_assets.py

**Interfaces:**
- The simulation xacro exports base_link, wheel joints, and laser; the Gazebo diff-drive plugin consumes /cmd_vel and publishes /odom.
- The planar LiDAR publishes a bridgeable sensor_msgs/msg/LaserScan on /scan, mounted around z=0.45 m so the 0.9 m dynamic box is guaranteed to intersect the scan plane.
- sim_odom_tf.py subscribes to /odom and broadcasts odom -> base_link with tf2_ros.TransformBroadcaster; it avoids a Gazebo-native TF bridge.
- bridge.yaml maps /clock (rosgraph_msgs/msg/Clock), /scan (sensor_msgs/msg/LaserScan), and /odom (nav_msgs/msg/Odometry) from Gazebo to ROS.
- The world bounds are x=[0,8], y=[-3,3]; the robot starts near (1.0,0.0), the default goal is near (7.0,0.0), static obstacles leave two unequal side routes, and the dynamic box is spawned separately near (4.0,0.0).
- The map is an ASCII PGM at 0.1 m/cell, 80x60, origin [0.0,-3.0,0.0]; it contains walls and static obstacles but not the dynamic box.

- [ ] Step 1: Write failing asset validation tests.

~~~
from pathlib import Path
import xml.etree.ElementTree as ET


def test_sim_model_has_required_frames_and_plugins():
    root = ET.parse(
        Path("src/traymover_robot_description/urdf/traymover_sim.urdf.xacro")
    ).getroot()
    links = {node.attrib.get("name") for node in root.findall("link")}
    assert {"base_link", "laser"} <= links
    text = Path(
        "src/traymover_robot_description/urdf/traymover_sim.urdf.xacro"
    ).read_text()
    assert "gz-sim-diff-drive-system" in text
    assert "gz-sim-lidar-system" in text or "gpu_lidar" in text


def test_demo_map_has_expected_metadata():
    yaml_text = Path("src/traymover_robot_sim/maps/detour_demo.yaml").read_text()
    assert "detour_demo.pgm" in yaml_text
    assert "resolution: 0.1" in yaml_text
    assert "origin: [0.0, -3.0, 0.0]" in yaml_text
~~~

- [ ] Step 2: Run the tests and verify they fail because the assets are absent.

Run: /usr/bin/python3 -m pytest -q src/traymover_robot_sim/test/test_sim_assets.py

Expected: file-not-found failure.

- [ ] Step 3: Add the simulation xacro and Gazebo assets.

Reuse the current body/wheel dimensions and frame names in a separate model. Add the Gazebo Sim diff-drive plugin with left/right wheel joints, wheel_separation=0.455, wheel_radius=0.085, /cmd_vel, and /odom; add a 2D LiDAR with frame_id=laser, topic=/scan, range_min=0.2, range_max=8.0, and a 270-degree field of view. Create a floor, outer walls, two static asymmetric blocks, and a separate dynamic box SDF sized 0.9 x 0.9 x 0.9 m.

Create the P2 PGM with white free space, black outer walls, and black cells for only the static blocks. Use the YAML values specified in the interface block. Add bridge.yaml with explicit Gazebo-to-ROS mappings and add sim_odom_tf.py with a single subscription callback that copies pose/twist from /odom into a TransformStamped stamped with the message header.

- [ ] Step 4: Expand Xacro and asset validation.

Run: /usr/bin/python3 -m pytest -q src/traymover_robot_sim/test/test_sim_assets.py

Run: xacro src/traymover_robot_description/urdf/traymover_sim.urdf.xacro >/tmp/traymover_sim.urdf

Expected: both commands pass and the generated URDF contains base_link, laser, and no parse errors.

- [ ] Step 5: Commit the simulation assets.

~~~
git add src/traymover_robot_description/urdf/traymover_sim.urdf.xacro src/traymover_robot_sim
git commit -m "feat: add Gazebo detour simulation assets"
~~~

### Task 5: Add AMCL/Nav2, collision monitor, behavior tree, and launch

**Files:**
- Create: src/traymover_robot_sim/behavior_trees/navigate_detour.xml
- Create: src/traymover_robot_sim/config/nav2_params_detour_sim.yaml
- Create: src/traymover_robot_sim/config/collision_monitor_detour_sim.yaml
- Create: src/traymover_robot_sim/launch/traymover_detour_sim.launch.py
- Create: src/traymover_robot_sim/rviz/traymover_detour.rviz
- Create: src/traymover_robot_sim/test/test_nav_config.py

**Interfaces:**
- Launch arguments: use_sim_time=true, enable_detour=true, spawn_dynamic_obstacle=true, obstacle_spawn_delay=12.0, obstacle_lifetime_sec=0.0, auto_send_goal=true, goal_x=7.0, goal_y=0.0, goal_yaw=0.0, launch_rviz=true, map, and world. A zero obstacle lifetime means the box persists.
- Nav2 lifecycle nodes: map_server, amcl, planner_server, controller_server, and bt_navigator.
- Local costmap observes /scan; global costmap observes only /scan_global for its obstacle_layer.
- The behavior tree uses RateController hz="1.0", ComputePathToPose planner_id="GridBased", and FollowPath controller_id="FollowPath".

- [ ] Step 1: Write failing config tests.

~~~
from pathlib import Path


def test_global_costmap_is_delayed_scan_only():
    text = Path("src/traymover_robot_sim/config/nav2_params_detour_sim.yaml").read_text()
    assert 'plugins: ["static_layer", "obstacle_layer", "inflation_layer"]' in text
    assert "topic: /scan_global" in text
    assert "topic: /scan" in text


def test_collision_monitor_keeps_the_option_16_safety_chain():
    text = Path(
        "src/traymover_robot_sim/config/collision_monitor_detour_sim.yaml"
    ).read_text()
    assert "cmd_vel_in_topic: cmd_vel_nav" in text
    assert "cmd_vel_out_topic: cmd_vel" in text
    assert "topic: /scan" in text


def test_tree_replans_periodically():
    text = Path("src/traymover_robot_sim/behavior_trees/navigate_detour.xml").read_text()
    assert 'RateController hz="1.0"' in text
    assert "ComputePathToPose" in text
    assert "FollowPath" in text
~~~

- [ ] Step 2: Run the config tests and verify they fail because the files are absent.

Run: /usr/bin/python3 -m pytest -q src/traymover_robot_sim/test/test_nav_config.py

Expected: file-not-found failure.

- [ ] Step 3: Add the Nav2 and collision monitor configuration.

Start from the option-16 controller/progress-checker values, replace FAST-LIO/NDT localization with AMCL parameters (set_initial_pose=true, initial_pose_x=1.0, initial_pose_y=0.0, initial_pose_a=0.0), set global_frame=map, robot_base_frame=base_link, and set use_sim_time=true. Configure global_costmap with static, delayed obstacle, and inflation layers; configure its observation source as /scan_global with marking and clearing enabled. Configure local_costmap and collision monitor to consume /scan. Keep the controller’s cmd_vel remapped to /cmd_vel_nav in launch so the monitor remains the final velocity publisher.

Use a recovery-free periodic behavior tree matching:

~~~
<PipelineSequence name="NavigateWithReplanning">
  <RateController hz="1.0">
    <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
  </RateController>
  <FollowPath path="{path}" controller_id="FollowPath"/>
</PipelineSequence>
~~~

- [ ] Step 4: Add the simulation launch.

Start Gazebo with ros_gz_sim and the world, publish the simulation xacro with robot_state_publisher, spawn the robot using ros_gz_sim create -topic robot_description, start ros_gz_bridge from bridge.yaml, start sim_odom_tf, and start the five Nav2 lifecycle nodes with the simulation parameters. Start nav2_collision_monitor with the simulation monitor parameters and start detour_supervisor conditionally from enable_detour. Use a TimerAction with obstacle_spawn_delay to run ros_gz_sim create for the dynamic box. When obstacle_lifetime_sec is greater than zero, schedule ros2 run ros_gz_sim delete_entity --name dynamic_box at obstacle_spawn_delay + obstacle_lifetime_sec; this makes the early-clear acceptance path deterministic. Start demo_goal_sender conditionally from auto_send_goal and RViz conditionally from launch_rviz.

- [ ] Step 5: Run config and launch syntax checks.

Run: /usr/bin/python3 -m pytest -q src/traymover_robot_sim/test/test_nav_config.py

Run: /usr/bin/python3 -m py_compile src/traymover_robot_sim/launch/traymover_detour_sim.launch.py src/traymover_robot_sim/traymover_robot_sim/*.py

Expected: PASS with no syntax errors.

- [ ] Step 6: Commit the Nav2 simulation bringup.

~~~
git add src/traymover_robot_sim/behavior_trees src/traymover_robot_sim/config src/traymover_robot_sim/launch src/traymover_robot_sim/rviz src/traymover_robot_sim/test/test_nav_config.py
git commit -m "feat: add Nav2 Gazebo detour bringup"
~~~

### Task 6: Implement the fixed-goal sender and simulation diagnostics

**Files:**
- Create: src/traymover_robot_sim/traymover_robot_sim/demo_goal_sender.py
- Create: src/traymover_robot_sim/test/test_demo_goal.py

**Interfaces:**
- DemoGoalSender(Node) declares goal_x, goal_y, goal_yaw, frame_id, and send_delay_sec.
- It waits for the navigate_to_pose action, sends one nav2_msgs.action.NavigateToPose.Goal, logs the accepted/result status, and exits after the result.
- RViz can still send a replacement goal; auto_send_goal=false disables the fixed automatic goal.

- [ ] Step 1: Write the failing goal-message construction test.

~~~
from math import isclose
from traymover_robot_sim.demo_goal_sender import make_goal_pose


def test_make_goal_pose_uses_map_frame_and_yaw_quaternion():
    pose = make_goal_pose(7.0, 0.0, 0.0, "map")
    assert pose.header.frame_id == "map"
    assert isclose(pose.pose.position.x, 7.0)
    assert isclose(pose.pose.orientation.w, 1.0)
    assert isclose(pose.pose.orientation.z, 0.0)
~~~

- [ ] Step 2: Run the test and verify the function is missing.

Run: /usr/bin/python3 -m pytest -q src/traymover_robot_sim/test/test_demo_goal.py

Expected: import failure.

- [ ] Step 3: Implement make_goal_pose() and the action client node.

Use sin(yaw/2) and cos(yaw/2) for the planar quaternion, use ROS clock timers for send_delay_sec, wait for the action server without sending repeated goals, and call rclpy.shutdown() after the result callback.

- [ ] Step 4: Run the test and commit.

Run: /usr/bin/python3 -m pytest -q src/traymover_robot_sim/test/test_demo_goal.py

Expected: PASS.

~~~
git add src/traymover_robot_sim/traymover_robot_sim/demo_goal_sender.py src/traymover_robot_sim/test/test_demo_goal.py
git commit -m "feat: add deterministic simulation goal sender"
~~~

### Task 7: Add menu option 17 without changing hardware options

**Files:**
- Modify: scripts/traymover.sh near print_menu() and the option dispatch.
- Create: src/traymover_robot_sim/test/test_option17_menu.py

**Interfaces:**
- Add action_start_nav_detour_sim().
- Add menu text 17) Simulation: Dynamic obstacle stop and detour.
- Add dispatch 17) action_start_nav_detour_sim ;;.

- [ ] Step 1: Write the failing menu test.

~~~
from pathlib import Path


def test_option17_is_simulation_only_and_has_detour_prompt():
    text = Path("scripts/traymover.sh").read_text()
    assert "17) Simulation: Dynamic obstacle stop and detour" in text
    assert "17) action_start_nav_detour_sim" in text
    assert "enable_detour" in text
    option_start = text.index("action_start_nav_detour_sim()")
    option_end = text.index("action_start_nav_speed_modes()", option_start)
    block = text[option_start:option_end]
    assert "traymover_robot_sim traymover_detour_sim.launch.py" in block
    assert "base_serial.launch.py" not in block
    assert "traymover_lidar.launch.py" not in block
~~~

- [ ] Step 2: Run the test and verify it fails because menu option 17 is absent.

Run: /usr/bin/python3 -m pytest -q src/traymover_robot_sim/test/test_option17_menu.py

Expected: assertion failure.

- [ ] Step 3: Add the launcher function and menu dispatch.

Prompt Enable 8 s detour? [Y/n] with default yes and Launch RViz? [Y/n] with default yes. Convert choices to enable_detour:=true|false and launch_rviz:=true|false, then call:

~~~
ros2 launch traymover_robot_sim traymover_detour_sim.launch.py \
  enable_detour:=true launch_rviz:=true
~~~

Use spawn_in_terminal so the existing launcher lifecycle and option-0 cleanup conventions remain intact. Do not call prompt_auto_estop, base_serial, or the hardware LiDAR launch from this function.

- [ ] Step 4: Run shell and menu tests.

Run: bash -n scripts/traymover.sh

Run: /usr/bin/python3 -m pytest -q src/traymover_robot_sim/test/test_option17_menu.py

Expected: both PASS.

- [ ] Step 5: Commit the menu integration.

~~~
git add scripts/traymover.sh src/traymover_robot_sim/test/test_option17_menu.py
git commit -m "feat: add option 17 detour simulation menu"
~~~

### Task 8: Build only the simulation packages and perform the two-path acceptance check

**Files:**
- Modify: docs/mapping_and_nav2_guide.md with the option-17 command and topic checks.
- Modify: src/traymover_robot_sim/test/test_sim_package.py if package installation exposes a missing asset.

**Interfaces:**
- Build target is exactly traymover_robot_description traymover_robot_sim.
- Acceptance topics are /scan, /scan_global, /cmd_vel_nav, /cmd_vel, /plan, /map, /odom, and /traymover_detour/state.

- [ ] Step 1: Run the complete static/unit test set before building.

Run: /usr/bin/python3 -m pytest -q src/traymover_robot_sim/test

Run: bash -n scripts/traymover.sh

Run: xacro src/traymover_robot_description/urdf/traymover_sim.urdf.xacro >/tmp/traymover_sim.urdf

Expected: all targeted tests pass and Xacro expands.

- [ ] Step 2: Build only the two relevant packages.

Run:

~~~
colcon build --symlink-install \
  --packages-select traymover_robot_description traymover_robot_sim \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
~~~

Expected: only the description and simulation packages are selected. If a Jazzy dependency is absent, record the exact package name and stop at a clear dependency diagnostic; do not expand the build to the rest of the repository.

- [ ] Step 3: Validate launch arguments after sourcing the targeted overlay.

Run:

~~~
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch traymover_robot_sim traymover_detour_sim.launch.py --show-args
~~~

Expected: enable_detour, spawn_dynamic_obstacle, obstacle_spawn_delay, obstacle_lifetime_sec, auto_send_goal, goal pose, map/world, and RViz arguments are listed.

- [ ] Step 4: Run the enabled-detour acceptance path.

Start bash scripts/traymover.sh, select 17, accept detour and RViz, then verify: the dynamic box appears; /cmd_vel becomes zero while /cmd_vel_nav remains nonzero; state remains STOP_WAITING for approximately eight simulated seconds; /scan_global begins; /plan bends around the box; and the robot reaches the fixed goal.

- [ ] Step 5: Run the stop-only and early-clear acceptance paths.

Repeat option 17 with detour disabled and verify the box causes a stop without a changed plan. For the early-clear path, run the launch with spawn_dynamic_obstacle:=true obstacle_spawn_delay:=8.0 obstacle_lifetime_sec:=4.0 and verify the box is deleted before the eight-second hold completes, the state returns to NORMAL, and /scan_global never becomes active.

- [ ] Step 6: Document the run and commit the final verification notes.

Add to docs/mapping_and_nav2_guide.md the exact option-17 flow, the three launch commands for enabled/disabled detour, and topic-based health checks. Commit:

~~~
git add docs/mapping_and_nav2_guide.md
git commit -m "docs: describe option 17 detour simulation"
~~~
