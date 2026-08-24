# Task 5 report

Status: DONE

Commit: `feat: add Nav2 Gazebo detour bringup` (the Task 5 implementation commit)

Implemented AMCL/Nav2 simulation parameters, delayed global `/scan_global`
obstacle layer, local `/scan` layer, collision monitor safety chain,
recovery-free periodic replanning tree, RViz profile, and complete Gazebo/Nav2
launch with conditional detour supervisor, dynamic-box spawn/cleanup, goal
sender, and RViz.  The simulation package install rules now preserve nested
model assets and declare the Nav2 plugin dependencies used by the config.

Tests:

* Initial config test run (before implementation): `3 failed` with the expected
  `FileNotFoundError` for the three absent files.
* `/usr/bin/python3 -m pytest -q src/traymover_robot_sim/test/test_nav_config.py`
  -> `3 passed in 0.00s`.
* `/usr/bin/python3 -m py_compile src/traymover_robot_sim/launch/traymover_detour_sim.launch.py src/traymover_robot_sim/traymover_robot_sim/*.py`
  -> passed with no output.

Concerns: Gazebo/Nav2 runtime smoke testing was not run because this task is
limited to focused config and syntax checks; launch behavior still depends on
the ROS 2/Nav2/ros_gz packages being installed in the deployment environment.

## Review fix round 1

Status: DONE

Added the Task-6-compatible `demo_goal_sender` executable with
`make_goal_pose`, a `NavigateToPose` action client, and the exact
`goal_x`/`goal_y`/`goal_yaw`/`frame_id`/`send_delay_sec` parameter interface.
Added direct `ament_index_python` and `nav2_common` runtime dependencies for
the simulation launch.

Tests:

* `/usr/bin/python3 -m pytest -q src/traymover_robot_sim/test/test_nav_config.py`
  -> `3 passed in 0.00s`.
* `/usr/bin/python3 -m py_compile src/traymover_robot_sim/launch/traymover_detour_sim.launch.py src/traymover_robot_sim/traymover_robot_sim/*.py`
  -> passed with no output.
* Package XML/setup metadata assertions -> `package metadata checks passed`.

Concern: the full Gazebo/Nav2 runtime remains unexecuted in this focused check.
