from pathlib import Path


ROOT = Path(__file__).parents[3]


def test_global_costmap_is_delayed_scan_only():
    text = (ROOT / "src/traymover_robot_sim/config/nav2_params_detour_sim.yaml").read_text()
    assert 'plugins: ["static_layer", "obstacle_layer", "inflation_layer"]' in text
    assert "topic: /scan_global" in text
    assert "topic: /scan" in text


def test_collision_monitor_keeps_the_option_16_safety_chain():
    text = (ROOT / "src/traymover_robot_sim/config/collision_monitor_detour_sim.yaml").read_text()
    assert "cmd_vel_in_topic: cmd_vel_nav" in text
    assert "cmd_vel_out_topic: cmd_vel_safety" in text
    assert "topic: /scan" in text


def test_jazzy_collision_monitor_uses_string_polygon_points():
    text = (ROOT / "src/traymover_robot_sim/config/collision_monitor_detour_sim.yaml").read_text()
    assert 'points: "[[0.00, 0.35], [0.90, 0.35], [0.90, -0.35], [0.00, -0.35]]"' in text
    assert 'points: "[[0.20, 0.50], [1.40, 0.50], [1.40, -0.50], [0.20, -0.50]]"' in text


def test_tree_replans_periodically():
    text = (ROOT / "src/traymover_robot_sim/behavior_trees/navigate_detour.xml").read_text()
    assert 'RateController hz="1.0"' in text
    assert "ComputePathToPose" in text
    assert "FollowPath" in text


def test_bt_planner_request_timeout_allows_simulated_planning_jitter():
    text = (ROOT / "src/traymover_robot_sim/config/nav2_params_detour_sim.yaml").read_text()
    assert "default_server_timeout: 500" in text


def test_detour_supervisor_runs_in_disabled_mode_and_reports_normal():
    text = (ROOT / "src/traymover_robot_sim/launch/traymover_detour_sim.launch.py").read_text()
    assert 'name="detour_supervisor"' in text
    assert '"enable_detour": enable_detour' in text
    assert 'condition=IfCondition(enable_detour)' not in text


def test_collision_monitor_is_activated_with_the_navigation_stack():
    text = (ROOT / "src/traymover_robot_sim/launch/traymover_detour_sim.launch.py").read_text()
    assert '"collision_monitor"' in text.split("NAV2_LIFECYCLE_NODES", 1)[1].split("]", 1)[0]


def test_supervisor_muxes_safety_output_to_the_gazebo_command_topic():
    text = (ROOT / "src/traymover_robot_sim/traymover_robot_sim/detour_supervisor.py").read_text()
    assert "'/cmd_vel_safety'" in text
    assert "'/cmd_vel'" in text
    assert "forward_global_scan" in text
    assert "detour_max_angular_speed" in text


def test_jazzy_uses_gz_sim_command_not_removed_ros_wrapper_executable():
    text = (ROOT / "src/traymover_robot_sim/launch/traymover_detour_sim.launch.py").read_text()
    assert 'cmd=["gz", "sim", "-r", world]' in text
    assert 'executable="gz_sim"' not in text


def test_launch_can_run_server_only_for_headless_acceptance_checks():
    text = (ROOT / "src/traymover_robot_sim/launch/traymover_detour_sim.launch.py").read_text()
    assert 'DeclareLaunchArgument("headless_gazebo", default_value="false")' in text
    assert 'cmd=["gz", "sim", "-s", "-r", world]' in text


def test_jazzy_costmap_window_dimensions_are_integer_parameters():
    text = (ROOT / "src/traymover_robot_sim/config/nav2_params_detour_sim.yaml").read_text()
    assert "      width: 4\n" in text
    assert "      height: 4\n" in text
    assert "      width: 4.0\n" not in text
    assert "      height: 4.0\n" not in text


def test_jazzy_navfn_plugin_uses_namespaced_class_name():
    text = (ROOT / "src/traymover_robot_sim/config/nav2_params_detour_sim.yaml").read_text()
    assert "plugin: nav2_navfn_planner::NavfnPlanner" in text
    assert "plugin: nav2_navfn_planner/NavfnPlanner" not in text


def test_amcl_initial_pose_matches_sim_spawn_pose():
    text = (ROOT / "src/traymover_robot_sim/config/nav2_params_detour_sim.yaml").read_text()
    assert "    initial_pose:\n      x: 1.0\n      y: 0.0\n      z: 0.0\n      yaw: 0.0" in text
    assert "initial_pose_x:" not in text
    assert "initial_pose_a:" not in text


def test_detour_demo_uses_two_times_linear_speed():
    text = (ROOT / "src/traymover_robot_sim/config/nav2_params_detour_sim.yaml").read_text()
    assert "desired_linear_vel: 0.20" in text
    assert "min_approach_linear_velocity: 0.08" in text
    assert "regulated_linear_scaling_min_speed: 0.07" in text


def test_detour_controller_uses_smoother_continuous_tracking():
    text = (ROOT / "src/traymover_robot_sim/config/nav2_params_detour_sim.yaml").read_text()
    assert "lookahead_dist: 1.40" in text
    assert "min_lookahead_dist: 1.10" in text
    assert "max_lookahead_dist: 2.60" in text
    assert "lookahead_time: 2.80" in text
    assert "use_rotate_to_heading: false" in text


def test_detour_global_inflation_leaves_the_close_side_corridor_open():
    text = (ROOT / "src/traymover_robot_sim/config/nav2_params_detour_sim.yaml").read_text()
    global_costmap = text.split("global_costmap:", 1)[1]
    assert "inflation_radius: 0.35" in global_costmap


def test_detour_anchor_heading_tolerance_prevents_rpp_orbiting_waypoints():
    text = (ROOT / "src/traymover_robot_sim/config/nav2_params_detour_sim.yaml").read_text()
    assert "yaw_goal_tolerance: 1.00" in text
