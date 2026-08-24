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
    assert "cmd_vel_out_topic: cmd_vel_safety" in text
    assert "topic: /scan" in text


def test_jazzy_collision_monitor_uses_string_polygon_points():
    text = Path(
        "src/traymover_robot_sim/config/collision_monitor_detour_sim.yaml"
    ).read_text()
    assert 'points: "[[0.40, 0.32], [0.72, 0.32], [0.72, -0.32], [0.40, -0.32]]"' in text
    assert 'points: "[[0.40, 0.45], [1.30, 0.45], [1.30, -0.45], [0.40, -0.45]]"' in text


def test_tree_replans_periodically():
    text = Path("src/traymover_robot_sim/behavior_trees/navigate_detour.xml").read_text()
    assert 'RateController hz="1.0"' in text
    assert "ComputePathToPose" in text
    assert "FollowPath" in text


def test_bt_planner_request_timeout_allows_simulated_planning_jitter():
    text = Path(
        "src/traymover_robot_sim/config/nav2_params_detour_sim.yaml"
    ).read_text()
    assert "default_server_timeout: 500" in text


def test_detour_supervisor_runs_in_disabled_mode_and_reports_normal():
    text = Path(
        "src/traymover_robot_sim/launch/traymover_detour_sim.launch.py"
    ).read_text()
    assert 'name="detour_supervisor"' in text
    assert '"enable_detour": enable_detour' in text
    assert 'condition=IfCondition(enable_detour)' not in text


def test_collision_monitor_is_activated_with_the_navigation_stack():
    text = Path(
        "src/traymover_robot_sim/launch/traymover_detour_sim.launch.py"
    ).read_text()
    assert '"collision_monitor"' in text.split("NAV2_LIFECYCLE_NODES", 1)[1].split("]", 1)[0]


def test_supervisor_muxes_safety_output_to_the_gazebo_command_topic():
    text = Path(
        "src/traymover_robot_sim/traymover_robot_sim/detour_supervisor.py"
    ).read_text()
    assert "'/cmd_vel_safety'" in text
    assert "'/cmd_vel'" in text
    assert "forward_global_scan" in text


def test_jazzy_uses_gz_sim_command_not_removed_ros_wrapper_executable():
    text = Path(
        "src/traymover_robot_sim/launch/traymover_detour_sim.launch.py"
    ).read_text()
    assert 'cmd=["gz", "sim", "-r", world]' in text
    assert 'executable="gz_sim"' not in text


def test_launch_can_run_server_only_for_headless_acceptance_checks():
    text = Path(
        "src/traymover_robot_sim/launch/traymover_detour_sim.launch.py"
    ).read_text()
    assert 'DeclareLaunchArgument("headless_gazebo", default_value="false")' in text
    assert 'cmd=["gz", "sim", "-s", "-r", world]' in text


def test_jazzy_costmap_window_dimensions_are_integer_parameters():
    text = Path(
        "src/traymover_robot_sim/config/nav2_params_detour_sim.yaml"
    ).read_text()
    assert "      width: 4\n" in text
    assert "      height: 4\n" in text
    assert "      width: 4.0\n" not in text
    assert "      height: 4.0\n" not in text


def test_jazzy_navfn_plugin_uses_namespaced_class_name():
    text = Path(
        "src/traymover_robot_sim/config/nav2_params_detour_sim.yaml"
    ).read_text()
    assert "plugin: nav2_navfn_planner::NavfnPlanner" in text
    assert "plugin: nav2_navfn_planner/NavfnPlanner" not in text


def test_amcl_initial_pose_matches_sim_spawn_pose():
    text = Path(
        "src/traymover_robot_sim/config/nav2_params_detour_sim.yaml"
    ).read_text()
    assert "    initial_pose:\n      x: 1.0\n      y: 0.0\n      z: 0.0\n      yaw: 0.0" in text
    assert "initial_pose_x:" not in text
    assert "initial_pose_a:" not in text
