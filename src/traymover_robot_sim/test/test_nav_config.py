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


def test_detour_supervisor_runs_in_disabled_mode_and_reports_normal():
    text = Path(
        "src/traymover_robot_sim/launch/traymover_detour_sim.launch.py"
    ).read_text()
    assert 'name="detour_supervisor"' in text
    assert '"enable_detour": enable_detour' in text
    assert 'condition=IfCondition(enable_detour)' not in text
