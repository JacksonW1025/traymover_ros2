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
