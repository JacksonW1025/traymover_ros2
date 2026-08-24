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


def test_launcher_selects_ros_setup_with_override_and_distro_fallbacks():
    text = Path("scripts/traymover.sh").read_text()
    assert 'ROS_DISTRO_SETUP="${ROS_DISTRO_SETUP:-}"' in text
    assert "/opt/ros/humble/setup.bash" in text
    assert "/opt/ros/jazzy/setup.bash" in text
    assert "No ROS 2 setup file found" in text
