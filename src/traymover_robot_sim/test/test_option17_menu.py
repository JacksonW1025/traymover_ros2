from pathlib import Path

ROOT = Path(__file__).parents[3]


def test_option17_is_simulation_only_and_has_detour_prompt():
    text = (ROOT / "scripts/traymover.sh").read_text()
    assert "17) Simulation: Dynamic obstacle stop and detour" in text
    assert "17) action_start_nav_detour_sim" in text
    assert "enable_detour" in text
    option_start = text.index("action_start_nav_detour_sim()")
    option_end = text.index("action_start_nav_speed_modes()", option_start)
    block = text[option_start:option_end]
    assert "traymover_robot_sim traymover_detour_sim.launch.py" in block
    assert "Select destination" in block
    assert "goal_x:=${goal_x}" in block
    assert "goal_y:=${goal_y}" in block
    assert "goal_yaw:=${goal_yaw}" in block
    assert "East center" in block
    assert "base_serial.launch.py" not in block
    assert "traymover_lidar.launch.py" not in block


def test_launcher_selects_ros_setup_with_override_and_distro_fallbacks():
    text = (ROOT / "scripts/traymover.sh").read_text()
    assert 'ROS_DISTRO_SETUP="${ROS_DISTRO_SETUP:-}"' in text
    assert "/opt/ros/humble/setup.bash" in text
    assert "/opt/ros/jazzy/setup.bash" in text
    assert "No ROS 2 setup file found" in text


def test_option17_prefers_jazzy_without_overriding_explicit_setup():
    text = (ROOT / "scripts/traymover.sh").read_text()
    option_start = text.index("action_start_nav_detour_sim()")
    option_end = text.index("action_start_nav_speed_modes()", option_start)
    block = text[option_start:option_end]
    assert "ROS_DISTRO_SETUP_EXPLICIT" in text
    assert "/opt/ros/jazzy/setup.bash" in block
    assert "ROS_DISTRO_SETUP=\"${previous_ros_setup}\"" in block


def test_option0_cleans_option17_simulation_processes():
    text = (ROOT / "scripts/traymover.sh").read_text()
    patterns_start = text.index("KILL_PATTERNS=(")
    patterns_end = text.index(")", patterns_start)
    patterns = text[patterns_start:patterns_end]
    assert "ros2 launch traymover_robot_sim" in patterns
    assert "traymover_detour.sdf" in patterns
    assert "sim_odom_tf" in patterns
    assert "detour_supervisor" in patterns
    assert "demo_goal_sender" in patterns


def test_option0_cleans_gazebo_sim_server_and_gui_processes():
    text = (ROOT / "scripts/traymover.sh").read_text()
    patterns_start = text.index("KILL_PATTERNS=(")
    patterns_end = text.index(")", patterns_start)
    patterns = text[patterns_start:patterns_end]
    # Gazebo Sim execs into ruby and drops the world filename from argv.  The
    # server/gui command names must therefore be cleanup targets themselves.
    assert "gz sim server" in patterns
    assert "gz sim gui" in patterns
    assert "traymover: detour_sim" in patterns


def test_option17_resets_previous_simulation_before_prompting():
    text = (ROOT / "scripts/traymover.sh").read_text()
    option_start = text.index("action_start_nav_detour_sim()")
    option_end = text.index("action_start_nav_speed_modes()", option_start)
    block = text[option_start:option_end]
    assert "kill_previous true" in block
    assert block.index("kill_previous true") < block.index("Select destination")


def test_spawned_terminal_closes_after_signal_terminated_command():
    text = (ROOT / "scripts/traymover.sh").read_text()
    spawn_start = text.index("spawn_in_terminal()")
    spawn_end = text.index("# ---- clean slate", spawn_start)
    block = text[spawn_start:spawn_end]
    # A command killed by option 0 exits with 128+signal; that path must not
    # wait for interactive input, otherwise the terminal window stays open.
    assert "status=$?" in block
    assert r'if [ \"\${status}\" -lt 128 ]' in block
    # Keep status expansion in the spawned shell; expanding it while building
    # the wrapper under `set -u` would abort every launcher action.
    assert r'\${status}' in block


def test_option17_uses_rviz_config_with_top_down_view():
    text = (ROOT / "src/traymover_robot_sim/rviz/traymover_detour.rviz").read_text()
    assert "Class: rviz_default_plugins/TopDownOrtho" in text
    assert "Background Color: 230; 235; 245" in text
