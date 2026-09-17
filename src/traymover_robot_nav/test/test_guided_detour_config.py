"""No-hardware contracts for Option 19's isolated sensors/safety launch."""
import importlib.util
import math
import os
from pathlib import Path
import signal
import subprocess

import pytest
import yaml


PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE.parent.parent


def parameters(filename):
    return yaml.safe_load((PACKAGE / 'config' / filename).read_text())


def test_test_collision_monitor_keeps_production_safety_geometry():
    production = parameters('collision_monitor_lidar_realsense_detour.yaml')
    guided = parameters('collision_monitor_guided_detour_test.yaml')
    params = guided['collision_monitor']['ros__parameters']
    baseline = production['collision_monitor']['ros__parameters']
    assert params['cmd_vel_in_topic'] == '/cmd_vel_detour_test'
    assert params['cmd_vel_out_topic'] == '/cmd_vel'
    assert params['odom_frame_id'] == params['base_frame_id'] == 'base_link'
    session = parameters('guided_detour_test.yaml')['guided_detour_test']['ros__parameters']
    assert params['stop_pub_timeout'] > session['max_test_duration_sec']
    for key in ('cmd_vel_in_topic', 'cmd_vel_out_topic', 'odom_frame_id', 'stop_pub_timeout'):
        params[key] = baseline[key]
    assert guided == production


def test_guided_defaults_are_bounded_and_keep_estop_margin():
    params = parameters('guided_detour_test.yaml')['guided_detour_test']['ros__parameters']
    assert params['detour_trigger_distance_m'] == 1.10
    assert params['detour_guard_distance_m'] == 0.95
    assert params['test_wait_sec'] == 2.0
    assert params['test_turn_angle_deg'] == 25.0
    assert params['test_turn_speed'] == 0.15
    assert params['test_turn_tolerance_deg'] == 5.0
    assert params['test_turn_timeout_sec'] == 5.0
    assert params['test_straight_speed'] == 0.03
    assert params['test_straight_duration'] == 1.5
    assert params['test_straight_timeout_sec'] == 2.0
    assert params['estop_test_speed'] == 0.02
    assert params['estop_test_max_duration'] == 5.0
    assert params['max_total_forward_command_sec'] == 5.0
    assert params['max_total_rotation_deg'] == 90.0
    assert params['imu_timeout_sec'] == 0.5
    assert params['scan_timeout_sec'] == params['realsense_timeout_sec'] == 1.0
    assert all(math.isfinite(value) and value > 0.0 for value in params.values())
    assert 'enable_auto_estop' not in params
    assert 'stop_distance_m' not in params


def test_menu_foreground_runner_has_only_visualization_startup_question():
    menu = (WORKSPACE / 'scripts' / 'traymover.sh').read_text()
    assert '19) Guided EStop-safe Detour Test (No Map / No Nav2)' in menu
    assert '19) action_start_guided_detour_test ;;' in menu
    body = menu.split('action_start_guided_detour_test() {', 1)[1].split('\n}', 1)[0]
    assert body.count('read -r -p') == 1
    assert 'Launch sensor visualization/RViz? [Y/n]' in body
    assert 'setsid ros2 launch traymover_robot_nav guided_detour_test.launch.py' in body
    assert 'ros2 run traymover_robot_nav guided_detour_test.py --ros-args' in body
    assert '--params-file "${NAV_CONFIG_DIR}/guided_detour_test.yaml"' in body
    assert 'trap cleanup_guided_session EXIT' in body
    assert 'kill -INT -- "-${sensor_pid}"' in body
    assert 'kill -TERM -- "-${sensor_pid}"' in body
    assert 'kill -KILL -- "-${sensor_pid}"' in body
    assert 'trap' not in menu.split('action_start_guided_detour_test() {', 1)[0]
    for forbidden in ('Select PCD', 'pcd2pgm.py', 'prompt_auto_estop',
                      'spawn_in_terminal', 'action_start_nav', 'pkill '):
        assert forbidden not in body
    subprocess.run(['bash', '-n', str(WORKSPACE / 'scripts' / 'traymover.sh')], check=True)


def test_rviz_uses_available_robot_root_and_sensor_topics():
    rviz = yaml.safe_load((PACKAGE / 'rviz' / 'guided_detour_test.rviz').read_text())
    manager = rviz['Visualization Manager']
    assert manager['Global Options']['Fixed Frame'] == 'base_link'
    displays = manager['Displays']
    scans = {item['Topic']['Value'] for item in displays
             if item['Class'] == 'rviz_default_plugins/LaserScan'}
    assert scans == {'/scan', '/scan_realsense'}
    assert any(item['Class'] == 'rviz_default_plugins/RobotModel' for item in displays)
    assert any(item['Class'] == 'rviz_default_plugins/TF' for item in displays)
    assert all('Map' not in item['Class'] for item in displays)
    assert not any('SetGoal' in tool['Class'] for tool in manager['Tools'])
    assert any(panel['Class'] == 'traymover_robot_safety/EstopPanel'
               for panel in rviz['Panels'])


def test_menu_cleans_only_owned_launch_and_preserves_parent_trap(tmp_path):
    """Exercise real process-group cleanup using fake ROS commands, never ROS."""
    menu = (WORKSPACE / 'scripts' / 'traymover.sh').read_text()
    function = 'action_start_guided_detour_test() {' + menu.split(
        'action_start_guided_detour_test() {', 1)[1].split('\n}', 1)[0] + '\n}'
    binary = tmp_path / 'bin'
    binary.mkdir()
    prefix = tmp_path / 'installed'
    executable = prefix / 'lib/traymover_robot_nav/guided_detour_test.py'
    executable.parent.mkdir(parents=True)
    executable.touch()
    executable.chmod(0o755)
    setup = tmp_path / 'setup.bash'
    setup.touch()
    pidfile = tmp_path / 'sensor.pid'
    trace = tmp_path / 'commands'
    ros2 = binary / 'ros2'
    ros2.write_text('''#!/usr/bin/python3
import os
from pathlib import Path
import signal
import sys
import time
with open(os.environ['FAKE_TRACE'], 'a') as stream:
    stream.write(' '.join(sys.argv[1:]) + '\\n')
if sys.argv[1:3] == ['pkg', 'prefix']:
    print(os.environ['FAKE_PREFIX'])
elif sys.argv[1] == 'launch':
    signal.signal(signal.SIGINT, lambda *args: sys.exit(0))
    signal.signal(signal.SIGTERM, lambda *args: sys.exit(0))
    Path(os.environ['FAKE_SENSOR_PID']).write_text(str(os.getpid()))
    time.sleep(30)
elif sys.argv[1] == 'run':
    for attempt in range(100):
        if Path(os.environ['FAKE_SENSOR_PID']).exists():
            break
        time.sleep(0.01)
    sys.exit(7)
''')
    ros2.chmod(0o755)
    script = '''set -u
WORKSPACE_DIR="$1"
ROS_DISTRO_SETUP="$2"
WS_SETUP="$2"
NAV_CONFIG_DIR="$3"
ROS_LOG_DIR_DEFAULT="$1/roslogs"
pick_terminal() { printf ''; }
trap ':' TERM
original_trap="$(trap -p TERM)"
''' + function + '''
action_start_guided_detour_test
status=$?
[[ "$(trap -p TERM)" == "$original_trap" ]] || exit 90
exit "$status"
'''
    env = dict(os.environ, PATH=str(binary) + os.pathsep + os.environ['PATH'],
               FAKE_TRACE=str(trace), FAKE_PREFIX=str(prefix), FAKE_SENSOR_PID=str(pidfile))
    try:
        result = subprocess.run(
            ['bash', '-c', script, 'guided-test', str(tmp_path), str(setup),
             str(PACKAGE / 'config')], input='n\n', text=True, capture_output=True,
            env=env, timeout=15)
        assert result.returncode == 7, result.stdout + result.stderr
        assert pidfile.exists()
        with pytest.raises(ProcessLookupError):
            os.killpg(int(pidfile.read_text()), 0)
        commands = trace.read_text()
        assert ('launch traymover_robot_nav guided_detour_test.launch.py '
                'launch_rviz:=false') in commands
        assert 'run traymover_robot_nav guided_detour_test.py --ros-args' in commands
        assert not list((tmp_path / 'logs').glob('.detour_session_*'))
    finally:
        if pidfile.exists():
            try:
                os.killpg(int(pidfile.read_text()), signal.SIGKILL)
            except ProcessLookupError:
                pass


def test_standalone_launch_requires_safety_without_navigation(monkeypatch):
    pytest.importorskip('launch_ros')
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
    from launch.utilities import perform_substitutions

    path = PACKAGE / 'launch' / 'guided_detour_test.launch.py'
    spec = importlib.util.spec_from_file_location('guided_launch_contract', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory',
                        lambda name: str(PACKAGE.parent / name))
    nodes = []
    monkeypatch.setattr(module, 'Node', lambda **kwargs: nodes.append(kwargs) or kwargs)
    actions = module.generate_launch_description().entities
    context = LaunchContext()
    declared = [action for action in actions if isinstance(action, DeclareLaunchArgument)]
    assert [action.name for action in declared] == ['launch_rviz']
    for action in declared:
        action.execute(context)
    nested = []
    for action in actions:
        nested.extend(action.get_sub_entities() if isinstance(action, GroupAction) else [action])
    includes = {}
    for item in nested:
        if isinstance(item, IncludeLaunchDescription):
            location = item.launch_description_source._LaunchDescriptionSource__location
            name = Path(perform_substitutions(context, location)).name
            assert name not in includes
            includes[name] = dict(item.launch_arguments)
    assert set(includes) == {
        'base_serial.launch.py', 'traymover_lidar.launch.py', 'rs_launch.py',
        'traymover_auto_estop.launch.py', 'collision_monitor.launch.py',
    }
    assert includes['base_serial.launch.py'] == {
        'use_imu': 'true', 'odom_source_mode': 'none', 'auto_estop_enabled': 'true',
    }
    assert includes['traymover_lidar.launch.py']['enable_scan_bridge'] == 'false'
    assert includes['traymover_auto_estop.launch.py']['start_realsense_driver'] == 'false'
    assert 'stop_distance_m' not in includes['traymover_auto_estop.launch.py']
    assert includes['collision_monitor.launch.py']['params_file'].endswith(
        '/config/collision_monitor_guided_detour_test.yaml')
    assert {node['package'] for node in nodes} == {
        'robot_state_publisher', 'joint_state_publisher', 'depthimage_to_laserscan',
        'pointcloud_to_laserscan', 'rviz2',
    }
    projectors = [node for node in nodes if node['package'] == 'pointcloud_to_laserscan']
    assert {dict(node['remappings'])['scan'] for node in projectors} == {
        '/scan', '/scan_detour_perimeter',
    }
    for node in projectors:
        assert dict(node['remappings'])['cloud_in'] == '/point_cloud_raw'
        params = node['parameters'][0]
        assert params['target_frame'] == 'base_link'
        assert params['angle_min'] == -math.pi
        assert params['angle_max'] == math.pi
        assert params['min_height'] == -0.25
        assert params['max_height'] == 1.05
