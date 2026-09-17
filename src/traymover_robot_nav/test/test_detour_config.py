"""Option 18 menu/config/launch contracts; never launches hardware or ROS nodes."""
import importlib.util
import math
from pathlib import Path
import subprocess

import pytest
import yaml


PKG = Path(__file__).resolve().parents[1]
WORKSPACE = PKG.parent.parent
MENU = WORKSPACE / 'scripts' / 'traymover.sh'


def config(name):
    return yaml.safe_load((PKG / 'config' / name).read_text())


def test_collision_monitor_only_changes_input_topic():
    baseline = config('collision_monitor_lidar_realsense.yaml')
    detour = config('collision_monitor_lidar_realsense_detour.yaml')
    assert detour['collision_monitor']['ros__parameters']['cmd_vel_in_topic'] == 'cmd_vel_detour'
    detour['collision_monitor']['ros__parameters']['cmd_vel_in_topic'] = 'cmd_vel_nav'
    assert detour == baseline


def test_global_dynamic_observations_are_separate_from_static_map():
    baseline = config('nav2_params_lidar_realsense_obstacle.yaml')
    detour = config('nav2_params_lidar_realsense_detour.yaml')
    global_map = detour['global_costmap']['global_costmap']['ros__parameters']
    assert global_map['plugins'] == ['static_layer', 'obstacle_layer', 'inflation_layer']
    dynamic = global_map.pop('obstacle_layer')
    assert dynamic['observation_sources'] == 'lidar_scan realsense_scan'
    for name, topic in [('lidar_scan', '/scan'), ('realsense_scan', '/scan_realsense')]:
        source = dynamic[name]
        assert source['topic'] == topic
        assert source['marking'] and source['clearing']
        assert source['observation_persistence'] == 5.0
        assert source['inf_is_valid'] is True
        assert source['raytrace_max_range'] > source['obstacle_max_range']
    global_map['plugins'].remove('obstacle_layer')
    assert detour == baseline


def run_prompts(answers):
    helper = MENU.read_text().split('prompt_wait_detour_settings() {', 1)[1].split('\n}', 1)[0]
    script = 'set -u\nprompt_wait_detour_settings() {' + helper + '\n}\n'
    script += '''prompt_wait_detour_settings || exit $?
printf 'RESULT:%s,%s,%s,%s,%s,%s,%s\\n' "$DETOUR_AUTO_ESTOP_ENABLED" "$DETOUR_ENABLED" \\
    "$DETOUR_WAIT_SEC" "$DETOUR_TRIGGER_M" "$DETOUR_GUARD_M" \\
    "$DETOUR_LINEAR_SPEED" "$DETOUR_ANGULAR_SPEED"
IFS= read -r remainder || true
printf 'REMAINDER:%s\\n' "${remainder:-}"
'''
    return subprocess.run(['bash', '-c', script], input=answers, text=True, capture_output=True)


def test_enter_defaults_enable_auto_and_detour():
    result = run_prompts('\n' * 7)
    assert result.returncode == 0, result.stderr
    assert 'RESULT:true,true,5.0,1.10,0.95,0.08,0.25' in result.stdout


def test_auto_disabled_does_not_prompt_or_consume_detour_answer():
    result = run_prompts('n\nsentinel\n')
    assert result.returncode == 0, result.stderr
    assert 'RESULT:false,false,' in result.stdout
    assert 'REMAINDER:sentinel' in result.stdout
    assert '[traymover] Wait-and-detour requires automatic EStop.' in result.stdout
    assert '[traymover] Detour has been forced OFF.' in result.stdout


def test_detour_can_be_disabled_while_auto_stays_on():
    result = run_prompts('y\nn\nsentinel\n')
    assert result.returncode == 0
    assert 'RESULT:true,false,' in result.stdout
    assert 'REMAINDER:sentinel' in result.stdout


@pytest.mark.parametrize('settings', [
    '5\n0.95\n1.1\n0.08\n0.25\n',
    '5\n1.1\n0.8\n0.08\n0.25\n',
    '5\n1.1\n0.95\n0.5\n0.25\n',
    '5\n1.1\n0.95\n0.08\n0.5\n',
    '$(touch /tmp/detour-should-never-run)\n1.1\n0.95\n0.08\n0.25\n',
])
def test_unsafe_and_non_numeric_menu_settings_rejected_before_startup(settings):
    result = run_prompts('y\ny\n' + settings)
    assert result.returncode != 0
    assert 'RESULT:' not in result.stdout


def test_option18_launch_chain_and_existing_menu_routes():
    text = MENU.read_text()
    assert '18) Navigation with LiDAR + RealSense + EStop-safe Wait Detour' in text
    for choice, function in [(10, 'nav'), (15, 'nav_speed_modes'),
                             (16, 'nav_lidar_realsense_obstacle')]:
        assert f'{choice}) action_start_{function} ;;' in text
    assert '17) action_test_auto_estop ;;' in text
    assert '18) action_start_nav_wait_detour ;;' in text
    body = text.split('action_start_nav_wait_detour() {', 1)[1].split('\n}', 1)[0]
    assert body.index('Select PCD') < body.index('Launch RViz')
    assert body.index('Launch RViz') < body.index('prompt_wait_detour_settings')
    assert '--z-reference absolute' in body
    assert 'auto_estop_enabled:=${DETOUR_AUTO_ESTOP_ENABLED}' in body
    assert 'action_start_auto_estop' not in body
    assert 'traymover_nav_lidar_realsense_detour.launch.py' in body


def load_launch(monkeypatch):
    pytest.importorskip('launch_ros')
    module_path = PKG / 'launch' / 'traymover_nav_lidar_realsense_detour.launch.py'
    spec = importlib.util.spec_from_file_location('test_detour_launch', module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory',
                        lambda name: str(PKG.parent / name))
    return module


@pytest.mark.parametrize('auto,detour,expected', [
    ('true', 'true', True), ('true', 'false', False), ('false', 'false', False),
    ('false', 'true', False),
])
def test_launch_dependency_single_camera_and_scoped_action_proxy(
        monkeypatch, auto, detour, expected):
    module = load_launch(monkeypatch)
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
    from launch.utilities import perform_substitutions
    from launch_ros.actions import SetRemap

    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    context.launch_configurations.update(enable_auto_estop=auto, enable_detour=detour)
    captured_nodes = []
    monkeypatch.setattr(module, 'Node', lambda **kwargs: captured_nodes.append(kwargs) or kwargs)
    actions = module._launch_nodes(context)
    supervisor = next(n for n in captured_nodes if n['executable'] == 'detour_supervisor.py')
    params = supervisor['parameters'][0]
    assert params['enable_detour'] is expected
    assert params['enable_auto_estop'] is (auto == 'true')
    assert params['localization_timeout_sec'].evaluate(context) == 5.0
    perimeter = [n for n in captured_nodes if n['name'] == 'traymover_detour_perimeter_scan']
    assert len(perimeter) == int(expected)
    if expected:
        projector = perimeter[0]
        assert projector['package'] == 'pointcloud_to_laserscan'
        assert projector['executable'] == 'pointcloud_to_laserscan_node'
        assert dict(projector['remappings']) == {
            'cloud_in': '/point_cloud_raw', 'scan': '/scan_detour_perimeter',
        }
        projection = projector['parameters'][0]
        assert projection['target_frame'] == 'base_link'
        assert projection['angle_min'] == -math.pi
        assert projection['angle_max'] == math.pi
        assert projection['range_min'] == 0.3
        assert projection['range_max'] == 60.0
        assert projection['min_height'] == -0.25
        assert projection['max_height'] == 1.05
        assert projection['use_inf'] is True

    nested = []
    remaps = []
    for action in actions:
        members = action.get_sub_entities() if isinstance(action, GroupAction) else [action]
        nested.extend(m for m in members if isinstance(m, IncludeLaunchDescription))
        remaps.extend(m for m in members if isinstance(m, SetRemap))
    assert len(remaps) == 1
    assert perform_substitutions(context, remaps[0].src) == '/navigate_to_pose'
    assert perform_substitutions(context, remaps[0].dst) == '/navigate_to_pose_detour_backend'

    includes = {}
    for item in nested:
        location = item.launch_description_source._LaunchDescriptionSource__location
        name = Path(perform_substitutions(context, location)).name
        includes[name] = dict(item.launch_arguments)
    assert list(includes).count('rs_launch.py') == 1
    obstacle = includes['traymover_nav_lidar_realsense_obstacle.launch.py']
    assert obstacle['start_realsense_driver'] == 'false'
    assert obstacle['bringup_hardware'] == 'false'
    assert obstacle['launch_rviz'] == 'false'  # RViz public goal action stays outside remap scope.
    assert includes['base_serial.launch.py']['auto_estop_enabled'] == auto
    if auto == 'true':
        assert includes['traymover_auto_estop.launch.py']['start_realsense_driver'] == 'false'
    else:
        assert 'traymover_auto_estop.launch.py' not in includes
    rewritten = yaml.safe_load(Path(obstacle['params_file']).read_text())
    global_map = rewritten['global_costmap']['global_costmap']['ros__parameters']
    assert global_map['obstacle_layer']['lidar_scan']['observation_persistence'] == 5.0


@pytest.mark.parametrize('name,value', [
    ('dynamic_obstacle_persistence_sec', '0'),
    ('dynamic_obstacle_persistence_sec', '-1'),
    ('dynamic_obstacle_persistence_sec', 'nan'),
    ('dynamic_obstacle_persistence_sec', 'inf'),
    ('detour_guard_distance_m', '0.8'),
    ('detour_guard_distance_m', '1.2'),
    ('detour_max_linear_speed', '0.5'),
    ('detour_max_angular_speed', '0.5'),
    ('obstacle_confirm_frames', '1.5'),
])
def test_launch_rejects_invalid_values_before_creating_nodes(monkeypatch, name, value):
    module = load_launch(monkeypatch)
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument

    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    context.launch_configurations[name] = value
    captured_nodes = []
    monkeypatch.setattr(module, 'Node', lambda **kwargs: captured_nodes.append(kwargs) or kwargs)
    with pytest.raises(ValueError):
        module._launch_nodes(context)
    assert captured_nodes == []
