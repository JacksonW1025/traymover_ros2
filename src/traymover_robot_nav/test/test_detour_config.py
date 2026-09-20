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


def test_collision_monitor_camera_stop_preserves_near_protection():
    baseline = config('collision_monitor_lidar_realsense.yaml')[
        'collision_monitor']['ros__parameters']
    detour = config('collision_monitor_lidar_realsense_detour.yaml')[
        'collision_monitor']['ros__parameters']
    assert detour['cmd_vel_in_topic'] == 'cmd_vel_detour'
    assert detour['odom_frame_id'] == 'base_link'
    assert detour['polygons'] == ['PolygonStop']
    assert detour['observation_sources'] == ['realsense_scan']
    assert detour['PolygonStop'] == baseline['PolygonStop']
    assert detour['realsense_scan'] == baseline['realsense_scan']
    assert detour['source_timeout'] == 1.5
    assert detour['cmd_vel_out_topic'] == baseline['cmd_vel_out_topic']
    assert 'PolygonSlow' not in detour and 'lidar_scan' not in detour


def test_option18_uses_option10_nav_params_plus_obstacle_layers_only(monkeypatch):
    baseline = config('nav2_params.yaml')
    module = load_launch(monkeypatch)
    context = launch_context(module)
    context.launch_configurations['enable_dynamic_costmaps'] = 'true'
    detour = yaml.safe_load(Path(module._navigation_params(context)).read_text())

    local_map = detour['local_costmap']['local_costmap']['ros__parameters']
    assert local_map['plugins'] == ['obstacle_layer', 'inflation_layer']
    local_dynamic = local_map.pop('obstacle_layer')
    assert local_dynamic['observation_sources'] == 'lidar_scan realsense_scan'
    local_map['plugins'].remove('obstacle_layer')

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
    assert 'RESULT:true,true,1.0,1.00,0.90,0.08,0.25' in result.stdout


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
    assert 'action_start_nav 18' in body
    body = text.split('action_start_nav() {', 1)[1].split('\n}', 1)[0]
    assert body.index('Select PCD') < body.index('Enable ground-plane fitting')
    assert body.index('cleanup_static_tf.py') < body.index('Generating runtime 2D nav map')
    assert '--z-reference "${z_reference}"' in body
    assert 'NAV_PGM_Z_MIN' in body
    assert 'NAV_OBSTACLE_PGM_Z_MIN' not in body
    assert body.index('Launch RViz') < body.index('prompt_wait_detour_settings')
    assert 'prompt_ndt_height_boost' in body
    assert 'auto_estop_enabled:=${nav_auto_estop_enabled}' in body
    assert '_option${nav_option}.' in body
    assert 'run_nav_logged.sh' in body
    assert 'auto_correction_enabled:=true' in body
    assert 'height_boost_enabled:=${NDT_HEIGHT_BOOST_ENABLED}' in body
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


def test_obstacle_launch_forwards_option10_localization_controls(monkeypatch):
    pytest.importorskip('launch_ros')
    module_path = PKG / 'launch' / 'traymover_nav_lidar_realsense_obstacle.launch.py'
    spec = importlib.util.spec_from_file_location('test_obstacle_launch', module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda name: str(PKG))

    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
    from launch.utilities import perform_substitutions

    description = module.generate_launch_description()
    context = LaunchContext()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    context.launch_configurations.update(
        height_boost_enabled='true',
        height_boost_min_z='2.75',
        auto_correction_enabled='true',
    )

    localization = None
    for action in description.entities:
        if not isinstance(action, IncludeLaunchDescription):
            continue
        location = action.launch_description_source._LaunchDescriptionSource__location
        if Path(perform_substitutions(context, location)).name == 'lidar_localization.launch.py':
            localization = dict(action.launch_arguments)
            break
    assert localization is not None
    assert localization['height_boost_enabled'].perform(context) == 'true'
    assert localization['height_boost_min_z'].perform(context) == '2.75'
    assert localization['auto_correction_enabled'].perform(context) == 'true'


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
    expected_remaps = {
        f'/{action}/_action/{endpoint}': f'/{action}_detour_backend/_action/{endpoint}'
        for action in ('navigate_to_pose', 'navigate_through_poses')
        for endpoint in ('send_goal', 'get_result', 'cancel_goal', 'feedback', 'status')
    }
    expected_remaps.update({'/goal_pose': '/goal_pose_detour_backend_unused',
                           '/cmd_vel': '/cmd_vel_nav'})
    assert {perform_substitutions(context, item.src):
            perform_substitutions(context, item.dst) for item in remaps} == expected_remaps

    includes = {}
    for item in nested:
        location = item.launch_description_source._LaunchDescriptionSource__location
        name = Path(perform_substitutions(context, location)).name
        includes[name] = dict(item.launch_arguments)
    assert list(includes).count('rs_launch.py') == 1
    assert 'traymover_nav_lidar_realsense_obstacle.launch.py' not in includes
    obstacle = includes['traymover_nav.launch.py']
    assert obstacle['publish_scan'] == 'true'
    assert includes['collision_monitor.launch.py']['params_file'].perform(context).endswith(
        'collision_monitor_lidar_realsense_detour.yaml')
    assert sum(n['executable'] == 'depthimage_to_laserscan_node' for n in captured_nodes) == 1
    assert obstacle['bringup_hardware'] == 'false'
    assert obstacle['launch_rviz'] == 'false'  # RViz public goal action stays outside remap scope.
    assert obstacle['height_boost_enabled'].perform(context) == 'false'
    assert obstacle['height_boost_min_z'].perform(context) == '2.0'
    assert obstacle['auto_correction_enabled'].perform(context) == 'false'
    assert includes['base_serial.launch.py']['auto_estop_enabled'] == auto
    if auto == 'true':
        estop = includes['traymover_auto_estop.launch.py']
        assert estop['start_realsense_driver'] == 'false'
        assert estop['depth_timeout_sec'] == '1.5'
        assert estop['min_near_pixel_fraction'] == '0.02'
        assert estop['min_near_tile_count'] == '4'
        assert estop['obstacle_frame_count'] == '3'
        assert estop['low_valid_frame_count'] == '4'
        assert estop['clear_frame_count'] == '12'
        assert estop['min_valid_fraction'] == '0.25'
        assert estop['severe_invalid_fraction'] == '0.10'
    else:
        assert 'traymover_auto_estop.launch.py' not in includes
    rewritten = yaml.safe_load(Path(obstacle['params_file']).read_text())
    assert rewritten == config('nav2_params.yaml')
    assert params['simple_detour'] is True
    assert params['wait_before_detour_sec'].evaluate(context) == 1.0
    assert params['detour_trigger_distance_m'].evaluate(context) == 1.0
    assert params['detour_guard_distance_m'].evaluate(context) == 0.9
    assert params['simple_obstacle_half_width_m'].evaluate(context) == 0.35
    assert params['obstacle_confirm_frames'].evaluate(context) == 3
    assert params['min_obstacle_points'].evaluate(context) == 8
    assert params['detour_first_leg_m'].evaluate(context) == 2.2
    assert params['detour_turn_angle_deg'].evaluate(context) == 45.0
    assert params['straight_phase_timeout_sec'].evaluate(context) == 60.0
    assert params['scan_timeout_sec'].evaluate(context) == 2.0
    assert params['realsense_timeout_sec'].evaluate(context) == 1.5
    assert obstacle['lifecycle_manager_executable'] == 'option18_lifecycle_manager'
    assert includes['collision_monitor.launch.py']['lifecycle_manager_executable'] == (
        'option18_lifecycle_manager')
    depth = next(n for n in captured_nodes if n['executable'] == 'depthimage_to_laserscan_node')
    assert depth['parameters'][0]['scan_height'].evaluate(context) == 320


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


def launch_context(module):
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument

    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    return context


def test_option18_inherits_future_baseline_changes_without_copying(monkeypatch, tmp_path):
    module = load_launch(monkeypatch)
    context = launch_context(module)
    baseline = config('nav2_params.yaml')
    baseline['controller_server']['ros__parameters']['FollowPath']['desired_linear_vel'] = 0.27
    baseline['bt_navigator']['ros__parameters']['future_parameter'] = 'inherited'
    baseline['local_costmap']['local_costmap']['ros__parameters']['resolution'] = 0.07
    path = tmp_path / 'custom_nav2.yaml'
    path.write_text(yaml.safe_dump(baseline))
    context.launch_configurations.update(
        params_file=str(path), dynamic_obstacle_persistence_sec='2.5',
        enable_dynamic_costmaps='true')
    actual = yaml.safe_load(Path(module._navigation_params(context)).read_text())
    for name in ('local_costmap', 'global_costmap'):
        costmap = actual[name][name]['ros__parameters']
        layer = costmap.pop('obstacle_layer')
        costmap['plugins'].remove('obstacle_layer')
        if name == 'global_costmap':
            assert all(layer[source]['observation_persistence'] == 2.5
                       for source in layer['observation_sources'].split())
    assert actual == baseline
    assert yaml.safe_load(path.read_text()) == baseline


def test_option18_overlay_contains_no_copied_navigation_settings():
    overlay = config('nav2_params_lidar_realsense_detour.yaml')
    assert set(overlay) == {'local_costmap', 'global_costmap'}
    for name in overlay:
        assert set(overlay[name][name]['ros__parameters']) == {'obstacle_layer'}


def test_option18_default_uses_exact_option10_parameter_file(monkeypatch):
    module = load_launch(monkeypatch)
    context = launch_context(module)
    path = module._navigation_params(context)
    assert Path(path) == PKG / 'config' / 'nav2_params.yaml'
    assert context.launch_configurations['enable_dynamic_costmaps'] == 'false'
    assert context.launch_configurations['simple_detour'] == 'true'


def test_actual_option18_defaults_require_eight_camera_points_in_three_frames(monkeypatch):
    import sys
    from dataclasses import fields

    sys.path.insert(0, str(PKG / 'scripts'))
    from detour_core import Config, DetourGate, Observation, Ray, Scan, Velocity

    module = load_launch(monkeypatch)
    context = launch_context(module)
    captured = []
    monkeypatch.setattr(module, 'Node', lambda **kwargs: captured.append(kwargs) or kwargs)
    module._launch_nodes(context)
    parameters = next(node['parameters'][0] for node in captured
                      if node['executable'] == 'detour_supervisor.py')
    names = {field.name for field in fields(Config)}
    settings = {name: value.evaluate(context) if hasattr(value, 'evaluate') else value
                for name, value in parameters.items() if name in names}
    gate = DetourGate(Config(**settings))
    for frame, count in enumerate([7, 7, 7, 8, 8, 8], 1):
        rays = tuple(Ray(0.45, 0.0, math.radians((i - 4) * 0.1), 0.95,
                         safety_distance_m=0.95) for i in range(count))
        observation = Observation(
            realsense=Scan(float(frame), rays), nav_cmd=Velocity(0.35, 0.0),
            nav_cmd_stamp=float(frame), nav_goal_active=True,
            auto_reason='NONE', estop_fresh=True)
        command = gate.tick(float(frame), observation)
        if frame < 6:
            assert gate.state == 'NORMAL'
            assert command is observation.nav_cmd
        else:
            assert gate.state == 'WAITING'
            assert command == Velocity()
