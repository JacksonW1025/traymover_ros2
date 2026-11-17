#!/usr/bin/env python3
import copy
import os
from pathlib import Path
import subprocess

import pytest
import yaml


PKG_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PKG_ROOT.parent
WORKSPACE_ROOT = SRC_ROOT.parent


def _run_option10_prompt(tmp_path, answers, converter_status=0, runs=1, cleanup_status=0,
                         option=10):
    """Execute the real menu function, replacing hardware and ROS side effects."""
    launcher = (WORKSPACE_ROOT / 'scripts/traymover.sh').read_text()
    body = launcher.split('action_start_nav() {', 1)[1].split('\n}', 1)[0]
    pcd_dir = tmp_path / 'pcd'
    pcd_dir.mkdir()
    (pcd_dir / 'example.pcd').write_text('test input')
    setup = tmp_path / 'setup.bash'
    setup.write_text(':\n')
    arguments = tmp_path / 'arguments'
    hardware = tmp_path / 'hardware'
    env = dict(os.environ, FASTLIO_PCD_DIR=str(pcd_dir),
               WORKSPACE_DIR=str(WORKSPACE_ROOT), SCRIPT_DIR=str(WORKSPACE_ROOT / 'scripts'),
               TRAYMOVER_NAV_LOG_ROOT=str(tmp_path / 'nav_logs'),
               NAV_RUNTIME_DIR=str(tmp_path / 'maps'),
               ROS_LOG_DIR_DEFAULT=str(tmp_path / 'logs'),
               ROS_DISTRO_SETUP=str(setup), WS_SETUP=str(setup),
               NAV_SCRIPTS_DIR=str(PKG_ROOT / 'scripts'), NAV_PGM_Z_MIN='0.10',
               NAV_PGM_Z_MAX='2.20', NAV_PGM_MIN_POINTS='1', NAV_PGM_DILATE='2',
               NAV_PGM_MIN_REGION_CELLS='2', AUTO_ESTOP_ENABLED='false',
               CAPTURE_ARGS=str(arguments), CAPTURE_HARDWARE=str(hardware),
               CONVERTER_STATUS=str(converter_status), CLEANUP_STATUS=str(cleanup_status))
    stubs = r'''
python3() {
    printf '%s\n' "$@" >> "$CAPTURE_ARGS"
    if [[ "$1" == */cleanup_static_tf.py ]]; then return "$CLEANUP_STATUS"; fi
    if [ "$CONVERTER_STATUS" != 0 ]; then return "$CONVERTER_STATUS"; fi
    while [ "$#" -gt 0 ]; do
        if [ "$1" = --out ]; then
            printf 'image: fixture.pgm\n' > "${2}.yaml"
            break
        fi
        shift
    done
}
spawn_in_terminal() { printf '%s\n' "$*" >> "$CAPTURE_HARDWARE"; }
prompt_auto_estop() { :; }
action_start_estop_keyboard() { :; }
action_start_auto_estop() { :; }
sleep() { :; }
'''
    prompt = launcher.split('prompt_ndt_height_boost() {', 1)[1].split('\n}', 1)[0]
    script = stubs + '\nprompt_ndt_height_boost() {' + prompt + '\n}\n'
    script += '\naction_start_nav() {' + body + '\n}\n'
    script += f'action_start_nav {option}\n' * runs
    result = subprocess.run(['bash', '-u', '-c', script], input=answers, env=env,
                            text=True, capture_output=True, timeout=10)
    args = arguments.read_text().splitlines() if arguments.exists() else []
    modes = [args[i + 1] for i, arg in enumerate(args) if arg == '--z-reference']
    return result, modes, hardware.exists()


@pytest.mark.parametrize('answer,mode', [
    ('y', 'plane'), ('Y', 'plane'), ('yes', 'plane'), ('YES', 'plane'),
    ('n', 'floor'), ('N', 'floor'), ('no', 'floor'), ('', 'floor'),
    ('invalid\ny', 'plane'),
])
def test_option10_ground_prompt_passes_this_runs_choice(tmp_path, answer, mode):
    result, modes, hardware = _run_option10_prompt(tmp_path, '1\n' + answer + '\nn\n\n')
    assert result.returncode == 0, result.stdout + result.stderr
    assert modes == [mode]
    assert hardware
    if answer.startswith('invalid'):
        assert 'Please enter y or n' in result.stdout


def test_option10_automatically_starts_logged_navigation(tmp_path):
    result, _, hardware = _run_option10_prompt(tmp_path, '1\nn\nn\n\n')
    assert result.returncode == 0, result.stderr
    assert hardware
    commands = (tmp_path / 'hardware').read_text()
    assert 'run_nav_logged.sh' in commands
    assert 'ros2 launch traymover_robot_nav traymover_nav.launch.py' in commands
    assert 'test_nav_replay.sh' not in commands
    assert 'ros2 bag play' not in commands
    sessions = list((tmp_path / 'nav_logs').glob('*_option10.*'))
    assert len(sessions) == 1
    assert 'use_sim_time=false' in (sessions[0] / 'run_info.txt').read_text()
    assert (sessions[0] / 'map.yaml').is_file()


def test_option10_repeated_runs_have_separate_log_directories(tmp_path):
    result, _, _ = _run_option10_prompt(tmp_path, '1\nn\nn\n\n' * 2, runs=2)
    assert result.returncode == 0, result.stderr
    assert len(list((tmp_path / 'nav_logs').glob('*_option10.*'))) == 2


def test_option10_ground_prompt_eof_aborts_before_conversion(tmp_path):
    result, modes, hardware = _run_option10_prompt(tmp_path, '1\n')
    assert result.returncode != 0
    assert modes == []
    assert not hardware


def test_option10_ground_fit_failure_aborts_before_hardware(tmp_path):
    result, modes, hardware = _run_option10_prompt(tmp_path, '1\ny\n', converter_status=1)
    assert result.returncode != 0
    assert modes == ['plane']
    assert not hardware


def test_option10_ground_choice_does_not_leak_into_next_run(tmp_path):
    result, modes, _ = _run_option10_prompt(tmp_path, '1\ny\nn\n\n1\n\nn\n\n', runs=2)
    assert result.returncode == 0, result.stdout + result.stderr
    assert modes == ['plane', 'floor']


class UniqueKeyLoader(yaml.SafeLoader):
    pass


def construct_mapping_without_duplicates(loader, node, deep=False):
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in mapping:
            raise AssertionError(f'duplicate YAML key: {key}')
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    construct_mapping_without_duplicates,
)


def load_unique_yaml(path):
    with path.open('r', encoding='utf-8') as f:
        return yaml.load(f, Loader=UniqueKeyLoader)


def assert_close(actual, expected, tolerance=1.0e-9):
    assert abs(actual - expected) <= tolerance


def test_nav2_option10_has_no_dynamic_obstacle_stop_chain():
    params = load_unique_yaml(PKG_ROOT / 'config' / 'nav2_params.yaml')
    controller_params = params['controller_server']['ros__parameters']
    follow_path = controller_params['FollowPath']
    progress_checker = controller_params['progress_checker']
    bt_params = params['bt_navigator']['ros__parameters']
    local_costmap = params['local_costmap']['local_costmap']['ros__parameters']
    global_costmap = params['global_costmap']['global_costmap']['ros__parameters']

    assert follow_path['plugin'] == (
        'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController'
    )
    assert follow_path['use_collision_detection'] is False
    assert follow_path['use_rotate_to_heading'] is True
    assert follow_path['allow_reversing'] is False
    assert 0.0 < follow_path['rotate_to_heading_min_angle'] <= 0.35
    assert 0.0 < follow_path['rotate_to_heading_angular_vel'] <= 0.45
    assert 0.0 < follow_path['max_angular_accel'] <= 0.8
    assert progress_checker['plugin'] == 'traymover_robot_nav::YawProgressChecker'
    assert progress_checker['required_movement_radius'] == 0.30
    assert 0.15 <= progress_checker['required_movement_angle'] <= 0.25
    assert progress_checker['movement_time_allowance'] == 45.0
    assert 'obstacle_layer' not in local_costmap['plugins']
    assert 'obstacle_layer' not in global_costmap['plugins']
    assert 'behavior_server' not in params
    assert 'waypoint_follower' not in params
    assert 'default_nav_to_pose_bt_xml' in bt_params
    assert 'default_nav_through_poses_bt_xml' in bt_params


def test_option10_uses_2_5x_linear_speed_without_changing_turn_speed():
    params = load_unique_yaml(PKG_ROOT / 'config' / 'nav2_params.yaml')
    follow_path = params['controller_server']['ros__parameters']['FollowPath']

    assert_close(follow_path['desired_linear_vel'], 0.14 * 2.5)
    assert_close(follow_path['min_approach_linear_velocity'], 0.06 * 2.5)
    assert_close(follow_path['regulated_linear_scaling_min_speed'], 0.035 * 2.5)
    assert_close(follow_path['rotate_to_heading_angular_vel'], 0.45)


def test_option10_launch_does_not_route_through_collision_monitor():
    launch_text = (PKG_ROOT / 'launch' / 'traymover_nav.launch.py').read_text(
        encoding='utf-8'
    )

    assert 'collision_monitor.launch.py' not in launch_text
    assert 'cmd_vel_nav' not in launch_text
    assert '/local_planner_path' not in launch_text
    assert '/fastlio_path' not in launch_text
    assert '/pcl_path' not in launch_text
    assert "'publish_scan': LaunchConfiguration('publish_scan')" in launch_text
    assert "'publish_scan', default_value='false'" in launch_text
    assert "'max_map_odom_update_translation': '0.50'" in launch_text
    assert "'max_map_odom_update_rotation': '0.25'" in launch_text
    assert "'behavior_server'" not in launch_text
    assert "'waypoint_follower'" not in launch_text


def test_option16_obstacle_nav_keeps_conservative_speed_with_local_obstacles():
    baseline = load_unique_yaml(PKG_ROOT / 'config' / 'nav2_params.yaml')
    params = load_unique_yaml(
        PKG_ROOT / 'config' / 'nav2_params_lidar_realsense_obstacle.yaml'
    )

    baseline_controller = baseline['controller_server']['ros__parameters']
    controller = params['controller_server']['ros__parameters']
    local_costmap = params['local_costmap']['local_costmap']['ros__parameters']
    global_costmap = params['global_costmap']['global_costmap']['ros__parameters']
    obstacle_layer = local_costmap['obstacle_layer']

    baseline_follow_path = copy.deepcopy(baseline_controller['FollowPath'])
    obstacle_follow_path = copy.deepcopy(controller['FollowPath'])
    linear_speed_keys = (
        'desired_linear_vel',
        'min_approach_linear_velocity',
        'regulated_linear_scaling_min_speed',
    )
    for key in linear_speed_keys:
        baseline_follow_path.pop(key)
        obstacle_follow_path.pop(key)

    assert obstacle_follow_path == baseline_follow_path
    assert_close(controller['FollowPath']['desired_linear_vel'], 0.14)
    assert_close(controller['FollowPath']['min_approach_linear_velocity'], 0.06)
    assert_close(controller['FollowPath']['regulated_linear_scaling_min_speed'], 0.035)
    assert controller['progress_checker'] == baseline_controller['progress_checker']
    assert controller['controller_plugins'] == baseline_controller['controller_plugins']
    assert local_costmap['plugins'] == ['obstacle_layer', 'inflation_layer']
    assert global_costmap['plugins'] == ['static_layer', 'inflation_layer']
    assert obstacle_layer['plugin'] == 'nav2_costmap_2d::ObstacleLayer'
    assert obstacle_layer['enabled'] is True
    assert obstacle_layer['observation_sources'] == 'lidar_scan realsense_scan'
    assert obstacle_layer['lidar_scan']['topic'] == '/scan'
    assert obstacle_layer['lidar_scan']['data_type'] == 'LaserScan'
    assert obstacle_layer['realsense_scan']['topic'] == '/scan_realsense'
    assert obstacle_layer['realsense_scan']['data_type'] == 'LaserScan'


def test_option16_launch_routes_scans_and_cmd_vel_through_collision_monitor():
    launch_text = (
        PKG_ROOT / 'launch' / 'traymover_nav_lidar_realsense_obstacle.launch.py'
    ).read_text(encoding='utf-8')
    collision = load_unique_yaml(
        PKG_ROOT / 'config' / 'collision_monitor_lidar_realsense.yaml'
    )['collision_monitor']['ros__parameters']

    assert 'nav2_params_lidar_realsense_obstacle.yaml' in launch_text
    assert 'collision_monitor_lidar_realsense.yaml' in launch_text
    assert "'publish_scan': 'true'" in launch_text
    assert "'scan_topic': '/scan'" in launch_text
    assert "package='depthimage_to_laserscan'" in launch_text
    assert "('depth', realsense_depth_topic)" in launch_text
    assert "('depth_camera_info', realsense_info_topic)" in launch_text
    assert "('scan', realsense_scan_topic)" in launch_text
    assert "'realsense_depth_topic'," in launch_text
    assert "default_value='/camera/camera/depth/image_rect_raw'" in launch_text
    assert "default_value='/camera/camera/depth/camera_info'" in launch_text
    assert "default_value='/scan_realsense'" in launch_text
    assert "('cmd_vel', '/cmd_vel_nav')" in launch_text
    assert 'collision_monitor.launch.py' in launch_text
    assert "'params_file': collision_params_file" in launch_text
    assert "'max_map_odom_update_translation': '0.50'" in launch_text
    assert "'max_map_odom_update_rotation': '0.25'" in launch_text

    assert collision['cmd_vel_in_topic'] == 'cmd_vel_nav'
    assert collision['cmd_vel_out_topic'] == 'cmd_vel'
    assert collision['polygons'] == ['PolygonStop', 'PolygonSlow']
    assert collision['observation_sources'] == ['realsense_scan', 'lidar_scan']
    assert collision['realsense_scan']['topic'] == '/scan_realsense'
    assert collision['lidar_scan']['topic'] == '/scan'


def test_option16_launcher_menu_keeps_option10_baseline_separate():
    launcher_text = (WORKSPACE_ROOT / 'scripts' / 'traymover.sh').read_text(
        encoding='utf-8'
    )

    assert '10) action_start_nav ;;' in launcher_text
    assert '16) action_start_nav_lidar_realsense_obstacle ;;' in launcher_text
    assert '16) Navigation with LiDAR + RealSense Dynamic Obstacle Avoidance' in launcher_text
    assert 'traymover_nav_lidar_realsense_obstacle.launch.py' in launcher_text
    assert 'realsense_depth_topic:=\'${realsense_depth_topic}\'' in launcher_text
    assert 'realsense_info_topic:=\'${realsense_info_topic}\'' in launcher_text
    assert 'realsense_scan_topic:=\'${realsense_scan_topic}\'' in launcher_text
    assert 'Option 16 expects RealSense camera driver to be running' in launcher_text


def test_option16_uses_validated_absolute_pcd_projection_rule():
    launcher_text = (WORKSPACE_ROOT / 'scripts' / 'traymover.sh').read_text(
        encoding='utf-8'
    )
    function_body = launcher_text.split(
        'action_start_nav_lidar_realsense_obstacle() {', maxsplit=1
    )[1].split('\n}', maxsplit=1)[0]

    assert 'NAV_OBSTACLE_PGM_RESOLUTION="0.10"' in launcher_text
    assert 'NAV_OBSTACLE_PGM_Z_MIN="0.50"' in launcher_text
    assert 'NAV_OBSTACLE_PGM_Z_MAX="7.00"' in launcher_text
    assert 'NAV_OBSTACLE_PGM_MIN_REGION_CELLS="1"' in launcher_text
    assert '--z-reference absolute' in function_body
    assert '--resolution "${NAV_OBSTACLE_PGM_RESOLUTION}"' in function_body
    assert '--min-region-cells "${NAV_OBSTACLE_PGM_MIN_REGION_CELLS}"' in function_body


def test_navigation_options_offer_automatic_estop_after_rviz_prompt():
    launcher_text = (WORKSPACE_ROOT / 'scripts' / 'traymover.sh').read_text(
        encoding='utf-8'
    )

    assert 'Start automatic EStop? [y/N]:' in launcher_text
    assert 'AUTO_ESTOP_ENABLED="false"' in launcher_text
    assert launcher_text.count('prompt_auto_estop') == 5  # definition + four calls
    assert launcher_text.count('action_start_auto_estop') == 5
    assert launcher_text.count('auto_estop_enabled:=${AUTO_ESTOP_ENABLED}') == 3
    # The shared baseline now has separate FAST-LIO and option-21 chassis commands.
    assert launcher_text.count('auto_estop_enabled:=${nav_auto_estop_enabled}') == 2
    assert 'traymover_auto_estop.launch.py' in launcher_text


def test_option16_declares_depthimage_to_laserscan_dependency():
    package_xml = (PKG_ROOT / 'package.xml').read_text(encoding='utf-8')

    assert '<exec_depend>depthimage_to_laserscan</exec_depend>' in package_xml


def test_lidar_localization_does_not_claim_nav2_map_topic_as_pointcloud():
    localization_source = (
        SRC_ROOT / 'traymover_robot_slam' / 'lidar_localization_ros2' /
        'src' / 'lidar_localization_component.cpp'
    ).read_text(encoding='utf-8')

    assert '"ndt_map"' in localization_source
    assert ('create_subscription<sensor_msgs::msg::PointCloud2>(\n    "map"'
            not in localization_source)


def test_option14_isolates_cmu_path_from_localization_diagnostics():
    launch_text = (PKG_ROOT / 'launch' / 'traymover_3d_nav.launch.py').read_text(
        encoding='utf-8'
    )
    localization_launch = (
        PKG_ROOT / 'launch' / 'lidar_localization.launch.py'
    ).read_text(encoding='utf-8')

    assert "'local_path_topic', default_value='/local_planner_path'" in launch_text
    assert "'fastlio_path_topic', default_value='/fastlio_path'" in launch_text
    assert "'localization_path_topic', default_value='/pcl_path'" in launch_text
    assert "'waypoint_target_frame', default_value='camera_init'" in launch_text
    assert "'target_frame': waypoint_target_frame" in launch_text
    assert "'publish_scan': 'false'" in launch_text
    assert 'collision_monitor.launch.py' not in launch_text
    assert "'out_topic': '/cmd_vel'" in launch_text
    assert '/cmd_vel_nav' not in launch_text
    assert "'fastlio_path_topic': fastlio_path_topic" in launch_text
    assert "'localization_path_topic': localization_path_topic" in launch_text
    assert "'max_map_odom_update_translation': '0.30'" in launch_text
    assert "'max_map_odom_update_rotation': '0.12'" in launch_text
    assert "('/path', local_path_topic)" in launch_text
    assert launch_text.count("('/path', local_path_topic)") == 2
    assert 'static_base_to_vehicle' in launch_text
    assert "'base_link', 'vehicle'" in launch_text

    assert "'fastlio_path_topic', default_value='/path'" in localization_launch
    assert "'localization_path_topic', default_value='/path'" in localization_launch
    assert "'max_map_odom_update_translation', default_value='-1.0'" in localization_launch
    assert "'max_map_odom_update_rotation', default_value='-1.0'" in localization_launch
    assert "('/path', fastlio_path_topic)" in localization_launch
    assert "('/path', localization_path_topic)" in localization_launch


def test_option14_local_planner_is_simple_goal_following_first():
    params = load_unique_yaml(PKG_ROOT / 'config' / 'local_planner.yaml')
    planner = params['localPlanner']['ros__parameters']

    assert planner['autonomyMode'] is True
    assert planner['checkObstacle'] is False
    assert planner['checkRotObstacle'] is False


def test_option10_behavior_tree_has_no_recovery_actions():
    bt_paths = [
        PKG_ROOT / 'behavior_trees' / 'simple_navigate_to_pose.xml',
        PKG_ROOT / 'behavior_trees' / 'simple_navigate_through_poses.xml',
    ]

    for bt_path in bt_paths:
        bt_text = bt_path.read_text(encoding='utf-8')
        assert '<FollowPath ' in bt_text
        for recovery_action in ('Spin', 'Wait', 'BackUp', 'ClearEntireCostmap'):
            assert recovery_action not in bt_text

    assert '<ComputePathToPose ' in bt_paths[0].read_text(encoding='utf-8')
    assert '<ComputePathThroughPoses ' in bt_paths[1].read_text(encoding='utf-8')


def test_localization_yaml_has_unique_keys_and_far_wall_range():
    params = load_unique_yaml(PKG_ROOT / 'config' / 'localization.yaml')
    loc_params = params['/**']['ros__parameters']

    assert 100.0 <= loc_params['scan_max_range'] <= 120.0
    assert loc_params['far_point_boost_min_range'] <= 15.0
    assert loc_params['far_point_boost_factor'] == 2
    assert loc_params['voxel_leaf_size'] >= 0.12
    assert loc_params['map_voxel_leaf_size'] == 0.15
    assert loc_params['max_bootstrap_pose_jump_translation'] == 2.5
    assert loc_params['max_bootstrap_pose_jump_rotation'] == 0.6
    # Fitness is tuned per field trial; validate it without pinning the user's choice.
    # The node declares a ROS double; YAML integers crash before configuration.
    assert type(loc_params['score_threshold']) is float
    assert 0.0 < loc_params['score_threshold'] < float('inf')
    assert loc_params['ndt_align_interval_s'] >= 2.0
    assert loc_params['ndt_resolution'] == 0.5
    assert loc_params['adaptive_ndt_resolution'] is True
    assert loc_params['adaptive_ndt_max_resolution'] >= 3.0
    assert 0.0 < loc_params['adaptive_ndt_resolution_step'] <= 0.1
    assert 0.0 < loc_params['adaptive_ndt_min_usable_voxel_ratio'] <= 0.01


def test_nav2_tolerates_short_tf_jitter_without_recoveries():
    params = load_unique_yaml(PKG_ROOT / 'config' / 'nav2_params.yaml')
    bt_params = params['bt_navigator']['ros__parameters']
    controller_params = params['controller_server']['ros__parameters']
    follow_path = controller_params['FollowPath']
    progress_checker = controller_params['progress_checker']
    local_costmap = params['local_costmap']['local_costmap']['ros__parameters']
    global_costmap = params['global_costmap']['global_costmap']['ros__parameters']

    assert bt_params['transform_tolerance'] >= 0.6
    assert controller_params['failure_tolerance'] >= 2.0
    assert controller_params['general_goal_checker']['yaw_goal_tolerance'] <= 0.30
    assert follow_path['desired_linear_vel'] <= 0.35
    assert follow_path['use_rotate_to_heading'] is True
    assert follow_path['allow_reversing'] is False
    assert follow_path['regulated_linear_scaling_min_radius'] >= 3.0
    assert follow_path['regulated_linear_scaling_min_speed'] <= 0.09
    assert follow_path['transform_tolerance'] >= 0.6
    assert progress_checker['plugin'] == 'traymover_robot_nav::YawProgressChecker'
    assert progress_checker['required_movement_angle'] >= 0.15
    assert local_costmap['transform_tolerance'] >= 0.6
    assert global_costmap['transform_tolerance'] >= 0.6


def test_navigation_speed_profiles_only_change_requested_speed_values():
    option10 = load_unique_yaml(PKG_ROOT / 'config' / 'nav2_params.yaml')
    linear_2x = load_unique_yaml(PKG_ROOT / 'config' / 'nav2_params_linear_2x.yaml')
    linear_2_5x_turn_2x = load_unique_yaml(
        PKG_ROOT / 'config' / 'nav2_params_linear_2_5x_turn_2x.yaml'
    )

    speed_keys = (
        'desired_linear_vel',
        'min_approach_linear_velocity',
        'regulated_linear_scaling_min_speed',
        'rotate_to_heading_angular_vel',
    )

    def follow_path(params):
        return params['controller_server']['ros__parameters']['FollowPath']

    def without_speed_values(params):
        clone = copy.deepcopy(params)
        clone_follow_path = follow_path(clone)
        for key in speed_keys:
            clone_follow_path[key] = '<speed-mode-value>'
        return clone

    assert without_speed_values(linear_2x) == without_speed_values(option10)
    assert without_speed_values(linear_2_5x_turn_2x) == without_speed_values(option10)

    option10_follow_path = follow_path(option10)
    linear_2x_follow_path = follow_path(linear_2x)
    linear_2_5x_turn_2x_follow_path = follow_path(linear_2_5x_turn_2x)

    assert_close(option10_follow_path['desired_linear_vel'], 0.35)
    assert_close(option10_follow_path['min_approach_linear_velocity'], 0.15)
    assert_close(option10_follow_path['regulated_linear_scaling_min_speed'], 0.0875)
    assert_close(option10_follow_path['rotate_to_heading_angular_vel'], 0.45)

    assert_close(linear_2x_follow_path['desired_linear_vel'], 0.28)
    assert_close(linear_2x_follow_path['min_approach_linear_velocity'], 0.12)
    assert_close(linear_2x_follow_path['regulated_linear_scaling_min_speed'], 0.070)
    assert_close(linear_2x_follow_path['rotate_to_heading_angular_vel'], 0.45)

    assert_close(linear_2_5x_turn_2x_follow_path['desired_linear_vel'], 0.35)
    assert_close(linear_2_5x_turn_2x_follow_path['min_approach_linear_velocity'], 0.15)
    assert_close(linear_2_5x_turn_2x_follow_path['regulated_linear_scaling_min_speed'], 0.0875)
    assert_close(linear_2_5x_turn_2x_follow_path['rotate_to_heading_angular_vel'], 0.90)


def test_option15_launcher_uses_dedicated_speed_configs_without_changing_option10_case():
    launcher_text = (WORKSPACE_ROOT / 'scripts' / 'traymover.sh').read_text(
        encoding='utf-8'
    )

    assert '10) action_start_nav ;;' in launcher_text
    assert '15) action_start_nav_speed_modes ;;' in launcher_text
    assert '15) Start navigation with speed mode' in launcher_text
    assert 'nav2_params_linear_2x.yaml' in launcher_text
    assert 'nav2_params_linear_2_5x_turn_2x.yaml' in launcher_text
    assert "params_file:='${nav_params_file}'" in launcher_text


def test_option10_progress_checker_plugin_exports_yaw_progress():
    plugin_text = (PKG_ROOT / 'plugins.xml').read_text(encoding='utf-8')
    source_text = (PKG_ROOT / 'src' / 'yaw_progress_checker.cpp').read_text(
        encoding='utf-8'
    )

    assert 'traymover_robot_nav::YawProgressChecker' in plugin_text
    assert 'base_class_type="nav2_core::ProgressChecker"' in plugin_text
    assert 'required_movement_angle' in source_text
    assert 'rotated_enough(pose)' in source_text
    assert 'moved_enough(pose)' in source_text
    assert 'tf2::getYaw' not in source_text


def test_option10_progress_checker_plugin_has_no_runtime_undefined_symbols():
    plugin_lib = (
        WORKSPACE_ROOT / 'build' / 'traymover_robot_nav' /
        'libtraymover_robot_nav_progress_checker.so'
    )

    assert plugin_lib.exists(), (
        'build traymover_robot_nav before running this runtime symbol check'
    )

    result = subprocess.run(
        ['ldd', '-r', str(plugin_lib)],
        capture_output=True,
        text=True,
        check=False,
    )
    output = result.stdout + result.stderr

    assert result.returncode == 0, output
    assert 'undefined symbol' not in output
    assert 'not found' not in output


def test_rviz_shows_map_rainbow_and_ndt_alignment_overlay():
    rviz_text = (PKG_ROOT / 'rviz' / 'traymover_nav.rviz').read_text(
        encoding='utf-8'
    )

    assert 'Class: traymover_robot_safety/EstopPanel' in rviz_text
    assert 'Name: Traymover EStop' in rviz_text
    assert 'Name: MapPCD' in rviz_text
    assert 'Value: /initial_map' in rviz_text
    assert 'Color Transformer: AxisColor' in rviz_text
    assert 'Use rainbow: true' in rviz_text
    assert 'Name: NDTAlignedScan' in rviz_text
    assert 'Value: /ndt_aligned_scan' in rviz_text
    assert 'Color: 255; 220; 0' in rviz_text
    assert 'Color Transformer: FlatColor' in rviz_text


def test_option14_rviz_exposes_3d_navigation_state():
    rviz_text = (PKG_ROOT / 'rviz' / 'traymover_3d_nav.rviz').read_text(
        encoding='utf-8'
    )

    assert 'Class: traymover_robot_safety/EstopPanel' in rviz_text
    assert 'Name: Traymover EStop' in rviz_text
    for expected in (
        'Fixed Frame: map',
        'Name: MapPCD',
        'Value: /initial_map',
        'Name: RegisteredScan',
        'Value: /cloud_registered',
        'Name: NDTAlignedScan',
        'Value: /ndt_aligned_scan',
        'Name: TerrainMap',
        'Value: /terrain_map',
        'Name: LocalPath',
        'Value: /local_planner_path',
        'Name: FastLIOPath',
        'Value: /fastlio_path',
        'Name: PCLLocalizationPath',
        'Value: /pcl_path',
        'Name: FreePaths',
        'Value: /free_paths',
    ):
        assert expected in rviz_text


def test_initialpose_is_only_ndt_guess_until_valid_alignment():
    source = (
        SRC_ROOT
        / 'traymover_robot_slam'
        / 'lidar_localization_ros2'
        / 'src'
        / 'lidar_localization_component.cpp'
    ).read_text(encoding='utf-8')
    header = (
        SRC_ROOT
        / 'traymover_robot_slam'
        / 'lidar_localization_ros2'
        / 'include'
        / 'lidar_localization'
        / 'lidar_localization_component.hpp'
    ).read_text(encoding='utf-8')

    assert 'ndt_bootstrap_pending_' in header
    assert 'NDT-validated map->odom only' in header
    assert 'Initialpose accepted as NDT guess only' in source
    assert 'Seeded map' not in source
    assert 'from initialpose' not in source
    assert 'last_align_stamp_s_ = -1.0e9' in source
    assert 'last_align_stamp_s_ = now_s;' in source
    assert source.index('last_align_stamp_s_ = now_s;') < source.index(
        'registration_->align(*output_cloud, init_guess);'
    )
    assert 'waiting for valid NDT before publishing map->odom' in source
    assert ('if (!localization_active_ || !corrent_pose_with_cov_stamped_ptr_ || '
            '!have_good_pose_) {return;}') in source
    assert 'have_good_pose_ && !ndt_bootstrap_pending_' in source
    assert 'tf_snapshot_.header.stamp = now();' in source
    assert 'if (!tf_snapshot_valid_) {return;}' in source
    assert 'ndt_aligned_scan_pub_' in header
    assert '"ndt_aligned_scan"' in source
    assert 'registration_->align(*output_cloud, init_guess);' in source
    assert 'ndt_aligned_scan_pub_->publish(aligned_scan_msg);' in source
    assert 'max_map_odom_update_translation' in source
    assert 'max_map_odom_update_rotation' in source
    assert 'NDT map→odom jump' in source
    assert source.index('NDT map→odom jump') < source.index(
        'ndt_aligned_scan_pub_->publish(aligned_scan_msg);'
    )


def test_option17_runs_only_the_stationary_auto_estop_launch():
    launcher_text = (WORKSPACE_ROOT / 'scripts' / 'traymover.sh').read_text(
        encoding='utf-8'
    )
    function_body = launcher_text.split(
        'action_test_auto_estop() {', maxsplit=1
    )[1].split('\n}', maxsplit=1)[0]

    assert (
        '17) Test automatic EStop while stationary'
        in launcher_text
    )
    assert '17) action_test_auto_estop ;;' in launcher_text
    assert (
        'ros2 launch traymover_robot_safety '
        'traymover_auto_estop.launch.py'
    ) in function_body
    assert 'base_serial.launch.py' not in function_body
    assert 'traymover_robot_nav' not in function_body
    assert 'cmd_vel' not in function_body
