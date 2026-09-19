"""Safety and frame contracts for the standalone AMCL fallback configuration."""
import ast
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]
PARAMS = yaml.safe_load((ROOT / 'config/nav2_option20.yaml').read_text())


def test_motion_is_slow_forward_only_and_collision_checked():
    controller = PARAMS['controller_server']['ros__parameters']
    follow = controller['FollowPath']
    assert 0 < follow['desired_linear_vel'] <= .14
    assert follow['allow_reversing'] is False
    assert follow['use_collision_detection'] is True
    assert controller['robot_base_frame'] == 'base_footprint'
    assert controller['odom_topic'] == '/odom'


def test_both_costmaps_observe_lidar_and_depth_and_preserve_unknown():
    for key in ('global_costmap', 'local_costmap'):
        costmap = PARAMS[key][key]['ros__parameters']
        assert costmap['track_unknown_space'] is True
        assert costmap['robot_base_frame'] == 'base_footprint'
        assert 'obstacle_layer' in costmap['plugins']
        footprint = ast.literal_eval(costmap['footprint'])
        assert len(footprint) == 4
        assert max(abs(p[0]) for p in footprint) >= .4
        assert max(abs(p[1]) for p in footprint) >= .275
        obstacle = costmap['obstacle_layer']
        assert set(obstacle['observation_sources'].split()) == {
            'lidar', 'depth', 'lidar_clear', 'depth_clear'}
        for source in ('lidar', 'depth'):
            assert obstacle[source]['topic'] == f'/option20/{source}_obstacles'
            assert obstacle[source]['marking'] is True
            assert obstacle[source]['clearing'] is False
            assert obstacle[source]['data_type'] == 'PointCloud2'
            clearing = obstacle[source + '_clear']
            assert clearing['topic'] == f'/option20/{source}_clearing'
            assert clearing['marking'] is False
            assert clearing['clearing'] is True
            assert clearing['max_obstacle_height'] <= .12
    assert PARAMS['local_costmap']['local_costmap']['ros__parameters']['global_frame'] == 'odom'
    assert PARAMS['global_costmap']['global_costmap']['ros__parameters']['global_frame'] == 'map'
    assert PARAMS['planner_server']['ros__parameters']['GridBased']['allow_unknown'] is False


def test_amcl_requires_explicit_initialization_and_correct_scan_frame_chain():
    amcl = PARAMS['amcl']['ros__parameters']
    assert amcl['set_initial_pose'] is False
    assert amcl['always_reset_initial_pose'] is True
    assert (amcl['global_frame_id'], amcl['odom_frame_id'], amcl['base_frame_id']) == (
        'map', 'odom', 'base_footprint')
    assert amcl['scan_topic'] == '/scan_localization'
    assert amcl['laser_model_type'] == 'likelihood_field_prob'
    assert amcl['do_beamskip'] is True
    assert amcl['tf_broadcast'] is True


def test_recovery_can_only_wait_without_clearing_or_blind_motion():
    behaviors = PARAMS['behavior_server']['ros__parameters']
    assert behaviors['behavior_plugins'] == ['wait']
    for filename in ('option20_navigate_to_pose.xml', 'option20_navigate_through_poses.xml'):
        tree = ET.parse(ROOT / 'behavior_trees' / filename)
        tags = {element.tag for element in tree.iter()}
        assert 'Wait' in tags
        assert 'FollowPath' in tags
        assert not tags.intersection({'Spin', 'BackUp', 'DriveOnHeading',
                                      'ClearEntireCostmap', 'ClearCostmapExceptRegion',
                                      'ClearCostmapAroundRobot'})
