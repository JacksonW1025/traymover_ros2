"""Physical geometry, occupancy and full Nav2 profile for the synthetic demo."""
import copy
import math

import numpy as np
from PIL import Image
import yaml

from option21_simulation import ROOM_PILLARS


FOOTPRINT = np.array([[-.4, -.275], [.4, -.275], [.4, .275], [-.4, .275]])


def rectangle(x, y, width):
    return np.array([[x-width/2, y-width/2], [x+width/2, y-width/2],
                     [x+width/2, y+width/2], [x-width/2, y+width/2]])


def footprint(pose):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return FOOTPRINT @ np.array([[c, s], [-s, c]]) + [x, y]


def overlaps(a, b):
    """Convex polygon intersection using the separating axis theorem."""
    for polygon in (a, b):
        edge = np.roll(polygon, -1, axis=0) - polygon
        for axis in np.column_stack([-edge[:, 1], edge[:, 0]]):
            p, q = a @ axis, b @ axis
            if p.max() <= q.min() or q.max() <= p.min():
                return False
    return True


def collides(pose):
    body = footprint(pose)
    if np.any(np.abs(body[:, 0]) >= 8) or np.any(np.abs(body[:, 1]) >= 6):
        return True
    return any(overlaps(body, rectangle(x, y, w)) for x, y, w, _ in ROOM_PILLARS)


def clearance(pose):
    body = footprint(pose)
    if collides(pose):
        return 0.
    best = min(8-abs(body[:, 0]).max(), 6-abs(body[:, 1]).max())
    for x, y, w, _ in ROOM_PILLARS:
        obstacle = rectangle(x, y, w)
        for points, polygon in ((body, obstacle), (obstacle, body)):
            a = polygon
            direction = np.roll(polygon, -1, axis=0)-a
            delta = points[:, None, :]-a[None, :, :]
            t = np.clip(np.sum(delta*direction, axis=2) / np.sum(direction**2, axis=1), 0, 1)
            best = min(best, float(np.linalg.norm(delta-t[:, :, None]*direction, axis=2).min()))
    return float(best)


def repair_simulation_map(prefix):
    """Rasterize known solid pillar interiors and outside walls, retaining projection.

    The synthetic PCD has 12 cm samples, much sparser than a real map. Do not
    use a connected-component filter that removes isolated 5 cm grid samples.
    Solid simulation geometry also prevents a pillar's hollow PCD shell from
    becoming a free goal location in the occupancy map.
    """
    config = yaml.safe_load(prefix.with_suffix('.yaml').read_text())
    path = prefix.parent / config['image']
    grid = np.array(Image.open(path))
    h, w = grid.shape
    resolution = config['resolution']
    xx = config['origin'][0] + (np.arange(w)+.5)*resolution
    yy = config['origin'][1] + (h-1-np.arange(h)+.5)*resolution
    x, y = np.meshgrid(xx, yy)
    solid = (np.abs(x) >= 8) | (np.abs(y) >= 6)
    for px, py, width, _ in ROOM_PILLARS:
        solid |= (np.abs(x-px) <= width/2) & (np.abs(y-py) <= width/2)
    grid[solid] = 0
    Image.fromarray(grid).save(path)
    return {'occupied_cells': int(np.sum(grid == 0)), 'solid_pillars': len(ROOM_PILLARS),
            'projection_min_region_cells': 1, 'solid_geometry_infill': True}


def full_navigation_parameters(baseline):
    config = copy.deepcopy(baseline)
    for name in ('global_costmap', 'local_costmap'):
        params = config[name][name]['ros__parameters']
        params['plugins'] = ['static_layer', 'obstacle_layer', 'inflation_layer']
        params['static_layer'] = {
            'plugin': 'nav2_costmap_2d::StaticLayer', 'map_subscribe_transient_local': True}
        params['obstacle_layer'] = {
            'plugin': 'nav2_costmap_2d::ObstacleLayer', 'enabled': True,
            'observation_sources': 'lidar', 'footprint_clearing_enabled': True,
            'lidar': {'topic': '/point_cloud_nav', 'sensor_frame': 'laser',
                      'data_type': 'PointCloud2', 'marking': True, 'clearing': True,
                      'min_obstacle_height': .10, 'max_obstacle_height': 2.20,
                      'obstacle_max_range': 12., 'raytrace_max_range': 18.,
                      'obstacle_min_range': .3, 'raytrace_min_range': .3}}
    local = config['local_costmap']['local_costmap']['ros__parameters']
    local.update(width=6, height=6)
    controller = config['controller_server']['ros__parameters']
    controller['FollowPath'] = {
        'plugin': 'dwb_core::DWBLocalPlanner', 'debug_trajectory_details': False,
        'min_vel_x': 0., 'min_vel_y': 0., 'max_vel_x': .35, 'max_vel_y': 0.,
        'max_vel_theta': .6, 'min_speed_xy': 0., 'max_speed_xy': .35,
        'min_speed_theta': 0., 'acc_lim_x': 1., 'acc_lim_y': 0., 'acc_lim_theta': 1.5,
        'decel_lim_x': -1., 'decel_lim_y': 0., 'decel_lim_theta': -1.5,
        'vx_samples': 12, 'vy_samples': 1, 'vtheta_samples': 24, 'sim_time': 1.7,
        'linear_granularity': .05, 'angular_granularity': .025,
        'transform_tolerance': .6, 'xy_goal_tolerance': .2,
        'trans_stopped_velocity': .03, 'short_circuit_trajectory_evaluation': True,
        'stateful': True, 'publish_local_plan': True, 'publish_trajectories': True,
        'critics': ['RotateToGoal', 'Oscillation', 'ObstacleFootprint', 'BaseObstacle',
                    'GoalAlign', 'PathAlign', 'PathDist', 'GoalDist'],
        'ObstacleFootprint.scale': 1., 'BaseObstacle.scale': .02,
        'PathAlign.scale': 32., 'PathAlign.forward_point_distance': .1,
        'GoalAlign.scale': 24., 'GoalAlign.forward_point_distance': .1,
        'PathDist.scale': 32., 'GoalDist.scale': 24., 'RotateToGoal.scale': 32.,
        'RotateToGoal.slowing_factor': 5., 'RotateToGoal.lookahead_time': -1.}
    config['behavior_server'] = {'ros__parameters': {
        'use_sim_time': True, 'costmap_topic': 'local_costmap/costmap_raw',
        'footprint_topic': 'local_costmap/published_footprint', 'cycle_frequency': 10.,
        'behavior_plugins': ['spin', 'backup', 'wait'],
        'spin': {'plugin': 'nav2_behaviors/Spin'},
        'backup': {'plugin': 'nav2_behaviors/BackUp'},
        'wait': {'plugin': 'nav2_behaviors/Wait'},
        'global_frame': 'odom', 'robot_base_frame': 'base_link',
        'transform_tolerance': .6, 'simulate_ahead_time': 2.,
        'max_rotational_vel': .6, 'min_rotational_vel': .15, 'rotational_acc_lim': 1.5}}
    return config


def decorate_rviz(manager):
    for display in manager['Displays']:
        name = display['Name']
        if name in ('MapPCD', 'NDTAlignedScan', 'TF'):
            display['Enabled'] = False
        if name in ('GlobalPlan', 'LocalPlan'):
            display.update({'Line Style': 'Billboards', 'Line Width': .075,
                            'Color': '30; 130; 255' if name == 'GlobalPlan' else '255; 220; 20',
                            'Offset': {'X': 0, 'Y': 0, 'Z': .10}})
    for name, topic, alpha in (
            ('Global Costmap - obstacle inflation', '/global_costmap/costmap', .42),
            ('Local Costmap - live obstacles', '/local_costmap/costmap', .55)):
        manager['Displays'].append({
            'Class': 'rviz_default_plugins/Map', 'Name': name, 'Enabled': True,
            'Alpha': alpha, 'Color Scheme': 'costmap', 'Draw Behind': False,
            'Topic': {'Value': topic, 'Durability Policy': 'Transient Local',
                      'Reliability Policy': 'Reliable', 'Depth': 1}})
    manager['Displays'].append({
        'Class': 'rviz_default_plugins/MarkerArray', 'Name': 'Solid pillars and demo goal',
        'Enabled': True, 'Topic': {'Value': '/simulation/obstacles',
            'Durability Policy': 'Transient Local', 'Reliability Policy': 'Reliable'}})
    manager['Displays'].append({
        'Class': 'rviz_default_plugins/Path', 'Name': 'Actual Robot Trail (green)',
        'Enabled': True, 'Topic': {'Value': '/simulation/truth_path'},
        'Color': '30; 220; 70', 'Line Style': 'Billboards', 'Line Width': .06,
        'Offset': {'X': 0, 'Y': 0, 'Z': .12}, 'Buffer Length': 1})
    manager['Displays'].append({
        'Class': 'rviz_default_plugins/Polygon', 'Name': 'Robot Footprint',
        'Enabled': True, 'Topic': {'Value': '/local_costmap/published_footprint'},
        'Color': '255; 255; 255', 'Alpha': 1.})


def markers(stamp):
    from visualization_msgs.msg import Marker, MarkerArray
    result = MarkerArray()
    for i, (x, y, width, height) in enumerate(ROOM_PILLARS):
        box = Marker()
        box.header.frame_id, box.header.stamp = 'map', stamp
        box.ns, box.id, box.type, box.action = 'solid_pillars', i, Marker.CUBE, Marker.ADD
        box.pose.position.x, box.pose.position.y, box.pose.position.z = x, y, height/2
        box.pose.orientation.w = 1.
        box.scale.x, box.scale.y, box.scale.z = width, width, height
        box.color.r, box.color.g, box.color.b, box.color.a = 1., .45, .10, .85
        result.markers.append(box)
        label = Marker()
        label.header = box.header
        label.ns, label.id, label.type = 'pillar_labels', i, Marker.TEXT_VIEW_FACING
        label.pose.position.x, label.pose.position.y, label.pose.position.z = x, y, height+.25
        label.pose.orientation.w = 1.
        label.scale.z = .3
        label.color.r = label.color.g = label.color.b = label.color.a = 1.
        label.text = f'Pillar {i+1}'
        result.markers.append(label)
    goal = Marker()
    goal.header.frame_id, goal.header.stamp = 'map', stamp
    goal.ns, goal.id, goal.type = 'demo_goal', 0, Marker.SPHERE
    goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = 6., -4.4, .15
    goal.pose.orientation.w = 1.
    goal.scale.x = goal.scale.y = goal.scale.z = .35
    goal.color.g = goal.color.b = goal.color.a = 1.
    result.markers.append(goal)
    return result


def path_metrics(points):
    points = np.asarray(points)
    if len(points) < 2:
        return {}
    start, end = points[0], points[-1]
    direction = end-start
    length = np.linalg.norm(direction)
    t = np.clip((points-start) @ direction / max(length**2, 1e-10), 0, 1)
    deviations = np.linalg.norm(points-(start+t[:, None]*direction), axis=1)
    dense = np.concatenate([a + np.linspace(0, 1, max(2, int(np.ceil(np.linalg.norm(b-a)/.025))+1))[:, None]*(b-a)
                            for a, b in zip(points[:-1], points[1:])])
    crosses = any(np.any((np.abs(dense[:, 0]-x) < width/2) &
                        (np.abs(dense[:, 1]-y) < width/2)) for x, y, width, _ in ROOM_PILLARS)
    return {'length_m': float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum()),
            'straight_distance_m': float(length),
            'max_deviation_from_straight_m': float(deviations.max()),
            'enters_solid_pillar': bool(crosses)}
