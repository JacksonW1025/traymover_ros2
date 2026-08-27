from math import inf, nan

from sensor_msgs.msg import LaserScan

from traymover_robot_sim.detour_supervisor import has_front_obstacle, set_global_scan_frame


def scan(ranges, angle_min=-0.4, angle_increment=0.2):
    msg = LaserScan()
    msg.angle_min = angle_min
    msg.angle_increment = angle_increment
    msg.range_min = 0.2
    msg.range_max = 5.0
    msg.ranges = ranges
    return msg


def test_front_obstacle_ignores_invalid_and_side_samples():
    assert has_front_obstacle(scan([inf, 0.7, nan, 2.0]), 0.8, 0.25)
    assert not has_front_obstacle(scan([0.5, inf, inf, inf]), 0.8, 0.25)


def test_global_scan_uses_base_frame_for_simulated_replanning():
    message = scan([1.0])
    assert set_global_scan_frame(message, "base_link").header.frame_id == "base_link"
