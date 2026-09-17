#!/usr/bin/env python3
"""ROS-independent sensor geometry shared by production and guided detour."""
import math

from detour_gate import Ray, Scan


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def valid_quaternion(q):
    values = (q.x, q.y, q.z, q.w)
    return all(math.isfinite(v) for v in values) and abs(
        sum(v * v for v in values) - 1.0) < 0.01


def yaw(q):
    if not valid_quaternion(q):
        raise ValueError('invalid quaternion')
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def scan_to_rays(message, transform, acquired, allow_infinite=False):
    """Transform scan rays into the requested frame; missing beams remain unknown."""
    t, q = transform.translation, transform.rotation
    if (not valid_quaternion(q) or not all(math.isfinite(v) for v in (
            t.x, t.y, t.z, message.angle_min, message.angle_increment,
            message.range_min, message.range_max)) or
            message.angle_increment == 0 or
            not 0 <= message.range_min < message.range_max):
        raise ValueError('invalid scan metadata or transform')
    rays = []
    for index, distance in enumerate(message.ranges):
        hit = math.isfinite(distance)
        if (math.isnan(distance) or distance < message.range_min
                or (not hit and not allow_infinite)):
            continue
        if hit and distance > message.range_max:
            continue
        distance = min(distance, message.range_max)
        angle = message.angle_min + index * message.angle_increment
        x, y = math.cos(angle), math.sin(angle)
        # First two columns of the full quaternion rotation, including tilt.
        dx = (1 - 2 * (q.y*q.y + q.z*q.z)) * x + 2*(q.x*q.y - q.z*q.w)*y
        dy = 2*(q.x*q.y + q.z*q.w)*x + (1 - 2*(q.x*q.x + q.z*q.z))*y
        planar_scale = math.hypot(dx, dy)
        if planar_scale < 0.90:
            continue
        # depthimage_to_laserscan stores radial range; Auto EStop evaluates
        # camera optical-axis depth. Keep that additional conservative metric.
        axial = max(0.0, distance * math.cos(angle)) if allow_infinite else None
        rays.append(Ray(t.x, t.y, math.atan2(dy, dx),
                        distance * planar_scale, hit, axial))
    return Scan(acquired, tuple(rays))
