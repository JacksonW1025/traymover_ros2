#!/usr/bin/env python3
"""Supply time-independent map/static TF and wait for RViz before bag playback."""
import argparse
import os
from pathlib import Path
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.serialization import deserialize_message
import rosbag2_py
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage


def load_static(bag_path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_path), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/simulation/map', '/tf_static']))
    messages = {}
    types = {'/simulation/map': PointCloud2, '/tf_static': TFMessage}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        messages[topic] = deserialize_message(data, types[topic])
        if len(messages) == 2:
            break
    if len(messages) != 2:
        raise ValueError('Bag must contain /simulation/map and /tf_static')
    # Old experiment bags stamped the static map with system time. Zero means
    # time-independent/latest in TF and remains valid across --loop clock resets.
    messages['/simulation/map'].header.stamp.sec = 0
    messages['/simulation/map'].header.stamp.nanosec = 0
    return messages


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--rviz-pid', type=int, required=True)
    parser.add_argument('--ready-file', type=Path, required=True)
    parser.add_argument('--timeout', type=float, default=30)
    args = parser.parse_args()
    messages = load_static(args.bag)
    rclpy.init()
    node = rclpy.create_node('option21_replay_static')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publishers = {topic: node.create_publisher(type(msg), topic, qos)
                  for topic, msg in messages.items()}
    required = ['/simulation/map', '/simulation/truth_path', '/simulation/wheel_path',
                '/simulation/localized_path', '/simulation/goals', '/point_cloud_localization']
    deadline = time.monotonic() + args.timeout
    ready = False
    missing = required
    next_publish = 0.
    try:
        while rclpy.ok():
            try:
                os.kill(args.rviz_pid, 0)
            except ProcessLookupError:
                raise RuntimeError('RViz exited before/during playback') from None
            if time.monotonic() >= next_publish:
                for topic, publisher in publishers.items():
                    publisher.publish(messages[topic])
                next_publish = time.monotonic() + 1.
            if not ready:
                missing = [topic for topic in required if not any(
                    endpoint.node_name.startswith('rviz')
                    for endpoint in node.get_subscriptions_info_by_topic(topic))]
                if not missing:
                    args.ready_file.touch()
                    ready = True
                    print('RViz subscriptions ready; static map and TF are being supplied.', flush=True)
                elif time.monotonic() >= deadline:
                    raise RuntimeError(f'RViz subscriptions not ready: {missing}')
            rclpy.spin_once(node, timeout_sec=.2)
            time.sleep(.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
