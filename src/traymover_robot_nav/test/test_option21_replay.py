"""Regression checks for static map clock repair and the RViz replay config."""
import importlib.util
from pathlib import Path

import pytest
from rclpy.serialization import serialize_message
import rosbag2_py
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
import yaml


ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location('option21_replay_static',
                                             ROOT / 'scripts/option21_replay_static.py')
REPLAY = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(REPLAY)


def write_bag(path, include_tf=True):
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    cloud = PointCloud2()
    cloud.header.frame_id = 'map'
    cloud.header.stamp.sec = 1794903207
    cloud.header.stamp.nanosec = 386097131
    cloud.width = 1
    cloud.data = b'preserve map bytes'
    transform = TransformStamped()
    transform.header.frame_id = 'base_footprint'
    transform.child_frame_id = 'base_link'
    messages = [('/simulation/map', 'sensor_msgs/msg/PointCloud2', cloud)]
    if include_tf:
        messages.append(('/tf_static', 'tf2_msgs/msg/TFMessage',
                         TFMessage(transforms=[transform])))
    for name, kind, msg in messages:
        writer.create_topic(rosbag2_py.TopicMetadata(name=name, type=kind,
                                                   serialization_format='cdr'))
        writer.write(name, serialize_message(msg), 10_000_000_000)


def test_old_bag_static_map_uses_time_independent_stamp_without_altering_source(tmp_path):
    bag = tmp_path / 'bag'
    write_bag(bag)
    database = next(bag.glob('*.db3'))
    before = database.read_bytes()
    messages = REPLAY.load_static(bag)
    cloud = messages['/simulation/map']
    assert (cloud.header.stamp.sec, cloud.header.stamp.nanosec) == (0, 0)
    assert cloud.header.frame_id == 'map'
    assert bytes(cloud.data) == b'preserve map bytes'
    assert messages['/tf_static'].transforms[0].child_frame_id == 'base_link'
    assert database.read_bytes() == before


def test_bag_without_static_transform_fails_clearly(tmp_path):
    write_bag(tmp_path / 'bag', include_tf=False)
    with pytest.raises(ValueError, match='/tf_static'):
        REPLAY.load_static(tmp_path / 'bag')


@pytest.mark.parametrize('name', ['option21_airport', 'option21_simulation'])
def test_rviz_config_has_visible_map_and_explicit_camera_frame(name):
    config = yaml.safe_load((ROOT / f'scripts/rviz/{name}.rviz').read_text())
    manager = config['Visualization Manager']
    assert manager['Enabled'] is True
    assert manager['Global Options']['Fixed Frame'] == 'map'
    assert manager['Views']['Current']['Target Frame'] == '<Fixed Frame>'
    displays = {d.get('Topic', {}).get('Value'): d for d in manager['Displays']}
    for topic in ('/simulation/map', '/simulation/truth_path', '/simulation/wheel_path',
                  '/simulation/localized_path', '/simulation/goals'):
        assert displays[topic]['Enabled'] is True
        assert displays[topic]['Topic']['Durability Policy'] == 'Transient Local'
