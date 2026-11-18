"""Keep the Traymover IMU topic available with either HiPNUC ROS driver."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
import pytest


PACKAGE = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('modern', [True, False])
def test_imu_driver_uses_configured_port_and_topic(monkeypatch, tmp_path, modern):
    spec = importlib.util.spec_from_file_location(
        'imu_launch', PACKAGE / 'launch' / 'traymover_imu.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    executables = tmp_path / 'lib' / 'hipnuc_imu'
    executables.mkdir(parents=True)
    executable = 'hipnuc_imu_node' if modern else 'talker'
    (executables / executable).touch()
    monkeypatch.setattr(module, 'get_package_prefix', lambda _: str(tmp_path))
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(PACKAGE))
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    context = LaunchContext()
    context.launch_configurations['imu_topic'] = '/test/imu'
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)

    node, = module.include_imu_launch(context)
    assert node['executable'] == executable
    parameters, = node['parameters']
    port = '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A6C071391-if00'
    if modern:
        assert parameters == {'port': port, 'baudrate': 115200, 'frame_id': 'imu_link'}
        assert node['remappings'] == [('imu/data', '/test/imu')]
    else:
        assert parameters == {'serial_port': port, 'baud_rate': 115200,
                              'frame_id': 'imu_link', 'imu_topic': '/test/imu'}
