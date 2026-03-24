import os
from glob import glob
from setuptools import setup

package_name = 'turn_on_traymover_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='traymover',
    maintainer_email='traymover@todo.todo',
    description='Serial driver node for traymover robot chassis (40-byte protocol)',
    license='BSD-2-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'traymover_robot_node = turn_on_traymover_robot.traymover_robot:main',
        ],
    },
)
