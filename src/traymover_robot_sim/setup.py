import glob
import os

from setuptools import setup


package_name = 'traymover_robot_sim'
asset_directories = (
    'launch',
    'config',
    'worlds',
    'models',
    'maps',
    'behavior_trees',
    'rviz',
)

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]
for directory in asset_directories:
    # Preserve nested asset paths (for example models/dynamic_box/model.sdf)
    # in the installed share directory used by the launch file.
    files = [
        path
        for path in glob.glob(os.path.join(directory, '**'), recursive=True)
        if os.path.isfile(path)
    ]
    for path in files:
        destination = os.path.join('share', package_name, os.path.dirname(path))
        data_files.append((destination, [path]))

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='traymover',
    maintainer_email='traymover@todo.todo',
    description='Simulation assets and nodes for the Traymover robot',
    license='BSD-2-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detour_supervisor = traymover_robot_sim.detour_supervisor:main',
            'demo_goal_sender = traymover_robot_sim.demo_goal_sender:main',
            'sim_odom_tf = traymover_robot_sim.sim_odom_tf:main',
        ],
    },
)
