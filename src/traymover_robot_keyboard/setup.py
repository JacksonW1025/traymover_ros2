from setuptools import setup

package_name = 'traymover_robot_keyboard'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='traymover',
    maintainer_email='traymover@todo.todo',
    description='Keyboard teleoperation for traymover robot',
    license='BSD-2-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'traymover_keyboard = traymover_robot_keyboard.traymover_keyboard:main',
        ],
    },
)
