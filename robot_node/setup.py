from glob import glob
from setuptools import find_packages, setup


setup(
    name='robot_node',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/robot_node']),
        ('share/robot_node', ['package.xml']),
        ('share/robot_node/licenses', ['licenses/LICENSE']),
        ('share/robot_node/config', glob('config/*.yaml')),
        ('share/robot_node/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Workspace Maintainer',
    maintainer_email='maintainer@example.com',
    description='Teleoperation, gesture behaviors, goal recording and Nav2 missions for TurtleBot 4.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_keyboard = robot_node.teleop_keyboard:main',
            'goal_recorder = robot_node.goal_recorder:main',
            'waypoint_navigator = robot_node.waypoint_navigator:main',
            'robot_behavior_node = robot_node.robot_behavior_node:main',
        ],
    },
)
