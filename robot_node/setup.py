from setuptools import find_packages, setup


setup(
    name='robot_node',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/robot_node']),
        ('share/robot_node', ['package.xml']),
        ('share/robot_node/licenses', ['licenses/LICENSE']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Workspace Maintainer',
    maintainer_email='maintainer@example.com',
    description='Keyboard teleoperation for TurtleBot 4.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_keyboard = robot_node.teleop_keyboard:main',
        ],
    },
)
