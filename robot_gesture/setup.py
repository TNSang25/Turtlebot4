from glob import glob
from setuptools import find_packages, setup

setup(
    name='robot_gesture', version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/robot_gesture']),
        ('share/robot_gesture', ['package.xml', 'requirements.txt']),
        ('share/robot_gesture/launch', glob('launch/*.launch.py')),
        ('share/robot_gesture/config', glob('config/*.yaml')),
        ('share/robot_gesture/models', glob('models/*')),
        ('share/robot_gesture/licenses', glob('licenses/*')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Workspace Maintainer', maintainer_email='maintainer@example.com',
    description='MediaPipe gestures and confirmed TurtleBot4 behavior control.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'gesture_recognition_node = robot_gesture.gesture_recognition_node:main',
    ]},
)
