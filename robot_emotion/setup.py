from glob import glob
from setuptools import find_packages, setup

setup(
    name='robot_emotion', version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/robot_emotion']),
        ('share/robot_emotion', ['package.xml', 'requirements.txt', 'requirements-fer.txt']),
        ('share/robot_emotion/config', glob('config/*.yaml')),
        ('share/robot_emotion/launch', glob('launch/*.launch.py')),
        ('share/robot_emotion/licenses', glob('licenses/*')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='ngocsang', maintainer_email='ngocsang@users.noreply.github.com',
    description='FER webcam expressions and TurtleBot control.', license='Apache-2.0',
    entry_points={'console_scripts': [
        'emotion_recognition_node = robot_emotion.emotion_recognition_node:main',
    ]},
)
