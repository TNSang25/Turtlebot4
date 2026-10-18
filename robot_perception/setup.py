from glob import glob
from setuptools import find_packages, setup

setup(
    name='robot_perception', version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/robot_perception']),
        ('share/robot_perception', ['package.xml', 'requirements.txt']),
        ('share/robot_perception/launch', glob('launch/*.launch.py')),
        ('share/robot_perception/config', glob('config/*.yaml')),
        ('share/robot_perception/models', glob('models/*')),
        ('share/robot_perception/licenses', glob('licenses/*')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='ngocsang', maintainer_email='ngocsang@users.noreply.github.com',
    description='YOLO detection from ROS RGB images using cv_bridge.',
    license='Apache-2.0',
    entry_points={'console_scripts': ['yolo_detector = robot_perception.yolo_detector:main']},
)
