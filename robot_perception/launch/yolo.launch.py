# SPDX-License-Identifier: Apache-2.0
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('robot_perception')
    return LaunchDescription([
        DeclareLaunchArgument('python_executable', default_value=sys.executable),
        DeclareLaunchArgument('model_path', default_value=PathJoinSubstitution([share, 'models', 'yolov8n.pt'])),
        DeclareLaunchArgument('image_topic', default_value='/camera/image'),
        DeclareLaunchArgument('show_window', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('device', default_value='cpu'),
        Node(package='robot_perception', executable='yolo_detector', name='yolo_detector',
             prefix=LaunchConfiguration('python_executable'), output='screen',
             parameters=[PathJoinSubstitution([share, 'config', 'yolo.yaml']), {
                 'model_path': LaunchConfiguration('model_path'),
                 'image_topic': LaunchConfiguration('image_topic'),
                 'device': LaunchConfiguration('device'),
                 'show_window': ParameterValue(LaunchConfiguration('show_window'), value_type=bool),
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
             }]),
    ])
