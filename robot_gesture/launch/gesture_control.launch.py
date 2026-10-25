# SPDX-License-Identifier: Apache-2.0
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent
from launch.conditions import IfCondition
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('robot_gesture')
    config = PathJoinSubstitution([share, 'config', 'gesture_control.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('python_executable', default_value=sys.executable),
        DeclareLaunchArgument('model_path', default_value=PathJoinSubstitution([
            share, 'models', 'gesture_recognizer.task'])),
        DeclareLaunchArgument('camera_index', default_value='0'),
        DeclareLaunchArgument('show_window', default_value='true'),
        DeclareLaunchArgument('control_robot', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(package='robot_node', executable='robot_behavior_node', output='screen',
             condition=IfCondition(LaunchConfiguration('control_robot')),
             parameters=[config, {'use_sim_time': ParameterValue(
                 LaunchConfiguration('use_sim_time'), value_type=bool)}]),
        Node(package='robot_gesture', executable='gesture_recognition_node', output='screen',
             prefix=LaunchConfiguration('python_executable'),
             parameters=[config, {
                 'model_path': LaunchConfiguration('model_path'),
                 'camera_index': ParameterValue(LaunchConfiguration('camera_index'), value_type=int),
                 'show_window': ParameterValue(LaunchConfiguration('show_window'), value_type=bool),
             }], on_exit=[EmitEvent(event=Shutdown(reason='Gesture recognition stopped'))]),
    ])
