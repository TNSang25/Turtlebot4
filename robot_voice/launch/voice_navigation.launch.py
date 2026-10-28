# SPDX-License-Identifier: Apache-2.0
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('robot_voice')
    config = PathJoinSubstitution([share, 'config', 'voice.yaml'])
    regions = LaunchConfiguration('regions_file')
    sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('regions_file', default_value=PathJoinSubstitution([
            FindPackageShare('robot_semantic'), 'config', 'regions.yaml'])),
        DeclareLaunchArgument('model_path', default_value=PathJoinSubstitution([
            share, 'models', 'faster-whisper-small'])),
        DeclareLaunchArgument('python_executable', default_value=sys.executable),
        DeclareLaunchArgument('device', default_value=''),
        DeclareLaunchArgument('speech', default_value='true', description='false: text-topic debugging only'),
        DeclareLaunchArgument('audio_file', default_value='', description='Optional mono PCM16 WAV instead of live microphone'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(package='robot_semantic', executable='semantic_map_node', output='screen', parameters=[{
            'regions_file': regions, 'use_sim_time': sim_time}]),
        Node(package='robot_voice', executable='voice_navigation_node', output='screen',
             parameters=[config, {'regions_file': regions, 'use_sim_time': sim_time}]),
        Node(package='robot_voice', executable='speech_recognition_node', output='screen',
             prefix=LaunchConfiguration('python_executable'), condition=IfCondition(LaunchConfiguration('speech')),
             parameters=[config, {'model_path': LaunchConfiguration('model_path'),
                                 'device': ParameterValue(LaunchConfiguration('device'), value_type=str),
                                 'audio_file': LaunchConfiguration('audio_file')}]),
    ])
