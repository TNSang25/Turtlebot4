# SPDX-License-Identifier: Apache-2.0
"""Gazebo + robot + bridge; SLAM/Nav2 start in their own packages."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim = Path(get_package_share_directory('robot_sim'))
    description = Path(get_package_share_directory('robot_description'))

    def include(filename, names):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(sim / 'launch' / filename)),
            launch_arguments={name: LaunchConfiguration(name) for name in names}.items())

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='demo'),
        DeclareLaunchArgument('headless', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('enable_camera', default_value='true', choices=['true', 'false'],
                              description='Enable the RGB-D camera sensor'),
        DeclareLaunchArgument('robot_name', default_value='turtlebot4'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.02'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('verbosity', default_value='2', choices=['0', '1', '2', '3', '4']),
        include('gazebo.launch.py', ['world', 'headless', 'verbosity']),
        include('spawn.launch.py', ['robot_name', 'enable_camera', 'x', 'y', 'z', 'yaw']),
        include('bridge.launch.py', []),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', str(description / 'rviz/description.rviz')],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
