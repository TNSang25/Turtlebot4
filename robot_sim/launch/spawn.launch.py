# SPDX-License-Identifier: Apache-2.0
"""Publish the URDF and spawn one robot in an already running Gazebo world."""

from pathlib import Path
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def _spawn(context):
    name = LaunchConfiguration('robot_name').perform(context)
    if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', name):
        raise ValueError('robot_name must start with a letter and contain letters/digits/underscores')
    path = Path(get_package_share_directory('robot_description')) / 'urdf/turtlebot4.urdf.xacro'
    description = xacro.process_file(str(path), mappings={
        'simulation': 'true', 'robot_name': name,
        'enable_camera': LaunchConfiguration('enable_camera').perform(context),
    }).toxml()
    # Without -world, create waits for Gazebo's world-discovery service before
    # spawning. This avoids an arbitrary startup delay. Run one simulator per
    # IGN_PARTITION; use different ROS_DOMAIN_IDs as well for separate robots.
    arguments = ['-name', name, '-topic', '/robot_description', '-allow_renaming', 'false']
    for arg, flag in [('x', '-x'), ('y', '-y'), ('z', '-z'), ('yaw', '-Y')]:
        arguments += [flag, LaunchConfiguration(arg).perform(context)]
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen', parameters=[{
                 'robot_description': description, 'use_sim_time': True}]),
        Node(package='ros_gz_sim', executable='create', name='spawn_robot',
             arguments=arguments, output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='turtlebot4'),
        DeclareLaunchArgument('enable_camera', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.02'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        OpaqueFunction(function=_spawn),
    ])
