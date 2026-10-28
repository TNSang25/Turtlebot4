# SPDX-License-Identifier: Apache-2.0
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('regions_file', default_value=PathJoinSubstitution([
            FindPackageShare('robot_semantic'), 'config', 'regions.yaml'])),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(package='robot_semantic', executable='semantic_map_node', output='screen', parameters=[{
            'regions_file': LaunchConfiguration('regions_file'),
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
        }]),
    ])
