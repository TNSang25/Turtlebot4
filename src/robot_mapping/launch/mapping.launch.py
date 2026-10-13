# SPDX-License-Identifier: Apache-2.0
"""Run asynchronous SLAM; the simulator supplies odom, scan, TF and clock."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('robot_mapping')
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([share, 'config', 'slam.yaml']),
            description='SLAM Toolbox parameter file'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([share, 'rviz', 'mapping.rviz'])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py',
            ])),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'slam_params_file': LaunchConfiguration('params_file'),
            }.items()),
        Node(
            package='rviz2', executable='rviz2', name='mapping_rviz',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen'),
    ])
