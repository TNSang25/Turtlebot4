# SPDX-License-Identifier: Apache-2.0
"""Run Nav2 using map and TF supplied by separately started AMCL or SLAM."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('robot_navigation')
    nav2_launch = PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch'])
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    common_arguments = {
        'use_sim_time': use_sim_time,
        'params_file': params_file,
        'autostart': LaunchConfiguration('autostart'),
        # Includes are standalone; there is no composition container in this launch.
        'use_composition': 'False',
        'use_respawn': 'False',
        'log_level': LaunchConfiguration('log_level'),
    }
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Open RViz if no visualization is running already'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([share, 'config', 'nav2.yaml'])),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([share, 'rviz', 'navigation.rviz'])),
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    nav2_launch, 'navigation_launch.py'])),
                launch_arguments=common_arguments.items()),
        ]),
        Node(
            package='rviz2', executable='rviz2', name='navigation_rviz',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen'),
    ])
