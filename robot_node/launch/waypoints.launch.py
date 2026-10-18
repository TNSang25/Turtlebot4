# SPDX-License-Identifier: Apache-2.0
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('goals_file', default_value=PathJoinSubstitution([
            FindPackageShare('robot_node'), 'config', 'goals.yaml'])),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('goal_timeout', default_value='300.0'),
        DeclareLaunchArgument('pause_at_goal', default_value='2.0'),
        DeclareLaunchArgument('result_file', default_value=''),
        Node(package='robot_node', executable='waypoint_navigator', output='screen',
             parameters=[{
                 'goals_file': LaunchConfiguration('goals_file'),
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                 'goal_timeout': ParameterValue(LaunchConfiguration('goal_timeout'), value_type=float),
                 'pause_at_goal': ParameterValue(LaunchConfiguration('pause_at_goal'), value_type=float),
                 'result_file': ParameterValue(LaunchConfiguration('result_file'), value_type=str),
             }]),
    ])
