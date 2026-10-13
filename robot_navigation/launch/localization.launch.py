# SPDX-License-Identifier: Apache-2.0
"""Load a saved map and start AMCL independently of the navigation stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    share = FindPackageShare('robot_navigation')
    use_sim_time = LaunchConfiguration('use_sim_time')
    localization_params = RewrittenYaml(
        source_file=LaunchConfiguration('params_file'),
        param_rewrites={
            'amcl.ros__parameters.set_initial_pose': LaunchConfiguration('set_initial_pose'),
            'amcl.ros__parameters.initial_pose.x': LaunchConfiguration('initial_pose_x'),
            'amcl.ros__parameters.initial_pose.y': LaunchConfiguration('initial_pose_y'),
            'amcl.ros__parameters.initial_pose.z': '0.0',
            'amcl.ros__parameters.initial_pose.yaw': LaunchConfiguration('initial_pose_yaw'),
        },
        convert_types=True)
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument(
            'set_initial_pose', default_value='false', choices=['true', 'false'],
            description='Initialize AMCL from the supplied pose; false uses RViz 2D Pose Estimate'),
        DeclareLaunchArgument(
            'initial_pose_x', default_value='0.0', description='Initial x in map frame (metres)'),
        DeclareLaunchArgument(
            'initial_pose_y', default_value='0.0', description='Initial y in map frame (metres)'),
        DeclareLaunchArgument(
            'initial_pose_yaw', default_value='0.0', description='Initial yaw in map frame (radians)'),
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([
                FindPackageShare('robot_sim'), 'maps', 'demo.yaml']),
            description='Saved occupancy map YAML to load for localization'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([share, 'config', 'nav2.yaml'])),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([share, 'rviz', 'navigation.rviz'])),
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'), 'launch',
                    'localization_launch.py'])),
                launch_arguments={
                    'map': LaunchConfiguration('map'),
                    'use_sim_time': use_sim_time,
                    'params_file': localization_params,
                    'autostart': LaunchConfiguration('autostart'),
                    'use_composition': 'False',
                    'use_respawn': 'False',
                    'log_level': LaunchConfiguration('log_level'),
                }.items()),
        ]),
        Node(
            package='rviz2', executable='rviz2', name='localization_rviz',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen'),
    ])
