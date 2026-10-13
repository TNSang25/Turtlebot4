# SPDX-License-Identifier: Apache-2.0
"""Save the occupancy map to <map_name>.yaml and <map_name>.pgm."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_name', default_value='src/robot_navigation/maps/my_map',
            description=(
                'Output path without extension; parent directory must exist. '
                'Run from the workspace root for the default path.')),
        DeclareLaunchArgument('map_topic', default_value='/map'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='nav2_map_server', executable='map_saver_cli',
            name='map_saver', output='screen',
            arguments=['-f', LaunchConfiguration('map_name')],
            remappings=[('map', LaunchConfiguration('map_topic'))],
            parameters=[{
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
                'save_map_timeout': 10.0,
                'free_thresh_default': 0.25,
                'occupied_thresh_default': 0.65,
                'map_subscribe_transient_local': True,
            }],
            on_exit=[EmitEvent(event=Shutdown(reason='Map saver finished'))]),
    ])
