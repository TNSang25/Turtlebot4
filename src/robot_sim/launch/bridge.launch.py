# SPDX-License-Identifier: Apache-2.0
"""Bridge simulation topics using one YAML configuration."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = Path(get_package_share_directory('robot_sim')) / 'config/bridge_config.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name='simulation_bridge', output='screen',
            parameters=[{
                'config_file': str(config),
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
            }],
        ),
    ])
