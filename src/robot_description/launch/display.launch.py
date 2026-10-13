"""View the model in RViz without starting Gazebo."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def _setup(context):
    share = Path(get_package_share_directory('robot_description'))
    description = xacro.process_file(
        str(share / 'urdf' / 'turtlebot4.urdf.xacro'),
        mappings={'simulation': 'false', 'robot_name': 'turtlebot4'},
    ).toxml()
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description, 'use_sim_time': False}]),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             parameters=[{'use_sim_time': False}]),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(share / 'rviz' / 'description.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz.'),
        OpaqueFunction(function=_setup),
    ])
