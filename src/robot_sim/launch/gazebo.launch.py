# SPDX-License-Identifier: Apache-2.0
"""Run a local SDF world; no Gazebo Fuel models or network access required."""

from pathlib import Path
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.actions import Shutdown
from launch.substitutions import LaunchConfiguration


def _launch(context):
    world_arg = LaunchConfiguration('world').perform(context)
    world = Path(world_arg).expanduser()
    if not world.is_file():
        world = Path(get_package_share_directory('robot_sim')) / 'worlds' / world_arg
        if world.suffix != '.sdf':
            world = world.with_suffix('.sdf')
    if not world.is_file():
        raise RuntimeError(f'World not found: {world_arg}. Use demo, empty, or an SDF path.')
    root = ET.parse(world).getroot()
    if root.find('world') is None:
        raise RuntimeError(f'{world} must contain an SDF <world>.')
    cmd = ['ign', 'gazebo', '-r', '-v', LaunchConfiguration('verbosity').perform(context)]
    if LaunchConfiguration('headless').perform(context) == 'true':
        cmd += ['-s', '--headless-rendering']
    cmd.append(str(world.resolve()))
    # SDF resolves package://robot_description meshes relative to this parent.
    # Keep the caller's model paths as well, for custom worlds.
    share_parent = str(Path(get_package_share_directory('robot_description')).parent)
    resources = {key: os.pathsep.join(filter(None, [share_parent, os.environ.get(key)]))
                 for key in ('IGN_GAZEBO_RESOURCE_PATH', 'GZ_SIM_RESOURCE_PATH')}
    return [LogInfo(msg=f'Gazebo Fortress world: {world}'),
            ExecuteProcess(cmd=cmd, output='screen', on_exit=Shutdown(),
                           additional_env=resources)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='demo',
                              description='Bundled world name or path to an SDF world'),
        DeclareLaunchArgument('headless', default_value='false', choices=['true', 'false'],
                              description='Server only with EGL sensor rendering'),
        DeclareLaunchArgument('verbosity', default_value='2', choices=['0', '1', '2', '3', '4']),
        OpaqueFunction(function=_launch),
    ])
