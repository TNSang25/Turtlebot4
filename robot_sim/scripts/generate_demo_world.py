#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Generate a local Gazebo world and matching ground-truth occupancy map."""

import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


def element(parent, tag, text=None, **attributes):
    child = ET.SubElement(parent, tag, attributes)
    if text is not None:
        child.text = str(text)
    return child


def make_world(layout, empty=False):
    sdf = ET.Element('sdf', version='1.8')
    world = element(sdf, 'world', name='empty' if empty else layout['world_name'])
    physics = element(world, 'physics', name='1ms', type='ignored')
    element(physics, 'max_step_size', 0.001)
    element(physics, 'real_time_factor', 1.0)
    for filename, classname in [
        ('physics', 'Physics'), ('user-commands', 'UserCommands'),
        ('scene-broadcaster', 'SceneBroadcaster'), ('sensors', 'Sensors'), ('imu', 'Imu'),
    ]:
        plugin = element(world, 'plugin', filename=f'ignition-gazebo-{filename}-system',
                         name=f'ignition::gazebo::systems::{classname}')
        if classname == 'Sensors':
            element(plugin, 'render_engine', 'ogre2')
    element(world, 'gravity', '0 0 -9.81')
    scene = element(world, 'scene')
    element(scene, 'ambient', '0.6 0.6 0.6 1')
    element(scene, 'background', '0.85 0.9 0.95 1')
    light = element(world, 'light', name='sun', type='directional')
    element(light, 'pose', '0 0 10 0 0 0')
    element(light, 'diffuse', '0.9 0.9 0.9 1')
    element(light, 'specular', '0.2 0.2 0.2 1')
    element(light, 'direction', '-0.5 0.1 -0.9')
    element(light, 'cast_shadows', 'true')
    attenuation = element(light, 'attenuation')
    element(attenuation, 'range', 1000)
    element(attenuation, 'constant', 0.9)
    element(attenuation, 'linear', 0.01)
    element(attenuation, 'quadratic', 0.001)

    def box(name, center, size, color):
        model = element(world, 'model', name=name)
        element(model, 'static', 'true')
        element(model, 'pose', f'{center[0]} {center[1]} {center[2]} 0 0 0')
        link = element(model, 'link', name='link')
        for tag in ('collision', 'visual'):
            obj = element(link, tag, name=tag)
            geometry = element(obj, 'geometry')
            element(element(geometry, 'box'), 'size', ' '.join(map(str, size)))
            if tag == 'visual':
                material = element(obj, 'material')
                element(material, 'ambient', color)
                element(material, 'diffuse', color)
    box('ground', [0, 0, -0.05], [20, 20, 0.1], '0.72 0.74 0.76 1')
    if not empty:
        for item in layout['obstacles']:
            height = item['height']
            color = '0.25 0.4 0.6 1' if 'wall' not in item['name'] else '0.75 0.68 0.56 1'
            box(item['name'], [*item['center'], height / 2], [*item['size'], height], color)
    ET.indent(sdf, space='  ')
    return ET.tostring(sdf, encoding='unicode', xml_declaration=True) + '\n'


def make_map(layout):
    resolution = layout['resolution']
    width, height = [round(size / resolution) for size in layout['size']]
    origin_x, origin_y = layout['origin']
    # Unknown beyond the enclosed room. PGM rows go top-to-bottom; ROS origin
    # is the lower-left corner. Fill every cell intersecting an obstacle.
    pixels = bytearray([205] * (width * height))
    for row in range(height):
        y = origin_y + (height - row - 0.5) * resolution
        for col in range(width):
            x = origin_x + (col + 0.5) * resolution
            if abs(x) < 4.9 and abs(y) < 4.9:
                pixels[row * width + col] = 254
    for item in layout['obstacles']:
        x, y = item['center']
        sx, sy = item['size']
        x0 = max(0, math.floor((x - sx / 2 - origin_x) / resolution + 1e-9))
        x1 = min(width, math.ceil((x + sx / 2 - origin_x) / resolution - 1e-9))
        y0 = max(0, math.floor((y - sy / 2 - origin_y) / resolution + 1e-9))
        y1 = min(height, math.ceil((y + sy / 2 - origin_y) / resolution - 1e-9))
        for iy in range(y0, y1):
            for ix in range(x0, x1):
                pixels[(height - 1 - iy) * width + ix] = 0
    return f'P5\n# Ground truth from demo_layout.yaml, not a SLAM recording\n{width} {height}\n255\n'.encode() + pixels


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package-dir', type=Path,
                        default=Path(__file__).resolve().parents[1],
                        help='Source robot_sim directory (do not target a read-only install)')
    args = parser.parse_args()
    root = args.package_dir
    layout = yaml.safe_load((root / 'config/demo_layout.yaml').read_text())
    for directory in ('worlds', 'maps'):
        (root / directory).mkdir(parents=True, exist_ok=True)
    (root / 'worlds/demo.sdf').write_text(make_world(layout))
    (root / 'worlds/empty.sdf').write_text(make_world(layout, empty=True))
    (root / 'maps/demo.pgm').write_bytes(make_map(layout))
    (root / 'maps/demo.yaml').write_text(yaml.safe_dump({
        'image': 'demo.pgm', 'mode': 'trinary', 'resolution': layout['resolution'],
        'origin': [*layout['origin'], 0.0], 'negate': 0,
        'occupied_thresh': 0.65, 'free_thresh': 0.196,
    }, sort_keys=False))
    print(f'Generated demo.sdf, empty.sdf and demo map in {root}')


if __name__ == '__main__':
    main()
