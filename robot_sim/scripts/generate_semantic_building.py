#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Generate a local building world, labelled floor and matching SLAM seed template.

This produces geometry and a layout illustration, not a recorded SLAM map.
Run from the source tree; all assets needed by Gazebo are checked in.
"""

from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET

from PIL import Image, ImageDraw, ImageFont
import yaml


def walls(layout):
    """Four rooms with doors opening onto the central east-west corridor."""
    width, depth = layout['size']
    half_x, half_y = width/2, depth/2
    t, h = layout['wall_thickness'], layout['wall_height']
    c, door = layout['corridor_width']/2, layout['door_width']
    result = [
        ('north_wall', (0, half_y+t/2), (width+2*t, t)),
        ('south_wall', (0, -half_y-t/2), (width+2*t, t)),
        ('west_wall', (-half_x-t/2, 0), (t, depth)),
        ('east_wall', (half_x+t/2, 0), (t, depth)),
    ]
    for side, sign in [('north', 1), ('south', -1)]:
        result.append((f'{side}_room_divider', (0, sign*(c+t+half_y)/2),
                       (t, half_y-c-t)))
        # Room centres at +/- width/4; clear door opening is exactly door_width.
        gaps = [(-half_x/2-door/2, -half_x/2+door/2),
                (half_x/2-door/2, half_x/2+door/2)]
        edges = [(-half_x, gaps[0][0]), (gaps[0][1], gaps[1][0]), (gaps[1][1], half_x)]
        for index, (left, right) in enumerate(edges):
            result.append((f'{side}_corridor_wall_{index}', ((left+right)/2, sign*(c+t/2)),
                           (right-left, t)))
    return [dict(name=name, center=list(center), size=list(size), height=h)
            for name, center, size in result]


def world_xml(layout):
    # Reuse the simulation plugins and lighting already verified in arena.sdf.
    sim = Path(__file__).resolve().parents[1]
    root = ET.parse(sim/'worlds/arena.sdf').getroot()
    world = root.find('world')
    world.set('name', layout['world_name'])
    ground = deepcopy(world.find("model[@name='ground']"))
    for model in list(world.findall('model')):
        world.remove(model)
    width, depth = layout['size']
    for size in ground.findall('.//box/size'):
        size.text = f'{width+2} {depth+2} 0.1'
    world.append(ground)
    for item in walls(layout)+layout['furniture']:
        model = ET.SubElement(world, 'model', name=item['name'])
        ET.SubElement(model, 'static').text = 'true'
        x, y = item['center']
        sx, sy = item['size']
        height = item['height']
        ET.SubElement(model, 'pose').text = f'{x} {y} {height/2} 0 0 0'
        link = ET.SubElement(model, 'link', name='link')
        for kind in ('collision', 'visual'):
            obj = ET.SubElement(link, kind, name=kind)
            geometry = ET.SubElement(obj, 'geometry')
            ET.SubElement(ET.SubElement(geometry, 'box'), 'size').text = f'{sx} {sy} {height}'
            if kind == 'visual':
                material = ET.SubElement(obj, 'material')
                color = '0.78 0.79 0.81 1' if height == layout['wall_height'] else '0.48 0.32 0.19 1'
                for tag in ('ambient', 'diffuse'):
                    ET.SubElement(material, tag).text = color
    # A visual plane cannot obstruct the wheels or produce lidar returns.
    model = ET.SubElement(world, 'model', name='labelled_floor')
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = '0 0 0.003 0 0 0'
    visual = ET.SubElement(ET.SubElement(model, 'link', name='link'), 'visual', name='floor_labels')
    plane = ET.SubElement(ET.SubElement(visual, 'geometry'), 'plane')
    ET.SubElement(plane, 'normal').text = '0 0 1'
    ET.SubElement(plane, 'size').text = f'{width} {depth}'
    material = ET.SubElement(visual, 'material')
    ET.SubElement(material, 'diffuse').text = '1 1 1 1'
    metal = ET.SubElement(ET.SubElement(material, 'pbr'), 'metal')
    ET.SubElement(metal, 'albedo_map').text = 'model://semantic_building/materials/textures/floor.png'
    ET.SubElement(metal, 'roughness').text = '1'
    ET.SubElement(metal, 'metalness').text = '0'
    ET.indent(root, space='  ')
    return ET.tostring(root, encoding='unicode', xml_declaration=True)+'\n'


def draw_layout(layout, output, preview=False):
    width, depth = layout['size']
    scale = 100
    image = Image.new('RGB', (round(width*scale), round(depth*scale)), (225, 225, 225))
    draw = ImageDraw.Draw(image)
    font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 27)
    small = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 19)

    def pixel(x, y):
        return round((x+width/2)*scale), round((depth/2-y)*scale)

    def rectangle(bounds, fill):
        xmin, ymin, xmax, ymax = bounds
        draw.rectangle([pixel(xmin, ymax), pixel(xmax, ymin)], fill=fill)

    for region in layout['regions']:
        rectangle(region['bounds'], tuple(region['color']))
        x, y = region['seed']['x'], region['seed']['y']
        label_y = y+.7 if region['id'] != 'E' else y+.6
        draw.text(pixel(x, label_y), f"{region['id']} — {region['name']}",
                  font=font, fill=(25, 30, 35), anchor='mm')
        draw.text(pixel(x, label_y-.4), f'({x:.1f}, {y:.1f})',
                  font=small, fill=(45, 45, 45), anchor='mm')
        px, py = pixel(x, y)
        draw.ellipse((px-8, py-8, px+8, py+8), fill=(25, 55, 110))
    if preview:
        for item in walls(layout)+layout['furniture']:
            x, y = item['center']
            sx, sy = item['size']
            rectangle([x-sx/2, y-sy/2, x+sx/2, y+sy/2], (75, 75, 75))
        draw.text(pixel(0, -.55), 'Robot / SLAM start: (0, 0, yaw=0)',
                  font=small, fill=(30, 30, 30), anchor='mm')
        draw.text((14, 14), 'Layout illustration — 16 m × 12 m — doors 1.8 m',
                  font=small, fill=(10, 10, 10))
    output.parent.mkdir(parents=True, exist_ok=True)
    image.save(output)


def main():
    sim = Path(__file__).resolve().parents[1]
    layout = yaml.safe_load((sim/'config/semantic_building.yaml').read_text())
    (sim/'worlds/semantic_building.sdf').write_text(world_xml(layout))
    draw_layout(layout, sim/'models/semantic_building/materials/textures/floor.png')
    draw_layout(layout, sim/'maps/semantic_building_layout.png', preview=True)
    seeds = [{k: v for k, v in region.items() if k != 'color'} for region in layout['regions']]
    target = sim.parent/'robot_semantic/config/semantic_building_seeds.yaml'
    target.write_text(
        '# Template for a NEW SLAM map of world semantic_building.\n'
        '# Start robot/SLAM at (0,0,0); check coordinates against the saved map.\n'
        '# bounds: [xmin, ymin, xmax, ymax] in map frame; edit if SLAM frame differs.\n'
        +yaml.safe_dump(dict(regions=seeds), allow_unicode=True, sort_keys=False))
    print('Generated semantic_building.sdf, labelled floor, layout preview and seed template.')


if __name__ == '__main__':
    main()
