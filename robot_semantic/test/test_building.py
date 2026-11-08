# SPDX-License-Identifier: Apache-2.0
"""Check actual SDF collisions, not a separate drawing of the room layout."""

from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import yaml

from robot_semantic.maps import read_map
from robot_semantic.segmentation import segment


def test_building_has_connected_robot_sized_doors_and_five_safe_named_goals(tmp_path):
    src = Path(__file__).resolve().parents[2]
    world = ET.parse(src/'robot_sim/worlds/semantic_building.sdf').getroot().find('world')
    seeds = yaml.safe_load((src/'robot_semantic/config/semantic_building_seeds.yaml').read_text())['regions']
    # Temporary ideal geometry fixture, NOT a SLAM map shipped to the user.
    resolution, origin = .05, [-8.5, -6.5, 0.]
    rows, cols = np.indices((260, 340))
    x, y = origin[0]+(cols+.5)*resolution, origin[1]+(260-rows-.5)*resolution
    image = np.where((np.abs(x) < 8) & (np.abs(y) < 6), 254, 205).astype(np.uint8)
    for model in world.findall('model'):
        if model.get('name') in ('ground', 'labelled_floor'):
            continue
        px, py, pz, *_ = map(float, model.findtext('pose').split())
        for collision in model.findall('link/collision'):
            sx, sy, sz = map(float, collision.findtext('geometry/box/size').split())
            image[(np.abs(x-px) <= sx/2) & (np.abs(y-py) <= sy/2)] = 0
    cv2.imwrite(str(tmp_path/'building.pgm'), image)
    (tmp_path/'building.yaml').write_text(yaml.safe_dump(dict(image='building.pgm', mode='trinary',
        resolution=resolution, origin=origin, negate=0, occupied_thresh=.65, free_thresh=.196)))
    map_image = read_map(tmp_path/'building.yaml')
    labels, regions = segment(map_image, seeds)
    assert len(regions) == 5
    traversable = (map_image.clearance >= .4).astype(np.uint8)
    _, components = cv2.connectedComponents(traversable, connectivity=4)
    points = []
    for r in regions:
        col, row = map_image.geometry.world_to_pixel(r['goal']['x'], r['goal']['y'])
        assert map_image.clearance[row, col] >= .4
        assert labels[row, col] == r['mask_value']
        points.append(components[row, col])
    assert points[0] > 0 and len(set(points)) == 1
    # Check each doorway connects the corridor and its room with clearance.
    for px in (-4., 4.):
        for py in (-1.6, 1.6):
            col, row = map_image.geometry.world_to_pixel(px, py)
            assert map_image.clearance[row, col] >= .8
