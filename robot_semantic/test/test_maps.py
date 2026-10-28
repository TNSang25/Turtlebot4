# SPDX-License-Identifier: Apache-2.0
import math
from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml

from robot_semantic.maps import Geometry, RegionSet, fingerprint, read_map
from robot_semantic.segmentation import export, segment


def fixture_map(tmp_path, negate=0, yaw=0.):
    image = np.full((80, 100), 254, np.uint8)
    image[[0, -1], :] = 0
    image[:, [0, -1]] = 0
    image[1:33, 50] = image[47:79, 50] = 0
    image[12:18, 12:18] = 205
    if negate:
        image = 255-image
    cv2.imwrite(str(tmp_path/'map.pgm'), image)
    data = dict(image='map.pgm', mode='trinary', resolution=.1, origin=[-5., -4., yaw],
                negate=negate, occupied_thresh=.65, free_thresh=.196)
    path = tmp_path/'map.yaml'
    path.write_text(yaml.safe_dump(data))
    return read_map(path)


@pytest.mark.parametrize('yaw', [0., .7, -1.2, math.pi])
def test_pixel_world_round_trip_includes_row_flip_rotation_and_half_cell(yaw):
    geometry = Geometry(100, 80, .05, (-3., 1., yaw))
    for pixel in ((0, 0), (99, 79), (50, 42)):
        assert geometry.world_to_pixel(*geometry.pixel_to_world(*pixel)) == pixel
    if yaw == 0:
        assert geometry.pixel_to_world(0, 79) == pytest.approx((-2.975, 1.025))
    with pytest.raises(ValueError):
        geometry.world_to_pixel(-100, -100)


@pytest.mark.parametrize('negate', [0, 1])
def test_occupancy_preserves_unknown_and_obstacles(tmp_path, negate):
    image = fixture_map(tmp_path, negate)
    assert image.occupancy[0, 0] == 100
    assert image.occupancy[15, 15] == -1
    assert image.occupancy[30, 30] == 0
    assert image.clearance[15, 15] == 0


def test_seeded_regions_respect_walls_unknown_and_safe_goals(tmp_path):
    image = fixture_map(tmp_path)
    seeds = []
    for identifier, pixel in zip('ABC', [(25, 25), (25, 60), (75, 40)]):
        x, y = image.geometry.pixel_to_world(*pixel)
        seeds.append(dict(id=identifier, name=identifier, aliases=[], seed=dict(x=x, y=y)))
    labels, regions = segment(image, seeds)
    assert set(np.unique(labels)) == {0, 1, 2, 3}
    assert not np.any(labels[~image.free])
    for r in regions:
        col, row = image.geometry.world_to_pixel(r['goal']['x'], r['goal']['y'])
        assert image.clearance[row, col] > .3
        assert labels[row, col] == r['mask_value']
    output = tmp_path/'config/regions.yaml'
    export(image, labels, regions, output)
    loaded = RegionSet(output)
    assert loaded.signature == image.signature
    assert loaded.region_at(regions[0]['goal']['x'], regions[0]['goal']['y']) == 'A'
    assert loaded.region_at(-100, -100) is None


def test_auto_generates_at_least_three_functional_areas(tmp_path):
    image = fixture_map(tmp_path)
    first, regions = segment(image, count=3)
    second, _ = segment(image, count=3)
    assert np.array_equal(first, second)
    assert len(regions) == 3
    assert min(r['area_m2'] for r in regions) >= 1.


def test_rejects_wall_seeds_and_disconnected_spaces(tmp_path):
    image = fixture_map(tmp_path)
    x, y = image.geometry.pixel_to_world(50, 10)
    with pytest.raises(ValueError, match='seed'):
        segment(image, [dict(id='A', seed=dict(x=x, y=y))])
    image.occupancy[:, 50] = 100
    seeds = []
    for identifier, pixel in zip('AB', [(25, 40), (75, 40)]):
        x, y = image.geometry.pixel_to_world(*pixel)
        seeds.append(dict(id=identifier, seed=dict(x=x, y=y)))
    with pytest.raises(ValueError, match='disconnected'):
        segment(image, seeds)


def test_fingerprint_changes_with_map_content_or_origin(tmp_path):
    image = fixture_map(tmp_path)
    old = image.signature
    image.occupancy[30, 30] = 100
    assert image.signature != old
    image.occupancy[30, 30] = 0
    assert image.signature == old
    other = fixture_map(tmp_path, yaw=.1)
    assert other.signature != old


def test_fingerprint_accepts_ros_float32_resolution_serialization(tmp_path):
    image = fixture_map(tmp_path)
    g = image.geometry
    ros_geometry = Geometry(g.width, g.height, float(np.float32(g.resolution)), g.origin)
    assert fingerprint(ros_geometry, image.occupancy) == image.signature


def test_checked_in_regions_match_saved_demo_map_and_have_safe_goals():
    src = Path(__file__).resolve().parents[2]
    image = read_map(src/'robot_navigation/maps/my_map.yaml')
    regions = RegionSet(src/'robot_semantic/config/regions.yaml')
    assert regions.signature == image.signature
    assert len(regions.regions) >= 3
    for r in regions.regions:
        col, row = image.geometry.world_to_pixel(r.x, r.y)
        assert image.free[row, col] and image.clearance[row, col] >= .3
