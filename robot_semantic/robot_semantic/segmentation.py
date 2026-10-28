# SPDX-License-Identifier: Apache-2.0
"""Seeded geodesic segmentation; labels never propagate through walls/unknown."""

import argparse
import heapq
import math
import os
from pathlib import Path

import cv2
import numpy as np
import yaml

from .maps import finite, read_map

COLORS = [(245, 160, 80), (90, 205, 110), (90, 120, 245), (200, 100, 200), (70, 205, 220)]  # BGR


def segment(map_image, seeds=None, count=3, clearance=.30, min_area=1.):
    if not math.isfinite(clearance) or clearance <= 0 or not math.isfinite(min_area) or min_area <= 0:
        raise ValueError('clearance and min_area must be positive')
    if not isinstance(count, int) or not 1 <= count <= 26:
        raise ValueError('count must be between 1 and 26')
    free = cv2.morphologyEx(map_image.free.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)) > 0
    distance = map_image.clearance
    safe = free & (distance >= clearance+map_image.geometry.resolution)
    components, component_labels = cv2.connectedComponents(free.astype(np.uint8), connectivity=4)
    if not np.any(safe):
        raise ValueError('No free cell has enough clearance for a robot goal')
    if seeds is None:
        # Choose the largest traversable component. Automatic labels describe
        # functional areas, not inferred room names or guaranteed doorways.
        largest = max(range(1, components), key=lambda k: np.count_nonzero(component_labels == k))
        safe &= component_labels == largest
        if np.count_nonzero(safe) < count:
            raise ValueError('Insufficient safe cells for requested region count')
        yy, xx = np.indices(free.shape)
        nearest = np.full(free.shape, np.inf)
        seeds = []
        for i in range(count):
            score = distance if i == 0 else nearest*np.minimum(distance, 1.)
            row, col = np.unravel_index(np.argmax(np.where(safe, score, -1)), free.shape)
            x, y = map_image.geometry.pixel_to_world(col, row)
            seeds.append(dict(id=chr(65+i), name=f'Khu vực {chr(65+i)}', aliases=[], seed=dict(x=x, y=y)))
            nearest = np.minimum(nearest, np.hypot(xx-col, yy-row)*map_image.geometry.resolution)
    if not isinstance(seeds, list) or not 1 <= len(seeds) <= 26:
        raise ValueError('Provide 1 to 26 seed regions')
    labels = np.zeros(free.shape, dtype=np.uint8)
    costs = np.full(free.shape, np.inf)
    queue, pixels, ids = [], [], set()
    component = None
    for index, seed in enumerate(seeds, 1):
        identifier = str(seed['id']).strip()
        if not identifier or identifier in ids:
            raise ValueError('Seed IDs must be unique and nonempty')
        ids.add(identifier)
        x, y = [finite(seed['seed'][k], k) for k in ('x', 'y')]
        col, row = map_image.geometry.world_to_pixel(x, y)
        if not safe[row, col]:
            raise ValueError(f'{identifier}: seed is occupied, unknown, or too near an obstacle')
        if (col, row) in pixels:
            raise ValueError('Seeds must occupy different pixels')
        pixels.append((col, row))
        current = component_labels[row, col]
        if component is not None and current != component:
            raise ValueError('Seeds are in disconnected free spaces; no route can join them')
        component = current
        costs[row, col] = 0
        labels[row, col] = index
        heapq.heappush(queue, (0., index, row, col))
    while queue:
        cost, label, row, col = heapq.heappop(queue)
        if cost != costs[row, col] or label != labels[row, col]:
            continue
        for dy, dx in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            nr, nc = row+dy, col+dx
            if 0 <= nr < free.shape[0] and 0 <= nc < free.shape[1] and free[nr, nc] and cost+1 < costs[nr, nc]:
                costs[nr, nc] = cost+1
                labels[nr, nc] = label
                heapq.heappush(queue, (cost+1, label, nr, nc))
    regions = []
    for index, seed in enumerate(seeds, 1):
        mask = labels == index
        area = np.count_nonzero(mask)*map_image.geometry.resolution**2
        if area < min_area:
            raise ValueError(f'{seed["id"]}: region too small ({area:.2f} m2); move seeds apart')
        # A supplied seed is an intentional destination and was clearance-checked.
        # Snap to its cell center to make pixel/ROS coordinate conversion explicit.
        x, y = map_image.geometry.pixel_to_world(*pixels[index-1])
        aliases = seed.get('aliases', [])
        if not isinstance(aliases, list) or any(not isinstance(a, str) for a in aliases):
            raise ValueError('Aliases must be strings')
        regions.append(dict(id=str(seed['id']), name=str(seed.get('name', seed['id'])),
                            aliases=aliases, mask_value=index, area_m2=round(area, 3),
                            goal=dict(x=round(x, 6), y=round(y, 6), yaw=finite(seed.get('yaw', 0.), 'yaw'))))
    return labels, regions


def preview(map_image, labels, regions):
    image = cv2.cvtColor(map_image.image, cv2.COLOR_GRAY2BGR)
    for region in regions:
        mask = labels == region['mask_value']
        color = np.array(COLORS[(region['mask_value']-1) % len(COLORS)], dtype=np.uint8)
        image[mask] = (.4*image[mask]+.6*color).astype(np.uint8)
    image = cv2.resize(image, None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST)
    for region in regions:
        col, row = map_image.geometry.world_to_pixel(region['goal']['x'], region['goal']['y'])
        p = (col*4+2, row*4+2)
        cv2.circle(image, p, 6, (0, 0, 0), -1)
        cv2.putText(image, region['id'], (p[0]+10, p[1]-8), cv2.FONT_HERSHEY_SIMPLEX, .8, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(image, region['id'], (p[0]+10, p[1]-8), cv2.FONT_HERSHEY_SIMPLEX, .8, (255, 255, 255), 1, cv2.LINE_AA)
    return image


def export(map_image, labels, regions, output):
    output = Path(output).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    image_dir = output.parent.parent/'maps'
    image_dir.mkdir(parents=True, exist_ok=True)
    label_path = image_dir/(output.stem+'_labels.png')
    preview_path = image_dir/(output.stem+'_preview.png')
    if not cv2.imwrite(str(label_path), labels) or not cv2.imwrite(str(preview_path), preview(map_image, labels, regions)):
        raise RuntimeError('Failed to write segmentation images')
    g = map_image.geometry
    data = dict(version=1, frame_id='map', map=dict(width=g.width, height=g.height,
                resolution=g.resolution, origin=list(g.origin), fingerprint=map_image.signature),
                labels_image=os.path.relpath(label_path, output.parent), regions=regions)
    output.write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False))
    return preview_path


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', required=True, help='Saved ROS map YAML')
    parser.add_argument('--seeds', help='Optional YAML with regions: [{id, name, aliases, seed: {x,y}}]')
    parser.add_argument('--count', type=int, default=3, help='Number of automatic functional areas without --seeds')
    parser.add_argument('--clearance', type=float, default=.30)
    parser.add_argument('--min-area', type=float, default=1.)
    parser.add_argument('--output', required=True, help='Semantic regions YAML; images written to sibling maps/')
    options = parser.parse_args(args)
    try:
        m = read_map(options.map)
        seeds = yaml.safe_load(Path(options.seeds).read_text())['regions'] if options.seeds else None
        labels, regions = segment(m, seeds, options.count, options.clearance, options.min_area)
        image = export(m, labels, regions, options.output)
        print(f'Saved {len(regions)} regions: {options.output}\nPreview: {image}')
        for r in regions:
            print(f'{r["id"]}: {r["name"]}; goal={r["goal"]}; area={r["area_m2"]} m2')
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.exit(1, f'Segmentation failed: {error}\n')


if __name__ == '__main__':
    main()
