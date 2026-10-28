# SPDX-License-Identifier: Apache-2.0
"""ROS map image conventions, geometry and validated semantic region files."""

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np
import yaml


def finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{name} must be finite')
    return float(value)


@dataclass(frozen=True)
class Geometry:
    width: int
    height: int
    resolution: float
    origin: tuple

    def pixel_to_world(self, col, row):
        x = (col+.5)*self.resolution
        y = (self.height-row-.5)*self.resolution
        ox, oy, yaw = self.origin
        return (ox+math.cos(yaw)*x-math.sin(yaw)*y,
                oy+math.sin(yaw)*x+math.cos(yaw)*y)

    def world_to_pixel(self, x, y):
        ox, oy, yaw = self.origin
        dx, dy = x-ox, y-oy
        col = math.floor((math.cos(yaw)*dx+math.sin(yaw)*dy)/self.resolution)
        row = self.height-1-math.floor((-math.sin(yaw)*dx+math.cos(yaw)*dy)/self.resolution)
        if not (0 <= col < self.width and 0 <= row < self.height):
            raise ValueError('Position is outside the map')
        return col, row


def fingerprint(geometry, occupancy):
    # OccupancyGrid.resolution is float32; YAML uses Python doubles.
    metadata = [geometry.width, geometry.height, round(geometry.resolution, 7),
                *[round(v, 9) for v in geometry.origin]]
    data = json.dumps(metadata, separators=(',', ':')).encode()+np.asarray(occupancy, dtype=np.int8).tobytes()
    return hashlib.sha256(data).hexdigest()


@dataclass
class MapImage:
    geometry: Geometry
    image: np.ndarray
    occupancy: np.ndarray  # image order: top row first; -1 / 0 / 100
    path: Path

    @property
    def free(self):
        return self.occupancy == 0

    @property
    def signature(self):
        return fingerprint(self.geometry, self.occupancy)

    @property
    def clearance(self):
        # Outside the image is blocked too, including maps without drawn walls.
        padded = np.pad(self.free.astype(np.uint8), 1)
        return cv2.distanceTransform(padded, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)[1:-1, 1:-1]*self.geometry.resolution


def read_map(path):
    path = Path(path).expanduser().resolve()
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, dict) or data.get('mode', 'trinary') != 'trinary':
        raise ValueError('A trinary ROS occupancy map YAML is required')
    resolution = finite(data.get('resolution'), 'resolution')
    origin = data.get('origin')
    if resolution <= 0 or not isinstance(origin, list) or len(origin) != 3:
        raise ValueError('Invalid resolution/origin')
    origin = tuple(finite(v, 'origin') for v in origin)
    free = finite(data.get('free_thresh'), 'free_thresh')
    occupied = finite(data.get('occupied_thresh'), 'occupied_thresh')
    if not 0 <= free < occupied <= 1 or data.get('negate', 0) not in (0, 1):
        raise ValueError('Invalid map thresholds/negate')
    image = cv2.imread(str(path.parent/str(data.get('image', ''))), cv2.IMREAD_GRAYSCALE)
    if image is None or image.size == 0:
        raise ValueError('Cannot read map image')
    probability = image.astype(np.float64)/255.
    if not data.get('negate', 0):
        probability = 1-probability
    occupancy = np.full(image.shape, -1, dtype=np.int8)
    occupancy[probability < free] = 0
    occupancy[probability > occupied] = 100
    geometry = Geometry(image.shape[1], image.shape[0], resolution, origin)
    return MapImage(geometry, image, occupancy, path)


@dataclass(frozen=True)
class Region:
    id: str
    name: str
    aliases: tuple
    mask_value: int
    x: float
    y: float
    yaw: float


class RegionSet:
    def __init__(self, path):
        self.path = Path(path).expanduser().resolve()
        data = yaml.safe_load(self.path.read_text())
        if not isinstance(data, dict) or data.get('frame_id') != 'map' or data.get('version') != 1:
            raise ValueError('Semantic file must use version: 1 and frame_id: map')
        meta = data['map']
        width, height = meta['width'], meta['height']
        if not all(isinstance(v, int) and not isinstance(v, bool) and v > 0 for v in (width, height)):
            raise ValueError('Invalid semantic map dimensions')
        resolution = finite(meta['resolution'], 'resolution')
        origin = meta['origin']
        if resolution <= 0 or not isinstance(origin, list) or len(origin) != 3:
            raise ValueError('Invalid semantic map geometry')
        self.geometry = Geometry(width, height, resolution, tuple(finite(v, 'origin') for v in origin))
        self.signature = str(meta['fingerprint'])
        if len(self.signature) != 64 or any(c not in '0123456789abcdef' for c in self.signature):
            raise ValueError('Invalid semantic map fingerprint')
        self.labels = cv2.imread(str(self.path.parent/data['labels_image']), cv2.IMREAD_UNCHANGED)
        if self.labels is None or self.labels.ndim != 2 or self.labels.shape != (height, width):
            raise ValueError('Missing or incompatible region label image')
        self.regions = []
        for row in data['regions']:
            identifier, name = str(row['id']).strip(), str(row['name']).strip()
            aliases = row.get('aliases', [])
            value = row['mask_value']
            if (not identifier or not name or not isinstance(aliases, list)
                    or any(not isinstance(a, str) or not a.strip() for a in aliases)
                    or not isinstance(value, int) or isinstance(value, bool) or not 1 <= value <= 255):
                raise ValueError('Invalid region name/aliases/mask value')
            target = row['goal']
            region = Region(identifier, name, tuple(aliases), value,
                            *[finite(target[k], k) for k in ('x', 'y', 'yaw')])
            col, pixel_row = self.geometry.world_to_pixel(region.x, region.y)
            if self.labels[pixel_row, col] != value:
                raise ValueError(f'{identifier}: goal does not belong to its region')
            self.regions.append(region)
        if (not self.regions or len({r.id for r in self.regions}) != len(self.regions)
                or len({r.mask_value for r in self.regions}) != len(self.regions)):
            raise ValueError('Regions and mask values must be nonempty and unique')
        if set(np.unique(self.labels))-({0}|{r.mask_value for r in self.regions}):
            raise ValueError('Label image contains undeclared regions')
        self.by_id = {r.id: r for r in self.regions}

    def region_at(self, x, y):
        try:
            col, row = self.geometry.world_to_pixel(x, y)
        except ValueError:
            return None
        value = int(self.labels[row, col])
        return next((r.id for r in self.regions if r.mask_value == value), None)
