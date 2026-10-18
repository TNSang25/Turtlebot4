# SPDX-License-Identifier: Apache-2.0
"""Goal-file validation and occupancy checks independent of ROS executors."""

from dataclasses import dataclass
import math
from pathlib import Path

import yaml


@dataclass(frozen=True)
class Goal:
    name: str
    x: float
    y: float
    yaw: float


def read_goals(path):
    data = yaml.safe_load(Path(path).expanduser().read_text())
    if not isinstance(data, dict) or data.get('frame_id') != 'map':
        raise ValueError('Goal file must use frame_id: map')
    rows = data.get('goals')
    if not isinstance(rows, list) or len(rows) < 3:
        raise ValueError('Provide at least three goals')
    goals = []
    for index, row in enumerate(rows, 1):
        if not isinstance(row, dict):
            raise ValueError(f'Goal {index} must contain x, y and yaw')
        values = []
        for key in ('x', 'y', 'yaw'):
            value = row.get(key)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f'Goal {index}: {key} must be a finite number')
            values.append(float(value))
        goals.append(Goal(str(row.get('name', f'Goal {index}')), *values))
    if len({goal.name for goal in goals}) != len(goals):
        raise ValueError('Goal names must be unique')
    if len({(round(goal.x, 3), round(goal.y, 3)) for goal in goals}) < 3:
        raise ValueError('Choose at least three different positions on the map')
    return goals


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def check_goal_clearance(goal, grid, radius=0.22):
    """Reject goals whose circular footprint overlaps occupied/unknown/outside cells."""
    if grid.header.frame_id != 'map':
        raise ValueError('The occupancy grid must be in frame map')
    resolution = grid.info.resolution
    if resolution <= 0 or len(grid.data) != grid.info.width * grid.info.height:
        raise ValueError('Invalid occupancy grid')
    origin = grid.info.origin
    yaw = yaw_from_quaternion(origin.orientation)
    dx, dy = goal.x - origin.position.x, goal.y - origin.position.y
    x = math.cos(yaw) * dx + math.sin(yaw) * dy
    y = -math.sin(yaw) * dx + math.cos(yaw) * dy
    if x - radius < 0 or y - radius < 0 or x + radius >= grid.info.width * resolution or y + radius >= grid.info.height * resolution:
        raise ValueError(f'{goal.name}: footprint is outside the saved map')
    for iy in range(math.floor((y - radius) / resolution), math.floor((y + radius) / resolution) + 1):
        for ix in range(math.floor((x - radius) / resolution), math.floor((x + radius) / resolution) + 1):
            # Distance to the nearest point in this cell, including partial overlap.
            nx = max(ix * resolution, min(x, (ix + 1) * resolution))
            ny = max(iy * resolution, min(y, (iy + 1) * resolution))
            if math.hypot(nx - x, ny - y) > radius:
                continue
            occupancy = grid.data[iy * grid.info.width + ix]
            if occupancy < 0 or occupancy >= 65:
                raise ValueError(f'{goal.name}: footprint overlaps unknown/occupied map cells')
