# SPDX-License-Identifier: Apache-2.0
import math
from types import SimpleNamespace as NS

import pytest
import yaml

from robot_node.goals import Goal, check_goal_clearance, read_goals


def grid(yaw=0.0):
    return NS(header=NS(frame_id='map'), info=NS(
        resolution=0.1, width=50, height=50,
        origin=NS(position=NS(x=0.0, y=0.0),
                  orientation=NS(x=0.0, y=0.0, z=math.sin(yaw/2), w=math.cos(yaw/2)))),
        data=[0]*2500)


def test_rotated_map_and_clearance():
    check_goal_clearance(Goal('good', -2, 2, 0), grid(math.pi/2))
    with pytest.raises(ValueError, match='outside'):
        check_goal_clearance(Goal('edge', 0.1, 2, 0), grid())


@pytest.mark.parametrize('occupancy', [-1, 65, 100])
def test_footprint_overlaps_unknown_or_obstacle(occupancy):
    message=grid()
    message.data[20*50+22]=occupancy
    with pytest.raises(ValueError, match='unknown/occupied'):
        check_goal_clearance(Goal('too close', 2, 2, 0), message)


def test_goal_file_preserves_yaw_and_rejects_invalid_routes(tmp_path):
    path=tmp_path/'goals.yaml'
    data={'frame_id':'map','goals':[
        {'name':f'G{i}', 'x':float(i), 'y':0.0,'yaw':0.3*i} for i in range(3)]}
    path.write_text(yaml.safe_dump(data))
    goals=read_goals(path)
    assert goals[-1].yaw==pytest.approx(0.6)
    for key,value in [('x',float('nan')),('yaw',True),('y','invalid')]:
        old=data['goals'][0][key]
        data['goals'][0][key]=value
        path.write_text(yaml.safe_dump(data))
        with pytest.raises(ValueError, match='finite'):
            read_goals(path)
        data['goals'][0][key]=old
    data['goals']=data['goals'][:2]
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='at least three'):
        read_goals(path)
