# SPDX-License-Identifier: Apache-2.0
import json
import math
from pathlib import Path
from types import SimpleNamespace as Point

import pytest

from robot_gesture.gestures import GestureFilter, WaveDetector, landmark_pose


def test_low_confidence_and_noise_do_not_select():
    gate = GestureFilter()
    assert gate.update('Victory', .3, .5, 0) == 'Unknown'
    for i in range(20):
        assert gate.update('Victory' if i%2 else 'Pointing_Up', .9, .5, i*.05) == 'Unknown'
    assert gate.update('Thumb_Up', .9, .5, 2) == 'Unknown'
    assert gate.update('Thumb_Up', .9, .5, 2.2) == 'Thumb_Up'


def test_safety_gestures_do_not_wait_for_debounce():
    gate = GestureFilter()
    assert gate.update('Open_Palm', .6, .5, 0) == 'Open_Palm'
    assert gate.update('Closed_Fist', .6, .5, .01) == 'Closed_Fist'
    assert gate.update('No_Hand', 0, 0, .1) == 'No_Hand'


def test_wave_requires_two_large_reversals_not_a_static_or_one_way_palm():
    detector = WaveDetector()
    assert not any(detector.update(.5+i*.001, i*.05) for i in range(20))
    detector.reset()
    assert not any(detector.update(.2+i*.03, i*.05) for i in range(10))
    detector.reset()
    values=[.3,.4,.5,.4,.3,.4,.5]
    result=[detector.update(value,i*.08) for i,value in enumerate(values)]
    assert result[-1]
    assert detector.update(.5,.8)  # Hold the temporal event for reliable debouncing.


def test_wave_is_canceled_by_another_gesture_and_tracking_gaps():
    gate = GestureFilter()
    for i,x in enumerate([.3,.45,.6,.45,.3,.45,.6]):
        label=gate.update('Open_Palm',.9,x,i*.08,'Left')
    assert gate.update('Open_Palm',.9,.6,.7,'Left') == 'Wave'
    assert gate.update('Closed_Fist',.9,.6,.71,'Left') == 'Closed_Fist'
    assert gate.update('Open_Palm',.9,.6,.8,'Left') == 'Open_Palm'
    detector = WaveDetector()
    for i,x in enumerate([.3,.5,.3]):
        detector.update(x,i*.1)
    assert not detector.update(.5,1.0)


@pytest.mark.parametrize('options',[{'confidence':2.0},{'stable_time':-1},{'wave_amplitude':0}])
def test_invalid_thresholds(options):
    with pytest.raises(ValueError):
        GestureFilter(**options)


FIXTURES = json.loads((Path(__file__).parent/'data/world_landmarks.json').read_text())


@pytest.mark.parametrize('fixture', FIXTURES, ids=lambda row: row['image'])
@pytest.mark.parametrize('transform', ['original', 'mirror', 'rotate_3d'])
def test_world_landmark_fallback_for_real_model_outputs(fixture, transform):
    points = []
    for x, y, z in fixture['world']:
        if transform == 'mirror':
            x = -x
        elif transform == 'rotate_3d':
            x, y = x*math.cos(.9)-y*math.sin(.9), x*math.sin(.9)+y*math.cos(.9)
            y, z = y*math.cos(.6)-z*math.sin(.6), y*math.sin(.6)+z*math.cos(.6)
        points.append(Point(x=2*x+.3, y=2*y-.7, z=2*z+.4))
    expected = {'fist.jpg': 'Closed_Fist', 'pointing_up.jpg': 'Pointing_Up',
                'victory.jpg': 'Victory', 'left_hands.jpg': 'Open_Palm',
                'thumbs_up.jpg': None}[fixture['image']]
    pose = landmark_pose(points)
    assert pose == expected
    gate = GestureFilter()
    # Reproduce a webcam frame where the gesture model returns None/weak
    # classification but its landmark model still locates the fingers.
    gate.update('None', .8, .5, 0., pose=pose)
    label = gate.update('None', .8, .5, .2, pose=pose)
    assert label == (expected or 'Unknown')
    assert label != 'Thumb_Up'  # Geometry can never confirm a motion.


def test_weak_model_confirmation_is_not_promoted_to_fist_or_thumb_up():
    row = next(row for row in FIXTURES if row['image'] == 'thumbs_up.jpg')
    pose = landmark_pose([Point(x=x, y=y, z=z) for x, y, z in row['world']])
    assert pose is None
    gate = GestureFilter()
    for i in range(20):
        assert gate.update('Thumb_Up', .6, .5, i*.05, pose=pose) == 'Unknown'


def test_wave_from_smaller_fingertip_motion_survives_one_uncertain_frame():
    gate = GestureFilter()
    for i, x in enumerate([.40, .46, .52, .46, .40, .46, .52]):
        if i == 3:
            # No command is replayed, but one weak frame does not discard
            # all the wave's motion history.
            assert gate.update('None', .8, x, i*.08, 'Left') == 'Unknown'
        else:
            gate.update('Open_Palm', .55, x, i*.08, 'Left' if i < 4 else 'Right',
                        pose='Open_Palm')
    assert gate.update('Open_Palm', .55, .52, .7, 'Right', pose='Open_Palm') == 'Wave'
    assert gate.update('No_Hand', 0, 0, .71) == 'No_Hand'
    assert gate.update('Open_Palm', .9, .52, .8) == 'Open_Palm'


def test_short_uncertain_frame_does_not_restart_static_debounce_or_repeat_commands():
    gate = GestureFilter()
    assert gate.update('Pointing_Up', .9, .5, 0) == 'Unknown'
    assert gate.update('None', .9, .5, .08) == 'Unknown'
    assert gate.update('Pointing_Up', .9, .5, .16) == 'Pointing_Up'
    assert gate.update('None', .9, .5, .5) == 'Unknown'
    assert gate.update('Pointing_Up', .9, .5, .55) == 'Unknown'
    assert gate.update('No_Hand', 0, 0, .6) == 'No_Hand'
    assert gate.update('Pointing_Up', .9, .5, .7) == 'Unknown'


def test_invalid_landmarks_and_partial_hand_cannot_create_gestures():
    assert landmark_pose([]) is None
    assert landmark_pose([Point(x=0, y=0, z=0)]*21) is None
    assert landmark_pose([Point(x=math.nan, y=0, z=0)]*21) is None
    row = next(row for row in FIXTURES if row['image'] == 'pointing_up.jpg')
    points = [Point(x=x, y=y, z=z) for x, y, z in row['world']]
    # A half-curled/occluded index is deliberately ambiguous.
    points[8] = Point(x=points[6].x, y=points[6].y, z=points[6].z)
    assert landmark_pose(points) is None
