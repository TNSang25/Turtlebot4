# SPDX-License-Identifier: Apache-2.0
"""Regressions for the complete recognition -> behavior input sequence."""

import pytest

from robot_gesture.gestures import GestureFilter
from robot_node.behavior import BehaviorEngine


@pytest.mark.parametrize('choice,action', [('Pointing_Up', 'back_forth'), ('Victory', 'circle')])
def test_delayed_thumb_at_076_confidence_starts_the_remembered_behavior(choice, action):
    recognition = GestureFilter()
    behavior = BehaviorEngine()

    def frame(raw, score, now):
        label = recognition.update(raw, score, .5, now)
        behavior.receive(label, now)
        return behavior.tick(now, (0, 0, 0))

    for i in range(10):
        assert frame(choice, .9, i/15) == (0, 0)
    assert behavior.selected == action
    for i in range(11, 150):
        assert frame('None' if i % 2 else 'No_Hand', .8, i/15) == (0, 0)
        assert behavior.selected == action
    for i in range(150, 160):
        command = frame('Thumb_Up', .76, i/15)
    assert behavior.active
    assert behavior.action == action
    assert command[0] > 0
    # A dropout during motion still stops it and consumes the old command.
    assert frame('No_Hand', 0., 11.) == (0, 0)
    assert not behavior.active
    assert behavior.selected is None
    for i in range(170, 180):
        assert frame('Thumb_Up', .76, i/15) == (0, 0)


def test_one_high_confidence_thumb_frame_cannot_execute_a_pending_behavior():
    recognition = GestureFilter()
    behavior = BehaviorEngine()
    for i in range(10):
        behavior.receive(recognition.update('Pointing_Up', .9, .5, i/15), i/15)
    assert behavior.selected == 'back_forth'
    for raw, score, now in [('Thumb_Up', .95, 2.), ('None', .9, 2.1), ('No_Hand', 0., 3.)]:
        behavior.receive(recognition.update(raw, score, .5, now), now)
        assert behavior.tick(now, (0, 0, 0)) == (0, 0)
    assert behavior.selected == 'back_forth'
