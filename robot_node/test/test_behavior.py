# SPDX-License-Identifier: Apache-2.0
import math
from types import SimpleNamespace as NS

import pytest

from robot_node.behavior import BehaviorConfig, BehaviorEngine, scan_distances


def hold(engine, label, start, duration=.35):
    for i in range(round(duration/.05)+1):
        engine.receive(label, start+i*.05)
    return start+duration


def running(choice='Pointing_Up'):
    engine = BehaviorEngine()
    hold(engine, choice, 0)
    now = hold(engine, 'Thumb_Up', .4)
    command = engine.tick(now, (0., 0., 0.))
    assert engine.active
    return engine, now, command


def test_selection_requires_confirmation_and_repeated_confirmation_does_not_restart():
    engine = BehaviorEngine()
    now = hold(engine, 'Pointing_Up', 0)
    assert engine.selected == 'back_forth'
    assert engine.tick(now, (0, 0, 0)) == (0, 0)
    now = hold(engine, 'Thumb_Up', .4)
    assert engine.tick(now, (0, 0, 0))[0] > 0
    started = engine.started
    for i in range(20):
        engine.receive('Thumb_Up', now+i*.05)
        engine.tick(now+i*.05, (0, 0, 0))
    assert engine.started == started


@pytest.mark.parametrize('label', ['Open_Palm', 'Closed_Fist'])
def test_stop_and_cancel_are_immediate(label):
    engine, now, _ = running()
    engine.receive(label, now+.01)
    assert not engine.active
    assert engine.selected is None
    assert engine.tick(now+.01, (0, 0, 0)) == (0, 0)


def test_wave_is_selected_not_executed_and_trailing_palm_keeps_selection():
    engine = BehaviorEngine()
    now = hold(engine, 'Wave', 0)
    assert engine.selected == 'greeting'
    assert engine.tick(now, (0, 0, 0)) == (0, 0)
    engine.receive('Open_Palm', .4)
    assert engine.selected == 'greeting'
    now = hold(engine, 'Thumb_Up', .45)
    assert engine.tick(now, (0, 0, 0)) == (0, .5)


def test_noisy_selections_and_confirmation_without_selection_do_nothing():
    engine = BehaviorEngine()
    for i in range(20):
        engine.receive('Pointing_Up' if i%2 else 'Victory', i*.05)
    assert engine.selected is None
    now = hold(engine, 'Thumb_Up', 1.)
    assert engine.tick(now, (0, 0, 0)) == (0, 0)


@pytest.mark.parametrize('lost', ['No_Hand', 'Unknown', 'garbage', None])
def test_signal_loss_aborts_and_requires_new_selection(lost):
    engine, now, _ = running()
    if lost is not None:
        engine.receive(lost, now+.45)
    assert engine.tick(now+.45, (0, 0, 0)) == (0, 0)
    assert engine.state == 'signal_lost'
    now = hold(engine, 'Thumb_Up', now+.5)
    assert engine.tick(now, (0, 0, 0)) == (0, 0)
    # Reacquiring the same selection works after control loss.
    hold(engine, 'Pointing_Up', now+.1)
    now = hold(engine, 'Thumb_Up', now+.5)
    assert engine.tick(now, (0, 0, 0))[0] > 0


def test_back_forth_is_exactly_four_complete_cycles_and_does_not_auto_repeat():
    engine, now, command = running()
    pose = (0., 0., 0.)
    directions = []
    for i in range(8):
        command = engine.next_command()
        directions.append(math.copysign(1, command[0]))
        pose = (pose[0]+math.copysign(.35, command[0]), 0., 0.)
        now += .1
        engine.receive('Thumb_Up', now)
        assert engine.tick(now, pose) == (0, 0)
    assert directions == [1, -1]*4
    assert engine.state == 'completed' and not engine.active
    assert pose[0] == pytest.approx(0)
    now = hold(engine, 'Thumb_Up', now+.1)
    assert engine.tick(now, pose) == (0, 0)


def test_circle_uses_unwrapped_odometry_yaw_not_an_initial_heading_match():
    engine, now, command = running('Victory')
    assert command == pytest.approx((.12, .2))
    assert engine.active
    for i in range(1, 64):
        now += .1
        yaw = i*.1
        engine.receive('Thumb_Up', now)
        engine.tick(now, (0, 0, math.atan2(math.sin(yaw), math.cos(yaw))))
    assert engine.state == 'completed'


def test_greeting_rotates_left_right_and_returns_to_original_heading():
    engine, now, _ = running('Wave')
    assert [row[1] for row in engine.phases] == [.5, -.5, .5]
    for yaw in (.45, -.45, 0.):
        now += .1
        engine.receive('Thumb_Up', now)
        engine.tick(now, (0, 0, yaw))
    assert engine.state == 'completed'


def test_obstacle_stale_sensors_progress_and_circle_preflight_stop_motion():
    for options in ({'sensor_error':'stale scan'}, {'clearance':.2}, {'around':.1}):
        engine, now, _ = running()
        assert engine.tick(now+.1, (0, 0, 0), **options) == (0, 0)
        assert engine.state == 'blocked'
    engine = BehaviorEngine()
    hold(engine, 'Victory', 0)
    now = hold(engine, 'Thumb_Up', .4)
    assert engine.tick(now, (0, 0, 0), around=1.0) == (0, 0)
    assert engine.state == 'blocked'
    engine, now, _ = running()
    engine.receive('Thumb_Up', now+5.1)
    assert engine.tick(now+5.1, (0, 0, 0)) == (0, 0)
    assert engine.state == 'blocked'


def scan():
    return NS(ranges=[float('inf')]*361, angle_min=-math.pi,
              angle_increment=math.pi/180, range_min=.164, range_max=12.)


def test_lidar_checks_correct_motion_sector_and_rejects_corrupt_scans():
    message = scan()
    assert scan_distances(message, 1) == pytest.approx((12, 12))
    message.ranges[0] = .35  # Behind the robot.
    assert scan_distances(message, 1) == pytest.approx((12, .35))
    assert scan_distances(message, -1) == pytest.approx((.35, .35))
    message.ranges[180] = .2
    assert scan_distances(message, 1) == pytest.approx((.2, .2))
    message.ranges = [float('nan')]*361
    with pytest.raises(ValueError):
        scan_distances(message, 1)
    message = scan()
    message.angle_increment = 0
    with pytest.raises(ValueError):
        scan_distances(message, 1)
    message = scan()
    message.ranges[180] = float('-inf')
    assert scan_distances(message, 1) == (0, 0)


def test_lidar_sectors_are_transformed_into_the_robot_frame():
    message = scan()
    message.ranges[45] = .35
    assert scan_distances(message, 1)[0] == pytest.approx(12)
    assert scan_distances(message, 1, sensor_yaw=math.pi/2)[0] == pytest.approx(.35)
    assert scan_distances(message, -1, sensor_yaw=math.pi/2)[0] == pytest.approx(12)


@pytest.mark.parametrize('options', [{'linear_speed':float('nan')}, {'linear_speed':.5},
                                    {'circle_radius':.01}, {'stable_time':0}])
def test_invalid_motion_parameters_are_rejected(options):
    with pytest.raises(ValueError):
        BehaviorConfig(**options)
