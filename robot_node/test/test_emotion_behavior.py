# SPDX-License-Identifier: Apache-2.0
import math
import pytest
from robot_node.emotion_behavior import EmotionConfig, EmotionEngine


def start(label):
    engine = EmotionEngine()
    engine.receive(label, 0.)
    assert engine.tick(0., (0., 0., 0.)) == (0., 0.)
    engine.receive(label, .1)
    command = engine.tick(.1, (0., 0., 0.))
    assert engine.action == label
    return engine, command


@pytest.mark.parametrize('label', ['happy', 'sad', 'angry', 'fear', 'surprise'])
def test_trajectory_endpoint_and_finite_motion_without_auto_repeating(label):
    engine, command = start(label)
    pose = [0., 0., 0.]
    total_yaw = 0.
    dt = .02
    for i in range(1600):
        now = .12+i*dt
        v, w = command
        yaw = pose[2]+w*dt/2
        pose[0] += v*math.cos(yaw)*dt
        pose[1] += v*math.sin(yaw)*dt
        total_yaw += w*dt
        pose[2] = math.atan2(math.sin(total_yaw), math.cos(total_yaw))
        engine.receive(label, now)
        command = engine.tick(now, pose)
        if engine.state == 'completed':
            break
    assert engine.state == 'completed'
    assert command == (0., 0.)
    if label in ('happy', 'sad'):
        assert pose[0] == pytest.approx(.60, abs=.03)
        assert pose[1] == pytest.approx(.60 if label == 'happy' else -.60, abs=.03)
    elif label == 'surprise':
        assert total_yaw == pytest.approx(2*math.pi, abs=.04)
        assert math.hypot(*pose[:2]) == 0.
    else:
        assert pose[0] == pytest.approx(.25 if label == 'fear' else -.25, abs=.02)
    for i in range(30):
        now += dt
        engine.receive(label, now)
        assert engine.tick(now, pose) == (0., 0.)
    engine.receive('neutral', now+.1)
    engine.receive(label, now+.2)
    engine.receive(label, now+.3)
    assert engine.tick(now+.3, pose) != (0., 0.)


@pytest.mark.parametrize('label', ['neutral', 'no_face', 'multiple_faces'])
def test_stop_and_face_loss_are_immediate(label):
    engine, _ = start('happy')
    engine.receive(label, .11)
    assert engine.action is None
    assert engine.tick(.11, (0., 0., 0.)) == (0., 0.)
    assert engine.inhibited == (label != 'neutral')


@pytest.mark.parametrize('messages', ['absent', 'uncertain', 'disgust'])
def test_stream_loss_or_continuous_uncertainty_stops_and_neutral_rearms(messages):
    engine, _ = start('surprise')
    for i in range(1, 15):
        now = .1+i*.1
        if messages != 'absent':
            engine.receive(messages, now)
        engine.tick(now, (0., 0., .1))
    assert engine.action is None and engine.inhibited
    for i in range(4):
        engine.receive('surprise', 2+i*.1)
        assert engine.tick(2+i*.1, (0., 0., .1)) == (0., 0.)
    engine.receive('neutral', 3.)
    engine.receive('surprise', 3.1)
    engine.receive('surprise', 3.2)
    assert engine.tick(3.2, (0., 0., .1))[1] > 0


def test_short_uncertain_dropout_can_recover_without_restarting_motion():
    engine, _ = start('happy')
    began = engine.started
    engine.receive('uncertain', .2)
    assert engine.tick(.2, (0., 0., .01))[0] > 0
    engine.receive('happy', .3)
    assert engine.tick(.3, (0., 0., .02))[0] > 0
    assert engine.started == began


def test_different_stable_expression_stops_then_changes_motion():
    engine, _ = start('happy')
    engine.receive('sad', .2)
    assert engine.action == 'happy'
    engine.receive('sad', .3)
    assert engine.action is None
    assert engine.tick(.3, (0., 0., .1))[1] < 0


@pytest.mark.parametrize('label', ['happy', 'sad', 'angry', 'surprise', 'fear'])
def test_sensor_errors_and_obstacles_stop_active_motion(label):
    engine, _ = start(label)
    engine.receive(label, .2)
    assert engine.tick(.2, (0., 0., 0.), sensor_error='Old scan') == (0., 0.)
    assert engine.inhibited
    engine, _ = start(label)
    engine.receive(label, .2)
    assert engine.tick(.2, (0., 0., 0.), clearance=.2, around=.2) == (0., 0.)
    assert engine.inhibited


def test_arc_needs_clear_surroundings_and_failed_start_does_not_repeat():
    engine = EmotionEngine()
    engine.receive('happy', 0.)
    engine.receive('happy', .1)
    assert engine.tick(.1, (0., 0., 0.), around=1.) == (0., 0.)
    engine.receive('happy', .2)
    assert engine.tick(.2, (0., 0., 0.), around=10.) == (0., 0.)
    assert engine.inhibited


def test_no_odometry_progress_times_out_even_with_a_visible_face():
    engine, _ = start('fear')
    for i in range(60):
        now = .2+i*.1
        engine.receive('fear', now)
        engine.tick(now, (0., 0., 0.))
    assert engine.inhibited and engine.action is None


@pytest.mark.parametrize('kwargs', [dict(stable_frames=True), dict(linear_speed=math.nan), dict(reverse_speed=1.), dict(arc_radius=.01)])
def test_invalid_motion_configuration(kwargs):
    with pytest.raises(ValueError):
        EmotionConfig(**kwargs)
