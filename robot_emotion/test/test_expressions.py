# SPDX-License-Identifier: Apache-2.0
import math
import pytest
from robot_emotion.expressions import ExpressionFilter, LABELS


def face(label, score=.9, box=(100, 100, 120, 120)):
    scores = {k: (1-score)/6 for k in LABELS}
    scores[label] = score
    return dict(box=list(box), emotions=scores)


@pytest.mark.parametrize('label', ['happy', 'sad', 'angry', 'surprise', 'fear'])
def test_stable_expressions_and_neutral_stop_without_waiting_for_smoothing(label):
    f = ExpressionFilter()
    assert [f.update([face(label)], i*.1)[0] for i in range(3)] == ['uncertain', 'uncertain', label]
    assert f.update([face('neutral')], .31)[0] == 'neutral'


def test_no_face_multiple_faces_disgust_and_invalid_scores_cannot_command_motion():
    f = ExpressionFilter(stable_frames=1)
    assert f.update([face('happy')], 0)[0] == 'happy'
    assert f.update([], .1)[0] == 'no_face'
    assert f.update([face('happy'), face('sad')], .2)[0] == 'multiple_faces'
    assert f.update([face('disgust')], .3)[0] == 'uncertain'
    invalid = face('happy')
    invalid['emotions']['sad'] = math.nan
    assert f.update([invalid], .4)[0] == 'uncertain'


def test_one_noisy_frame_and_ambiguous_scores_never_produce_motion():
    f = ExpressionFilter()
    for i in range(30):
        assert f.update([face('happy' if i%2 else 'sad')], i*.08)[0] == 'uncertain'
    ambiguous = face('happy', .5)
    ambiguous['emotions'] = {k: 0. for k in LABELS}
    ambiguous['emotions'].update(happy=.5, sad=.48, neutral=.02)
    for i in range(10):
        assert f.update([ambiguous], 3+i*.1)[0] == 'uncertain'


def test_frames_separated_by_dropout_or_new_face_position_do_not_accumulate():
    f = ExpressionFilter()
    assert f.update([face('happy')], 0)[0] == 'uncertain'
    assert f.update([face('happy')], .1)[0] == 'uncertain'
    assert f.update([face('happy')], 2)[0] == 'uncertain'
    assert f.update([face('happy', box=(400, 100, 120, 120))], 2.1)[0] == 'uncertain'
    assert f.update([face('happy', box=(400, 100, 120, 120))], 2.2)[0] == 'uncertain'
    assert f.update([face('happy', box=(400, 100, 120, 120))], 2.3)[0] == 'happy'


@pytest.mark.parametrize('kwargs', [dict(confidence=math.nan), dict(smoothing=0), dict(stable_frames=True), dict(max_gap=-1)])
def test_invalid_filter_settings(kwargs):
    with pytest.raises(ValueError):
        ExpressionFilter(**kwargs)
