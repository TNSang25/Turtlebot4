# SPDX-License-Identifier: Apache-2.0
import numpy as np
import pytest

from robot_voice.audio import UtteranceBuffer

QUIET = np.zeros(800, dtype='<i2').tobytes()
VOICE = np.full(800, 2000, dtype='<i2').tobytes()


def test_silence_and_a_short_noise_click_do_not_create_commands():
    buffer = UtteranceBuffer(16000)
    assert all(buffer.feed(QUIET) is None for _ in range(100))
    assert buffer.feed(VOICE) is None
    assert all(buffer.feed(QUIET) is None for _ in range(25))


def test_sentence_waits_for_silence_and_emits_only_once():
    buffer = UtteranceBuffer(16000)
    for _ in range(10):
        buffer.feed(QUIET)
    for _ in range(15):
        assert buffer.feed(VOICE) is None
    results = [buffer.feed(QUIET) for _ in range(30)]
    audio = [row for row in results if row is not None]
    assert len(audio) == 1
    assert VOICE in audio[0]
    assert audio[0].startswith(QUIET)  # pre-roll retains the start of speech


def test_long_conversation_is_discarded_instead_of_submitting_a_partial_command():
    buffer = UtteranceBuffer(16000, maximum=1.)
    assert all(buffer.feed(VOICE) is None for _ in range(50))
    assert all(buffer.feed(QUIET) is None for _ in range(25))
    assert not buffer.dropping
    for _ in range(8):
        buffer.feed(VOICE)
    assert sum(buffer.feed(QUIET) is not None for _ in range(25)) == 1


@pytest.mark.parametrize('options', [dict(rate=0), dict(rate=16000, silence=-1.), dict(rate=16000, rms_threshold=0.)])
def test_invalid_audio_settings(options):
    with pytest.raises(ValueError):
        UtteranceBuffer(**options)
