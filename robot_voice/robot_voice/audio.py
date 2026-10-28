# SPDX-License-Identifier: Apache-2.0
"""Bounded PCM utterances with a silence boundary and a short pre-roll."""

from collections import deque
import math
import numpy as np


class UtteranceBuffer:
    def __init__(self, rate, rms_threshold=.008, silence=.8, minimum=.3, maximum=12.):
        if (rate <= 0 or not all(math.isfinite(v) and v > 0 for v in (rms_threshold, silence, minimum, maximum))
                or minimum >= maximum):
            raise ValueError('Invalid audio segmentation settings')
        self.rate, self.threshold = rate, rms_threshold
        self.silence, self.minimum, self.maximum = silence, minimum, maximum
        self.reset()

    def reset(self):
        self.pre = deque(maxlen=5)
        self.frames = []
        self.quiet = self.voiced = self.duration = 0.
        self.dropping = False

    def feed(self, pcm):
        samples = np.frombuffer(pcm, dtype='<i2').astype(np.float32)/32768.
        if not samples.size:
            return None
        seconds = samples.size/self.rate
        voiced = float(np.sqrt(np.mean(samples*samples))) >= self.threshold
        if self.dropping:
            self.quiet = 0. if voiced else self.quiet+seconds
            if self.quiet >= self.silence:
                self.reset()
            return None
        if not self.frames:
            self.pre.append(pcm)
            if not voiced:
                return None
            self.frames = list(self.pre)
            self.duration = sum(len(frame) for frame in self.frames)/(2*self.rate)
        else:
            self.frames.append(pcm)
            self.duration += seconds
        self.voiced += seconds if voiced else 0.
        self.quiet = 0. if voiced else self.quiet+seconds
        if self.duration-self.quiet >= self.maximum:
            # Do not submit a truncated long conversation as a command.
            self.frames = []
            self.dropping = True
            return None
        if self.quiet >= self.silence:
            result = b''.join(self.frames) if self.voiced >= self.minimum else None
            self.reset()
            return result
        return None
