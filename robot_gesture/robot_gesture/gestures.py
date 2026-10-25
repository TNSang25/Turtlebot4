# SPDX-License-Identifier: Apache-2.0
"""Debounce static gestures and detect a Wave from a moving open palm."""

import math

STATIC = {'Open_Palm', 'Closed_Fist', 'Pointing_Up', 'Victory', 'Thumb_Up'}


def landmark_pose(points):
    """Conservative fallback using 21 MediaPipe *world* landmarks.

    Joint angles and wrist distances work for either hand and tilted palms.
    This is a geometric rule, not an additional model confidence score.
    Confirmation (Thumb_Up) always comes from the trained classifier.
    """
    if len(points) != 21:
        return None
    xyz = [(p.x, p.y, p.z) for p in points]
    if not all(math.isfinite(v) for point in xyz for v in point):
        return None

    def distance(a, b):
        return math.dist(xyz[a], xyz[b])

    def angle(a, b, c):
        u = [x-y for x, y in zip(xyz[a], xyz[b])]
        v = [x-y for x, y in zip(xyz[c], xyz[b])]
        norm = math.sqrt(sum(x*x for x in u)*sum(x*x for x in v))
        if norm < 1e-12:
            return 0.0
        return math.degrees(math.acos(max(-1., min(1., sum(x*y for x, y in zip(u, v))/norm))))

    palm = distance(0, 9)
    if palm < 1e-6 or not .25 < distance(5, 17)/palm < 2:
        return None
    fingers = []
    for mcp in (5, 9, 13, 17):
        pip_angle = angle(mcp, mcp+1, mcp+2)
        dip_angle = angle(mcp+1, mcp+2, mcp+3)
        reach = (distance(0, mcp+3)-distance(0, mcp+1))/palm
        if pip_angle >= 140 and dip_angle >= 135 and reach > .25:
            fingers.append('open')
        elif reach < .10 and (pip_angle < 140 or dip_angle < 145):
            fingers.append('closed')
        else:
            fingers.append('uncertain')
    if fingers == ['open']*4:
        return 'Open_Palm'
    if fingers == ['open', 'closed', 'closed', 'closed'] and distance(4, 5)/palm < .75:
        return 'Pointing_Up'
    if fingers == ['open', 'open', 'closed', 'closed']:
        return 'Victory'
    # An extended thumb must never be converted from a weak Thumb_Up to Fist.
    if fingers == ['closed']*4 and distance(4, 5)/palm < .65 and distance(4, 0)/palm < 1.2:
        return 'Closed_Fist'
    return None


def gesture_evidence(raw, score, pose, confidence=.65, stop_confidence=.5):
    """Prefer confident model output, then a supported geometric fallback."""
    if not math.isfinite(score):
        return 'Unknown', 'invalid score'
    threshold = stop_confidence if raw in ('Open_Palm', 'Closed_Fist') else confidence
    if raw in STATIC and score >= threshold:
        return raw, 'MediaPipe'
    if pose in STATIC-{'Thumb_Up'}:
        return pose, 'landmarks'
    return ('No_Hand' if raw == 'No_Hand' else 'Unknown'), 'no matching pose'


class WaveDetector:
    def __init__(self, amplitude=0.04, window=2.5, hold=1.0):
        self.amplitude, self.window, self.hold = amplitude, window, hold
        self.reset()

    def reset(self):
        self.anchor = None
        self.direction = self.reversals = 0
        self.started = self.last_time = self.hold_until = 0.0

    def update(self, x, now):
        if not math.isfinite(x) or not 0 <= x <= 1:
            self.reset()
            return False
        if now < self.hold_until:
            return True
        if self.anchor is None or now-self.started > self.window or now-self.last_time > .4:
            self.anchor, self.started = x, now
            self.direction = self.reversals = 0
        self.last_time = now
        delta = x-self.anchor
        if self.direction == 0 and abs(delta) >= self.amplitude:
            self.direction = 1 if delta > 0 else -1
            self.anchor = x
        elif self.direction > 0:
            if x > self.anchor:
                self.anchor = x
            elif delta <= -self.amplitude:
                self.direction, self.anchor = -1, x
                self.reversals += 1
        elif self.direction < 0:
            if x < self.anchor:
                self.anchor = x
            elif delta >= self.amplitude:
                self.direction, self.anchor = 1, x
                self.reversals += 1
        if self.reversals >= 2:
            self.hold_until = now+self.hold
            self.anchor = None
            self.direction = self.reversals = 0
            return True
        return False


class GestureFilter:
    def __init__(self, confidence=.65, stop_confidence=.5, stable_time=.15,
                 wave_amplitude=.04, wave_window=2.5):
        for name, value in [('confidence', confidence), ('stop_confidence', stop_confidence)]:
            if not math.isfinite(value) or not 0 < value <= 1:
                raise ValueError(f'{name} must be in (0, 1]')
        if stop_confidence > confidence:
            raise ValueError('stop_confidence must not exceed confidence')
        if not math.isfinite(stable_time) or stable_time < 0:
            raise ValueError('stable_time must be nonnegative')
        if not 0 < wave_amplitude < .5 or not math.isfinite(wave_window) or wave_window <= 0:
            raise ValueError('Invalid Wave thresholds')
        self.confidence, self.stop_confidence, self.stable_time = confidence, stop_confidence, stable_time
        self.wave = WaveDetector(wave_amplitude, wave_window)
        self.candidate, self.since, self.hand = None, 0.0, None
        self.last_supported = -math.inf
        self.source = ''

    def update(self, raw, score, x, now, hand=None, pose=None):
        raw, self.source = gesture_evidence(raw, score, pose, self.confidence, self.stop_confidence)
        if now-self.last_supported > .30:
            self.candidate = None
        # Handedness can flip while waving. Track motion of the single visible
        # hand; absence, multiple hands and other gestures explicitly reset it.
        self.hand = hand
        if raw in ('Unknown', 'No_Hand'):
            if raw == 'No_Hand' or now-self.last_supported > .30:
                self.candidate = None
            if raw == 'No_Hand' or now-self.wave.last_time > .4:
                self.wave.reset()
            # Never replay a previous command during uncertain frames.
            return raw
        self.last_supported = now
        if raw == 'Closed_Fist':
            self.wave.reset()
            self.candidate = None
            return raw
        if raw == 'Open_Palm':
            if not self.wave.update(x, now):
                self.candidate = None
                return 'Open_Palm'
            raw = 'Wave'
        else:
            self.wave.reset()
        if raw != self.candidate:
            self.candidate, self.since = raw, now
        return raw if now-self.since >= self.stable_time else 'Unknown'
