# SPDX-License-Identifier: Apache-2.0
"""Validate FER observations, smooth scores and debounce expression changes."""
import math

LABELS = ('angry', 'disgust', 'fear', 'happy', 'sad', 'surprise', 'neutral')


class ExpressionFilter:
    def __init__(self, confidence=.5, score_margin=.08, smoothing=.5, stable_frames=3, max_gap=.5):
        if (not all(math.isfinite(v) for v in (confidence, score_margin, smoothing, max_gap))
                or not 0 < confidence <= 1 or not 0 <= score_margin < 1
                or not 0 < smoothing <= 1 or max_gap <= 0
                or type(stable_frames) is not int or stable_frames < 1):
            raise ValueError('Invalid recognition filter settings')
        self.confidence, self.margin, self.smoothing = confidence, score_margin, smoothing
        self.stable_frames, self.max_gap = stable_frames, max_gap
        self.reset()

    def reset(self):
        self.scores = self.candidate = self.last_seen = self.box = None
        self.count = 0

    def update(self, detections, now):
        if not isinstance(detections, list) or len(detections) != 1:
            self.reset()
            return ('no_face' if not detections else 'multiple_faces'), 0., 'none'
        try:
            observation = detections[0]
            scores = {k: float(observation['emotions'][k]) for k in LABELS}
            box = tuple(float(v) for v in observation['box'])
            if (len(box) != 4 or not all(math.isfinite(v) for v in box) or min(box[2:]) <= 0
                    or not all(math.isfinite(v) and 0 <= v <= 1 for v in scores.values())
                    or not .8 <= sum(scores.values()) <= 1.2):
                raise ValueError('Malformed FER output')
        except (ValueError, TypeError, KeyError, OverflowError):
            self.reset()
            return 'uncertain', 0., 'invalid'
        if self.last_seen is not None and now-self.last_seen > self.max_gap:
            self.reset()
        if self.box is not None:
            x, y, w, h = box
            px, py, pw, ph = self.box
            overlap = max(0., min(x+w, px+pw)-max(x, px))*max(0., min(y+h, py+ph)-max(y, py))
            if overlap/(w*h+pw*ph-overlap) < .15:
                self.reset()  # Do not combine scores across different face positions.
        self.box, self.last_seen = box, now
        order = sorted(scores, key=scores.get, reverse=True)
        raw, raw_confidence = order[0], scores[order[0]]
        # A confident stop never waits for temporal smoothing/debouncing.
        if raw == 'neutral' and raw_confidence >= self.confidence and raw_confidence-scores[order[1]] >= self.margin:
            self.scores, self.candidate, self.count = scores, 'neutral', 0
            return 'neutral', raw_confidence, raw
        if raw == 'disgust':
            self.scores, self.candidate, self.count = None, None, 0
            return 'uncertain', raw_confidence, raw
        self.scores = (scores if self.scores is None else
                       {k: self.smoothing*scores[k]+(1-self.smoothing)*self.scores[k] for k in LABELS})
        ranked = sorted(self.scores, key=self.scores.get, reverse=True)
        label, value = ranked[0], self.scores[ranked[0]]
        if (label != raw or value < self.confidence or value-self.scores[ranked[1]] < self.margin
                or label == 'disgust'):
            self.candidate, self.count = None, 0
            return 'uncertain', raw_confidence, raw
        self.count = self.count+1 if self.candidate == label else 1
        self.candidate = label
        return (label if self.count >= self.stable_frames else 'uncertain'), value, raw
