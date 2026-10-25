# SPDX-License-Identifier: Apache-2.0
"""Gesture state machine and odometry-based motion primitives, independent of ROS."""

from dataclasses import dataclass
import math


CHOICES = {'Wave': 'greeting', 'Pointing_Up': 'back_forth', 'Victory': 'circle'}
VALID = set(CHOICES) | {'Thumb_Up', 'Open_Palm', 'Closed_Fist'}


@dataclass(frozen=True)
class BehaviorConfig:
    stable_time: float = 0.25
    signal_timeout: float = 0.75
    hand_timeout: float = 0.40
    linear_speed: float = 0.12
    angular_speed: float = 0.5
    leg_distance: float = 0.35
    circle_radius: float = 0.60
    greeting_angle: float = 0.45
    progress_timeout: float = 5.0
    max_duration: float = 120.0
    stop_distance: float = 0.40
    rotation_clearance: float = 0.30

    def __post_init__(self):
        for name, value in vars(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if self.linear_speed > 0.26 or self.angular_speed > 1.0:
            raise ValueError('Speeds exceed the supported simulation limits')
        if self.linear_speed / self.circle_radius > 1.0:
            raise ValueError('Circle requires angular velocity above 1 rad/s')


def scan_distances(scan, linear, sensor_yaw=0.0, sensor_x=0.0, sensor_y=0.0):
    """Return (motion-sector clearance, full-circle clearance), or reject bad scans."""
    if (len(scan.ranges) < 180 or not math.isfinite(scan.angle_increment)
            or scan.angle_increment <= 0 or not math.isfinite(scan.angle_min)
            or not math.isfinite(scan.range_max) or scan.range_max <= 0
            or scan.angle_increment * (len(scan.ranges) - 1) < 6.0):
        raise ValueError('A valid 360-degree scan is required')
    all_ranges, sector = [], []
    for i, distance in enumerate(scan.ranges):
        if math.isnan(distance) or (math.isfinite(distance) and distance < 0):
            continue
        # REP-117: -inf is a return below the sensor's minimum range, not free space.
        distance = 0.0 if distance == float('-inf') else min(distance, scan.range_max)
        if distance < scan.range_min:
            distance = 0.0  # Too close to measure safely.
        angle = scan.angle_min + i * scan.angle_increment + sensor_yaw
        x = sensor_x + distance*math.cos(angle)
        y = sensor_y + distance*math.sin(angle)
        distance = math.hypot(x, y)
        all_ranges.append(distance)
        cosine = x/distance if distance > 0 else 0.0
        if linear == 0 or (linear > 0 and cosine > -0.1) or (linear < 0 and cosine < 0.1):
            sector.append(distance)
    if len(all_ranges) < 0.9 * len(scan.ranges) or not sector:
        raise ValueError('Insufficient valid laser returns')
    return min(sector), min(all_ranges)


class BehaviorEngine:
    def __init__(self, config=None):
        self.config = config or BehaviorConfig()
        self.state, self.reason = 'idle', 'Select a gesture, then confirm with Thumb_Up'
        self.selected = self.action = None
        self.last_message = self.last_valid = None
        self.candidate = self.committed = None
        self.candidate_since = 0.0
        self.start_requested = False
        self.phase = 0
        self.phases = []
        self.rotation = 0.0

    @property
    def active(self):
        return self.action is not None

    def stop(self, state, reason, clear_selection=True):
        self.action = None
        self.start_requested = False
        self.state, self.reason = state, reason
        if clear_selection:
            self.selected = None

    def receive(self, label, now):
        self.last_message = now
        if label not in VALID:
            self.candidate = None
            return
        self.last_valid = now
        if label == 'Open_Palm':
            # Keep a pending selection so the palm at the end of a Wave does
            # not erase it. An interrupted running motion must be selected again.
            self.stop('stopped', 'Open_Palm: stopped', clear_selection=self.active)
            self.candidate = self.committed = label
            return
        if label == 'Closed_Fist':
            self.stop('idle', 'Closed_Fist: canceled')
            self.candidate = self.committed = label
            return
        if label != self.candidate:
            self.candidate, self.candidate_since = label, now
        if now - self.candidate_since < self.config.stable_time or label == self.committed:
            return
        self.committed = label
        if self.active:
            return
        if label in CHOICES:
            self.selected = CHOICES[label]
            self.state, self.reason = 'selected', 'Confirm with Thumb_Up'
        elif label == 'Thumb_Up' and self.selected:
            self.start_requested = True

    def _phase_start(self, pose, now):
        self.origin = pose
        self.last_yaw = pose[2]
        self.rotation = self.best_progress = 0.0
        self.last_progress = now

    def next_command(self):
        return self.phases[self.phase][:2] if self.active else (0.0, 0.0)

    def tick(self, now, pose, sensor_error='', clearance=float('inf'), around=float('inf')):
        c = self.config
        if (self.last_message is None or self.last_valid is None
                or now - self.last_message > c.signal_timeout
                or now - self.last_valid > c.hand_timeout):
            self.stop('signal_lost', 'Hand/control signal lost: select and confirm again')
            self.candidate = self.committed = None
            # A held confirmation must never restart a canceled behavior.
            return 0.0, 0.0
        if self.start_requested:
            if sensor_error or pose is None:
                self.stop('blocked', sensor_error or 'No odometry')
                return 0.0, 0.0
            if self.selected == 'circle' and around < 2*c.circle_radius + c.stop_distance:
                self.stop('blocked', 'Not enough clear space for a full circle')
                return 0.0, 0.0
            self.action, self.selected = self.selected, None
            self.start_requested = False
            if self.action == 'back_forth':
                self.phases = [(sign*c.linear_speed, 0.0, c.leg_distance)
                               for _ in range(4) for sign in (1, -1)]
            elif self.action == 'circle':
                self.phases = [(c.linear_speed, c.linear_speed/c.circle_radius, 2*math.pi)]
            else:
                self.phases = [(0.0, sign*c.angular_speed, angle*c.greeting_angle)
                               for sign, angle in ((1, 1), (-1, 2), (1, 1))]
            self.phase = 0
            self.started = now
            self._phase_start(pose, now)
            self.state, self.reason = 'running', self.action
        if not self.active:
            return 0.0, 0.0
        if sensor_error or pose is None:
            self.stop('blocked', sensor_error or 'No odometry')
            return 0.0, 0.0
        linear, angular, target = self.phases[self.phase]
        if around < c.rotation_clearance or (linear != 0 and clearance < c.stop_distance):
            self.stop('blocked', 'Obstacle: select and confirm again after clearing the area')
            return 0.0, 0.0
        delta = math.atan2(math.sin(pose[2]-self.last_yaw), math.cos(pose[2]-self.last_yaw))
        self.rotation += delta
        self.last_yaw = pose[2]
        if self.action == 'back_forth':
            x, y, yaw = self.origin
            progress = math.copysign(1, linear)*((pose[0]-x)*math.cos(yaw)+(pose[1]-y)*math.sin(yaw))
            tolerance = 0.015
        else:
            progress = math.copysign(1, angular)*self.rotation
            tolerance = 0.03
        if progress >= target - tolerance:
            self.phase += 1
            if self.phase == len(self.phases):
                self.stop('completed', 'Behavior completed; select a new behavior')
            else:
                self._phase_start(pose, now)
            return 0.0, 0.0
        if progress > self.best_progress + 0.005:
            self.best_progress, self.last_progress = progress, now
        if now - self.last_progress > c.progress_timeout or now - self.started > c.max_duration:
            self.stop('blocked', 'Motion timeout or no odometry progress')
            return 0.0, 0.0
        return linear, angular

    def snapshot(self):
        return {'state': self.state, 'selected': self.selected, 'action': self.action,
                'phase': self.phase, 'total_phases': len(self.phases), 'reason': self.reason}
