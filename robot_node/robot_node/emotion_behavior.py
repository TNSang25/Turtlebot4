# SPDX-License-Identifier: Apache-2.0
"""Odometry-based, finite emotion motions; independent of ROS and inference."""
from dataclasses import dataclass
import math

EMOTIONS = {'happy', 'sad', 'neutral', 'angry', 'surprise', 'fear'}


@dataclass(frozen=True)
class EmotionConfig:
    stable_frames: int = 2
    signal_timeout: float = .8
    uncertain_timeout: float = .5
    linear_speed: float = .15
    reverse_speed: float = .10
    angular_speed: float = .60
    arc_radius: float = .60
    arc_angle: float = math.pi/2
    step_distance: float = .25
    stop_distance: float = .40
    rotation_clearance: float = .30
    progress_timeout: float = 5.
    max_duration: float = 45.

    def __post_init__(self):
        if type(self.stable_frames) is not int or self.stable_frames < 1:
            raise ValueError('stable_frames must be a positive integer')
        for name, value in vars(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if max(self.linear_speed, self.reverse_speed) > .5 or self.angular_speed > 1.2:
            raise ValueError('Velocity exceeds simulation limits')
        if self.linear_speed/self.arc_radius > 1.2:
            raise ValueError('Arc requires angular velocity above 1.2 rad/s')


class EmotionEngine:
    def __init__(self, config=None):
        self.config = config or EmotionConfig()
        self.state, self.reason = 'idle', 'Waiting for a stable expression'
        self.action = self.pending = self.latched = None
        self.candidate, self.count = None, 0
        self.last_message = self.last_valid = None
        self.inhibited = False
        self.progress = self.target = 0.

    def halt(self, state, reason, inhibit=False):
        self.action = self.pending = None
        self.state, self.reason = state, reason
        self.inhibited |= inhibit

    def receive(self, label, now):
        label = label.strip().lower()
        self.last_message = now
        if label == 'neutral':
            self.last_valid = now
            self.halt('stopped', 'Neutral: stopped; ready for a new expression')
            self.inhibited = False
            self.latched = self.candidate = None
            self.count = 0
            return
        if label in ('no_face', 'multiple_faces'):
            need_reset = self.action is not None or self.pending is not None or self.inhibited
            self.halt(label, 'Use one visible face; show Neutral before restarting'
                      if need_reset else 'Waiting for one visible face', inhibit=need_reset)
            self.candidate, self.count = None, 0
            return
        if label not in EMOTIONS:
            self.candidate, self.count = None, 0
            return
        self.last_valid = now
        if label != self.candidate:
            self.candidate, self.count = label, 1
        else:
            self.count += 1
        if self.inhibited or self.count < self.config.stable_frames or label == self.latched:
            return
        if self.action:
            self.halt('switching', 'Changing expression; previous motion stopped')
        self.pending = label

    def command_for(self, label):
        c = self.config
        return {
            'happy': (c.linear_speed, c.linear_speed/c.arc_radius),
            'sad': (c.linear_speed, -c.linear_speed/c.arc_radius),
            'angry': (-c.reverse_speed, 0.), 'fear': (c.linear_speed, 0.),
            'surprise': (0., c.angular_speed),
        }.get(label, (0., 0.))

    def next_command(self):
        return self.command_for(self.action or self.pending)

    def tick(self, now, pose, sensor_error='', clearance=math.inf, around=math.inf):
        c = self.config
        fresh = (self.last_message is not None and self.last_valid is not None
                 and now-self.last_message <= c.signal_timeout
                 and now-self.last_valid <= c.uncertain_timeout)
        if not fresh:
            if self.action:
                self.halt('signal_lost', 'Face/control signal lost; show Neutral to re-arm', True)
            self.pending = None
            self.candidate, self.count = None, 0
            return 0., 0.
        if self.pending and not self.inhibited:
            if sensor_error or pose is None:
                self.state, self.reason = 'waiting_for_sensors', sensor_error or 'Waiting for odometry'
                return 0., 0.
            label = self.pending
            self.latched = label
            linear, _ = self.command_for(label)
            if (around < c.rotation_clearance or (linear != 0 and clearance < c.stop_distance)
                    or (label in ('happy', 'sad') and around < 2*c.arc_radius+c.stop_distance)):
                self.halt('blocked', 'Obstacle: move to a clear area, then show Neutral', True)
                return 0., 0.
            self.action, self.pending = label, None
            self.origin, self.last_yaw = pose, pose[2]
            self.rotation = self.progress = self.best_progress = 0.
            self.started = self.last_progress = now
            self.target = (2*math.pi if label == 'surprise' else c.arc_angle
                           if label in ('happy', 'sad') else c.step_distance)
            self.state, self.reason = 'running', label
        if self.action is None:
            return 0., 0.
        if sensor_error or pose is None:
            self.halt('blocked', (sensor_error or 'No odometry')+'; show Neutral to re-arm', True)
            return 0., 0.
        linear, angular = self.next_command()
        if around < c.rotation_clearance or (linear != 0 and clearance < c.stop_distance):
            self.halt('blocked', 'Obstacle detected; show Neutral after clearing the area', True)
            return 0., 0.
        self.rotation += math.atan2(math.sin(pose[2]-self.last_yaw), math.cos(pose[2]-self.last_yaw))
        self.last_yaw = pose[2]
        if angular:
            self.progress = math.copysign(1., angular)*self.rotation
            tolerance = .03
        else:
            x, y, yaw = self.origin
            self.progress = math.copysign(1., linear)*((pose[0]-x)*math.cos(yaw)+(pose[1]-y)*math.sin(yaw))
            tolerance = .015
        if self.progress >= self.target-tolerance:
            self.halt('completed', 'Motion completed; show Neutral to repeat this expression')
            return 0., 0.
        if self.progress > self.best_progress+.003:
            self.best_progress, self.last_progress = self.progress, now
        if now-self.last_progress > c.progress_timeout or now-self.started > c.max_duration:
            self.halt('blocked', 'No odometry progress or motion timeout; show Neutral', True)
            return 0., 0.
        return linear, angular

    def snapshot(self):
        return dict(state=self.state, reason=self.reason, action=self.action,
                    last_emotion=self.latched, inhibited=self.inhibited,
                    progress=round(self.progress, 3), target=round(self.target, 3))
