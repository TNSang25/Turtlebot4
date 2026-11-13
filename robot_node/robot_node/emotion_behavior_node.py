# SPDX-License-Identifier: Apache-2.0
"""Map stable expressions to /cmd_vel with scan and odometry interlocks."""
import json
import math
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.msg import ParameterDescriptor
from tf2_ros import Buffer, TransformListener, TransformException

from .behavior import scan_distances
from .emotion_behavior import EmotionConfig, EmotionEngine
from .runtime import publish_stop, stop_on_signals


class EmotionBehaviorNode(Node):
    def __init__(self):
        super().__init__('emotion_behavior_node')
        defaults = {**vars(EmotionConfig()), 'sensor_timeout': .8, 'publish_rate': 20.}
        for name, value in defaults.items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        values = {name: self.get_parameter(name).value for name in defaults}
        self.engine = EmotionEngine(EmotionConfig(**{k: values[k] for k in vars(EmotionConfig())}))
        self.sensor_timeout = values['sensor_timeout']
        rate = values['publish_rate']
        if not math.isfinite(self.sensor_timeout) or self.sensor_timeout <= 0:
            raise ValueError('sensor_timeout must be positive and finite')
        if not math.isfinite(rate) or not 0 < rate <= 100:
            raise ValueError('publish_rate must be in (0, 100]')
        self.pose = self.scan = self.odom_stamp = None
        self.odom_arrival = self.scan_arrival = 0.
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.command = self.create_publisher(Twist, '/cmd_vel', 1)
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status = self.create_publisher(String, '/emotion_behavior/state', durable)
        self.emotion_sub = self.create_subscription(String, '/detected_emotion', self.receive, 1)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odometry, qos_profile_sensor_data)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.laser, qos_profile_sensor_data)
        self.timer = self.create_timer(1/rate, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.last_status, self.last_publish = None, 0.

    def receive(self, message):
        active = self.engine.action
        self.engine.receive(message.data, time.monotonic())
        if message.data.strip().lower() in ('neutral', 'no_face', 'multiple_faces') or (active and self.engine.action != active):
            self.command.publish(Twist())

    def odometry(self, message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        values = (p.x, p.y, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(v) for v in values) or abs(sum(v*v for v in values[2:])-1) > .05:
            self.pose = None
            return
        self.pose = (p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
        self.odom_stamp, self.odom_arrival = message.header.stamp, time.monotonic()

    def laser(self, message):
        self.scan, self.scan_arrival = message, time.monotonic()

    def sensor_error(self, now):
        if self.pose is None or self.scan is None:
            return 'Waiting for /odom and /scan'
        if now-self.odom_arrival > self.sensor_timeout or now-self.scan_arrival > self.sensor_timeout:
            return 'Odometry/scan stopped arriving'
        ros_now = self.get_clock().now().nanoseconds/1e9
        for stamp in (self.odom_stamp, self.scan.header.stamp):
            if not -.2 <= ros_now-(stamp.sec+stamp.nanosec/1e9) <= self.sensor_timeout:
                return 'Sensor timestamps do not match ROS clock'
        return ''

    def tick(self):
        now = time.monotonic()
        error = self.sensor_error(now)
        clearance = around = 0.
        if not error:
            try:
                if not math.isfinite(self.scan.range_min) or not 0 <= self.scan.range_min < self.scan.range_max:
                    raise ValueError('Invalid scan range limits')
                transform = self.tf.lookup_transform('base_link', self.scan.header.frame_id, rclpy.time.Time()).transform
                q = transform.rotation
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                linear, _ = self.engine.next_command()
                clearance, around = scan_distances(self.scan, linear, yaw,
                                                   transform.translation.x, transform.translation.y)
            except (ValueError, TransformException) as exception:
                error = str(exception)
        linear, angular = self.engine.tick(now, self.pose, error, clearance, around)
        command = Twist()
        command.linear.x, command.angular.z = linear, angular
        self.command.publish(command)
        payload = json.dumps(self.engine.snapshot())
        if payload != self.last_status or now-self.last_publish >= 1.:
            self.status.publish(String(data=payload))
            # Progress changes at every tick; log only state/action/reason changes.
            signature = (self.engine.state, self.engine.action, self.engine.reason)
            if signature != getattr(self, 'last_log', None):
                self.get_logger().info(payload)
                self.last_log = signature
            self.last_status, self.last_publish = payload, now


def main(args=None):
    with stop_on_signals():
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
        node, code = None, 0
        try:
            node = EmotionBehaviorNode()
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        except Exception as error:
            print(f'emotion_behavior_node: {error}')
            code = 1
        finally:
            if node is not None:
                publish_stop(node, node.command)
                node.destroy_node()
            rclpy.try_shutdown()
    return code
