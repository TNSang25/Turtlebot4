# SPDX-License-Identifier: Apache-2.0
"""Execute confirmed gestures with scan/odometry interlocks and wall-time watchdogs."""

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

from .behavior import BehaviorConfig, BehaviorEngine, scan_distances
from .runtime import publish_stop, stop_on_signals


class RobotBehaviorNode(Node):
    def __init__(self):
        super().__init__('robot_behavior_node')
        defaults = {**vars(BehaviorConfig()), 'sensor_timeout': 0.8, 'publish_rate': 20.0}
        for name, value in defaults.items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        values = {name: self.get_parameter(name).value for name in defaults}
        self.engine = BehaviorEngine(BehaviorConfig(**{name: values[name] for name in vars(BehaviorConfig())}))
        self.sensor_timeout = values['sensor_timeout']
        if not math.isfinite(self.sensor_timeout) or self.sensor_timeout <= 0:
            raise ValueError('sensor_timeout must be positive and finite')
        rate = values['publish_rate']
        if not math.isfinite(rate) or rate <= 0 or rate > 100:
            raise ValueError('publish_rate must be in (0, 100]')
        self.pose = self.scan = None
        self.odom_arrival = self.scan_arrival = 0.0
        self.odom_stamp = None
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.command = self.create_publisher(Twist, '/cmd_vel', 1)
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, '/robot_behavior/state', durable)
        # Volatile depth=1 prevents stale gesture commands on process startup.
        self.gesture_sub = self.create_subscription(String, '/detected_gesture', self.gesture, 1)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odometry, qos_profile_sensor_data)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.laser, qos_profile_sensor_data)
        self.timer = self.create_timer(1.0/rate, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.last_state, self.last_state_time = None, 0.0

    def gesture(self, message):
        active = self.engine.active
        self.engine.receive(message.data, time.monotonic())
        if message.data in ('Open_Palm', 'Closed_Fist') or (active and not self.engine.active):
            self.command.publish(Twist())  # Safety commands bypass the regular timer.

    def odometry(self, message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        values = (p.x, p.y, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(value) for value in values):
            self.pose = None
            return
        norm = sum(value*value for value in values[2:])
        if abs(norm-1) > 0.05:
            self.pose = None
            return
        self.pose = (p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
        self.odom_arrival = time.monotonic()
        self.odom_stamp = message.header.stamp

    def laser(self, message):
        self.scan, self.scan_arrival = message, time.monotonic()

    def sensor_error(self, now):
        if self.pose is None or self.scan is None:
            return 'Waiting for /odom and /scan'
        if now-self.odom_arrival > self.sensor_timeout or now-self.scan_arrival > self.sensor_timeout:
            return 'Odometry/scan stopped arriving'
        ros_now = self.get_clock().now().nanoseconds / 1e9
        for stamp in (self.odom_stamp, self.scan.header.stamp):
            age = ros_now - (stamp.sec + stamp.nanosec / 1e9)
            if age < -0.2 or age > self.sensor_timeout:
                return 'Sensor timestamps do not match the ROS clock'
        return ''

    def tick(self):
        now = time.monotonic()
        error = self.sensor_error(now)
        clearance = around = 0.0
        if not error:
            try:
                linear, _ = self.engine.next_command()
                # First leg of either translational behavior starts forward.
                if self.engine.start_requested and self.engine.selected != 'greeting':
                    linear = self.engine.config.linear_speed
                transform = self.tf.lookup_transform('base_link', self.scan.header.frame_id,
                                                     rclpy.time.Time()).transform
                q = transform.rotation
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                clearance, around = scan_distances(self.scan, linear, yaw,
                                                   transform.translation.x, transform.translation.y)
            except (ValueError, TransformException) as exception:
                error = str(exception)
        linear, angular = self.engine.tick(now, self.pose, error, clearance, around)
        command = Twist()
        command.linear.x, command.angular.z = linear, angular
        self.command.publish(command)
        state = json.dumps(self.engine.snapshot())
        if state != self.last_state or now-self.last_state_time >= 1:
            self.state_pub.publish(String(data=state))
            if state != self.last_state:
                self.get_logger().info(state)
            self.last_state, self.last_state_time = state, now


def main(args=None):
    with stop_on_signals():
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
        node = None
        code = 0
        try:
            node = RobotBehaviorNode()
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        except Exception as error:
            print(f'robot_behavior_node: {error}')
            code = 1
        finally:
            if node is not None:
                publish_stop(node, node.command)
                node.destroy_node()
            rclpy.try_shutdown()
    return code
