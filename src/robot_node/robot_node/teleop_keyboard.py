# SPDX-License-Identifier: Apache-2.0
"""Terminal keyboard teleoperation with bounded speeds and key timeout."""

import os
import select
import sys
import termios
import time
import tty

from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from robot_node.runtime import publish_stop, stop_on_signals
from robot_node.velocity_filter import positive_finite, VelocityFilter


KEYS = {
    'w': (1.0, 0.0), 's': (-1.0, 0.0),
    'a': (0.0, 1.0), 'd': (0.0, -1.0),
    'q': (1.0, 1.0), 'e': (1.0, -1.0),
    'z': (-1.0, -1.0), 'c': (-1.0, 1.0),
}


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        defaults = {
            'output_topic': '/cmd_vel',
            'linear_speed': 0.20,
            'angular_speed': 0.80,
            'max_linear': 0.31,
            'max_angular': 1.9,
            'key_timeout': 0.6,
            'publish_rate': 20.0,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default, ParameterDescriptor(read_only=True))
        values = {name: self.get_parameter(name).value for name in defaults}
        self.filter = VelocityFilter(
            values['max_linear'], values['max_angular'], values['key_timeout'])
        self.linear_speed = min(
            positive_finite('linear_speed', values['linear_speed']), self.filter.max_linear)
        self.angular_speed = min(
            positive_finite('angular_speed', values['angular_speed']), self.filter.max_angular)
        self.period = 1.0 / positive_finite('publish_rate', values['publish_rate'])
        self.publisher = self.create_publisher(Twist, values['output_topic'], 1)

    def run(self):
        descriptor = sys.stdin.fileno()
        settings = termios.tcgetattr(descriptor)
        print(
            '\nTurtleBot 4 keyboard control (focus this terminal):\n'
            '  Lowercase w/s: forward/backward    a/d: rotate left/right\n'
            '  q/e: forward arcs       z/c: backward arcs\n'
            '  Space or other keys: stop    Ctrl-C: quit\n'
            f'Speed: {self.linear_speed:.2f} m/s, {self.angular_speed:.2f} rad/s. '
            f'Stops after {self.filter.command_timeout:.2f}s without a key.\n'
            'Hold a movement key to repeat. Do not run teleop alongside navigation.\n',
            flush=True,
        )
        try:
            tty.setcbreak(descriptor)
            next_publish = time.monotonic()
            last_command = None
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.0)
                ready, _, _ = select.select(
                    [descriptor], [], [], max(0.0, next_publish - time.monotonic()))
                if ready:
                    key_bytes = os.read(descriptor, 1)
                    key = key_bytes.decode('ascii', errors='ignore')
                    if not key_bytes or key == '\x03':
                        break
                    direction = KEYS.get(key)
                    if direction is None:
                        self.filter.stop()
                        if key != ' ':
                            self.get_logger().warning(
                                f'Unrecognized key {key_bytes!r}: stopping. '
                                'Use lowercase w/s/a/d/q/e/z/c.',
                                throttle_duration_sec=2.0,
                            )
                    else:
                        self.filter.receive(
                            (direction[0] * self.linear_speed, 0.0, 0.0),
                            (0.0, 0.0, direction[1] * self.angular_speed),
                            time.monotonic(),
                        )
                now = time.monotonic()
                if now >= next_publish:
                    message = Twist()
                    message.linear.x, message.angular.z = self.filter.sample(now)
                    self.publisher.publish(message)
                    command = (message.linear.x, message.angular.z)
                    if command != last_command:
                        self.get_logger().info(
                            f'{self.publisher.topic_name}: '
                            f'linear.x={command[0]:.2f}, angular.z={command[1]:.2f}')
                        last_command = command
                    next_publish = now + self.period
        finally:
            termios.tcsetattr(descriptor, termios.TCSADRAIN, settings)


def main(args=None):
    if not sys.stdin.isatty():
        print('teleop_keyboard needs an interactive terminal. Run: '
              'ros2 run robot_node teleop_keyboard', file=sys.stderr)
        return 1
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    with stop_on_signals():
        try:
            node = TeleopKeyboard()
            node.run()
        except KeyboardInterrupt:
            pass
        finally:
            if node is not None:
                publish_stop(node, node.publisher)
                node.destroy_node()
            rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
