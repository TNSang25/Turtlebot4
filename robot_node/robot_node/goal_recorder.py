# SPDX-License-Identifier: Apache-2.0
"""Record map points from RViz or the current localized pose into a goal YAML."""

import math
from pathlib import Path

from geometry_msgs.msg import PointStamped
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
import yaml

from .goals import yaw_from_quaternion
from .runtime import stop_on_signals


class GoalRecorder(Node):
    def __init__(self):
        super().__init__('goal_recorder')
        self.declare_parameter('output_file', 'src/robot_navigation/maps/goals.yaml')
        self.declare_parameter('clicked_yaw', 0.0)
        self.path = Path(self.get_parameter('output_file').value).expanduser().resolve()
        self.data = {'frame_id': 'map', 'goals': []}
        if self.path.exists():
            self.data = yaml.safe_load(self.path.read_text())
            if not isinstance(self.data, dict) or self.data.get('frame_id') != 'map' or not isinstance(self.data.get('goals'), list):
                raise ValueError('Existing output file must contain frame_id: map and a goals list')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.subscription = self.create_subscription(PointStamped, '/clicked_point', self.point, 10)
        self.service = self.create_service(Trigger, '/record_current_pose', self.current)
        self.get_logger().info(f'Append goals to {self.path}. RViz Publish Point uses clicked_yaw (radians).')

    def append(self, x, y, yaw):
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            raise ValueError('Coordinates and yaw must be finite')
        row = {'name': f'Goal {len(self.data["goals"]) + 1}',
               'x': round(x, 4), 'y': round(y, 4), 'yaw': round(yaw, 4)}
        self.data['goals'].append(row)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix('.yaml.tmp')
        try:
            temporary.write_text(yaml.safe_dump(self.data, sort_keys=False, allow_unicode=True))
            temporary.replace(self.path)
        except Exception:
            self.data['goals'].pop()
            raise
        self.get_logger().info(f'SAVED {row}')

    def point(self, message):
        if message.header.frame_id != 'map':
            self.get_logger().error('Publish Point must use Fixed Frame = map')
            return
        try:
            self.append(message.point.x, message.point.y, self.get_parameter('clicked_yaw').value)
        except Exception as error:
            self.get_logger().error(str(error))

    def current(self, request, response):
        try:
            tf = self.tf.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.append(tf.transform.translation.x, tf.transform.translation.y,
                        yaw_from_quaternion(tf.transform.rotation))
            response.success = True
            response.message = f'Saved goal {len(self.data["goals"])} to {self.path}'
        except Exception as error:
            response.success = False
            response.message = str(error)
        return response


def main(args=None):
    with stop_on_signals():
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
        node = None
        try:
            node = GoalRecorder()
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            if node is not None:
                node.destroy_node()
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
