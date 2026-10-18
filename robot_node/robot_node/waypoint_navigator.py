# SPDX-License-Identifier: Apache-2.0
"""Visit surveyed poses sequentially using Nav2's NavigateToPose action."""

import json
import math
from pathlib import Path
import time

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from .goals import check_goal_clearance, read_goals
from .runtime import publish_stop, stop_on_signals


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        default = str(Path(get_package_share_directory('robot_node')) / 'config/goals.yaml')
        for name, value in [('goals_file', default), ('startup_timeout', 120.0),
                            ('goal_timeout', 300.0), ('pause_at_goal', 2.0),
                            ('goal_clearance', 0.22), ('result_file', '')]:
            self.declare_parameter(name, value)
        self.goals = read_goals(self.get_parameter('goals_file').value)
        for name in ('startup_timeout', 'goal_timeout', 'goal_clearance'):
            value = self.get_parameter(name).value
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be positive and finite')
        pause = self.get_parameter('pause_at_goal').value
        if not math.isfinite(pause) or pause < 0:
            raise ValueError('pause_at_goal must be nonnegative and finite')
        self.map = None
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self._map, durable)
        self.status_pub = self.create_publisher(String, '/mission/status', durable)
        self.stop_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.action = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.lifecycle_clients = {name: self.create_client(GetState, f'/{name}/get_state')
                        for name in ('bt_navigator', 'planner_server', 'controller_server')}
        self.goal_handle = None
        self.pending_goal = None
        self.feedback_time = 0.0
        self.results = []

    def _map(self, message):
        self.map = message

    def wait(self, predicate, timeout, label):
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return
        raise TimeoutError(f'Timed out waiting for {label}')

    def status(self, state, **extra):
        payload = {'state': state, 'completed': sum(row['status'] == GoalStatus.STATUS_SUCCEEDED for row in self.results), 'total': len(self.goals), **extra}
        self.status_pub.publish(String(data=json.dumps(payload)))

    def ready(self):
        if self.map is None or not self.action.server_is_ready():
            return False
        if self.get_clock().now().nanoseconds <= 0:
            return False
        if not self.tf.can_transform('map', 'base_link', rclpy.time.Time()):
            return False
        transform = self.tf.lookup_transform('map', 'base_link', rclpy.time.Time())
        stamp = rclpy.time.Time.from_msg(transform.header.stamp, clock_type=self.get_clock().clock_type)
        if abs((self.get_clock().now() - stamp).nanoseconds) > 3e9:
            return False
        return all(client.service_is_ready() for client in self.lifecycle_clients.values())

    def wait_for_navigation(self):
        timeout = self.get_parameter('startup_timeout').value
        deadline = time.monotonic() + timeout
        self.status('waiting_for_navigation')
        self.get_logger().info('Waiting for /map, map -> base_link, and active Nav2. Set initial pose in RViz.')
        while time.monotonic() < deadline:
            self.wait(self.ready, max(0.1, deadline - time.monotonic()), 'localization and Nav2')
            pending = [client.call_async(GetState.Request()) for client in self.lifecycle_clients.values()]
            self.wait(lambda: all(f.done() for f in pending), min(5.0, max(0.1, deadline - time.monotonic())), 'lifecycle services')
            if all(f.result() is not None and f.result().current_state.id == 3 for f in pending):
                return
            rclpy.spin_once(self, timeout_sec=0.3)
        raise TimeoutError('Nav2 nodes did not become active')

    def feedback(self, message):
        now = time.monotonic()
        if now - self.feedback_time >= 2.0:
            self.get_logger().info(f'Distance remaining: {message.feedback.distance_remaining:.2f} m')
            self.feedback_time = now

    def run(self):
        self.wait_for_navigation()
        # Validate the entire route before sending the first action.
        for goal in self.goals:
            check_goal_clearance(goal, self.map, self.get_parameter('goal_clearance').value)
        for index, goal in enumerate(self.goals, 1):
            check_goal_clearance(goal, self.map, self.get_parameter('goal_clearance').value)
            request = NavigateToPose.Goal()
            request.pose.header.frame_id = 'map'
            request.pose.header.stamp = self.get_clock().now().to_msg()
            request.pose.pose.position.x, request.pose.pose.position.y = goal.x, goal.y
            request.pose.pose.orientation.z = math.sin(goal.yaw / 2)
            request.pose.pose.orientation.w = math.cos(goal.yaw / 2)
            self.status('navigating', goal=goal.name, index=index)
            self.get_logger().info(f'{index}/{len(self.goals)} {goal.name}: x={goal.x:.3f}, y={goal.y:.3f}, yaw={goal.yaw:.3f}')
            self.pending_goal = self.action.send_goal_async(request, feedback_callback=self.feedback)
            self.wait(self.pending_goal.done, 10.0, 'goal acceptance')
            self.goal_handle = self.pending_goal.result()
            self.pending_goal = None
            if not self.goal_handle.accepted:
                self.goal_handle = None
                raise RuntimeError(f'Nav2 rejected {goal.name}; route stopped')
            result = self.goal_handle.get_result_async()
            self.wait(result.done, self.get_parameter('goal_timeout').value, goal.name)
            outcome = result.result().status
            self.goal_handle = None
            self.results.append({'name': goal.name, 'x': goal.x, 'y': goal.y,
                                 'yaw': goal.yaw, 'status': outcome})
            if outcome != GoalStatus.STATUS_SUCCEEDED:
                raise RuntimeError(f'{goal.name} ended with action status {outcome}; route stopped')
            self.get_logger().info(f'ARRIVED {goal.name}')
            self.status('arrived', goal=goal.name, index=index)
            end = time.monotonic() + self.get_parameter('pause_at_goal').value
            while time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.05)
        self.status('completed', results=self.results)
        self.get_logger().info(f'COMPLETED {len(self.goals)}/{len(self.goals)} goals')

    def cancel(self):
        if self.pending_goal is not None:
            self.wait(self.pending_goal.done, 5.0, 'pending goal response before cancellation')
            self.goal_handle = self.pending_goal.result()
            self.pending_goal = None
        if self.goal_handle is not None and self.goal_handle.accepted:
            canceled = self.goal_handle.cancel_goal_async()
            self.wait(canceled.done, 5.0, 'goal cancellation')
            result = self.goal_handle.get_result_async()
            self.wait(result.done, 5.0, 'canceled goal result')
            self.goal_handle = None


def main(args=None):
    with stop_on_signals():
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
        node = None
        code = 0
        try:
            node = WaypointNavigator()
            node.run()
        except KeyboardInterrupt:
            code = 130
            if node is not None:
                node.status('canceled')
        except Exception as error:
            code = 1
            if node is not None:
                node.get_logger().error(str(error))
                node.status('failed', error=str(error), results=node.results)
            else:
                print(f'waypoint_navigator: {error}')
        finally:
            if node is not None:
                try:
                    node.cancel()
                except Exception as error:
                    node.get_logger().error(f'Cancellation did not complete: {error}')
                publish_stop(node, node.stop_pub)
                output = node.get_parameter('result_file').value
                if output:
                    path = Path(output).expanduser()
                    path.parent.mkdir(parents=True, exist_ok=True)
                    path.write_text(json.dumps({'exit_code': code, 'results': node.results}, indent=2) + '\n')
                node.destroy_node()
            rclpy.try_shutdown()
    return code


if __name__ == '__main__':
    raise SystemExit(main())
