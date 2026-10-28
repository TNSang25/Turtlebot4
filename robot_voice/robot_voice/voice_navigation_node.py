# SPDX-License-Identifier: Apache-2.0
"""Turn final spoken commands into a sequential, cancelable Nav2 mission."""

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
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from robot_node.goals import Goal, check_goal_clearance, yaw_from_quaternion
from robot_node.runtime import publish_stop, stop_on_signals
from robot_semantic.maps import Geometry, RegionSet, fingerprint
from .commands import CommandParser, make_route


class VoiceNavigationNode(Node):
    def __init__(self):
        super().__init__('voice_navigation_node')
        default = str(Path(get_package_share_directory('robot_semantic'))/'config/regions.yaml')
        for name, value in [('regions_file', default), ('goal_clearance', .30), ('goal_timeout', 300.),
                            ('accept_timeout', 10.), ('cancel_timeout', 5.), ('tf_max_age', 3.)]:
            self.declare_parameter(name, value)
        self.options = {name: self.get_parameter(name).value for name in (
            'goal_clearance', 'goal_timeout', 'accept_timeout', 'cancel_timeout', 'tf_max_age')}
        if any(not math.isfinite(v) or v <= 0 for v in self.options.values()):
            raise ValueError('Navigation thresholds must be positive and finite')
        self.regions = RegionSet(self.get_parameter('regions_file').value)
        self.parser = CommandParser(self.regions.regions)
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '/voice/navigation_status', durable)
        self.stop_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.on_map, durable)
        self.text_sub = self.create_subscription(String, '/voice/text', self.on_text, 10)
        self.action = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.lifecycle = {name: self.create_client(GetState, f'/{name}/get_state')
                          for name in ('amcl', 'map_server', 'bt_navigator', 'planner_server', 'controller_server')}
        self.lifecycle_pending = {}
        self.active_nodes = {}
        self.map = None
        self.map_error = 'Chưa nhận /map'
        self.state, self.reason = 'waiting_for_navigation', 'Chờ map, Initial Pose và Nav2 active'
        self.route, self.index, self.completed = [], 0, []
        self.pending = self.goal_handle = self.result = self.cancel_future = None
        self.cancel_requested = False
        self.cancel_reason = ''
        self.text, self.notice = '', ''
        self.sent_at = self.cancel_at = self.last_poll = self.last_publish = self.last_feedback = 0.
        self.timer = self.create_timer(.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.emit()

    @property
    def busy(self):
        return bool(self.route) or self.pending is not None or self.goal_handle is not None

    def emit(self, state=None, reason=None, **extra):
        if state is not None:
            self.state = state
        if reason is not None:
            self.reason = reason
        payload = dict(state=self.state, reason=self.reason, busy=self.busy,
                       text=self.text, route=self.route, index=self.index,
                       completed=self.completed, notice=self.notice,
                       regions={r.id: r.name for r in self.regions.regions}, **extra)
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        self.last_publish = time.monotonic()
        if state is not None or extra:
            self.get_logger().info(json.dumps(payload, ensure_ascii=False))

    def on_map(self, message):
        try:
            g = message.info
            if message.header.frame_id != 'map' or g.width*g.height != len(message.data):
                raise ValueError('OccupancyGrid không hợp lệ hoặc không thuộc frame map')
            geometry = Geometry(g.width, g.height, g.resolution,
                                (g.origin.position.x, g.origin.position.y, yaw_from_quaternion(g.origin.orientation)))
            image_order = np.flipud(np.asarray(message.data, dtype=np.int8).reshape(g.height, g.width))
            if fingerprint(geometry, image_order) != self.regions.signature:
                raise ValueError('Bản đồ không khớp file phân vùng; hãy phân vùng lại đúng map đang nạp')
            self.map, self.map_error = message, ''
        except (ValueError, TypeError) as error:
            self.map, self.map_error = None, str(error)
            if self.busy:
                self.request_cancel(self.map_error)

    def poll_lifecycle(self, now):
        for name, future in list(self.lifecycle_pending.items()):
            if future.done():
                try:
                    reply = future.result()
                    self.active_nodes[name] = (reply is not None and reply.current_state.id == 3, now)
                except Exception:
                    self.active_nodes[name] = (False, now)
                del self.lifecycle_pending[name]
        if now-self.last_poll >= 1.:
            self.last_poll = now
            for name, client in self.lifecycle.items():
                if name not in self.lifecycle_pending and client.service_is_ready():
                    self.lifecycle_pending[name] = client.call_async(GetState.Request())

    def current_position(self):
        transform = self.tf.lookup_transform('map', 'base_link', rclpy.time.Time())
        stamp = rclpy.time.Time.from_msg(transform.header.stamp, clock_type=self.get_clock().clock_type)
        if abs((self.get_clock().now()-stamp).nanoseconds)/1e9 > self.options['tf_max_age']:
            raise ValueError('TF map -> base_link đã cũ; kiểm tra AMCL và /clock')
        return transform.transform.translation.x, transform.transform.translation.y

    def not_ready(self):
        if self.map_error:
            return self.map_error
        if self.get_clock().now().nanoseconds <= 0:
            return 'Chưa có thời gian mô phỏng /clock'
        if not self.action.server_is_ready():
            return 'Chưa có action /navigate_to_pose'
        now = time.monotonic()
        inactive = [name for name in self.lifecycle if not self.active_nodes.get(name, (False, 0))[0]
                    or now-self.active_nodes[name][1] > 3.]
        if inactive:
            return 'Chưa active: '+', '.join(inactive)
        try:
            self.current_position()
        except (TransformException, ValueError) as error:
            return 'Chưa định vị: '+str(error)
        return ''

    def reject(self, message, text):
        self.notice = message
        self.emit(None if self.busy else 'rejected', None if self.busy else message,
                  event='command_rejected', rejected_text=text)

    def on_text(self, message):
        text = message.data.strip()
        if not text:
            return
        try:
            command = self.parser.parse(text)
        except ValueError as error:
            self.reject(str(error), text)
            return
        if command.kind == 'stop':
            self.request_cancel('Dừng theo lệnh giọng nói')
            return
        if self.busy:
            self.reject('Robot đang thực hiện lệnh khác. Nói "dừng lại" trước khi đổi hành trình.', text)
            return
        reason = self.not_ready()
        if reason:
            self.reject(reason, text)
            return
        try:
            current = self.regions.region_at(*self.current_position())
            route = make_route(command, current)
            for identifier in route:
                r = self.regions.by_id[identifier]
                check_goal_clearance(Goal(r.name, r.x, r.y, r.yaw), self.map, self.options['goal_clearance'])
        except (TransformException, ValueError) as error:
            self.reject(str(error), text)
            return
        self.text, self.notice = text, ''
        self.route, self.index, self.completed = route, 0, []
        self.cancel_requested = False
        self.send_next()

    def send_next(self):
        reason = self.not_ready()
        if reason:
            self.route = []
            self.emit('failed', reason)
            return
        r = self.regions.by_id[self.route[self.index]]
        try:
            check_goal_clearance(Goal(r.name, r.x, r.y, r.yaw), self.map, self.options['goal_clearance'])
        except ValueError as error:
            self.route = []
            self.emit('failed', str(error))
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = r.x, r.y
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(r.yaw/2), math.cos(r.yaw/2)
        self.sent_at = time.monotonic()
        self.pending = self.action.send_goal_async(goal, feedback_callback=self.feedback)
        self.pending.add_done_callback(self.accepted)
        self.emit('sending_goal', f'Đi đến {r.id}: {r.name}')

    def accepted(self, future):
        self.pending = None
        try:
            handle = future.result()
        except Exception as error:
            self.route = []
            self.emit('failed', f'Lỗi gửi goal: {error}')
            self.cancel_requested = False
            return
        if handle is None or not handle.accepted:
            self.route = []
            self.emit('canceled' if self.cancel_requested else 'failed', 'Nav2 từ chối goal')
            self.cancel_requested = False
            return
        self.goal_handle = handle
        self.result = handle.get_result_async()
        self.result.add_done_callback(self.finished)
        if self.cancel_requested:
            self.issue_cancel()
        else:
            self.emit('navigating')

    def finished(self, future):
        self.goal_handle = self.result = self.cancel_future = None
        try:
            result = future.result()
            outcome = result.status
        except Exception as error:
            self.route = []
            self.cancel_requested = False
            self.emit('failed', f'Lỗi nhận kết quả Nav2: {error}')
            return
        if self.cancel_requested:
            self.route = []
            self.cancel_requested = False
            self.emit('canceled', self.cancel_reason)
        elif outcome == GoalStatus.STATUS_SUCCEEDED:
            identifier = self.route[self.index]
            self.completed.append(identifier)
            self.index += 1
            if self.index >= len(self.route):
                self.route = []
                self.emit('completed', 'Đã đến: '+', '.join(self.completed))
            else:
                self.emit('arrived', f'Đã đến {identifier}; chuyển sang vùng tiếp theo')
                self.send_next()
        else:
            self.route = []
            self.emit('failed', f'Nav2 kết thúc với status {outcome}; không gửi các goal còn lại')

    def feedback(self, message):
        if not self.cancel_requested and time.monotonic()-self.last_feedback >= 1.:
            self.last_feedback = time.monotonic()
            self.emit(distance_remaining=round(message.feedback.distance_remaining, 3))

    def issue_cancel(self):
        if self.goal_handle is not None and self.cancel_future is None:
            self.cancel_future = self.goal_handle.cancel_goal_async()

    def request_cancel(self, reason):
        if not self.busy:
            self.emit('idle', 'Không có hành trình đang chạy')
            return
        self.route = []
        if not self.cancel_requested:
            self.cancel_at = time.monotonic()
        self.cancel_requested, self.cancel_reason = True, reason
        self.issue_cancel()
        self.stop_pub.publish(Twist())
        self.emit('canceling', reason)

    def tick(self):
        now = time.monotonic()
        self.poll_lifecycle(now)
        if self.cancel_requested:
            self.stop_pub.publish(Twist())
            if now-self.cancel_at > self.options['cancel_timeout'] and self.state != 'cancel_timeout':
                self.emit('cancel_timeout', 'Đang chờ Nav2 xác nhận hủy; chưa nhận lệnh mới')
        elif self.pending is not None and now-self.sent_at > self.options['accept_timeout']:
            self.request_cancel('Quá thời gian chờ Nav2 nhận goal')
        elif self.goal_handle is not None and now-self.sent_at > self.options['goal_timeout']:
            self.request_cancel('Quá thời gian điều hướng tới vùng')
        elif not self.busy and self.state in ('waiting_for_navigation', 'ready'):
            reason = self.not_ready()
            state = 'waiting_for_navigation' if reason else 'ready'
            if state != self.state or reason != self.reason:
                self.emit(state, reason or 'Sẵn sàng nhận lệnh giọng nói')
        if now-self.last_publish >= 1.:
            self.emit()

    def shutdown_mission(self):
        owned = self.busy
        if owned:
            self.request_cancel('Đóng node điều hướng giọng nói')
            deadline = time.monotonic()+self.options['cancel_timeout']
            while self.busy and rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=.05)
            publish_stop(self, self.stop_pub)


def main(args=None):
    with stop_on_signals():
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
        node = None
        code = 0
        try:
            node = VoiceNavigationNode()
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        except Exception as error:
            print(f'voice_navigation_node: {error}')
            code = 1
        finally:
            if node is not None:
                node.shutdown_mission()
                node.destroy_node()
            rclpy.try_shutdown()
    return code
