# SPDX-License-Identifier: Apache-2.0
"""Interactive text input through the same command topic used by speech."""

import codecs
import json
import os
import select
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from robot_node.runtime import stop_on_signals


HELP = '''
Nhập câu lệnh rồi nhấn Enter, ví dụ:
  Đi đến phòng học
  Đi đến phòng thí nghiệm
  Di chuyển từ phòng học đến phòng họp
  Dừng lại
:help: hướng dẫn; :q hoặc Ctrl+C: dừng hành trình đang chạy và thoát.
'''


class TextCommandNode(Node):
    def __init__(self):
        super().__init__('text_command_node')
        self.publisher = self.create_publisher(String, '/voice/text', 10)
        self.subscription = self.create_subscription(
            String, '/voice/navigation_status', self.on_status,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.status = {}
        self.status_at = 0.
        self.last_summary = None
        self.regions_shown = False
        self.sent_command = False

    def on_status(self, message):
        try:
            status = json.loads(message.data)
            if not isinstance(status, dict):
                return
        except (TypeError, ValueError):
            return
        self.status, self.status_at = status, time.monotonic()
        if not self.regions_shown and status.get('regions'):
            self.regions_shown = True
            print('\nKhu vực: '+', '.join(f'{key}: {name}' for key, name in status['regions'].items()), flush=True)
        summary = (status.get('state', ''), status.get('reason', ''), status.get('notice', ''))
        if summary != self.last_summary:
            self.last_summary = summary
            state, reason, notice = summary
            print(f'\n[{state}] {reason}', flush=True)
            if notice and notice != reason:
                print(f'Thông báo: {notice}', flush=True)

    def connected(self):
        return self.publisher.get_subscription_count() > 0 and time.monotonic()-self.status_at < 3.

    def send(self, text):
        text = text.strip()
        if not text:
            return True
        if text in (':q', ':quit'):
            return False
        if text == ':help':
            print(HELP, flush=True)
        else:
            # Status can arrive before discovery of the command subscription.
            # Give a first command time to connect instead of silently losing it.
            deadline = time.monotonic()+2.
            while rclpy.ok() and not self.connected() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=.05)
            if not self.connected():
                print('Chưa kết nối bộ nhận lệnh. Chạy voice_navigation.launch.py với speech:=false và kiểm tra ROS_DOMAIN_ID.', flush=True)
            else:
                self.publisher.publish(String(data=text))
                self.sent_command = True
                print(f'Đã gửi: {text}', flush=True)
        return True

    def run(self):
        print(HELP, flush=True)
        print('Đang chờ /voice/navigation_status. Gõ lệnh trên dòng nhập bên dưới.', flush=True)
        descriptor = sys.stdin.fileno()
        decoder = codecs.getincrementaldecoder('utf-8')(errors='replace')
        pending = ''
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=.03)
            readable, _, _ = select.select([descriptor], [], [], .03)
            if not readable:
                continue
            data = os.read(descriptor, 4096)
            if not data:
                return
            pending += decoder.decode(data)
            while '\n' in pending:
                line, pending = pending.split('\n', 1)
                if not self.send(line):
                    return

    def stop_mission(self):
        if not self.connected() or not (self.sent_command or self.status.get('busy')):
            return
        print('\nGửi "dừng lại" trước khi thoát...', flush=True)
        self.publisher.publish(String(data='Dừng lại'))
        deadline = time.monotonic()+2.
        flush_until = time.monotonic()+.3
        while rclpy.ok() and time.monotonic() < deadline:
            if time.monotonic() >= flush_until and not self.status.get('busy'):
                break
            rclpy.spin_once(self, timeout_sec=.05)


def main(args=None):
    if not sys.stdin.isatty():
        print('Chạy trong terminal: ros2 run robot_voice text_command_node', file=sys.stderr)
        return 1
    with stop_on_signals():
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
        node, code = None, 0
        try:
            node = TextCommandNode()
            node.run()
        except KeyboardInterrupt:
            pass
        except Exception as error:
            print(f'text_command_node: {error}', file=sys.stderr)
            code = 1
        finally:
            if node is not None:
                node.stop_mission()
                node.destroy_node()
            rclpy.try_shutdown()
    return code
