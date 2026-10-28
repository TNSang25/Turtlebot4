# SPDX-License-Identifier: Apache-2.0
"""Opt-in ROS action integration: TB4_RUN_ROS_TESTS=1, private ROS_DOMAIN_ID."""

import os
import threading
import time
from pathlib import Path

import pytest

if os.environ.get('TB4_RUN_ROS_TESTS') != '1':
    pytest.skip('Set TB4_RUN_ROS_TESTS=1 for ROS/DDS integration', allow_module_level=True)

import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from robot_semantic.maps import read_map
from robot_voice.voice_navigation_node import VoiceNavigationNode


class FakeNavigation(Node):
    def __init__(self, regions):
        super().__init__('voice_nav_test_server')
        self.regions = regions
        self.mode = 'success'
        self.received, self.executed = [], []
        self.pose = (regions.by_id['C'].x, regions.by_id['C'].y)
        src = Path(__file__).resolve().parents[2]
        image = read_map(src/'robot_navigation/maps/my_map.yaml')
        g = image.geometry
        m = OccupancyGrid();m.header.frame_id = 'map'
        m.info.width, m.info.height, m.info.resolution = g.width, g.height, g.resolution
        m.info.origin.position.x, m.info.origin.position.y = g.origin[:2]
        m.info.origin.orientation.w = 1.
        m.data = np.flipud(image.occupancy).ravel().tolist()
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.map_pub.publish(m)
        self.state_services = [self.create_service(GetState, f'/{name}/get_state', self.active)
                         for name in ('amcl', 'map_server', 'bt_navigator', 'planner_server', 'controller_server')]
        self.tf = TransformBroadcaster(self)
        self.timer = self.create_timer(.05, self.transform)
        self.server = ActionServer(self, NavigateToPose, '/navigate_to_pose', execute_callback=self.execute,
                                   goal_callback=self.goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
                                   callback_group=ReentrantCallbackGroup())

    def active(self, request, response):
        response.current_state.id = 3
        response.current_state.label = 'active'
        return response

    def transform(self):
        t = TransformStamped();t.header.frame_id = 'map';t.child_frame_id = 'base_link'
        t.header.stamp = self.get_clock().now().to_msg()
        t.transform.translation.x, t.transform.translation.y = self.pose
        t.transform.rotation.w = 1.
        self.tf.sendTransform(t)

    def goal(self, request):
        p = request.pose.pose.position
        region = min(self.regions.regions, key=lambda r: (p.x-r.x)**2+(p.y-r.y)**2)
        self.received.append(region.id)
        if self.mode == 'delay_accept':
            time.sleep(.4)
        return GoalResponse.REJECT if self.mode == 'reject' else GoalResponse.ACCEPT

    def execute(self, handle):
        mode = self.mode
        self.executed.append(handle)
        end = time.monotonic()+(2. if mode in ('hold', 'delay_accept') else .15)
        while time.monotonic() < end:
            if handle.is_cancel_requested:
                handle.canceled()
                return NavigateToPose.Result()
            time.sleep(.01)
        if mode == 'abort':
            handle.abort()
        else:
            p = handle.request.pose.pose.position
            self.pose = (p.x, p.y)
            handle.succeed()
        return NavigateToPose.Result()


def test_navigation_routes_stop_unknown_regions_failures_and_late_acceptance():
    rclpy.init(args=['--ros-args', '--log-level', 'warn'])
    client = VoiceNavigationNode()
    server = FakeNavigation(client.regions)
    publisher = server.create_publisher(String, '/voice/text', 10)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(client);executor.add_node(server)
    thread = threading.Thread(target=executor.spin, daemon=True);thread.start()

    def wait(predicate, timeout=6.):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            if predicate():
                return
            time.sleep(.02)
        pytest.fail(f'Timed out: {client.state}, {client.reason}, received={server.received}')

    def say(text):
        publisher.publish(String(data=text))

    def relocate(identifier):
        r = client.regions.by_id[identifier]
        server.pose = (r.x, r.y)
        wait(lambda: client.regions.region_at(*client.current_position()) == identifier)

    try:
        wait(lambda: client.state == 'ready', 10.)
        say('Đi đến khu vực Z')
        wait(lambda: client.state == 'rejected')
        assert not server.received and not client.busy
        assert 'Không có khu vực' in client.notice
        assert 'Không có khu vực' in client.reason

        say('Di chuyển từ khu vực A đến khu vực B')
        wait(lambda: len(server.received) == 2 and client.state == 'completed')
        assert server.received == ['A', 'B']
        assert client.completed == ['A', 'B']
        relocate('A')
        say('Di chuyển từ khu vực A đến khu vực B')
        wait(lambda: len(server.received) == 3 and client.state == 'completed')
        assert server.received[-1] == 'B' and client.completed == ['B']

        server.mode = 'hold'
        say('Đi đến phòng thí nghiệm')
        wait(lambda: client.goal_handle is not None)
        count = len(server.received)
        say('Đi đến khu vực D')
        wait(lambda: 'Không có khu vực' in client.notice)
        assert client.busy and len(server.received) == count
        say('Dừng lại')
        wait(lambda: client.state == 'canceled' and not client.busy)

        server.mode = 'abort'
        relocate('C')
        before = len(server.received)
        say('Đi từ A đến B')
        wait(lambda: client.state == 'failed' and len(server.received) > before)
        assert server.received[before:] == ['A']  # Never send B after A aborts.

        server.mode = 'delay_accept'
        say('Đi đến khu vực A')
        wait(lambda: client.pending is not None)
        say('Hủy điều hướng')
        wait(lambda: client.state == 'canceled' and not client.busy)
        assert not any(handle.is_active for handle in server.executed)

        server.mode = 'reject'
        say('Đi đến khu vực B')
        wait(lambda: client.state == 'failed')
        assert not client.busy
    finally:
        if client.busy:
            say('Dừng lại')
            time.sleep(.7)
        executor.shutdown(timeout_sec=5.)
        thread.join(timeout=2.)
        server.server.destroy()
        client.destroy_node();server.destroy_node();rclpy.try_shutdown()
