#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Check running simulation topics and TF; optionally drive a short distance."""

import argparse
import math
import sys
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


class Probe(Node):
    def __init__(self, camera=False):
        super().__init__('robot_sim_smoke_test')
        self.messages = {}
        self.camera = camera
        self.subscriptions_ = []
        topics = [('clock', Clock), ('scan', LaserScan), ('odom', Odometry),
                  ('imu', Imu), ('joint_states', JointState)]
        if camera:
            topics += [('camera/image', Image), ('camera/camera_info', CameraInfo),
                       ('camera/depth_image', Image)]
        for key, msg_type in topics:
            self.subscriptions_.append(self.create_subscription(
                msg_type, '/' + key, lambda msg, k=key: self.messages.__setitem__(k, msg),
                qos_profile_sensor_data))
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def wait_until(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def ready(self):
        if any(key not in self.messages for key in ('clock', 'scan', 'odom', 'imu', 'joint_states')):
            return False
        if self.camera and any(key not in self.messages
                               for key in ('camera/image', 'camera/camera_info',
                                           'camera/depth_image')):
            return False
        return all(self.buffer.can_transform(parent, child, rclpy.time.Time())
                   for parent, child in [('odom', 'base_link'), ('base_link', 'rplidar_link')])

    def validate(self):
        scan = self.messages['scan']
        odom = self.messages['odom']
        assert scan.header.frame_id == 'rplidar_link', scan.header.frame_id
        assert len(scan.ranges) >= 180, 'Incomplete laser scan'
        assert any(math.isfinite(r) and scan.range_min <= r <= scan.range_max
                   for r in scan.ranges), 'No lidar returns; run in the demo world'
        assert odom.header.frame_id == 'odom', odom.header.frame_id
        assert odom.child_frame_id == 'base_link', odom.child_frame_id
        assert self.messages['imu'].header.frame_id == 'imu_link'
        assert len(self.messages['joint_states'].name) >= 2, 'Missing wheel joint states'
        stamp = self.messages['clock'].clock
        assert stamp.sec > 0 or stamp.nanosec > 0, 'Simulation clock is stopped at zero'
        self.buffer.lookup_transform('odom', 'rplidar_link', rclpy.time.Time())
        if self.camera:
            image = self.messages['camera/image']
            info = self.messages['camera/camera_info']
            depth = self.messages['camera/depth_image']
            assert image.header.frame_id == info.header.frame_id == 'oakd_rgb_camera_optical_frame'
            assert depth.header.frame_id == info.header.frame_id
            assert depth.width == image.width and depth.height == image.height
            assert depth.encoding == '32FC1' and len(depth.data) == depth.step * depth.height
            assert image.width == info.width and image.height == info.height
            assert len(image.data) == image.step * image.height and image.width > 0
            self.buffer.lookup_transform('base_link', 'oakd_rgb_camera_optical_frame',
                                         rclpy.time.Time())
            print(f'PASS: RGB + depth + camera info {image.width}x{image.height}, optical TF')

    def drive(self):
        start = self.messages['odom'].pose.pose.position
        command = Twist()
        command.linear.x = 0.12
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            self.publisher.publish(command)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.03)
        # Gazebo receives /cmd_vel directly; explicitly command a stop.
        self.publisher.publish(Twist())
        assert self.wait_until(lambda: abs(self.messages['odom'].twist.twist.linear.x) < 0.02,
                               5.0), 'Robot did not stop after the zero velocity command'
        end = self.messages['odom'].pose.pose.position
        distance = math.hypot(end.x - start.x, end.y - start.y)
        assert distance > 0.03, f'Robot did not move: {distance:.3f} m'
        print(f'PASS: moved {distance:.3f} m, then stopped with a zero velocity command')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=45.0)
    parser.add_argument('--drive', action='store_true',
                        help='Drive forward at 0.12 m/s for 2 s in a clear simulation')
    parser.add_argument('--camera', action='store_true',
                        help='Also check optional camera image, calibration and optical TF')
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    probe = Probe(camera=args.camera)
    result = 0
    try:
        if not probe.wait_until(probe.ready, args.timeout):
            raise AssertionError('Timed out waiting for topics/TF. Received: ' +
                                 ', '.join(sorted(probe.messages)))
        probe.validate()
        print('PASS: /clock /scan /odom /imu /joint_states and odom -> base_link -> rplidar_link')
        if args.drive:
            probe.drive()
    except (AssertionError, TransformException) as error:
        print(f'FAIL: {error}', file=sys.stderr)
        result = 1
    except KeyboardInterrupt:
        result = 130
    finally:
        if args.drive and rclpy.ok():
            probe.publisher.publish(Twist())
        probe.destroy_node()
        rclpy.try_shutdown()
    return result


if __name__ == '__main__':
    sys.exit(main())
