# SPDX-License-Identifier: Apache-2.0
"""Show region boundaries, names and safe goals on the existing ROS map."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import cv2
from geometry_msgs.msg import Point
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from robot_node.runtime import stop_on_signals

from .maps import RegionSet
from .segmentation import COLORS


class SemanticMapNode(Node):
    def __init__(self):
        super().__init__('semantic_map_node')
        default = str(Path(get_package_share_directory('robot_semantic'))/'config/regions.yaml')
        self.declare_parameter('regions_file', default)
        self.regions = RegionSet(self.get_parameter('regions_file').value)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.markers = self.create_publisher(MarkerArray, '/semantic_map/markers', qos)
        self.images = self.create_publisher(Image, '/semantic_map/image', qos)
        self.marker_data = self.make_markers()
        image_path = self.regions.path.parent.parent/'maps'/(self.regions.path.stem+'_preview.png')
        image = cv2.imread(str(image_path))
        if image is None:
            # The label file alone is sufficient for RViz visualization.
            import numpy as np
            image = np.full((*self.regions.labels.shape, 3), 205, dtype=np.uint8)
            for region in self.regions.regions:
                image[self.regions.labels == region.mask_value] = COLORS[(region.mask_value-1) % len(COLORS)]
        self.image_data = cv2.resize(image, (self.regions.geometry.width, self.regions.geometry.height), interpolation=cv2.INTER_AREA)
        self.timer = self.create_timer(1., self.publish, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.publish()
        self.get_logger().info('Semantic areas: '+', '.join(f'{r.id}: {r.name}' for r in self.regions.regions))

    def make_markers(self):
        array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        array.markers.append(clear)
        for region in self.regions.regions:
            b, g, r = COLORS[(region.mask_value-1) % len(COLORS)]
            for kind in ('boundary', 'goal', 'name'):
                marker = Marker()
                marker.header.frame_id = 'map'
                marker.ns, marker.id = 'semantic_'+kind, region.mask_value
                marker.action = Marker.ADD
                marker.pose.orientation.w = 1.
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = r/255., g/255., b/255., 1.
                if kind == 'boundary':
                    marker.type = Marker.LINE_LIST
                    marker.scale.x = .035
                    mask = (self.regions.labels == region.mask_value).astype('uint8')
                    contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
                    for contour in contours:
                        contour = cv2.approxPolyDP(contour, .6, True).reshape(-1, 2)
                        for i, (col, row) in enumerate(contour):
                            for pixel in ((col, row), contour[(i+1) % len(contour)]):
                                x, y = self.regions.geometry.pixel_to_world(*pixel)
                                marker.points.append(Point(x=x, y=y, z=.035))
                else:
                    marker.pose.position.x, marker.pose.position.y = region.x, region.y
                    if kind == 'goal':
                        marker.type = Marker.SPHERE
                        marker.pose.position.z = .10
                        marker.scale.x = marker.scale.y = marker.scale.z = .18
                    else:
                        marker.type = Marker.TEXT_VIEW_FACING
                        marker.pose.position.z = .45
                        marker.scale.z = .28
                        marker.text = f'{region.id}: {region.name}'
                array.markers.append(marker)
        return array

    def publish(self):
        now = self.get_clock().now().to_msg()
        for marker in self.marker_data.markers:
            marker.header.stamp = now
        self.markers.publish(self.marker_data)
        image = Image()
        image.header.stamp, image.header.frame_id = now, 'map'
        image.height, image.width = self.image_data.shape[:2]
        image.encoding, image.step = 'bgr8', image.width*3
        image.data = self.image_data.tobytes()
        self.images.publish(image)


def main(args=None):
    with stop_on_signals():
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
        node = None
        try:
            node = SemanticMapNode()
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            if node is not None:
                node.destroy_node()
            rclpy.try_shutdown()
