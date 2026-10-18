# SPDX-License-Identifier: Apache-2.0
"""Convert ROS RGB frames with cv_bridge, run YOLO and publish annotated results."""

import json
import math
from pathlib import Path
import signal
import threading
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image
from std_msgs.msg import String


class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        for name, value in [
            ('model_path', ''), ('image_topic', '/camera/image'),
            ('annotated_topic', '/detections/image'), ('detections_topic', '/detections/objects'),
            ('device', 'cpu'), ('confidence', 0.35), ('image_size', 640),
            ('inference_rate', 3.0), ('cpu_threads', 2), ('max_frame_age', 2.0),
            ('show_window', False), ('log_interval', 2.0),
        ]:
            self.declare_parameter(name, value)
        self.model_path = Path(self.get_parameter('model_path').value).expanduser()
        if not self.model_path.is_file():
            raise FileNotFoundError('YOLO weights missing. Download models/yolov8n.pt or pass model_path:=/path/to/best.pt')
        confidence = self.get_parameter('confidence').value
        if not math.isfinite(confidence) or not 0 < confidence <= 1:
            raise ValueError('confidence must be in (0, 1]')
        for name in ('inference_rate', 'max_frame_age', 'log_interval'):
            value = self.get_parameter(name).value
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be positive and finite')
        for name in ('image_size', 'cpu_threads'):
            if self.get_parameter(name).value <= 0:
                raise ValueError(f'{name} must be positive')
        # Delayed imports keep model dependencies out of teleop/navigation and colcon build.
        try:
            import torch
            from ultralytics import YOLO
        except ImportError as error:
            raise RuntimeError('Install robot_perception/requirements.txt in .venv-yolo and select that Python interpreter') from error
        torch.set_num_threads(self.get_parameter('cpu_threads').value)
        cv2.setNumThreads(1)
        self.model = YOLO(str(self.model_path), task='detect')
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest = None
        self.last_arrival = time.monotonic()
        self.stop_event = threading.Event()
        self.last_log = 0.0
        self.output = self.create_publisher(Image, self.get_parameter('annotated_topic').value, 1)
        self.detections = self.create_publisher(String, self.get_parameter('detections_topic').value, 10)
        self.subscription = self.create_subscription(
            Image, self.get_parameter('image_topic').value, self.receive, qos_profile_sensor_data)
        self.timer = self.create_timer(5.0, self.check_camera)
        self.worker = threading.Thread(target=self.inference, name='yolo-inference', daemon=True)
        self.worker.start()
        self.get_logger().info(f'YOLO ready: {self.model_path.name}; RGB input {self.get_parameter("image_topic").value}')

    def receive(self, message):
        # Store only the newest frame so inference cannot create an image backlog.
        with self.lock:
            self.latest = message
            self.last_arrival = time.monotonic()

    def check_camera(self):
        if time.monotonic() - self.last_arrival > 5:
            self.get_logger().warning('No fresh RGB image. Enable the camera and check image_topic / simulation clock.')

    def inference(self):
        period = 1.0 / self.get_parameter('inference_rate').value
        while not self.stop_event.is_set() and rclpy.ok():
            begin = time.monotonic()
            with self.lock:
                message, self.latest = self.latest, None
                arrival = self.last_arrival
            if message is not None:
                age = (self.get_clock().now().nanoseconds -
                       (message.header.stamp.sec * 10**9 + message.header.stamp.nanosec)) / 1e9
                if time.monotonic() - arrival > self.get_parameter('max_frame_age').value or age > self.get_parameter('max_frame_age').value:
                    continue
                try:
                    frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
                    result = self.model.predict(
                        source=frame, conf=self.get_parameter('confidence').value,
                        imgsz=self.get_parameter('image_size').value,
                        device=self.get_parameter('device').value, verbose=False)[0]
                    rows = []
                    if result.boxes is not None:
                        boxes = result.boxes.cpu()
                        for box, score, category in zip(boxes.xyxy.tolist(), boxes.conf.tolist(), boxes.cls.tolist()):
                            class_id = int(category)
                            rows.append({'class_id': class_id, 'class_name': result.names[class_id],
                                         'confidence': round(float(score), 4),
                                         'bbox_xyxy': [round(float(v), 2) for v in box]})
                    annotated = self.bridge.cv2_to_imgmsg(result.plot(), encoding='bgr8')
                    annotated.header = message.header
                    if self.stop_event.is_set() or not rclpy.ok():
                        break
                    self.output.publish(annotated)
                    self.detections.publish(String(data=json.dumps({
                        'frame_id': message.header.frame_id,
                        'stamp': {'sec': message.header.stamp.sec, 'nanosec': message.header.stamp.nanosec},
                        'image_width': message.width, 'image_height': message.height,
                        'inference_ms': round((time.monotonic() - begin) * 1000, 1),
                        'detections': rows,
                    })))
                    if time.monotonic() - self.last_log >= self.get_parameter('log_interval').value:
                        summary = ', '.join(f'{r["class_name"]} {r["confidence"]:.2f}' for r in rows)
                        self.get_logger().info(f'DETECTED {summary}' if rows else 'No object above confidence threshold')
                        self.last_log = time.monotonic()
                    if self.get_parameter('show_window').value:
                        cv2.imshow('TurtleBot4 YOLO', self.bridge.imgmsg_to_cv2(annotated, 'bgr8'))
                        if cv2.waitKey(1) & 0xff == ord('q'):
                            self.stop_event.set()
                except Exception as error:
                    if not self.stop_event.is_set() and rclpy.ok():
                        self.get_logger().error(f'YOLO frame failed: {error}')
                        self.stop_event.wait(1.0)
            self.stop_event.wait(max(0.01, period - (time.monotonic() - begin)))

    def destroy_node(self):
        self.stop_event.set()
        # Join before destroying publishers or the underlying ROS node.
        self.worker.join()
        if self.get_parameter('show_window').value:
            cv2.destroyAllWindows()
        return super().destroy_node()


def main(args=None):
    interrupted = False

    def interrupt(signum, frame):
        nonlocal interrupted
        if not interrupted:
            interrupted = True
            raise KeyboardInterrupt

    previous = {sig: signal.signal(sig, interrupt) for sig in (signal.SIGINT, signal.SIGTERM)}
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    code = 0
    try:
        node = YoloDetector()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as error:
        print(f'yolo_detector: {error}')
        code = 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
    return code


if __name__ == '__main__':
    raise SystemExit(main())
