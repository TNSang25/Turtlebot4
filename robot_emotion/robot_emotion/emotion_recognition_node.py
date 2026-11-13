# SPDX-License-Identifier: Apache-2.0
"""Live webcam FER inference and OpenCV preview, with only the newest frame."""
import json
import math
import os
import threading
import time
import textwrap

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.msg import ParameterDescriptor

from robot_node.runtime import stop_on_signals
from .expressions import ExpressionFilter, LABELS


class LatestCamera:
    def __init__(self, cv2, options):
        self.camera = cv2.VideoCapture(options['camera_index'], cv2.CAP_V4L2)
        if not self.camera.isOpened():
            self.camera.release()
            raise RuntimeError(f'Cannot open webcam {options["camera_index"]}; check /dev/video* and camera use')
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, options['width'])
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, options['height'])
        self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.lock, self.stop = threading.Lock(), threading.Event()
        self.frame, self.stamp, self.sequence, self.error = None, 0., 0, ''
        self.worker = threading.Thread(target=self.capture, name='emotion-webcam', daemon=True)
        self.worker.start()

    def capture(self):
        try:
            while not self.stop.is_set():
                ok, frame = self.camera.read()
                stamp = time.monotonic()
                if not ok or frame is None:
                    self.error = 'Webcam stopped providing frames'
                    return
                with self.lock:
                    self.frame, self.stamp, self.sequence = frame, stamp, self.sequence+1
        except Exception as error:
            self.error = str(error)

    def newest(self):
        with self.lock:
            return self.frame, self.stamp, self.sequence

    def close(self):
        self.stop.set()
        self.worker.join(timeout=1.)
        if not self.worker.is_alive():
            self.camera.release()


class EmotionRecognitionNode(Node):
    def __init__(self):
        super().__init__('emotion_recognition_node')
        defaults = dict(camera_index=0, width=640, height=480, frame_rate=12.,
                        confidence=.5, score_margin=.08, smoothing=.5, stable_frames=3,
                        max_frame_age=.5, mirror=True, show_window=True)
        for name, value in defaults.items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        self.options = {name: self.get_parameter(name).value for name in defaults}
        o = self.options
        if o['camera_index'] < 0 or min(o['width'], o['height']) <= 0:
            raise ValueError('Camera index must be nonnegative and image dimensions positive')
        if not math.isfinite(o['frame_rate']) or not 0 < o['frame_rate'] <= 60:
            raise ValueError('frame_rate must be in (0, 60]')
        self.filter = ExpressionFilter(**{k: o[k] for k in (
            'confidence', 'score_margin', 'smoothing', 'stable_frames')}, max_gap=o['max_frame_age'])
        self.output = self.create_publisher(String, '/detected_emotion', 1)
        self.details = self.create_publisher(String, '/emotion/recognition', 1)
        self.images = self.create_publisher(Image, '/emotion/image', 1)
        self.bridge = CvBridge()
        self.behavior, self.behavior_time = {}, 0.
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_sub = self.create_subscription(String, '/emotion_behavior/state', self.state, durable)

    def state(self, message):
        try:
            payload = json.loads(message.data)
            self.behavior = payload if isinstance(payload, dict) else {}
            self.behavior_time = time.monotonic()
        except (TypeError, ValueError):
            self.behavior = {}

    def draw(self, cv2, frame, detections, label, score, raw, elapsed):
        import numpy as np
        height, width = frame.shape[:2]
        canvas = np.zeros((max(height, 480), width+350, 3), dtype=np.uint8)
        canvas[:height, :width] = frame
        for face in detections:
            x, y, w, h = [int(v) for v in face['box']]
            cv2.rectangle(canvas, (max(0, x), max(0, y)), (min(width-1, x+w), min(height-1, y+h)), (30, 220, 30), 2)
        x = width+12
        lines = [f'Emotion: {label}', f'Confidence: {score:.2f} | raw: {raw}', f'Inference: {elapsed*1000:.0f} ms']
        state = self.behavior if time.monotonic()-self.behavior_time < 2 else {}
        lines += [f'Robot: {state.get("state", "controller not connected")}',
                  f'Running: {state.get("action") or "-"}']
        for i, line in enumerate(lines):
            cv2.putText(canvas, line, (x, 24+i*23), cv2.FONT_HERSHEY_SIMPLEX, .48, (240, 240, 240), 1, cv2.LINE_AA)
        scores = detections[0]['emotions'] if len(detections) == 1 else {}
        for i, emotion in enumerate(LABELS):
            y = 150+i*27
            value = float(scores.get(emotion, 0.))
            cv2.putText(canvas, f'{emotion:8} {value:.2f}', (x, y), cv2.FONT_HERSHEY_SIMPLEX, .45, (230, 230, 230), 1, cv2.LINE_AA)
            cv2.rectangle(canvas, (x+160, y-11), (x+160+int(155*value), y), (70, 190, 80), -1)
        message = state.get('reason', 'Recognition only, or controller in a different ROS domain')
        for i, line in enumerate(textwrap.wrap(message, 39)[:3]):
            cv2.putText(canvas, line, (x, 350+i*20), cv2.FONT_HERSHEY_SIMPLEX, .43, (255, 210, 120), 1, cv2.LINE_AA)
        for i, line in enumerate(['Neutral: STOP / re-arm', 'One motion per expression.', 'Neutral then same emotion: repeat.', 'Close this window to stop.']):
            cv2.putText(canvas, line, (x, 417+i*18), cv2.FONT_HERSHEY_SIMPLEX, .42, (210, 210, 210), 1, cv2.LINE_AA)
        return canvas

    def run(self):
        os.environ.setdefault('TF_CPP_MIN_LOG_LEVEL', '2')
        os.environ.setdefault('TF_NUM_INTRAOP_THREADS', '2')
        os.environ.setdefault('TF_NUM_INTEROP_THREADS', '1')
        try:
            import cv2
            from fer.fer import FER
        except ImportError as error:
            raise RuntimeError('Run robot_emotion/scripts/install_dependencies.py and use .venv-emotion/bin/python') from error
        detector = FER(mtcnn=False, use_tflite=True)
        camera, last_sequence, previous = None, -1, None
        waiting_since = time.monotonic()
        try:
            camera = LatestCamera(cv2, self.options)
            self.get_logger().info('Webcam FER ready: one visible face; Neutral stops the robot.')
            while rclpy.ok():
                begin = time.monotonic()
                rclpy.spin_once(self, timeout_sec=0)
                if camera.error:
                    raise RuntimeError(camera.error)
                frame, stamp, sequence = camera.newest()
                if frame is None or sequence == last_sequence:
                    if begin-waiting_since > self.options['max_frame_age']:
                        self.filter.reset()
                        self.output.publish(String(data='no_face'))
                    if begin-waiting_since > 5.:
                        raise RuntimeError('No fresh webcam image received for 5 seconds')
                    time.sleep(.01)
                    continue
                waiting_since, last_sequence = begin, sequence
                if self.options['mirror']:
                    frame = cv2.flip(frame, 1)
                detections = detector.detect_emotions(frame)
                now = time.monotonic()
                if now-stamp > self.options['max_frame_age']:
                    self.filter.reset()
                    label, score, raw = 'no_face', 0., 'stale_frame'
                    detections = []
                else:
                    label, score, raw = self.filter.update(detections, now)
                self.output.publish(String(data=label))
                self.details.publish(String(data=json.dumps(dict(
                    emotion=label, raw=raw, confidence=round(score, 4), faces=len(detections),
                    inference_ms=round((now-begin)*1000, 1)), ensure_ascii=False)))
                if label != previous:
                    self.get_logger().info(f'{label} | raw={raw}, confidence={score:.2f}')
                    previous = label
                annotated = self.draw(cv2, frame, detections, label, score, raw, now-begin)
                image = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
                image.header.stamp = self.get_clock().now().to_msg()
                self.images.publish(image)
                if self.options['show_window']:
                    cv2.imshow('TurtleBot4 facial expressions', annotated)
                    cv2.waitKey(1)
                    try:
                        visible = cv2.getWindowProperty('TurtleBot4 facial expressions', cv2.WND_PROP_VISIBLE)
                    except cv2.error:
                        visible = 0
                    if visible < 1:
                        break
                time.sleep(max(0., 1/self.options['frame_rate']-(time.monotonic()-begin)))
        finally:
            for _ in range(3):
                self.output.publish(String(data='no_face'))
                time.sleep(.02)
            if camera is not None:
                camera.close()
            if self.options['show_window']:
                cv2.destroyAllWindows()


def main(args=None):
    with stop_on_signals():
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
        node, code = None, 0
        try:
            node = EmotionRecognitionNode()
            node.run()
        except KeyboardInterrupt:
            pass
        except Exception as error:
            print(f'emotion_recognition_node: {error}')
            code = 1
        finally:
            if node is not None:
                node.destroy_node()
            rclpy.try_shutdown()
    return code
