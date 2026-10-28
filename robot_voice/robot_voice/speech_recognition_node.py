# SPDX-License-Identifier: Apache-2.0
"""Offline Vietnamese Whisper recognition; publish complete utterances only."""

import io
import json
import math
from pathlib import Path
import queue
import threading
import time
import wave

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from robot_node.runtime import stop_on_signals
from .audio import UtteranceBuffer


class SpeechRecognitionNode(Node):
    def __init__(self):
        super().__init__('speech_recognition_node')
        default = str(Path(get_package_share_directory('robot_voice'))/'models/faster-whisper-small')
        for name, value in [('model_path', default), ('device', ''), ('sample_rate', 0),
                            ('min_confidence', .55), ('max_audio_age', 1.5), ('cpu_threads', 4),
                            ('rms_threshold', .008), ('silence_duration', .8), ('audio_file', ''),
                            ('initial_prompt', 'Robot. Khu vực A, B, C, D. Phòng học, hành lang, phòng thí nghiệm.')]:
            self.declare_parameter(name, value)
        self.options = {name: self.get_parameter(name).value for name in (
            'model_path', 'device', 'sample_rate', 'min_confidence', 'max_audio_age', 'audio_file',
            'cpu_threads', 'rms_threshold', 'silence_duration', 'initial_prompt')}
        if (not math.isfinite(self.options['min_confidence']) or not 0 <= self.options['min_confidence'] <= 1
                or any(not math.isfinite(self.options[k]) or self.options[k] <= 0
                       for k in ('max_audio_age', 'rms_threshold', 'silence_duration'))
                or self.options['sample_rate'] < 0 or self.options['cpu_threads'] <= 0):
            raise ValueError('Invalid audio confidence, timing, sample rate or CPU threads')
        model = Path(self.options['model_path']).expanduser()
        if not (model/'model.bin').is_file():
            raise FileNotFoundError('Missing Whisper model. Run src/robot_voice/scripts/download_model.py first')
        self.text_pub = self.create_publisher(String, '/voice/text', 10)
        self.status_pub = self.create_publisher(String, '/voice/speech_status', 10)
        self.queue = queue.Queue(maxsize=30)
        self.overflow = threading.Event()
        self.processing = threading.Event()

    def status(self, state, **extra):
        self.status_pub.publish(String(data=json.dumps(dict(state=state, **extra), ensure_ascii=False)))

    def transcribe(self, audio):
        self.processing.set()
        self.status('processing')
        try:
            segments, _ = self.model.transcribe(
                audio, language='vi', task='transcribe', beam_size=5, temperature=0.,
                condition_on_previous_text=False, vad_filter=True, word_timestamps=True,
                initial_prompt=self.options['initial_prompt'])
            segments = list(segments)
            text = ' '.join(s.text.strip() for s in segments).strip()
            words = [word for segment in segments for word in (segment.words or [])]
            confidence = sum(word.probability for word in words)/len(words) if words else 0.
            if not text:
                return
            if (not math.isfinite(confidence) or confidence < self.options['min_confidence']
                    or any(s.avg_logprob < -1. for s in segments)):
                self.status('uncertain', text=text, confidence=confidence)
                self.get_logger().warning(f'Chưa đủ tin cậy ({confidence:.2f}), hãy nói lại: {text}')
                return
            self.text_pub.publish(String(data=text))
            self.status('recognized', text=text, confidence=confidence)
            self.get_logger().info(f'HEARD {text} ({confidence:.2f})')
        finally:
            self.processing.clear()

    def callback(self, data, frames, timing, status):
        # The operator waits for listening before the next sentence. Do not
        # accumulate delayed commands while CPU inference is processing one.
        if self.processing.is_set():
            return
        if status:
            self.overflow.set()
        try:
            self.queue.put_nowait((bytes(data), time.monotonic()))
        except queue.Full:
            self.overflow.set()

    def clear_queue(self):
        while True:
            try:
                self.queue.get_nowait()
            except queue.Empty:
                return

    def run(self):
        try:
            from faster_whisper import WhisperModel
            from faster_whisper.audio import decode_audio
        except ImportError as error:
            raise RuntimeError('Install robot_voice/requirements.txt in .venv-voice and pass python_executable') from error
        self.status('loading_model')
        self.model = WhisperModel(str(Path(self.options['model_path']).expanduser()), device='cpu',
                                  compute_type='int8', cpu_threads=self.options['cpu_threads'], local_files_only=True)
        if self.options['audio_file']:
            self.transcribe(str(Path(self.options['audio_file']).expanduser()))
            self.status('file_completed')
            # Let DDS deliver the final message before destroying publishers.
            end = time.monotonic()+.3
            while rclpy.ok() and time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=.02)
            return
        try:
            import sounddevice as sd
        except ImportError as error:
            raise RuntimeError('Install sounddevice in .venv-voice') from error
        device = self.options['device'] or None
        if isinstance(device, str) and device.lstrip('-').isdigit():
            device = int(device)
        rate = self.options['sample_rate'] or int(sd.query_devices(device, 'input')['default_samplerate'])
        sd.check_input_settings(device=device, channels=1, dtype='int16', samplerate=rate)
        buffer = UtteranceBuffer(rate, self.options['rms_threshold'], self.options['silence_duration'])
        with sd.RawInputStream(samplerate=rate, blocksize=max(1, rate//20), device=device,
                               dtype='int16', channels=1, callback=self.callback):
            self.status('listening', sample_rate=rate)
            self.get_logger().info('Microphone ready. Nói một câu, ngừng khoảng 0,8 giây và chờ nhận dạng xong.')
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0)
                if self.overflow.is_set():
                    self.overflow.clear()
                    self.clear_queue()
                    buffer.reset()
                    self.status('audio_overrun', reason='Đã bỏ âm thanh tồn đọng; hãy nói lại')
                    continue
                try:
                    chunk, stamp = self.queue.get(timeout=.1)
                except queue.Empty:
                    continue
                if time.monotonic()-stamp > self.options['max_audio_age']:
                    buffer.reset()
                    continue
                utterance = buffer.feed(chunk)
                if utterance:
                    # PyAV resamples the device's native rate to Whisper 16 kHz.
                    data = io.BytesIO()
                    with wave.open(data, 'wb') as output:
                        output.setnchannels(1); output.setsampwidth(2); output.setframerate(rate)
                        output.writeframes(utterance)
                    data.seek(0)
                    self.transcribe(decode_audio(data, sampling_rate=16000))
                    self.clear_queue()
                    buffer.reset()
                    self.status('listening', sample_rate=rate)


def main(args=None):
    with stop_on_signals():
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
        node, code = None, 0
        try:
            node = SpeechRecognitionNode()
            node.run()
        except KeyboardInterrupt:
            pass
        except Exception as error:
            code = 1
            if node is not None:
                node.status('error', reason=str(error))
            print(f'speech_recognition_node: {error}')
        finally:
            if node is not None:
                node.destroy_node()
            rclpy.try_shutdown()
    return code
