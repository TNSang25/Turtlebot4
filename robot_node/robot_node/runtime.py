# SPDX-License-Identifier: Apache-2.0
"""Keep ROS alive long enough to publish a stop when a process is interrupted."""

from contextlib import contextmanager
import signal
import time

from geometry_msgs.msg import Twist
import rclpy


@contextmanager
def stop_on_signals():
    """Convert SIGINT/SIGTERM into normal cleanup before rclpy shutdown."""
    interrupted = False

    def interrupt(signum, frame):
        nonlocal interrupted
        # A terminal signal and launch's forwarded signal can arrive together.
        # The first begins cleanup; later signals must not interrupt final zero.
        if not interrupted:
            interrupted = True
            raise KeyboardInterrupt

    previous = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
    for sig in previous:
        signal.signal(sig, interrupt)
    try:
        yield
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)


def publish_stop(node, publisher) -> None:
    """Give the local DDS publisher time to transmit its final zero command."""
    if rclpy.ok(context=node.context):
        for _ in range(3):
            publisher.publish(Twist())
            time.sleep(0.02)
