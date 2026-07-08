#!/usr/bin/env python3

"""Smoke test for the /inverse3_state topic contract."""

import argparse
import math
import threading
import time

import rclpy
from haply_msgs.msg import Inverse3State
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class FakeInverse3StatePublisher(Node):
    """Publishes synthetic Inverse3 state for self-contained topic checks."""

    def __init__(self):
        super().__init__("fake_inverse3_state_publisher")

        self.frequency = 30.0
        self.center_x = 0.0
        self.center_y = -0.13
        self.center_z = 0.0
        self.radius_x = 0.08
        self.radius_y = 0.08
        self.radius_z = 0.02

        self.publisher = self.create_publisher(Inverse3State, "inverse3_state", 10)
        self.start_time = time.monotonic()
        self.timer = self.create_timer(1.0 / self.frequency, self._publish_state)

    def _publish_state(self):
        elapsed = time.monotonic() - self.start_time
        angle = elapsed * 1.2

        msg = Inverse3State()
        msg.position.x = self.center_x + self.radius_x * math.cos(angle)
        msg.position.y = self.center_y + self.radius_y * math.sin(angle)
        msg.position.z = self.center_z + self.radius_z * math.sin(angle * 0.5)

        msg.velocity.x = -self.radius_x * 1.2 * math.sin(angle)
        msg.velocity.y = self.radius_y * 1.2 * math.cos(angle)
        msg.velocity.z = self.radius_z * 0.6 * math.cos(angle * 0.5)

        self.publisher.publish(msg)


class Inverse3StateTopicChecker(Node):
    """Waits for one Inverse3State message."""

    def __init__(self):
        super().__init__("inverse3_state_topic_checker")
        self.message = None
        self.received = threading.Event()
        self.subscription = self.create_subscription(
            Inverse3State, "inverse3_state", self._inverse3_state, 10
        )

    def _inverse3_state(self, msg):
        self.message = msg
        self.received.set()


def _print_message(msg):
    print("Received /inverse3_state:")
    print(
        "  position: "
        f"x={msg.position.x:.4f}, "
        f"y={msg.position.y:.4f}, "
        f"z={msg.position.z:.4f}"
    )
    print(
        "  velocity: "
        f"x={msg.velocity.x:.4f}, "
        f"y={msg.velocity.y:.4f}, "
        f"z={msg.velocity.z:.4f}"
    )


def main(args=None):
    parser = argparse.ArgumentParser(
        description="Wait for one /inverse3_state message."
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=8.0,
        help="Timeout in seconds.",
    )
    parser.add_argument(
        "--fake",
        action="store_true",
        help="Publish synthetic Inverse3State data while checking the topic.",
    )
    parsed_args, ros_args = parser.parse_known_args(args)

    rclpy.init(args=ros_args)
    publisher = FakeInverse3StatePublisher() if parsed_args.fake else None
    node = Inverse3StateTopicChecker()

    timer = node.create_timer(parsed_args.timeout, rclpy.shutdown)
    try:
        while rclpy.ok() and not node.received.is_set():
            if publisher is not None:
                rclpy.spin_once(publisher, timeout_sec=0.02)
            rclpy.spin_once(node, timeout_sec=0.1)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_timer(timer)

    if node.message is None:
        if publisher is not None:
            publisher.destroy_node()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print(f"No /inverse3_state message received within {parsed_args.timeout:.1f}s.")
        return 1

    _print_message(node.message)
    if publisher is not None:
        publisher.destroy_node()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
