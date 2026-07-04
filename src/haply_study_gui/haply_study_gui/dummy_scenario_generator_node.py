#!/usr/bin/env python3

"""Dummy Scenario Generator used for local GUI testing."""

import math
import random
import time

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Bool, String


class DummyScenarioGenerator(Node):
    """Minimal Scenario Generator stand-in for GUI testing."""

    PHASES = ("aggressive", "normal", "careful")

    def __init__(self):
        """Create the dummy scenario publishers and subscriptions."""
        super().__init__("dummy_scenario_generator")

        self.declare_parameter("seed", 7)
        self.declare_parameter("workspace_x_min", -0.10)
        self.declare_parameter("workspace_x_max", 0.10)
        self.declare_parameter("workspace_y_min", -0.25)
        self.declare_parameter("workspace_y_max", -0.03)
        self.declare_parameter("min_segment_length", 0.10)
        self.declare_parameter("min_phase_duration_s", 0.8)
        self.declare_parameter("controller_modes", "adaptive,fixed")

        self.random = random.Random(int(self.get_parameter("seed").value))
        self.workspace = {
            "x_min": float(self.get_parameter("workspace_x_min").value),
            "x_max": float(self.get_parameter("workspace_x_max").value),
            "y_min": float(self.get_parameter("workspace_y_min").value),
            "y_max": float(self.get_parameter("workspace_y_max").value),
        }
        self.min_segment_length = float(self.get_parameter("min_segment_length").value)
        self.min_phase_duration_s = float(
            self.get_parameter("min_phase_duration_s").value
        )
        self.controller_modes = self._parse_modes(
            str(self.get_parameter("controller_modes").value)
        )

        self.phase_index = 0
        self.mode_index = 0
        self.endpoint_was_reached = False
        self.endpoint_armed = True
        self.is_running = False
        self.last_rollout_time = 0.0
        self.start_point = Point()
        self.end_point = Point()
        self.reference_position = Point()

        self.start_pub = self.create_publisher(Point, "study_start_point", 10)
        self.end_pub = self.create_publisher(Point, "study_end_point", 10)
        self.reference_pub = self.create_publisher(Point, "reference_position", 10)
        self.phase_pub = self.create_publisher(String, "study_phase", 10)
        self.mode_pub = self.create_publisher(String, "study_controller_mode", 10)

        self.create_subscription(
            Bool, "study_endpoint_reached", self._endpoint_reached, 10
        )
        self.create_subscription(Bool, "study_is_running", self._is_running, 10)

        self._rollout_phase()
        self.republish_timer = self.create_timer(0.5, self._publish_phase)

    def _parse_modes(self, value):
        modes = [mode.strip().lower() for mode in value.split(",")]
        modes = [mode for mode in modes if mode in ("adaptive", "fixed")]
        return modes or ["adaptive", "fixed"]

    def _endpoint_reached(self, msg):
        reached = bool(msg.data)
        if not reached:
            self.endpoint_was_reached = False
            self.endpoint_armed = True
            return

        if not self.endpoint_armed:
            self.get_logger().debug("Ignoring repeated endpoint reached message")
            return

        if self.endpoint_was_reached:
            return

        if not self._can_rollout():
            self.get_logger().debug(
                "Ignoring endpoint reached while study is not ready"
            )
            return

        self.phase_index = (self.phase_index + 1) % len(self.PHASES)
        self.mode_index = (self.mode_index + 1) % len(self.controller_modes)
        self._rollout_phase()
        self.endpoint_was_reached = True
        self.endpoint_armed = True

    def _can_rollout(self):
        if not self.is_running:
            return False
        elapsed_s = time.monotonic() - self.last_rollout_time
        return elapsed_s >= self.min_phase_duration_s

    def _is_running(self, msg):
        self.is_running = bool(msg.data)

    def _rollout_phase(self):
        self.start_point, self.end_point = self._random_segment()
        self.reference_position = self.end_point
        self.last_rollout_time = time.monotonic()
        self.endpoint_armed = True
        self.get_logger().info(
            "Dummy phase rollout: "
            f"phase={self.PHASES[self.phase_index]}, "
            f"mode={self.controller_modes[self.mode_index]}"
        )
        self._publish_phase()

    def _random_segment(self):
        for _ in range(100):
            start = self._random_point()
            end = self._random_point()
            if self._distance(start, end) >= self.min_segment_length:
                return start, end
        return Point(x=-0.08, y=-0.20, z=0.0), Point(x=0.08, y=-0.08, z=0.0)

    def _random_point(self):
        point = Point()
        point.x = self.random.uniform(self.workspace["x_min"], self.workspace["x_max"])
        point.y = self.random.uniform(self.workspace["y_min"], self.workspace["y_max"])
        point.z = 0.0
        return point

    def _distance(self, first, second):
        dx = float(first.x) - float(second.x)
        dy = float(first.y) - float(second.y)
        return math.sqrt((dx * dx) + (dy * dy))

    def _publish_phase(self):
        phase_msg = String()
        phase_msg.data = self.PHASES[self.phase_index]
        self.phase_pub.publish(phase_msg)

        mode_msg = String()
        mode_msg.data = self.controller_modes[self.mode_index]
        self.mode_pub.publish(mode_msg)

        self.start_pub.publish(self.start_point)
        self.end_pub.publish(self.end_point)
        self.reference_pub.publish(self.reference_position)


def main(args=None):
    """Start the dummy scenario generator."""
    rclpy.init(args=args)
    node = DummyScenarioGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
