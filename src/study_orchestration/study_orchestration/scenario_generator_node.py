#!/usr/bin/env python3

"""Scenario Generator for the Haply shared-control study."""

import time

import rclpy
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from study_orchestration.scenario_logic import (
    StudyPoint,
    WorkspaceBounds,
    chained_segment,
    endpoint_reached,
    validate_task_points,
)


class ScenarioGenerator(Node):
    """Own phase rollout, task points, controller mode, and endpoint detection."""

    PHASES = ("aggressive", "normal", "careful")

    def __init__(self):
        """Create publishers, subscriptions, and validate the configured task."""
        super().__init__("scenario_generator")

        self.declare_parameter("point_0_x", -0.08)
        self.declare_parameter("point_0_y", -0.08)
        self.declare_parameter("point_0_z", 0.0)
        self.declare_parameter("point_1_x", 0.08)
        self.declare_parameter("point_1_y", 0.08)
        self.declare_parameter("point_1_z", 0.0)
        self.declare_parameter("point_2_x", 0.08)
        self.declare_parameter("point_2_y", -0.08)
        self.declare_parameter("point_2_z", 0.0)
        self.declare_parameter("workspace_x_min", -0.12)
        self.declare_parameter("workspace_x_max", 0.12)
        self.declare_parameter("workspace_y_min", -0.15)
        self.declare_parameter("workspace_y_max", 0.15)
        self.declare_parameter("min_segment_length", 0.10)
        self.declare_parameter("endpoint_reached_radius", 0.01)
        self.declare_parameter("min_phase_duration_s", 0.8)
        self.declare_parameter("inter_trial_delay_s", 3.0)
        self.declare_parameter("publish_hz", 10.0)
        self.declare_parameter("initial_segment_index", 0)
        self.declare_parameter("controller_modes", "adaptive,fixed")

        self.points = self._read_task_points()
        self.bounds = WorkspaceBounds(
            x_min=float(self.get_parameter("workspace_x_min").value),
            x_max=float(self.get_parameter("workspace_x_max").value),
            y_min=float(self.get_parameter("workspace_y_min").value),
            y_max=float(self.get_parameter("workspace_y_max").value),
        )
        self.min_segment_length = float(self.get_parameter("min_segment_length").value)
        validate_task_points(self.points, self.bounds, self.min_segment_length)

        self.endpoint_reached_radius = float(
            self.get_parameter("endpoint_reached_radius").value
        )
        self.min_phase_duration_s = float(
            self.get_parameter("min_phase_duration_s").value
        )
        self.inter_trial_delay_s = max(
            0.0, float(self.get_parameter("inter_trial_delay_s").value)
        )
        self.controller_modes = self._parse_modes(
            str(self.get_parameter("controller_modes").value)
        )
        self.segment_index = int(self.get_parameter("initial_segment_index").value) % 3
        self.phase_index = self.segment_index % len(self.PHASES)
        self.mode_index = self.segment_index % len(self.controller_modes)
        self.is_running = False
        self.cursor_position: StudyPoint | None = None
        self.endpoint_latched = False
        self.start_gate_reached = False
        self.last_rollout_time = time.monotonic()
        self.rollout_due_time: float | None = None

        task_qos = self._task_qos()
        self.start_pub = self.create_publisher(
            Point, "study_start_point", task_qos
        )
        self.end_pub = self.create_publisher(Point, "study_end_point", task_qos)
        self.phase_pub = self.create_publisher(String, "study_phase", task_qos)
        self.mode_pub = self.create_publisher(
            String, "study_controller_mode", task_qos
        )
        self.endpoint_pub = self.create_publisher(Bool, "study_endpoint_reached", 10)

        self.create_subscription(Bool, "study_is_running", self._is_running, 10)
        self.create_subscription(
            Point, "experiment_cursor_position", self._cursor_position, 10
        )

        publish_period_s = 1.0 / max(float(self.get_parameter("publish_hz").value), 0.1)
        self.republish_timer = self.create_timer(publish_period_s, self._tick)
        self._publish_task_definition()
        self._publish_endpoint_state(False)

    def _task_qos(self) -> QoSProfile:
        return QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

    def _read_task_points(self) -> list[StudyPoint]:
        return [
            StudyPoint(
                x=float(self.get_parameter(f"point_{index}_x").value),
                y=float(self.get_parameter(f"point_{index}_y").value),
                z=float(self.get_parameter(f"point_{index}_z").value),
            )
            for index in range(3)
        ]

    def _parse_modes(self, value: str) -> list[str]:
        modes = [mode.strip().lower() for mode in value.split(",")]
        modes = [mode for mode in modes if mode in ("adaptive", "fixed")]
        return modes or ["adaptive", "fixed"]

    def _is_running(self, msg: Bool) -> None:
        was_running = self.is_running
        self.is_running = bool(msg.data)
        if self.is_running and not was_running:
            if not self.endpoint_latched:
                self.start_gate_reached = True
                self.last_rollout_time = time.monotonic()
        elif not self.is_running:
            self.start_gate_reached = False

    def _cursor_position(self, msg: Point) -> None:
        self.cursor_position = StudyPoint(x=float(msg.x), y=float(msg.y), z=0.0)

    def _tick(self) -> None:
        if self._rollout_delay_elapsed():
            self._rollout_next_segment()
            return

        if self.rollout_due_time is not None:
            self._publish_endpoint_state(True)
            return

        reached = self._current_endpoint_reached()
        self._publish_endpoint_state(reached)
        if reached:
            self._schedule_next_segment()

    def _rollout_delay_elapsed(self) -> bool:
        return (
            self.rollout_due_time is not None
            and time.monotonic() >= self.rollout_due_time
        )

    def _current_endpoint_reached(self) -> bool:
        if not self.is_running or self.cursor_position is None or self.endpoint_latched:
            return False

        elapsed_s = time.monotonic() - self.last_rollout_time
        if elapsed_s < self.min_phase_duration_s:
            return False

        start, end = chained_segment(self.points, self.segment_index)
        if not self.start_gate_reached:
            self.start_gate_reached = endpoint_reached(
                self.cursor_position,
                start,
                self.endpoint_reached_radius,
            )
            return False

        return endpoint_reached(
            self.cursor_position,
            end,
            self.endpoint_reached_radius,
        )

    def _rollout_next_segment(self) -> None:
        self.segment_index = (self.segment_index + 1) % len(self.points)
        self.phase_index = (self.phase_index + 1) % len(self.PHASES)
        self.mode_index = (self.mode_index + 1) % len(self.controller_modes)
        self.last_rollout_time = time.monotonic()
        self.rollout_due_time = None
        self.get_logger().info(
            "Scenario rollout: "
            f"segment={self.segment_index}, "
            f"phase={self.PHASES[self.phase_index]}, "
            f"mode={self.controller_modes[self.mode_index]}"
        )
        self.endpoint_latched = False
        self.start_gate_reached = False
        self._publish_task_definition()
        self._publish_endpoint_state(False)

    def _schedule_next_segment(self) -> None:
        self.endpoint_latched = True
        self.rollout_due_time = time.monotonic() + self.inter_trial_delay_s
        self.start_gate_reached = False
        self.get_logger().info(
            "Endpoint reached; waiting "
            f"{self.inter_trial_delay_s:.2f}s before next segment"
        )

    def _publish_task_definition(self) -> None:
        start, end = chained_segment(self.points, self.segment_index)
        self.start_pub.publish(self._to_point_msg(start))
        self.end_pub.publish(self._to_point_msg(end))

        phase_msg = String()
        phase_msg.data = self.PHASES[self.phase_index]
        self.phase_pub.publish(phase_msg)

        mode_msg = String()
        mode_msg.data = self.controller_modes[self.mode_index]
        self.mode_pub.publish(mode_msg)

    def _publish_endpoint_state(self, endpoint_reached_value: bool) -> None:
        reached_msg = Bool()
        reached_msg.data = bool(endpoint_reached_value)
        self.endpoint_pub.publish(reached_msg)

    def _to_point_msg(self, point: StudyPoint) -> Point:
        msg = Point()
        msg.x = point.x
        msg.y = point.y
        msg.z = point.z
        return msg


def main(args=None):
    """Start the Scenario Generator."""
    rclpy.init(args=args)
    node = ScenarioGenerator()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
