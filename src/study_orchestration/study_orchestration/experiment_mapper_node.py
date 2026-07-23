#!/usr/bin/env python3

"""Map raw Haply or mouse-simulated state into experiment task coordinates."""

import math
import time

import rclpy
from geometry_msgs.msg import Point
from haply_msgs.msg import HaplyState, StudyButtonPress, StudyCursor, StudyTask
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from study_orchestration.mapper_logic import (
    AnchoredDeltaMapper,
    MappingConfig,
    TaskPoint,
    map_identity,
    validate_mapping_config,
)


class ExperimentMapper(Node):
    """Publish the task-frame cursor used by GUI, Scenario, Controller, and Logger."""

    VALID_MAPPING_MODES = {"identity", "anchored_delta"}

    def __init__(self):
        """Create mapper subscriptions and publishers."""
        super().__init__("experiment_mapper")

        self.declare_parameter("mapping_mode", "anchored_delta")
        self.declare_parameter("scale_x", 1.0)
        self.declare_parameter("scale_y", 1.0)
        self.declare_parameter("invert_x", False)
        self.declare_parameter("invert_y", False)
        self.declare_parameter("use_z_as_y", True)
        self.declare_parameter("clamp_raw", False)
        self.declare_parameter("raw_x_min", -0.10)
        self.declare_parameter("raw_x_max", 0.10)
        self.declare_parameter("raw_second_min", 0.0)
        self.declare_parameter("raw_second_max", 0.15)
        self.declare_parameter("publish_hz", 100.0)
        self.declare_parameter("task_anchor_x", 0.0)
        self.declare_parameter("task_anchor_y", 0.0)
        self.declare_parameter("button_debounce_s", 0.05)
        self.declare_parameter("input_timeout_s", 0.2)
        self.declare_parameter("workspace_x_min", -0.12)
        self.declare_parameter("workspace_x_max", 0.12)
        self.declare_parameter("workspace_y_min", -0.18)
        self.declare_parameter("workspace_y_max", 0.15)

        self.mapping_mode = (
            str(self.get_parameter("mapping_mode").value).strip().lower()
        )
        if self.mapping_mode not in self.VALID_MAPPING_MODES:
            raise ValueError(
                f"mapping_mode must be one of {sorted(self.VALID_MAPPING_MODES)}"
            )

        config = MappingConfig(
            scale_x=float(self.get_parameter("scale_x").value),
            scale_y=float(self.get_parameter("scale_y").value),
            invert_x=bool(self.get_parameter("invert_x").value),
            invert_y=bool(self.get_parameter("invert_y").value),
            use_z_as_y=bool(self.get_parameter("use_z_as_y").value),
            clamp_raw=bool(self.get_parameter("clamp_raw").value),
            raw_x_min=float(self.get_parameter("raw_x_min").value),
            raw_x_max=float(self.get_parameter("raw_x_max").value),
            raw_second_min=float(self.get_parameter("raw_second_min").value),
            raw_second_max=float(self.get_parameter("raw_second_max").value),
        )
        self.anchored_mapper = AnchoredDeltaMapper(config)
        self.latest_raw_position: TaskPoint | None = None
        self.latest_mapped_position: TaskPoint | None = None
        self.current_session_id: str | None = None
        self.current_trial_id: int | None = None
        self.mapping_ready = False
        self.previous_button_a = False
        self.last_button_edge_time = float("-inf")
        self.button_debounce_s = max(
            0.0, float(self.get_parameter("button_debounce_s").value)
        )
        self.input_timeout_s = max(
            0.0, float(self.get_parameter("input_timeout_s").value)
        )
        self.last_raw_update_time = float("-inf")
        self.input_valid = False
        self.task_anchor = TaskPoint(
            x=float(self.get_parameter("task_anchor_x").value),
            y=float(self.get_parameter("task_anchor_y").value),
            z=0.0,
        )
        validate_mapping_config(
            config,
            self.task_anchor,
            (
                float(self.get_parameter("workspace_x_min").value),
                float(self.get_parameter("workspace_x_max").value),
                float(self.get_parameter("workspace_y_min").value),
                float(self.get_parameter("workspace_y_max").value),
            ),
        )
        self._last_clamp_state = (False, False)

        state_qos = self._state_qos()
        self.cursor_pub = self.create_publisher(
            Point, "experiment_cursor_position", state_qos
        )
        self.study_cursor_pub = self.create_publisher(
            StudyCursor, "study_cursor", state_qos
        )
        retained_state_qos = self._retained_state_qos()
        self.mapping_ready_pub = self.create_publisher(
            Bool, "study_mapping_ready", retained_state_qos
        )
        self.button_pressed_pub = self.create_publisher(
            StudyButtonPress, "study_button_pressed", 10
        )
        self.input_valid_pub = self.create_publisher(
            Bool, "experiment_input_valid", retained_state_qos
        )
        self.create_subscription(
            HaplyState, "haply_state", self._haply_state, state_qos
        )
        self.create_subscription(
            StudyTask, "study_task", self._study_task, retained_state_qos
        )
        publish_hz = max(float(self.get_parameter("publish_hz").value), 1.0)
        self.publish_timer = self.create_timer(
            1.0 / publish_hz,
            self._publish_latest_cursor,
        )
        self._publish_mapping_ready(False)
        self._publish_input_valid(False)

    def _retained_state_qos(self) -> QoSProfile:
        return QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

    def _state_qos(self) -> QoSProfile:
        return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)

    def _haply_state(self, msg: HaplyState) -> None:
        raw_position = self._from_point_msg(msg.position)
        if not all(math.isfinite(value) for value in (raw_position.x, raw_position.y, raw_position.z)):
            self.get_logger().warning("Ignoring non-finite raw input")
            self._publish_input_valid(False)
            return
        self.latest_raw_position = raw_position
        now = time.monotonic()
        self.last_raw_update_time = now
        self._publish_input_valid(True)
        pressed = bool(msg.buttons.a)
        rising_edge = pressed and not self.previous_button_a
        self.previous_button_a = pressed
        if rising_edge:
            self._handle_button_press(now)

    def _study_task(self, msg: StudyTask) -> None:
        """Associate mapped samples with the atomically received task."""
        self.current_session_id = str(msg.session_id)
        self.current_trial_id = int(msg.trial_id)

    def _handle_button_press(self, now: float | None = None) -> None:
        if now is None:
            now = time.monotonic()
        if now - self.last_button_edge_time < self.button_debounce_s:
            return
        self.last_button_edge_time = now

        if not self.mapping_ready:
            if self.latest_raw_position is None:
                return
            if self.mapping_mode == "anchored_delta":
                self.anchored_mapper.capture_anchor(
                    raw_position=self.latest_raw_position,
                    task_start=self.task_anchor,
                )
            self.mapping_ready = True
            self._publish_mapping_ready(True)
            self.get_logger().info(
                "Mapper calibrated: "
                f"raw=({self.latest_raw_position.x:.4f}, {self.latest_raw_position.y:.4f}), "
                f"task=({self.task_anchor.x:.4f}, {self.task_anchor.y:.4f}), "
                f"axes={'x/z' if self.anchored_mapper.config.use_z_as_y else 'x/y'}, "
                f"scale=({self.anchored_mapper.config.scale_x:.3f}, "
                f"{self.anchored_mapper.config.scale_y:.3f})"
            )
            return

        if self.current_session_id is None or self.current_trial_id is None:
            return
        msg = StudyButtonPress()
        msg.session_id = self.current_session_id
        msg.trial_id = self.current_trial_id
        msg.stamp = self.get_clock().now().to_msg()
        self.button_pressed_pub.publish(msg)

    def _publish_mapping_ready(self, ready: bool) -> None:
        msg = Bool()
        msg.data = bool(ready)
        self.mapping_ready_pub.publish(msg)

    def _publish_input_valid(self, valid: bool) -> None:
        valid = bool(valid)
        if valid == self.input_valid and self.last_raw_update_time != float("-inf"):
            return
        self.input_valid = valid
        msg = Bool()
        msg.data = valid
        self.input_valid_pub.publish(msg)
        if not valid:
            self._publish_study_cursor(self.latest_mapped_position, False)

    def _publish_latest_cursor(self) -> None:
        if time.monotonic() - self.last_raw_update_time > self.input_timeout_s:
            self._publish_input_valid(False)
        if (
            not self.input_valid
            or not self.mapping_ready
            or self.latest_raw_position is None
        ):
            return

        if self.mapping_mode == "identity":
            self._publish_cursor(map_identity(self.latest_raw_position))
            return

        mapped_position = self.anchored_mapper.map_position(self.latest_raw_position)
        if mapped_position is not None:
            clamp_state = (
                self.anchored_mapper.last_clamped_x,
                self.anchored_mapper.last_clamped_second,
            )
            if clamp_state != getattr(self, "_last_clamp_state", (False, False)):
                self._last_clamp_state = clamp_state
                if any(clamp_state):
                    self.get_logger().warning(
                        "Mapper raw input saturated: "
                        f"x={clamp_state[0]}, second_axis={clamp_state[1]}"
                    )
            self._publish_cursor(mapped_position)

    def _publish_cursor(self, position: TaskPoint) -> None:
        self.latest_mapped_position = position
        msg = Point()
        msg.x = position.x
        msg.y = position.y
        msg.z = position.z
        self.cursor_pub.publish(msg)
        self._publish_study_cursor(position, True)

    def _publish_study_cursor(
        self, position: TaskPoint | None, input_valid: bool
    ) -> None:
        """Publish a timestamped, task-identified cursor state when known."""
        if self.current_session_id is None or self.current_trial_id is None:
            return
        msg = StudyCursor()
        msg.session_id = self.current_session_id
        msg.trial_id = self.current_trial_id
        msg.stamp = self.get_clock().now().to_msg()
        if position is not None:
            msg.position.x = position.x
            msg.position.y = position.y
            msg.position.z = position.z
        msg.input_valid = bool(input_valid)
        self.study_cursor_pub.publish(msg)

    def _from_point_msg(self, msg: Point) -> TaskPoint:
        return TaskPoint(x=float(msg.x), y=float(msg.y), z=float(msg.z))

def main(args=None):
    """Start the Experiment Mapper."""
    rclpy.init(args=args)
    node = ExperimentMapper()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        if rclpy.ok():
            node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
