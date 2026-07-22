#!/usr/bin/env python3
"""ROS runtime dedicated to timestamped virtual-fixture state feedback."""

from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Vector3
from haply_msgs.msg import HaplyControl, StudyCursor, StudyTask, StudyTrialState
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64MultiArray, String

from control_node.state_feedback_controller.virtual_fixture_controller import (
    StateFeedbackForceConfig,
    VirtualFixtureStateFeedbackController,
)


class StateFeedbackControlNode(Node):
    """Apply bounded Cartesian force once per valid, identified cursor sample."""

    RUNNING_STATES = {"RUNNING", "DWELL"}

    def __init__(self) -> None:
        super().__init__("control_node")
        self._declare_parameters()
        self._configure_logging()
        self.force_config = self._force_config()
        self.cursor_timeout_s = float(self.get_parameter("cursor_timeout_s").value)
        if self.cursor_timeout_s <= 0.0:
            raise ValueError("cursor_timeout_s must be positive")

        self.controller: VirtualFixtureStateFeedbackController | None = None
        self.session_id = ""
        self.trial_id: int | None = None
        self.controller_mode = ""
        self.running = False
        self.last_cursor_receipt_s: float | None = None
        self.force_active = False

        retained_state_qos = self._retained_state_qos()
        self.create_subscription(
            StudyTask,
            "/study_task",
            self.study_task_callback,
            retained_state_qos,
        )
        self.create_subscription(
            StudyTrialState,
            "/study_trial_state",
            self.study_trial_state_callback,
            retained_state_qos,
        )
        self.create_subscription(
            StudyCursor, "/study_cursor", self.study_cursor_callback, self._state_qos()
        )
        self.create_subscription(
            Float64MultiArray, "/estimation/K_h", self.estimation_callback, 10
        )

        self.force_pub = self.create_publisher(HaplyControl, "/haply_target", 10)
        self.control_pub = self.create_publisher(Vector3, "/control/U_a", 10)
        self.parameter_pub = self.create_publisher(
            String, "/control/K_a", retained_state_qos
        )
        self.ready_pub = self.create_publisher(
            Bool, "/study_controller_ready", retained_state_qos
        )
        self.create_timer(0.5, self._publish_ready)
        self.create_timer(0.02, self._enforce_cursor_timeout)
        self.get_logger().info(
            "State-feedback force controller started; output is Cartesian newtons."
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("log_level", "INFO")
        self.declare_parameter("along_stiffness_n_per_m", 10.0)
        self.declare_parameter("along_damping_ns_per_m", 2.0)
        self.declare_parameter("fixture_stiffness_n_per_m", 20.0)
        self.declare_parameter("fixture_damping_ns_per_m", 2.0)
        self.declare_parameter("max_force_n", 2.0)
        self.declare_parameter("velocity_filter_alpha", 0.25)
        self.declare_parameter("cursor_timeout_s", 0.2)
        self.declare_parameter("docking_enabled", False)
        self.declare_parameter("docking_start_percent", 85.0)
        self.declare_parameter("docking_stiffness_scale", 2.0)
        self.declare_parameter("docking_max_cross_track_m", 0.02)
        self.declare_parameter("adaptation_normalization", 50.0)
        self.declare_parameter("adaptation_strength", 0.7)

    def _configure_logging(self) -> None:
        levels = {
            "DEBUG": LoggingSeverity.DEBUG,
            "INFO": LoggingSeverity.INFO,
            "WARN": LoggingSeverity.WARN,
            "WARNING": LoggingSeverity.WARN,
            "ERROR": LoggingSeverity.ERROR,
            "FATAL": LoggingSeverity.FATAL,
        }
        name = str(self.get_parameter("log_level").value).upper()
        self.get_logger().set_level(levels.get(name, LoggingSeverity.INFO))

    def _force_config(self) -> StateFeedbackForceConfig:
        return StateFeedbackForceConfig(
            along_stiffness_n_per_m=float(
                self.get_parameter("along_stiffness_n_per_m").value
            ),
            along_damping_ns_per_m=float(
                self.get_parameter("along_damping_ns_per_m").value
            ),
            fixture_stiffness_n_per_m=float(
                self.get_parameter("fixture_stiffness_n_per_m").value
            ),
            fixture_damping_ns_per_m=float(
                self.get_parameter("fixture_damping_ns_per_m").value
            ),
            max_force_n=float(self.get_parameter("max_force_n").value),
            velocity_filter_alpha=float(
                self.get_parameter("velocity_filter_alpha").value
            ),
            docking_enabled=bool(self.get_parameter("docking_enabled").value),
            docking_start_percent=float(
                self.get_parameter("docking_start_percent").value
            ),
            docking_stiffness_scale=float(
                self.get_parameter("docking_stiffness_scale").value
            ),
            docking_max_cross_track_m=float(
                self.get_parameter("docking_max_cross_track_m").value
            ),
            adaptation_normalization=float(
                self.get_parameter("adaptation_normalization").value
            ),
            adaptation_strength=float(self.get_parameter("adaptation_strength").value),
        )

    @staticmethod
    def _retained_state_qos() -> QoSProfile:
        return QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

    @staticmethod
    def _state_qos() -> QoSProfile:
        return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)

    def _publish_ready(self) -> None:
        self.ready_pub.publish(Bool(data=self.controller is not None))

    def study_task_callback(self, msg: StudyTask) -> None:
        mode = str(msg.controller_mode).strip().lower()
        if mode not in {"fixed", "adaptive"}:
            self.get_logger().error(f"Unsupported state-feedback mode: {mode}")
            self.controller = None
            self._publish_zero_force()
            return
        try:
            self.controller = VirtualFixtureStateFeedbackController(
                (msg.start_point.x, msg.start_point.y),
                (msg.end_point.x, msg.end_point.y),
                config=self.force_config,
                adaptive=mode == "adaptive",
            )
        except ValueError as exc:
            self.controller = None
            self.get_logger().error(f"Invalid state-feedback task: {exc}")
            self._publish_zero_force()
            return
        self.session_id = str(msg.session_id)
        self.trial_id = int(msg.trial_id)
        self.controller_mode = mode
        self.running = False
        self.last_cursor_receipt_s = None
        self._publish_zero_force()
        self._publish_parameters()

    def study_trial_state_callback(self, msg: StudyTrialState) -> None:
        if (
            self.controller is None
            or str(msg.session_id) != self.session_id
            or int(msg.trial_id) != self.trial_id
        ):
            return
        should_run = str(msg.state).upper() in self.RUNNING_STATES
        if should_run and not self.running:
            self.controller.reset_kinematics()
            self.last_cursor_receipt_s = None
        elif self.running and not should_run:
            self._publish_zero_force()
            self.controller.reset_kinematics()
        self.running = should_run

    def study_cursor_callback(self, msg: StudyCursor) -> None:
        if (
            not self.running
            or self.controller is None
            or str(msg.session_id) != self.session_id
            or int(msg.trial_id) != self.trial_id
        ):
            return
        if not msg.input_valid:
            self._publish_zero_force()
            self.controller.reset_kinematics()
            return
        timestamp_s = float(msg.stamp.sec) + float(msg.stamp.nanosec) * 1e-9
        try:
            force = self.controller.compute_force(
                (msg.position.x, msg.position.y), timestamp_s
            )
        except ValueError as exc:
            self.get_logger().warning(f"Rejected cursor/control sample: {exc}")
            self._publish_zero_force()
            self.controller.reset_kinematics()
            return
        self.last_cursor_receipt_s = self._now_s()
        self._publish_force(force)

    def estimation_callback(self, msg: Float64MultiArray) -> None:
        if self.controller is None or not self.controller.adaptive:
            return
        values = np.asarray(msg.data, dtype=float)
        if values.size < 8:
            self.get_logger().warning("Ignoring K_h with fewer than eight entries")
            return
        active = [values[0], values[1], values[6], values[7]]
        try:
            self.controller.update_effective_interaction_model(active)
        except ValueError as exc:
            self.get_logger().warning(f"Ignoring invalid effective K_h: {exc}")
            return
        self._publish_parameters()

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _enforce_cursor_timeout(self) -> None:
        if not self.running or not self.force_active:
            return
        if (
            self.last_cursor_receipt_s is None
            or self._now_s() - self.last_cursor_receipt_s > self.cursor_timeout_s
        ):
            self.get_logger().warning("Cursor input stale; publishing zero force")
            self._publish_zero_force()
            if self.controller is not None:
                self.controller.reset_kinematics()

    def _publish_parameters(self) -> None:
        if self.controller is not None:
            self.parameter_pub.publish(String(data=self.controller.parameter_json()))

    def _publish_force(self, force: np.ndarray) -> None:
        task_force = Vector3(x=float(force[0]), y=float(force[1]), z=0.0)
        self.control_pub.publish(task_force)
        command = HaplyControl()
        command.use_position = False
        command.target_position = Point()
        command.force = Vector3(x=float(force[0]), y=0.0, z=float(force[1]))
        self.force_pub.publish(command)
        self.force_active = True

    def _publish_zero_force(self) -> None:
        zero = np.zeros(2, dtype=float)
        self._publish_force(zero)
        self.force_active = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StateFeedbackControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.get_logger().info("Stopping state-feedback force controller")
    finally:
        # Launch may have invalidated the ROS context before spin returns from
        # SIGINT. Trial-stop callbacks already publish zero while ROS is live;
        # only perform this final best-effort stop while publishing is valid.
        if rclpy.ok():
            node._publish_zero_force()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
