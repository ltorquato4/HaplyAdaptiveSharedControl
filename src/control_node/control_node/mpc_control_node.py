#!/usr/bin/env python3
"""ROS runtime dedicated to timestamped MPC study control."""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Vector3
from haply_msgs.msg import HaplyControl, StudyCursor, StudyTask, StudyTrialState
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64MultiArray, String

from control_node.mpc_controller.adaptive_mpc_controller import (
    AdaptiveMpcController,
)
from control_node.mpc_controller.mpc_controller import MpcController


class MpcControlNode(Node):
    """Apply MPC behavior through the identified study protocol."""

    RUNNING_STATES = {"RUNNING", "DWELL"}
    VALID_MODES = {"fixed", "adaptive"}

    def __init__(self) -> None:
        super().__init__("control_node")
        self._declare_parameters()
        self._configure_logging()
        self._read_parameters()

        self.controller: MpcController | AdaptiveMpcController | None = None
        self.active_task: StudyTask | None = None
        self.session_id = ""
        self.trial_id: int | None = None
        self.controller_mode = ""
        self.running = False
        self.controller_healthy = False
        self.control_iteration = -1
        self.adapt_iteration = -1
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
            StudyCursor,
            "/study_cursor",
            self.study_cursor_callback,
            self._state_qos(),
        )
        self.create_subscription(
            Float64MultiArray,
            "/estimation/K_h",
            self.estimation_callback,
            10,
        )

        self.control_pub = self.create_publisher(Vector3, "/control/U_a", 10)
        self.parameter_pub = self.create_publisher(
            String,
            "/control/K_a",
            retained_state_qos,
        )
        self.force_pub = self.create_publisher(HaplyControl, "/haply_target", 10)
        self.ready_pub = self.create_publisher(
            Bool,
            "/study_controller_ready",
            retained_state_qos,
        )
        self.create_timer(0.5, self._publish_ready)
        self.create_timer(0.02, self._enforce_cursor_timeout)
        self.get_logger().info(
            "MPC controller started; command and force semantics are preserved."
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("log_level", "INFO")
        self.declare_parameter("delta_time", 0.1)
        self.declare_parameter("max_control_amplitude", 10.0)
        self.declare_parameter("max_velocity_amplitude", 10.0)
        self.declare_parameter("acceleration_to_force_factor", 0.2)
        self.declare_parameter("mpc_control_every_i_th_iteration", 1)
        self.declare_parameter("adapt_every_i_th_iterarion", 3)
        self.declare_parameter("prediction_horizon", 5)
        self.declare_parameter("x_bounds", 0.12)
        self.declare_parameter("y_bounds", 0.18)
        self.declare_parameter("cursor_timeout_s", 0.2)
        self.declare_parameter("docking_enabled", True)
        self.declare_parameter("docking_start_percent", 85.0)
        self.declare_parameter("docking_comfort_reduction", 0.9)
        self.declare_parameter("docking_trajectory_weight_scale", 2.0)
        self.declare_parameter("docking_goal_weight_scale", 1000000.0)

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

    def _read_parameters(self) -> None:
        self.model_dt_s = float(self.get_parameter("delta_time").value)
        self.max_control = float(self.get_parameter("max_control_amplitude").value)
        self.max_velocity = float(self.get_parameter("max_velocity_amplitude").value)
        self.force_conversion = float(self.get_parameter("acceleration_to_force_factor").value)
        self.control_every_n = int(self.get_parameter("mpc_control_every_i_th_iteration").value)
        self.adapt_every_n = int(self.get_parameter("adapt_every_i_th_iterarion").value)
        self.prediction_horizon = int(self.get_parameter("prediction_horizon").value)
        self.x_bounds = float(self.get_parameter("x_bounds").value)
        self.y_bounds = float(self.get_parameter("y_bounds").value)
        self.cursor_timeout_s = float(self.get_parameter("cursor_timeout_s").value)
        self.docking_enabled = bool(self.get_parameter("docking_enabled").value)
        self.docking_start_percent = float(self.get_parameter("docking_start_percent").value)
        self.docking_comfort_reduction = float(
            self.get_parameter("docking_comfort_reduction").value
        )
        self.docking_trajectory_weight_scale = float(
            self.get_parameter("docking_trajectory_weight_scale").value
        )
        self.docking_goal_weight_scale = float(
            self.get_parameter("docking_goal_weight_scale").value
        )
        positive = {
            "delta_time": self.model_dt_s,
            "max_control_amplitude": self.max_control,
            "max_velocity_amplitude": self.max_velocity,
            "acceleration_to_force_factor": self.force_conversion,
            "prediction_horizon": self.prediction_horizon,
            "x_bounds": self.x_bounds,
            "y_bounds": self.y_bounds,
            "cursor_timeout_s": self.cursor_timeout_s,
            "mpc_control_every_i_th_iteration": self.control_every_n,
            "adapt_every_i_th_iterarion": self.adapt_every_n,
        }
        invalid = [name for name, value in positive.items() if value <= 0]
        if invalid:
            raise ValueError("MPC parameters must be positive: " + ", ".join(invalid))

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
        ready = self.controller is not None and self.controller_healthy
        self.ready_pub.publish(Bool(data=ready))

    def study_task_callback(self, msg: StudyTask) -> None:
        """Configure the optimizer atomically from one retained task."""
        mode = str(msg.controller_mode).strip().lower()
        points = np.array(
            [
                msg.start_point.x,
                msg.start_point.y,
                msg.end_point.x,
                msg.end_point.y,
            ],
            dtype=float,
        )
        if mode not in self.VALID_MODES:
            self._invalidate_controller(f"Unsupported MPC mode: {mode}")
            return
        if not np.isfinite(points).all():
            self._invalidate_controller("MPC task points must be finite")
            return
        if np.linalg.norm(points[2:] - points[:2]) <= 1e-9:
            self._invalidate_controller("MPC task path must have nonzero length")
            return

        self.active_task = msg
        self.session_id = str(msg.session_id)
        self.trial_id = int(msg.trial_id)
        self.controller_mode = mode
        self.running = False
        self._publish_zero()
        self._build_controller()

    def _build_controller(self) -> bool:
        if self.active_task is None:
            return False
        self._destroy_controller()
        task = self.active_task
        start = (task.start_point.x, task.start_point.y)
        end = (task.end_point.x, task.end_point.y)
        common = {
            "prediction_horizon": self.prediction_horizon,
            "max_control": (self.max_control, self.max_control),
            "max_velocity": (self.max_velocity, self.max_velocity),
            "x_bounds": (-self.x_bounds, self.x_bounds),
            "y_bounds": (-self.y_bounds, self.y_bounds),
        }
        try:
            if self.controller_mode == "adaptive":
                self.controller = AdaptiveMpcController(
                    start,
                    end,
                    self.model_dt_s,
                    **common,
                    docking_enabled=self.docking_enabled,
                    docking_start_percent=self.docking_start_percent,
                    docking_comfort_reduction=self.docking_comfort_reduction,
                    docking_trajectory_weight_scale=(self.docking_trajectory_weight_scale),
                    docking_goal_weight_scale=self.docking_goal_weight_scale,
                )
            else:
                self.controller = MpcController(
                    start,
                    end,
                    self.model_dt_s,
                    **common,
                )
        except Exception as exc:
            self.controller = None
            self.controller_healthy = False
            self.get_logger().error(f"Failed to initialize MPC: {exc}")
            self._publish_ready()
            return False

        self.controller_healthy = True
        self._reset_runtime()
        self._publish_parameters()
        self._publish_ready()
        return True

    def _invalidate_controller(self, reason: str) -> None:
        self.get_logger().error(reason)
        self.active_task = None
        self.session_id = ""
        self.trial_id = None
        self.controller_mode = ""
        self.running = False
        self.controller_healthy = False
        self._publish_zero()
        self._destroy_controller()
        self._publish_ready()

    def study_trial_state_callback(self, msg: StudyTrialState) -> None:
        """Apply only lifecycle transitions for the configured task identity."""
        if (
            self.active_task is None
            or str(msg.session_id) != self.session_id
            or int(msg.trial_id) != self.trial_id
        ):
            return
        should_run = str(msg.state).upper() in self.RUNNING_STATES
        if should_run and not self.running:
            if self.controller is None or not self.controller_healthy:
                if not self._build_controller():
                    return
            self._reset_runtime()
        elif self.running and not should_run:
            self._publish_zero()
            self._reset_runtime()
        self.running = should_run

        if not should_run and not self.controller_healthy:
            self._build_controller()

    def study_cursor_callback(self, msg: StudyCursor) -> None:
        """Compute once for each valid, matching cursor sample."""
        if (
            not self.running
            or self.controller is None
            or not self.controller_healthy
            or str(msg.session_id) != self.session_id
            or int(msg.trial_id) != self.trial_id
        ):
            return
        if not msg.input_valid:
            self._publish_zero()
            self._reset_runtime()
            return

        position = np.array([msg.position.x, msg.position.y], dtype=float)
        timestamp_s = float(msg.stamp.sec) + float(msg.stamp.nanosec) * 1e-9
        if not np.isfinite(position).all() or not np.isfinite(timestamp_s):
            self._publish_zero()
            self._reset_runtime()
            return

        self.last_cursor_receipt_s = self._now_s()
        self.control_iteration += 1
        if self.control_iteration % self.control_every_n != 0:
            return
        try:
            command = self.controller.compute_control(position, timestamp_s)
        except (RuntimeError, ValueError, FloatingPointError) as exc:
            self._handle_controller_failure(exc)
            return
        command_array = np.asarray(command, dtype=float).reshape(2)
        if not np.isfinite(command_array).all():
            self._handle_controller_failure(RuntimeError("MPC produced a non-finite command"))
            return
        self._publish_command(command_array)

    def estimation_callback(self, msg: Float64MultiArray) -> None:
        """Adapt MPC from the four active effective interaction coefficients."""
        if (
            not self.running
            or not self.controller_healthy
            or self.controller_mode != "adaptive"
            or not isinstance(self.controller, AdaptiveMpcController)
        ):
            return
        values = np.asarray(msg.data, dtype=float)
        if values.size < 8 or not np.isfinite(values).all():
            self.get_logger().warning("Ignoring invalid effective K_h")
            return
        self.adapt_iteration += 1
        if self.adapt_iteration % self.adapt_every_n != 0:
            return
        active = [[values[0], values[1]], [values[6], values[7]]]
        self.controller.adapt(active)
        self._publish_parameters()

    def _handle_controller_failure(self, exc: Exception) -> None:
        self.get_logger().error(f"MPC control failure: {exc}")
        self.controller_healthy = False
        self._publish_zero()
        self._publish_ready()

    def _reset_runtime(self) -> None:
        self.control_iteration = -1
        self.adapt_iteration = -1
        self.last_cursor_receipt_s = None
        if self.controller is not None:
            self.controller.reset_runtime_state()

    def _publish_parameters(self) -> None:
        if self.controller is not None:
            self.parameter_pub.publish(String(data=self.controller.publish_control_parameter()))

    def _publish_command(self, command: Sequence[float]) -> None:
        raw = np.asarray(command, dtype=float).reshape(2)
        self.control_pub.publish(Vector3(x=float(raw[0]), y=float(raw[1]), z=0.0))
        force = raw * self.force_conversion
        message = HaplyControl()
        message.use_position = False
        message.target_position = Point()
        message.force = Vector3(
            x=float(force[0]),
            y=0.0,
            z=float(force[1]),
        )
        self.force_pub.publish(message)
        self.force_active = bool(np.any(raw))

    def _publish_zero(self) -> None:
        self._publish_command(np.zeros(2, dtype=float))
        self.force_active = False

    def _enforce_cursor_timeout(self) -> None:
        if not self.running or not self.force_active:
            return
        if (
            self.last_cursor_receipt_s is None
            or self._now_s() - self.last_cursor_receipt_s > self.cursor_timeout_s
        ):
            self.get_logger().warning("Cursor input stale; publishing zero MPC force")
            self._publish_zero()
            self._reset_runtime()

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _destroy_controller(self) -> None:
        if self.controller is not None:
            self.controller.destroy()
            self.controller = None


def main(args=None) -> None:
    """Start the dedicated MPC control node."""
    rclpy.init(args=args)
    node = MpcControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.get_logger().info("Stopping MPC controller")
    finally:
        if rclpy.ok():
            node._publish_zero()
        node._destroy_controller()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
