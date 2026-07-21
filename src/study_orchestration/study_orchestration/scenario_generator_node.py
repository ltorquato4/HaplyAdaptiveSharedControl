#!/usr/bin/env python3

"""Scenario Generator for the Haply shared-control study."""

import random
import secrets
import time
import uuid
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from haply_msgs.msg import (
    StudyAbortRequest,
    StudyDwellProgress,
    StudyCursor,
    StudyStartRequest,
    StudyTask,
    StudyTrialState,
)
from std_msgs.msg import Bool, String

from study_orchestration.scenario_logic import (
    StudyPoint,
    WorkspaceBounds,
    chained_segment,
    endpoint_reached,
    validate_task_points,
)


@dataclass(frozen=True)
class ScheduledTask:
    """One complete behavioral-state, path, and controller-mode condition."""

    segment_index: int
    phase: str
    controller_mode: str


class ScenarioGenerator(Node):
    """Own phase rollout, task points, controller mode, and endpoint detection."""

    PHASES = ("aggressive", "normal", "careful")
    POINT_COUNT = 5

    def __init__(self):
        """Create publishers, subscriptions, and validate the configured task."""
        super().__init__("scenario_generator")

        self.declare_parameter("point_0_x", -0.08)
        self.declare_parameter("point_0_y", -0.08)
        self.declare_parameter("point_0_z", 0.0)
        self.declare_parameter("point_1_x", 0.08)
        self.declare_parameter("point_1_y", -0.08)
        self.declare_parameter("point_1_z", 0.0)
        self.declare_parameter("point_2_x", 0.08)
        self.declare_parameter("point_2_y", 0.08)
        self.declare_parameter("point_2_z", 0.0)
        self.declare_parameter("point_3_x", -0.08)
        self.declare_parameter("point_3_y", 0.08)
        self.declare_parameter("point_3_z", 0.0)
        self.declare_parameter("point_4_x", 0.0)
        self.declare_parameter("point_4_y", -0.15)
        self.declare_parameter("point_4_z", 0.0)
        self.declare_parameter("workspace_x_min", -0.12)
        self.declare_parameter("workspace_x_max", 0.12)
        self.declare_parameter("workspace_y_min", -0.18)
        self.declare_parameter("workspace_y_max", 0.15)
        self.declare_parameter("min_segment_length", 0.10)
        self.declare_parameter("endpoint_reached_radius", 0.01)
        self.declare_parameter("start_reached_radius", 0.01)
        self.declare_parameter("min_phase_duration_s", 0.8)
        self.declare_parameter("endpoint_dwell_s", 1.0)
        self.declare_parameter("inter_trial_delay_s", 1.0)
        self.declare_parameter("max_trial_duration_s", 0.0)
        self.declare_parameter("timeout_policy", "retry")
        self.declare_parameter("publish_hz", 10.0)
        self.declare_parameter("initial_segment_index", 0)
        self.declare_parameter("controller_modes", "adaptive,fixed")
        self.declare_parameter("repetitions", 1)
        self.declare_parameter("loop_tasks", False)
        self.declare_parameter("order_strategy", "seeded_random")
        self.declare_parameter("order_seed", -1)
        self.declare_parameter("session_id", "")

        self.points = self._read_task_points()
        self.bounds = WorkspaceBounds(
            x_min=float(self.get_parameter("workspace_x_min").value),
            x_max=float(self.get_parameter("workspace_x_max").value),
            y_min=float(self.get_parameter("workspace_y_min").value),
            y_max=float(self.get_parameter("workspace_y_max").value),
        )
        self.min_segment_length = float(self.get_parameter("min_segment_length").value)
        validate_task_points(
            self.points,
            self.bounds,
            self.min_segment_length,
            expected_count=self.POINT_COUNT,
        )

        self.endpoint_reached_radius = float(
            self.get_parameter("endpoint_reached_radius").value
        )
        self.start_reached_radius = float(
            self.get_parameter("start_reached_radius").value
        )
        self.min_phase_duration_s = float(
            self.get_parameter("min_phase_duration_s").value
        )
        self.endpoint_dwell_s = max(
            0.0, float(self.get_parameter("endpoint_dwell_s").value)
        )
        self.inter_trial_delay_s = max(
            0.0, float(self.get_parameter("inter_trial_delay_s").value)
        )
        self.max_trial_duration_s = max(
            0.0, float(self.get_parameter("max_trial_duration_s").value)
        )
        self.timeout_policy = str(
            self.get_parameter("timeout_policy").value
        ).strip().lower()
        if self.timeout_policy not in {"retry", "advance", "end_session"}:
            raise ValueError(
                "timeout_policy must be one of: retry, advance, end_session"
            )
        self.controller_modes = self._parse_modes(
            str(self.get_parameter("controller_modes").value)
        )
        self.repetitions = max(1, int(self.get_parameter("repetitions").value))
        self.loop_tasks = bool(self.get_parameter("loop_tasks").value)
        self.order_strategy = str(
            self.get_parameter("order_strategy").value
        ).strip().lower()
        if self.order_strategy not in {"fixed", "seeded_random"}:
            raise ValueError("order_strategy must be fixed or seeded_random")
        configured_seed = int(self.get_parameter("order_seed").value)
        self.order_seed = (
            secrets.randbits(32) if configured_seed < 0 else configured_seed
        )
        self._schedule_rng = random.Random(self.order_seed)
        configured_session_id = str(self.get_parameter("session_id").value).strip()
        self.session_id = configured_session_id or str(uuid.uuid4())
        self.tasks = self._expand_session_tasks()
        self.get_logger().info(
            "Resolved study schedule: "
            f"session={self.session_id}, strategy={self.order_strategy}, "
            f"seed={self.order_seed}, order={self._format_schedule()}"
        )
        self.task_index = 0
        # A bounded study must begin at schedule entry zero so it covers every
        # combination exactly once. Nonzero starts remain a loop-mode/debug aid.
        initial_segment_index = int(self.get_parameter("initial_segment_index").value)
        if initial_segment_index and not self.loop_tasks:
            raise ValueError(
                "initial_segment_index must be 0 when loop_tasks is false"
            )
        if initial_segment_index:
            self.task_index = next(
                (
                    index
                    for index, task in enumerate(self.tasks)
                    if task.segment_index == initial_segment_index % len(self.points)
                ),
                0,
            )
        self.is_running = False
        self.trial_id = self.task_index
        self.session_finished = False
        self.cursor_position: StudyPoint | None = None
        self.input_valid = False
        self.endpoint_latched = False
        self.endpoint_entered_time: float | None = None
        self.start_gate_reached = False
        self.abort_requested_for_current_trial = False
        self.last_rollout_time = time.monotonic()
        self.rollout_due_time: float | None = None

        task_qos = self._task_qos()
        # LEGACY (remove after the branch-51 Controller, Estimator, and Logger
        # migrate to StudyTask/StudyTrialState): these consumers still require
        # split task fields and the running Bool. They are compatibility-only;
        # the GUI uses the typed, ID-bearing messages below.
        self.start_pub = self.create_publisher(Point, "study_start_point", task_qos)
        self.end_pub = self.create_publisher(Point, "study_end_point", task_qos)
        self.phase_pub = self.create_publisher(String, "study_phase", task_qos)
        self.mode_pub = self.create_publisher(String, "study_controller_mode", task_qos)
        self.running_pub = self.create_publisher(Bool, "study_is_running", 10)
        self.endpoint_pub = self.create_publisher(Bool, "study_endpoint_reached", 10)
        self.task_pub = self.create_publisher(StudyTask, "study_task", task_qos)
        self.trial_state_pub = self.create_publisher(
            StudyTrialState, "study_trial_state", task_qos
        )
        self.dwell_progress_pub = self.create_publisher(
            StudyDwellProgress, "study_endpoint_dwell_progress", task_qos
        )

        self.create_subscription(
            StudyStartRequest, "study_start_requested", self._start_requested, 10
        )
        self.create_subscription(
            StudyAbortRequest, "study_abort_requested", self._abort_requested, 10
        )
        self.create_subscription(
            StudyCursor, "study_cursor", self._cursor_position, self._state_qos()
        )

        publish_period_s = 1.0 / max(float(self.get_parameter("publish_hz").value), 0.1)
        self.republish_timer = self.create_timer(publish_period_s, self._tick)
        self._publish_task_definition()
        self._publish_endpoint_state(False)
        self._publish_dwell_progress(0.0)
        self._publish_trial_state("READY")

    def _task_qos(self) -> QoSProfile:
        return QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

    def _state_qos(self) -> QoSProfile:
        return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)

    def _read_task_points(self) -> list[StudyPoint]:
        return [
            StudyPoint(
                x=float(self.get_parameter(f"point_{index}_x").value),
                y=float(self.get_parameter(f"point_{index}_y").value),
                z=float(self.get_parameter(f"point_{index}_z").value),
            )
            for index in range(self.POINT_COUNT)
        ]

    def _parse_modes(self, value: str) -> list[str]:
        modes = [mode.strip().lower() for mode in value.split(",")]
        modes = [mode for mode in modes if mode in ("adaptive", "fixed")]
        return modes or ["adaptive", "fixed"]

    def _expand_session_tasks(self) -> list[ScheduledTask]:
        """Expand every condition with reproducible phase and path ordering."""
        tasks = []
        for _ in range(self.repetitions):
            for mode in self.controller_modes:
                phases = list(self.PHASES)
                if self.order_strategy == "seeded_random":
                    self._schedule_rng.shuffle(phases)
                for phase in phases:
                    segments = list(range(len(self.points)))
                    if self.order_strategy == "seeded_random":
                        self._schedule_rng.shuffle(segments)
                    tasks.extend(
                        ScheduledTask(segment_index, phase, mode)
                        for segment_index in segments
                    )
        return tasks

    def _format_schedule(self) -> str:
        return ",".join(
            f"{task.controller_mode}:{task.phase}:P{task.segment_index}"
            for task in self.tasks
        )

    def _current_task(self) -> ScheduledTask:
        return self.tasks[self.task_index]

    def _start_requested(self, msg: StudyStartRequest) -> None:
        if self.session_finished:
            self._publish_trial_state("SESSION_FINISHED", "session_finished")
            return
        if str(msg.session_id) != self.session_id:
            self._publish_trial_state("READY", "stale_session_id")
            return
        if int(msg.trial_id) != self.trial_id:
            self._publish_trial_state("READY", "stale_trial_id")
            return
        if self.abort_requested_for_current_trial:
            self._publish_trial_state("ABORTED", "abort_requested")
            return
        if self.is_running or not self.input_valid or self.cursor_position is None:
            self._publish_trial_state("READY", "start_rejected")
            return
        start, _ = chained_segment(self.points, self._current_task().segment_index)
        if not endpoint_reached(
            self.cursor_position, start, self.start_reached_radius
        ):
            self._publish_trial_state("READY", "start_outside_radius")
            return
        self.is_running = True
        self.start_gate_reached = True
        self.last_rollout_time = time.monotonic()
        self._publish_running(True)
        self._publish_trial_state("RUNNING")

    def _cursor_position(self, msg: StudyCursor) -> None:
        """Accept only fresh cursor state belonging to the active task."""
        if (
            str(msg.session_id) != self.session_id
            or int(msg.trial_id) != self.trial_id
        ):
            return
        self.input_valid = bool(msg.input_valid)
        if not self.input_valid:
            if self.is_running:
                self._abort_trial("input_lost")
            return
        self.cursor_position = StudyPoint(
            x=float(msg.position.x), y=float(msg.position.y), z=0.0
        )

    def _abort_requested(self, msg: StudyAbortRequest) -> None:
        """Abort only the currently active task requested by the GUI."""
        if (
            self.session_finished
            or str(msg.session_id) != self.session_id
            or int(msg.trial_id) != self.trial_id
        ):
            return
        reason = str(msg.reason).strip() or "abort_requested"
        self.abort_requested_for_current_trial = True
        if self.is_running:
            self._abort_trial(reason)
        else:
            # A GUI can close after sending a start request but before the
            # callback is processed. Latching the abort keeps that queued
            # request from activating the controller during teardown.
            self._publish_trial_state("ABORTED", reason)

    def _tick(self) -> None:
        if self.session_finished:
            return
        if self._rollout_delay_elapsed():
            self._rollout_next_segment()
            return

        if self.rollout_due_time is not None:
            self._publish_endpoint_state(True)
            return

        if self._trial_timed_out():
            self._abort_trial("timeout")
            if self.timeout_policy == "advance":
                self._rollout_next_segment()
            elif self.timeout_policy == "end_session":
                self._finish_session()
            self._publish_endpoint_state(False)
            return

        reached = self._current_endpoint_reached()
        self._publish_endpoint_state(reached)
        self._publish_dwell_progress(self._dwell_progress())
        if reached:
            self._schedule_next_segment()

    def _rollout_delay_elapsed(self) -> bool:
        return (
            self.rollout_due_time is not None
            and time.monotonic() >= self.rollout_due_time
        )

    def _current_endpoint_reached(self) -> bool:
        if (
            not self.is_running
            or not self.input_valid
            or self.cursor_position is None
            or self.endpoint_latched
        ):
            self.endpoint_entered_time = None
            return False

        now = time.monotonic()
        elapsed_s = now - self.last_rollout_time
        if elapsed_s < self.min_phase_duration_s:
            self.endpoint_entered_time = None
            return False

        start, end = chained_segment(self.points, self._current_task().segment_index)
        if not self.start_gate_reached:
            self.start_gate_reached = endpoint_reached(
                self.cursor_position,
                start,
                self.endpoint_reached_radius,
            )
            self.endpoint_entered_time = None
            return False

        inside = endpoint_reached(
            self.cursor_position,
            end,
            self.endpoint_reached_radius,
        )
        if not inside:
            self.endpoint_entered_time = None
            return False
        if self.endpoint_entered_time is None:
            self.endpoint_entered_time = now
            self._publish_trial_state("DWELL")
        return now - self.endpoint_entered_time >= self.endpoint_dwell_s

    def _rollout_next_segment(self) -> None:
        if self.task_index + 1 >= len(self.tasks):
            if not self.loop_tasks:
                self._finish_session()
                return
            self.task_index = 0
        else:
            self.task_index += 1
        self.trial_id += 1
        self.last_rollout_time = time.monotonic()
        self.rollout_due_time = None
        task = self._current_task()
        self.get_logger().info(
            "Scenario rollout: "
            f"trial={self.trial_id}/{len(self.tasks) - 1}, "
            f"segment={task.segment_index}, phase={task.phase}, "
            f"mode={task.controller_mode}"
        )
        self.endpoint_latched = False
        self.endpoint_entered_time = None
        self.start_gate_reached = False
        self.abort_requested_for_current_trial = False
        self._publish_task_definition()
        self._publish_endpoint_state(False)
        self._publish_running(False)
        self._publish_trial_state("READY")

    def _finish_session(self) -> None:
        self.session_finished = True
        self.rollout_due_time = None
        self.endpoint_latched = True
        self.endpoint_entered_time = None
        self.start_gate_reached = False
        self.abort_requested_for_current_trial = False
        self.is_running = False
        self._publish_running(False)
        self._publish_endpoint_state(False)
        self._publish_dwell_progress(0.0)
        self._publish_trial_state("SESSION_FINISHED")
        self.get_logger().info(
            f"Session {self.session_id} finished after {len(self.tasks)} trials"
        )

    def _schedule_next_segment(self) -> None:
        self.endpoint_latched = True
        self.endpoint_entered_time = None
        self.rollout_due_time = time.monotonic() + self.inter_trial_delay_s
        self.start_gate_reached = False
        self.abort_requested_for_current_trial = False
        self.is_running = False
        self._publish_running(False)
        self._publish_trial_state("COMPLETED")
        self.get_logger().info(
            "Endpoint reached; waiting "
            f"{self.inter_trial_delay_s:.2f}s before next segment"
        )

    def _abort_trial(self, reason: str) -> None:
        """Put the active task into a safe, retryable stopped state."""
        self.endpoint_entered_time = None
        self.endpoint_latched = False
        self.start_gate_reached = False
        self.is_running = False
        self._publish_running(False)
        self._publish_endpoint_state(False)
        self._publish_dwell_progress(0.0)
        self._publish_trial_state("ABORTED", reason)

    def _publish_task_definition(self) -> None:
        task = self._current_task()
        start, end = chained_segment(self.points, task.segment_index)
        self.start_pub.publish(self._to_point_msg(start))
        self.end_pub.publish(self._to_point_msg(end))
        self.phase_pub.publish(String(data=task.phase))
        self.mode_pub.publish(String(data=task.controller_mode))
        task_msg = StudyTask()
        task_msg.session_id = self.session_id
        task_msg.trial_id = self.trial_id
        task_msg.start_point = self._to_point_msg(start)
        task_msg.end_point = self._to_point_msg(end)
        task_msg.phase = task.phase
        task_msg.controller_mode = task.controller_mode
        self.task_pub.publish(task_msg)

    def _publish_dwell_progress(self, progress: float) -> None:
        msg = StudyDwellProgress()
        msg.session_id = self.session_id
        msg.trial_id = self.trial_id
        msg.progress = max(0.0, min(1.0, float(progress)))
        self.dwell_progress_pub.publish(msg)

    def _publish_endpoint_state(self, reached: bool) -> None:
        self.endpoint_pub.publish(Bool(data=bool(reached)))

    def _publish_running(self, running: bool) -> None:
        self.running_pub.publish(Bool(data=bool(running)))

    def _publish_trial_state(self, state: str, reason: str = "") -> None:
        msg = StudyTrialState()
        msg.session_id = self.session_id
        msg.trial_id = self.trial_id
        msg.state = state
        msg.reason = reason
        self.trial_state_pub.publish(msg)

    def _trial_timed_out(self) -> bool:
        return (
            self.is_running
            and self.max_trial_duration_s > 0.0
            and time.monotonic() - self.last_rollout_time >= self.max_trial_duration_s
        )

    def _dwell_progress(self) -> float:
        if self.endpoint_entered_time is None:
            return 0.0
        if self.endpoint_dwell_s == 0.0:
            return 1.0
        return (time.monotonic() - self.endpoint_entered_time) / self.endpoint_dwell_s

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
        if rclpy.ok():
            node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
