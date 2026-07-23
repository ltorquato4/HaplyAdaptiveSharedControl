#!/usr/bin/env python3

"""Scenario Generator for the Haply shared-control study."""

import random
import secrets
import time
import uuid
from dataclasses import dataclass
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import Point
from haply_msgs.msg import (
    StudyAbortRequest,
    StudyCursor,
    StudyDwellProgress,
    StudySession,
    StudyStartRequest,
    StudyTask,
    StudyTrialState,
)
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from study_orchestration.scenario_logic import (
    StudyPoint,
    WorkspaceBounds,
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
    SCHEMA_VERSION = 2

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
        self.declare_parameter("input_source", "unknown")
        self.declare_parameter("controller_family", "none")
        self.declare_parameter("estimator_state_policy", "persist_session")
        self.declare_parameter("max_control_amplitude", 10.0)
        self.declare_parameter("task_file", "")
        self.declare_parameter("require_controller_ready", False)
        self.declare_parameter("require_estimator_ready", False)
        self.declare_parameter("require_logger_ready", False)
        self.declare_parameter("component_heartbeat_timeout_s", 2.0)
        self.declare_parameter("cursor_max_age_s", 0.5)

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
        self.segments = self._read_yaml_segments() or [
            (self.points[index], self.points[(index + 1) % len(self.points)])
            for index in range(len(self.points))
        ]

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
        self.timeout_policy = (
            str(self.get_parameter("timeout_policy").value).strip().lower()
        )
        if self.timeout_policy not in {"retry", "advance", "end_session"}:
            raise ValueError(
                "timeout_policy must be one of: retry, advance, end_session"
            )
        self.controller_modes = self._parse_modes(
            str(self.get_parameter("controller_modes").value)
        )
        self.repetitions = max(1, int(self.get_parameter("repetitions").value))
        self.loop_tasks = bool(self.get_parameter("loop_tasks").value)
        self.order_strategy = (
            str(self.get_parameter("order_strategy").value).strip().lower()
        )
        if self.order_strategy not in {"fixed", "seeded_random"}:
            raise ValueError("order_strategy must be fixed or seeded_random")
        configured_seed = int(self.get_parameter("order_seed").value)
        self.order_seed = (
            secrets.randbits(32) if configured_seed < 0 else configured_seed
        )
        self._schedule_rng = random.Random(self.order_seed)
        configured_session_id = str(self.get_parameter("session_id").value).strip()
        self.session_id = configured_session_id or str(uuid.uuid4())
        self.input_source = (
            str(self.get_parameter("input_source").value).strip().lower()
        )
        self.controller_family = (
            str(self.get_parameter("controller_family").value).strip().lower()
        )
        self.estimator_state_policy = (
            str(self.get_parameter("estimator_state_policy").value).strip().lower()
        )
        self.max_control_amplitude = float(
            self.get_parameter("max_control_amplitude").value
        )
        if self.input_source not in {"mouse", "haply", "unknown"}:
            raise ValueError("input_source must be mouse, haply, or unknown")
        if self.controller_family not in {"mpc", "state_feedback", "none"}:
            raise ValueError("controller_family must be mpc, state_feedback, or none")
        if self.estimator_state_policy != "persist_session":
            raise ValueError("estimator_state_policy must be persist_session")
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
            raise ValueError("initial_segment_index must be 0 when loop_tasks is false")
        if initial_segment_index:
            self.task_index = next(
                (
                    index
                    for index, task in enumerate(self.tasks)
                    if task.segment_index == initial_segment_index % len(self.segments)
                ),
                0,
            )
        self.is_running = False
        self.trial_id = self.task_index
        self.session_finished = False
        self.component_required = {
            "controller": bool(self.get_parameter("require_controller_ready").value),
            "estimator": bool(self.get_parameter("require_estimator_ready").value),
            "logger": bool(self.get_parameter("require_logger_ready").value),
        }
        self.component_ready = {
            name: not required for name, required in self.component_required.items()
        }
        self.component_last_seen = {
            name: float("-inf") for name in self.component_ready
        }
        self.component_heartbeat_timeout_s = max(
            0.1, float(self.get_parameter("component_heartbeat_timeout_s").value)
        )
        self.cursor_max_age_s = max(
            0.0, float(self.get_parameter("cursor_max_age_s").value)
        )
        self.cursor_position: StudyPoint | None = None
        self.input_valid = False
        self.endpoint_latched = False
        self.endpoint_entered_time: float | None = None
        self.start_gate_reached = False
        self.abort_requested_for_current_trial = False
        self.last_rollout_time = time.monotonic()
        self.rollout_due_time: float | None = None

        retained_state_qos = self._retained_state_qos()
        # LEGACY: split task fields remain as compatibility/debug inputs.
        # Production lifecycle consumers use the typed, ID-bearing messages.
        self.start_pub = self.create_publisher(
            Point, "study_start_point", retained_state_qos
        )
        self.end_pub = self.create_publisher(
            Point, "study_end_point", retained_state_qos
        )
        self.phase_pub = self.create_publisher(
            String, "study_phase", retained_state_qos
        )
        self.mode_pub = self.create_publisher(
            String, "study_controller_mode", retained_state_qos
        )
        self.endpoint_pub = self.create_publisher(Bool, "study_endpoint_reached", 10)
        self.task_pub = self.create_publisher(
            StudyTask, "study_task", retained_state_qos
        )
        self.session_pub = self.create_publisher(
            StudySession, "study_session", retained_state_qos
        )
        self.trial_state_pub = self.create_publisher(
            StudyTrialState, "study_trial_state", retained_state_qos
        )
        self.dwell_progress_pub = self.create_publisher(
            StudyDwellProgress,
            "study_endpoint_dwell_progress",
            retained_state_qos,
        )
        self.system_ready_pub = self.create_publisher(
            Bool, "study_system_ready", retained_state_qos
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
        for component in self.component_ready:
            self.create_subscription(
                Bool,
                f"study_{component}_ready",
                lambda msg, name=component: self._component_ready(name, msg),
                retained_state_qos,
            )

        publish_period_s = 1.0 / max(float(self.get_parameter("publish_hz").value), 0.1)
        self.republish_timer = self.create_timer(publish_period_s, self._tick)
        self._publish_session_definition()
        self._publish_task_definition()
        self._publish_endpoint_state(False)
        self._publish_dwell_progress(0.0)
        self._publish_trial_state("READY")
        self._publish_system_ready()

    def _retained_state_qos(self) -> QoSProfile:
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

    def _read_yaml_segments(self) -> list[tuple[StudyPoint, StudyPoint]]:
        """Load independently defined paths when a researcher supplies YAML."""
        task_file = str(self.get_parameter("task_file").value).strip()
        if not task_file:
            return []
        path = Path(task_file)
        if not path.is_file():
            raise ValueError(f"task_file does not exist: {task_file}")
        with path.open(encoding="utf-8") as stream:
            entries = (yaml.safe_load(stream) or {}).get("paths")
        if not isinstance(entries, list) or not entries:
            raise ValueError("task_file must contain a non-empty paths list")
        segments = []
        for index, entry in enumerate(entries):
            try:
                start = StudyPoint(*map(float, entry["start_point"]))
                end = StudyPoint(*map(float, entry["end_point"]))
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError(f"invalid path entry {index} in {task_file}") from exc
            if not all(
                self.bounds.x_min <= point.x <= self.bounds.x_max
                and self.bounds.y_min <= point.y <= self.bounds.y_max
                for point in (start, end)
            ):
                raise ValueError(f"path entry {index} is outside workspace bounds")
            if (
                (end.x - start.x) ** 2 + (end.y - start.y) ** 2
            ) ** 0.5 < self.min_segment_length:
                raise ValueError(
                    f"path entry {index} is shorter than min_segment_length"
                )
            segments.append((start, end))
        return segments

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
                    segments = list(range(len(self.segments)))
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
        if not all(self.component_ready.values()):
            self._publish_trial_state("READY", "system_not_ready")
            return
        if self.abort_requested_for_current_trial:
            self._publish_trial_state("ABORTED", "abort_requested")
            return
        if self.is_running or not self.input_valid or self.cursor_position is None:
            self._publish_trial_state("READY", "start_rejected")
            return
        start, _ = self._current_segment()
        if not endpoint_reached(self.cursor_position, start, self.start_reached_radius):
            self._publish_trial_state("READY", "start_outside_radius")
            return
        self.is_running = True
        self.start_gate_reached = True
        self.last_rollout_time = time.monotonic()
        self._publish_trial_state("RUNNING")

    def _cursor_position(self, msg: StudyCursor) -> None:
        """Accept only fresh cursor state belonging to the active task."""
        if str(msg.session_id) != self.session_id or int(msg.trial_id) != self.trial_id:
            return
        if not self._cursor_is_fresh(msg):
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
        self._expire_component_heartbeats()
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

        start, end = self._current_segment()
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
        # Cursor samples are task-identified.  Do not carry an accepted sample
        # from the prior task across the new identity while waiting for Mapper.
        self.cursor_position = None
        self.input_valid = False
        self.is_running = False
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
        self._publish_trial_state("READY")

    def _finish_session(self) -> None:
        self.session_finished = True
        self.rollout_due_time = None
        self.endpoint_latched = True
        self.endpoint_entered_time = None
        self.start_gate_reached = False
        self.abort_requested_for_current_trial = False
        self.is_running = False
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
        self._publish_endpoint_state(False)
        self._publish_dwell_progress(0.0)
        self._publish_trial_state("ABORTED", reason)

    def _publish_task_definition(self) -> None:
        task = self._current_task()
        start, end = self._current_segment()
        self.start_pub.publish(self._to_point_msg(start))
        self.end_pub.publish(self._to_point_msg(end))
        self.phase_pub.publish(String(data=task.phase))
        self.mode_pub.publish(String(data=task.controller_mode))
        self.task_pub.publish(self._task_message(task, self.trial_id))

    def _task_message(self, task: ScheduledTask, trial_id: int) -> StudyTask:
        start, end = self.segments[task.segment_index]
        msg = StudyTask()
        msg.session_id = self.session_id
        msg.trial_id = trial_id
        msg.start_point = self._to_point_msg(start)
        msg.end_point = self._to_point_msg(end)
        msg.phase = task.phase
        msg.controller_mode = task.controller_mode
        return msg

    def _publish_session_definition(self) -> None:
        msg = StudySession()
        msg.schema_version = self.SCHEMA_VERSION
        msg.session_id = self.session_id
        msg.input_source = self.input_source
        msg.controller_family = self.controller_family
        msg.order_strategy = self.order_strategy
        msg.order_seed = self.order_seed
        msg.estimator_state_policy = self.estimator_state_policy
        msg.max_control_amplitude = self.max_control_amplitude
        msg.loop_tasks = self.loop_tasks
        msg.schedule = [
            self._task_message(task, trial_id)
            for trial_id, task in enumerate(self.tasks)
        ]
        self.session_pub.publish(msg)

    def _publish_dwell_progress(self, progress: float) -> None:
        msg = StudyDwellProgress()
        msg.session_id = self.session_id
        msg.trial_id = self.trial_id
        msg.progress = max(0.0, min(1.0, float(progress)))
        self.dwell_progress_pub.publish(msg)

    def _publish_endpoint_state(self, reached: bool) -> None:
        self.endpoint_pub.publish(Bool(data=bool(reached)))

    def _component_ready(self, component: str, msg: Bool) -> None:
        if not self.component_required[component]:
            return
        self.component_ready[component] = bool(msg.data)
        if msg.data:
            self.component_last_seen[component] = time.monotonic()
        elif self.is_running:
            self._abort_trial("system_not_ready")
        self._publish_system_ready()

    def _publish_system_ready(self) -> None:
        self.system_ready_pub.publish(Bool(data=all(self.component_ready.values())))

    def _expire_component_heartbeats(self) -> None:
        now = time.monotonic()
        changed = False
        for component, last_seen in self.component_last_seen.items():
            if (
                self.component_required[component]
                and self.component_ready[component]
                and now - last_seen > self.component_heartbeat_timeout_s
            ):
                self.component_ready[component] = False
                changed = True
        if changed:
            if self.is_running:
                self._abort_trial("system_not_ready")
            self._publish_system_ready()

    def _cursor_is_fresh(self, msg: StudyCursor) -> bool:
        stamp_s = float(msg.stamp.sec) + (float(msg.stamp.nanosec) * 1e-9)
        if stamp_s <= 0.0 or self.cursor_max_age_s <= 0.0:
            return True
        return (
            self.get_clock().now().nanoseconds * 1e-9
        ) - stamp_s <= self.cursor_max_age_s

    def _current_segment(self) -> tuple[StudyPoint, StudyPoint]:
        return self.segments[self._current_task().segment_index]

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
