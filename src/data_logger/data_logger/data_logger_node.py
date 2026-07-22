#!/usr/bin/env python3

import csv
import json
import os
import re
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Point, Vector3
from haply_msgs.msg import (
    HaplyState,
    StudyCursor,
    StudySession,
    StudyTask,
    StudyTrialState,
)
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64MultiArray, String

from .csv_logger import CSVLogger


@dataclass
class LoggerConfig:
    log_rate_hz: float = 100.0
    flush_interval: int = 100
    file_prefix: str = "trajectory"


class DataLoggerNode(Node):
    """Record task samples and session/attempt metadata."""

    ATTEMPT_FIELDS = [
        "session_id",
        "trial_id",
        "attempt_id",
        "filename",
        "start_timestamp",
        "end_timestamp",
        "outcome",
        "reason",
    ]

    def __init__(self):
        super().__init__("data_logger_node")
        self.declare_parameter("save_directory", "./logs")
        self.declare_parameter("log_level", "info")
        self.base_directory = str(self.get_parameter("save_directory").value)
        self.log_level = str(self.get_parameter("log_level").value)
        self.get_logger().set_level(self._resolve_log_level(self.log_level))

        self.config = LoggerConfig()
        self.save_directory = None
        self.csv_logger = None
        self.session_metadata = {}
        self.trial_metadata = {}
        self.latest_sample = {}
        self.controller_parameters = None
        self.session_received = False
        self.task_received = False
        self.pending_task = None
        self.trial_active = False
        self.pending_trial_state = None
        self.recording = False
        self.flush_counter = 0
        self.next_write_deadline = None
        self.missed_cycle_count = 0
        self.cursor_sample_sequence = 0
        self.typed_cursor_received = False
        self.attempt_counts = {}
        self.active_attempt = None

        self.setup_subscribers()
        self.ready_pub = self.create_publisher(
            Bool, "/study_logger_ready", self._retained_state_qos()
        )
        self.ready_timer = self.create_timer(0.5, self._publish_ready)
        self.timer = self.create_timer(1.0 / self.config.log_rate_hz, self.write_row)
        self.get_logger().info(
            "Data logger waiting for retained /study_session metadata."
        )

    @staticmethod
    def _retained_state_qos():
        return QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

    @staticmethod
    def _state_qos():
        return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)

    def _publish_ready(self):
        self.ready_pub.publish(
            Bool(
                data=(
                    self.session_received
                    and self.task_received
                    and self.csv_logger is not None
                )
            )
        )

    def _resolve_log_level(self, log_level_name):
        levels = {
            "debug": LoggingSeverity.DEBUG,
            "info": LoggingSeverity.INFO,
            "warn": LoggingSeverity.WARN,
            "warning": LoggingSeverity.WARN,
            "error": LoggingSeverity.ERROR,
            "fatal": LoggingSeverity.FATAL,
        }
        return levels.get(str(log_level_name).strip().lower(), LoggingSeverity.INFO)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _monotonic_now():
        return time.monotonic()

    @staticmethod
    def _safe_session_directory(session_id):
        safe = re.sub(r"[^A-Za-z0-9_.-]", "_", str(session_id))
        return safe or "unknown-session"

    @staticmethod
    def _point_dict(point):
        return {"x": point.x, "y": point.y, "z": point.z}

    def _task_dict(self, task):
        return {
            "session_id": str(task.session_id),
            "trial_id": int(task.trial_id),
            "start_point": self._point_dict(task.start_point),
            "end_point": self._point_dict(task.end_point),
            "phase": str(task.phase),
            "controller_mode": str(task.controller_mode),
        }

    def study_session_callback(self, msg: StudySession):
        session_id = str(msg.session_id)
        if self.session_received and session_id == self.session_metadata["session_id"]:
            return
        if self.recording:
            self.stop_recording()
        self._finalize_attempt("SESSION_CHANGED", "new_session")

        self.session_metadata = {
            "schema_version": int(msg.schema_version),
            "session_id": session_id,
            "input_source": str(msg.input_source),
            "controller_family": str(msg.controller_family),
            "order_strategy": str(msg.order_strategy),
            "order_seed": int(msg.order_seed),
            "estimator_state_policy": str(msg.estimator_state_policy),
            "max_control_amplitude": float(msg.max_control_amplitude),
            "loop_tasks": bool(msg.loop_tasks),
            "schedule": [self._task_dict(task) for task in msg.schedule],
        }
        session_directory = self._safe_session_directory(session_id)
        self.save_directory = os.path.join(self.base_directory, session_directory)
        os.makedirs(self.save_directory, exist_ok=True)
        self.csv_logger = CSVLogger(
            self.save_directory, self.config.file_prefix, self.fieldnames()
        )
        manifest_path = os.path.join(self.save_directory, "session_manifest.json")
        with open(manifest_path, "w", encoding="utf-8") as stream:
            json.dump(self.session_metadata, stream, indent=2, sort_keys=True)
            stream.write("\n")
        self.session_received = True
        self.task_received = False
        self.trial_active = False
        self.typed_cursor_received = False
        self.cursor_sample_sequence = 0
        self.attempt_counts = {}
        self.get_logger().info(f"Session log directory: {self.save_directory}")

        if (
            self.pending_task is not None
            and str(self.pending_task.session_id) == session_id
        ):
            pending = self.pending_task
            self.pending_task = None
            self._apply_task(pending)

    def study_task_callback(self, msg: StudyTask):
        if not self.session_received or str(
            msg.session_id
        ) != self.session_metadata.get("session_id"):
            self.pending_task = msg
            return
        self._apply_task(msg)

    def _apply_task(self, msg):
        self.trial_metadata = {
            "session_id": str(msg.session_id),
            "trial_id": int(msg.trial_id),
            "study_phase": str(msg.phase),
            "study_controller_mode": str(msg.controller_mode),
            "start": msg.start_point,
            "end": msg.end_point,
        }
        self.task_received = True
        self.typed_cursor_received = False
        self.cursor_sample_sequence = 0
        self.latest_sample.pop("cursor", None)
        self.latest_sample.pop("cursor_timestamp", None)
        self.latest_sample.pop("cursor_sample_sequence", None)
        self._apply_pending_trial_state()

    def setup_subscribers(self):
        subscriptions = [
            (
                StudySession,
                "/study_session",
                self.study_session_callback,
                self._retained_state_qos(),
            ),
            (
                StudyTask,
                "/study_task",
                self.study_task_callback,
                self._retained_state_qos(),
            ),
            (
                StudyTrialState,
                "/study_trial_state",
                self.trial_state_callback,
                self._retained_state_qos(),
            ),
            (String, "/study_phase", self.phase_callback, 10),
            (String, "/study_controller_mode", self.mode_callback, 10),
            (Point, "/study_start_point", self.start_point_callback, 10),
            (Point, "/study_end_point", self.end_point_callback, 10),
            (Point, "/experiment_cursor_position", self.cursor_callback, 10),
            (
                StudyCursor,
                "/study_cursor",
                self.study_cursor_callback,
                self._state_qos(),
            ),
            (HaplyState, "/haply_state", self.haply_callback, 10),
            (Float64MultiArray, "/estimation/K_h", self.kh_callback, 10),
            (Vector3, "/estimation/u_h", self.uh_callback, 10),
            (
                String,
                "/control/K_a",
                self.ka_callback,
                self._retained_state_qos(),
            ),
            (Vector3, "/control/U_a", self.ua_callback, 10),
            (Bool, "/study_endpoint_reached", self.endpoint_callback, 10),
        ]
        for message_type, topic, callback, qos in subscriptions:
            self.create_subscription(message_type, topic, callback, qos)

    def fieldnames(self):
        return [
            "schema_version",
            "session_id",
            "trial_id",
            "attempt_id",
            "input_source",
            "controller_family",
            "order_strategy",
            "order_seed",
            "estimator_state_policy",
            "max_control_amplitude",
            "timestamp",
            "monotonic_timestamp",
            "missed_cycle_count",
            "cursor_timestamp",
            "cursor_sample_sequence",
            "study_running",
            "study_phase",
            "study_controller_mode",
            "start_x",
            "start_y",
            "start_z",
            "end_x",
            "end_y",
            "end_z",
            "cursor_x",
            "cursor_y",
            "cursor_z",
            "haply_pos_x",
            "haply_pos_y",
            "haply_pos_z",
            "haply_vel_x",
            "haply_vel_y",
            "haply_vel_z",
            "K_h",
            "u_h",
            "K_a",
            "u_a",
            "endpoint_reached",
        ]

    def start_recording(self):
        if self.recording:
            return
        if not (self.session_received and self.task_received and self.csv_logger):
            self.get_logger().error("Cannot record before session and task metadata.")
            return
        self._finalize_attempt("STOPPED", "superseded")
        trial_id = int(self.trial_metadata["trial_id"])
        attempt_id = self.attempt_counts.get(trial_id, 0) + 1
        self.attempt_counts[trial_id] = attempt_id
        filename = f"trial_{trial_id:06d}_attempt_{attempt_id:03d}.csv"
        _, filepath = self.csv_logger.start(filename)
        self.latest_sample = {}
        self.flush_counter = 0
        self.next_write_deadline = self._monotonic_now() + (
            1.0 / self.config.log_rate_hz
        )
        self.missed_cycle_count = 0
        self.active_attempt = {
            "session_id": self.trial_metadata["session_id"],
            "trial_id": trial_id,
            "attempt_id": attempt_id,
            "filename": filename,
            "start_timestamp": self._now(),
            "end_timestamp": "",
            "outcome": "",
            "reason": "",
        }
        self.recording = True
        self.get_logger().info(f"Recording trial attempt: {filepath}")

    def stop_recording(self):
        if not self.recording:
            return
        self.csv_logger.stop()
        self.recording = False
        if self.active_attempt is not None:
            self.active_attempt["end_timestamp"] = self._now()
        self.latest_sample = {}
        self.next_write_deadline = None

    def _finalize_attempt(self, outcome, reason=""):
        if self.active_attempt is None:
            return
        if self.recording:
            self.stop_recording()
        if not self.active_attempt["end_timestamp"]:
            self.active_attempt["end_timestamp"] = self._now()
        self.active_attempt["outcome"] = str(outcome)
        self.active_attempt["reason"] = str(reason)
        path = os.path.join(self.save_directory, "trial_attempts.csv")
        write_header = not os.path.exists(path)
        with open(path, "a", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=self.ATTEMPT_FIELDS)
            if write_header:
                writer.writeheader()
            writer.writerow(self.active_attempt)
        self.active_attempt = None

    def trial_state_callback(self, msg):
        """Drive retry-aware recording from the identified Scenario state."""
        self.pending_trial_state = msg
        self._apply_pending_trial_state()

    def _apply_pending_trial_state(self):
        msg = self.pending_trial_state
        if msg is None or not self.session_received or not self.task_received:
            return
        if (
            str(msg.session_id) != self.trial_metadata.get("session_id")
            or int(msg.trial_id) != self.trial_metadata.get("trial_id")
        ):
            if (
                str(msg.session_id) == self.trial_metadata.get("session_id")
                and int(msg.trial_id) < self.trial_metadata.get("trial_id", 0)
            ):
                self.pending_trial_state = None
            return

        state = str(msg.state).upper()
        active = state in {"RUNNING", "DWELL"}
        if active:
            if not self.trial_active:
                self.trial_active = True
                self.start_recording()
        else:
            self.trial_active = False
            if state in {"COMPLETED", "ABORTED"}:
                self._finalize_attempt(state, msg.reason)
            elif state == "SESSION_FINISHED":
                self._finalize_attempt(state, msg.reason or "session_finished")
            elif state == "READY" and self.active_attempt is not None:
                self._finalize_attempt("STOPPED", msg.reason or "ready")
        self.pending_trial_state = None

    def phase_callback(self, msg):
        if not self.task_received:
            self.trial_metadata["study_phase"] = msg.data

    def mode_callback(self, msg):
        if not self.task_received:
            self.trial_metadata["study_controller_mode"] = msg.data

    def start_point_callback(self, msg):
        if not self.task_received:
            self.trial_metadata["start"] = msg

    def end_point_callback(self, msg):
        if not self.task_received:
            self.trial_metadata["end"] = msg

    def cursor_callback(self, msg):
        if self.typed_cursor_received:
            return
        self.latest_sample["cursor"] = msg

    def study_cursor_callback(self, msg):
        if (
            not self.task_received
            or str(msg.session_id) != self.trial_metadata.get("session_id")
            or int(msg.trial_id) != self.trial_metadata.get("trial_id")
        ):
            return
        self.typed_cursor_received = True
        stamp = float(msg.stamp.sec) + float(msg.stamp.nanosec) * 1e-9
        if not msg.input_valid:
            self.latest_sample.pop("cursor", None)
            self.latest_sample.pop("cursor_timestamp", None)
            self.latest_sample.pop("cursor_sample_sequence", None)
            return
        self.cursor_sample_sequence += 1
        self.latest_sample["cursor"] = msg.position
        self.latest_sample["cursor_timestamp"] = stamp if stamp > 0.0 else None
        self.latest_sample["cursor_sample_sequence"] = self.cursor_sample_sequence

    def haply_callback(self, msg):
        self.latest_sample["haply"] = msg

    def kh_callback(self, msg):
        self.latest_sample["K_h"] = msg.data

    def uh_callback(self, msg):
        self.latest_sample["u_h"] = msg

    def ka_callback(self, msg):
        self.controller_parameters = msg.data

    def ua_callback(self, msg):
        self.latest_sample["u_a"] = msg

    def endpoint_callback(self, msg):
        self.latest_sample["endpoint_reached"] = msg.data

    def write_row(self):
        if not self.recording or not self.trial_active:
            return
        monotonic_now = self._monotonic_now()
        if self.next_write_deadline is not None:
            period = 1.0 / self.config.log_rate_hz
            lateness = monotonic_now - self.next_write_deadline
            missed_now = max(0, int(lateness // period))
            self.missed_cycle_count += missed_now
            self.next_write_deadline += (missed_now + 1) * period
        metadata = self.session_metadata
        row = {
            "schema_version": metadata["schema_version"],
            "session_id": self.trial_metadata["session_id"],
            "trial_id": self.trial_metadata["trial_id"],
            "attempt_id": self.active_attempt["attempt_id"],
            "input_source": metadata["input_source"],
            "controller_family": metadata["controller_family"],
            "order_strategy": metadata["order_strategy"],
            "order_seed": metadata["order_seed"],
            "estimator_state_policy": metadata["estimator_state_policy"],
            "max_control_amplitude": metadata["max_control_amplitude"],
            "timestamp": self._now(),
            "monotonic_timestamp": monotonic_now,
            "missed_cycle_count": self.missed_cycle_count,
            "cursor_timestamp": self.latest_sample.get("cursor_timestamp"),
            "cursor_sample_sequence": self.latest_sample.get(
                "cursor_sample_sequence"
            ),
            "study_running": True,
            "study_phase": self.trial_metadata["study_phase"],
            "study_controller_mode": self.trial_metadata["study_controller_mode"],
            "K_a": self.controller_parameters,
            "endpoint_reached": self.latest_sample.get("endpoint_reached"),
        }
        for prefix, point in (
            ("start", self.trial_metadata.get("start")),
            ("end", self.trial_metadata.get("end")),
            ("cursor", self.latest_sample.get("cursor")),
        ):
            if point is not None:
                row[f"{prefix}_x"] = point.x
                row[f"{prefix}_y"] = point.y
                row[f"{prefix}_z"] = point.z
        haply = self.latest_sample.get("haply")
        if haply is not None:
            for axis in ("x", "y", "z"):
                row[f"haply_pos_{axis}"] = getattr(haply.position, axis)
                row[f"haply_vel_{axis}"] = getattr(haply.velocity, axis)
        kh = self.latest_sample.get("K_h")
        uh = self.latest_sample.get("u_h")
        ua = self.latest_sample.get("u_a")
        if kh:
            row["K_h"] = json.dumps(list(kh))
        if uh is not None:
            row["u_h"] = json.dumps([uh.x, uh.y, uh.z])
        if ua is not None:
            row["u_a"] = json.dumps([ua.x, ua.y, ua.z])
        saved_row = {name: row.get(name) for name in self.csv_logger.fieldnames}
        self.csv_logger.write(saved_row)
        self.flush_counter += 1
        if self.flush_counter >= self.config.flush_interval:
            self.csv_logger.flush()
            self.flush_counter = 0

    def destroy_node(self):
        self._finalize_attempt("STOPPED", "logger_shutdown")
        if self.csv_logger is not None:
            self.csv_logger.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
