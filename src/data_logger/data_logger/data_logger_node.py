#!/usr/bin/env python3

import os
from dataclasses import dataclass
from datetime import datetime

import rclpy
from geometry_msgs.msg import Point, Vector3
from haply_msgs.msg import HaplyState, StudyTask
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
    def __init__(self):
        super().__init__("data_logger_node")

        self.declare_parameter("save_directory", "./logs")
        self.declare_parameter("log_level", "info")

        # Get the base directory from parameters
        base_directory = (
            self.get_parameter("save_directory").get_parameter_value().string_value
        )
        
        # Generate a timestamp for this specific run
        run_timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        
        # Combine them so all trajectories in this run go into the timestamped folder
        self.save_directory = os.path.join(base_directory, run_timestamp)

        self.log_level = (
            self.get_parameter("log_level").get_parameter_value().string_value
        )

        self.get_logger().set_level(self._resolve_log_level(self.log_level))

        self.config = LoggerConfig()

        # Track running state solely on study_is_running
        self.study_is_running = False
        self.recording = False
        
        # Explicit state splitting
        self.trial_metadata = {}  
        self.latest_sample = {}  
        self.flush_counter = 0
        self.task_received = False

        self.csv_logger = CSVLogger(
            self.save_directory,
            self.config.file_prefix,
            self.fieldnames(),
        )

        self.setup_subscribers()
        self.ready_pub = self.create_publisher(
            Bool, "/study_logger_ready",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE),
        )
        self.ready_timer = self.create_timer(0.5, self._publish_ready)

        self.timer = self.create_timer(
            1.0 / self.config.log_rate_hz,
            self.write_row,
        )

        self.get_logger().info(f"Data logger ready. Saving to: {self.save_directory}")

    def _publish_ready(self):
        self.ready_pub.publish(Bool(data=self.task_received and self.csv_logger is not None))

    @staticmethod
    def _task_qos():
        return QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

    def _message_to_debug_value(self, msg):
        if hasattr(msg, "data"):
            return msg.data

        if hasattr(msg, "position") and hasattr(msg, "velocity"):
            return {
                "position": self._message_to_debug_value(msg.position),
                "velocity": self._message_to_debug_value(msg.velocity),
            }

        if all(hasattr(msg, attr) for attr in ("x", "y", "z")):
            return {"x": msg.x, "y": msg.y, "z": msg.z}

        return repr(msg)

    def _log_received_message(self, topic_name, msg):
        self.get_logger().debug(
            f"received {topic_name}: {self._message_to_debug_value(msg)}"
        )

    def _normalize_row_for_debug(self, row):
        return {
            fieldname: row.get(fieldname)
            for fieldname in self.csv_logger.fieldnames
        }

    def _resolve_log_level(self, log_level_name):
        log_levels = {
            "debug": LoggingSeverity.DEBUG,
            "info": LoggingSeverity.INFO,
            "warn": LoggingSeverity.WARN,
            "warning": LoggingSeverity.WARN,
            "error": LoggingSeverity.ERROR,
            "fatal": LoggingSeverity.FATAL,
        }
        normalized = str(log_level_name).strip().lower()
        return log_levels.get(normalized, LoggingSeverity.DEBUG)

    def fieldnames(self):
        return [
            "timestamp",
            "session_id",
            "trial_id",
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

    def reset_sample_state(self):  
        self.latest_sample = {}  

    def reset_all_state(self):  
        self.trial_metadata = {}  
        self.latest_sample = {}  

    def start_recording(self):
        if self.recording:
            return

        self.reset_sample_state()  
        trajectory_id, filepath = self.csv_logger.start()
        self.recording = True
        self.get_logger().info(f"Study started! Recording trajectory {trajectory_id}: {filepath}")

    def stop_recording(self):
        if not self.recording:
            return

        self.csv_logger.stop()
        self.recording = False
        self.reset_sample_state()  
        self.get_logger().info("Study stopped. Data logging stopped.")

    def setup_subscribers(self):
        self.create_subscription(
            StudyTask,
            "/study_task",
            self.study_task_callback,
            self._task_qos(),
        )
        self.create_subscription(
            Bool,
            "/study_is_running",
            self.study_running_callback,
            10,
        )

        self.create_subscription(
            String,
            "/study_phase",
            self.phase_callback,
            10,
        )

        self.create_subscription(
            String,
            "/study_controller_mode",
            self.mode_callback,
            10,
        )

        self.create_subscription(
            Point,
            "/study_start_point",
            self.start_point_callback,
            10,
        )

        self.create_subscription(
            Point,
            "/study_end_point",
            self.end_point_callback,
            10,
        )

        self.create_subscription(
            Point,
            "/experiment_cursor_position",
            self.cursor_callback,
            10,
        )

        self.create_subscription(
            HaplyState,
            "/haply_state",
            self.haply_callback,
            10,
        )

        self.create_subscription(
            Float64MultiArray,
            "/estimation/K_h",
            self.kh_callback,
            10,
        )

        self.create_subscription(
            Vector3,
            "/estimation/u_h",
            self.uh_callback,
            10,
        )

        self.create_subscription(
            String,
            "/control/K_a",
            self.ka_callback,
            10,
        )

        self.create_subscription(
            Vector3,
            "/control/U_a",
            self.ua_callback,
            10,
        )

        self.create_subscription(
            Bool,
            "/study_endpoint_reached",
            self.endpoint_callback,
            10,
        )

    def study_running_callback(self, msg):
        self._log_received_message("/study_is_running", msg)

        new_study_is_running = msg.data
        if self.study_is_running == new_study_is_running:
            return

        self.study_is_running = new_study_is_running
        self.get_logger().debug(f"Study is running state changed: {self.study_is_running}")

        if self.study_is_running:
            self.start_recording()
        else:
            self.stop_recording()

    def phase_callback(self, msg):  
        if self.task_received:
            return
        self._log_received_message("/study_phase", msg)  
        self.trial_metadata["study_phase"] = msg.data  

    def mode_callback(self, msg):  
        if self.task_received:
            return
        self._log_received_message("/study_controller_mode", msg)  
        self.trial_metadata["study_controller_mode"] = msg.data  

    def start_point_callback(self, msg):  
        if self.task_received:
            return
        self._log_received_message("/study_start_point", msg)  
        self.trial_metadata["start"] = msg  

    def end_point_callback(self, msg):  
        if self.task_received:
            return
        self._log_received_message("/study_end_point", msg)  
        self.trial_metadata["end"] = msg  

    def study_task_callback(self, msg: StudyTask):
        """Set complete trial metadata atomically from the retained task."""
        self.trial_metadata.update(
            {
                "session_id": str(msg.session_id),
                "trial_id": int(msg.trial_id),
                "study_phase": str(msg.phase),
                "study_controller_mode": str(msg.controller_mode),
                "start": msg.start_point,
                "end": msg.end_point,
            }
        )
        self.task_received = True

    def cursor_callback(self, msg):  
        self._log_received_message("/experiment_cursor_position", msg)  
        self.latest_sample["cursor"] = msg  

    def kh_callback(self, msg):  
        self._log_received_message("/estimation/K_h", msg)  
        self.latest_sample["K_h"] = msg.data  

    def uh_callback(self, msg):  
        self._log_received_message("/estimation/u_h", msg)  
        self.latest_sample["u_h"] = msg  

    def ka_callback(self, msg):  
        self._log_received_message("/control/K_a", msg)  
        self.latest_sample["K_a"] = msg.data  

    def ua_callback(self, msg):  
        self._log_received_message("/control/U_a", msg)  
        self.latest_sample["u_a"] = msg  

    def endpoint_callback(self, msg):
        self._log_received_message("/study_endpoint_reached", msg)
        self.latest_sample["endpoint_reached"] = msg.data

    def haply_callback(self, msg):  
        self._log_received_message("/haply_state", msg)  
        self.latest_sample["haply"] = msg  

    def write_row(self):
        if not self.recording or not self.study_is_running:
            return

        row = {}
        row["timestamp"] = self.get_clock().now().nanoseconds * 1e-9
        row["session_id"] = self.trial_metadata.get("session_id")
        row["trial_id"] = self.trial_metadata.get("trial_id")
        row["study_running"] = self.study_is_running  
        row["study_phase"] = self.trial_metadata.get("study_phase")  
        row["study_controller_mode"] = self.trial_metadata.get("study_controller_mode")  

        start = self.trial_metadata.get("start")  
        end = self.trial_metadata.get("end")  

        cursor = self.latest_sample.get("cursor")  
        uh = self.latest_sample.get("u_h")  
        ua = self.latest_sample.get("u_a")  
        Kh = self.latest_sample.get("K_h")  
        haply = self.latest_sample.get("haply")  

        if start:
            row["start_x"] = start.x
            row["start_y"] = start.y
            row["start_z"] = start.z

        if end:
            row["end_x"] = end.x
            row["end_y"] = end.y
            row["end_z"] = end.z

        if cursor:
            row["cursor_x"] = cursor.x
            row["cursor_y"] = cursor.y
            row["cursor_z"] = cursor.z

        if haply:
            row["haply_pos_x"] = haply.position.x
            row["haply_pos_y"] = haply.position.y
            row["haply_pos_z"] = haply.position.z
            row["haply_vel_x"] = haply.velocity.x
            row["haply_vel_y"] = haply.velocity.y
            row["haply_vel_z"] = haply.velocity.z

        if Kh:
            row["K_h"] = str(list(Kh))
            
        if uh:
            row["u_h"] = str([uh.x, uh.y, uh.z])

        if ua:
            row["u_a"] = str([ua.x, ua.y, ua.z])

        row["K_a"] = self.latest_sample.get("K_a")
        row["endpoint_reached"] = self.latest_sample.get("endpoint_reached")

        saved_row = self._normalize_row_for_debug(row)
        self.csv_logger.write(saved_row)
        self.get_logger().debug(f"saved csv row: {saved_row}")

        self.flush_counter += 1
        if self.flush_counter >= self.config.flush_interval:
            self.csv_logger.flush()
            self.flush_counter = 0

    def destroy_node(self):
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
        rclpy.shutdown()


if __name__ == "__main__":
    main()
