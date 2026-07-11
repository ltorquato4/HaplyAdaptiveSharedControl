#!/usr/bin/env python3

from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Point, Vector3
from haply_msgs.msg import HaplyState
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray, String

from .csv_logger import CSVLogger


@dataclass
class LoggerConfig:
    log_rate_hz: float = 100.0
    flush_interval: int = 100
    file_prefix: str = "trial"


class DataLoggerNode(Node):
    def __init__(self):
        super().__init__("data_logger_node")

        self.declare_parameter("save_directory", "./logs")
        self.declare_parameter("log_level", "debug")

        self.save_directory = (
            self.get_parameter("save_directory").get_parameter_value().string_value
        )

        self.log_level = (
            self.get_parameter("log_level").get_parameter_value().string_value
        )

        self.get_logger().set_level(self._resolve_log_level(self.log_level))

        self.config = LoggerConfig()

        self.recording = False
        self.current_button_a_state = False
        self.endpoint_reached_flag = False
        
        self.latest = {}
        self.flush_counter = 0

        self.csv_logger = CSVLogger(
            self.save_directory,
            self.config.file_prefix,
            self.fieldnames(),
        )

        self.setup_subscribers()

        self.timer = self.create_timer(
            1.0 / self.config.log_rate_hz,
            self.write_row,
        )

        self.get_logger().info("Data logger ready.")

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

    def reset_state(self):
        self.latest = {}

    def start_recording(self):

        if self.recording:
            return

        self.reset_state()

        trial_id, filepath = self.csv_logger.start()

        self.recording = True

        self.get_logger().info(
            f"Button A pressed! Started trial {trial_id}: {filepath}"
        )

    def stop_recording(self):

        if not self.recording:
            return

        self.csv_logger.stop()

        self.recording = False

        self.reset_state()

        self.get_logger().info(
            "Endpoint reached message received. Data logging stopped."
        )

    def setup_subscribers(self):

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
        self.latest["study_running"] = msg.data

    def phase_callback(self, msg):
        self._log_received_message("/study_phase", msg)
        self.latest["study_phase"] = msg.data

    def mode_callback(self, msg):
        self._log_received_message("/study_controller_mode", msg)
        self.latest["study_controller_mode"] = msg.data

    def start_point_callback(self, msg):
        self._log_received_message("/study_start_point", msg)
        self.latest["start"] = msg

    def end_point_callback(self, msg):
        self._log_received_message("/study_end_point", msg)
        self.latest["end"] = msg

    def cursor_callback(self, msg):
        self._log_received_message("/experiment_cursor_position", msg)
        self.latest["cursor"] = msg

    def kh_callback(self, msg):
        self._log_received_message("/estimation/K_h", msg)
        self.latest["K_h"] = msg.data

    def uh_callback(self, msg):
        self._log_received_message("/estimation/u_h", msg)
        self.latest["u_h"] = msg

    def ka_callback(self, msg):
        self._log_received_message("/control/K_a", msg)
        self.latest["K_a"] = msg.data

    def ua_callback(self, msg):
        self._log_received_message("/control/U_a", msg)
        self.latest["u_a"] = msg

    def endpoint_callback(self, msg):
        self._log_received_message("/study_endpoint_reached", msg)
        self.latest["endpoint_reached"] = msg.data

        if not self.endpoint_reached_flag and msg.data:
            self.endpoint_reached_flag = True
            
            if self.recording:
                self.stop_recording()

    def haply_callback(self, msg):
        self._log_received_message("/haply_state", msg)
        self.latest["haply"] = msg
        self.current_button_a_state = msg.buttons.a

        if not self.endpoint_reached_flag and self.current_button_a_state:
            if not self.recording:
                self.start_recording()
                
        elif self.endpoint_reached_flag and not self.current_button_a_state:
            self.endpoint_reached_flag = False

    def write_row(self):
        if not self.recording:
            return

        row = {}

        row["timestamp"] = self.get_clock().now().nanoseconds * 1e-9
        row["study_running"] = self.latest.get("study_running")
        row["study_phase"] = self.latest.get("study_phase")
        row["study_controller_mode"] = self.latest.get("study_controller_mode")

        start = self.latest.get("start")
        end = self.latest.get("end")
        cursor = self.latest.get("cursor")
        uh = self.latest.get("u_h")
        ua = self.latest.get("u_a")
        Kh = self.latest.get("K_h")
        haply = self.latest.get("haply")

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

        row["K_a"] = self.latest.get("K_a")
        row["endpoint_reached"] = self.latest.get("endpoint_reached")

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
