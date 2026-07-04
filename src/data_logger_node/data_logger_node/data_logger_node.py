#!/usr/bin/env python3
import csv
import os

from dataclasses import dataclass

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Float64, String
from geometry_msgs.msg import Point, Vector3

from haply_msgs.msg import HaplyState

from csv_logger import CSVLogger

@dataclass
class LoggerConfig:
    log_rate_hz: float = 100.0
    flush_interval: int = 100
    file_prefix: str = "trial"


class DataLoggerNode(Node):
    def __init__(self):
        super().__init__("data_logger_node")

        self.declare_parameter("save_directory", "./logs")
        self.save_directory = (self.get_parameter("save_directory").get_parameter_value().string_value)

        self.config = LoggerConfig()
        self.recording = False
        self.latest = {}
        self.flush_counter = 0

        self.csv_logger = CSVLogger(self.save_directory, self.config.file_prefix, self.fieldnames())

        self.setup_subscribers()
        self.timer = self.create_timer(1.0 / self.config.log_rate_hz, self.write_row)
        
        self.get_logger().info("Data logger ready.")


    def fieldnames(self):
        return [
            "trial_id",
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

            "force_x",
            "force_y",
            "force_z",

            "K_h",

            "u_h_x",
            "u_h_y",
            "u_h_z",

            "K_a",

            "U_a_x",
            "U_a_y",
            "U_a_z",

            "endpoint_reached",

            "estimator_status"
        ]

    
    def reset_state(self):
        self.latest = {}


    def start_recording(self):
        self.reset_state()
        trial_id, filepath = self.csv_logger.start()
        self.recording = True
        self.get_logger().info(f"Started trial {trial_id}: {filepath}")


    def stop_recording(self):
        self.csv_logger.stop()
        self.recording = False
        self.reset_state()
        self.get_logger().info("Trial finished.")


    def setup_subscribers(self):
        self.create_subscription(Bool, "/study_is_running", self.study_running_callback, 10)
        self.create_subscription(String, "/study_phase", self.phase_callback, 10)
        self.create_subscription(String, "/study_controller_mode", self.mode_callback, 10)
        self.create_subscription(Point, "/study_start_point", self.start_point_callback, 10)
        self.create_subscription(Point, "/study_end_point", self.end_point_callback, 10)
        self.create_subscription(Point, "/experiment_cursor_position", self.cursor_callback, 10)
        self.create_subscription(HaplyState, "/haply_state", self.haply_callback, 10)
        self.create_subscription(Vector3, "/haply_endeffector_force", self.force_callback, 10)
        self.create_subscription(Float64, "/estimation/K_h", self.kh_callback, 10)
        self.create_subscription(Vector3, "/estimation/u_h", self.uh_callback, 10)
        self.create_subscription(Float64, "/control/K_a", self.ka_callback, 10)
        self.create_subscription(Vector3, "/control/U_a", self.ua_callback, 10)
        self.create_subscription(String, "/estimator_status", self.estimator_status_callback, 10)
        self.create_subscription(Bool, "/study_endpoint_reached", self.endpoint_callback, 10)


    def study_running_callback(self, msg):
        old_state = self.recording
        new_state = msg.data

        self.latest["study_running"] = new_state

        if new_state and not old_state:
            self.start_recording()

        elif not new_state and old_state:
            self.stop_recording()


    def phase_callback(self, msg):
        self.latest["study_phase"] = msg.data


    def mode_callback(self, msg):
        self.latest["study_controller_mode"] = msg.data


    def start_point_callback(self, msg):
        self.latest["start"] = msg


    def end_point_callback(self, msg):
        self.latest["end"] = msg


    def cursor_callback(self, msg):
        self.latest["cursor"] = msg


    def force_callback(self, msg):
        self.latest["force"] = msg


    def kh_callback(self, msg):
        self.latest["K_h"] = msg.data


    def uh_callback(self, msg):
        self.latest["u_h"] = msg


    def ka_callback(self, msg):
        self.latest["K_a"] = msg.data


    def ua_callback(self, msg):
        self.latest["U_a"] = msg


    def estimator_status_callback(self, msg):
        self.latest["estimator_status"] = msg.data


    def endpoint_callback(self, msg):
        self.latest["endpoint_reached"] = msg.data


    def haply_callback(self, msg):
        self.latest["haply"] = msg


    def write_row(self):
        if not self.recording:
            return

        row = {}

        row["trial_id"] = self.csv_logger.trial_id
        row["timestamp"] = self.get_clock().now().nanoseconds * 1e-9
        row["study_running"] = self.latest.get("study_running")
        row["study_phase"] = self.latest.get("study_phase")
        row["study_controller_mode"] = self.latest.get("study_controller_mode")

        start = self.latest.get("start")
        end = self.latest.get("end")
        cursor = self.latest.get("cursor")
        force = self.latest.get("force")
        uh = self.latest.get("u_h")
        ua = self.latest.get("U_a")
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

        if force:
            row["force_x"] = force.x
            row["force_y"] = force.y
            row["force_z"] = force.z

        if uh:
            row["u_h_x"] = uh.x
            row["u_h_y"] = uh.y
            row["u_h_z"] = uh.z

        if ua:
            row["U_a_x"] = ua.x
            row["U_a_y"] = ua.y
            row["U_a_z"] = ua.z

        if haply:
            row["haply_pos_x"] = (
                haply.position.x
            )
            row["haply_pos_y"] = (
                haply.position.y
            )
            row["haply_pos_z"] = (
                haply.position.z
            )
            row["haply_vel_x"] = (
                haply.velocity.x
            )
            row["haply_vel_y"] = (
                haply.velocity.y
            )
            row["haply_vel_z"] = (
                haply.velocity.z
            )

        row["K_h"] = (
            self.latest.get("K_h")
        )

        row["K_a"] = (
            self.latest.get("K_a")
        )

        row["endpoint_reached"] = (
            self.latest.get(
                "endpoint_reached"
            )
        )

        row["estimator_status"] = (
            self.latest.get(
                "estimator_status"
            )
        )

        self.csv_logger.write(row)

        self.flush_counter += 1

        if (
            self.flush_counter
            >= self.config.flush_interval
        ):
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