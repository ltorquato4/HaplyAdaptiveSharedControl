#!/usr/bin/env python3

import csv
import os
from datetime import datetime

import rclpy
from rclpy.node import Node

from std_msgs.msg import (
    Bool,
    Float64,
    String
)

from geometry_msgs.msg import (
    Point,
    Vector3
)

from haply_msgs.msg import HaplyState


class DataLoggerNode(Node):

    def __init__(self):
        super().__init__("data_logger_node")

        self.log_rate_hz = 100.0

        self.latest = {}

        self.initialize_storage()
        self.create_subscribers()

        self.timer = self.create_timer(
            1.0 / self.log_rate_hz,
            self.write_row
        )

        self.get_logger().info("Data Logger started")

    # ---------------------------------------------------
    # CSV
    # ---------------------------------------------------

    def initialize_storage(self):

        timestamp = datetime.now().strftime(
            "%Y%m%d_%H%M%S"
        )

        os.makedirs("logs", exist_ok=True)

        filename = f"logs/study_{timestamp}.csv"

        self.csv_file = open(
            filename,
            "w",
            newline=""
        )

        self.writer = csv.DictWriter(
            self.csv_file,
            fieldnames=self.fieldnames()
        )

        self.writer.writeheader()

        self.get_logger().info(
            f"Logging to {filename}"
        )

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

    # ---------------------------------------------------
    # Subscribers
    # ---------------------------------------------------

    def create_subscribers(self):

        qos = 10

        self.create_subscription(
            Bool,
            "/study_is_running",
            self.study_running_cb,
            qos
        )

        self.create_subscription(
            String,
            "/study_phase",
            self.phase_cb,
            qos
        )

        self.create_subscription(
            String,
            "/study_controller_mode",
            self.mode_cb,
            qos
        )

        self.create_subscription(
            Point,
            "/study_start_point",
            self.start_cb,
            qos
        )

        self.create_subscription(
            Point,
            "/study_end_point",
            self.end_cb,
            qos
        )

        self.create_subscription(
            Point,
            "/experiment_cursor_position",
            self.cursor_cb,
            qos
        )

        self.create_subscription(
            HaplyState,
            "/haply_state",
            self.haply_state_cb,
            qos
        )

        self.create_subscription(
            Vector3,
            "/haply_endeffector_force",
            self.force_cb,
            qos
        )

        self.create_subscription(
            Float64,
            "/estimation/K_h",
            self.kh_cb,
            qos
        )

        self.create_subscription(
            Vector3,
            "/estimation/u_h",
            self.uh_cb,
            qos
        )

        self.create_subscription(
            String,
            "/estimator_status",
            self.estimator_status_cb,
            qos
        )

        self.create_subscription(
            Float64,
            "/control/K_a",
            self.ka_cb,
            qos
        )

        self.create_subscription(
            Vector3,
            "/control/U_a",
            self.ua_cb,
            qos
        )

        self.create_subscription(
            Bool,
            "/study_endpoint_reached",
            self.endpoint_cb,
            qos
        )

    # ---------------------------------------------------
    # Callbacks
    # ---------------------------------------------------

    def study_running_cb(self, msg):
        self.latest["study_running"] = msg.data

    def phase_cb(self, msg):
        self.latest["study_phase"] = msg.data

    def mode_cb(self, msg):
        self.latest["study_controller_mode"] = msg.data

    def start_cb(self, msg):
        self.latest["start"] = msg

    def end_cb(self, msg):
        self.latest["end"] = msg

    def cursor_cb(self, msg):
        self.latest["cursor"] = msg

    def force_cb(self, msg):
        self.latest["force"] = msg

    def kh_cb(self, msg):
        self.latest["K_h"] = msg.data

    def uh_cb(self, msg):
        self.latest["u_h"] = msg

    def ka_cb(self, msg):
        self.latest["K_a"] = msg.data

    def ua_cb(self, msg):
        self.latest["U_a"] = msg

    def estimator_status_cb(self, msg):
        self.latest["estimator_status"] = msg.data

    def endpoint_cb(self, msg):
        self.latest["endpoint_reached"] = msg.data

    def haply_state_cb(self, msg):
        self.latest["haply_state"] = msg

    # ---------------------------------------------------
    # Logging
    # ---------------------------------------------------

    def write_row(self):

        row = {}

        row["timestamp"] = (
            self.get_clock()
            .now()
            .nanoseconds
            * 1e-9
        )

        row["study_running"] = \
            self.latest.get("study_running")

        row["study_phase"] = \
            self.latest.get("study_phase")

        row["study_controller_mode"] = \
            self.latest.get(
                "study_controller_mode"
            )

        start = self.latest.get("start")
        end = self.latest.get("end")
        cursor = self.latest.get("cursor")
        force = self.latest.get("force")
        uh = self.latest.get("u_h")
        ua = self.latest.get("U_a")

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

        haply = self.latest.get("haply_state")

        if haply:
            row["haply_pos_x"] = haply.position.x
            row["haply_pos_y"] = haply.position.y
            row["haply_pos_z"] = haply.position.z

            row["haply_vel_x"] = haply.velocity.x
            row["haply_vel_y"] = haply.velocity.y
            row["haply_vel_z"] = haply.velocity.z

        if force:
            row["force_x"] = force.x
            row["force_y"] = force.y
            row["force_z"] = force.z

        row["K_h"] = self.latest.get("K_h")
        row["K_a"] = self.latest.get("K_a")

        if uh:
            row["u_h_x"] = uh.x
            row["u_h_y"] = uh.y
            row["u_h_z"] = uh.z

        if ua:
            row["U_a_x"] = ua.x
            row["U_a_y"] = ua.y
            row["U_a_z"] = ua.z

        row["endpoint_reached"] = \
            self.latest.get(
                "endpoint_reached"
            )

        row["estimator_status"] = \
            self.latest.get(
                "estimator_status"
            )

        self.writer.writerow(row)

    def destroy_node(self):

        self.csv_file.flush()
        self.csv_file.close()

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