#!/usr/bin/env python3

import json
import time
import tkinter as tk
from tkinter import ttk

import rclpy
from geometry_msgs.msg import Point
from haply_msgs.msg import HaplyState
from rclpy.node import Node
from std_msgs.msg import String


class StudyTrafficLightGui(Node):
    """Traffic-light GUI and scenario-state publisher for Haply studies."""

    COLORS = {
        "red": {
            "fill": "#ff1a1a",
            "muted": "#ffd1d1",
            "behavior": "aggressive",
        },
        "yellow": {
            "fill": "#fff200",
            "muted": "#fff7a8",
            "behavior": "normal",
        },
        "green": {
            "fill": "#00b894",
            "muted": "#b9f3df",
            "behavior": "careful",
        },
    }

    def __init__(self):
        super().__init__("study_traffic_light_gui")

        self.declare_parameter("width", 960)
        self.declare_parameter("height", 540)
        self.declare_parameter("phase_duration_s", 60.0)
        self.declare_parameter("run_duration_s", 900.0)
        self.declare_parameter("condition_order", "red,yellow,green")
        self.declare_parameter("controller_type", "adaptive")
        self.declare_parameter("auto_start", False)
        self.declare_parameter("start_x", -0.08)
        self.declare_parameter("start_y", -0.20)
        self.declare_parameter("start_z", 0.0)
        self.declare_parameter("end_x", 0.08)
        self.declare_parameter("end_y", -0.08)
        self.declare_parameter("end_z", 0.0)
        self.declare_parameter("workspace_x_min", -0.12)
        self.declare_parameter("workspace_x_max", 0.12)
        self.declare_parameter("workspace_y_min", -0.28)
        self.declare_parameter("workspace_y_max", 0.02)

        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.phase_duration_s = float(
            self.get_parameter("phase_duration_s").value
        )
        self.run_duration_s = float(self.get_parameter("run_duration_s").value)
        self.controller_type = str(self.get_parameter("controller_type").value)
        self.auto_start = bool(self.get_parameter("auto_start").value)

        condition_order = str(self.get_parameter("condition_order").value)
        self.condition_order = self._parse_condition_order(condition_order)

        self.start_point = Point(
            x=float(self.get_parameter("start_x").value),
            y=float(self.get_parameter("start_y").value),
            z=float(self.get_parameter("start_z").value),
        )
        self.end_point = Point(
            x=float(self.get_parameter("end_x").value),
            y=float(self.get_parameter("end_y").value),
            z=float(self.get_parameter("end_z").value),
        )
        self.workspace = {
            "x_min": float(self.get_parameter("workspace_x_min").value),
            "x_max": float(self.get_parameter("workspace_x_max").value),
            "y_min": float(self.get_parameter("workspace_y_min").value),
            "y_max": float(self.get_parameter("workspace_y_max").value),
        }

        self.current_position = Point()
        self.condition_index = 0
        self.is_running = self.auto_start
        self.run_start_time = time.monotonic() if self.auto_start else None
        self.phase_start_time = time.monotonic() if self.auto_start else None
        self.elapsed_run_s = 0.0
        self.elapsed_phase_s = 0.0

        self.behavior_pub = self.create_publisher(
            String, "study_behavior_state", 10
        )
        self.trial_state_pub = self.create_publisher(
            String, "study_trial_state", 10
        )
        self.start_point_pub = self.create_publisher(
            Point, "study_start_point", 10
        )
        self.end_point_pub = self.create_publisher(Point, "study_end_point", 10)

        self.create_subscription(HaplyState, "haply_state", self._haply_state, 10)
        self.publish_timer = self.create_timer(0.1, self._publish_study_state)

        self.root = tk.Tk()
        self.root.title("Haply Study GUI")
        self.root.configure(bg="white")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self.canvas = tk.Canvas(
            self.root,
            width=self.width,
            height=self.height,
            bg="white",
            highlightthickness=0,
        )
        self.canvas.pack(fill=tk.BOTH, expand=True)

        self._build_controls()
        self._draw_static_scene()
        self._draw_dynamic_scene()

    def _parse_condition_order(self, value):
        order = [item.strip().lower() for item in value.split(",")]
        order = [item for item in order if item in self.COLORS]
        if not order:
            order = ["red", "yellow", "green"]
        return order

    def _build_controls(self):
        self.control_frame = tk.Frame(self.root, bg="white")
        self.control_frame.place(x=20, y=self.height - 44)

        self.start_button = ttk.Button(
            self.control_frame, text="Start", command=self._start_run
        )
        self.start_button.grid(row=0, column=0, padx=(0, 8))

        self.pause_button = ttk.Button(
            self.control_frame, text="Pause", command=self._toggle_pause
        )
        self.pause_button.grid(row=0, column=1, padx=(0, 8))

        self.reset_button = ttk.Button(
            self.control_frame, text="Reset", command=self._reset_run
        )
        self.reset_button.grid(row=0, column=2, padx=(0, 12))

        self.status_var = tk.StringVar()
        self.status_label = tk.Label(
            self.control_frame,
            textvariable=self.status_var,
            bg="white",
            fg="#111111",
            font=("Arial", 11),
        )
        self.status_label.grid(row=0, column=3)

    def _draw_static_scene(self):
        self.canvas.delete("static")

        sx, sy = self._world_to_canvas(self.start_point)
        ex, ey = self._world_to_canvas(self.end_point)

        self.canvas.create_line(
            sx, sy, ex, ey, fill="#d0d7de", width=3, tags="static"
        )
        self._draw_target_marker(sx, sy, "#111111", "static")
        self._draw_target_marker(ex, ey, "#111111", "static")

    def _draw_dynamic_scene(self):
        self.canvas.delete("dynamic")
        self._draw_traffic_light()
        self._draw_current_position()
        self._update_status_text()

    def _draw_traffic_light(self):
        radius = 13
        x = self.width - 55
        y0 = 24
        spacing = 32
        active = self.current_condition

        for index, color_name in enumerate(["red", "yellow", "green"]):
            y = y0 + index * spacing
            color = self.COLORS[color_name]["fill"]
            outline = "#111111" if color_name == active else color
            width = 3 if color_name == active else 1
            self.canvas.create_oval(
                x - radius,
                y - radius,
                x + radius,
                y + radius,
                fill=color,
                outline=outline,
                width=width,
                tags="dynamic",
            )

    def _draw_current_position(self):
        x, y = self._world_to_canvas(self.current_position)
        radius = 6
        self.canvas.create_oval(
            x - radius,
            y - radius,
            x + radius,
            y + radius,
            fill="#0969da",
            outline="#033d8b",
            width=1,
            tags="dynamic",
        )

    def _draw_target_marker(self, x, y, color, tag):
        radius = 8
        self.canvas.create_oval(
            x - radius,
            y - radius,
            x + radius,
            y + radius,
            fill="white",
            outline=color,
            width=2,
            tags=tag,
        )

    def _world_to_canvas(self, point):
        margin = 74
        usable_width = max(self.width - (2 * margin), 1)
        usable_height = max(self.height - (2 * margin), 1)

        x_span = self.workspace["x_max"] - self.workspace["x_min"]
        y_span = self.workspace["y_max"] - self.workspace["y_min"]
        x_span = x_span if x_span != 0.0 else 1.0
        y_span = y_span if y_span != 0.0 else 1.0

        x_norm = (float(point.x) - self.workspace["x_min"]) / x_span
        y_norm = (float(point.y) - self.workspace["y_min"]) / y_span

        x_norm = max(0.0, min(1.0, x_norm))
        y_norm = max(0.0, min(1.0, y_norm))

        x = margin + (x_norm * usable_width)
        y = self.height - margin - (y_norm * usable_height)
        return x, y

    @property
    def current_condition(self):
        return self.condition_order[self.condition_index]

    @property
    def current_behavior(self):
        return self.COLORS[self.current_condition]["behavior"]

    def _start_run(self):
        now = time.monotonic()
        self.is_running = True
        self.run_start_time = now
        self.phase_start_time = now
        self.condition_index = 0
        self._draw_dynamic_scene()

    def _toggle_pause(self):
        if self.run_start_time is None:
            self._start_run()
            return
        self.is_running = not self.is_running
        if self.is_running:
            now = time.monotonic()
            self.run_start_time = now - self.elapsed_run_s
            self.phase_start_time = now - self.elapsed_phase_s
        self._draw_dynamic_scene()

    def _reset_run(self):
        self.is_running = False
        self.run_start_time = None
        self.phase_start_time = None
        self.elapsed_run_s = 0.0
        self.elapsed_phase_s = 0.0
        self.condition_index = 0
        self._draw_dynamic_scene()

    def _haply_state(self, msg):
        self.current_position = msg.position

    def _update_scenario_clock(self):
        if not self.is_running or self.run_start_time is None:
            return

        now = time.monotonic()
        self.elapsed_run_s = now - self.run_start_time
        self.elapsed_phase_s = now - self.phase_start_time

        if self.elapsed_run_s >= self.run_duration_s:
            self.is_running = False
            return

        if self.elapsed_phase_s >= self.phase_duration_s:
            self.condition_index = (
                self.condition_index + 1
            ) % len(self.condition_order)
            self.phase_start_time = now
            self.elapsed_phase_s = 0.0

    def _publish_study_state(self):
        self._update_scenario_clock()

        behavior_msg = String()
        behavior_msg.data = self.current_behavior
        self.behavior_pub.publish(behavior_msg)
        self.start_point_pub.publish(self.start_point)
        self.end_point_pub.publish(self.end_point)

        trial_msg = String()
        trial_msg.data = json.dumps(
            {
                "running": self.is_running,
                "condition": self.current_condition,
                "behavior": self.current_behavior,
                "controller_type": self.controller_type,
                "elapsed_run_s": round(getattr(self, "elapsed_run_s", 0.0), 3),
                "elapsed_phase_s": round(
                    getattr(self, "elapsed_phase_s", 0.0), 3
                ),
                "phase_duration_s": self.phase_duration_s,
                "run_duration_s": self.run_duration_s,
                "start_point": self._point_to_dict(self.start_point),
                "end_point": self._point_to_dict(self.end_point),
            },
            separators=(",", ":"),
        )
        self.trial_state_pub.publish(trial_msg)
        self._draw_dynamic_scene()

    def _update_status_text(self):
        remaining = max(
            0.0,
            self.phase_duration_s - getattr(self, "elapsed_phase_s", 0.0),
        )
        state = "running" if self.is_running else "paused"
        self.status_var.set(
            f"{self.controller_type} | {self.current_behavior} | "
            f"{state} | {remaining:0.1f}s"
        )

    def _point_to_dict(self, point):
        return {"x": point.x, "y": point.y, "z": point.z}

    def _ros_tick(self):
        if rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            self.root.after(10, self._ros_tick)
        else:
            self._on_close()

    def _on_close(self):
        self.root.quit()

    def run(self):
        self._ros_tick()
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = StudyTrafficLightGui()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
