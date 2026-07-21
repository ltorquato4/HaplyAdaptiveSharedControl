#!/usr/bin/env python3

"""Participant-facing Pygame GUI for the Haply study."""

import os
import time
from math import isfinite

os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"
os.environ["SDL_AUDIODRIVER"] = "dummy"
os.environ["AUDIODEV"] = "null"

import pygame  # noqa: E402
import rclpy  # noqa: E402
from geometry_msgs.msg import Point  # noqa: E402
from haply_msgs.msg import (  # noqa: E402
    HaplyState,
    StudyAbortRequest,
    StudyDwellProgress,
    StudyStartRequest,
    StudyTask,
    StudyTrialState,
)
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from std_msgs.msg import Bool, Empty  # noqa: E402


class StudyGui(Node):
    """Participant-facing Pygame GUI and study run-state publisher."""

    COLORS = {
        "red": {
            "fill": (198, 40, 40),
            "muted": (255, 205, 210),
            "text": (198, 40, 40),
            "behavior": "aggressive",
            "icon": "!",
        },
        "yellow": {
            "fill": (245, 127, 23),
            "muted": (255, 249, 196),
            "text": (245, 127, 23),
            "behavior": "normal",
            "icon": "-",
        },
        "green": {
            "fill": (0, 105, 92),
            "muted": (178, 223, 219),
            "text": (0, 105, 92),
            "behavior": "careful",
            "icon": "+",
        },
    }

    APP_BACKGROUND = (233, 236, 239)
    WORKSPACE_BACKGROUND = (248, 249, 250)
    SURFACE = (255, 255, 255)
    TEXT = (33, 37, 41)
    MUTED_TEXT = (73, 80, 87)
    BORDER = (206, 212, 218)
    SHADOW = (210, 215, 220)
    BLUE = (49, 130, 206)
    DARK_BLUE = (44, 82, 130)
    PATH = (52, 58, 64)
    MODE_INSTRUCTIONS = {
        "aggressive": "Move quickly and decisively.",
        "normal": "Use your natural, comfortable pace.",
        "careful": "Move slowly and precisely.",
    }

    def __init__(self):
        """Create ROS interfaces and initialize the Pygame window."""
        super().__init__("study_gui")

        self.declare_parameter("width", 1280)
        self.declare_parameter("height", 720)
        self.declare_parameter("side_panel_width", 300)
        self.declare_parameter("workspace_padding", 0)
        self.declare_parameter("render_fps", 100.0)
        self.declare_parameter("state_publish_hz", 100.0)
        self.declare_parameter("source", "haply")
        self.declare_parameter("mouse_simulation", False)
        self.declare_parameter("mouse_simulation_hz", 100.0)
        self.declare_parameter("auto_start", False)
        self.declare_parameter("debug_controls_enabled", False)
        self.declare_parameter("max_callbacks_per_frame", 16)
        self.declare_parameter("mode_overlay_duration_s", 2.0)
        self.declare_parameter("start_x", -0.08)
        self.declare_parameter("start_y", -0.08)
        self.declare_parameter("start_z", 0.0)
        self.declare_parameter("end_x", 0.08)
        self.declare_parameter("end_y", 0.08)
        self.declare_parameter("end_z", 0.0)
        self.declare_parameter("workspace_x_min", -0.12)
        self.declare_parameter("workspace_x_max", 0.12)
        self.declare_parameter("workspace_y_min", -0.18)
        self.declare_parameter("workspace_y_max", 0.15)
        self.declare_parameter("start_reached_radius", 0.01)
        self.declare_parameter("endpoint_reached_radius", 0.01)

        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.side_panel_width = int(self.get_parameter("side_panel_width").value)
        self.workspace_padding = int(self.get_parameter("workspace_padding").value)
        self.render_fps = float(self.get_parameter("render_fps").value)
        self.state_publish_hz = float(self.get_parameter("state_publish_hz").value)
        self.mouse_simulation = bool(self.get_parameter("mouse_simulation").value)
        self.source = self._parse_source(
            str(self.get_parameter("source").value),
            self.mouse_simulation,
        )
        self.mouse_simulation_hz = float(
            self.get_parameter("mouse_simulation_hz").value
        )
        self.debug_controls_enabled = bool(
            self.get_parameter("debug_controls_enabled").value
        )
        self.auto_start = bool(self.get_parameter("auto_start").value)
        if self.auto_start and not self.debug_controls_enabled:
            self.get_logger().warning(
                "Ignoring auto_start because debug_controls_enabled is false"
            )
            self.auto_start = False
        self.max_callbacks_per_frame = max(
            1, int(self.get_parameter("max_callbacks_per_frame").value)
        )
        self.mode_overlay_duration_s = max(
            0.0, float(self.get_parameter("mode_overlay_duration_s").value)
        )
        self.endpoint_reached_radius = float(
            self.get_parameter("endpoint_reached_radius").value
        )
        self.start_reached_radius = float(
            self.get_parameter("start_reached_radius").value
        )

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
        self.cursor_received = False
        self.cursor_in_bounds = False
        self.mouse_in_workspace = True
        self.input_valid = False
        self.start_point_received = False
        self.end_point_received = False
        self.endpoint_dwell_progress = 0.0
        self.last_abort_reason = ""
        self.current_session_id = None
        self.current_trial_id = None
        self.session_finished = False
        self.previous_mouse_position = Point()
        self.previous_mouse_time = time.monotonic()
        self.study_phase = "normal"
        self.mode_overlay_until = None
        self.controller_mode = "adaptive"
        self.is_drawing_line = False
        self.finished_line_this_frame = False
        self.drawn_line = []
        self.trial_started = self.auto_start
        self.is_running = self.auto_start
        self.endpoint_reached = False
        self.trial_completion_latched = False
        self.mapping_ready = False
        self.running = True

        self.start_requested_pub = self.create_publisher(
            StudyStartRequest, "study_start_requested", 10
        )
        self.abort_requested_pub = self.create_publisher(
            StudyAbortRequest, "study_abort_requested", 10
        )
        task_qos = self._task_qos()

        state_qos = self._state_qos()
        self.create_subscription(
            Point, "experiment_cursor_position", self._experiment_cursor_position, state_qos
        )
        self.create_subscription(
            Bool, "study_mapping_ready", self._mapping_ready, task_qos
        )
        self.create_subscription(Empty, "study_button_pressed", self._button_pressed, 10)
        self.create_subscription(
            Bool, "experiment_input_valid", self._input_valid, task_qos
        )
        self.create_subscription(
            StudyDwellProgress,
            "study_endpoint_dwell_progress",
            self._dwell_progress,
            task_qos,
        )
        self.create_subscription(StudyTask, "study_task", self._study_task, task_qos)
        self.create_subscription(
            StudyTrialState, "study_trial_state", self._trial_state, task_qos
        )
        if self.source == "mouse":
            self.mouse_state_pub = self.create_publisher(
                HaplyState, "haply_state", state_qos
            )
            mouse_period_s = 1.0 / max(self.mouse_simulation_hz, 1.0)
            self.mouse_timer = self.create_timer(
                mouse_period_s, self._publish_mouse_haply_state
            )

        pygame.display.init()
        pygame.font.init()
        pygame.display.set_caption("Haply Study GUI")
        self.screen = self._create_display()
        self.frame = pygame.Surface((self.width, self.height)).convert()
        self.draw_target = self.frame
        self.clock = pygame.time.Clock()
        self.title_font = self._load_font(22, bold=True)
        self.body_font = self._load_font(23)
        self.label_font = self._load_font(17)
        self.pill_font = self._load_font(20, bold=True)
        self.icon_font = self._load_font(17, bold=True)
        if self.source == "mouse":
            self.previous_mouse_position = self._screen_to_world(pygame.mouse.get_pos())

    def _task_qos(self) -> QoSProfile:
        return QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

    def _state_qos(self) -> QoSProfile:
        return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)

    def _load_font(self, size, bold=False):
        preferred = ["Inter", "Roboto", "Open Sans", "DejaVu Sans", "Arial"]
        available = {
            name.lower().replace(" ", ""): name for name in pygame.font.get_fonts()
        }
        for font_name in preferred:
            key = font_name.lower().replace(" ", "")
            if key in available:
                return pygame.font.SysFont(available[key], size, bold=bold)
        return pygame.font.SysFont(None, size, bold=bold)

    def _create_display(self):
        try:
            return pygame.display.set_mode(
                (self.width, self.height), pygame.DOUBLEBUF, vsync=1
            )
        except TypeError:
            return pygame.display.set_mode((self.width, self.height), pygame.DOUBLEBUF)

    def _parse_source(self, value, mouse_simulation):
        source = value.strip().lower()
        if source not in ("mouse", "haply"):
            self.get_logger().warning(
                f"Unknown source '{value}', falling back to source=haply"
            )
            source = "haply"
        if source == "haply" and mouse_simulation:
            source = "mouse"
        return source

    @property
    def current_condition(self):
        """Return the color key for the current behavioral state."""
        phase = self.study_phase.strip().lower()
        for color_name, config in self.COLORS.items():
            if phase in (color_name, config["behavior"]):
                return color_name
        return "yellow"

    @property
    def current_behavior(self):
        """Return the human-readable behavioral state."""
        return self.COLORS[self.current_condition]["behavior"]

    def _experiment_cursor_position(self, msg):
        self.current_position = self._copy_point_2d(msg)
        self.cursor_received = True
        self.cursor_in_bounds = self._point_in_workspace(self.current_position)

    def _mapping_ready(self, msg):
        self.mapping_ready = bool(msg.data)

    def _button_pressed(self, _msg):
        if self.mapping_ready:
            self._start_trial_if_ready()

    def _study_task(self, msg):
        previous_phase = self.study_phase.strip().lower()
        received_previous_task = self.start_point_received and self.end_point_received
        next_phase = str(msg.phase).strip().lower()
        self.start_point = self._copy_point_2d(msg.start_point)
        self.end_point = self._copy_point_2d(msg.end_point)
        self.study_phase = next_phase
        self.controller_mode = str(msg.controller_mode)
        self.current_session_id = str(msg.session_id)
        self.current_trial_id = int(msg.trial_id)
        self.session_finished = False
        self.last_abort_reason = ""
        self.start_point_received = True
        self.end_point_received = True
        if not received_previous_task or next_phase != previous_phase:
            self.mode_overlay_until = (
                time.monotonic() + self.mode_overlay_duration_s
            )
        self._reset_drawn_path()
        self.get_logger().info(
            "Applied study task "
            f"{msg.trial_id}: start=({msg.start_point.x:.3f}, {msg.start_point.y:.3f}), "
            f"end=({msg.end_point.x:.3f}, {msg.end_point.y:.3f})"
        )

    def _trial_state(self, msg):
        if (
            self.current_session_id is None
            or str(msg.session_id) != self.current_session_id
            or self.current_trial_id is None
            or int(msg.trial_id) != self.current_trial_id
        ):
            return
        if msg.state == "RUNNING":
            self.is_running = True
        elif msg.state == "ABORTED":
            self._reset_drawn_path()
            self.last_abort_reason = str(msg.reason).strip() or "trial aborted"
        elif msg.state == "COMPLETED":
            self.endpoint_reached = True
            self.trial_completion_latched = True
            self.is_running = False
            self.is_drawing_line = False
            self.finished_line_this_frame = False
        elif msg.state == "SESSION_FINISHED":
            self.session_finished = True
            self.is_running = False
            self.is_drawing_line = False
        elif msg.state == "READY" and self.trial_started and not self.is_running:
            self._reset_drawn_path()

    def _input_valid(self, msg):
        self.input_valid = bool(msg.data)
        if not self.input_valid and self.is_running:
            self.is_running = False
            self.is_drawing_line = False

    def _dwell_progress(self, msg):
        if (
            self.current_session_id is None
            or str(msg.session_id) != self.current_session_id
            or self.current_trial_id is None
            or int(msg.trial_id) != self.current_trial_id
        ):
            return
        self.endpoint_dwell_progress = max(0.0, min(1.0, float(msg.progress)))

    def _handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_ESCAPE, pygame.K_q):
                    self.running = False
                elif self.debug_controls_enabled and event.key == pygame.K_SPACE:
                    self._start_trial_if_ready()
                elif self.debug_controls_enabled and event.key == pygame.K_s:
                    self._start_trial_if_ready()

    def _publish_mouse_haply_state(self):
        mouse_pos = pygame.mouse.get_pos()
        self.mouse_in_workspace = self._screen_pos_in_workspace(mouse_pos)
        if not self.mouse_in_workspace:
            # Do not turn side-panel/outside movement into a clamped experiment
            # cursor. Mapper freshness will mark the input invalid shortly.
            return
        now = time.monotonic()
        dt = max(now - self.previous_mouse_time, 1e-6)
        position = self._screen_to_world(mouse_pos)
        msg = HaplyState()
        msg.position = position
        msg.velocity.x = (position.x - self.previous_mouse_position.x) / dt
        msg.velocity.y = (position.y - self.previous_mouse_position.y) / dt
        msg.velocity.z = 0.0
        msg.quaternion.w = 1.0

        pressed = pygame.mouse.get_pressed(3)
        msg.buttons.a = bool(pressed[0])
        msg.buttons.b = bool(pressed[1])
        msg.buttons.c = bool(pressed[2])

        self.previous_mouse_position = position
        self.previous_mouse_time = now
        self.mouse_state_pub.publish(msg)

    def _at_start_position(self):
        return self.cursor_in_bounds and self._is_near(
            self.current_position,
            self.start_point,
            self.start_reached_radius,
        )

    def _start_trial_if_ready(self):
        if self.session_finished or self.trial_started or self.trial_completion_latched:
            return
        if self._mode_overlay_visible():
            return

        if (
            self.mapping_ready
            and self.input_valid
            and self.cursor_received
            and self._task_received()
            and self._at_start_position()
        ):
            self.trial_started = True
            self.last_abort_reason = ""
            self._append_drawn_point(self.current_position, force=True)
            self.is_drawing_line = True
            request = StudyStartRequest()
            request.session_id = getattr(self, "current_session_id", "") or ""
            request.trial_id = getattr(self, "current_trial_id", 0) or 0
            self.start_requested_pub.publish(request)

    def _update_line_drawing(self):
        if self.trial_completion_latched:
            return

        if not self.trial_started:
            return

        self.is_drawing_line = True
        self._append_drawn_point(self.current_position)


    def _draw_scene(self):
        self.draw_target = self.frame
        self.frame.fill(self.APP_BACKGROUND)
        self._draw_workspace()
        self._draw_side_panel()
        self._draw_behavioral_state_legend()
        self._draw_status_text()
        self._draw_mode_overlay()
        self.screen.blit(self.frame, (0, 0))
        pygame.display.update()

    def _draw_workspace(self):
        workspace = self._drawing_rect()
        pygame.draw.rect(self.draw_target, self.WORKSPACE_BACKGROUND, workspace)
        pygame.draw.rect(self.draw_target, self.BORDER, workspace, width=2)

        sx, sy = self._marker_canvas_position(
            self.start_point,
            self._task_radius_to_pixels(self.start_reached_radius),
        )
        ex, ey = self._marker_canvas_position(
            self.end_point,
            self._task_radius_to_pixels(self.endpoint_reached_radius),
        )
        cx, cy = self._world_to_canvas(self.current_position)

        if len(self.drawn_line) >= 2:
            points = [self._world_to_canvas(point) for point in self.drawn_line]
            pygame.draw.lines(self.draw_target, self.PATH, False, points, 6)

        self._draw_target_marker(sx, sy)
        self._draw_endpoint_marker(ex, ey)
        self._draw_marker_label("Start", sx, sy - 30)
        self._draw_marker_label("End", ex, ey - 34)
        if (
            self.mapping_ready
            and self.input_valid
            and self.cursor_received
            and self.cursor_in_bounds
        ):
            pygame.draw.circle(self.draw_target, self.BLUE, (cx, cy), 10)
            pygame.draw.circle(self.draw_target, self.DARK_BLUE, (cx, cy), 10, 2)

    def _draw_side_panel(self):
        panel = self._side_panel_rect()
        card = self._side_card_rect()
        shadow = card.move(0, 3)
        pygame.draw.rect(self.draw_target, self.SHADOW, shadow, border_radius=12)
        pygame.draw.rect(self.draw_target, self.SURFACE, card, border_radius=12)
        pygame.draw.rect(self.draw_target, self.BORDER, card, width=1, border_radius=12)
        pygame.draw.line(
            self.draw_target,
            self.BORDER,
            (panel.x, panel.y),
            (panel.x, panel.bottom),
            1,
        )

    def _draw_behavioral_state_legend(self):
        card = self._side_card_rect()
        x = card.x + 24
        y = card.y + 24

        title = self.title_font.render("Behavioral State", True, self.TEXT)
        self.draw_target.blit(title, (x, y))

        y += 44
        for index, color_name in enumerate(["red", "yellow", "green"]):
            pill = pygame.Rect(x, y + (index * 56), card.width - 48, 42)
            self._draw_state_pill(pill, color_name)

    def _draw_state_pill(self, rect, color_name):
        config = self.COLORS[color_name]
        active = color_name == self.current_condition
        background = (
            config["muted"]
            if active
            else self._blend_color(config["muted"], self.SURFACE, 0.86)
        )
        fill = (
            config["fill"]
            if active
            else self._blend_color(config["fill"], self.SURFACE, 0.82)
        )
        text_color = config["text"] if active else self._blend_color(
            self.MUTED_TEXT, self.SURFACE, 0.55
        )
        border = (
            config["fill"]
            if active
            else self._blend_color(self.BORDER, self.SURFACE, 0.72)
        )
        border_width = 3 if active else 1

        pygame.draw.rect(self.draw_target, background, rect, border_radius=16)
        pygame.draw.rect(
            self.draw_target,
            border,
            rect,
            width=border_width,
            border_radius=16,
        )

        dot_center = (rect.x + 23, rect.centery)
        pygame.draw.circle(self.draw_target, fill, dot_center, 10)

        icon = self.icon_font.render(config["icon"], True, self.SURFACE)
        icon_rect = icon.get_rect(center=dot_center)
        self.draw_target.blit(icon, icon_rect)

        label = self.pill_font.render(config["behavior"].capitalize(), True, text_color)
        label_rect = label.get_rect(midleft=(rect.x + 46, rect.centery))
        self.draw_target.blit(label, label_rect)

    def _mode_overlay_visible(self):
        return (
            self.mode_overlay_until is not None
            and time.monotonic() < self.mode_overlay_until
        )

    def _draw_mode_overlay(self):
        if not self._mode_overlay_visible():
            return

        remaining = self.mode_overlay_until - time.monotonic()
        fade_window_s = min(0.4, self.mode_overlay_duration_s)
        opacity = 235
        if fade_window_s > 0.0 and remaining < fade_window_s:
            opacity = round(235 * max(0.0, remaining) / fade_window_s)

        workspace = self._workspace_rect()
        overlay = pygame.Surface(workspace.size, pygame.SRCALPHA)
        overlay.fill((255, 255, 255, opacity))
        self.draw_target.blit(overlay, workspace.topleft)

        config = self.COLORS[self.current_condition]
        title = self.title_font.render(
            f"{self.current_behavior.capitalize()} mode", True, config["text"]
        )
        instruction = self.body_font.render(
            self.MODE_INSTRUCTIONS[self.current_behavior], True, self.TEXT
        )
        title_rect = title.get_rect(center=(workspace.centerx, workspace.centery - 20))
        instruction_rect = instruction.get_rect(
            center=(workspace.centerx, workspace.centery + 20)
        )
        self.draw_target.blit(title, title_rect)
        self.draw_target.blit(instruction, instruction_rect)

    def _blend_color(self, color, target, amount):
        return tuple(
            int(channel + ((target_channel - channel) * amount))
            for channel, target_channel in zip(color, target, strict=True)
        )

    def _legend_rect(self):
        card = self._side_card_rect()
        return pygame.Rect(card.x, card.y, card.width, 230)

    def _draw_target_marker(self, x, y):
        radius = self._task_radius_to_pixels(self.start_reached_radius)
        pygame.draw.circle(self.draw_target, self.SURFACE, (x, y), radius)
        pygame.draw.circle(self.draw_target, self.TEXT, (x, y), radius, 3)

    def _draw_endpoint_marker(self, x, y):
        radius = self._task_radius_to_pixels(self.endpoint_reached_radius)
        pygame.draw.circle(self.draw_target, self.SURFACE, (x, y), radius)
        pygame.draw.circle(self.draw_target, self.TEXT, (x, y), radius, 3)
        pygame.draw.line(self.draw_target, self.TEXT, (x - 7, y - 7), (x + 7, y + 7), 3)
        pygame.draw.line(self.draw_target, self.TEXT, (x - 7, y + 7), (x + 7, y - 7), 3)

    def _draw_marker_label(self, label, x, y):
        text = self.label_font.render(label, True, self.MUTED_TEXT)
        self.draw_target.blit(text, text.get_rect(center=(x, y)))

    def _draw_status_text(self):
        card = self._side_card_rect()
        x = card.x + 24
        y = self._legend_rect().bottom + 18

        title = self.title_font.render("Run Status", True, self.TEXT)
        self.draw_target.blit(title, (x, y))

        row_y = y + 44
        value_x = x + 98
        value_width = max(card.right - value_x - 18, 1)
        for label, value in self._status_rows():
            label_text = self.label_font.render(label, True, self.MUTED_TEXT)
            self.draw_target.blit(label_text, (x, row_y))
            lines = self._wrap_sidebar_text(value, value_width)
            for line_index, line in enumerate(lines):
                value_text = self.body_font.render(line, True, self.TEXT)
                self.draw_target.blit(
                    value_text,
                    (value_x, row_y - 4 + (line_index * self.body_font.get_linesize())),
                )
            row_y += max(46, (len(lines) * self.body_font.get_linesize()) + 8)

    def _wrap_sidebar_text(self, value, max_width, max_lines=3):
        """Wrap a status value to the available sidebar width (up to three lines)."""
        words = str(value).split()
        if not words:
            return [""]

        lines = []
        current = ""
        for word in words:
            candidate = f"{current} {word}".strip()
            if self.body_font.size(candidate)[0] <= max_width:
                current = candidate
                continue
            if current:
                lines.append(current)
                current = word
            else:
                current = word
            if len(lines) == max_lines:
                return lines

        if current:
            lines.append(current)
        return lines[:max_lines]

    def _status_rows(self):
        if not self._task_received():
            state = "waiting for scenario"
        elif self.session_finished:
            state = "session complete"
        elif self.source == "mouse" and not self.mouse_in_workspace:
            state = "pointer outside workspace"
        elif not self.input_valid:
            state = "device disconnected"
        elif self.cursor_received and not self.cursor_in_bounds:
            state = "cursor outside workspace"
        elif not self.mapping_ready:
            state = "press A at neutral"
        elif self._mode_overlay_visible():
            state = "read mode instruction"
        elif self.last_abort_reason:
            state = f"trial aborted: {self.last_abort_reason.replace('_', ' ')}"
        elif self.is_running:
            state = (
                "hold at endpoint"
                if self.endpoint_dwell_progress > 0.0
                else "running"
            )
        else:
            state = "move to start then press A"
        return [
            ("State", state),
            ("Trial", str(self.current_trial_id) if self.current_trial_id is not None else "-"),
            ("Control", self.controller_mode),
        ]

    def _world_to_canvas(self, point):
        scale_x, scale_y, left, bottom = self._canvas_transform()
        return (
            int(left + ((float(point.x) - self.workspace["x_min"]) * scale_x)),
            int(bottom - ((float(point.y) - self.workspace["y_min"]) * scale_y)),
        )

    def _screen_to_world(self, pos):
        scale_x, scale_y, left, bottom = self._canvas_transform()
        x_screen, y_screen = pos

        point = Point()
        point.x = self.workspace["x_min"] + ((x_screen - left) / scale_x)
        point.y = self.workspace["y_min"] + ((bottom - y_screen) / scale_y)
        point.z = 0.0
        return point

    def _canvas_transform(self):
        draw_rect = self._drawing_rect()
        x_span = max(self.workspace["x_max"] - self.workspace["x_min"], 1e-9)
        y_span = max(self.workspace["y_max"] - self.workspace["y_min"], 1e-9)
        # Mouse mode uses every pixel of the visible white workspace.  Using
        # one shared scale letterboxed the task into an invisible central
        # rectangle, so pointer movement in the visible side areas was
        # converted outside the task bounds and then appeared clamped.
        scale_x = draw_rect.width / x_span
        scale_y = draw_rect.height / y_span
        return scale_x, scale_y, draw_rect.x, draw_rect.bottom

    def _screen_pos_in_workspace(self, pos):
        # The white task rectangle is the entire mouse workspace. It is also
        # the rectangle used for marker and cursor coordinate transforms.
        return self._drawing_rect().collidepoint(pos)

    def _point_in_workspace(self, point):
        x, y = float(point.x), float(point.y)
        return (
            isfinite(x)
            and isfinite(y)
            and self.workspace["x_min"] <= x <= self.workspace["x_max"]
            and self.workspace["y_min"] <= y <= self.workspace["y_max"]
        )

    def _task_radius_to_pixels(self, radius):
        scale_x, scale_y, _left, _bottom = self._canvas_transform()
        return max(1, round(float(radius) * min(scale_x, scale_y)))

    def _marker_canvas_position(self, point, radius):
        """Keep a visual marker fully visible without changing task geometry."""
        x, y = self._world_to_canvas(point)
        workspace = self._workspace_rect()
        return (
            min(max(x, workspace.left + radius), workspace.right - radius),
            min(max(y, workspace.top + radius), workspace.bottom - radius),
        )

    def _workspace_rect(self):
        return pygame.Rect(
            0,
            0,
            max(self.width - self.side_panel_width, 1),
            self.height,
        )

    def _side_panel_rect(self):
        return pygame.Rect(
            max(self.width - self.side_panel_width, 0),
            0,
            min(self.side_panel_width, self.width),
            self.height,
        )

    def _side_card_rect(self):
        panel = self._side_panel_rect()
        padding = 18
        return pygame.Rect(
            panel.x + padding,
            panel.y + padding,
            max(panel.width - (2 * padding), 1),
            max(panel.height - (2 * padding), 1),
        )

    def _drawing_rect(self):
        # There is one participant-visible task frame: the full white left
        # workspace. Do not create a letterboxed or padded inner boundary.
        return self._workspace_rect()

    def _copy_point_2d(self, point):
        copy = Point()
        copy.x = float(point.x)
        copy.y = float(point.y)
        copy.z = 0.0
        return copy

    def _append_drawn_point(self, point, force=False):
        new_point = self._copy_point_2d(point)
        if not self.drawn_line:
            self.drawn_line.append(new_point)
            return

        last_point = self.drawn_line[-1]
        dx = new_point.x - last_point.x
        dy = new_point.y - last_point.y
        min_step = 0.001
        if force or ((dx * dx + dy * dy) ** 0.5) >= min_step:
            self.drawn_line.append(new_point)

    def _is_near(self, first, second, radius):
        dx = float(first.x) - float(second.x)
        dy = float(first.y) - float(second.y)
        return ((dx * dx) + (dy * dy)) ** 0.5 <= radius

    def _reset_drawn_path(self):
        self.drawn_line = []
        self.is_drawing_line = False
        self.trial_started = False
        self.is_running = False
        self.endpoint_reached = False
        self.trial_completion_latched = False
        self.endpoint_dwell_progress = 0.0

    def _task_received(self):
        return self.start_point_received and self.end_point_received

    def run(self):
        """Run the GUI event loop until ROS or the window closes."""
        while rclpy.ok() and self.running:
            for _ in range(self.max_callbacks_per_frame):
                rclpy.spin_once(self, timeout_sec=0.0)
            self._handle_events()
            self._update_line_drawing()
            self._draw_scene()
            self.clock.tick(max(self.render_fps, 1.0))

    def request_abort_on_exit(self) -> None:
        """Tell orchestration to stop the active trial before GUI teardown."""
        if (
            not (self.trial_started or self.is_running)
            or self.current_session_id is None
            or self.current_trial_id is None
        ):
            return
        request = StudyAbortRequest()
        request.session_id = self.current_session_id
        request.trial_id = self.current_trial_id
        request.reason = "gui_closed"
        self.abort_requested_pub.publish(request)
        # Give the ROS client a short opportunity to flush the request before
        # the launch exit handler tears down the rest of the study stack.
        for _ in range(3):
            if not rclpy.ok():
                break
            rclpy.spin_once(self, timeout_sec=0.02)


def main(args=None):
    """Start the study GUI node."""
    rclpy.init(args=args)
    node = StudyGui()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.request_abort_on_exit()
        node.destroy_node()
        pygame.font.quit()
        pygame.display.quit()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
