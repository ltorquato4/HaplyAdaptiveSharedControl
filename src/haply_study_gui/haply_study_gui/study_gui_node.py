#!/usr/bin/env python3

"""Participant-facing Pygame GUI for the Haply study."""

import os
import time

os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"
os.environ["SDL_AUDIODRIVER"] = "dummy"
os.environ["AUDIODEV"] = "null"

import pygame  # noqa: E402
import rclpy  # noqa: E402
from geometry_msgs.msg import Point  # noqa: E402
from haply_msgs.msg import HandleButtons, HaplyState  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402


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

    def __init__(self):
        """Create ROS interfaces and initialize the Pygame window."""
        super().__init__("study_gui")

        self.declare_parameter("width", 960)
        self.declare_parameter("height", 540)
        self.declare_parameter("side_panel_width", 220)
        self.declare_parameter("workspace_padding", 36)
        self.declare_parameter("render_fps", 100.0)
        self.declare_parameter("state_publish_hz", 100.0)
        self.declare_parameter("source", "haply")
        self.declare_parameter("mouse_simulation", False)
        self.declare_parameter("mouse_simulation_hz", 100.0)
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
        self.auto_start = bool(self.get_parameter("auto_start").value)
        self.endpoint_reached_radius = float(
            self.get_parameter("endpoint_reached_radius").value
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
        self.previous_mouse_position = Point()
        self.previous_mouse_time = time.monotonic()
        self.current_buttons = HandleButtons()
        self.study_phase = "normal"
        self.controller_mode = "adaptive"
        self.draw_button_pressed = False
        self.is_drawing_line = False
        self.finished_line_this_frame = False
        self.drawn_line = []
        self.is_running = self.auto_start
        self.endpoint_reached = False
        self.trial_completion_latched = False
        self.running = True

        self.study_is_running_pub = self.create_publisher(Bool, "study_is_running", 10)

        self.create_subscription(HaplyState, "haply_state", self._haply_state, 10)
        self.create_subscription(
            Point, "experiment_cursor_position", self._experiment_cursor_position, 10
        )
        self.create_subscription(Point, "study_start_point", self._start_point, 10)
        self.create_subscription(Point, "study_end_point", self._end_point, 10)
        self.create_subscription(String, "study_phase", self._study_phase, 10)
        self.create_subscription(
            String, "study_controller_mode", self._controller_mode, 10
        )
        if self.source == "mouse":
            self.mouse_state_pub = self.create_publisher(HaplyState, "haply_state", 10)
            mouse_period_s = 1.0 / max(self.mouse_simulation_hz, 1.0)
            self.mouse_timer = self.create_timer(
                mouse_period_s, self._publish_mouse_haply_state
            )
        publish_period_s = 1.0 / max(self.state_publish_hz, 0.1)
        self.publish_timer = self.create_timer(
            publish_period_s, self._publish_study_state
        )

        pygame.display.init()
        pygame.font.init()
        pygame.display.set_caption("Haply Study GUI")
        self.screen = self._create_display()
        self.frame = pygame.Surface((self.width, self.height)).convert()
        self.draw_target = self.frame
        self.clock = pygame.time.Clock()
        self.title_font = self._load_font(16, bold=True)
        self.body_font = self._load_font(18)
        self.label_font = self._load_font(14)
        self.pill_font = self._load_font(15, bold=True)
        self.icon_font = self._load_font(13, bold=True)
        if self.source == "mouse":
            self.current_position = self._screen_to_world(pygame.mouse.get_pos())
            self.previous_mouse_position = self.current_position

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

    def _haply_state(self, msg):
        if self.source != "haply":
            return
        self.current_buttons = msg.buttons

    def _experiment_cursor_position(self, msg):
        if self.source == "mouse":
            return
        self.current_position = self._copy_point_2d(msg)

    def _start_point(self, msg):
        point = self._copy_point_2d(msg)
        if self._point_changed(self.start_point, point):
            self.start_point = point
            self._reset_drawn_path()

    def _end_point(self, msg):
        point = self._copy_point_2d(msg)
        if self._point_changed(self.end_point, point):
            self.end_point = point
            self._reset_drawn_path()

    def _study_phase(self, msg):
        phase = msg.data.strip().lower()
        valid_phases = {"aggressive", "normal", "careful", "red", "yellow", "green"}
        if phase in valid_phases:
            self.study_phase = phase
        else:
            self.get_logger().warning(f"Ignoring unknown study_phase '{msg.data}'")

    def _controller_mode(self, msg):
        mode = msg.data.strip().lower()
        if mode in ("adaptive", "fixed"):
            self.controller_mode = mode
        else:
            self.get_logger().warning(
                f"Ignoring unknown study_controller_mode '{msg.data}'"
            )

    def _publish_study_state(self):
        running_msg = Bool()
        running_msg.data = bool(self.is_running)
        self.study_is_running_pub.publish(running_msg)

    def _handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_ESCAPE, pygame.K_q):
                    self.running = False
                elif event.key == pygame.K_SPACE:
                    self.is_running = not self.is_running
                elif event.key == pygame.K_s:
                    self.is_running = True

    def _publish_mouse_haply_state(self):
        mouse_pos = pygame.mouse.get_pos()
        now = time.monotonic()
        dt = max(now - self.previous_mouse_time, 1e-6)
        position = self._screen_to_world(mouse_pos)
        self.current_position = position

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

    def _update_line_drawing(self):
        self.finished_line_this_frame = False
        if self.source == "mouse":
            mouse_pos = pygame.mouse.get_pos()
            self.current_position = self._screen_to_world(mouse_pos)
            pressed = pygame.mouse.get_pressed(3)[0] and self._screen_pos_in_workspace(
                mouse_pos
            )
        else:
            pressed = bool(self.current_buttons.a)

        if pressed and not self.draw_button_pressed:
            self.is_running = True
            self.drawn_line = []
            self._append_drawn_point(self.current_position, force=True)
            self.is_drawing_line = True
        elif pressed and self.is_drawing_line:
            self._append_drawn_point(self.current_position)
        elif not pressed and self.draw_button_pressed and self.is_drawing_line:
            self._append_drawn_point(self.current_position, force=True)
            self.is_drawing_line = False
            self.finished_line_this_frame = True

        self.draw_button_pressed = pressed

    def _update_endpoint_feedback(self):
        if self.trial_completion_latched:
            self.endpoint_reached = True
            return

        cursor_reached_endpoint = self._is_near(
            self.current_position, self.end_point, self.endpoint_reached_radius
        )
        drawing_active = self.is_drawing_line or self.finished_line_this_frame
        if cursor_reached_endpoint and drawing_active:
            self._append_drawn_point(self.current_position, force=True)
        completed = (
            cursor_reached_endpoint
            and drawing_active
            and self._drawn_line_connects_start_to_end()
        )
        self.endpoint_reached = completed
        if completed:
            self.trial_completion_latched = True
            self.get_logger().info("Endpoint reached; waiting for next phase")

    def _draw_scene(self):
        self.draw_target = self.frame
        self.frame.fill(self.APP_BACKGROUND)
        self._draw_workspace()
        self._draw_side_panel()
        self._draw_behavioral_state_legend()
        self._draw_status_text()
        self.screen.blit(self.frame, (0, 0))
        pygame.display.update()

    def _draw_workspace(self):
        workspace = self._workspace_rect()
        pygame.draw.rect(self.draw_target, self.WORKSPACE_BACKGROUND, workspace)
        pygame.draw.rect(self.draw_target, self.BORDER, workspace, width=2)

        sx, sy = self._world_to_canvas(self.start_point)
        ex, ey = self._world_to_canvas(self.end_point)
        cx, cy = self._world_to_canvas(self.current_position)

        if len(self.drawn_line) >= 2:
            points = [self._world_to_canvas(point) for point in self.drawn_line]
            pygame.draw.lines(self.draw_target, self.PATH, False, points, 4)

        self._draw_target_marker(sx, sy)
        self._draw_endpoint_marker(ex, ey)
        pygame.draw.circle(self.draw_target, self.BLUE, (cx, cy), 7)
        pygame.draw.circle(self.draw_target, self.DARK_BLUE, (cx, cy), 7, 1)

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
        x = card.x + 18
        y = card.y + 18

        title = self.title_font.render("Behavioral State", True, self.TEXT)
        self.draw_target.blit(title, (x, y))

        y += 34
        for index, color_name in enumerate(["red", "yellow", "green"]):
            pill = pygame.Rect(x, y + (index * 42), card.width - 36, 32)
            self._draw_state_pill(pill, color_name)

    def _draw_state_pill(self, rect, color_name):
        config = self.COLORS[color_name]
        active = color_name == self.current_condition
        background = (
            config["muted"]
            if active
            else self._blend_color(config["muted"], self.SURFACE, 0.72)
        )
        fill = (
            config["fill"]
            if active
            else self._blend_color(config["fill"], self.SURFACE, 0.68)
        )
        text_color = config["text"] if active else self.MUTED_TEXT
        border = (
            config["fill"]
            if active
            else self._blend_color(self.BORDER, self.SURFACE, 0.45)
        )
        border_width = 2 if active else 1

        pygame.draw.rect(self.draw_target, background, rect, border_radius=16)
        pygame.draw.rect(
            self.draw_target,
            border,
            rect,
            width=border_width,
            border_radius=16,
        )

        dot_center = (rect.x + 18, rect.centery)
        pygame.draw.circle(self.draw_target, fill, dot_center, 7)

        icon = self.icon_font.render(config["icon"], True, self.SURFACE)
        icon_rect = icon.get_rect(center=dot_center)
        self.draw_target.blit(icon, icon_rect)

        label = self.pill_font.render(config["behavior"].capitalize(), True, text_color)
        label_rect = label.get_rect(midleft=(rect.x + 34, rect.centery))
        self.draw_target.blit(label, label_rect)

    def _blend_color(self, color, target, amount):
        return tuple(
            int(channel + ((target_channel - channel) * amount))
            for channel, target_channel in zip(color, target, strict=True)
        )

    def _legend_rect(self):
        card = self._side_card_rect()
        return pygame.Rect(card.x, card.y, card.width, 178)

    def _draw_target_marker(self, x, y):
        pygame.draw.circle(self.draw_target, self.SURFACE, (x, y), 9)
        pygame.draw.circle(self.draw_target, self.TEXT, (x, y), 9, 2)

    def _draw_endpoint_marker(self, x, y):
        radius = 11
        pygame.draw.circle(self.draw_target, self.SURFACE, (x, y), radius)
        pygame.draw.circle(self.draw_target, self.TEXT, (x, y), radius, 2)
        pygame.draw.line(self.draw_target, self.TEXT, (x - 5, y - 5), (x + 5, y + 5), 2)
        pygame.draw.line(self.draw_target, self.TEXT, (x - 5, y + 5), (x + 5, y - 5), 2)

    def _draw_status_text(self):
        card = self._side_card_rect()
        x = card.x + 18
        y = self._legend_rect().bottom + 18

        title = self.title_font.render("Run Status", True, self.TEXT)
        self.draw_target.blit(title, (x, y))

        for index, (label, value) in enumerate(self._status_rows()):
            row_y = y + 34 + (index * 36)
            label_text = self.label_font.render(label, True, self.MUTED_TEXT)
            value_text = self.body_font.render(value, True, self.TEXT)
            self.draw_target.blit(label_text, (x, row_y))
            self.draw_target.blit(value_text, (x + 78, row_y - 3))

    def _status_rows(self):
        state = "running" if self.is_running else "ready"
        return [
            ("State", state),
            ("Control", self.controller_mode),
        ]

    def _world_to_canvas(self, point):
        draw_rect = self._drawing_rect()
        usable_width = max(draw_rect.width, 1)
        usable_height = max(draw_rect.height, 1)

        x_span = self.workspace["x_max"] - self.workspace["x_min"]
        y_span = self.workspace["y_max"] - self.workspace["y_min"]
        x_span = x_span if x_span != 0.0 else 1.0
        y_span = y_span if y_span != 0.0 else 1.0

        x_norm = (float(point.x) - self.workspace["x_min"]) / x_span
        y_norm = (float(point.y) - self.workspace["y_min"]) / y_span

        x_norm = max(0.0, min(1.0, x_norm))
        y_norm = max(0.0, min(1.0, y_norm))

        x = int(draw_rect.x + (x_norm * usable_width))
        y = int(draw_rect.bottom - (y_norm * usable_height))
        return x, y

    def _screen_to_world(self, pos):
        draw_rect = self._drawing_rect()
        usable_width = max(draw_rect.width, 1)
        usable_height = max(draw_rect.height, 1)
        x_screen, y_screen = pos

        x_norm = (x_screen - draw_rect.x) / usable_width
        y_norm = (draw_rect.bottom - y_screen) / usable_height
        x_norm = max(0.0, min(1.0, x_norm))
        y_norm = max(0.0, min(1.0, y_norm))

        point = Point()
        point.x = self.workspace["x_min"] + (
            x_norm * (self.workspace["x_max"] - self.workspace["x_min"])
        )
        point.y = self.workspace["y_min"] + (
            y_norm * (self.workspace["y_max"] - self.workspace["y_min"])
        )
        point.z = 0.0
        return point

    def _screen_pos_in_workspace(self, pos):
        return self._drawing_rect().collidepoint(pos)

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
        padding = 14
        return pygame.Rect(
            panel.x + padding,
            panel.y + padding,
            max(panel.width - (2 * padding), 1),
            max(panel.height - (2 * padding), 1),
        )

    def _drawing_rect(self):
        workspace = self._workspace_rect()
        padding = max(
            0,
            min(
                self.workspace_padding,
                workspace.width // 3,
                workspace.height // 3,
            ),
        )
        return workspace.inflate(-2 * padding, -2 * padding)

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

    def _drawn_line_connects_start_to_end(self):
        if len(self.drawn_line) < 2:
            return False
        radius = self.endpoint_reached_radius
        return self._is_near(
            self.drawn_line[0], self.start_point, radius
        ) and self._is_near(self.drawn_line[-1], self.end_point, radius)

    def _is_near(self, first, second, radius):
        dx = float(first.x) - float(second.x)
        dy = float(first.y) - float(second.y)
        return ((dx * dx) + (dy * dy)) ** 0.5 <= radius

    def _reset_drawn_path(self):
        self.drawn_line = []
        self.draw_button_pressed = False
        self.is_drawing_line = False
        self.finished_line_this_frame = False
        self.endpoint_reached = False
        self.trial_completion_latched = False

    def _point_changed(self, first, second):
        return (
            abs(float(first.x) - float(second.x)) > 1e-6
            or abs(float(first.y) - float(second.y)) > 1e-6
        )

    def run(self):
        """Run the GUI event loop until ROS or the window closes."""
        while rclpy.ok() and self.running:
            rclpy.spin_once(self, timeout_sec=0.0)
            self._handle_events()
            self._update_line_drawing()
            self._draw_scene()
            self.clock.tick(max(self.render_fps, 1.0))


def main(args=None):
    """Start the study GUI node."""
    rclpy.init(args=args)
    node = StudyGui()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        pygame.font.quit()
        pygame.display.quit()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
