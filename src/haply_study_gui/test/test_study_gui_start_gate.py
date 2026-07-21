from geometry_msgs.msg import Point
from haply_msgs.msg import StudyCursor, StudyDwellProgress, StudyTask, StudyTrialState
from haply_study_gui.study_gui_node import StudyGui
from std_msgs.msg import Bool, Empty


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeLogger:
    def info(self, _message):
        pass


class FakeFont:
    def size(self, value):
        return (len(value) * 10, 20)


def _point(x, y):
    msg = Point()
    msg.x, msg.y, msg.z = x, y, 0.0
    return msg


def _state(state, trial_id=0):
    return StudyTrialState(
        session_id="test-session", trial_id=trial_id, state=state
    )


def _cursor(x, y, trial_id=0, valid=True):
    return StudyCursor(
        session_id="test-session",
        trial_id=trial_id,
        position=_point(x, y),
        input_valid=valid,
    )


def _task(trial_id=0, start=(0.0, 0.0), end=(1.0, 0.0), phase="normal"):
    return StudyTask(
        session_id="test-session",
        trial_id=trial_id,
        start_point=_point(*start),
        end_point=_point(*end),
        phase=phase,
        controller_mode="adaptive",
    )


def _gui():
    gui = object.__new__(StudyGui)
    gui.current_position = _point(0.0, 0.0)
    gui.cursor_received = True
    gui.input_valid = True
    gui.start_point_received = True
    gui.end_point_received = True
    gui.start_point = _point(0.0, 0.0)
    gui.end_point = _point(1.0, 0.0)
    gui.endpoint_reached_radius = 0.1
    gui.start_reached_radius = 0.1
    gui.workspace = {"x_min": -1.0, "x_max": 1.0, "y_min": -1.0, "y_max": 1.0}
    gui.cursor_in_bounds = True
    gui.drawn_line = []
    gui.max_drawn_points = 4
    gui.trial_started = False
    gui.is_running = False
    gui.endpoint_reached = False
    gui.endpoint_dwell_progress = 0.0
    gui.last_abort_reason = ""
    gui.study_phase = "normal"
    gui.mode_overlay_duration_s = 2.0
    gui.mode_overlay_until = None
    gui.trial_completion_latched = False
    gui.mapping_ready = False
    gui.session_finished = False
    gui.start_requested_pub = FakePublisher()
    gui.abort_requested_pub = FakePublisher()
    gui.get_logger = lambda: FakeLogger()
    gui.current_session_id = "test-session"
    gui.current_trial_id = 0
    return gui


def test_mapping_ready_does_not_start_trial():
    gui = _gui()
    ready = Bool(data=True)
    gui._mapping_ready(ready)
    assert not gui.trial_started
    assert not gui.is_running


def test_invalid_input_aborts_an_active_trial():
    gui = _gui()
    gui._mapping_ready(Bool(data=True))
    gui._button_pressed(Empty())
    gui._trial_state(_state("RUNNING"))

    gui._experiment_cursor_position(_cursor(0.0, 0.0, valid=False))

    assert not gui.is_running


def test_press_event_outside_start_does_not_start_trial():
    gui = _gui()
    gui._mapping_ready(Bool(data=True))
    gui.current_position = _point(0.5, 0.0)
    gui._button_pressed(Empty())
    assert not gui.trial_started
    assert not gui.is_running


def test_press_event_at_start_starts_trial():
    gui = _gui()
    gui._mapping_ready(Bool(data=True))
    gui._button_pressed(Empty())
    gui._trial_state(_state("RUNNING"))
    assert gui.trial_started
    assert gui.is_running
    assert len(gui.drawn_line) == 1


def test_no_press_event_is_reused_for_next_scenario():
    gui = _gui()
    gui._mapping_ready(Bool(data=True))
    gui._button_pressed(Empty())
    gui._trial_state(_state("RUNNING"))
    gui._trial_state(_state("COMPLETED"))
    gui._study_task(_task(trial_id=1, start=(0.1, 0.0)))
    assert not gui.trial_started
    assert not gui.is_running


def test_path_continues_without_holding_button():
    gui = _gui()
    gui._mapping_ready(Bool(data=True))
    gui._button_pressed(Empty())
    gui._trial_state(_state("RUNNING"))
    gui.current_position = _point(0.2, 0.0)
    gui._update_line_drawing()
    first_path_length = len(gui.drawn_line)
    gui.current_position = _point(0.4, 0.0)
    gui._update_line_drawing()
    assert gui.is_running
    assert len(gui.drawn_line) > first_path_length


def test_canvas_transform_preserves_scale_and_round_trips():
    gui = _gui()
    gui.width = 1280
    gui.height = 720
    gui.side_panel_width = 300
    gui.workspace_padding = 52
    gui.workspace = {
        "x_min": -0.12,
        "x_max": 0.12,
        "y_min": -0.15,
        "y_max": 0.15,
    }
    point = _point(0.03, -0.04)

    screen = gui._world_to_canvas(point)
    restored = gui._screen_to_world(screen)

    assert abs(restored.x - point.x) < 0.001
    assert abs(restored.y - point.y) < 0.001
    assert gui._canvas_transform()[0] > 0.0


def test_mouse_workspace_includes_task_boundary():
    gui = _gui()
    gui.width = 1280
    gui.height = 720
    gui.side_panel_width = 300
    gui.workspace_padding = 52

    rect = gui._drawing_rect()

    assert gui._screen_pos_in_workspace((rect.centerx, rect.bottom - 1))


def test_out_of_bounds_cursor_cannot_start_a_trial():
    gui = _gui()
    gui._mapping_ready(Bool(data=True))
    gui._experiment_cursor_position(_cursor(2.0, 0.0))

    gui._button_pressed(Empty())

    assert gui.cursor_in_bounds is False
    assert gui.trial_started is False
    assert gui.start_requested_pub.messages == []


def test_cursor_from_an_old_trial_is_ignored():
    gui = _gui()
    gui.current_position = _point(0.0, 0.0)

    gui._experiment_cursor_position(_cursor(0.5, 0.0, trial_id=1))

    assert (gui.current_position.x, gui.current_position.y) == (0.0, 0.0)


def test_display_path_is_bounded_by_decimation():
    gui = _gui()
    for index in range(20):
        gui._append_drawn_point(_point(index * 0.01, 0.0), force=True)

    assert len(gui.drawn_line) <= gui.max_drawn_points


def test_dwell_progress_requires_matching_task_identity():
    gui = _gui()
    gui._dwell_progress(
        StudyDwellProgress(session_id="old", trial_id=0, progress=1.0)
    )
    assert gui.endpoint_dwell_progress == 0.0

    gui._dwell_progress(
        StudyDwellProgress(session_id="test-session", trial_id=0, progress=0.5)
    )
    assert gui.endpoint_dwell_progress == 0.5


def test_controller_failure_abort_is_visible_to_the_gui():
    gui = _gui()
    gui._trial_state(
        StudyTrialState(
            session_id="test-session",
            trial_id=0,
            state="ABORTED",
            reason="controller_failure",
        )
    )

    assert gui.is_running is False
    assert gui.last_abort_reason == "controller_failure"


def test_mode_change_shows_overlay_and_delays_start(monkeypatch):
    gui = _gui()
    clock = [10.0]
    monkeypatch.setattr(
        "haply_study_gui.study_gui_node.time.monotonic", lambda: clock[0]
    )
    gui._study_task(_task(phase="careful"))
    gui._mapping_ready(Bool(data=True))

    assert gui._mode_overlay_visible()
    gui._button_pressed(Empty())
    assert gui.trial_started is False

    clock[0] = 12.1
    assert not gui._mode_overlay_visible()
    gui._button_pressed(Empty())
    assert gui.trial_started is True


def test_sidebar_status_values_wrap_to_three_lines():
    gui = _gui()
    gui.body_font = FakeFont()

    lines = gui._wrap_sidebar_text(
        "move to start then press A after calibration", max_width=100
    )

    assert lines == ["move to", "start then", "press A"]


def test_gui_exit_requests_abort_for_running_trial(monkeypatch):
    gui = _gui()
    gui.is_running = True
    monkeypatch.setattr("haply_study_gui.study_gui_node.rclpy.ok", lambda: False)

    gui.request_abort_on_exit()

    request = gui.abort_requested_pub.messages[-1]
    assert request.session_id == "test-session"
    assert request.trial_id == 0
    assert request.reason == "gui_closed"


def test_gui_exit_requests_abort_after_start_was_requested(monkeypatch):
    gui = _gui()
    gui.trial_started = True
    monkeypatch.setattr("haply_study_gui.study_gui_node.rclpy.ok", lambda: False)

    gui.request_abort_on_exit()

    assert gui.abort_requested_pub.messages[-1].reason == "gui_closed"
