from geometry_msgs.msg import Point
from haply_msgs.msg import HandleButtons
from haply_study_gui.study_gui_node import StudyGui
from std_msgs.msg import Bool


def _point(x, y):
    msg = Point()
    msg.x = x
    msg.y = y
    msg.z = 0.0
    return msg


def _gui():
    gui = object.__new__(StudyGui)
    gui.source = "haply"
    gui.current_position = _point(0.0, 0.0)
    gui.start_point = _point(0.0, 0.0)
    gui.end_point = _point(1.0, 0.0)
    gui.endpoint_reached_radius = 0.1
    gui.current_buttons = HandleButtons()
    gui.draw_button_pressed = False
    gui.is_drawing_line = False
    gui.finished_line_this_frame = False
    gui.drawn_line = []
    gui.trial_started = False
    gui.is_running = False
    gui.endpoint_reached = False
    gui.trial_completion_latched = False
    return gui


def test_button_a_away_from_start_does_not_start_trial():
    gui = _gui()
    gui.current_position = _point(0.5, 0.0)
    gui.current_buttons.a = True

    gui._update_line_drawing()

    assert not gui.trial_started
    assert not gui.is_running
    assert gui.drawn_line == []


def test_button_a_held_while_entering_start_radius_starts_trial():
    gui = _gui()
    gui.current_position = _point(0.5, 0.0)
    gui.current_buttons.a = True
    gui._update_line_drawing()

    gui.current_position = _point(0.05, 0.0)
    gui._update_line_drawing()

    assert gui.trial_started
    assert gui.is_running
    assert len(gui.drawn_line) == 1


def test_button_release_after_start_continues_existing_path():
    gui = _gui()
    gui.current_buttons.a = True
    gui._update_line_drawing()
    gui.current_position = _point(0.2, 0.0)
    gui._update_line_drawing()
    first_path_length = len(gui.drawn_line)

    gui.current_buttons.a = False
    gui._update_line_drawing()
    gui.current_position = _point(0.4, 0.0)
    gui._update_line_drawing()

    assert gui.trial_started
    assert gui.is_running
    assert len(gui.drawn_line) > first_path_length
    assert gui.drawn_line[0].x == 0.0


def test_endpoint_reached_stops_and_blocks_until_new_task():
    gui = _gui()
    gui.current_buttons.a = True
    gui._update_line_drawing()

    reached = Bool()
    reached.data = True
    gui._study_endpoint_reached(reached)

    assert not gui.is_running
    assert gui.trial_completion_latched

    gui.current_position = _point(0.0, 0.0)
    gui._update_line_drawing()
    assert not gui.is_running

    gui._start_point(_point(0.1, 0.0))
    assert not gui.trial_completion_latched
    assert not gui.trial_started
    assert gui.drawn_line == []


def test_endpoint_false_after_completion_allows_next_trial_start():
    gui = _gui()
    gui.current_buttons.a = True
    gui._update_line_drawing()

    reached = Bool()
    reached.data = True
    gui._study_endpoint_reached(reached)
    assert gui.trial_completion_latched

    gui.current_buttons.a = False
    gui._update_line_drawing()

    cleared = Bool()
    cleared.data = False
    gui._study_endpoint_reached(cleared)

    assert not gui.trial_completion_latched
    assert not gui.trial_started
    assert gui.drawn_line == []

    gui.current_position = gui.start_point
    gui.current_buttons.a = True
    gui._update_line_drawing()

    assert gui.trial_started
    assert gui.is_running
