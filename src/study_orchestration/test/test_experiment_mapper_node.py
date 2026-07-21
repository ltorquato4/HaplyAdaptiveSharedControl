from geometry_msgs.msg import Point
from haply_msgs.msg import HaplyState, StudyCursor, StudyTask
from std_msgs.msg import Empty
from study_orchestration import experiment_mapper_node
from study_orchestration.experiment_mapper_node import ExperimentMapper
from study_orchestration.mapper_logic import AnchoredDeltaMapper, MappingConfig, TaskPoint


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class FakeLogger:
    def info(self, _msg):
        pass


class FakeClock:
    class Now:
        def to_msg(self):
            from builtin_interfaces.msg import Time

            return Time(sec=123, nanosec=456)

    def now(self):
        return self.Now()


def _point(x, y, z=0.0):
    msg = Point()
    msg.x, msg.y, msg.z = x, y, z
    return msg


def _haply_state(x, y, z, pressed=False):
    msg = HaplyState()
    msg.position = _point(x, y, z)
    msg.buttons.a = pressed
    return msg


def _mapper():
    node = object.__new__(ExperimentMapper)
    node.mapping_mode = "anchored_delta"
    node.anchored_mapper = AnchoredDeltaMapper(MappingConfig())
    node.latest_raw_position = None
    node.latest_mapped_position = None
    node.current_session_id = None
    node.current_trial_id = None
    node.mapping_ready = False
    node.previous_button_a = False
    node.last_button_edge_time = float("-inf")
    node.button_debounce_s = 0.05
    node.input_timeout_s = 0.2
    node.last_raw_update_time = float("-inf")
    node.input_valid = False
    node.task_anchor = TaskPoint(0.0, 0.0, 0.0)
    node.cursor_pub = FakePublisher()
    node.study_cursor_pub = FakePublisher()
    node.mapping_ready_pub = FakePublisher()
    node.button_pressed_pub = FakePublisher()
    node.input_valid_pub = FakePublisher()
    node.get_logger = lambda: FakeLogger()
    node.get_clock = lambda: FakeClock()
    return node


def test_mapper_does_not_publish_before_calibration():
    node = _mapper()
    node._haply_state(_haply_state(1.0, 2.0, 0.0))
    node._publish_latest_cursor()
    assert node.cursor_pub.messages == []


def test_first_rising_edge_captures_anchor_without_press_event(monkeypatch):
    node = _mapper()
    monkeypatch.setattr(experiment_mapper_node.time, "monotonic", lambda: 1.0)

    node._haply_state(_haply_state(1.0, 2.0, 0.0, pressed=True))
    node._publish_latest_cursor()

    assert node.mapping_ready
    assert node.anchored_mapper.raw_anchor == TaskPoint(1.0, 2.0, 0.0)
    assert node.anchored_mapper.task_anchor == TaskPoint(0.0, 0.0, 0.0)
    assert node.mapping_ready_pub.messages[-1].data is True
    assert node.button_pressed_pub.messages == []
    assert node.cursor_pub.messages[-1].x == 0.0
    assert node.cursor_pub.messages[-1].y == 0.0


def test_mapper_publishes_stamped_cursor_for_current_task(monkeypatch):
    node = _mapper()
    node._study_task(StudyTask(session_id="session-a", trial_id=7))
    monkeypatch.setattr(experiment_mapper_node.time, "monotonic", lambda: 1.0)
    node._haply_state(_haply_state(1.0, 2.0, 0.0, pressed=True))
    node._publish_latest_cursor()

    cursor = node.study_cursor_pub.messages[-1]
    assert isinstance(cursor, StudyCursor)
    assert (cursor.session_id, cursor.trial_id) == ("session-a", 7)
    assert (cursor.stamp.sec, cursor.stamp.nanosec) == (123, 456)
    assert cursor.input_valid
    assert (cursor.position.x, cursor.position.y) == (0.0, 0.0)


def test_held_button_does_not_emit_another_event(monkeypatch):
    node = _mapper()
    monkeypatch.setattr(experiment_mapper_node.time, "monotonic", lambda: 1.0)
    node._haply_state(_haply_state(0.0, 0.0, 0.0, pressed=True))
    node._haply_state(_haply_state(0.0, 0.0, 0.0, pressed=True))
    assert node.button_pressed_pub.messages == []


def test_release_then_second_press_emits_one_event(monkeypatch):
    node = _mapper()
    times = iter((1.0, 1.5, 2.0))
    monkeypatch.setattr(experiment_mapper_node.time, "monotonic", lambda: next(times))
    node._haply_state(_haply_state(0.0, 0.0, 0.0, pressed=True))
    node._haply_state(_haply_state(0.0, 0.0, 0.0, pressed=False))
    node._haply_state(_haply_state(0.0, 0.0, 0.0, pressed=True))
    assert len(node.button_pressed_pub.messages) == 1
    assert isinstance(node.button_pressed_pub.messages[0], Empty)


def test_button_bounce_inside_debounce_window_is_ignored(monkeypatch):
    node = _mapper()
    times = iter((1.0, 1.01, 1.02))
    monkeypatch.setattr(experiment_mapper_node.time, "monotonic", lambda: next(times))
    node._haply_state(_haply_state(0.0, 0.0, 0.0, pressed=True))
    node._haply_state(_haply_state(0.0, 0.0, 0.0, pressed=False))
    node._haply_state(_haply_state(0.0, 0.0, 0.0, pressed=True))
    assert node.button_pressed_pub.messages == []


def test_new_scenario_does_not_change_calibration_anchor(monkeypatch):
    node = _mapper()
    monkeypatch.setattr(experiment_mapper_node.time, "monotonic", lambda: 1.0)
    node._haply_state(_haply_state(1.0, 2.0, 0.0, pressed=True))
    raw_anchor = node.anchored_mapper.raw_anchor

    node._haply_state(_haply_state(2.0, 3.0, 0.0, pressed=False))

    assert node.anchored_mapper.raw_anchor == raw_anchor


def test_stale_input_is_marked_invalid_and_cursor_stops(monkeypatch):
    node = _mapper()
    node.mapping_ready = True
    node.latest_raw_position = TaskPoint(0.0, 0.0, 0.0)
    node.input_valid = True
    node.last_raw_update_time = 1.0
    monkeypatch.setattr(experiment_mapper_node.time, "monotonic", lambda: 1.3)

    node._publish_latest_cursor()

    assert node.input_valid_pub.messages[-1].data is False
    assert node.cursor_pub.messages == []
