from std_msgs.msg import Bool
from study_orchestration.scenario_generator_node import ScenarioGenerator
from study_orchestration.scenario_logic import StudyPoint

from study_orchestration import scenario_generator_node


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class FakeLogger:
    def info(self, _msg):
        pass


def _generator():
    node = object.__new__(ScenarioGenerator)
    node.points = [
        StudyPoint(0.0, 0.0, 0.0),
        StudyPoint(1.0, 0.0, 0.0),
        StudyPoint(1.0, 1.0, 0.0),
        StudyPoint(0.0, 1.0, 0.0),
        StudyPoint(-1.0, 0.0, 0.0),
    ]
    node.endpoint_reached_radius = 0.1
    node.min_phase_duration_s = 0.0
    node.inter_trial_delay_s = 1.0
    node.controller_modes = ["adaptive", "fixed"]
    node.segment_index = 0
    node.phase_index = 0
    node.mode_index = 0
    node.is_running = True
    node.cursor_position = StudyPoint(1.0, 0.0, 0.0)
    node.endpoint_latched = False
    node.start_gate_reached = True
    node.last_rollout_time = 0.0
    node.rollout_due_time = None
    node.start_pub = FakePublisher()
    node.end_pub = FakePublisher()
    node.phase_pub = FakePublisher()
    node.mode_pub = FakePublisher()
    node.endpoint_pub = FakePublisher()
    node.get_logger = lambda: FakeLogger()
    return node


def test_endpoint_reached_schedules_delay_before_rollout(monkeypatch):
    node = _generator()
    monkeypatch.setattr(scenario_generator_node.time, "monotonic", lambda: 10.0)

    node._tick()

    assert node.segment_index == 0
    assert node.endpoint_latched
    assert node.rollout_due_time == 11.0
    assert node.endpoint_pub.messages[-1].data is True
    assert node.start_pub.messages == []
    assert node.end_pub.messages == []


def test_running_true_trusts_gui_start_gate(monkeypatch):
    node = _generator()
    node.is_running = False
    node.start_gate_reached = False
    node.cursor_position = StudyPoint(1.0, 0.0, 0.0)

    running = Bool()
    running.data = True
    monkeypatch.setattr(scenario_generator_node.time, "monotonic", lambda: 10.0)

    node._is_running(running)
    node._tick()

    assert node.endpoint_latched
    assert node.rollout_due_time == 11.0
    assert node.endpoint_pub.messages[-1].data is True


def test_scenario_keeps_current_task_during_delay(monkeypatch):
    node = _generator()
    node.endpoint_latched = True
    node.rollout_due_time = 13.0
    monkeypatch.setattr(scenario_generator_node.time, "monotonic", lambda: 12.0)

    node._tick()

    assert node.segment_index == 0
    assert node.start_pub.messages == []
    assert node.end_pub.messages == []
    assert node.endpoint_pub.messages[-1].data is True


def test_scenario_rolls_out_after_delay_and_resets_endpoint(monkeypatch):
    node = _generator()
    node.endpoint_latched = True
    node.rollout_due_time = 13.0
    monkeypatch.setattr(scenario_generator_node.time, "monotonic", lambda: 13.0)

    node._tick()

    assert node.segment_index == 1
    assert not node.endpoint_latched
    assert node.rollout_due_time is None
    assert node.start_pub.messages[-1].x == 1.0
    assert node.end_pub.messages[-1].y == 1.0
    assert node.endpoint_pub.messages[-1].data is False


def test_phase_advances_after_all_five_segments():
    node = _generator()
    node.phase_index = 0

    for expected_segment in range(1, 5):
        node._rollout_next_segment()
        assert node.segment_index == expected_segment
        assert node.phase_index == 0

    node._rollout_next_segment()

    assert node.segment_index == 5
    assert node.phase_index == 1
    assert node.start_pub.messages[-1].x == 0.0
    assert node.end_pub.messages[-1].x == 1.0
