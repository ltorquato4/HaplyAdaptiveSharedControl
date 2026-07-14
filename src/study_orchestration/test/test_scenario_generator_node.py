from std_msgs.msg import Bool
from study_orchestration.scenario_generator_node import ScenarioGenerator
from study_orchestration.scenario_logic import (
    ScenarioPath,
    StudyPoint,
    expand_scenario_tasks,
)

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
    paths = [
        ScenarioPath(StudyPoint(1.0, -1.0, 0.0), StudyPoint(1.0, 1.0, 0.0)),
        ScenarioPath(StudyPoint(1.0, 1.0, 0.0), StudyPoint(-1.0, 1.0, 0.0)),
        ScenarioPath(StudyPoint(-1.0, 1.0, 0.0), StudyPoint(-1.0, -1.0, 0.0)),
        ScenarioPath(StudyPoint(-1.0, -1.0, 0.0), StudyPoint(1.0, -1.0, 0.0)),
    ]
    node.tasks = expand_scenario_tasks(paths, ["fixed"])
    node.endpoint_reached_radius = 0.1
    node.min_phase_duration_s = 0.0
    node.inter_trial_delay_s = 1.0
    node.task_index = 0
    node.is_running = True
    node.cursor_position = StudyPoint(1.0, 1.0, 0.0)
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

    assert node.task_index == 0
    assert node.endpoint_latched
    assert node.rollout_due_time == 11.0
    assert node.endpoint_pub.messages[-1].data is True
    assert node.start_pub.messages == []
    assert node.end_pub.messages == []


def test_running_true_trusts_gui_start_gate(monkeypatch):
    node = _generator()
    node.is_running = False
    node.start_gate_reached = False
    node.cursor_position = StudyPoint(1.0, 1.0, 0.0)

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

    assert node.task_index == 0
    assert node.start_pub.messages == []
    assert node.end_pub.messages == []
    assert node.endpoint_pub.messages[-1].data is True


def test_scenario_rolls_out_after_delay_and_resets_endpoint(monkeypatch):
    node = _generator()
    node.endpoint_latched = True
    node.rollout_due_time = 13.0
    monkeypatch.setattr(scenario_generator_node.time, "monotonic", lambda: 13.0)

    node._tick()

    assert node.task_index == 1
    assert not node.endpoint_latched
    assert node.rollout_due_time is None
    assert node.start_pub.messages[-1].x == 1.0
    assert node.end_pub.messages[-1].y == 1.0
    assert node.phase_pub.messages[-1].data == "aggressive"
    assert node.mode_pub.messages[-1].data == "fixed"
    assert node.endpoint_pub.messages[-1].data is False


def test_scenario_keeps_phase_for_four_paths_before_advancing():
    node = _generator()

    for expected_task_index in range(1, 4):
        node._rollout_next_task()
        assert node.task_index == expected_task_index
        assert node.phase_pub.messages[-1].data == "aggressive"

    node._rollout_next_task()

    assert node.task_index == 4
    assert node.phase_pub.messages[-1].data == "normal"
    assert node.mode_pub.messages[-1].data == "fixed"


def test_scenario_wraps_after_all_twelve_tasks():
    node = _generator()

    for _ in range(12):
        node._rollout_next_task()

    assert node.task_index == 0
    assert node.start_pub.messages[-1].x == 1.0
    assert node.end_pub.messages[-1].x == 1.0
    assert node.phase_pub.messages[-1].data == "aggressive"
    assert node.mode_pub.messages[-1].data == "fixed"


def test_initial_task_definition_matches_task_zero():
    node = _generator()

    node._publish_task_definition()

    assert node.start_pub.messages[-1].x == 1.0
    assert node.start_pub.messages[-1].y == -1.0
    assert node.end_pub.messages[-1].x == 1.0
    assert node.end_pub.messages[-1].y == 1.0
    assert node.phase_pub.messages[-1].data == "aggressive"
    assert node.mode_pub.messages[-1].data == "fixed"
