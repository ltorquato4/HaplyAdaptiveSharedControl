import random

from haply_msgs.msg import StudyAbortRequest, StudyCursor, StudyStartRequest
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
    node.start_reached_radius = 0.1
    node.min_phase_duration_s = 0.0
    node.max_trial_duration_s = 0.0
    node.timeout_policy = "retry"
    node.endpoint_dwell_s = 1.0
    node.inter_trial_delay_s = 1.0
    node.controller_modes = ["adaptive", "fixed"]
    node.repetitions = 1
    node.order_strategy = "fixed"
    node.order_seed = 1
    node._schedule_rng = random.Random(node.order_seed)
    node.tasks = node._expand_session_tasks()
    node.task_index = 0
    node.trial_id = 0
    node.session_id = "test-session"
    node.loop_tasks = False
    node.session_finished = False
    node.is_running = True
    node.input_valid = True
    node.cursor_position = StudyPoint(1.0, 0.0, 0.0)
    node.endpoint_latched = False
    node.endpoint_entered_time = None
    node.start_gate_reached = True
    node.abort_requested_for_current_trial = False
    node.last_rollout_time = 0.0
    node.rollout_due_time = None
    node.start_pub = FakePublisher()
    node.end_pub = FakePublisher()
    node.phase_pub = FakePublisher()
    node.mode_pub = FakePublisher()
    node.dwell_progress_pub = FakePublisher()
    node.running_pub = FakePublisher()
    node.endpoint_pub = FakePublisher()
    node.task_pub = FakePublisher()
    node.trial_state_pub = FakePublisher()
    node.get_logger = lambda: FakeLogger()
    return node


def test_endpoint_dwell_requires_continuous_second(monkeypatch):
    node = _generator()
    clock = [10.0]
    monkeypatch.setattr(scenario_generator_node.time, "monotonic", lambda: clock[0])

    node._tick()
    assert not node.endpoint_latched
    assert node.endpoint_entered_time == 10.0
    assert node.dwell_progress_pub.messages[-1].progress == 0.0

    clock[0] = 10.9
    node._tick()
    assert not node.endpoint_latched

    clock[0] = 11.0
    node._tick()

    assert node.endpoint_latched
    assert node.rollout_due_time == 12.0
    assert node.dwell_progress_pub.messages[-1].progress == 1.0


def test_start_request_requires_current_trial_id(monkeypatch):
    node = _generator()
    node.is_running = False
    node.start_gate_reached = False
    node.cursor_position = StudyPoint(0.0, 0.0, 0.0)

    monkeypatch.setattr(scenario_generator_node.time, "monotonic", lambda: 10.0)

    node._start_requested(StudyStartRequest(session_id="test-session", trial_id=1))
    assert not node.is_running
    assert node.trial_state_pub.messages[-1].reason == "stale_trial_id"

    node._start_requested(StudyStartRequest(session_id="test-session", trial_id=0))
    assert node.is_running
    assert node.trial_state_pub.messages[-1].state == "RUNNING"


def test_cursor_from_old_trial_cannot_replace_current_cursor():
    node = _generator()
    node.cursor_position = StudyPoint(0.25, 0.0, 0.0)

    node._cursor_position(
        StudyCursor(
            session_id="test-session",
            trial_id=1,
            input_valid=True,
        )
    )

    assert node.cursor_position == StudyPoint(0.25, 0.0, 0.0)


def test_invalid_cursor_for_current_trial_aborts_running_trial():
    node = _generator()
    node._cursor_position(
        StudyCursor(
            session_id="test-session",
            trial_id=0,
            input_valid=False,
        )
    )

    assert not node.is_running
    assert node.trial_state_pub.messages[-1].state == "ABORTED"
    assert node.trial_state_pub.messages[-1].reason == "input_lost"


def test_scenario_keeps_current_task_during_delay(monkeypatch):
    node = _generator()
    node.endpoint_latched = True
    node.rollout_due_time = 13.0
    monkeypatch.setattr(scenario_generator_node.time, "monotonic", lambda: 12.0)

    node._tick()

    assert node.task_index == 0
    assert node.rollout_due_time == 13.0


def test_scenario_rolls_out_after_delay_and_resets_endpoint(monkeypatch):
    node = _generator()
    node.endpoint_latched = True
    node.rollout_due_time = 13.0
    monkeypatch.setattr(scenario_generator_node.time, "monotonic", lambda: 13.0)

    node._tick()

    assert node.task_index == 1
    assert not node.endpoint_latched
    assert node.rollout_due_time is None
    assert node.task_pub.messages[-1].controller_mode == "adaptive"
    assert node.task_pub.messages[-1].start_point.x == 1.0


def test_leaving_endpoint_resets_dwell(monkeypatch):
    node = _generator()
    clock = [10.0]
    monkeypatch.setattr(scenario_generator_node.time, "monotonic", lambda: clock[0])
    node._tick()
    clock[0] = 10.9
    node.cursor_position = StudyPoint(0.5, 0.0, 0.0)
    node._tick()
    assert node.endpoint_entered_time is None
    clock[0] = 11.0
    node.cursor_position = StudyPoint(1.0, 0.0, 0.0)
    node._tick()
    clock[0] = 12.0
    node._tick()
    assert node.endpoint_latched


def test_rollout_clears_dwell_timestamp():
    node = _generator()
    node.endpoint_entered_time = 10.0
    node._rollout_next_segment()
    assert node.endpoint_entered_time is None


def test_schedule_covers_each_phase_segment_mode_combination():
    node = _generator()
    combinations = {
        (task.phase, task.segment_index, task.controller_mode) for task in node.tasks
    }

    assert len(node.tasks) == 30
    assert len(combinations) == 30


def test_schedule_completes_one_controller_mode_before_switching():
    node = _generator()

    modes = [task.controller_mode for task in node.tasks]

    assert modes[:15] == ["adaptive"] * 15
    assert modes[15:] == ["fixed"] * 15


def test_seeded_random_schedule_shuffles_phases_and_segments_reproducibly():
    first = _generator()
    first.order_strategy = "seeded_random"
    first.order_seed = 20260721
    first._schedule_rng = random.Random(first.order_seed)
    first.tasks = first._expand_session_tasks()

    second = _generator()
    second.order_strategy = "seeded_random"
    second.order_seed = 20260721
    second._schedule_rng = random.Random(second.order_seed)
    second.tasks = second._expand_session_tasks()

    assert first.tasks == second.tasks
    for offset in (0, 15):
        phase_order = [first.tasks[offset + (index * 5)].phase for index in range(3)]
        assert set(phase_order) == set(ScenarioGenerator.PHASES)
        segment_orders = [
            [task.segment_index for task in first.tasks[offset + index * 5:offset + (index + 1) * 5]]
            for index in range(3)
        ]
        assert all(set(order) == set(range(5)) for order in segment_orders)
        assert any(order != list(range(5)) for order in segment_orders)


def test_final_task_finishes_session_and_rejects_new_requests():
    node = _generator()
    node.task_index = len(node.tasks) - 1
    node.trial_id = node.task_index

    node._rollout_next_segment()

    assert node.session_finished
    assert node.trial_state_pub.messages[-1].state == "SESSION_FINISHED"
    node._start_requested(
        StudyStartRequest(session_id="test-session", trial_id=node.trial_id)
    )
    assert node.trial_state_pub.messages[-1].state == "SESSION_FINISHED"


def test_start_request_rejects_stale_session_id():
    node = _generator()
    node.is_running = False
    node._start_requested(StudyStartRequest(session_id="old-session", trial_id=0))
    assert node.trial_state_pub.messages[-1].reason == "stale_session_id"


def test_timeout_retry_stops_trial_and_allows_a_fresh_start(monkeypatch):
    node = _generator()
    node.max_trial_duration_s = 1.0
    node.last_rollout_time = 10.0
    monkeypatch.setattr(scenario_generator_node.time, "monotonic", lambda: 11.0)

    node._tick()

    assert not node.is_running
    assert node.trial_state_pub.messages[-1].state == "ABORTED"
    assert node.trial_state_pub.messages[-1].reason == "timeout"
    node.cursor_position = StudyPoint(0.0, 0.0, 0.0)
    node._start_requested(StudyStartRequest(session_id="test-session", trial_id=0))
    assert node.is_running


def test_timeout_advance_rolls_out_next_task(monkeypatch):
    node = _generator()
    node.timeout_policy = "advance"
    node.max_trial_duration_s = 1.0
    node.last_rollout_time = 10.0
    monkeypatch.setattr(scenario_generator_node.time, "monotonic", lambda: 11.0)

    node._tick()

    assert node.task_index == 1
    assert node.trial_state_pub.messages[-1].state == "READY"


def test_abort_request_stops_only_the_matching_active_trial():
    node = _generator()

    node._abort_requested(
        StudyAbortRequest(session_id="test-session", trial_id=0, reason="gui_closed")
    )

    assert not node.is_running
    assert node.trial_state_pub.messages[-1].state == "ABORTED"
    assert node.trial_state_pub.messages[-1].reason == "gui_closed"


def test_abort_request_before_start_blocks_a_queued_start():
    node = _generator()
    node.is_running = False
    node.cursor_position = StudyPoint(0.0, 0.0, 0.0)

    node._abort_requested(
        StudyAbortRequest(session_id="test-session", trial_id=0, reason="gui_closed")
    )
    node._start_requested(StudyStartRequest(session_id="test-session", trial_id=0))

    assert not node.is_running
    assert node.trial_state_pub.messages[-1].state == "ABORTED"
