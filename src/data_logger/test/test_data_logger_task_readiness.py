import csv

from data_logger.data_logger_node import DataLoggerNode
from haply_msgs.msg import StudyCursor, StudySession, StudyTask, StudyTrialState


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeLogger:
    def info(self, _message):
        pass

    def error(self, _message):
        pass


def _node(tmp_path):
    node = object.__new__(DataLoggerNode)
    node.base_directory = str(tmp_path)
    node.save_directory = None
    node.csv_logger = None
    node.session_metadata = {}
    node.trial_metadata = {}
    node.latest_sample = {}
    node.controller_parameters = None
    node.session_received = False
    node.task_received = False
    node.pending_task = None
    node.trial_active = False
    node.pending_trial_state = None
    node.recording = False
    node.flush_counter = 0
    node.next_write_deadline = None
    node.missed_cycle_count = 0
    node.cursor_sample_sequence = 0
    node.typed_cursor_received = False
    node.attempt_counts = {}
    node.active_attempt = None
    node.config = type(
        "Config",
        (),
        {
            "file_prefix": "trajectory",
            "flush_interval": 100,
            "log_rate_hz": 100.0,
        },
    )()
    node.ready_pub = FakePublisher()
    node.get_logger = lambda: FakeLogger()
    node._now = lambda: 123.0
    node._monotonic_now = lambda: 10.0
    node._directory_timestamp = lambda: "2026-07-23_16-42-08Z"
    return node


def _task():
    task = StudyTask(
        session_id="session", trial_id=3, phase="careful", controller_mode="fixed"
    )
    task.start_point.x, task.start_point.y = -0.08, -0.08
    task.end_point.x, task.end_point.y = 0.08, 0.08
    return task


def _session(task):
    message = StudySession(
        schema_version=3,
        session_id="session",
        participant_id="P03",
        input_source="haply",
        controller_family="mpc",
        order_strategy="seeded_random",
        order_seed=7,
        estimator_state_policy="persist_session",
        max_control_amplitude=10.0,
    )
    message.schedule = [task]
    return message


def _state(state, reason=""):
    return StudyTrialState(
        session_id="session",
        trial_id=3,
        state=state,
        reason=reason,
    )


def test_session_and_task_populate_complete_metadata(tmp_path):
    node = _node(tmp_path)
    task = _task()
    node.study_task_callback(task)
    node._publish_ready()
    assert node.ready_pub.messages[-1].data is False

    node.study_session_callback(_session(task))

    assert node.task_received
    assert node.trial_metadata["session_id"] == "session"
    assert node.trial_metadata["trial_id"] == 3
    assert node.trial_metadata["study_phase"] == "careful"
    assert node.trial_metadata["study_controller_mode"] == "fixed"
    assert node.session_metadata["order_seed"] == 7
    assert node.session_metadata["participant_id"] == "P03"
    session_directory = tmp_path / "P03_2026-07-23_16-42-08Z"
    assert node.save_directory == str(session_directory)
    assert (session_directory / "session_manifest.json").exists()
    node._publish_ready()
    assert node.ready_pub.messages[-1].data is True


def test_session_directory_collision_gets_numeric_suffix(tmp_path):
    node = _node(tmp_path)

    first = node._create_session_directory("P03")
    second = node._create_session_directory("P03")

    assert first.endswith("P03_2026-07-23_16-42-08Z")
    assert second.endswith("P03_2026-07-23_16-42-08Z_02")


def test_retry_attempts_get_distinct_files_and_outcomes(tmp_path):
    node = _node(tmp_path)
    task = _task()
    node.study_session_callback(_session(task))
    node.study_task_callback(task)

    node.trial_state_callback(_state("RUNNING"))
    first_filename = node.active_attempt["filename"]
    node.trial_state_callback(_state("ABORTED", "timeout"))
    node.trial_state_callback(_state("RUNNING"))
    second_filename = node.active_attempt["filename"]

    assert first_filename == "trial_000003_attempt_001.csv"
    assert second_filename == "trial_000003_attempt_002.csv"
    with (
        tmp_path / "P03_2026-07-23_16-42-08Z" / "trial_attempts.csv"
    ).open() as stream:
        rows = list(csv.DictReader(stream))
    assert rows[0]["participant_id"] == "P03"
    assert rows[0]["outcome"] == "ABORTED"
    assert rows[0]["reason"] == "timeout"


def test_rows_include_source_and_monotonic_timing_with_missed_cycles(tmp_path):
    node = _node(tmp_path)
    task = _task()
    node.study_session_callback(_session(task))
    node.study_task_callback(task)
    node.trial_state_callback(_state("RUNNING"))

    cursor = StudyCursor(
        session_id="session", trial_id=3, input_valid=True
    )
    cursor.stamp.sec = 21
    cursor.stamp.nanosec = 250_000_000
    cursor.position.x = 0.1
    cursor.position.y = -0.2
    node.study_cursor_callback(cursor)
    node._monotonic_now = lambda: 10.031
    node.write_row()
    node.stop_recording()

    path = (
        tmp_path
        / "P03_2026-07-23_16-42-08Z"
        / "trial_000003_attempt_001.csv"
    )
    with path.open() as stream:
        row = next(csv.DictReader(stream))
    assert row["participant_id"] == "P03"
    assert float(row["monotonic_timestamp"]) == 10.031
    assert float(row["cursor_timestamp"]) == 21.25
    assert int(row["cursor_sample_sequence"]) == 1
    assert int(row["missed_cycle_count"]) == 2


def test_retained_running_state_waits_for_matching_session_and_task(tmp_path):
    node = _node(tmp_path)
    node.trial_state_callback(_state("RUNNING"))

    task = _task()
    node.study_task_callback(task)
    assert node.recording is False

    node.study_session_callback(_session(task))

    assert node.trial_active is True
    assert node.recording is True
    assert node.active_attempt["attempt_id"] == 1
