from estimator_node.estimator.rls_estimator import RLSEstimator
from haply_msgs.msg import StudyCursor, StudySession, StudyTask, StudyTrialState

from estimator_node.estimator_node import RLSEstimatorNode


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeLogger:
    def error(self, _message):
        pass

    def debug(self, _message):
        pass


def _node():
    node = object.__new__(RLSEstimatorNode)
    node.start_point = None
    node.goal = None
    node.prev_pos = object()
    node.prev_vel = object()
    node.prev_time = object()
    node.initialized = False
    node.session_received = False
    node.task_received = False
    node.current_session_id = None
    node.pending_task = None
    node.cursor = None
    node.cursor_sample_time = None
    node.cursor_sample_id = 0
    node.processed_cursor_sample_id = 0
    node.typed_cursor_received = False
    node.current_trial_id = None
    node.trial_active = False
    node.pending_trial_state = None
    node.rls = RLSEstimator()
    node.ready_pub = FakePublisher()
    node.get_logger = lambda: FakeLogger()
    return node


def _task(session="session", trial=3):
    task = StudyTask(session_id=session, trial_id=trial)
    task.start_point.x, task.start_point.y = -0.08, -0.08
    task.end_point.x, task.end_point.y = 0.08, 0.08
    return task


def _session(session="session"):
    return StudySession(
        schema_version=1,
        session_id=session,
        estimator_state_policy="persist_session",
    )


def _state(state, session="session", trial=3):
    return StudyTrialState(
        session_id=session,
        trial_id=trial,
        state=state,
    )


def test_task_waits_for_session_then_configures_estimator_and_readiness():
    node = _node()
    task = _task()
    node.study_task_callback(task)
    node._publish_ready()
    assert node.ready_pub.messages[-1].data is False

    node.study_session_callback(_session())

    assert node.task_received
    assert node.initialized
    assert (node.start_point.x, node.start_point.y) == (-0.08, -0.08)
    assert (node.goal.x, node.goal.y) == (0.08, 0.08)
    assert node.prev_pos is None
    assert node.prev_vel is None
    assert node.prev_time is None
    node._publish_ready()
    assert node.ready_pub.messages[-1].data is True


def test_rls_persists_across_trials_and_resets_for_new_session():
    node = _node()
    node.study_session_callback(_session())
    node.study_task_callback(_task())
    node.rls.theta_x[0, 0] = 42.0

    node.study_task_callback(_task(trial=4))
    assert node.rls.theta_x[0, 0] == 42.0

    node.study_session_callback(_session("new-session"))
    assert node.rls.theta_x[0, 0] == 1.0
    assert node.task_received is False


def test_timestamped_cursor_samples_are_processed_once_and_update_rls():
    node = _node()
    node.kh_pub = FakePublisher()
    node.uh_pub = FakePublisher()
    node.study_session_callback(_session())
    node.study_task_callback(_task())
    node.study_trial_state_callback(_state("RUNNING"))
    initial = node.rls.get_matrix().copy()

    first = StudyCursor(session_id="session", trial_id=3, input_valid=True)
    first.stamp.sec = 1
    first.position.x = -0.08
    first.position.y = -0.08
    node.study_cursor_callback(first)
    node.update_estimator()
    node.update_estimator()
    assert node.kh_pub.messages == []

    second = StudyCursor(session_id="session", trial_id=3, input_valid=True)
    second.stamp.sec = 1
    second.stamp.nanosec = 10_000_000
    second.position.x = -0.079
    second.position.y = -0.08
    node.study_cursor_callback(second)
    node.update_estimator()

    assert len(node.kh_pub.messages) == 1
    assert node.cursor_sample_id == node.processed_cursor_sample_id
    assert (node.rls.get_matrix() != initial).any()


def test_retry_start_resets_only_kinematic_history():
    node = _node()
    node.study_session_callback(_session())
    node.study_task_callback(_task())
    node.prev_pos = object()
    node.prev_vel = object()
    node.prev_time = 1.0
    node.cursor = object()
    node.cursor_sample_time = 1.0
    node.rls.theta_x[0, 0] = 42.0

    node.study_trial_state_callback(_state("RUNNING"))

    assert node.prev_pos is None
    assert node.prev_vel is None
    assert node.prev_time is None
    assert node.cursor is None
    assert node.cursor_sample_time is None
    assert node.rls.theta_x[0, 0] == 42.0


def test_retained_running_state_waits_for_matching_session_and_task():
    node = _node()
    node.study_trial_state_callback(_state("RUNNING"))
    node.study_task_callback(_task())
    assert node.trial_active is False

    node.study_session_callback(_session())

    assert node.trial_active is True
    node.study_trial_state_callback(_state("ABORTED"))
    assert node.trial_active is False
