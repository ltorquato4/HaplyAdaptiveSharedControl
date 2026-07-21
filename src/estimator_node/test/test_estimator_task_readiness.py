from haply_msgs.msg import StudyTask

from estimator_node.estimator.rls_estimator import RLSEstimator
from estimator_node.estimator_node import RLSEstimatorNode


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def test_study_task_configures_estimator_and_marks_it_ready():
    node = object.__new__(RLSEstimatorNode)
    node.start_point = None
    node.goal = None
    node.prev_pos = object()
    node.prev_vel = object()
    node.prev_time = object()
    node.initialized = False
    node.task_received = False
    node.rls = RLSEstimator()
    node.ready_pub = FakePublisher()
    task = StudyTask(session_id="session", trial_id=3)
    task.start_point.x, task.start_point.y = -0.08, -0.08
    task.end_point.x, task.end_point.y = 0.08, 0.08

    node.study_task_callback(task)

    assert node.task_received
    assert node.initialized
    assert (node.start_point.x, node.start_point.y) == (-0.08, -0.08)
    assert (node.goal.x, node.goal.y) == (0.08, 0.08)
    assert node.prev_pos is None
    assert node.prev_vel is None
    assert node.prev_time is None
    node._publish_ready()
    assert node.ready_pub.messages[-1].data is True
