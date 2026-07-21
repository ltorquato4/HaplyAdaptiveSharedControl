from haply_msgs.msg import StudyTask

from data_logger.data_logger_node import DataLoggerNode


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def test_study_task_populates_complete_logger_metadata():
    node = object.__new__(DataLoggerNode)
    node.trial_metadata = {}
    node.task_received = False
    node.csv_logger = object()
    node.ready_pub = FakePublisher()
    task = StudyTask(
        session_id="session", trial_id=3, phase="careful", controller_mode="fixed"
    )
    task.start_point.x, task.start_point.y = -0.08, -0.08
    task.end_point.x, task.end_point.y = 0.08, 0.08

    node.study_task_callback(task)

    assert node.task_received
    assert node.trial_metadata["session_id"] == "session"
    assert node.trial_metadata["trial_id"] == 3
    assert node.trial_metadata["study_phase"] == "careful"
    assert node.trial_metadata["study_controller_mode"] == "fixed"
    assert node.trial_metadata["start"] is task.start_point
    assert node.trial_metadata["end"] is task.end_point
    node._publish_ready()
    assert node.ready_pub.messages[-1].data is True
