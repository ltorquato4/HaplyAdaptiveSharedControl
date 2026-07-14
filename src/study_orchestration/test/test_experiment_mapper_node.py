from geometry_msgs.msg import Point
from haply_msgs.msg import HaplyState
from std_msgs.msg import Bool
from study_orchestration.experiment_mapper_node import ExperimentMapper
from study_orchestration.mapper_logic import (
    AnchoredDeltaMapper,
    MappingConfig,
    TaskPoint,
)


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class FakeLogger:
    def info(self, _msg):
        pass


def _point(x, y, z=0.0):
    msg = Point()
    msg.x = x
    msg.y = y
    msg.z = z
    return msg


def _haply_state(x, y, z):
    msg = HaplyState()
    msg.position = _point(x, y, z)
    return msg


def _running(value):
    msg = Bool()
    msg.data = value
    return msg


def _mapper():
    node = object.__new__(ExperimentMapper)
    node.mapping_mode = "anchored_delta"
    node.anchored_mapper = AnchoredDeltaMapper(MappingConfig())
    node.latest_raw_position = None
    node.study_start_point = None
    node.is_running = False
    node.anchor_pending = True
    node.trial_anchor_locked = False
    node.cursor_pub = FakePublisher()
    node.get_logger = lambda: FakeLogger()
    return node


def test_mapper_does_not_publish_until_raw_pose_and_task_start_exist():
    node = _mapper()

    node._publish_latest_cursor()
    assert node.cursor_pub.messages == []

    node._haply_state(_haply_state(1.0, 2.0, 0.0))
    node._publish_latest_cursor()

    assert node.cursor_pub.messages == []


def test_mapper_publishes_pretrial_cursor_before_study_is_running():
    node = _mapper()
    node._study_start_point(_point(-0.08, -0.08))
    node._haply_state(_haply_state(1.0, 2.0, 0.0))

    node._publish_latest_cursor()

    assert not node.is_running
    assert not node.trial_anchor_locked
    assert node.cursor_pub.messages[-1].x == -0.08
    assert node.cursor_pub.messages[-1].y == -0.08


def test_mapper_locks_existing_anchor_when_trial_starts():
    node = _mapper()
    node._study_start_point(_point(-0.08, -0.08))
    node._haply_state(_haply_state(1.0, 2.0, 0.0))

    node._study_is_running(_running(True))
    node._haply_state(_haply_state(1.02, 2.0, 0.03))
    node._publish_latest_cursor()

    assert node.trial_anchor_locked
    assert node.cursor_pub.messages[-1].x > -0.08
    assert node.cursor_pub.messages[-1].y > -0.08


def test_mapper_does_not_recapture_anchor_when_trial_stops_without_new_task():
    node = _mapper()
    node._study_start_point(_point(-0.08, -0.08))
    node._haply_state(_haply_state(1.0, 2.0, 0.0))
    node._study_is_running(_running(True))
    raw_anchor = node.anchored_mapper.raw_anchor

    node._study_is_running(_running(False))
    node._haply_state(_haply_state(2.0, 2.0, 0.0))
    node._publish_latest_cursor()

    assert node.anchored_mapper.raw_anchor == raw_anchor


def test_mapper_resets_anchor_when_new_task_start_arrives():
    node = _mapper()
    node._study_start_point(_point(-0.08, -0.08))
    node._haply_state(_haply_state(1.0, 2.0, 0.0))
    node._study_is_running(_running(True))

    node._study_is_running(_running(False))
    node._haply_state(_haply_state(2.0, 2.0, 0.0))
    node._study_start_point(_point(0.08, -0.08))

    assert node.anchored_mapper.raw_anchor == TaskPoint(2.0, 2.0, 0.0)
    assert node.anchored_mapper.task_anchor == TaskPoint(0.08, -0.08, 0.0)
    assert not node.trial_anchor_locked
