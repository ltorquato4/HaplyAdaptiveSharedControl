"""Headless end-to-end test for GUI, Mapper, and Scenario Generator."""

import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import pytest
import rclpy
from geometry_msgs.msg import Point
from haply_msgs.msg import HaplyState, StudyTask, StudyTrialState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


pytest.importorskip("pygame", reason="GUI end-to-end test requires pygame")


@pytest.mark.launch_test
def generate_test_description():
    """Launch the participant GUI without hardware or a real display."""
    return (
        launch.LaunchDescription(
            [
                launch_ros.actions.Node(
                    package="study_orchestration",
                    executable="experiment_mapper",
                    name="experiment_mapper",
                    parameters=[
                        {
                            "mapping_mode": "anchored_delta",
                            "use_z_as_y": False,
                            "button_debounce_s": 0.01,
                            "input_timeout_s": 1.0,
                            "publish_hz": 100.0,
                        }
                    ],
                ),
                launch_ros.actions.Node(
                    package="study_orchestration",
                    executable="scenario_generator",
                    name="scenario_generator",
                    parameters=[
                        {
                            "publish_hz": 50.0,
                            "min_phase_duration_s": 0.0,
                            "endpoint_dwell_s": 0.15,
                            "inter_trial_delay_s": 0.0,
                            "start_reached_radius": 0.02,
                            "endpoint_reached_radius": 0.02,
                        }
                    ],
                ),
                launch_ros.actions.Node(
                    package="haply_study_gui",
                    executable="study_gui",
                    name="study_gui",
                    additional_env={
                        "SDL_VIDEODRIVER": "dummy",
                        "SDL_AUDIODRIVER": "dummy",
                        "PYGAME_HIDE_SUPPORT_PROMPT": "1",
                    },
                    parameters=[
                        {
                            "source": "haply",
                            "render_fps": 30.0,
                            "mode_overlay_duration_s": 0.05,
                        }
                    ],
                ),
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {},
    )


class TestGuiMouseEndToEnd(unittest.TestCase):
    """Verify that real GUI callbacks make the second press start a trial."""

    def setUp(self):
        rclpy.init()
        self.node = Node("gui_mouse_end_to_end_test")
        self.haply_pub = self.node.create_publisher(HaplyState, "haply_state", 10)
        self.ready_values = []
        self.tasks = []
        self.states = []
        retained_state_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.node.create_subscription(
            Bool,
            "study_mapping_ready",
            self.ready_values.append,
            retained_state_qos,
        )
        self.node.create_subscription(
            StudyTask, "study_task", self.tasks.append, retained_state_qos
        )
        self.node.create_subscription(
            StudyTrialState,
            "study_trial_state",
            self.states.append,
            retained_state_qos,
        )

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def _spin_until(self, predicate, timeout_s=4.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if predicate():
                return
        self.fail("timed out waiting for end-to-end GUI state")

    def _publish_input(self, x, y, pressed=False, samples=4):
        for _ in range(samples):
            msg = HaplyState()
            msg.position.x = x
            msg.position.y = y
            msg.quaternion.w = 1.0
            msg.buttons.a = pressed
            self.haply_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.01)
            time.sleep(0.02)

    def test_gui_calibration_start_dwell_and_rollout(self):
        self._spin_until(lambda: self.tasks)
        self._spin_until(lambda: self.haply_pub.get_subscription_count() >= 1)
        self._spin_until(lambda: self.ready_values)
        task = self.tasks[-1]
        # Let the participant GUI's render/executor loop consume the cached
        # task and initial mapping state before the first input edge.
        time.sleep(0.30)

        # First physical/mouse-equivalent click calibrates only.
        self._publish_input(0.0, 0.0, False)
        self._publish_input(0.0, 0.0, True)
        self._publish_input(0.0, 0.0, False)
        self._spin_until(lambda: any(message.data for message in self.ready_values))
        self.assertFalse(any(state.state == "RUNNING" for state in self.states))

        # Move to the mapped start and let the real GUI receive cursor updates.
        self._publish_input(task.start_point.x, task.start_point.y, False, samples=12)
        time.sleep(0.15)

        # Second edge goes Mapper -> GUI -> StudyStartRequest -> Scenario.
        self._publish_input(task.start_point.x, task.start_point.y, True)
        self._publish_input(task.start_point.x, task.start_point.y, False)
        self._spin_until(lambda: any(state.state == "RUNNING" for state in self.states))

        # The real GUI remains active while Scenario evaluates endpoint dwell.
        deadline = time.monotonic() + 0.35
        while time.monotonic() < deadline:
            self._publish_input(task.end_point.x, task.end_point.y)
        self._spin_until(lambda: any(state.state == "COMPLETED" for state in self.states))
        self._spin_until(lambda: len(self.tasks) >= 2)
        self.assertEqual(self.tasks[-1].trial_id, task.trial_id + 1)
