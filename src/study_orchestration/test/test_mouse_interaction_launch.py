"""Headless launch test for the mouse-equivalent study interaction.

The test publishes the same HaplyState messages produced by mouse simulation:
first button edge calibrates, a later edge requests a start at the mapped start
point, and continuous endpoint input completes the dwell before task rollout.
"""

import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import pytest
import rclpy
from geometry_msgs.msg import Point
from haply_msgs.msg import (
    HaplyState,
    StudyButtonPress,
    StudyCursor,
    StudyStartRequest,
    StudyTask,
    StudyTrialState,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


@pytest.mark.launch_test
def generate_test_description():
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
                            "scale_x": 1.0,
                            "scale_y": 1.0,
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
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {},
    )


class TestMouseInteractionLaunch(unittest.TestCase):
    """Exercise the Mapper/Scenario graph with simulated mouse input."""

    def setUp(self):
        rclpy.init()
        self.node = Node("mouse_interaction_launch_test")
        self.haply_pub = self.node.create_publisher(HaplyState, "haply_state", 10)
        self.start_pub = self.node.create_publisher(
            StudyStartRequest, "study_start_requested", 10
        )
        self.mapping_ready = False
        self.mapping_ready_messages = 0
        self.button_events = 0
        self.tasks = []
        self.states = []
        self.cursors = []
        self.study_cursors = []
        retained_state_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        state_history_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.node.create_subscription(
            Bool,
            "study_mapping_ready",
            self._mapping_ready,
            retained_state_qos,
        )
        self.node.create_subscription(
            StudyButtonPress, "study_button_pressed", self._button_pressed, 10
        )
        self.node.create_subscription(
            StudyTask, "study_task", self.tasks.append, retained_state_qos
        )
        self.node.create_subscription(
            StudyTrialState,
            "study_trial_state",
            self.states.append,
            state_history_qos,
        )
        self.node.create_subscription(
            Point, "experiment_cursor_position", self.cursors.append, 10
        )
        self.node.create_subscription(
            StudyCursor, "study_cursor", self.study_cursors.append, 10
        )

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def _button_pressed(self, _msg):
        self.button_events += 1

    def _mapping_ready(self, msg):
        self.mapping_ready = bool(msg.data)
        self.mapping_ready_messages += 1

    def _spin_until(self, predicate, timeout_s=3.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if predicate():
                return
        self.fail("timed out waiting for study interaction state")

    def _publish_input(self, x, y, pressed=False, samples=3):
        for _ in range(samples):
            msg = HaplyState()
            msg.position.x = x
            msg.position.y = y
            msg.position.z = 0.0
            msg.quaternion.w = 1.0
            msg.buttons.a = pressed
            self.haply_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.01)
            # Mapper is a separate process; retain each level long enough for
            # DDS delivery and its rising-edge detector to observe it.
            time.sleep(0.02)

    def test_calibrate_start_dwell_and_rollout(self):
        self._spin_until(lambda: len(self.tasks) == 1)
        self._spin_until(lambda: self.haply_pub.get_subscription_count() >= 1)
        self._spin_until(lambda: self.mapping_ready_messages >= 1)
        first_task = self.tasks[-1]

        # First edge: neutral calibration. It must not become a start event.
        self._publish_input(0.0, 0.0, pressed=False)
        self._publish_input(0.0, 0.0, pressed=True)
        self._publish_input(0.0, 0.0, pressed=False)
        self._spin_until(lambda: self.mapping_ready)
        self.assertEqual(self.button_events, 0)

        # Anchored-delta mapping: raw displacement reaches the task start.
        self._publish_input(first_task.start_point.x, first_task.start_point.y)
        self._spin_until(
            lambda: (
                self.cursors
                and abs(self.cursors[-1].x - first_task.start_point.x) < 0.01
                and abs(self.cursors[-1].y - first_task.start_point.y) < 0.01
            )
        )
        self._spin_until(
            lambda: (
                self.study_cursors
                and self.study_cursors[-1].session_id == first_task.session_id
                and self.study_cursors[-1].trial_id == first_task.trial_id
                and self.study_cursors[-1].input_valid
            )
        )

        # Second edge is the post-calibration mouse click; send its matching
        # GUI-equivalent ID-bearing request after Mapper emits the event.
        self._publish_input(first_task.start_point.x, first_task.start_point.y, True)
        self._publish_input(first_task.start_point.x, first_task.start_point.y, False)
        self._spin_until(lambda: self.button_events == 1)
        self.start_pub.publish(
            StudyStartRequest(
                session_id=first_task.session_id,
                trial_id=first_task.trial_id,
            )
        )
        self._spin_until(lambda: any(state.state == "RUNNING" for state in self.states))

        # Endpoint must be held continuously long enough to complete the task.
        deadline = time.monotonic() + 0.30
        while time.monotonic() < deadline:
            self._publish_input(first_task.end_point.x, first_task.end_point.y)
            rclpy.spin_once(self.node, timeout_sec=0.01)
        self._spin_until(
            lambda: any(state.state == "COMPLETED" for state in self.states)
        )
        self._spin_until(lambda: len(self.tasks) >= 2)
        self.assertEqual(self.tasks[-1].trial_id, first_task.trial_id + 1)
        self.assertEqual(self.tasks[-1].session_id, first_task.session_id)
