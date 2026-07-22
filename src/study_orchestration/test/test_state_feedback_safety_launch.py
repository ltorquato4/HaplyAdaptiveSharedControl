"""Launch-level safety test for the state-feedback study stack.

This deliberately does not launch the Haply driver.  It verifies the ROS-level
safe-stop contract that is shared by mouse and hardware operation: when Mapper
input becomes stale, Scenario aborts the current trial, Controller publishes a
zero force command, and Estimator stops producing new estimates.
"""

import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import pytest
import rclpy
from geometry_msgs.msg import Vector3
from haply_msgs.msg import (
    HaplyControl,
    HaplyState,
    StudyStartRequest,
    StudyTask,
    StudyTrialState,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


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
                            "button_debounce_s": 0.01,
                            "input_timeout_s": 0.12,
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
                            "endpoint_dwell_s": 1.0,
                            "max_trial_duration_s": 0.0,
                        }
                    ],
                ),
                launch_ros.actions.Node(
                    package="control_node",
                    executable="state_feedback_control_node",
                    name="control_node",
                    parameters=[{"log_level": "WARN"}],
                ),
                launch_ros.actions.Node(
                    package="estimator_node",
                    executable="estimator_node",
                    name="estimator_node",
                    parameters=[{"log_level": "WARN"}],
                ),
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {},
    )


class TestStateFeedbackSafetyLaunch(unittest.TestCase):
    """Verify stale input always stops state feedback safely."""

    def setUp(self):
        rclpy.init()
        self.node = Node("state_feedback_safety_launch_test")
        self.haply_pub = self.node.create_publisher(HaplyState, "haply_state", 10)
        self.start_request_pub = self.node.create_publisher(
            StudyStartRequest, "study_start_requested", 10
        )
        self.tasks = []
        self.states = []
        self.force_commands = []
        self.estimates = []
        retained_state_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
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
        self.node.create_subscription(
            HaplyControl, "/haply_target", self.force_commands.append, 10
        )
        self.node.create_subscription(
            Vector3, "/estimation/u_h", self.estimates.append, 10
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
        self.fail("timed out waiting for state-feedback safety condition")

    def _publish_input(self, x, y, pressed=False, samples=4):
        for _ in range(samples):
            msg = HaplyState()
            msg.position.x = x
            msg.position.y = y
            msg.position.z = 0.0
            msg.quaternion.w = 1.0
            msg.buttons.a = pressed
            self.haply_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.01)
            time.sleep(0.02)

    @staticmethod
    def _is_zero_force(command):
        return (
            command.use_position is False
            and command.force.x == 0.0
            and command.force.y == 0.0
            and command.force.z == 0.0
        )

    def test_stale_input_aborts_and_zeros_state_feedback(self):
        self._spin_until(lambda: self.tasks)
        self._spin_until(lambda: self.haply_pub.get_subscription_count() >= 1)
        task = self.tasks[-1]

        # First edge calibrates; the second begins a valid state-feedback run.
        self._publish_input(0.0, 0.0, pressed=False)
        self._publish_input(0.0, 0.0, pressed=True)
        self._publish_input(0.0, 0.0, pressed=False)
        self._publish_input(
            task.start_point.x, task.start_point.y, pressed=False, samples=8
        )
        self._publish_input(task.start_point.x, task.start_point.y, pressed=True)
        self._publish_input(task.start_point.x, task.start_point.y, pressed=False)
        self.start_request_pub.publish(
            StudyStartRequest(session_id=task.session_id, trial_id=task.trial_id)
        )
        self._spin_until(lambda: any(state.state == "RUNNING" for state in self.states))

        # Let Controller and Estimator process at least one live cursor sample,
        # then stop input entirely. Mapper must invalidate it after 0.12 s.
        self._publish_input(task.start_point.x + 0.01, task.start_point.y, samples=8)
        self._spin_until(lambda: self.force_commands)
        self._spin_until(lambda: self.estimates)
        self._spin_until(
            lambda: any(
                state.state == "ABORTED" and state.reason == "input_lost"
                for state in self.states
            )
        )
        self._spin_until(
            lambda: self.force_commands and self._is_zero_force(self.force_commands[-1])
        )

        estimate_count = len(self.estimates)
        deadline = time.monotonic() + 0.25
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertEqual(len(self.estimates), estimate_count)

        # A retry of unchanged geometry must rebuild the stopped controller
        # and resume non-zero force feedback rather than merely reporting RUNNING.
        force_count = len(self.force_commands)
        self._publish_input(task.start_point.x, task.start_point.y, samples=8)
        self.start_request_pub.publish(
            StudyStartRequest(session_id=task.session_id, trial_id=task.trial_id)
        )
        self._spin_until(
            lambda: sum(state.state == "RUNNING" for state in self.states) >= 2
        )
        self._publish_input(task.start_point.x + 0.01, task.start_point.y, samples=8)
        self._spin_until(
            lambda: any(
                not self._is_zero_force(command)
                for command in self.force_commands[force_count:]
            )
        )
