"""Launch-level lifecycle and stale-input test for the dedicated MPC node."""

import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import pytest
import rclpy
from haply_msgs.msg import HaplyControl, StudyCursor, StudyTask, StudyTrialState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


TEST_DOMAIN_ID = 74


@pytest.mark.launch_test
def generate_test_description():
    """Launch the production MPC executable without hardware."""
    return (
        launch.LaunchDescription(
            [
                launch.actions.SetEnvironmentVariable(
                    name="ROS_DOMAIN_ID",
                    value=str(TEST_DOMAIN_ID),
                ),
                launch_ros.actions.Node(
                    package="control_node",
                    executable="mpc_control_node",
                    name="control_node",
                    parameters=[
                        {
                            "log_level": "WARN",
                            "cursor_timeout_s": 0.12,
                        }
                    ],
                ),
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {},
    )


class TestMpcSafetyLaunch(unittest.TestCase):
    """Verify identified lifecycle, timeout zeroing, and same-task retry."""

    def setUp(self):
        rclpy.init(domain_id=TEST_DOMAIN_ID)
        self.node = Node("mpc_safety_launch_test")
        retained_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.task_pub = self.node.create_publisher(
            StudyTask, "/study_task", retained_qos
        )
        self.state_pub = self.node.create_publisher(
            StudyTrialState, "/study_trial_state", retained_qos
        )
        self.cursor_pub = self.node.create_publisher(
            StudyCursor, "/study_cursor", 10
        )
        self.readiness = []
        self.forces = []
        self.node.create_subscription(
            Bool,
            "/study_controller_ready",
            self.readiness.append,
            retained_qos,
        )
        self.node.create_subscription(
            HaplyControl,
            "/haply_target",
            self.forces.append,
            10,
        )

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def _spin_until(self, predicate, timeout_s=30.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if predicate():
                return
        self.fail("timed out waiting for MPC safety condition")

    @staticmethod
    def _zero_force(message):
        return (
            message.use_position is False
            and message.force.x == 0.0
            and message.force.y == 0.0
            and message.force.z == 0.0
        )

    def _publish_state(self, state):
        message = StudyTrialState()
        message.session_id = "mpc-test-session"
        message.trial_id = 4
        message.state = state
        self.state_pub.publish(message)

    def _publish_cursor(self, x, y, count=4):
        for _index in range(count):
            message = StudyCursor()
            message.session_id = "mpc-test-session"
            message.trial_id = 4
            message.stamp = self.node.get_clock().now().to_msg()
            message.position.x = x
            message.position.y = y
            message.input_valid = True
            self.cursor_pub.publish(message)
            rclpy.spin_once(self.node, timeout_sec=0.01)
            time.sleep(0.015)

    def test_typed_lifecycle_timeout_and_retry(self):
        task = StudyTask()
        task.session_id = "mpc-test-session"
        task.trial_id = 4
        task.start_point.x = -0.08
        task.start_point.y = -0.08
        task.end_point.x = 0.08
        task.end_point.y = 0.08
        task.phase = "normal"
        task.controller_mode = "fixed"
        self.task_pub.publish(task)
        self._spin_until(lambda: self.readiness and self.readiness[-1].data)

        self._publish_state("RUNNING")
        self._publish_cursor(-0.08, -0.08)
        self._spin_until(
            lambda: any(not self._zero_force(force) for force in self.forces)
        )

        self._spin_until(
            lambda: self.forces and self._zero_force(self.forces[-1]),
            timeout_s=3.0,
        )

        self._publish_state("ABORTED")
        force_count = len(self.forces)
        self._publish_state("RUNNING")
        self._publish_cursor(-0.07, -0.08)
        self._spin_until(
            lambda: any(
                not self._zero_force(force)
                for force in self.forces[force_count:]
            )
        )
