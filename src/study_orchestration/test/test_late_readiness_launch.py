"""Verify late Estimator and Logger receive Scenario's retained StudyTask."""

import time
import unittest

import launch
import launch.actions
import launch_ros.actions
import launch_testing
import pytest
import rclpy
from haply_msgs.msg import StudyTask
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


@pytest.mark.launch_test
def generate_test_description():
    task_qos = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )
    scenario = launch_ros.actions.Node(
        package="study_orchestration",
        executable="scenario_generator",
        name="scenario_generator",
        parameters=[
            {
                "require_estimator_ready": True,
                "require_logger_ready": True,
                "publish_hz": 50.0,
            }
        ],
    )
    late_consumers = launch.actions.TimerAction(
        period=0.5,
        actions=[
            launch_ros.actions.Node(
                package="estimator_node",
                executable="estimator_node",
                name="estimator_node",
            ),
            launch_ros.actions.Node(
                package="data_logger",
                executable="data_logger_node",
                name="data_logger_node",
                parameters=[{"save_directory": "/tmp/study-readiness-test"}],
            ),
        ],
    )
    return launch.LaunchDescription([scenario, late_consumers, launch_testing.actions.ReadyToTest()]), {"task_qos": task_qos}


class TestLateReadinessLaunch(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = Node("late_readiness_launch_test")
        self.tasks = []
        self.system_ready = []
        task_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.node.create_subscription(StudyTask, "study_task", self.tasks.append, task_qos)
        self.node.create_subscription(Bool, "study_system_ready", self.system_ready.append, task_qos)

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_late_nodes_receive_task_and_unlock_system(self):
        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.tasks and any(message.data for message in self.system_ready):
                break

        self.assertTrue(self.tasks)
        self.assertTrue(any(message.data for message in self.system_ready))
