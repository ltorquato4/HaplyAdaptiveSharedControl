#!/usr/bin/env python3

import numpy as np
import rclpy

from geometry_msgs.msg import Point, Vector3
from haply_msgs.msg import StudyTask
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64MultiArray

from estimator_node.estimator.rls_estimator import RLSEstimator


class RLSEstimatorNode(Node):
    def __init__(self):

        super().__init__("estimator_node")

        #
        # Logging Setup
        #

        self.log_level = self.declare_parameter("log_level", "INFO").value

        log_levels = {
            "DEBUG": LoggingSeverity.DEBUG,
            "INFO": LoggingSeverity.INFO,
            "WARN": LoggingSeverity.WARN,
            "WARNING": LoggingSeverity.WARN,
            "ERROR": LoggingSeverity.ERROR,
            "FATAL": LoggingSeverity.FATAL,
        }

        self.get_logger().set_level(
            log_levels.get(str(self.log_level).upper(), LoggingSeverity.DEBUG)
        )

        self.cursor = None
        self.goal = None
        self.start_point = None

        self.prev_pos = None
        self.prev_vel = None
        self.prev_time = None

        self.initialized = False
        self.task_received = False
        self.study_is_running = False

        self.rls = RLSEstimator()

        self.create_subscription(
            Point, "/experiment_cursor_position", self.cursor_callback, 10
        )
        self.create_subscription(Point, "/study_end_point", self.goal_callback, 10)
        self.create_subscription(Point, "/study_start_point", self.start_callback, 10)
        self.create_subscription(
            StudyTask, "/study_task", self.study_task_callback, self._task_qos()
        )
        self.study_is_running_sub = self.create_subscription(
            Bool, "/study_is_running", self.study_is_running_callback, 10
        )

        self.kh_pub = self.create_publisher(Float64MultiArray, "/estimation/K_h", 10)
        self.uh_pub = self.create_publisher(Vector3, "/estimation/u_h", 10)
        self.ready_pub = self.create_publisher(
            Bool, "/study_estimator_ready",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE),
        )
        self.ready_timer = self.create_timer(0.5, self._publish_ready)

        self.timer = self.create_timer(0.01, self.update_estimator)

        self.get_logger().info("RLS Estimator node started.")

    def _publish_ready(self):
        self.ready_pub.publish(Bool(data=self.task_received))

    @staticmethod
    def _task_qos():
        return QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

    def study_task_callback(self, msg: StudyTask):
        """Apply a complete retained task before declaring estimator readiness."""
        self.start_point = msg.start_point
        self.goal = msg.end_point
        self.prev_pos = None
        self.prev_vel = None
        self.prev_time = None
        if not self.initialized:
            self.rls.initialize_from_start_point(self.start_point)
            self.initialized = True
        self.task_received = True

    def start_callback(self, msg):
        if self.task_received:
            return

        self.start_point = msg

        self.get_logger().debug(f"Start point updated: [{msg.x}, {msg.y}]")

        if not self.initialized:
            self.rls.initialize_from_start_point(msg)
            self.initialized = True

            self.get_logger().info("Initialized from first start point")

    def cursor_callback(self, msg):
        self.cursor = msg
        self.get_logger().debug(f"Cursor position received: [{msg.x}, {msg.y}]")

    def goal_callback(self, msg):
        if self.task_received:
            return
        self.goal = msg
        self.get_logger().debug(f"Goal point updated: [{msg.x}, {msg.y}]")

    #########################################################

    def update_estimator(self):

        if not self.study_is_running:
            return

        if self.cursor is None:
            return

        if self.goal is None:
            return

        now = self.get_clock().now().nanoseconds * 1e-9

        pos = np.array([self.cursor.x, self.cursor.y])

        #
        # first sample
        #

        if self.prev_time is None:

            self.prev_time = now
            self.prev_pos = pos
            self.prev_vel = np.zeros(2)

            self.get_logger().debug(
                "First sample recorded, initializing previous state variables."
            )

            return

        dt = now - self.prev_time

        if dt <= 1e-6:
            return

        #
        # velocity
        #

        vel = (pos - self.prev_pos) / dt

        #
        # acceleration
        #

        acc = (vel - self.prev_vel) / dt

        self.get_logger().debug(f"Computed kinematics - Vel: {vel}, Acc: {acc}")

        #
        # goal error
        #

        ex = self.goal.x - self.cursor.x
        ey = self.goal.y - self.cursor.y

        self.get_logger().debug(f"Goal error - ex: {ex}, ey: {ey}")

        vx = vel[0]
        vy = vel[1]

        ax = acc[0]
        ay = acc[1]

        #
        # RLS update
        #

        self.rls.update(ex, vx, ey, vy, ax, ay)

        kh = self.rls.get_matrix()

        self.get_logger().debug(f"RLS updated. K_h matrix computed: {kh.flatten().tolist()}")

        #
        # estimated human control
        #

        state = np.array([ex, vx, ey, vy])
        uh = kh @ state

        self.get_logger().debug(f"Estimated human control u_h: {uh}")

        #
        # Publish Kh
        #

        kh_msg = Float64MultiArray()
        kh_msg.data = kh.flatten().tolist()

        self.kh_pub.publish(kh_msg)

        self.get_logger().debug("Published K_h estimation.")

        #
        # Publish Uh
        #

        uh_msg = Vector3()
        uh_msg.x = float(uh[0])
        uh_msg.y = float(uh[1])
        uh_msg.z = 0.0

        self.uh_pub.publish(uh_msg)

        self.get_logger().debug(f"Published u_h vector: ({uh_msg.x}, {uh_msg.y}, {uh_msg.z})")

        #
        # save previous
        #

        self.prev_pos = pos
        self.prev_vel = vel
        self.prev_time = now

    def study_is_running_callback(self, msg: Bool):
        new_study_is_running = msg.data
        
        if self.study_is_running == new_study_is_running:
            return
        
        self.study_is_running = new_study_is_running
        self.get_logger().debug(f"Study is running: {self.study_is_running}")


def main(args=None):

    rclpy.init(args=args)

    node = RLSEstimatorNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Shutting down RLS Estimator node.")

    finally:
        node.destroy_node()
        # Launch shutdown may already have closed the shared ROS context.
        # Avoid turning an otherwise clean state-feedback safety stop into a
        # process failure during teardown.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
