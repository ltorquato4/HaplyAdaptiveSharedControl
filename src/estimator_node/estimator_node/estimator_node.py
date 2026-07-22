#!/usr/bin/env python3

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Vector3
from haply_msgs.msg import StudyCursor, StudySession, StudyTask, StudyTrialState
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
        self.cursor_sample_time = None
        self.cursor_sample_id = 0
        self.processed_cursor_sample_id = 0
        self.typed_cursor_received = False
        self.goal = None
        self.start_point = None
        self.current_trial_id = None

        self.prev_pos = None
        self.prev_vel = None
        self.prev_time = None

        self.initialized = False
        self.session_received = False
        self.task_received = False
        self.current_session_id = None
        self.pending_task = None
        self.trial_active = False
        self.pending_trial_state = None

        self.rls = RLSEstimator()

        self.create_subscription(
            Point, "/experiment_cursor_position", self.cursor_callback, 10
        )
        self.create_subscription(
            StudyCursor, "/study_cursor", self.study_cursor_callback, self._state_qos()
        )
        self.create_subscription(Point, "/study_end_point", self.goal_callback, 10)
        self.create_subscription(Point, "/study_start_point", self.start_callback, 10)
        self.create_subscription(
            StudySession,
            "/study_session",
            self.study_session_callback,
            self._retained_state_qos(),
        )
        self.create_subscription(
            StudyTask,
            "/study_task",
            self.study_task_callback,
            self._retained_state_qos(),
        )
        self.create_subscription(
            StudyTrialState,
            "/study_trial_state",
            self.study_trial_state_callback,
            self._retained_state_qos(),
        )

        self.kh_pub = self.create_publisher(Float64MultiArray, "/estimation/K_h", 10)
        self.uh_pub = self.create_publisher(Vector3, "/estimation/u_h", 10)
        self.ready_pub = self.create_publisher(
            Bool, "/study_estimator_ready", self._retained_state_qos()
        )
        self.ready_timer = self.create_timer(0.5, self._publish_ready)

        self.timer = self.create_timer(0.01, self.update_estimator)

        self.get_logger().info(
            "RLS estimator started; K_h is an effective closed-loop model, "
            "not a direct human-force measurement."
        )

    def _publish_ready(self):
        self.ready_pub.publish(Bool(data=self.session_received and self.task_received))

    @staticmethod
    def _retained_state_qos():
        return QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

    @staticmethod
    def _state_qos():
        return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)

    def study_session_callback(self, msg: StudySession):
        """Reset learning at session boundaries and retain it across trials."""
        session_id = str(msg.session_id)
        if msg.estimator_state_policy != "persist_session":
            self.get_logger().error(
                f"Unsupported estimator state policy: {msg.estimator_state_policy}"
            )
            return

        if session_id != self.current_session_id:
            self.current_session_id = session_id
            self.rls = RLSEstimator()
            self.initialized = False
            self.task_received = False
            self.start_point = None
            self.goal = None
            self.current_trial_id = None
            self.cursor = None
            self.cursor_sample_time = None
            self.cursor_sample_id = 0
            self.processed_cursor_sample_id = 0
            self.typed_cursor_received = False
            self.trial_active = False
            if (
                self.pending_trial_state is not None
                and str(self.pending_trial_state.session_id) != session_id
            ):
                self.pending_trial_state = None
            self._reset_kinematics()

        self.session_received = True
        if (
            self.pending_task is not None
            and str(self.pending_task.session_id) == self.current_session_id
        ):
            pending_task = self.pending_task
            self.pending_task = None
            self._apply_task(pending_task)

    def study_task_callback(self, msg: StudyTask):
        """Apply a task only after its retained session definition is known."""
        if not self.session_received or str(msg.session_id) != self.current_session_id:
            self.pending_task = msg
            return
        self._apply_task(msg)

    def _reset_kinematics(self):
        self.prev_pos = None
        self.prev_vel = None
        self.prev_time = None

    def _apply_task(self, msg: StudyTask):
        self.start_point = msg.start_point
        self.goal = msg.end_point
        self.current_trial_id = int(msg.trial_id)
        self.cursor = None
        self.cursor_sample_time = None
        self._reset_kinematics()
        if not self.initialized:
            self.rls.initialize_from_start_point(self.start_point)
            self.initialized = True
        self.task_received = True
        self._apply_pending_trial_state()

    def study_trial_state_callback(self, msg: StudyTrialState):
        """Use the identified Scenario lifecycle as the estimator run gate."""
        self.pending_trial_state = msg
        self._apply_pending_trial_state()

    def _apply_pending_trial_state(self):
        msg = self.pending_trial_state
        if msg is None or not self.session_received or not self.task_received:
            return
        if (
            str(msg.session_id) != self.current_session_id
            or int(msg.trial_id) != self.current_trial_id
        ):
            if (
                str(msg.session_id) == self.current_session_id
                and self.current_trial_id is not None
                and int(msg.trial_id) < self.current_trial_id
            ):
                self.pending_trial_state = None
            return

        active = str(msg.state).upper() in {"RUNNING", "DWELL"}
        if active and not self.trial_active:
            # A retry can reuse the same StudyTask. Derivative history is local
            # to one continuous movement and must never span the stopped gap.
            self._reset_kinematics()
            self.cursor = None
            self.cursor_sample_time = None
        self.trial_active = active
        self.pending_trial_state = None
        self.get_logger().debug(
            f"Study trial state: {msg.state}; active={self.trial_active}"
        )

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
        if self.typed_cursor_received:
            return
        self.cursor = msg
        self.cursor_sample_time = None
        self.cursor_sample_id += 1
        self.get_logger().debug(f"Cursor position received: [{msg.x}, {msg.y}]")

    def study_cursor_callback(self, msg: StudyCursor):
        """Accept each valid, task-identified cursor sample exactly once."""
        if (
            not self.session_received
            or str(msg.session_id) != self.current_session_id
            or int(msg.trial_id) != self.current_trial_id
        ):
            return
        self.typed_cursor_received = True
        if not msg.input_valid:
            self.cursor = None
            return
        self.cursor = msg.position
        stamp = float(msg.stamp.sec) + float(msg.stamp.nanosec) * 1e-9
        self.cursor_sample_time = stamp if stamp > 0.0 else None
        self.cursor_sample_id += 1

    def goal_callback(self, msg):
        if self.task_received:
            return
        self.goal = msg
        self.get_logger().debug(f"Goal point updated: [{msg.x}, {msg.y}]")

    #########################################################

    def update_estimator(self):

        if not self.trial_active:
            return

        if self.cursor is None:
            return

        if self.cursor_sample_id == self.processed_cursor_sample_id:
            return

        if self.goal is None:
            return

        now = self.cursor_sample_time
        if now is None:
            now = self.get_clock().now().nanoseconds * 1e-9
        self.processed_cursor_sample_id = self.cursor_sample_id

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

        self.get_logger().debug(
            f"RLS updated. K_h matrix computed: {kh.flatten().tolist()}"
        )

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

        self.get_logger().debug(
            f"Published u_h vector: ({uh_msg.x}, {uh_msg.y}, {uh_msg.z})"
        )

        #
        # save previous
        #

        self.prev_pos = pos
        self.prev_vel = vel
        self.prev_time = now


def main(args=None):

    rclpy.init(args=args)

    node = RLSEstimatorNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        if rclpy.ok():
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
