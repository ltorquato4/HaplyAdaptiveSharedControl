#!/usr/bin/env python3

import numpy as np
import rclpy

from geometry_msgs.msg import Point, Vector3
from haply_msgs.msg import HaplyState
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
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
        self.estimator_running = False

        self.current_button_a_state = False
        self.endpoint_reached = False

        self.rls = RLSEstimator()

        self.create_subscription(Point, "/experiment_cursor_position", self.cursor_callback, 10)
        self.create_subscription(Point, "/study_end_point", self.goal_callback, 10)
        self.create_subscription(Point, "/study_start_point", self.start_callback, 10)
        self.create_subscription(HaplyState, "/haply_state", self.haply_state_callback, 10)
        self.create_subscription(Bool, "/study_endpoint_reached", self.endpoint_reached_callback, 10,)

        self.kh_pub = self.create_publisher(Float64MultiArray, "/estimation/K_h", 10)
        self.uh_pub = self.create_publisher(Vector3, "/estimation/u_h", 10)

        self.timer = self.create_timer(0.01, self.update_estimator)

        self.get_logger().info("RLS Estimator node started.")

    def haply_state_callback(self, msg: HaplyState):
        """Activate estimator when Button A is pressed."""
        self.current_button_a_state = msg.buttons.a

        if not self.endpoint_reached and self.current_button_a_state:
            if not self.estimator_running:

                self.estimator_running = True

                self.prev_time = None
                self.prev_pos = None
                self.prev_vel = None

                self.get_logger().debug("Button A pressed! Estimator activated.")
                
        elif self.endpoint_reached and not self.current_button_a_state:
            self.endpoint_reached = False

    def endpoint_reached_callback(self, msg: Bool):
        """Stop estimator when endpoint is reached."""
        if not self.endpoint_reached and msg.data:
            self.endpoint_reached = True
        
            if self.estimator_running:

                self.estimator_running = False

                self.get_logger().debug( "Endpoint reached message received. Estimator stopped." )
                

    def start_callback(self, msg):

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
        self.goal = msg
        self.get_logger().debug(f"Goal point updated: [{msg.x}, {msg.y}]")

    #########################################################

    def update_estimator(self):

        if not self.estimator_running:
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

            self.get_logger().debug("First sample recorded, initializing previous state variables.")

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


def main(args=None):

    rclpy.init(args=args)

    node = RLSEstimatorNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Shutting down RLS Estimator node.")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()