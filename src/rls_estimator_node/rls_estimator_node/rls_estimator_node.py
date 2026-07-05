#!/usr/bin/env python3

import numpy as np

import rclpy

from rclpy.node import Node

from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String

from rls_estimator import RLSEstimator


class RLSEstimatorNode(Node):

    def __init__(self):

        super().__init__("rls_estimator_node")

        self.cursor = None
        self.goal = None
        self.start_point = None

        self.prev_pos = None
        self.prev_vel = None

        self.prev_time = None

        self.initialized = False

        self.rls = RLSEstimator()

        #
        # Subscribers
        #

        self.create_subscription(
            Point,
            "/experiment_cursor_position",
            self.cursor_callback,
            10
        )

        self.create_subscription(
            Point,
            "/study_end_point",
            self.goal_callback,
            10
        )

        self.create_subscription(
            Point,
            "/study_start_point",
            self.start_callback,
            10
        )

        #
        # Publishers
        #

        self.kh_pub = self.create_publisher(
            Float64MultiArray,
            "/estimation/K_h",
            10
        )

        self.uh_pub = self.create_publisher(
            Vector3,
            "/estimation/u_h",
            10
        )

        self.status_pub = self.create_publisher(
            String,
            "/estimator_status",
            10
        )

        #
        # 100 Hz
        #

        self.timer = self.create_timer(
            0.01,
            self.update_estimator
        )

        self.get_logger().info(
            "RLS Estimator Started"
        )

    #########################################################

    def start_callback(self, msg):

        self.start_point = msg

        if not self.initialized:

            self.rls.initialize_from_start_point(msg)

            self.initialized = True

            self.get_logger().info(
                "Initialized from first start point"
            )

    def cursor_callback(self, msg):
        self.cursor = msg

    def goal_callback(self, msg):
        self.goal = msg

    #########################################################

    def update_estimator(self):

        if self.cursor is None:
            return

        if self.goal is None:
            return

        now = (
            self.get_clock()
            .now()
            .nanoseconds
            * 1e-9
        )

        pos = np.array([
            self.cursor.x,
            self.cursor.y
        ])

        #
        # first sample
        #

        if self.prev_time is None:

            self.prev_time = now
            self.prev_pos = pos
            self.prev_vel = np.zeros(2)

            return

        dt = now - self.prev_time

        if dt <= 1e-6:
            return

        #
        # velocity
        #

        vel = (
            pos - self.prev_pos
        ) / dt

        #
        # acceleration
        #

        acc = (
            vel - self.prev_vel
        ) / dt

        #
        # goal error
        #

        ex = self.goal.x - self.cursor.x
        ey = self.goal.y - self.cursor.y

        vx = vel[0]
        vy = vel[1]

        ax = acc[0]
        ay = acc[1]

        #
        # RLS update
        #

        self.rls.update(
            ex,
            vx,
            ey,
            vy,
            ax,
            ay
        )

        kh = self.rls.get_matrix()

        #
        # estimated human control
        #

        state = np.array([
            ex,
            vx,
            ey,
            vy
        ])

        uh = kh @ state

        #
        # Publish Kh
        #

        kh_msg = Float64MultiArray()

        kh_msg.data = kh.flatten().tolist()

        self.kh_pub.publish(kh_msg)

        #
        # Publish Uh
        #

        uh_msg = Vector3()

        uh_msg.x = float(uh[0])
        uh_msg.y = float(uh[1])
        uh_msg.z = 0.0

        self.uh_pub.publish(uh_msg)

        #
        # Status
        #

        status = String()

        status.data = "ok"

        self.status_pub.publish(status)

        #
        # save previous
        #

        self.prev_pos = pos
        self.prev_vel = vel
        self.prev_time = now


def main(args=None):

    rclpy.init(args=args)

    node = RLSEstimatorNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()