#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Point, Vector3
from haply_msgs.msg import HaplyControl
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, Float64MultiArray, String


class ControlSystemTest(Node):
    def __init__(self):
        super().__init__("control_system_test")

        #
        # Publishers
        #

        self.study_running_pub = self.create_publisher(
            Bool,
            "/study_is_running",
            10,
        )

        self.mode_pub = self.create_publisher(
            String,
            "/study_controller_mode",
            10,
        )

        self.start_pub = self.create_publisher(
            Point,
            "/study_start_point",
            10,
        )

        self.goal_pub = self.create_publisher(
            Point,
            "/study_end_point",
            10,
        )

        self.cursor_pub = self.create_publisher(
            Point,
            "/experiment_cursor_position",
            10,
        )

        #
        # Subscribers
        #

        self.create_subscription(
            Vector3,
            "/control/U_a",
            self.control_callback,
            10,
        )

        self.create_subscription(
            String,
            "/control/K_a",
            self.parameter_callback,
            10,
        )

        self.create_subscription(
            HaplyControl,
            "/haply_target",
            self.force_callback,
            10,
        )

        self.create_subscription(
            Float64MultiArray,
            "/estimation/K_h",
            self.kh_callback,
            10,
        )

        self.create_subscription(
            Vector3,
            "/estimation/u_h",
            self.uh_callback,
            10,
        )

        #
        # Initial experiment setup
        #

        self.publish_initial_messages()

        #
        # Cursor simulation (100 Hz)
        #

        self.t = 0.0
        self.timer = self.create_timer(0.01, self.publish_cursor)

        self.get_logger().info("Test node started.")

    #########################################################

    def publish_initial_messages(self):

        running = Bool()
        running.data = True
        self.study_running_pub.publish(running)

        mode = String()
        mode.data = "adaptive"
        self.mode_pub.publish(mode)

        start = Point()
        start.x = 100.0
        start.y = 100.0
        self.start_pub.publish(start)

        goal = Point()
        goal.x = 700.0
        goal.y = 500.0
        self.goal_pub.publish(goal)

        self.get_logger().info(
            "Published start point, goal point, mode and running state."
        )

    #########################################################

    def publish_cursor(self):

        msg = Point()

        #
        # Smooth trajectory
        #

        msg.x = 100.0 + 300.0 * (1.0 - math.exp(-0.2 * self.t))
        msg.y = 100.0 + 150.0 * math.sin(0.5 * self.t)

        self.cursor_pub.publish(msg)

        self.t += 0.01

    #########################################################
    # Subscribers
    #########################################################

    def control_callback(self, msg):

        self.get_logger().info(
            f"Control U_a = ({msg.x:.3f}, {msg.y:.3f})"
        )

    def parameter_callback(self, msg):

        self.get_logger().info(
            f"K_a = {msg.data}"
        )

    def force_callback(self, msg):

        self.get_logger().info(
            f"Force = ({msg.force.x:.3f}, {msg.force.y:.3f})"
        )

    def kh_callback(self, msg):

        self.get_logger().info(
            f"K_h = {[round(v,3) for v in msg.data]}"
        )

    def uh_callback(self, msg):

        self.get_logger().info(
            f"U_h = ({msg.x:.3f}, {msg.y:.3f})"
        )


############################################################


def main(args=None):

    rclpy.init(args=args)

    node = ControlSystemTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()