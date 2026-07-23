#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Point, Vector3
from haply_msgs.msg import HaplyControl
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String


class ControlSystemTest(Node):
    def __init__(self):
        super().__init__("control_system_test")

        self.get_logger().set_level(LoggingSeverity.INFO)

        #
        # Publishers
        #

        self.mode_pub = self.create_publisher(String, "/study_controller_mode", 10)
        self.start_pub = self.create_publisher(Point, "/study_start_point", 10)
        self.goal_pub = self.create_publisher(Point, "/study_end_point", 10)
        self.cursor_pub = self.create_publisher(Point, "/experiment_cursor_position", 10)

        #
        # Subscribers
        #

        self.create_subscription(Vector3, "/control/U_a", self.control_callback, 10)
        self.create_subscription(String, "/control/K_a", self.parameter_callback, 10)
        self.create_subscription(HaplyControl, "/haply_target", self.force_callback, 10)
        self.create_subscription(Float64MultiArray, "/estimation/K_h", self.kh_callback, 10)
        self.create_subscription(Vector3, "/estimation/u_h", self.uh_callback, 10)

        #
        # Initial experiment setup
        #

        self.publish_initial_messages()
        self.i = 0

        #
        # Cursor simulation (100 Hz)
        #

        self.t = 0.0
        self.timer = self.create_timer(0.01, self.publish_cursor)

        self.get_logger().info("Test node started.")

    #########################################################
    # Initial messages
    #########################################################

    def publish_initial_messages(self):
        mode = String(data="adaptive")
        self.mode_pub.publish(mode)
        self.get_logger().info(f"Published controller mode: {mode.data}")

        start = Point(x=100.0, y=100.0, z=0.0)
        self.start_pub.publish(start)
        self.get_logger().info(f"Published start point: ({start.x}, {start.y})")

        goal = Point(x=700.0, y=500.0, z=0.0)
        self.goal_pub.publish(goal)
        self.get_logger().info(f"Published goal point: ({goal.x}, {goal.y})")

    #########################################################
    # Cursor simulation
    #########################################################

    def publish_cursor(self):
        if self.i < 75:
            self.publish_initial_messages()
            self.i = self.i + 1
        else:
            msg = Point(
                x=100.0 + 300.0 * (1.0 - math.exp(-0.2 * self.t)),
                y=100.0 + 150.0 * math.sin(0.5 * self.t),
            )

            self.cursor_pub.publish(msg)
            self.get_logger().debug(f"Published cursor: ({msg.x:.3f}, {msg.y:.3f})")

            self.t += 0.01

    #########################################################
    # Subscribers
    #########################################################

    def control_callback(self, msg):
        self.get_logger().debug(
            f"Control U_a: ({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f})"
        )

    def parameter_callback(self, msg):
        self.get_logger().debug(f"Controller parameters K_a: {msg.data}")

    def force_callback(self, msg):
        self.get_logger().debug(
            "Haply force: "
            f"({msg.force.x:.3f}, {msg.force.y:.3f}, {msg.force.z:.3f})"
        )

    def kh_callback(self, msg):
        values = [round(value, 5) for value in msg.data]
        self.get_logger().debug(f"Estimated K_h: {values}")

    def uh_callback(self, msg):
        self.get_logger().debug(
            "Estimated human control U_h: "
            f"({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f})"
        )


############################################################


def main(args=None):
    rclpy.init(args=args)

    node = ControlSystemTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down test node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
