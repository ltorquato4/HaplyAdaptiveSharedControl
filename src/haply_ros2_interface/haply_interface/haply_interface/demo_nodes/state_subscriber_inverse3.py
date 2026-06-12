#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from haply_msgs.msg import Inverse3State

class Inverse3StateSubscriber(Node):
    """ROS2 Node that subscribes to the inverse3_state topic and prints received data."""

    def __init__(self):
        super().__init__('state_subscriber_inverse3')
        self.subscription = self.create_subscription(
            Inverse3State,
            'inverse3_state',
            self.state_callback,
            10
        )
        self.get_logger().info("Inverse3StateSubscriber node has been started.")

    def state_callback(self, msg):
        """Callback function to process received Inverse3State messages."""
        self.get_logger().info(
            f"Received Inverse3State:\n"
            f"  Position: [{msg.position.x:.3f}, {msg.position.y:.3f}, {msg.position.z:.3f}]\n"
            f"  Velocity: [{msg.velocity.x:.3f}, {msg.velocity.y:.3f}, {msg.velocity.z:.3f}]"
        )

def main(args=None):
    rclpy.init(args=args)
    node = Inverse3StateSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
