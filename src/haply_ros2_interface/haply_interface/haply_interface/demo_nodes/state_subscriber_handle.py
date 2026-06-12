#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from haply_msgs.msg import HandleState

class HandleStateSubscriber(Node):
    """ROS2 Node that subscribes to the handle_state topic and prints received data."""
    
    def __init__(self):
        super().__init__('state_subscriber_handle')
        self.subscription = self.create_subscription(
            HandleState,
            'handle_state',
            self.state_callback,
            10
        )
        self.get_logger().info("HandleStateSubscriber node has been started.")
    
    def state_callback(self, msg):
        self.get_logger().info(
            f"Received HandleState:\n"
            f"  Quaternion: [{msg.quaternion.x:.3f}, {msg.quaternion.y:.3f}, {msg.quaternion.z:.3f}, {msg.quaternion.w:.3f}]\n"
            f"  Buttons: A = {msg.buttons.a}, B = {msg.buttons.b}, C = {msg.buttons.c}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = HandleStateSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()