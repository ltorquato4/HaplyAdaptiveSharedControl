#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from haply_msgs.msg import HaplyState

class HaplyStateSubscriber(Node):
    """ROS2 Node that subscribes to the haply_state topic and prints received data."""
    
    def __init__(self):
        super().__init__('state_subscriber_haply')
        
        # Subscriber to the haply_state topic
        self.subscription = self.create_subscription(
            HaplyState,
            'haply_state',
            self.state_callback,
            10  # Queue size
        )
        self.get_logger().info("HaplyStateSubscriber node has been started. new")
    
    def state_callback(self, msg):
        """Callback function to process received HaplyState messages."""
        self.get_logger().info(
            f"Received HaplyState:\n"
            f"  Position: [{msg.position.x:.3f}, {msg.position.y:.3f}, {msg.position.z:.3f}]\n"
            f"  Velocity: [{msg.velocity.x:.3f}, {msg.velocity.y:.3f}, {msg.velocity.z:.3f}]\n"
            f"  Quaternion: [{msg.quaternion.x:.3f}, {msg.quaternion.y:.3f}, {msg.quaternion.z:.3f}, {msg.quaternion.w:.3f}]\n"
            f"  Buttons: A = {msg.buttons.a}, B = {msg.buttons.b}, C = {msg.buttons.c}"
        )

def main(args=None):
    """Main function to initialize and run the subscriber node."""
    rclpy.init(args=args)
    node = HaplyStateSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()