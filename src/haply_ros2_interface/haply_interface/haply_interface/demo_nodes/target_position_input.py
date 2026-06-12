#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3
from haply_msgs.msg import HaplyControl

class HaplyControlPublisher(Node):
    """ROS2 Node that publishes target positions to move the Haply device."""

    def __init__(self):
        super().__init__('target_position_input')

        # Publisher for sending HaplyControl messages
        self.publisher = self.create_publisher(HaplyControl, 'haply_target', 10)

        # Log initialization message
        self.get_logger().info("HaplyControlPublisher initialized. Enter 'x y z' coordinates to move the device.")

        # Start user input handling
        self.user_input_loop()

    def user_input_loop(self):
        """Continuously reads user input and publishes target positions."""
        while rclpy.ok():
            try:
                # Read user input
                user_input = input("Enter target position (x y z): ")
                
                # Split and convert input into float values
                x, y, z = map(float, user_input.split())

                # Create a HaplyControl message
                msg = HaplyControl()
                msg.use_position = True  # We are controlling position, not force
                msg.target_position = Point(x=x, y=y, z=z)
                msg.force = Vector3(x=0.0, y=0.0, z=0.0)  # Force is not used in position mode

                # Publish the message
                self.publisher.publish(msg)
                self.get_logger().info(f"Published target position: x={x:.3f}, y={y:.3f}, z={z:.3f}")

            except ValueError:
                self.get_logger().error("Invalid input. Please enter three numeric values separated by spaces.")

def main(args=None):
    """Main function to initialize and run the ROS 2 node."""
    rclpy.init(args=args)
    node = HaplyControlPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
