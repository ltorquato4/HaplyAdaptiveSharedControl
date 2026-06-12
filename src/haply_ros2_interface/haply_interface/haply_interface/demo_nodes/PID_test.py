#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3
from haply_msgs.msg import HaplyControl


class HaplyAutoControlPublisher(Node):
    """ROS2 node that alternates between two predefined positions and publishes them."""

    def __init__(self):
        super().__init__('PID_test')

        # Parameters
        self.declare_parameter("interval", 2.0)
        self.interval = float(self.get_parameter("interval").value)

        # Publisher
        self.publisher = self.create_publisher(HaplyControl, 'haply_target', 10)

        # Predefined positions (alternating)
        self.positions = [
            Point(x=-0.08, y=-0.15, z=-0.03),
            Point(x= 0.23, y=-0.16, z= 0.15),
        ]
        self.current_index = 0

        # Timer to publish targets
        self.timer = self.create_timer(self.interval, self.publish_next_position)

        self.get_logger().info(
            f"Initialized. Alternating every {self.interval:.1f}s between two target positions."
        )

    def publish_next_position(self):
        """Publish the next predefined target position."""
        target_position = self.positions[self.current_index]

        msg = HaplyControl()
        msg.use_position = True
        msg.target_position = target_position
        msg.force = Vector3(x=0.0, y=0.0, z=0.0)

        self.publisher.publish(msg)
        self.get_logger().info(
            f"Published target: x={target_position.x:.3f}, "
            f"y={target_position.y:.3f}, z={target_position.z:.3f}"
        )

        self.current_index = (self.current_index + 1) % len(self.positions)


def main(args=None):
    rclpy.init(args=args)
    node = HaplyAutoControlPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
