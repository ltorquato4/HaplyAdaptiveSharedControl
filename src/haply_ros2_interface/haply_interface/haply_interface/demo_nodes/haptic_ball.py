#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math

from geometry_msgs.msg import Point, Vector3
from haply_msgs.msg import HaplyState, HaplyControl
from visualization_msgs.msg import Marker


class HaplyForceController(Node):
    """Haptic ball demo"""

    def __init__(self):
        super().__init__("haptic_ball")

        # Parameters 
        self.declare_parameter("stiffness", 200.0)  # Haptic stiffness
        self.declare_parameter("position_scale", 10.0)  # Visualization scale

        self.stiffness = float(self.get_parameter("stiffness").value)
        self.position_scale = float(self.get_parameter("position_scale").value)

        # Virtual sphere
        self.sphere_center = [-0.07, -0.2, 0.2]
        self.sphere_radius = 0.08

        # Subscribers
        self.state_subscriber = self.create_subscription(
            HaplyState,
            'haply_state',
            self.state_callback,
            10
        )

        # Publishers
        self.force_publisher = self.create_publisher(HaplyControl, 'haply_target', 10)
        self.marker_publisher = self.create_publisher(Marker, 'visualization_marker', 10)

        # Timer to continuously republish the marker (e.g. 10 Hz)
        self.marker_timer = self.create_timer(1/10, self.publish_sphere_marker)

        self.get_logger().info(
            f"Haply Force Controller initialized: stiffness={self.stiffness}, scale={self.position_scale}"
        )

    def state_callback(self, msg):
        """Receives HaplyState and computes the force to apply"""

        # Extract device position
        device_position = [msg.position.x, msg.position.y, msg.position.z]

        # Compute the haptic force
        force = self.compute_haptic_force(device_position)

        # Create the force control message
        control_msg = HaplyControl()
        control_msg.use_position = False  # force control
        control_msg.force = Vector3(x=force[0], y=force[1], z=force[2])
        control_msg.target_position = Point(x=0.0, y=0.0, z=0.0)  # unused

        # Publish
        self.force_publisher.publish(control_msg)
        self.get_logger().debug(
            f"Published Force: x={force[0]:.3f}, y={force[1]:.3f}, z={force[2]:.3f}"
        )

    def compute_haptic_force(self, device_position):
        """Force based on virtual sphere interaction"""

        distance = math.sqrt(
            sum([(device_position[i] - self.sphere_center[i]) ** 2 for i in range(3)])
        )

        if distance > self.sphere_radius:
            return [0.0, 0.0, 0.0]
        else:
            direction = [
                (device_position[i] - self.sphere_center[i]) / self.sphere_radius
                for i in range(3)
            ]
            force = [
                direction[i] * (self.sphere_radius - distance) * self.stiffness
                for i in range(3)
            ]
            return force

    def publish_sphere_marker(self):
        """Publishes a scaled sphere marker to RViz"""

        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "haply_sphere"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Position (scaled for RViz visualization)
        marker.pose.position.x = self.sphere_center[0] * self.position_scale
        marker.pose.position.y = self.sphere_center[1] * self.position_scale
        marker.pose.position.z = self.sphere_center[2] * self.position_scale
        marker.pose.orientation.w = 1.0

        # Scale = diameter, also scaled
        marker.scale.x = self.sphere_radius * 2.0 * self.position_scale
        marker.scale.y = self.sphere_radius * 2.0 * self.position_scale
        marker.scale.z = self.sphere_radius * 2.0 * self.position_scale

        # Color (red, semi-transparent)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.5

        self.marker_publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = HaplyForceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
