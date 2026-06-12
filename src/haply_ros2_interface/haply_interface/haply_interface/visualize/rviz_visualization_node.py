#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from haply_msgs.msg import HaplyState
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, Point, TransformStamped

from tf2_ros import TransformBroadcaster

import sys, time


class RvizVisualizationNode(Node):
    """ROS2 node: visualize Haply position and orientation in RViz"""

    def __init__(self):
        super().__init__("rviz_visualization_node")

        self.start_time = time.time()

        # Parameters 
        self.declare_parameter("position_scale", 10.0)
        self.position_scale = float(self.get_parameter("position_scale").value)

        self.declare_parameter("publish_frequency", 100.0)
        self.publish_frequency = float(self.get_parameter("publish_frequency").value)

        # Fixed frames 
        self.world_frame = "world"
        self.handle_frame = "handle_link"

        # State 
        self.latest_haply_state = HaplyState()

        # Subscriber 
        self.create_subscription(
            HaplyState,
            "haply_state",
            self.haply_state_callback,
            10
        )

        # Publisher
        self.marker_pub = self.create_publisher(Marker, "visualization_marker", 10)

        # TF broadcaster
        self.tf_br = TransformBroadcaster(self)

        # Timers
        self.timer = self.create_timer(1.0 / self.publish_frequency, self.publish_all)

        self.get_logger().info(
            f"RViz visualization node running with freq123={self.publish_frequency} Hz, scale={self.position_scale}."
        )

    # Callbacks & helpers 
    def haply_state_callback(self, msg: HaplyState):
        self.latest_haply_state = msg

    def scale_position(self, p: Point) -> Point:
        s = self.position_scale
        return Point(x=p.x * s, y=p.y * s, z=p.z * s)

    #  Publishers 
    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.world_frame
        t.child_frame_id = self.handle_frame

        # Translation (scaled)
        scaled = self.scale_position(self.latest_haply_state.position)
        t.transform.translation.x = float(scaled.x)
        t.transform.translation.y = float(scaled.y)
        t.transform.translation.z = float(scaled.z)

        # Orientation directly from Haply
        t.transform.rotation = self.latest_haply_state.quaternion

        self.tf_br.sendTransform(t)

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "haply_device"
        marker.id = 0
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD

        marker.pose.position = self.scale_position(self.latest_haply_state.position)
        marker.pose.orientation = self.latest_haply_state.quaternion

        marker.mesh_resource = "package://haply_meshes/meshes/ArrowY.stl"
        marker.mesh_use_embedded_materials = False  

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        marker.color.r = 0.2
        marker.color.g = 0.2
        marker.color.b = 0.2
        marker.color.a = 1.0

        self.marker_pub.publish(marker)

        # Uptime print
        elapsed_time = int(time.time() - self.start_time)
        sys.stdout.write(f"\rviz_visualization_node is running: {elapsed_time} s")
        sys.stdout.flush()

    def publish_all(self):
        self.publish_tf()
        self.publish_marker()


def main(args=None):
    rclpy.init(args=args)
    node = RvizVisualizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
