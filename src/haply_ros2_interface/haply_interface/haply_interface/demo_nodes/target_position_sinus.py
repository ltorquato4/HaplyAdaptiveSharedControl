import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3
from haply_msgs.msg import HaplyControl
import math
import sys
import time

class TargetPublisherNode(Node):
    """ROS2 Node to publish sinusoidal target positions for Inverse3 control."""
    
    def __init__(self):
        super().__init__('target_position_sinus')

        self.publisher = self.create_publisher(HaplyControl, 'haply_target', 10)
        self.timer = self.create_timer(0.01, self.publish_target)
        self.get_logger().info('Target Publisher Node initialized')
        
        self.t = 0.0  # Time variable for sinusoidal motion
        self.magnitude = 0.1  # Amplitude of oscillation

        self.start_time = time.time()

    def publish_target(self):
        """Publishes a sinusoidal target position message to HaplyControl."""
        msg = HaplyControl()
        msg.use_position = True

        # Set sinusoidal position motion
        msg.target_position = Point()
        msg.target_position.x = 0.03 
        msg.target_position.y = -0.13 
        msg.target_position.z = 0.2 + self.magnitude * math.sin(0.2 * self.t)

        # Set force to zero (not used in this case)
        msg.force = Vector3(x=0.0, y=0.0, z=0.0)

        # Publish message
        self.publisher.publish(msg)
        #self.get_logger().info(f'Published target position: x={msg.target_position.x:.3f}, f'y={msg.target_position.y:.3f}, z={msg.target_position.z:.3f}')

        # Uptime print
        elapsed_time = int(time.time() - self.start_time)
        sys.stdout.write(f"\rhaply_driver_node is running: {elapsed_time} s")
        sys.stdout.flush()

        # Increment time step
        self.t += 0.1  # Adjust for faster/slower oscillation

def main(args=None):
    rclpy.init(args=args)
    node = TargetPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()
