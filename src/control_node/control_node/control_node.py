import math
import threading
import pygame

import rclpy
from geometry_msgs.msg import Point, Vector3
from haply_msgs.msg import HaplyControl, HaplyState
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray, String

from control_node.mpc_controller.adaptive_mpc_controller import AdaptiveMpcController
from control_node.mpc_controller.mpc_controller import MpcController
from control_node.state_feedback_controller.adaptive_state_feedback_controller import (
    AdaptiveStateFeedbackController,
)
from control_node.state_feedback_controller.state_feedback_controller import (
    StateFeedbackController,
)


def draw_arrow(surface, color, start, end, arrow_size=10):
    """Helper function to draw an arrow using Pygame."""
    pygame.draw.line(surface, color, start, end, 3)
    
    # Calculate the angle for the arrowhead
    angle = math.atan2(end[1] - start[1], end[0] - start[0])
    
    # Calculate arrowhead points
    p1 = (
        end[0] - arrow_size * math.cos(angle - math.pi / 6),
        end[1] - arrow_size * math.sin(angle - math.pi / 6)
    )
    p2 = (
        end[0] - arrow_size * math.cos(angle + math.pi / 6),
        end[1] - arrow_size * math.sin(angle + math.pi / 6)
    )
    
    # Draw the arrowhead
    if start != end:
        pygame.draw.polygon(surface, color, [end, p1, p2])


class ControlNode(Node):
    def __init__(self):
        super().__init__("control_node")

        # Control Parameters
        self.dt = self.declare_parameter("delta_time", 0.01).value
        self.use_mpc_controller = self.declare_parameter("use_mpc_controller", True).value
        self.controller_mode = "adaptive" if self.declare_parameter("adaptive_control_enabled", False).value else "fixed"
        self.max_control_amplitude = self.declare_parameter("max_control_amplitude", 10.0).value
        self.max_velocity_amplitude = self.declare_parameter("max_velocity_amplitude", 10.0).value
        self.acceleration_to_force_factor = self.declare_parameter("acceleration_to_force_factor", 0.1).value

        # MPC specific Parameters
        self.prediction_horizon = self.declare_parameter("prediction_horizon", 10).value

        # GUI-Experiment parameters
        self.x_bounds_limit = self.declare_parameter("x_bounds", 400.0).value
        self.y_bounds_limit = self.declare_parameter("y_bounds", 700.0).value

        # Logging
        self.log_level = self.declare_parameter("log_level", "DEBUG").value

        log_levels = {
            "DEBUG": LoggingSeverity.DEBUG,
            "INFO": LoggingSeverity.INFO,
            "WARN": LoggingSeverity.WARN,
            "WARNING": LoggingSeverity.WARN,
            "ERROR": LoggingSeverity.ERROR,
            "FATAL": LoggingSeverity.FATAL,
        }

        self.get_logger().set_level(log_levels.get(str(self.log_level).upper(), LoggingSeverity.DEBUG))

        self.get_logger().info("Control node started.")
        self.get_logger().debug(f"Configuration: dt={self.dt}, use_mpc_controller={self.use_mpc_controller}, controller_mode={self.controller_mode}, prediction_horizon={self.prediction_horizon}")

        self.study_running: bool = False
        self.control_node_running: bool = False  # Activation state tracker
        self.current_button_a_state: bool = False
        self.endpoint_reached_flag: bool = False
        
        self.start_point: list[float] = []
        self.end_point: list[float] = []
        self.current_point: list[float] = []
        self.controller_initialized: bool = False

        # Pygame tracking variables (updated regardless of display state, harmless overhead)
        self.latest_control_x = 0.0
        self.latest_control_y = 0.0

        # Publishers
        self.control_output_pub = self.create_publisher(Vector3, "/control/U_a", 10)
        self.control_parameter_pub = self.create_publisher(String, "/control/K_a", 10)
        self.force_output_pub = self.create_publisher(HaplyControl, "/haply_target", 10)

        # Subscribers
        self.controller_mode_sub = self.create_subscription(String, "/study_controller_mode", self.controller_mode_callback, 10)
        self.start_point_sub = self.create_subscription(Point, "/study_start_point", self.start_point_callback, 10)
        self.end_point_sub = self.create_subscription(Point, "/study_end_point", self.end_point_callback, 10)
        self.current_point_sub = self.create_subscription(Point, "/experiment_cursor_position", self.current_point_callback, 10)
        self.estimation_kh_sub = self.create_subscription(Float64MultiArray, "/estimation/K_h", self.estimation_kh_callback, 10)
        self.haply_state_sub = self.create_subscription(HaplyState, "/haply_state", self.haply_state_callback, 10)
        self.endpoint_reached_sub = self.create_subscription(Bool, "/study_endpoint_reached", self.endpoint_reached_callback, 10)

    def initialize_controller(self):
        if self.start_point == []: 
            self.controller_initialized = False
            return False
        if self.end_point == []: 
            self.controller_initialized = False
            return False
        
        self.get_logger().debug(f"Start Controller Initialization")
        
        if self.controller_mode == "adaptive":
            if self.use_mpc_controller:
                self.controller = AdaptiveMpcController(
                    self.start_point, self.end_point, self.dt,
                    prediction_horizon=int(self.prediction_horizon),
                    max_control=(self.max_control_amplitude, self.max_control_amplitude),
                    max_velocity=(self.max_velocity_amplitude, self.max_velocity_amplitude),
                    x_bounds=(0.0, self.x_bounds_limit),
                    y_bounds=(0.0, self.y_bounds_limit),
                )
            else:
                self.controller = AdaptiveStateFeedbackController(
                    self.start_point,
                    self.end_point,
                    self.dt,
                    self,
                    max_control=(
                        self.max_control_amplitude,
                        self.max_control_amplitude,
                    ),
                    max_velocity=(
                        self.max_velocity_amplitude,
                        self.max_velocity_amplitude,
                    ),
                )
        else:
            if self.use_mpc_controller:
                self.controller = MpcController(
                    self.start_point, self.end_point, self.dt,
                    prediction_horizon=int(self.prediction_horizon),
                    max_control=(self.max_control_amplitude, self.max_control_amplitude),
                    max_velocity=(self.max_velocity_amplitude, self.max_velocity_amplitude),
                    x_bounds=(0.0, self.x_bounds_limit),
                    y_bounds=(0.0, self.y_bounds_limit),
                )
            else:
                self.controller = StateFeedbackController(
                    self.start_point,
                    self.end_point,
                    self.dt,
                    self,
                    max_control=(
                        self.max_control_amplitude,
                        self.max_control_amplitude,
                    ),
                    max_velocity=(
                        self.max_velocity_amplitude,
                        self.max_velocity_amplitude,
                    ),
                )
        self.get_logger().debug(f"Initialized controller: {type(self.controller).__name__}")
        self.controller_initialized = True
        return True

    def haply_state_callback(self, msg: HaplyState):
        self.current_button_a_state = msg.buttons.a

        if not self.endpoint_reached_flag and self.current_button_a_state:
            if not self.control_node_running:
                self.get_logger().debug("Button A pressed! Assistance control activated.")
                self.control_node_running = True
                if not self.controller_initialized:
                    self.controller_initialized = self.initialize_controller()
                
        elif self.endpoint_reached_flag and not self.current_button_a_state:
            self.endpoint_reached_flag = False

    def endpoint_reached_callback(self, msg: Bool):
        if not self.endpoint_reached_flag and msg.data:
            self.endpoint_reached_flag = True
            del self.controller
            self.controller_initialized = False
            
            if self.control_node_running:
                self.get_logger().debug("Endpoint reached message received. Assistance control stopped.")
                self.control_node_running = False

    def study_running_callback(self, msg: Bool):
        self.study_running = msg.data
        self.get_logger().debug(f"Study running state changed to {self.study_running}")

    def controller_mode_callback(self, msg: String):
        self.get_logger().debug("Message on /controller_mode")
        self.controller_mode = msg.data.lower()

        self.get_logger().debug(f"Controller mode changed to {self.controller_mode}")

        if self.initialize_controller():
            control_parameter_msg = String()
            control_parameter_msg.data = self.controller.publish_control_parameter()
            self.control_parameter_pub.publish(control_parameter_msg)
            self.get_logger().debug("Published controller parameters.")

    def start_point_callback(self, msg: Point):
        self.start_point = [msg.x, msg.y]
        self.get_logger().debug(f"Start point updated: {self.start_point}")
        if not self.controller_initialized:
            self.initialize_controller()

    def end_point_callback(self, msg: Point):
        self.end_point = [msg.x, msg.y]
        self.get_logger().debug(f"End point updated: {self.end_point}")
        if not self.controller_initialized:
            self.initialize_controller()

    def current_point_callback(self, msg: Point):
        self.get_logger().debug("Message on /experiment_cursor_position")
        control_output = [0.0, 0.0]

        if self.control_node_running:
            if self.controller_initialized:
                self.current_point = [msg.x, msg.y]
                self.get_logger().debug(f"Current point received: {self.current_point}")
                control_output = self.controller.compute_control(self.current_point)

                self.get_logger().debug(f"Control output: {control_output}")

                control_output_ros_msg = Vector3()
                control_output_ros_msg.x = control_output[0]
                control_output_ros_msg.y = control_output[1]
                control_output_ros_msg.z = 0.0

                self.control_output_pub.publish(control_output_ros_msg)
            else:
                self.get_logger().warn("Controller not initialized, publishing zero control output.")
        else:
            self.get_logger().debug("Waiting for Button A engagement, publishing zero active assistance force.")

        # Update tracking variables
        self.latest_control_x = control_output[0]
        self.latest_control_y = control_output[1]

        force_feedback_vector = Vector3()
        force_feedback_vector.x = self.acceleration_to_force_factor * control_output[0]
        force_feedback_vector.y = 0.0
        force_feedback_vector.z = self.acceleration_to_force_factor * control_output[1]

        force_feedback = HaplyControl()
        force_feedback.use_position = False
        force_feedback.target_position = Point()
        force_feedback.force = force_feedback_vector

        self.force_output_pub.publish(force_feedback)
        self.get_logger().debug(f"Published force feedback: ({force_feedback_vector.x}, {force_feedback_vector.y}, {force_feedback_vector.z})")

    def estimation_kh_callback(self, msg: Float64MultiArray):
        self.get_logger().debug(f"Received estimated K_h")

        if self.control_node_running and self.controller_initialized:
            if self.controller_mode == "adaptive":
                k_h = [[msg.data[0], msg.data[1]], [msg.data[2], msg.data[3]]]
                self.get_logger().debug(f"Received K_h estimate: {k_h}")
                self.controller.adapt(k_h)

                control_parameter_msg = String()
                control_parameter_msg.data = self.controller.publish_control_parameter()
                self.control_parameter_pub.publish(control_parameter_msg)
                self.get_logger().debug("Published updated controller parameters.")


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()

    # Check the log level parameter parsed by the node at startup
    is_debug_mode = str(node.log_level).upper() == "DEBUG"

    if is_debug_mode:
        node.get_logger().debug("DEBUG mode detected. Launching Pygame visualizer...")
        
        # Start ROS executor in a background thread
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        ros_thread = threading.Thread(target=executor.spin, daemon=True)
        ros_thread.start()

        # Initialize Pygame
        pygame.init()
        width, height = 600, 600
        screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Control Output Visualizer (DEBUG)")
        clock = pygame.time.Clock()
        font = pygame.font.SysFont(None, 24)

        center = (width // 2, height // 2)
        scale = 20.0 

        running = True
        try:
            while running:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False

                screen.fill((30, 30, 30))
                pygame.draw.line(screen, (70, 70, 70), (0, center[1]), (width, center[1]), 1)
                pygame.draw.line(screen, (70, 70, 70), (center[0], 0), (center[0], height), 1)

                current_x = node.latest_control_x
                current_y = node.latest_control_y

                end_x = (center[0] + current_x * scale, center[1])
                end_y = (center[0], center[1] - current_y * scale)
                end_sum = (center[0] + current_x * scale, center[1] - current_y * scale)

                draw_arrow(screen, (255, 50, 50), center, end_x)          # Red X
                draw_arrow(screen, (50, 255, 50), center, end_y)          # Green Y
                draw_arrow(screen, (100, 150, 255), center, end_sum)      # Blue Sum

                if current_x != 0 or current_y != 0:
                    pygame.draw.line(screen, (100, 100, 100), end_x, end_sum, 1)
                    pygame.draw.line(screen, (100, 100, 100), end_y, end_sum, 1)

                text_x = font.render(f"X Control: {current_x:.2f}", True, (255, 50, 50))
                text_y = font.render(f"Y Control: {current_y:.2f}", True, (50, 255, 50))
                magnitude = math.sqrt(current_x**2 + current_y**2)
                text_sum = font.render(f"Sum Mag: {magnitude:.2f}", True, (100, 150, 255))

                screen.blit(text_x, (10, 10))
                screen.blit(text_y, (10, 35))
                screen.blit(text_sum, (10, 60))

                pygame.display.flip()
                clock.tick(60)

        except KeyboardInterrupt:
            pass
        finally:
            pygame.quit()
            executor.shutdown()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    else:
        # Standard headless ROS execution
        node.get_logger().info("Running headless mode. Set log_level to 'DEBUG' to see the Pygame visualizer.")
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

if __name__ == "__main__":
    main()