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
        self.control_active_by_button: bool = False  # Activation state tracker
        self.start_point: list[float] = []
        self.end_point: list[float] = []
        self.current_point: list[float] = []
        self.controller_initialized: bool = False

        # ----------
        # Publishers
        # ----------
        self.control_output_pub = self.create_publisher(Vector3, "/control/U_a", 10)
        self.control_parameter_pub = self.create_publisher(String, "/control/K_a", 10)
        self.force_output_pub = self.create_publisher(HaplyControl, "/haply_target", 10)

        # -----------
        # Subscribers
        # -----------
        # self.study_running_sub = self.create_subscription(Bool, "/study_is_running", self.study_running_callback, 10)
        self.controller_mode_sub = self.create_subscription(String, "/study_controller_mode", self.controller_mode_callback, 10)
        self.start_point_sub = self.create_subscription(Point, "/study_start_point", self.start_point_callback, 10)
        self.end_point_sub = self.create_subscription(Point, "/study_end_point", self.end_point_callback, 10)
        self.current_point_sub = self.create_subscription(Point, "/experiment_cursor_position", self.current_point_callback, 10)
        self.estimation_kh_sub = self.create_subscription(Float64MultiArray, "/estimation/K_h", self.estimation_kh_callback, 10)
        
        # New subscribers for Button handling and endpoint state
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
        print(f"Start Point: {self.start_point}, End Point: {self.end_point}")
        if self.controller_mode == "adaptive":
            if self.use_mpc_controller:
                self.controller = AdaptiveMpcController(
                    self.start_point,
                    self.end_point,
                    self.dt,
                    prediction_horizon=int(self.prediction_horizon),
                    max_control=(
                        self.max_control_amplitude,
                        self.max_control_amplitude,
                    ),
                    max_velocity=(
                        self.max_velocity_amplitude,
                        self.max_velocity_amplitude,
                    ),
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
                    self.start_point,
                    self.end_point,
                    self.dt,
                    prediction_horizon=int(self.prediction_horizon),
                    max_control=(
                        self.max_control_amplitude,
                        self.max_control_amplitude,
                    ),
                    max_velocity=(
                        self.max_velocity_amplitude,
                        self.max_velocity_amplitude,
                    ),
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

    # ------------------------
    # Callbacks
    # ------------------------
    def haply_state_callback(self, msg: HaplyState):
        """Activates control computation if Button A is engaged (pressed or clicked)."""
        if msg.buttons.a:
            if not self.control_active_by_button:
                self.get_logger().info("Button A pressed! Assistance control activated.")
                self.control_active_by_button = True

    def endpoint_reached_callback(self, msg: Bool):
        """Halts control assistance immediately when the endpoint is reached."""
        if msg.data:
            if self.control_active_by_button:
                self.get_logger().info("Endpoint reached message received. Assistance control stopped.")
                self.control_active_by_button = False

    def study_running_callback(self, msg: Bool):
        self.get_logger().debug("Message on /study_is_running")
        self.study_running = msg.data
        self.get_logger().debug(f"Study running state changed to {self.study_running}")
        # Reset activation if study is manually stopped
        # if not self.study_running:
        #     self.control_active_by_button = False

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
        self.get_logger().debug("Message on /study_start_point")
        self.start_point = [msg.x, msg.y]
        self.get_logger().debug(f"Start point updated: {self.start_point}")

    def end_point_callback(self, msg: Point):
        self.get_logger().debug("Message on /study_end_point")
        self.end_point = [msg.x, msg.y]
        self.get_logger().debug(f"End point updated: {self.end_point}")

    def current_point_callback(self, msg: Point):
        self.get_logger().debug("Message on /experiment_cursor_position")
        control_output = [0.0, 0.0]

        # Condition checks both overall study loop state AND the local button toggle status
        # if self.study_running and self.control_active_by_button:
        if self.control_active_by_button:
            if self.controller_initialized or self.initialize_controller():
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
            if not self.study_running:
                self.get_logger().warn("Study is not running, publishing zero force.")
            elif not self.control_active_by_button:
                self.get_logger().debug("Waiting for Button A engagement, publishing zero active assistance force.")

        # Compute force on haply (forces default safely back to 0.0 if inactive)
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
        """TODO: Test once the estimation is available"""
        self.get_logger().debug(f"Received estimated K_h")
        # if self.study_running and self.control_active_by_button:
        if self.control_active_by_button:    
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

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down control node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()