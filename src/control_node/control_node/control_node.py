import rclpy
from geometry_msgs.msg import Point, Vector3
from haply_msgs.msg import HaplyControl
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray, String

from control_node.controller_interface import Controller
from control_node.mpc_controller.adaptive_mpc_controller import AdaptiveMpcController
from control_node.mpc_controller.mpc_controller import MpcController
from control_node.state_feedback_controller.adaptive_state_feedback_controller import AdaptiveStateFeedbackController
from control_node.state_feedback_controller.state_feedback_controller import StateFeedbackController


class ControlNode(Node):
    def __init__(self):
        super().__init__("control_node")

        self.define_logger()

        self.define_static_parameters()

        self.reset_scenario_state()
        self.reset_runtime_state()

        self.define_publishers()
        self.define_subscribers()

        self.get_logger().info("Control node started.")
        self.get_logger().debug(
            f"Config: dt={self.dt}, use_mpc={self.use_mpc_controller}, "
            f"default_mode={self.controller_mode}, horizon={self.prediction_horizon}"
        )

    def define_logger(self):
        self.log_level = self.declare_parameter("log_level", "INFO").value
        log_levels = {
            "DEBUG": LoggingSeverity.DEBUG,
            "INFO": LoggingSeverity.INFO,
            "WARN": LoggingSeverity.WARN,
            "WARNING": LoggingSeverity.WARN,
            "ERROR": LoggingSeverity.ERROR,
            "FATAL": LoggingSeverity.FATAL,
        }
        severity = log_levels.get(str(self.log_level).upper(), LoggingSeverity.INFO)
        self.get_logger().set_level(severity)

    def get_or_declare_parameter(self, name, default_value):
        if self.has_parameter(name):
            return self.get_parameter(name).value
        return self.declare_parameter(name, default_value).value

    def define_static_parameters(self):
        self.dt = self.get_or_declare_parameter("delta_time", 0.1)
        self.use_mpc_controller = self.get_or_declare_parameter("use_mpc_controller", True)
        
        adaptive_enabled = self.get_or_declare_parameter("adaptive_control_enabled", False)
        self.default_controller_mode = "adaptive" if adaptive_enabled else "fixed"

        self.max_control_amplitude = self.get_or_declare_parameter("max_control_amplitude", 10.0)
        self.max_velocity_amplitude = self.get_or_declare_parameter("max_velocity_amplitude", 10.0)
        self.acceleration_to_force_factor = self.get_or_declare_parameter("acceleration_to_force_factor", 0.2)
        
        self.mpc_control_every_i_th_iteration = self.get_or_declare_parameter("mpc_control_every_i_th_iteration", 1)
        self.adapt_every_i_th_iterarion = self.get_or_declare_parameter("adapt_every_i_th_iterarion", 3)
        
        self.prediction_horizon = self.get_or_declare_parameter("prediction_horizon", 5)
        self.x_bounds_limit = self.get_or_declare_parameter("x_bounds", 0.10)
        self.y_bounds_limit = self.get_or_declare_parameter("y_bounds", 0.10)

    def reset_scenario_state(self):
        self.start_point: list[float] = []
        self.end_point: list[float] = []
        self.controller_mode: str = self.default_controller_mode

    def reset_runtime_state(self):
        self.controller: Controller = None
        self.controller_initialized = False
        self.study_is_running: bool = False
        
        self.control_iteration = -1
        self.adapt_iteration = -1
        self.current_point: list[float] = []

    def define_publishers(self):
        self.control_output_pub = self.create_publisher(Vector3, "/control/U_a", 10)
        self.control_parameter_pub = self.create_publisher(String, "/control/K_a", 10)
        self.force_output_pub = self.create_publisher(HaplyControl, "/haply_target", 10)
        
    def define_subscribers(self):
        self.controller_mode_sub = self.create_subscription(String, "/study_controller_mode", self.controller_mode_callback, 10)
        self.start_point_sub = self.create_subscription(Point, "/study_start_point", self.start_point_callback, 10)
        self.end_point_sub = self.create_subscription(Point, "/study_end_point", self.end_point_callback, 10)
        self.study_is_running_sub = self.create_subscription(Bool, "/study_is_running", self.study_is_running_callback, 10)
        self.current_point_sub = self.create_subscription(Point, "/experiment_cursor_position", self.current_point_callback, 10)
        self.estimation_kh_sub = self.create_subscription(Float64MultiArray, "/estimation/K_h", self.estimation_kh_callback, 10)

    def publish_zero_force(self):
        """Forces the haptic device to clear commands and neutralise safely."""
        force_feedback = HaplyControl()
        force_feedback.use_position = False
        force_feedback.target_position = Point()
        force_feedback.force = Vector3(x=0.0, y=0.0, z=0.0)
        self.force_output_pub.publish(force_feedback)

    def calculate_force(self, control_output):
        force_feedback_vector = Vector3()
        force_feedback_vector.x = self.acceleration_to_force_factor * control_output[0]
        force_feedback_vector.y = 0.0
        force_feedback_vector.z = self.acceleration_to_force_factor * control_output[1]

        force_feedback = HaplyControl()
        force_feedback.use_position = False
        force_feedback.target_position = Point()
        force_feedback.force = force_feedback_vector

        self.force_output_pub.publish(force_feedback)

    def initialize_controller(self) -> bool:
        if not self.start_point or not self.end_point or not self.controller_mode: 
            return False
        
        self.get_logger().debug("Start Controller Initialization")
        
        if self.controller:
            self.controller.destroy()
            del self.controller
            self.controller = None
        
        if self.controller_mode == "adaptive":
            if self.use_mpc_controller:
                self.controller = AdaptiveMpcController(
                    self.start_point, self.end_point, self.dt,
                    prediction_horizon=int(self.prediction_horizon),
                    max_control=(self.max_control_amplitude, self.max_control_amplitude),
                    max_velocity=(self.max_velocity_amplitude, self.max_velocity_amplitude),
                    x_bounds=(-self.x_bounds_limit, self.x_bounds_limit),
                    y_bounds=(-self.y_bounds_limit, self.y_bounds_limit),
                )
            else:
                self.controller = AdaptiveStateFeedbackController(
                    self.start_point, self.end_point, self.dt, self,
                    max_control=(self.max_control_amplitude, self.max_control_amplitude),
                    max_velocity=(self.max_velocity_amplitude, self.max_velocity_amplitude),
                )
        else:
            if self.use_mpc_controller:
                self.controller = MpcController(
                    self.start_point, self.end_point, self.dt,
                    prediction_horizon=int(self.prediction_horizon),
                    max_control=(self.max_control_amplitude, self.max_control_amplitude),
                    max_velocity=(self.max_velocity_amplitude, self.max_velocity_amplitude),
                    x_bounds=(-self.x_bounds_limit, self.x_bounds_limit),
                    y_bounds=(-self.y_bounds_limit, self.y_bounds_limit),
                )
            else:
                self.controller = StateFeedbackController(
                    self.start_point, self.end_point, self.dt, self,
                    max_control=(self.max_control_amplitude, self.max_control_amplitude),
                    max_velocity=(self.max_velocity_amplitude, self.max_velocity_amplitude),
                )

        self.get_logger().debug(f"Initialized controller: {type(self.controller).__name__}")
        self.controller_initialized = True
        return True

    def controller_mode_callback(self, msg: String):
        new_mode = msg.data.lower()
        if self.controller_mode == new_mode:
            return
        self.controller_mode = new_mode
        self.get_logger().debug(f"Updated controller mode: {self.controller_mode}")
        
        if self.initialize_controller():
            control_parameter_msg = String()
            control_parameter_msg.data = self.controller.publish_control_parameter()
            self.control_parameter_pub.publish(control_parameter_msg)

    def start_point_callback(self, msg: Point):
        new_start = [msg.x, msg.y]
        if self.start_point == new_start:
            return
        self.start_point = new_start
        self.get_logger().debug(f"Updated start point: {self.start_point}")
        self.initialize_controller()

    def end_point_callback(self, msg: Point):
        new_end = [msg.x, msg.y]
        if self.end_point == new_end:
            return
        self.end_point = new_end
        self.get_logger().debug(f"Updated end point: {self.end_point}")
        self.initialize_controller()

    def current_point_callback(self, msg: Point):
        control_output = [0.0, 0.0]

        if self.study_is_running and self.controller_initialized:
            if self.use_mpc_controller:
                self.control_iteration += 1
                if self.control_iteration % self.mpc_control_every_i_th_iteration != 0:
                    return
            
            self.current_point = [msg.x, msg.y]
            control_output = self.controller.compute_control(self.current_point)
            
            if self.control_iteration % 17 == 0:  
                self.get_logger().debug(f"Control Output: {control_output}")
                
            control_output_ros_msg = Vector3()
            control_output_ros_msg.x = control_output[0]
            control_output_ros_msg.y = control_output[1]
            control_output_ros_msg.z = 0.0
            self.control_output_pub.publish(control_output_ros_msg)

        self.calculate_force(control_output)
        
    def estimation_kh_callback(self, msg: Float64MultiArray):
        if not (self.study_is_running and self.controller_initialized):
            return

        self.adapt_iteration += 1
        if self.adapt_iteration % self.adapt_every_i_th_iterarion != 0:
            return

        if self.adapt_iteration % 27 == 0:
            self.get_logger().debug(f"Adaptation K_h: {msg.data}")
            
        if self.controller_mode == "adaptive":
            kp_x = msg.data[0]
            kd_x = msg.data[1]
            kp_y = msg.data[6]
            kd_y = msg.data[7]
            
            k_h = [[kp_x, kd_x], [kp_y, kd_y]]
            
            self.controller.adapt(k_h)

            control_parameter_msg = String()
            control_parameter_msg.data = self.controller.publish_control_parameter()
            self.control_parameter_pub.publish(control_parameter_msg)

    def study_is_running_callback(self, msg: Bool):
        new_study_is_running = msg.data
        if self.study_is_running == new_study_is_running:
            return
        
        self.study_is_running = new_study_is_running
        self.get_logger().debug(f"Study execution flag modified: {self.study_is_running}")
        
        if not self.study_is_running:
            self.publish_zero_force()
            if self.controller:
                self.controller.destroy()
                del self.controller
            
            self.reset_runtime_state()


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()

if __name__ == "__main__":
    main()