import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Bool, Float32MultiArray, String

from haply_ros2_interface.haply_msgs.msg import HaplyContrl 

from control_node.state_feedback_controller.state_feedback_controller import StateFeedbackController
from control_node.state_feedback_controller.adaptive_state_feedback_controller import AdaptiveStateFeedbackController
from control_node.mpc_controller.mpc_controller import MpcController
from control_node.mpc_controller.adaptive_mpc_controller import AdaptiveMpcController

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        
        # Control Parameters
        self.dt = self.declare_parameter('delta_time', 0.1).value
        self.use_mpc_controller = self.declare_parameter('use_mpc_controller', True).value
        self.controller_mode = self.declare_parameter('adaptive_control_enabled', True).value
        self.max_control_amplitude = self.declare_parameter('max_control_amplitude', 10.0).value
        self.max_velocity_amplitude = self.declare_parameter('max_velocity_amplitude', 10.0).value
        self.acceleration_to_force_factor = self.declare_parameter('acceleration_to_force_factor', 1.0).value
        
        # MPC specific Parameters
        self.prediction_horizon = self.declare_parameter('prediction_horizon', 10).value

        # GUI-Experiment parameters
        self.x_bounds_limit = self.declare_parameter('x_bounds', 400.0).value
        self.y_bounds_limit = self.declare_parameter('y_bounds', 700.0).value

        self.study_running: bool = False
        self.start_point: list[float] = []
        self.end_point: list[float] = []
        self.current_point: list[float] = []
        
        if self.use_mpc_controller:
            self.controller: MpcController
        else:
            self.controller: StateFeedbackController

        # ----------
        # Publishers 
        # ----------
        self.control_output_pub = self.create_publisher(Point, '/control/u_a', 10)
        self.control_parameter_pub = self.create_publisher(String, '/control/K_a', 10)
        self.force_output_pub = self.create_publisher(HaplyContrl, '/haply_target', 10)

        # -----------
        # Subscribers 
        # -----------
        self.study_running_sub = self.create_subscription(Bool, '/study_running', self.study_running_callback,10)
        self.controller_mode_sub = self.create_subscription(Bool, '/study_controller_mode', self.controller_mode_callback, 10)
        self.start_point_sub = self.create_subscription(Point, '/study_start_point', self.start_point_callback, 10)
        self.end_point_sub = self.create_subscription(Point, '/study_end_point', self.end_point_callback, 10)
        self.current_point_sub = self.create_subscription(Point, '/study_current_point', self.current_point_callback, 10)
        self.estimation_kh_sub = self.create_subscription(Float32MultiArray, '/estimation/K_h', self.estimation_kh_callback, 10)
        self.estimation_uh_sub = self.create_subscription(Point, '/estimation/u_h', self.estimation_uh_callback, 10) #TODO: consider the importance of this

    # ------------------------
    # Callbacks
    # ------------------------
    def study_running_callback(self, msg: Bool):
        self.study_running = msg.data

    def controller_mode_callback(self, msg: Bool):
        self.controller_mode = msg.data

        if self.controller_mode:
            if self.use_mpc_controller:
                self.controller = AdaptiveMpcController(
                    self.start_point,
                    self.end_point,
                    self.dt,
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
                    max_control=(self.max_control_amplitude, self.max_control_amplitude),
                    max_velocity=(self.max_velocity_amplitude, self.max_velocity_amplitude),
                )
        else:
            if self.use_mpc_controller:
                self.controller = MpcController(
                    self.start_point,
                    self.end_point,
                    self.dt,
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
                    max_control=(self.max_control_amplitude, self.max_control_amplitude),
                    max_velocity=(self.max_velocity_amplitude, self.max_velocity_amplitude),
                )

        control_parameter_msg = String()
        control_parameter_msg.data = self.controller.publish_control_parameter()
        self.control_parameter_pub.publish(control_parameter_msg)

    def start_point_callback(self, msg: Point):
        self.start_point = [msg.x, msg.y]

    def end_point_callback(self, msg: Point):
        self.end_point = [msg.x, msg.y]

    def current_point_callback(self, msg: Point):
        if self.study_running:

            self.current_point = [msg.x, msg.y]
        
            # compute control
            control_output = self.controller.compute_control(self.current_point)   

            control_output_ros_msg = Point()
            control_output_ros_msg.x = control_output[0]
            control_output_ros_msg.y = control_output[1]
            control_output_ros_msg.z = 0.0

            self.control_output_pub.publish(control_output_ros_msg)

            # compute force on haply
            force_feedback_vector = Vector3()
            force_feedback_vector.x = self.acceleration_to_force_factor * control_output[0]
            force_feedback_vector.y = self.acceleration_to_force_factor * control_output[1]
            force_feedback_vector.z = 0
            
            force_feedback = HaplyContrl()
            force_feedback.use_position = False
            force_feedback.target_position = Point()
            force_feedback.force = force_feedback_vector

            self.force_output_pub.publish(force_feedback) 

    def estimation_uh_callback(self, msg: Point):
        # TODO: consider importance of this method
        if self.study_running:
            u_h = [msg.x, msg.y]

    def estimation_kh_callback(self, msg):
        """
        TODO: Test once the estimation is available
        """
        if self.study_running:
            if self.controller_mode:    
                k_h = [[msg.data[0], msg.data[1]],
                    [msg.data[2], msg.data[3]]]
            
                self.controller.adapt(k_h)
                control_parameter_msg = String()
                control_parameter_msg.data = self.controller.publish_control_parameter()
                self.control_parameter_pub.publish(control_parameter_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down control node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        