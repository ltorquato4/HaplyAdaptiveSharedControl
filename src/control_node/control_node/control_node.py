import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Float32MultiArray

from control_node.state_feedback_controller.state_feedback_controller import StateFeedbackController
from control_node.state_feedback_controller.adaptive_state_feedback_controller import AdaptiveStateFeedbackController

"""
    from study_gui_node to control_node:
        /study_running
            boolean value: true == running, false == stopped
			event-based
	
	from scenario_generator to control_node:
        /study_controller_mode
            boolean value: true == adaptive, false == fixed
			event-based
		
		/study_start_point
            geometry_msgs/msg/Point Point: only x and y read, z ignored
			event-based
		
		/study_end_point --> to be added to mermaid
            geometry_msgs/msg/Point Point: only x and y read, z ignored
			event-based
		
		/study_current_point --> to be added to mermaid
            geometry_msgs/msg/Point Point: only x and y read, z ignored
			continous
			
		/reference_position
            what is this?
		
		/virtual_fixture
            what is this?
			
	from estimator_node to control_node
        /estimation/K_h
            Float32MultiArray
            continous

        /estimation/u_h
            geometry_msgs/msg/Point Point: only x and y read, z ignored
			continous

    from control_node to study_gui_node or scenario_generator_node and to data_logger_node:
        /control_output
            geometry_msgs/msg/Point Point: only x and y read, z ignored
			continous
"""

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        self.dt = 0.1    # TODO find appropriate value

        self.study_running: bool = False
        self.controller_mode: bool = True
        self.start_point: list[float] = []
        self.end_point: list[float] = []
        self.current_point: list[float] = []
        
        self.controller: StateFeedbackController

        # ----------
        # Publishers 
        # ----------
        self.control_output_pub = self.create_publisher(Point, '/control_output', 10)

        # -----------
        # Subscribers 
        # -----------
        self.study_running_sub = self.create_subscription(Bool, '/study_running', self.study_running_callback,10)
        self.controller_mode_sub = self.create_subscription(Bool, '/study_controller_mode', self.controller_mode_callback, 10)
        self.start_point_sub = self.create_subscription(Point, '/study_start_point', self.start_point_callback, 10)
        self.end_point_sub = self.create_subscription(Point, '/study_end_point', self.end_point_callback, 10)
        self.current_point_sub = self.create_subscription(Point, '/study_current_point', self.current_point_callback, 10)
        self.estimation_kh_sub = self.create_subscription(Float32MultiArray, '/estimation/K_h', self.estimation_kh_callback, 10)
        self.estimation_uh_sub = self.create_subscription(Point, '/estimation/u_h', self.estimation_uh_callback, 10)

        # ------------------------
        # Undefined topics (placeholders)
        # ------------------------
        # /reference_position → TODO: define if needed and type/role
        # self.reference_position_sub = self.create_subscription(<MessageType>, '/reference_position', self.reference_position_callback, 10)

        # /virtual_fixture → TODO: define if needed and type/role
        # self.virtual_fixture_sub = self.create_subscription(<MessageType>, '/virtual_fixture', self.virtual_fixture_callback, 10)


    # ------------------------
    # Callbacks
    # ------------------------
    def study_running_callback(self, msg: Bool):
        self.study_running = msg.data

    def controller_mode_callback(self, msg: Bool):
        self.controller_mode = msg.data

        if self.controller_mode:
            self.controller = AdaptiveStateFeedbackController(self.start_point, self.end_point, self.dt, self)
        else:
            self.controller = StateFeedbackController(self.start_point, self.end_point, self.dt, self)

    def start_point_callback(self, msg: Point):
        self.start_point = [msg.x, msg.y]

    def end_point_callback(self, msg: Point):
        self.end_point = [msg.x, msg.y]

    def current_point_callback(self, msg: Point):
        self.current_point = [msg.x, msg.y]
        
        if self.study_running:
            self.controller.compute_control(self.current_point)    

    def estimation_uh_callback(self, msg: Point):
        """
        TODO: Think about combining uh callback function and kh callback function
        """
        u_h = [msg.x, msg.y]

        control_output = self.controller.compute_shared_control(u_h)

        control_output_ros_msg = Point()
        control_output_ros_msg.x = control_output[0]
        control_output_ros_msg.y = control_output[1]
        control_output_ros_msg.z = 0.0

        self.control_output_pub.publish(control_output_ros_msg)

    def estimation_kh_callback(self, msg):
        k_h = [[msg.data[0], msg.data[1]],
               [msg.data[2], msg.data[3]]]
        
        if self.controller_mode:
            self.controller.adapt_gain(k_h)
    
    # Placeholder callbacks:
    # def reference_position_callback(self, msg):
    #     pass

    # def virtual_fixture_callback(self, msg):
    #     pass

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