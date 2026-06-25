import rclpy
from rclpy.node import Node

from std_msgs.msg import Point, Bool


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
            format to be determined
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
        # self.estimation_kh_sub = self.create_subscription(<MessageType>, '/estimation/K_h', self.estimation_kh_callback, 10) # /estimation/K_h (continuous) → TODO: format TBD
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
        pass

    def controller_mode_callback(self, msg: Bool):
        pass

    def start_point_callback(self, msg: Point):
        pass

    def end_point_callback(self, msg: Point):
        pass

    def current_point_callback(self, msg: Point):
        pass

    def estimation_uh_callback(self, msg: Point):
        pass

    # Placeholder callbacks:
    # def estimation_kh_callback(self, msg):
    #     pass

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