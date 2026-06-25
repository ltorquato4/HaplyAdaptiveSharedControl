from control_node.control_node.state_feedback_controller.state_feedback_controller import Controller

class AdaptiveStateFeedbackController(Controller):
    def __init__(self, start_point, end_point, dt, node=None):
        super().__init__(start_point, end_point, dt, node)
    
    def adapt_gain(self, K_h: list[list[float]]) -> None:
        # TODO
        pass
