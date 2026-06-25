import numpy as np

class Controller:
    def __init__(self, start_point: list[float], end_point: list[float], dt: float, node=None):
        self.experiment_start_point = np.array(start_point)
        self.experiment_end_point = np.array(end_point)

        self.dt = dt 

        if node is not None:
            kp = node.declare_parameter('K_p', [0.5, 0.5]).value
            kd = node.declare_parameter('K_d', [0.1, 0.1]).value
        else:
            kp = [0.5, 0.5]
            kd = [0.1, 0.1]

        self.K_p = np.diag(kp)
        self.K_d = np.diag(kd)
        self.prev_position = None
        self.u_a = np.zeros(2)

    def compute_control(self, current_point: list[float]) -> list[float]:
        position = np.array(current_point)

        if self.prev_position is None:
            velocity = np.zeros(2)
        else:
            velocity = (position - self.prev_position) / self.dt

        self.prev_position = position

        e = position - self.experiment_end_point
        e_dot = velocity

        self.u_a = - self.K_p @ e - self.K_d @ e_dot

        return self.u_a

    def compute_shared_control(self, u_h: list[float]):
        u_h = np.array(u_h)
        return 0.5 * (self.u_a + u_h)