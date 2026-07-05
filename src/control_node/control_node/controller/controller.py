from collections.abc import Sequence

import numpy as np


class Controller:
    def __init__(
        self,
        start_point: Sequence[float],
        end_point: Sequence[float],
        dt: float,
        node=None,
    ):
        self.experiment_start_point = np.asarray(start_point, dtype=float)
        self.experiment_end_point = np.asarray(end_point, dtype=float)

        self.dt = dt

        if node is not None:
            kp = node.declare_parameter("K_p", [0.5, 0.5]).value
            kd = node.declare_parameter("K_d", [0.1, 0.1]).value
        else:
            kp = [0.5, 0.5]
            kd = [0.1, 0.1]

        self.K_p = np.diag(kp)
        self.K_d = np.diag(kd)
        self.prev_position: np.ndarray | None = None
        self.u_a: np.ndarray = np.zeros(2, dtype=float)

    def compute_control(self, current_point: Sequence[float]):
        position = np.asarray(current_point, dtype=float)

        if self.prev_position is None:
            velocity = np.zeros(2)
        else:
            velocity = (position - self.prev_position) / self.dt

        self.prev_position = position

        e = position - self.experiment_end_point
        e_dot = velocity

        self.u_a = -self.K_p @ e - self.K_d @ e_dot

        return self.u_a

    def share_control(self, u_h: Sequence[float]):
        human_control = np.asarray(u_h, dtype=float)
        return 0.5 * (self.u_a + human_control)
