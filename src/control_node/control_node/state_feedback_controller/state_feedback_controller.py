import json
from collections.abc import Sequence

import numpy as np

from ..controller_interface import Controller


class StateFeedbackController(Controller):
    def __init__(
        self,
        start_point: Sequence[float],
        end_point: Sequence[float],
        dt: float,
        node=None,
        max_control: tuple[float, float] = (1.0, 1.0),
        max_velocity: tuple[float, float] = (1.0, 1.0),
    ):
        super().__init__(start_point, end_point, dt)

        kp = [0.5, 0.5]
        kd = [0.1, 0.1]

        self.K_p = np.diag(kp)
        self.K_d = np.diag(kd)
        self.max_control = np.asarray(max_control, dtype=float)
        self.max_velocity = np.asarray(max_velocity, dtype=float)

    def compute_control(self, current_point: Sequence[float]) -> list[float]:
        position = self._build_position(current_point)
        velocity = self._estimate_velocity(position)
        print(f"Current position: {position}, Current velocity: {velocity}")
        print(f"Start point: {self.experiment_start_point}, End point: {self.experiment_end_point}")

        e = position - self.experiment_end_point
        e_dot = velocity

        u_command = -self.K_p @ e - self.K_d @ e_dot

        # If the current velocity is already outside the allowed range, do not
        # add control in the same direction that would push it further out.
        if velocity[0] > self.max_velocity[0]:
            u_command[0] = min(u_command[0], 0.0)
        elif velocity[0] < -self.max_velocity[0]:
            u_command[0] = max(u_command[0], 0.0)

        if velocity[1] > self.max_velocity[1]:
            u_command[1] = min(u_command[1], 0.0)
        elif velocity[1] < -self.max_velocity[1]:
            u_command[1] = max(u_command[1], 0.0)

        self.u_a = np.clip(u_command*100, -self.max_control, self.max_control)

        return self.u_a.tolist()

    def publish_control_parameter(self):
        return json.dumps(
            {
                "Controller Type": "State Feedback Controller",
                "K_p": np.diag(self.K_p).tolist(),
                "K_d": np.diag(self.K_d).tolist(),
            }
        )
