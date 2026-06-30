from __future__ import annotations
from typing import List

import json
import numpy as np

from ..controller import Controller

from .batch_predictor import BatchPredictor
from .constraints import Constraints
from .costfunction import CostFunction
from .linear_system_model import LinearSystemModel
from .optimization import ModelDependencies, Optimization, StateDependencies


class MpcController(Controller):
    """Encapsulates 2D point-to-point MPC setup and control solve."""

    def __init__(
        self,
        start_point: list[float],
        end_point: list[float],
        dt: float,
        prediction_horizon: int = 10,
        max_control: tuple[float, float] = (1.0, 1.0),
        max_velocity: tuple[float, float] = (1.0, 1.0),
        x_bounds: tuple[float, float] | None = None,
        y_bounds: tuple[float, float] | None = None,
        weight_comfort: float = 1.0,
        weight_trajectory: float = 1.0,
        weight_goal: float = 1.0,
    ) -> None:
        super().__init__(start_point, end_point, dt)
        self.prediction_horizon = prediction_horizon
        self.max_control = max_control
        self.max_velocity = max_velocity
        self.x_bounds = x_bounds or (
            float(min(self.experiment_start_point[0], self.experiment_end_point[0]) - 0.5),
            float(max(self.experiment_start_point[0], self.experiment_end_point[0]) + 0.5),
        )
        self.y_bounds = y_bounds or (
            float(min(self.experiment_start_point[1], self.experiment_end_point[1]) - 0.5),
            float(max(self.experiment_start_point[1], self.experiment_end_point[1]) + 0.5),
        )
        self.u_init = np.zeros(self.prediction_horizon * 2, dtype=float)

        self.linear_system_model = LinearSystemModel(dt)

        self.constraints = Constraints(
            max_control=max_control,
            max_velocity=max_velocity,
            x_bounds=self.x_bounds,
            y_bounds=self.y_bounds,
        )

        self.cost_function = CostFunction(
            weight_comfort=weight_comfort,
            weight_trajectory=weight_trajectory,
            weight_goal=weight_goal,
        )

        self.batch_predictor = BatchPredictor(self.linear_system_model, self.prediction_horizon)

        self.optimizer = Optimization(
            model_dependencies=ModelDependencies(
                system_model=self.linear_system_model,
                constraints=self.constraints,
                cost_function=self.cost_function,
                batch_predictor=self.batch_predictor,
            ),
            state_dimension=4,
            input_dimension=2,
        )

    def _build_initial_state(self, current_point: list[float]) -> np.ndarray:
        """Build [x, vx, y, vy] from the current 2D point and previous sample."""
        current_position = self._build_position(current_point)
        velocity = self._estimate_velocity(current_position)

        return np.array([current_position[0], velocity[0], current_position[1], velocity[1]], dtype=float)

    def _build_goal_state(self) -> np.ndarray:
        """Build the terminal MPC state for the configured end point."""
        return np.array([self.experiment_end_point[0], 0.0, self.experiment_end_point[1], 0.0], dtype=float)

    @staticmethod
    def _shift_warm_start(u_optimum: np.ndarray) -> np.ndarray:
        """Shift the open-loop solution forward by one control step."""
        control_sequence = u_optimum.reshape(-1, 2)
        if control_sequence.shape[0] == 0:
            return u_optimum

        shifted_sequence = np.vstack([control_sequence[1:], control_sequence[-1]])
        return shifted_sequence.reshape(-1)

    def compute_control(
        self,
        current_point: list[float],
    ) -> List[float]:
        """Solve the MPC problem and return the first control input."""

        x0 = self._build_initial_state(current_point)
        goal_state = self._build_goal_state()

        self.optimizer.set_state_dependencies(
            StateDependencies(
                x0=x0,
                u_init=self.u_init,
                goal_state=goal_state,
            )
        )

        u_optimum = self.optimizer.solve()
        self.u_init = self._shift_warm_start(u_optimum)

        u_command = np.clip(u_optimum[:2], -np.asarray(self.max_control), np.asarray(self.max_control))
        self.u_a = u_command

        return u_command.tolist()

    def publish_control_parameter(self):
        return json.dumps(self.cost_function.get_parameters)