from __future__ import annotations

import json
from collections.abc import Sequence

import numpy as np

from ..controller_interface import Controller
from .batch_predictor import BatchPredictor
from .constraints import Constraints
from .costfunction import CostFunction
from .linear_system_model import LinearSystemModel
from .optimization import ModelDependencies, Optimization, StateDependencies


class MpcController(Controller):
    """Encapsulates 2D point-to-point MPC setup and control solve."""

    def __init__(
        self,
        start_point: Sequence[float],
        end_point: Sequence[float],
        dt: float,
        prediction_horizon: int = 5,
        max_control: tuple[float, float] = (1.0, 1.0),
        max_velocity: tuple[float, float] = (1.0, 1.0),
        x_bounds: tuple[float, float] | None = None,
        y_bounds: tuple[float, float] | None = None,
        weight_comfort: float = 50e1,
        weight_trajectory: float = 10e2,
        weight_goal: float = 30e2,
    ) -> None:
        super().__init__(start_point, end_point, dt)
        self.prediction_horizon = prediction_horizon
        self.max_control = max_control
        self.max_velocity = max_velocity
        self.x_bounds = x_bounds or (
            float(
                min(self.experiment_start_point[0], self.experiment_end_point[0]) - 0.5
            ),
            float(
                max(self.experiment_start_point[0], self.experiment_end_point[0]) + 0.5
            ),
        )
        self.y_bounds = y_bounds or (
            float(
                min(self.experiment_start_point[1], self.experiment_end_point[1]) - 0.5
            ),
            float(
                max(self.experiment_start_point[1], self.experiment_end_point[1]) + 0.5
            ),
        )
        self.u_init = np.zeros(self.prediction_horizon * 2, dtype=float)
        self.prev_cursor_timestamp_s: float | None = None

        self.weight_comfort = weight_comfort
        self.weight_trajectory = weight_trajectory
        self.weight_goal = weight_goal

        self.linear_system_model = LinearSystemModel(dt)

        self.constraints = Constraints(
            max_control=max_control,
            max_velocity=max_velocity,
            x_bounds=self.x_bounds,
            y_bounds=self.y_bounds,
        )

        self.cost_function = CostFunction(
            weight_comfort=self.weight_comfort,
            weight_trajectory=self.weight_trajectory,
            weight_goal=self.weight_goal,
        )

        self.batch_predictor = BatchPredictor(
            self.linear_system_model, self.prediction_horizon
        )

        self.optimizer = Optimization(
            model_dependencies=ModelDependencies(
                system_model=self.linear_system_model,
                constraints=self.constraints,
                cost_function=self.cost_function,
                batch_predictor=self.batch_predictor,
            ),
            state_dimension=4,
            input_dimension=2,
            start_point=self.experiment_start_point,
            end_point=self.experiment_end_point,
        )

    def _build_initial_state(
        self,
        current_point: Sequence[float],
        timestamp_s: float | None = None,
    ) -> np.ndarray:
        current_position = self._build_position(current_point)
        if timestamp_s is None:
            velocity = self._estimate_velocity(current_position)
        else:
            timestamp_s = float(timestamp_s)
            if not np.isfinite(timestamp_s):
                raise ValueError("cursor timestamp must be finite")
            if (
                self.prev_cursor_timestamp_s is not None
                and timestamp_s <= self.prev_cursor_timestamp_s
            ):
                raise ValueError("cursor timestamp must increase")
            if self.prev_position is None:
                velocity = np.zeros(2, dtype=float)
            else:
                sample_dt = timestamp_s - self.prev_cursor_timestamp_s
                velocity = (current_position - self.prev_position) / sample_dt
            self.prev_position = current_position.copy()
            self.prev_cursor_timestamp_s = timestamp_s

        return np.array(
            [current_position[0], velocity[0], current_position[1], velocity[1]],
            dtype=float,
        )

    def _build_goal_state(self) -> np.ndarray:
        return np.array(
            [self.experiment_end_point[0], 0.0, self.experiment_end_point[1], 0.0],
            dtype=float,
        )

    @staticmethod
    def _shift_warm_start(u_optimum: np.ndarray) -> np.ndarray:
        control_sequence = u_optimum.reshape(-1, 2)
        if control_sequence.shape[0] == 0:
            return u_optimum

        shifted_sequence = np.vstack([control_sequence[1:], control_sequence[-1]])
        return shifted_sequence.reshape(-1)

    def compute_control(
        self,
        current_point: Sequence[float],
        timestamp_s: float | None = None,
    ) -> list[float]:
        x0 = self._build_initial_state(current_point, timestamp_s)
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

        u_command = np.clip(
            u_optimum[:2], -np.asarray(self.max_control), np.asarray(self.max_control)
        )
        self.u_a = u_command

        return u_command.tolist()

    def publish_control_parameter(self):
        return json.dumps(self.cost_function.get_parameters())

    def reset_runtime_state(self) -> None:
        """Reset sample history and the optimizer warm start for a retry."""
        self.current_point = None
        self.prev_position = None
        self.prev_cursor_timestamp_s = None
        self.u_a = np.zeros(2, dtype=float)
        self.u_init = np.zeros(self.prediction_horizon * 2, dtype=float)

    def destroy(self) -> None:
        """
        Explicitly tear down the MPC controller and free its memory.

        This cleans up internal NumPy state buffers, cascades down to the CasADi
        Optimization orchestrator, and strips sub-component references to force
        immediate garbage collection.
        """
        # 1. Force cleanup of the symbolic optimizer structures
        if hasattr(self, "optimizer") and self.optimizer is not None:
            try:
                self.optimizer.destroy()
            except Exception:
                pass
            del self.optimizer
            self.optimizer = None

        # 2. Iteratively clear child components to isolate memory footprints
        child_components = [
            "linear_system_model",
            "constraints",
            "cost_function",
            "batch_predictor",
        ]
        for component_attr in child_components:
            component = getattr(self, component_attr, None)
            if component is not None:
                # If these classes get upgraded later to have internal destroy loops
                if hasattr(component, "destroy"):
                    try:
                        component.destroy()
                    except Exception:
                        pass
                setattr(self, component_attr, None)

        # 3. Wipe out structural runtime arrays and cached sequences
        self.u_init = None
        if hasattr(self, "u_a"):
            self.u_a = None

        import gc
        gc.collect()
