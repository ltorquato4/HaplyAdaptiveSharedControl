from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Sequence

import numpy as np


class Controller(ABC):
    def __init__(
        self, start_point: Sequence[float], end_point: Sequence[float], dt: float
    ) -> None:
        self.experiment_start_point = np.asarray(start_point, dtype=float).reshape(2)
        self.experiment_end_point = np.asarray(end_point, dtype=float).reshape(2)
        self.dt = dt

        self.current_point: np.ndarray | None = None
        self.prev_position: np.ndarray | None = None
        self.u_a: np.ndarray = np.zeros(2, dtype=float)

    def _build_position(self, current_point: Sequence[float]) -> np.ndarray:
        position = np.asarray(current_point, dtype=float).reshape(2)
        self.current_point = position
        return position

    def _estimate_velocity(self, position: np.ndarray) -> np.ndarray:
        if self.prev_position is None:
            velocity = np.zeros(2, dtype=float)
        else:
            velocity = (position - self.prev_position) / self.dt

        self.prev_position = position
        return velocity

    @abstractmethod
    def compute_control(self, current_point: Sequence[float]) -> list[float]:
        raise NotImplementedError

    def compute_shared_control(self, u_h: Sequence[float]):
        human_control = np.asarray(u_h, dtype=float).reshape(2)
        return 0.5 * (self.u_a + human_control)

    @abstractmethod
    def publish_control_parameter(self) -> str:
        raise NotImplementedError
    
    def destroy(self):
        return None


class AdaptiveController(Controller, ABC):
    """Common base for adaptive controllers that adjust behavior at runtime."""

    @abstractmethod
    def adapt(self, K_h: Sequence[Sequence[float]]) -> None:
        """Update controller parameters from the estimated human gain matrix."""
        raise NotImplementedError
