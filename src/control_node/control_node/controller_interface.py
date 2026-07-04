from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np

class Controller(ABC):
    def __init__(self, start_point: list[float], end_point: list[float], dt: float) -> None:
        self.experiment_start_point = np.asarray(start_point, dtype=float).reshape(2)
        self.experiment_end_point = np.asarray(end_point, dtype=float).reshape(2)
        self.dt = dt

        self.current_point: np.ndarray | None = None
        self.prev_position: np.ndarray | None = None
        self.u_a = np.zeros(2, dtype=float)

    def _build_position(self, current_point: list[float]) -> np.ndarray:
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
    def compute_control(self, current_point: list[float]) -> list[float]:
        raise NotImplementedError

    def compute_shared_control(self, u_h: list[float]):
        u_h = np.asarray(u_h, dtype=float).reshape(2)
        return 0.5 * (np.asarray(self.u_a, dtype=float) + u_h)
    
    @abstractmethod
    def publish_control_parameter(self) -> str:
        raise NotImplementedError
    
    
class AdaptiveController(Controller, ABC):
    """Common base for adaptive controllers that adjust behavior at runtime."""

    @abstractmethod
    def adapt(self, K_h: list[list[float]]) -> None:
        """Update controller parameters from the estimated human gain matrix."""
        raise NotImplementedError