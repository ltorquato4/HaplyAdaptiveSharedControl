from __future__ import annotations

from abc import ABC, abstractmethod

from .controller import Controller


class AdaptiveController(Controller, ABC):
    """Common base for adaptive controllers that adjust behavior at runtime."""

    @abstractmethod
    def adapt(self, K_h: list[list[float]]) -> None:
        """Update controller parameters from the estimated human gain matrix."""
        raise NotImplementedError