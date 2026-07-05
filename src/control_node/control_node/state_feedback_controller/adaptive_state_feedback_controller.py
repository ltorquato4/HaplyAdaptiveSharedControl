from collections.abc import Sequence

import numpy as np

from control_node.controller_interface import AdaptiveController
from control_node.state_feedback_controller.state_feedback_controller import (
    StateFeedbackController,
)

FACTOR_DEFAULT = 0.5
FACTOR_RANGE: list[float] = []
BORDER_ONE_DEFAULT = 0.3
BORDER_ONE_RANGE = [0.2, 0.4]
BORDER_TWO_DEFAULT = 0.7
BORDER_TWO_RANGE = [0.6, 0.8]


class AdaptiveStateFeedbackController(AdaptiveController, StateFeedbackController):
    def __init__(self, start_point, end_point, dt, node=None):
        super().__init__(start_point, end_point, dt, node)

    def adapt(self, K_h: Sequence[Sequence[float]]) -> None:
        """Adapt proportional and derivative gains along the trajectory."""

        # --- Distance calculations ---
        distance_start_to_end = np.linalg.norm(
            self.experiment_end_point - self.experiment_start_point
        )

        distance_start_to_current = np.linalg.norm(
            np.array(self.current_point) - self.experiment_start_point
        )

        # Normalized progress along trajectory in range [0, 1]
        self.progress_along_path = (
            distance_start_to_current / distance_start_to_end
            if distance_start_to_end > 1e-6
            else 0.0
        )
        progress_along_path = np.clip(self.progress_along_path, 0.0, 1.0)

        # --- Human gain magnitude ---
        K_h_array = np.asarray(K_h, dtype=float)
        K_h_magnitude = np.linalg.norm(K_h_array)

        # Smooth normalization into [0, 1]
        normalized_human_gain = np.tanh(
            K_h_magnitude
        )  # TODO this needs to be done based on the actual values of K_h

        # --- Adaptive borders ---
        border_one = BORDER_ONE_RANGE[0] + normalized_human_gain * (
            BORDER_ONE_RANGE[1] - BORDER_ONE_RANGE[0]
        )

        border_two = BORDER_TWO_RANGE[1] - normalized_human_gain * (
            BORDER_TWO_RANGE[1] - BORDER_TWO_RANGE[0]
        )

        # --- Adaptive scaling factor ---
        scaling_factor = FACTOR_DEFAULT
        if FACTOR_RANGE:
            scaling_factor = FACTOR_RANGE[0] + normalized_human_gain * (
                FACTOR_RANGE[1] - FACTOR_RANGE[0]
            )

        # --- Compute adaptive gain scaling ---
        if progress_along_path < border_one:
            # Near start: high gain decreasing toward middle
            gain_scale = 1.0 - scaling_factor * (progress_along_path / border_one)

        elif progress_along_path > border_two:
            # Near end: high gain increasing from middle
            gain_scale = 1.0 - scaling_factor * (
                (1.0 - progress_along_path) / (1.0 - border_two)
            )

        else:
            # Middle region: low gain
            gain_scale = 1.0 - scaling_factor

        # Clamp gain scale to safe range
        gain_scale = np.clip(gain_scale, 0.0, 1.0)

        # --- Apply scaling safely ---
        if not hasattr(self, "proportional_gain_base"):
            self.proportional_gain_base = self.K_p.copy()
            self.derivative_gain_base = self.K_d.copy()

        self.K_p = gain_scale * self.proportional_gain_base
        self.K_d = gain_scale * self.derivative_gain_base
