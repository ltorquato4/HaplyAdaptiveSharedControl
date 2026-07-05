from __future__ import annotations

from collections.abc import Sequence

import numpy as np

from control_node.controller_interface import AdaptiveController

from .mpc_controller import MpcController

FACTOR_DEFAULT = 0.5
FACTOR_RANGE: list[float] = []

BORDER_ONE_DEFAULT = 0.3
BORDER_ONE_RANGE = [0.2, 0.4]

BORDER_TWO_DEFAULT = 0.7
BORDER_TWO_RANGE = [0.6, 0.8]


class AdaptiveMpcController(AdaptiveController, MpcController):
    def __init__(
        self,
        start_point: Sequence[float],
        end_point: Sequence[float],
        dt: float,
        prediction_horizon=10,
        max_control=(1.0, 1.0),
        max_velocity=(1.0, 1.0),
        x_bounds=None,
        y_bounds=None,
        weight_comfort=1.0,
        weight_trajectory=1.0,
        weight_goal=1.0,
    ):
        super().__init__(
            start_point,
            end_point,
            dt,
            prediction_horizon,
            max_control,
            max_velocity,
            x_bounds,
            y_bounds,
            weight_comfort,
            weight_trajectory,
            weight_goal,
        )

        self.weight_comfort_base = weight_comfort
        self.weight_trajectory_base = weight_trajectory
        self.weight_goal_base = weight_goal

        self.progress_along_path = 0.0

    def adapt(self, K_h: Sequence[Sequence[float]]) -> None:
        """
        Adapt MPC objective weights based on:
        1. Progress along the path.
        2. Magnitude of human gain K_h.

        Increasing K_h:
            - decreases comfort weight
            - increases trajectory weight
            - increases goal weight

        Decreasing K_h:
            - increases comfort weight
            - decreases trajectory weight
            - decreases goal weight
        """

        distance_start_to_end = np.linalg.norm(
            self.experiment_end_point - self.experiment_start_point
        )

        if self.current_point is None:
            current_point = self.experiment_start_point
        else:
            current_point = self.current_point

        distance_start_to_current = np.linalg.norm(
            np.asarray(current_point, dtype=float) - self.experiment_start_point
        )

        self.progress_along_path = (
            distance_start_to_current / distance_start_to_end
            if distance_start_to_end > 1e-6
            else 0.0
        )

        progress_along_path = np.clip(
            self.progress_along_path,
            0.0,
            1.0,
        )

        # Human gain magnitude
        K_h_array = np.asarray(K_h, dtype=float)
        K_h_magnitude = np.linalg.norm(K_h_array)

        # Normalize to approximately [0, 1]
        normalized_human_gain = np.tanh(K_h_magnitude)

        # Adapt borders based on human gain
        border_one = BORDER_ONE_RANGE[0] + normalized_human_gain * (
            BORDER_ONE_RANGE[1] - BORDER_ONE_RANGE[0]
        )

        border_two = BORDER_TWO_RANGE[1] - normalized_human_gain * (
            BORDER_TWO_RANGE[1] - BORDER_TWO_RANGE[0]
        )

        scaling_factor = FACTOR_DEFAULT

        if FACTOR_RANGE:
            scaling_factor = FACTOR_RANGE[0] + normalized_human_gain * (
                FACTOR_RANGE[1] - FACTOR_RANGE[0]
            )

        # Progress-based adaptation
        if progress_along_path < border_one:
            gain_scale = 1.0 - scaling_factor * (progress_along_path / border_one)

        elif progress_along_path > border_two:
            gain_scale = 1.0 - scaling_factor * (
                (1.0 - progress_along_path) / (1.0 - border_two)
            )

        else:
            gain_scale = 1.0 - scaling_factor

        gain_scale = np.clip(gain_scale, 0.0, 1.0)

        # ------------------------------------------------------------------
        # Direct human-gain adaptation
        # Higher K_h:
        #   less comfort
        #   more trajectory tracking
        #   more goal seeking
        # ------------------------------------------------------------------

        comfort_gain_factor = 1.5 - normalized_human_gain
        trajectory_gain_factor = 0.5 + normalized_human_gain
        goal_gain_factor = 0.5 + 1.5 * normalized_human_gain

        comfort_weight = (
            self.weight_comfort_base * (1.0 - 0.75 * gain_scale) * comfort_gain_factor
        )

        trajectory_weight = (
            self.weight_trajectory_base
            * (0.5 + 0.5 * gain_scale)
            * trajectory_gain_factor
        )

        goal_weight = (
            self.weight_goal_base * (0.5 + 1.5 * gain_scale) * goal_gain_factor
        )

        self.cost_function.set_weights(
            weight_comfort=comfort_weight,
            weight_trajectory=trajectory_weight,
            weight_goal=goal_weight,
        )
