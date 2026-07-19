from __future__ import annotations

from collections.abc import Sequence
import numpy as np

from control_node.controller_interface import AdaptiveController
from .mpc_controller import MpcController

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
        self.in_docking_zone_already_once = False

    def adapt(self, K_h: Sequence[Sequence[float]] | Sequence[float]) -> None:
        """
        Adapt MPC objective weights based on:
        1. Progress along the path (with a Terminal Docking Zone).
        2. Balance of human gain K_h (Cooperative authority model using 
           aggressive vs. careful stiffness components).
        """
        # 1. Calculate path progress metrics
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

        progress_along_path = np.clip(self.progress_along_path, 0.0, 1.0)

        # 2. Extract active components from K_h safely
        K_h_flat = np.asarray(K_h, dtype=float).flatten()
        K_0 = K_h_flat[0] if K_h_flat.size > 0 else 0.0
        K_1 = K_h_flat[1] if K_h_flat.size > 1 else 0.0
        K_6 = K_h_flat[6] if K_h_flat.size > 6 else 0.0
        K_7 = K_h_flat[7] if K_h_flat.size > 7 else 0.0

        # Mathematically corrected averaging
        comfort_factor = (abs(K_0) + abs(K_6)) / 2.0
        trajectory_factor = (abs(K_1) + abs(K_7)) / 2.0

        # 3. Compute Normalized Dominance Difference to keep things bounded
        # We divide by a soft-normalization factor (e.g., 50.0 N/m) to scale the influence,
        # then clip it to prevent extreme weights.
        normalization_scale = 50.0
        stiffness_diff = (comfort_factor - trajectory_factor) / normalization_scale
        stiffness_diff = np.clip(stiffness_diff, -1.0, 1.0) # Keeps delta factors between [-1, 1]

        # 4. Calculate adaptive weights
        # When comfort_factor > trajectory_factor:
        #   - Comfort weight increases (robot complies with user's direct intent)
        #   - Trajectory weight decreases (robot loosens its track-keeping)
        # makes sure waits never become negative
        comfort_weight = max(0, self.weight_comfort_base * (1.0 + 1.5 * stiffness_diff))
        trajectory_weight = max(0, self.weight_trajectory_base * (1.0 - 1.5 * stiffness_diff))
        
        # Goal weight remains constant and is only adapted in close proximity to the docking zone 
        goal_weight = self.weight_goal_base 

        # 5. Ensure All Weights Stay Strictly Positive (Safety Guard)
        comfort_weight = max(comfort_weight, self.weight_comfort_base * 0.1)
        trajectory_weight = max(trajectory_weight, self.weight_trajectory_base * 0.1)
        goal_weight = max(goal_weight, self.weight_goal_base * 0.1)

        # 6. Terminal Docking Zone Override (Last 20% of path)
        TERMINAL_THRESHOLD = 0.85 
        if progress_along_path > TERMINAL_THRESHOLD or self.in_docking_zone_already_once:
            self.in_docking_zone_already_once = True
            terminal_fraction = (progress_along_path - TERMINAL_THRESHOLD) / (1.0 - TERMINAL_THRESHOLD)
            t_scale = terminal_fraction ** 2

            # Smoothly shift authority to the computer to lock onto the target
            comfort_weight   *= (1.0 - 0.9 * t_scale)
            trajectory_weight *= (1.0 + 2.0 * t_scale)
            goal_weight *= (1.0 + 10e5 * t_scale)

        # Push calculated changes down to active solver parameters
        self.cost_function.set_weights(
            weight_comfort=comfort_weight,
            weight_trajectory=trajectory_weight,
            weight_goal=goal_weight,
        )