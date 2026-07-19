from collections.abc import Sequence
import numpy as np

from control_node.controller_interface import AdaptiveController
from control_node.state_feedback_controller.state_feedback_controller import StateFeedbackController

class AdaptiveStateFeedbackController(AdaptiveController, StateFeedbackController):
    def __init__(self, start_point, end_point, dt, node=None, 
                 max_control: tuple[float, float] = (1.0, 1.0),
                 max_velocity: tuple[float, float] = (1.0, 1.0)):
        super().__init__(start_point, end_point, dt, node, max_control, max_velocity)
         
        self.in_docking_zone_already_once = False

    def adapt(self, K_h: Sequence[Sequence[float]] | Sequence[float]) -> None:
        """
        Adapt proportional and derivative gain matrices along the trajectory based on:
        1. Progress along the path (with an explicit Terminal Docking Zone).
        2. Balance of human gain K_h (Cooperative intent vs. stabilization model).
        """
        # --- 1. Path Progress Calculations ---
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

        # --- 2. Extract Authority Factors from K_h Safely ---
        K_h_flat = np.asarray(K_h, dtype=float).flatten()
        K_0 = K_h_flat[0] if K_h_flat.size > 0 else 0.0
        K_1 = K_h_flat[1] if K_h_flat.size > 1 else 0.0
        K_2 = K_h_flat[2] if K_h_flat.size > 2 else 0.0
        K_3 = K_h_flat[3] if K_h_flat.size > 3 else 0.0

        # Map stiffness roles: comfort/translation driver vs. trajectory tracking/stabilization
        comfort_factor = (abs(K_0) + abs(K_2)) / 2.0
        trajectory_factor = (abs(K_1) + abs(K_3)) / 2.0

        # --- 3. Compute Bounded Dominance Difference ---
        normalization_scale = 50.0
        stiffness_diff = (comfort_factor - trajectory_factor) / normalization_scale
        stiffness_diff = np.clip(stiffness_diff, -1.0, 1.0)

        # --- 4. Capture Base Gains Once ---
        if not hasattr(self, "proportional_gain_base"):
            self.proportional_gain_base = self.K_p.copy()
            self.derivative_gain_base = self.K_d.copy()

        # --- 5. Calculate Adaptive Scaling Factor ---
        # If comfort > trajectory (user actively driving): stiffness_diff > 0 -> gain_scale drops (compliance)
        # If trajectory > comfort (user hesitating/stabilizing): stiffness_diff < 0 -> gain_scale rises (assistance)
        gain_scale = 1.0 - 0.7 * stiffness_diff

        # Multiply gain scales directly across the matrices
        adapted_Kp = self.proportional_gain_base * gain_scale
        adapted_Kd = self.derivative_gain_base * gain_scale

        # --- 6. Lower Bound Safety Guards ---
        adapted_Kp = np.maximum(adapted_Kp, self.proportional_gain_base * 0.1)
        adapted_Kd = np.maximum(adapted_Kd, self.derivative_gain_base * 0.1)

        # --- 7. Terminal Docking Zone Override ---
        TERMINAL_THRESHOLD = 0.85
        if progress_along_path > TERMINAL_THRESHOLD or self.in_docking_zone_already_once:
            self.in_docking_zone_already_once = True
            
            terminal_fraction = (progress_along_path - TERMINAL_THRESHOLD) / (1.0 - TERMINAL_THRESHOLD)
            t_scale = terminal_fraction ** 2

            # Drastically scale up positioning authority (P gain) to drive tracking error to zero
            # Scale up derivative damping (D gain) concurrently to guarantee loop stability near docking
            adapted_Kp *= (1.0 + 10.0 * t_scale)
            adapted_Kd *= (1.0 + 3.0 * t_scale)

        # --- 8. Update State Feedback Matrices ---
        self.K_p = adapted_Kp
        self.K_d = adapted_Kd