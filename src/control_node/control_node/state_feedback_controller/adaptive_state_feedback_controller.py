import numpy as np

from math import sqrt

from control_node.state_feedback_controller.state_feedback_controller import StateFeedbackController

FACTOR_DEFAULT = 0.5
FACTOR_RANGE = []
BORDER_ONE_DEFAULT = 0.3
BORDER_ONE_RANGE = [0.2, 0.4]
BORDER_TWO_DEFAULT = 0.7
BORDER_TWO_RANGE = [0.6, 0.8]

class AdaptiveStateFeedbackController(StateFeedbackController):
    def __init__(self, start_point, end_point, dt, node=None):
        super().__init__(start_point, end_point, dt, node)
    
    def adapt_gain(self, K_h: list[list[float]]) -> None:
        """
        The point to point connection is modelled as

        o--------------------------------------------o
      start     25%         50%         75%         end

      
        Rules for Adaptivity
        
        1. Close proximity to start and end point result in higher control gain
        2. K_a is to be low when the current_point is between 30% and 70%, the low value LOW_K_a is to be determined
        3. K_a is to be rising with K_a~-FACTOR in 0% till BORDER_ONE=30% and to be rising with K_a~FACTOR between BORDER_TWO=70% and 100%
        4. For higher K_h, border_one will be moved up to 40% and border_two low to 60%
        5. For lower K_h, border_one will be moved low to 20% and border_two up to 80%
        6. The value for FACTOR_DEFAULT and FACTOR_RANGE
        """

        # --- Distance calculations ---
        distance_start_to_end = np.linalg.norm(
            self.experiment_end_point - self.experiment_start_point
        )

        distance_start_to_current = np.linalg.norm(
            np.array(self.current_point) - self.experiment_start_point
        )

        # Normalized progress along trajectory in range [0, 1]
        progress_along_path = (
            distance_start_to_current / distance_start_to_end
            if distance_start_to_end > 1e-6
            else 0.0
        )
        progress_along_path = np.clip(progress_along_path, 0.0, 1.0)

        # --- Human gain magnitude ---
        K_h = np.array(K_h)
        K_h_magnitude = np.linalg.norm(K_h)

        # Smooth normalization into [0, 1]
        normalized_human_gain = np.tanh(K_h) # TODO this needs to be done based on the actual values of K_h

        # --- Adaptive borders ---
        border_one = (
            BORDER_ONE_RANGE[0]
            + normalized_human_gain * (BORDER_ONE_RANGE[1] - BORDER_ONE_RANGE[0])
        )

        border_two = (
            BORDER_TWO_RANGE[1]
            - normalized_human_gain * (BORDER_TWO_RANGE[1] - BORDER_TWO_RANGE[0])
        )

        # --- Adaptive scaling factor ---
        scaling_factor = FACTOR_DEFAULT
        if FACTOR_RANGE:
            scaling_factor = (
                FACTOR_RANGE[0]
                + normalized_human_gain * (FACTOR_RANGE[1] - FACTOR_RANGE[0])
            )

        # --- Compute adaptive gain scaling ---
        if progress_along_path < border_one:
            # Near start: high gain decreasing toward middle
            gain_scale = 1.0 - scaling_factor * (
                progress_along_path / border_one
            )

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


