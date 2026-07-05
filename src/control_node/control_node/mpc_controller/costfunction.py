import casadi as ca
import numpy as np

WEIGHT_COMFORT = 1
WEIGHT_TRAJECTORY = 1
WEIGHT_GOAL = 1


class CostFunction:
    def __init__(
        self,
        weight_comfort=WEIGHT_COMFORT,
        weight_trajectory=WEIGHT_TRAJECTORY,
        weight_goal=WEIGHT_GOAL,
    ):
        self.weight_comfort = weight_comfort
        self.weight_trajectory = weight_trajectory
        self.weight_goal = weight_goal
        self.set_weights(weight_comfort, weight_trajectory, weight_goal)

    def set_weights(
        self,
        weight_comfort=None,
        weight_trajectory=None,
        weight_goal=None,
    ):
        if weight_comfort is not None:
            self.weight_comfort = weight_comfort
        if weight_trajectory is not None:
            self.weight_trajectory = weight_trajectory
        if weight_goal is not None:
            self.weight_goal = weight_goal

        self.R = ca.diag(
            [
                self.weight_comfort * 0.01,  # ux
                self.weight_comfort * 0.01,  # uy
            ]
        )
        self.Q = ca.diag(
            [
                self.weight_trajectory * 10.0,  # x
                0,  # vx
                self.weight_trajectory * 10.0,  # y
                0,  # vy
            ]
        )
        self.P = ca.diag(
            [
                self.weight_goal * 100.0,  # x
                self.weight_goal * 10.0,  # vx
                self.weight_goal * 100.0,  # y
                self.weight_goal * 10.0,  # vy
            ]
        )

    @staticmethod
    def _reshape_control_sequence(u, prediction_horizon):
        """Convert a flattened control vector into an N x 2 sequence."""
        return ca.reshape(u, 2, prediction_horizon).T

    def get_parameters(self):
        return {
            "Controller Type": "MPC Controller",
            "weight_comfort": self.weight_comfort,
            "weight_trajectory": self.weight_trajectory,
            "weight_goal": self.weight_goal,
            "Q": np.array(self.Q).tolist(),
            "R": np.array(self.R).tolist(),
            "P": np.array(self.P).tolist(),
        }

    def get_symbolic_cost(
        self,
        x_predicted,
        u,
        reference_trajectory,
        goal_state=None,
    ):
        J = 0

        prediction_horizon = int(x_predicted.shape[0])
        u_sequence = self._reshape_control_sequence(u, prediction_horizon)

        for k in range(prediction_horizon):
            e_track = x_predicted[k, :] - reference_trajectory[k, :]
            uk = u_sequence[k, :]

            J += ca.mtimes([e_track, self.Q, e_track.T])
            J += ca.mtimes([uk, self.R, uk.T])

        final_goal_state = (
            goal_state if goal_state is not None else reference_trajectory[-1, :]
        )
        goal_T = final_goal_state.T
        e_goal = x_predicted[k, :] - goal_T
        J += ca.mtimes([e_goal, self.P, e_goal.T])

        return J
