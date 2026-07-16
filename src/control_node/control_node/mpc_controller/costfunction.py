import casadi as ca
import numpy as np

WEIGHT_COMFORT = 1
WEIGHT_TRAJECTORY = 10
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
                self.weight_comfort,        # ux
                self.weight_comfort,        # uy
            ]
        )
        self.Q = ca.diag(
            [
                self.weight_trajectory,     # x
                0,                          # vx
                self.weight_trajectory,     # y
                0,                          # vy
            ]
        )
        self.P = ca.diag(
            [
                self.weight_goal,           # x
                self.weight_goal * 100.0,   # vx
                self.weight_goal,           # y
                self.weight_goal * 100.0,   # vy
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
        sym_weights=None,
    ):
        J = 0

        prediction_horizon = int(x_predicted.shape[0])
        u_sequence = self._reshape_control_sequence(u, prediction_horizon)

        # Use symbolic weights if provided, otherwise fallback to numerical matrices
        if sym_weights is not None:
            w_comfort, w_traj, w_goal = sym_weights
            R = ca.diag(ca.vcat([w_comfort * 0.01, w_comfort * 0.01]))
            Q = ca.diag(ca.vcat([w_traj * 10.0, 0.0, w_traj * 10.0, 0.0]))
            P = ca.diag(ca.vcat([w_goal * 100.0, w_goal * 10.0, w_goal * 100.0, w_goal * 10.0]))
        else:
            R = self.R
            Q = self.Q
            P = self.P

        # Stage costs
        for k in range(prediction_horizon):
            e_track = x_predicted[k, :] - reference_trajectory[k, :]
            uk = u_sequence[k, :]

            J += ca.mtimes([e_track, Q, e_track.T])
            J += ca.mtimes([uk, R, uk.T])

        # Terminal cost (Using explicit -1 indexing)
        if goal_state is not None:
            # If goal_state is passed as a 4x1 column vector, reshape/transpose to 1x4
            final_goal_state = ca.reshape(goal_state, 1, 4)
        else:
            final_goal_state = reference_trajectory[-1, :]  # Already 1x4

        e_goal = x_predicted[-1, :] - final_goal_state
        J += ca.mtimes([e_goal, P, e_goal.T])

        return J
    
    def destroy(self) -> None:
        """Release CasADi weight matrices (R, Q, P) held by this instance."""
        self.R = None
        self.Q = None
        self.P = None