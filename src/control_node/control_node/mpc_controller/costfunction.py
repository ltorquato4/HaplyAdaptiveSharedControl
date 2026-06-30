import casadi as ca

WEIGHT_COMFORT = 1
WEIGHT_TRAJECTORY = 1
WEIGHT_GOAL = 1

class CostFunction:
    def __init__(
        self,
        weight_comfort = WEIGHT_COMFORT,
        weight_trajectory = WEIGHT_TRAJECTORY,
        weight_goal = WEIGHT_GOAL,
    ):
        self.R = ca.diag([
            weight_comfort * 0.01,  # ux
            weight_comfort * 0.01,  # uy
        ])
        self.Q = ca.diag([
            weight_trajectory * 10.0,  # x
            0,   # vx
            weight_trajectory * 10.0,  # y
            0,   # vy
        ])
        self.P = ca.diag([
            weight_goal * 100.0, # x
            weight_goal * 10.0,  # vx
            weight_goal * 100.0, # y
            weight_goal * 10.0,  # vy
        ])

    @staticmethod
    def _reshape_control_sequence(u, prediction_horizon):
        """Convert a flattened control vector into an N x 2 sequence."""
        return ca.reshape(u, 2, prediction_horizon).T

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

        final_goal_state = goal_state if goal_state is not None else reference_trajectory[-1, :]
        e_goal = x_predicted[-1, :] - final_goal_state
        J += ca.mtimes([e_goal, self.P, e_goal.T])

        return J