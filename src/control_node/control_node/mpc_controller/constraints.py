import casadi as ca

class Constraints:
    def __init__(
        self,
        max_control: tuple[float, float],
        max_velocity: tuple[float, float],
        x_bounds: tuple[float, float],
        y_bounds: tuple[float, float],
    ):
        self.u_max_x, self.u_max_y = max_control
        self.v_max_x, self.v_max_y = max_velocity

        self.x_min, self.x_max = x_bounds
        self.y_min, self.y_max = y_bounds

    @staticmethod
    def _reshape_control_sequence(u, prediction_horizon):
        """Convert a flattened control vector into an N x 2 sequence."""
        return ca.reshape(u, 2, prediction_horizon).T

    def get_symbolic_constraints(self, x_predicted, u):
        prediction_horizon = int(x_predicted.shape[0])
        u_sequence = self._reshape_control_sequence(u, prediction_horizon)

        x = x_predicted[:, 0]
        vx = x_predicted[:, 1]
        y = x_predicted[:, 2]
        vy = x_predicted[:, 3]

        ux = u_sequence[:, 0]
        uy = u_sequence[:, 1]

        return [
            # Control limits
            ux - self.u_max_x,
            -ux - self.u_max_x,
            uy - self.u_max_y,
            -uy - self.u_max_y,

            # Velocity limits
            vx - self.v_max_x,
            -vx - self.v_max_x,
            vy - self.v_max_y,
            -vy - self.v_max_y,

            # Position limits
            self.x_min - x,
            x - self.x_max,
            self.y_min - y,
            y - self.y_max,
        ]