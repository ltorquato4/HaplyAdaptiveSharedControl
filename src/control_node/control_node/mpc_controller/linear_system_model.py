import casadi as ca
import numpy as np


class LinearSystemModel:
    """Linear state-space model x(k+1) = A x(k) + B u(k) + z(k)."""

    def __init__(self, dt):
        """Initialize the model with sampling time ``dt``."""
        self.dt = dt
        self._A = None
        self._B = None
        self._construct_system_matrices()

    def _construct_system_matrices(self):
        """Construct state transition matrix ``A`` and input matrix ``B``."""
        dt = self.dt
        dt2 = dt * dt

        self._A = np.array(
            [
                [1.0, dt, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, dt],
                [0.0, 0.0, 0.0, 1],
            ]
        )

        self._B = np.array([[0.5 * dt2, 0], [dt, 0], [0.0, 0.5 * dt2], [0.0, dt]])

    @property
    def A(self):
        """State transition matrix."""
        if self._A is None:
            self._construct_system_matrices()
        return self._A

    @property
    def B(self):
        """Input matrix."""
        if self._B is None:
            self._construct_system_matrices()
        return self._B

    def get_casadi_system_matrices(self):
        """Return system matrices as CasADi DM objects."""
        return {
            "A": ca.DM(self._A),
            "B": ca.DM(self._B),
        }

    def initialize_x(self, start_point: list[float]):
        """Initializes state vector x(t)"""
        x = np.array([start_point[0], 0, start_point[1], 0])
        return x

    def initialize_x_casadi(self, start_point: list[float]):
        """Return initial state as a CasADi column vector DM(4x1)."""
        return ca.DM(self.initialize_x(start_point)).reshape((4, 1))
