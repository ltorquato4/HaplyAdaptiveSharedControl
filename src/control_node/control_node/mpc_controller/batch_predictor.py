import casadi as ca
import numpy as np


class BatchPredictor:
    """Compute horizon predictions with precomputed batch matrices."""

    def __init__(self, system_model, N):
        """Initialize predictor for ``system_model`` and horizon ``N``."""
        self.system_model = system_model
        self.N = N

        self._A_bar = self.build_A_bar()
        self._B_bar = self.build_B_bar()

    def build_A_bar(self):
        """Build stacked powers of A: [A; A^2; ...; A^N]."""
        state_dimension = self.system_model.A.shape[0]
        A_bar = np.zeros((state_dimension * self.N, state_dimension))
        A_power = self.system_model.A.copy()

        for i in range(self.N):
            row_start = i * state_dimension
            row_end = (i + 1) * state_dimension
            A_bar[row_start:row_end, :] = A_power
            A_power = self.system_model.A @ A_power

        return A_bar

    def build_B_bar(self):
        """Build lower-triangular input influence matrix for the horizon.

        B_bar has the block structure:
            [B      0      0    ...  0]
            [A B    B      0    ...  0]
            [A^2 B  A B    B    ...  0]
            [...]
        """
        state_dimension = self.system_model.A.shape[0]
        input_dimension = self.system_model.B.shape[1]
        B_bar = np.zeros((state_dimension * self.N, input_dimension * self.N))

        for i in range(self.N):
            for j in range(i + 1):
                steps = i - j
                A_power = np.linalg.matrix_power(self.system_model.A, steps)
                row_start = i * state_dimension
                row_end = (i + 1) * state_dimension
                col_start = j * input_dimension
                col_end = (j + 1) * input_dimension
                B_bar[row_start:row_end, col_start:col_end] = (
                    A_power @ self.system_model.B
                )

        return B_bar

    def predict(self, x0, u_sequence, z_sequence=None):
        """Predict a state trajectory from an initial state and control sequence."""
        state_dimension = self.system_model.A.shape[0]
        input_dimension = self.system_model.B.shape[1]

        x0_flat = np.asarray(x0, dtype=float).reshape(-1)
        if x0_flat.size != state_dimension:
            raise ValueError(
                f"x0 length ({x0_flat.size}) must match state dimension "
                f"({state_dimension})"
            )

        u_sequence_flat = np.array(u_sequence, dtype=float).reshape(-1)
        if u_sequence_flat.size != self.N * input_dimension:
            raise ValueError(
                f"u_sequence must contain {self.N * input_dimension} values, "
                f"got {u_sequence_flat.size}"
            )

        if z_sequence is None:
            z_sequence_flat = np.zeros(self.N * state_dimension)
        else:
            z_sequence_flat = np.array(z_sequence, dtype=float).reshape(-1)
            if z_sequence_flat.size != self.N * state_dimension:
                raise ValueError(
                    f"z_sequence must contain {self.N * state_dimension} values, "
                    f"got {z_sequence_flat.size}"
                )

        # Compute predicted states over the horizon
        x_s = self._A_bar @ x0_flat
        x_s += self._B_bar @ u_sequence_flat

        # Reshape x_s into list of state vectors
        predicted_states = [
            np.array(x_s[i * state_dimension : (i + 1) * state_dimension])
            for i in range(self.N)
        ]

        return predicted_states

    def get_batch_matrices_casadi(self):
        """Return precomputed batch matrices as CasADi DM objects."""
        return {
            "A_bar": ca.DM(self._A_bar),
            "B_bar": ca.DM(self._B_bar),
        }

    def destroy(self) -> None:
        """Clear cached batch matrices and the system_model reference."""
        self._A_bar = None
        self._B_bar = None
        self.system_model = None
