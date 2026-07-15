from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import casadi as ca
import numpy as np

from .batch_predictor import BatchPredictor
from .constraints import Constraints
from .costfunction import CostFunction
from .linear_system_model import LinearSystemModel


@dataclass(frozen=True)
class ModelDependencies:
    """Pre-instantiated helper classes that define the optimization model."""

    system_model: LinearSystemModel
    constraints: Constraints
    cost_function: CostFunction
    batch_predictor: BatchPredictor


@dataclass
class StateDependencies:
    """Runtime data used to parameterize the optimization problem state."""

    x0: np.ndarray | None = None
    u_init: np.ndarray | None = None
    z_sequence: np.ndarray | None = None
    goal_state: np.ndarray | None = None


@dataclass(frozen=True)
class SymbolicProblem:
    """Container for symbolic MPC optimization components."""

    u: ca.MX
    x_pred: ca.MX
    J_total: ca.MX
    g: list[ca.MX]
    p: ca.MX


class Optimization:
    """MPC optimization orchestrator for symbolic setup and solve workflow."""

    def __init__(
        self,
        model_dependencies: ModelDependencies,
        state_dependencies: StateDependencies | None = None,
        state_dimension: int = 4,
        input_dimension: int = 2,
        start_point: Sequence[float] = (0.0, 0.0),
        end_point: Sequence[float] = (0.0, 0.0),
    ) -> None:
        self.model_dependencies = model_dependencies
        self.state_dependencies = state_dependencies or StateDependencies()
        self.state_dimension = state_dimension
        self.input_dimension = input_dimension

        # Save true, static start and end points of the path
        self.start_point = start_point
        self.end_point = end_point

        self.system_model = model_dependencies.system_model
        self.constraints = model_dependencies.constraints
        self.cost_function = model_dependencies.cost_function
        self.batch_predictor = model_dependencies.batch_predictor
        self.prediction_horizon = self.batch_predictor.N

        # 1. Setup the problem and solver exactly ONCE during initialization
        self.symbolic_problem = self._setup_nlp_problem()
        
        # 2. Give the solver a unique name tied to this object instance
        solver_name = f"mpc_solver_{id(self)}"
        self.solver = self._create_solver(self.symbolic_problem, solver_name)

    @staticmethod
    def _coerce_state_vector(
        state: np.ndarray | None, state_dimension: int
    ) -> np.ndarray:
        if state is None:
            raise ValueError("state must be provided")

        state_array = np.asarray(state, dtype=float).reshape(-1)

        if state_array.size == state_dimension:
            return state_array
        if state_dimension == 4 and state_array.size == 2:
            return np.array([state_array[0], 0.0, state_array[1], 0.0], dtype=float)

        raise ValueError(
            f"Expected state with {state_dimension} values or a 2D position, "
            f"got {state_array.size}"
        )

    def _build_reference_trajectory(
        self, x0: ca.MX, goal_state: ca.MX, prediction_horizon: int
    ) -> ca.MX:
        if prediction_horizon <= 0:
            raise ValueError("prediction_horizon must be positive")

        # True start and end coordinates of the global path segment
        x_start = self.start_point[0]
        y_start = self.start_point[1]
        x_goal = self.end_point[0]
        y_goal = self.end_point[1]

        # 1. Vector of the global path line: V = End - Start
        V_x = x_goal - x_start
        V_y = y_goal - y_start
        V_lensq = V_x**2 + V_y**2 + 1e-8  # Small epsilon to avoid division by zero

        # 2. Vector from Start to current position: W = Current - Start
        W_x = x0[0] - x_start
        W_y = x0[2] - y_start

        # 3. Project current position onto path: t_current = (W dot V) / (V dot V)
        t_current = (W_x * V_x + W_y * V_y) / V_lensq
        t_current = ca.fmax(0.0, ca.fmin(1.0, t_current))  # Keep progress bound to [0, 1]

        # 4. Step velocity calculation
        # To avoid over-saturating actuators, step forward at a percentage of max velocity
        dt = self.system_model.dt
        v_max = min(self.constraints.v_max_x, self.constraints.v_max_y)
        v_target = v_max * 0.8  # Target velocity (e.g., 80% of constraint limits)

        # Convert velocity (m/s) to progress velocity per second along our path
        path_length = ca.sqrt(V_lensq)
        t_step = (v_target * dt) / path_length

        rows = []
        for step in range(prediction_horizon):
            # Generate forward step points on the path starting from current projection
            t_step_k = t_current + step * t_step
            t_step_k = ca.fmax(0.0, ca.fmin(1.0, t_step_k))

            x_ref = x_start + t_step_k * V_x
            y_ref = y_start + t_step_k * V_y

            rows.append(ca.horzcat(x_ref, 0, y_ref, 0))

        return ca.vertcat(*rows)
    
    def _create_solver(self, symbolic_problem: SymbolicProblem, solver_name: str) -> ca.Function:
        g_concat = ca.vertcat(*symbolic_problem.g)

        nlp = {
            "x": symbolic_problem.u,
            "f": symbolic_problem.J_total,
            "g": g_concat,
            "p": symbolic_problem.p,
        }

        opts = {"ipopt.print_level": 0, "print_time": 0}
        return ca.nlpsol(solver_name, "ipopt", nlp, opts)

    def _get_disturbance_sequence_vector(self) -> np.ndarray:
        prediction_horizon = self.prediction_horizon
        state_dimension = self.state_dimension

        if self.state_dependencies.z_sequence is None:
            return np.zeros(prediction_horizon * state_dimension)

        return self.state_dependencies.z_sequence.flatten()

    def _predict_state_trajectory(
        self,
        x0: ca.MX,
        u: ca.MX,
        _z_seq: ca.MX,
        state_dimension: int,
        prediction_horizon: int,
    ) -> ca.MX:
        batch_matrices = self.batch_predictor.get_batch_matrices_casadi()
        A_bar = batch_matrices["A_bar"]
        B_bar = batch_matrices["B_bar"]

        x_pred_flat = A_bar @ x0 + B_bar @ u
        return ca.reshape(x_pred_flat, state_dimension, prediction_horizon).T

    def _prepare_optimization_inputs(self) -> tuple[np.ndarray, np.ndarray]:
        prediction_horizon = self.prediction_horizon

        x0_val = self._coerce_state_vector(
            self.state_dependencies.x0, self.state_dimension
        )
        z_sequence_val = np.asarray(
            self._get_disturbance_sequence_vector(), dtype=float
        ).reshape(-1)
        goal_state_val = self._coerce_state_vector(
            self.state_dependencies.goal_state, self.state_dimension
        )

        # Fetch current dynamic weights from the cost function
        current_weights = np.array([
            self.cost_function.weight_comfort,
            self.cost_function.weight_trajectory,
            self.cost_function.weight_goal
        ], dtype=float)

        # Append weights to the parameter vector
        p_val = np.concatenate(
            [
                x0_val,
                z_sequence_val,
                goal_state_val,
                current_weights
            ]
        )

        u_init = (
            self.state_dependencies.u_init
            if self.state_dependencies.u_init is not None
            else np.zeros(prediction_horizon * self.input_dimension)
        )

        return u_init, p_val

    def _setup_nlp_problem(self) -> SymbolicProblem:
        prediction_horizon = self.prediction_horizon
        state_dimension, input_dimension = self.state_dimension, self.input_dimension

        u = ca.MX.sym("u", prediction_horizon * input_dimension)
        x0 = ca.MX.sym("x0", state_dimension)
        z_sequence = ca.MX.sym("z_seq", prediction_horizon * state_dimension)
        goal_state = ca.MX.sym("goal_state", state_dimension)
        
        # Define weights as symbolic parameters so they can change without recompiling
        w_comfort = ca.MX.sym("w_comfort", 1)
        w_trajectory = ca.MX.sym("w_trajectory", 1)
        w_goal = ca.MX.sym("w_goal", 1)

        x_pred = self._predict_state_trajectory(
            x0, u, z_sequence, state_dimension, prediction_horizon
        )

        x_ref_full = self._build_reference_trajectory(
            x0, goal_state, prediction_horizon
        )

        J_total = self.cost_function.get_symbolic_cost(
            x_pred,
            u,
            x_ref_full,
            goal_state,
            sym_weights=(w_comfort, w_trajectory, w_goal)
        )
        g = self.constraints.get_symbolic_constraints(x_pred, u)
        
        # Include weights in parameter vector p
        p = ca.vertcat(x0, z_sequence, goal_state, w_comfort, w_trajectory, w_goal)

        return SymbolicProblem(u=u, x_pred=x_pred, J_total=J_total, g=g, p=p)

    def _solve_nlp(
        self,
        solver: ca.Function,
        g: list[ca.MX],
        u_init: np.ndarray,
        p_val: np.ndarray,
    ) -> np.ndarray:
        n_constraints = int(ca.vertcat(*g).shape[0])

        sol = solver(
            x0=u_init,
            p=p_val,
            lbg=-np.inf * np.ones(n_constraints),
            ubg=np.zeros(n_constraints),
        )

        return np.array(sol["x"]).flatten()

    def set_state_dependencies(self, state_dependencies: StateDependencies) -> None:
        """Replace runtime state dependencies used by the optimization."""
        self.state_dependencies = state_dependencies

    def solve(self) -> np.ndarray:
        if self.state_dependencies.x0 is None:
            raise ValueError("Initial state x0 must be set in state_dependencies")
        if self.state_dependencies.goal_state is None:
            raise ValueError("Goal state must be set in state_dependencies")

        u_init, p_val = self._prepare_optimization_inputs()

        # Use the pre-compiled solver rather than re-creating it on every step
        u_optimum = self._solve_nlp(self.solver, self.symbolic_problem.g, u_init, p_val)

        return u_optimum

    def destroy(self) -> None:
        import gc

        attrs = (
            "state_dependencies",
            "model_dependencies",
            "system_model",
            "constraints",
            "cost_function",
            "batch_predictor",
            "solver",
            "symbolic_problem"
        )
        for attr in attrs:
            if hasattr(self, attr):
                setattr(self, attr, None)

        gc.collect()