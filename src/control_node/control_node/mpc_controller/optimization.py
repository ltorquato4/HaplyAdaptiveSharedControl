from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

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

	x0: Optional[np.ndarray] = None
	u_init: Optional[np.ndarray] = None
	z_sequence: Optional[np.ndarray] = None
	goal_state: Optional[np.ndarray] = None


@dataclass(frozen=True)
class SymbolicProblem:
	"""Container for symbolic MPC optimization components."""

	u: ca.MX
	x_pred: ca.MX
	J_total: ca.MX
	g: list[ca.MX]
	p: ca.MX


class Optimization:
	"""MPC optimization orchestrator for symbolic setup and solve workflow.

	The class assembles CasADi symbols, creates the NLP solver, prepares
	runtime parameters, solves for the optimal input sequence, and computes
	the predicted trajectory from the optimized control inputs.
	"""

	def __init__(
		self,
		model_dependencies: ModelDependencies,
		state_dependencies: Optional[StateDependencies] = None,
		state_dimension: int = 4,
		input_dimension: int = 2,
	) -> None:
		"""Store model and state dependencies.

		Args:
			model_dependencies: Pre-built helper class instances.
			state_dependencies: Runtime values for initial state, references and optional warm-start vectors.
			state_dimension: State dimension for optimization.
			input_dimension: Input dimension for optimization.
		"""
		self.model_dependencies = model_dependencies
		self.state_dependencies = state_dependencies or StateDependencies()
		self.state_dimension = state_dimension
		self.input_dimension = input_dimension

		self.system_model = model_dependencies.system_model
		self.constraints = model_dependencies.constraints
		self.cost_function = model_dependencies.cost_function
		self.batch_predictor = model_dependencies.batch_predictor
		self.prediction_horizon = self.batch_predictor.N

	@staticmethod
	def _coerce_state_vector(state: np.ndarray, state_dimension: int) -> np.ndarray:
		"""Expand a 2D position into [x, vx, y, vy] or validate a 4D state."""
		state_array = np.asarray(state, dtype=float).reshape(-1)

		if state_array.size == state_dimension:
			return state_array
		if state_dimension == 4 and state_array.size == 2:
			return np.array([state_array[0], 0.0, state_array[1], 0.0], dtype=float)

		raise ValueError(
			f"Expected state with {state_dimension} values or a 2D position, got {state_array.size}"
		)

	def _build_reference_trajectory(self, x0: ca.MX, goal_state: ca.MX, prediction_horizon: int) -> ca.MX:
		"""Build a straight-line reference trajectory from start to goal.

		Args:
			x0: Initial state symbol.
			goal_state: Goal state symbol.
			prediction_horizon: Prediction horizon.

		Returns:
			Reference state trajectory of shape (prediction_horizon, 4).
		"""
		if prediction_horizon <= 0:
			raise ValueError("prediction_horizon must be positive")

		x_start = x0[0]
		y_start = x0[2]
		x_goal = goal_state[0]
		y_goal = goal_state[2]

		rows = []
		for step in range(prediction_horizon):
			alpha = 0.0 if prediction_horizon == 1 else step / (prediction_horizon - 1)
			x_ref = x_start + alpha * (x_goal - x_start)
			y_ref = y_start + alpha * (y_goal - y_start)
			rows.append(ca.horzcat(x_ref, 0, y_ref, 0))

		return ca.vertcat(*rows)

	def _create_solver(self, symbolic_problem: SymbolicProblem) -> ca.Function:
		"""Create CasADi NLP solver.

		Args:
			symbolic_problem: Symbolic problem definition.

		Returns:
			Configured NLP solver.
		"""
		g_concat = ca.vertcat(*symbolic_problem.g)

		nlp = {
			'x': symbolic_problem.u,
			'f': symbolic_problem.J_total,
			'g': g_concat,
			'p': symbolic_problem.p,
		}

		opts = {'ipopt.print_level': 0, 'print_time': 0}
		return ca.nlpsol('solver', 'ipopt', nlp, opts)

	def _get_disturbance_sequence_vector(self) -> np.ndarray:
		"""Return flattened disturbance sequence or a zero fallback vector."""
		prediction_horizon = self.prediction_horizon
		state_dimension = self.state_dimension

		if self.state_dependencies.z_sequence is None:
			return np.zeros(prediction_horizon * state_dimension)

		return self.state_dependencies.z_sequence.flatten()

	def _predict_state_trajectory(
		self, x0: ca.MX, u: ca.MX, z_seq: ca.MX, state_dimension: int, prediction_horizon: int
	) -> ca.MX:
		"""Predict state trajectory using batch matrices.

		Args:
			x0: Initial state symbol.
			u: Input sequence symbol.
			z_seq: Disturbance sequence symbol.
			state_dimension: State dimension.
			prediction_horizon: Prediction horizon.

		Returns:
			Predicted state trajectory of shape (prediction_horizon, state_dimension).
		"""
		batch_matrices = self.batch_predictor.get_batch_matrices_casadi()
		A_bar = batch_matrices["A_bar"]
		B_bar = batch_matrices["B_bar"]

		x_pred_flat = A_bar @ x0 + B_bar @ u
		return ca.reshape(x_pred_flat, state_dimension, prediction_horizon).T

	def _prepare_optimization_inputs(self) -> tuple[np.ndarray, np.ndarray]:
		"""Prepare initial guess and parameter values.

		Returns:
			Tuple of (u_init, p_val).
		"""
		prediction_horizon = self.prediction_horizon

		x0_val = self._coerce_state_vector(self.state_dependencies.x0, self.state_dimension)
		z_sequence_val = np.asarray(self._get_disturbance_sequence_vector(), dtype=float).reshape(-1)
		goal_state_val = self._coerce_state_vector(self.state_dependencies.goal_state, self.state_dimension)

		p_val = np.concatenate([
			x0_val,
			z_sequence_val,
			goal_state_val,
		])

		u_init = (
			self.state_dependencies.u_init
			if self.state_dependencies.u_init is not None
			else np.zeros(prediction_horizon * self.input_dimension)
		)

		return u_init, p_val

	def _setup_nlp_problem(self) -> SymbolicProblem:
		"""Setup symbolic NLP problem components.

		Returns:
			Symbolic NLP components packed in a dataclass.
		"""
		prediction_horizon = self.prediction_horizon
		state_dimension, input_dimension = self.state_dimension, self.input_dimension

		u = ca.MX.sym('u', prediction_horizon * input_dimension)
		x0 = ca.MX.sym('x0', state_dimension)
		z_sequence = ca.MX.sym('z_seq', prediction_horizon * state_dimension)
		goal_state = ca.MX.sym('goal_state', state_dimension)

		x_pred = self._predict_state_trajectory(x0, u, z_sequence, state_dimension, prediction_horizon)

		x_ref_full = self._build_reference_trajectory(x0, goal_state, prediction_horizon)

		J_total = self.cost_function.get_symbolic_cost(
			x_pred,
			u,
			x_ref_full,
			goal_state,
		)
		g = self.constraints.get_symbolic_constraints(x_pred, u)
		p = ca.vertcat(x0, z_sequence, goal_state)

		return SymbolicProblem(u=u, x_pred=x_pred, J_total=J_total, g=g, p=p)

	def _solve_nlp(
		self,
		solver: ca.Function,
		g: list[ca.MX],
		u_init: np.ndarray,
		p_val: np.ndarray,
	) -> np.ndarray:
		"""Solve the NLP problem.

		Args:
			solver: CasADi solver instance.
			g: Constraint expressions.
			u_init: Initial input guess.
			p_val: Parameter vector.

		Returns:
			Optimal input sequence.
		"""
		n_constraints = int(ca.vertcat(*g).shape[0])

		sol = solver(
			x0=u_init,
			p=p_val,
			lbg=-np.inf * np.ones(n_constraints),
			ubg=np.zeros(n_constraints),
		)

		return np.array(sol['x']).flatten()

	def set_state_dependencies(self, state_dependencies: StateDependencies) -> None:
		"""Replace runtime state dependencies used by the optimization."""
		self.state_dependencies = state_dependencies

	def solve(self) -> np.ndarray:
		"""Solve the MPC optimization problem using CasADi.

		Returns:
			Optimal input sequence.
		"""
		if self.state_dependencies.x0 is None:
			raise ValueError("Initial state x0 must be set in state_dependencies")
		if self.state_dependencies.goal_state is None:
			raise ValueError("Goal state must be set in state_dependencies")

		symbolic_problem = self._setup_nlp_problem()

		solver = self._create_solver(symbolic_problem)

		u_init, p_val = self._prepare_optimization_inputs()

		u_optimum = self._solve_nlp(solver, symbolic_problem.g, u_init, p_val)

		return u_optimum
