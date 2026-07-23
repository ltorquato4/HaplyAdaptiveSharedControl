"""Focused tests for timestamped MPC runtime behavior."""

import casadi as ca
import numpy as np
import pytest
from control_node.mpc_controller.mpc_controller import MpcController
from control_node.mpc_controller.optimization import Optimization


class FakeOptimizer:
    """Capture MPC state without invoking IPOPT."""

    def __init__(self):
        self.state = None

    def set_state_dependencies(self, state):
        self.state = state

    def solve(self):
        return np.zeros(4, dtype=float)


class FakeSolver:
    """Provide configurable CasADi-like solve status."""

    def __init__(self, success, values):
        self.success = success
        self.values = values

    def __call__(self, **_kwargs):
        return {"x": self.values}

    def stats(self):
        return {
            "success": self.success,
            "return_status": "Solve_Succeeded" if self.success else "Failed",
        }


def _controller_without_solver():
    controller = object.__new__(MpcController)
    controller.experiment_start_point = np.array([0.0, 0.0])
    controller.experiment_end_point = np.array([1.0, 0.0])
    controller.dt = 0.1
    controller.current_point = None
    controller.prev_position = None
    controller.prev_cursor_timestamp_s = None
    controller.u_a = np.zeros(2)
    controller.prediction_horizon = 2
    controller.max_control = (10.0, 10.0)
    controller.u_init = np.zeros(4)
    controller.optimizer = FakeOptimizer()
    return controller


def test_mpc_velocity_uses_source_timestamp_and_rejects_duplicates():
    controller = _controller_without_solver()

    controller.compute_control((0.0, 0.0), timestamp_s=1.0)
    controller.compute_control((0.001, 0.0), timestamp_s=1.01)

    state = controller.optimizer.state.x0
    assert state == pytest.approx([0.001, 0.1, 0.0, 0.0])
    with pytest.raises(ValueError, match="timestamp must increase"):
        controller.compute_control((0.002, 0.0), timestamp_s=1.01)


def test_mpc_retry_reset_clears_timestamp_and_warm_start():
    controller = _controller_without_solver()
    controller.compute_control((0.0, 0.0), timestamp_s=1.0)
    controller.u_init[:] = 3.0

    controller.reset_runtime_state()
    controller.compute_control((0.5, 0.0), timestamp_s=10.0)

    assert controller.optimizer.state.x0 == pytest.approx([0.5, 0.0, 0.0, 0.0])
    assert controller.u_init == pytest.approx(np.zeros(4))


def test_optimizer_rejects_failed_and_nonfinite_solutions():
    constraints = [ca.MX.zeros(1)]
    failed = FakeSolver(False, np.zeros(2))
    with pytest.raises(RuntimeError, match="MPC solver failed"):
        Optimization._solve_nlp(None, failed, constraints, np.zeros(2), np.zeros(2))

    nonfinite = FakeSolver(True, np.array([np.nan, 0.0]))
    with pytest.raises(RuntimeError, match="non-finite"):
        Optimization._solve_nlp(
            None,
            nonfinite,
            constraints,
            np.zeros(2),
            np.zeros(2),
        )

    successful = FakeSolver(True, np.array([1.0, -1.0]))
    optimum = Optimization._solve_nlp(
        None,
        successful,
        constraints,
        np.zeros(2),
        np.zeros(2),
    )
    assert optimum == pytest.approx([1.0, -1.0])
