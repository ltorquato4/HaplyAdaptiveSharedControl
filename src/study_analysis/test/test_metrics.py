import json
from pathlib import Path

import numpy as np
import pandas as pd
import pytest
from study_analysis.loader import Attempt
from study_analysis.metrics import (
    calculate_trial_metrics,
    summarize_conditions,
)


def _frame():
    return pd.DataFrame(
        {
            "timestamp": [10.0, 11.0, 12.0],
            "elapsed_s": [0.0, 1.0, 2.0],
            "session_id": ["session"] * 3,
            "trial_id": [1] * 3,
            "attempt_id": [1] * 3,
            "study_phase": ["normal"] * 3,
            "study_controller_mode": ["fixed"] * 3,
            "start_x": [0.0] * 3,
            "start_y": [0.0] * 3,
            "end_x": [1.0] * 3,
            "end_y": [0.0] * 3,
            "cursor_x": [0.0, 0.5, 1.0],
            "cursor_y": [0.0, 0.0, 0.0],
            "max_control_amplitude": [2.0] * 3,
            "u_h": [json.dumps([1.0, 0.0, 0.0])] * 3,
            "u_a": [json.dumps([-1.0, 0.0, 0.0])] * 3,
            "K_h": [json.dumps([2.0, -0.4, 0, 0, 0, 0, 1.5, -0.25])] * 3,
            "K_a": [
                json.dumps(
                    {
                        "weight_comfort": 1.0,
                        "weight_trajectory": 2.0,
                        "weight_goal": 3.0,
                    }
                )
            ]
            * 3,
        }
    )


def test_straight_trajectory_metrics_have_known_values():
    attempt = Attempt(
        Path("trial.csv"),
        _frame(),
        {
            "session_id": "session",
            "trial_id": 1,
            "attempt_id": 1,
            "controller_family": "mpc",
            "study_controller_mode": "fixed",
            "study_phase": "normal",
        },
        "COMPLETED",
        "",
    )

    metrics, enriched = calculate_trial_metrics(attempt)

    assert metrics["endpoint_error"] == 0.0
    assert metrics["cross_track_rmse"] == 0.0
    assert metrics["path_length_ratio"] == 1.0
    assert metrics["estimated_input_alignment_mean"] == -1.0
    assert metrics["estimated_input_opposing_mean"] == 1.0
    assert metrics["assistant_effort"] == 2.0
    assert metrics["estimator_kp_y_final"] == 1.5
    assert np.allclose(enriched.progress, [0.0, 0.5, 1.0])


def test_condition_summary_is_descriptive():
    frame = pd.DataFrame(
        [
            {
                "controller_family": "mpc",
                "controller_mode": "fixed",
                "phase": "normal",
                "segment": "A",
                "duration_s": 1.0,
            },
            {
                "controller_family": "mpc",
                "controller_mode": "fixed",
                "phase": "normal",
                "segment": "A",
                "duration_s": 3.0,
            },
        ]
    )
    summary = summarize_conditions(frame)
    assert summary.loc[0, "count"] == 2
    assert summary.loc[0, "duration_s_mean"] == 2.0
    assert summary.loc[0, "duration_s_median"] == 2.0


def test_state_feedback_saturation_uses_force_vector_norm():
    frame = _frame()
    frame["u_a"] = [json.dumps([1.5, 1.5, 0.0])] * len(frame)
    attempt = Attempt(
        Path("trial.csv"),
        frame,
        {
            "session_id": "session",
            "trial_id": 1,
            "attempt_id": 1,
            "controller_family": "state_feedback",
            "study_controller_mode": "fixed",
            "study_phase": "normal",
        },
        "COMPLETED",
        "",
    )

    metrics, _ = calculate_trial_metrics(attempt)

    assert metrics["assistant_saturation_fraction"] == 1.0


def test_mpc_saturation_retains_component_limit_semantics():
    frame = _frame()
    frame["u_a"] = [json.dumps([1.5, 1.5, 0.0])] * len(frame)
    attempt = Attempt(
        Path("trial.csv"),
        frame,
        {
            "session_id": "session",
            "trial_id": 1,
            "attempt_id": 1,
            "controller_family": "mpc",
            "study_controller_mode": "fixed",
            "study_phase": "normal",
        },
        "COMPLETED",
        "",
    )

    metrics, _ = calculate_trial_metrics(attempt)

    assert metrics["assistant_saturation_fraction"] == 0.0


def test_cursor_velocity_uses_unique_source_timestamps():
    frame = _frame().iloc[[0, 0, 1, 1, 2]].reset_index(drop=True)
    frame["timestamp"] = [10.0, 10.01, 11.0, 11.01, 12.0]
    frame["elapsed_s"] = [0.0, 0.01, 1.0, 1.01, 2.0]
    frame["cursor_elapsed_s"] = [0.0, 0.0, 1.0, 1.0, 2.0]
    attempt = Attempt(
        Path("trial.csv"),
        frame,
        {
            "session_id": "session",
            "trial_id": 1,
            "attempt_id": 1,
            "controller_family": "state_feedback",
            "study_controller_mode": "fixed",
            "study_phase": "normal",
        },
        "COMPLETED",
        "",
    )

    metrics, enriched = calculate_trial_metrics(attempt)

    assert metrics["speed_mean"] == pytest.approx(0.5)
    assert enriched["cursor_speed"].notna().sum() == 3
