import json

import pandas as pd
import pytest
from study_analysis.loader import SchemaError, load_session


def _write_attempt(directory, **overrides):
    row = {
        "timestamp": 1.0,
        "session_id": "session",
        "trial_id": 0,
        "attempt_id": 1,
        "study_phase": "normal",
        "study_controller_mode": "fixed",
        "start_x": 0.0,
        "start_y": 0.0,
        "end_x": 1.0,
        "end_y": 0.0,
        "cursor_x": 0.0,
        "cursor_y": 0.0,
        "u_h": json.dumps([0.0, 0.0, 0.0]),
        "u_a": json.dumps([0.0, 0.0, 0.0]),
        "K_h": json.dumps([0.0] * 8),
        "K_a": json.dumps({"weight_comfort": 1.0}),
    }
    row.update(overrides)
    pd.DataFrame([row, {**row, "timestamp": 1.01, "cursor_x": 0.1}]).to_csv(
        directory / "trial_000000_attempt_001.csv", index=False
    )


def test_loader_accepts_metadata_overrides_for_pre_session_current_logs(tmp_path):
    _write_attempt(tmp_path)
    attempts, quality, manifest = load_session(
        tmp_path, controller_family="mpc", input_source="mouse"
    )
    assert attempts[0].key == ("session", 0, 1)
    assert attempts[0].data.elapsed_s.iloc[0] == 0.0
    assert quality == []
    assert manifest == {}


def test_loader_rejects_legacy_component_columns(tmp_path):
    _write_attempt(tmp_path, u_h_x=0.0)
    with pytest.raises(SchemaError, match="legacy"):
        load_session(tmp_path, controller_family="mpc", input_source="mouse")


def test_loader_rejects_missing_required_column(tmp_path):
    _write_attempt(tmp_path)
    path = tmp_path / "trial_000000_attempt_001.csv"
    frame = pd.read_csv(path).drop(columns=["cursor_y"])
    frame.to_csv(path, index=False)
    with pytest.raises(SchemaError, match="cursor_y"):
        load_session(tmp_path, controller_family="mpc", input_source="mouse")


def test_loader_rejects_duplicate_timestamps(tmp_path):
    _write_attempt(tmp_path)
    path = tmp_path / "trial_000000_attempt_001.csv"
    frame = pd.read_csv(path)
    frame.loc[1, "timestamp"] = frame.loc[0, "timestamp"]
    frame.to_csv(path, index=False)
    with pytest.raises(SchemaError, match="strictly increasing"):
        load_session(tmp_path, controller_family="mpc", input_source="mouse")


def test_loader_rejects_task_that_disagrees_with_manifest(tmp_path):
    _write_attempt(
        tmp_path,
        controller_family="state_feedback",
        input_source="mouse",
    )
    manifest = {
        "session_id": "session",
        "controller_family": "state_feedback",
        "input_source": "mouse",
        "loop_tasks": False,
        "schedule": [
            {
                "phase": "normal",
                "controller_mode": "fixed",
                "start_point": {"x": 0.0, "y": 0.0},
                "end_point": {"x": 2.0, "y": 0.0},
            }
        ],
    }
    (tmp_path / "session_manifest.json").write_text(json.dumps(manifest))
    with pytest.raises(SchemaError, match="end_x disagrees"):
        load_session(tmp_path)


def test_loader_uses_monotonic_and_source_timing_diagnostics(tmp_path):
    _write_attempt(tmp_path)
    path = tmp_path / "trial_000000_attempt_001.csv"
    row = pd.read_csv(path).iloc[0].to_dict()
    rows = []
    for index in range(4):
        rows.append(
            {
                **row,
                "timestamp": [1.0, 1.01, 2.0, 2.01][index],
                "monotonic_timestamp": 10.0 + 0.01 * index,
                "missed_cycle_count": [0, 0, 2, 2][index],
                "cursor_timestamp": [20.0, 20.01, 20.20, 20.21][index],
                "cursor_sample_sequence": index + 1,
                "cursor_x": 0.1 * index,
            }
        )
    pd.DataFrame(rows).to_csv(path, index=False)

    attempts, quality, _ = load_session(
        tmp_path, controller_family="mpc", input_source="mouse"
    )

    assert attempts[0].data.elapsed_s.iloc[-1] == pytest.approx(0.03)
    assert attempts[0].data.cursor_elapsed_s.iloc[-1] == pytest.approx(0.21)
    issues = {entry["issue"] for entry in quality}
    assert "logger_missed_cycles" in issues
    assert "wall_clock_step_s" in issues
    assert "cursor_timestamp_gap_s" in issues
