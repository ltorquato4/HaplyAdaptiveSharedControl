"""Strict loading and validation of study logger output."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd


class SchemaError(ValueError):
    """Raised when input cannot be analyzed safely."""


@dataclass
class Attempt:
    """One uniquely identified recorded trial attempt."""

    path: Path
    data: pd.DataFrame
    metadata: dict
    outcome: str = "UNKNOWN"
    reason: str = ""

    @property
    def key(self):
        return (
            str(self.metadata["session_id"]),
            int(self.metadata["trial_id"]),
            int(self.metadata["attempt_id"]),
        )


REQUIRED_COLUMNS = {
    "timestamp",
    "session_id",
    "trial_id",
    "attempt_id",
    "study_phase",
    "study_controller_mode",
    "start_x",
    "start_y",
    "end_x",
    "end_y",
    "cursor_x",
    "cursor_y",
    "u_h",
    "u_a",
    "K_h",
    "K_a",
}
CONSTANT_COLUMNS = (
    "session_id",
    "trial_id",
    "attempt_id",
    "study_phase",
    "study_controller_mode",
    "start_x",
    "start_y",
    "end_x",
    "end_y",
)
LEGACY_COMPONENT_COLUMNS = {"u_h_x", "u_h_y", "U_a_x", "U_a_y"}


def parse_json_array(value, minimum_length=2):
    """Parse a logger JSON array, returning NaNs for missing samples."""
    if value is None or (isinstance(value, float) and np.isnan(value)):
        return np.full(minimum_length, np.nan)
    try:
        parsed = json.loads(value) if isinstance(value, str) else value
        array = np.asarray(parsed, dtype=float).reshape(-1)
    except (TypeError, ValueError, json.JSONDecodeError) as exc:
        raise SchemaError(f"invalid JSON array: {value!r}") from exc
    if array.size < minimum_length:
        raise SchemaError(
            f"expected at least {minimum_length} array entries, got {array.size}"
        )
    return array


def _constant_value(frame, column):
    values = frame[column].dropna().unique()
    if len(values) != 1:
        raise SchemaError(f"{column} must be constant within one attempt")
    return values[0]


def _load_outcomes(input_dir):
    path = input_dir / "trial_attempts.csv"
    if not path.exists():
        return {}
    frame = pd.read_csv(path)
    required = {"session_id", "trial_id", "attempt_id", "outcome", "reason"}
    if not required.issubset(frame.columns):
        raise SchemaError("trial_attempts.csv is missing required columns")
    return {
        (str(row.session_id), int(row.trial_id), int(row.attempt_id)): (
            str(row.outcome),
            "" if pd.isna(row.reason) else str(row.reason),
        )
        for row in frame.itertuples()
    }


def load_session(input_directory, controller_family=None, input_source=None):
    """Load current-schema attempt CSVs and return attempts and quality rows."""
    input_dir = Path(input_directory)
    if not input_dir.is_dir():
        raise SchemaError(f"input directory does not exist: {input_dir}")

    manifest_path = input_dir / "session_manifest.json"
    manifest = {}
    if manifest_path.exists():
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise SchemaError("invalid session_manifest.json") from exc

    files = sorted(input_dir.glob("trial_*_attempt_*.csv"))
    if not files:
        files = sorted(
            path
            for path in input_dir.glob("*.csv")
            if path.name != "trial_attempts.csv"
        )
    if not files:
        raise SchemaError(f"no trial CSV files found in {input_dir}")

    outcomes = _load_outcomes(input_dir)
    attempts = []
    quality = []
    for path in files:
        frame = pd.read_csv(path)
        if LEGACY_COMPONENT_COLUMNS.intersection(frame.columns):
            raise SchemaError(
                f"{path.name} uses the unsupported legacy component-column schema"
            )
        missing = sorted(REQUIRED_COLUMNS.difference(frame.columns))
        if missing:
            raise SchemaError(f"{path.name} is missing columns: {', '.join(missing)}")
        if frame.empty:
            quality.append(
                {"file": path.name, "severity": "warning", "issue": "empty_attempt"}
            )
            continue

        for column in (
            "timestamp",
            "trial_id",
            "attempt_id",
            "start_x",
            "start_y",
            "end_x",
            "end_y",
            "cursor_x",
            "cursor_y",
        ):
            frame[column] = pd.to_numeric(frame[column], errors="coerce")
        for column in (
            "monotonic_timestamp",
            "missed_cycle_count",
            "cursor_timestamp",
            "cursor_sample_sequence",
        ):
            if column in frame.columns:
                frame[column] = pd.to_numeric(frame[column], errors="coerce")
        if frame["timestamp"].isna().any():
            raise SchemaError(f"{path.name} contains invalid timestamps")
        if not frame["timestamp"].is_monotonic_increasing:
            raise SchemaError(f"{path.name} timestamps are not monotonic")
        timestamp_deltas = frame["timestamp"].diff().dropna()
        if (timestamp_deltas <= 0.0).any():
            raise SchemaError(f"{path.name} timestamps must be strictly increasing")

        metadata = {
            column: _constant_value(frame, column) for column in CONSTANT_COLUMNS
        }
        metadata["controller_family"] = (
            controller_family
            or _optional_constant(frame, "controller_family")
            or manifest.get("controller_family")
        )
        metadata["input_source"] = (
            input_source
            or _optional_constant(frame, "input_source")
            or manifest.get("input_source")
        )
        metadata["participant_id"] = (
            _optional_constant(frame, "participant_id")
            or manifest.get("participant_id")
            or "unknown"
        )
        if metadata["controller_family"] not in {"mpc", "state_feedback"}:
            raise SchemaError(
                f"{path.name} needs --controller-family mpc|state_feedback"
            )
        if metadata["input_source"] not in {"mouse", "haply"}:
            raise SchemaError(f"{path.name} needs --input-source mouse|haply")
        _validate_manifest_attempt(path, metadata, manifest)

        frame = frame.copy()
        timing_column = "timestamp"
        if "monotonic_timestamp" in frame.columns:
            monotonic = frame["monotonic_timestamp"]
            if monotonic.isna().any():
                raise SchemaError(
                    f"{path.name} contains invalid monotonic timestamps"
                )
            monotonic_deltas = monotonic.diff().dropna()
            if (monotonic_deltas <= 0.0).any():
                raise SchemaError(
                    f"{path.name} monotonic timestamps must be strictly increasing"
                )
            timing_column = "monotonic_timestamp"
        frame["elapsed_s"] = (
            frame[timing_column] - frame[timing_column].iloc[0]
        )

        if "cursor_timestamp" in frame.columns:
            cursor_timestamps = frame["cursor_timestamp"].dropna()
            if not cursor_timestamps.is_monotonic_increasing:
                raise SchemaError(
                    f"{path.name} cursor timestamps must not decrease"
                )
            frame["cursor_elapsed_s"] = np.nan
            if len(cursor_timestamps):
                first_cursor_timestamp = cursor_timestamps.iloc[0]
                frame.loc[cursor_timestamps.index, "cursor_elapsed_s"] = (
                    cursor_timestamps - first_cursor_timestamp
                )

        if "missed_cycle_count" in frame.columns:
            missed_cycles = frame["missed_cycle_count"]
            if (
                missed_cycles.isna().any()
                or (missed_cycles < 0).any()
                or not missed_cycles.is_monotonic_increasing
            ):
                raise SchemaError(f"{path.name} has invalid missed-cycle counts")
            if missed_cycles.iloc[-1] > 0:
                quality.append(
                    {
                        "file": path.name,
                        "severity": "warning",
                        "issue": "logger_missed_cycles",
                        "value": int(missed_cycles.iloc[-1]),
                    }
                )
        cursor_missing = frame[["cursor_x", "cursor_y"]].isna().any(axis=1).mean()
        if cursor_missing:
            quality.append(
                {
                    "file": path.name,
                    "severity": "warning",
                    "issue": "missing_cursor_fraction",
                    "value": float(cursor_missing),
                }
            )
        gaps = frame[timing_column].diff().dropna()
        if len(gaps) and gaps.max() > max(0.05, 5.0 * gaps.median()):
            quality.append(
                {
                    "file": path.name,
                    "severity": "warning",
                    "issue": f"{timing_column}_gap_s",
                    "value": float(gaps.max()),
                }
            )
        if timing_column == "monotonic_timestamp":
            clock_disagreement = (timestamp_deltas - gaps).abs()
            if len(clock_disagreement) and clock_disagreement.max() > 0.05:
                quality.append(
                    {
                        "file": path.name,
                        "severity": "warning",
                        "issue": "wall_clock_step_s",
                        "value": float(clock_disagreement.max()),
                    }
                )
        if "cursor_timestamp" in frame.columns:
            source_times = frame["cursor_timestamp"].dropna().drop_duplicates()
            source_gaps = source_times.diff().dropna()
            if (
                len(source_gaps)
                and source_gaps.max() > max(0.05, 5.0 * source_gaps.median())
            ):
                quality.append(
                    {
                        "file": path.name,
                        "severity": "warning",
                        "issue": "cursor_timestamp_gap_s",
                        "value": float(source_gaps.max()),
                    }
                )
        key = (
            str(metadata["session_id"]),
            int(metadata["trial_id"]),
            int(metadata["attempt_id"]),
        )
        outcome, reason = outcomes.get(key, ("UNKNOWN", ""))
        attempts.append(Attempt(path, frame, metadata, outcome, reason))

    if not attempts:
        raise SchemaError("no non-empty attempts can be analyzed")
    return attempts, quality, manifest


def _validate_manifest_attempt(path, metadata, manifest):
    """Ensure an attempt agrees with its retained session definition."""
    if not manifest:
        return
    manifest_session = str(manifest.get("session_id", ""))
    if manifest_session and manifest_session != str(metadata["session_id"]):
        raise SchemaError(f"{path.name} session_id disagrees with the manifest")
    for key in ("participant_id", "controller_family", "input_source"):
        value = manifest.get(key)
        if value is not None and str(value) != str(metadata[key]):
            raise SchemaError(f"{path.name} {key} disagrees with the manifest")

    schedule = manifest.get("schedule", [])
    if not schedule:
        raise SchemaError("session_manifest.json has no task schedule")
    trial_id = int(metadata["trial_id"])
    if manifest.get("loop_tasks", False):
        task = schedule[trial_id % len(schedule)]
    else:
        if trial_id < 0 or trial_id >= len(schedule):
            raise SchemaError(f"{path.name} trial_id is outside the manifest schedule")
        task = schedule[trial_id]

    expected = {
        "study_phase": task.get("phase"),
        "study_controller_mode": task.get("controller_mode"),
        "start_x": task.get("start_point", {}).get("x"),
        "start_y": task.get("start_point", {}).get("y"),
        "end_x": task.get("end_point", {}).get("x"),
        "end_y": task.get("end_point", {}).get("y"),
    }
    for key, value in expected.items():
        if value is None:
            raise SchemaError(f"manifest task {trial_id} is missing {key}")
        actual = metadata[key]
        if isinstance(value, (int, float)):
            agrees = np.isclose(float(value), float(actual), rtol=0.0, atol=1e-9)
        else:
            agrees = str(value) == str(actual)
        if not agrees:
            raise SchemaError(f"{path.name} {key} disagrees with the manifest")


def _optional_constant(frame, column):
    if column not in frame.columns:
        return None
    values = frame[column].dropna().astype(str).unique()
    if len(values) > 1:
        raise SchemaError(f"{column} must be constant within one attempt")
    return values[0] if len(values) else None
