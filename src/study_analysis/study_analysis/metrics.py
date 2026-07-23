"""Per-attempt and grouped study metrics."""

from __future__ import annotations

import json

import numpy as np
import pandas as pd

from .loader import Attempt, SchemaError, parse_json_array


def _arrays(series, minimum_length):
    values = []
    for value in series:
        values.append(parse_json_array(value, minimum_length))
    width = max(array.size for array in values)
    result = np.full((len(values), width), np.nan)
    for index, value in enumerate(values):
        result[index, : value.size] = value
    return result


def _safe_mean(values):
    finite = np.asarray(values, dtype=float)
    finite = finite[np.isfinite(finite)]
    return float(np.mean(finite)) if finite.size else np.nan


def _safe_std(values):
    finite = np.asarray(values, dtype=float)
    finite = finite[np.isfinite(finite)]
    return float(np.std(finite)) if finite.size else np.nan


def enrich_attempt(attempt: Attempt):
    """Add task-frame geometry and parsed signal columns without reordering."""
    frame = attempt.data.copy()
    start = np.array(
        [frame.start_x.iloc[0], frame.start_y.iloc[0]], dtype=float
    )
    end = np.array([frame.end_x.iloc[0], frame.end_y.iloc[0]], dtype=float)
    path = end - start
    length = float(np.linalg.norm(path))
    if not np.isfinite(length) or length <= 0:
        raise SchemaError(f"{attempt.path.name} has a zero-length task")
    position = frame[["cursor_x", "cursor_y"]].to_numpy(dtype=float)
    relative = position - start
    frame["progress"] = relative @ path / (length * length)
    frame["cross_track_error"] = (
        np.abs(relative[:, 0] * path[1] - relative[:, 1] * path[0]) / length
    )

    timing_column = (
        "cursor_elapsed_s" if "cursor_elapsed_s" in frame else "elapsed_s"
    )
    elapsed = frame[timing_column].to_numpy(dtype=float)
    valid = np.isfinite(position).all(axis=1) & np.isfinite(elapsed)
    speed = np.full(len(frame), np.nan)
    if valid.sum() >= 3:
        valid_indices = np.flatnonzero(valid)
        valid_positions = position[valid]
        valid_time = elapsed[valid]
        # Logger and Mapper timers are independent, so more than one logger row
        # may contain the same StudyCursor sample. Keep the last copy before
        # differentiating; its source timestamp is authoritative for kinematics.
        keep = np.r_[np.diff(valid_time) > 0.0, True]
        valid_indices = valid_indices[keep]
        valid_positions = valid_positions[keep]
        valid_time = valid_time[keep]
    if valid.sum() >= 3 and len(valid_time) >= 3:
        velocity = np.column_stack(
            [
                np.gradient(valid_positions[:, axis], valid_time)
                for axis in range(2)
            ]
        )
        speed[valid_indices] = np.linalg.norm(velocity, axis=1)
    frame["cursor_speed"] = speed

    frame.attrs["path_length"] = length
    return frame


def calculate_trial_metrics(attempt: Attempt):
    frame = enrich_attempt(attempt)
    valid = frame[["cursor_x", "cursor_y"]].dropna().index
    if len(valid) < 2:
        raise SchemaError(
            f"{attempt.path.name} has fewer than two cursor samples"
        )
    position = frame.loc[valid, ["cursor_x", "cursor_y"]].to_numpy(dtype=float)
    progress = frame.loc[valid, "progress"].to_numpy(dtype=float)
    error = frame.loc[valid, "cross_track_error"].to_numpy(dtype=float)
    elapsed = frame.loc[valid, "elapsed_s"].to_numpy(dtype=float)
    end = frame.loc[valid, ["end_x", "end_y"]].iloc[-1].to_numpy(dtype=float)
    steps = np.linalg.norm(np.diff(position, axis=0), axis=1)
    path_length = frame.attrs["path_length"]

    uh = _arrays(frame["u_h"], 2)[:, :2]
    ua = _arrays(frame["u_a"], 2)[:, :2]
    both = np.isfinite(uh).all(axis=1) & np.isfinite(ua).all(axis=1)
    uh_mag = np.linalg.norm(uh, axis=1)
    ua_mag = np.linalg.norm(ua, axis=1)
    denominator = uh_mag * ua_mag
    alignment = np.full(len(frame), np.nan)
    usable = both & (denominator > 1e-12)
    alignment[usable] = (
        np.sum(uh[usable] * ua[usable], axis=1) / denominator[usable]
    )
    opposing = np.full(len(frame), np.nan)
    opposing[usable] = np.maximum(
        0.0, -np.sum(uh[usable] * ua[usable], axis=1) / uh_mag[usable]
    )
    ratio = np.full(len(frame), np.nan)
    ratio[usable] = ua_mag[usable] / uh_mag[usable]
    effort = np.nan
    if np.isfinite(ua_mag).sum() >= 2:
        effort = float(
            np.trapz(np.nan_to_num(ua_mag) ** 2, frame["elapsed_s"])
        )

    max_control = _numeric_constant(frame, "max_control_amplitude")
    saturation = np.nan
    finite_ua = np.isfinite(ua).all(axis=1)
    if np.isfinite(max_control) and max_control > 0 and finite_ua.any():
        if attempt.metadata["controller_family"] == "state_feedback":
            # State feedback clamps the Cartesian force norm. A diagonal force
            # can therefore be saturated while both components remain below
            # the scalar limit.
            force_norm = np.linalg.norm(ua[finite_ua], axis=1)
            saturated = force_norm >= 0.99 * max_control
        else:
            # MPC uses per-component control constraints.
            saturated = np.any(
                np.abs(ua[finite_ua]) >= 0.99 * max_control,
                axis=1,
            )
        saturation = float(np.mean(saturated))

    kh = _arrays(frame["K_h"], 8)
    result = {
        "participant_id": str(
            attempt.metadata.get("participant_id", "unknown")
        ),
        "session_id": str(attempt.metadata["session_id"]),
        "trial_id": int(attempt.metadata["trial_id"]),
        "attempt_id": int(attempt.metadata["attempt_id"]),
        "controller_family": attempt.metadata["controller_family"],
        "controller_mode": attempt.metadata["study_controller_mode"],
        "phase": attempt.metadata["study_phase"],
        "segment": _segment_label(frame),
        "outcome": attempt.outcome,
        "reason": attempt.reason,
        "sample_count": len(frame),
        "duration_s": float(elapsed[-1] - elapsed[0]),
        "endpoint_error": float(np.linalg.norm(position[-1] - end)),
        "cross_track_rmse": float(np.sqrt(np.mean(error**2))),
        "cross_track_p95": float(np.percentile(error, 95)),
        "cross_track_max": float(np.max(error)),
        "overshoot_distance": float(
            max(0.0, np.max(progress) - 1.0) * path_length
        ),
        "backtracking_distance": float(
            np.maximum(0.0, -np.diff(progress)).sum() * path_length
        ),
        "path_length_ratio": float(steps.sum() / path_length),
        "speed_mean": _safe_mean(frame["cursor_speed"]),
        "speed_peak": float(np.nanmax(frame["cursor_speed"]))
        if frame["cursor_speed"].notna().any()
        else np.nan,
        "assistant_effort": effort,
        "assistant_saturation_fraction": saturation,
        "estimated_input_alignment_mean": _safe_mean(alignment),
        "estimated_input_opposing_mean": _safe_mean(opposing),
        "assistant_to_estimated_input_ratio_mean": _safe_mean(ratio),
    }
    for name, index in {"kp_x": 0, "kd_x": 1, "kp_y": 6, "kd_y": 7}.items():
        result[f"estimator_{name}_mean"] = _safe_mean(kh[:, index])
        result[f"estimator_{name}_std"] = _safe_std(kh[:, index])
        finite = kh[:, index][np.isfinite(kh[:, index])]
        result[f"estimator_{name}_final"] = (
            float(finite[-1]) if finite.size else np.nan
        )
    result.update(
        _controller_parameter_metrics(
            frame, attempt.metadata["controller_family"]
        )
    )
    return result, frame


def _numeric_constant(frame, column):
    if column not in frame.columns:
        return np.nan
    values = pd.to_numeric(frame[column], errors="coerce").dropna().unique()
    return float(values[0]) if len(values) == 1 else np.nan


def _segment_label(frame):
    return (
        f"({frame.start_x.iloc[0]:.6g},{frame.start_y.iloc[0]:.6g})->"
        f"({frame.end_x.iloc[0]:.6g},{frame.end_y.iloc[0]:.6g})"
    )


def _controller_parameter_metrics(frame, family):
    parsed = []
    for value in frame["K_a"].dropna():
        try:
            parsed.append(json.loads(value))
        except (TypeError, json.JSONDecodeError) as exc:
            raise SchemaError(f"invalid K_a JSON: {value!r}") from exc
    if not parsed:
        return {}
    if family == "mpc":
        return {
            f"mpc_{name}_mean": _safe_mean(
                [entry.get(f"weight_{name}", np.nan) for entry in parsed]
            )
            for name in ("comfort", "trajectory", "goal")
        }
    kp = [entry.get("K_p", [np.nan, np.nan]) for entry in parsed]
    kd = [entry.get("K_d", [np.nan, np.nan]) for entry in parsed]
    result = {
        "state_feedback_kp_x_mean": _safe_mean([value[0] for value in kp]),
        "state_feedback_kp_y_mean": _safe_mean([value[1] for value in kp]),
        "state_feedback_kd_x_mean": _safe_mean([value[0] for value in kd]),
        "state_feedback_kd_y_mean": _safe_mean([value[1] for value in kd]),
    }
    for name in (
        "adaptation_scale",
        "along_stiffness_n_per_m",
        "along_damping_ns_per_m",
        "fixture_stiffness_n_per_m",
        "fixture_damping_ns_per_m",
        "max_force_n",
        "docking_start_percent",
        "docking_stiffness_scale",
        "docking_max_cross_track_m",
    ):
        result[f"state_feedback_{name}_mean"] = _safe_mean(
            [entry.get(name, np.nan) for entry in parsed]
        )
    result["state_feedback_docking_enabled_fraction"] = _safe_mean(
        [
            float(entry["docking_enabled"])
            for entry in parsed
            if "docking_enabled" in entry
        ]
    )
    return result


def summarize_conditions(trial_metrics):
    """Return descriptive statistics without inferential claims."""
    groups = ["controller_family", "controller_mode", "phase", "segment"]
    numeric = [
        column
        for column in trial_metrics.select_dtypes(include=[np.number]).columns
        if column not in {"trial_id", "attempt_id"}
    ]
    rows = []
    for key, group in trial_metrics.groupby(groups, dropna=False):
        row = dict(zip(groups, key, strict=True))
        row["count"] = len(group)
        for column in numeric:
            values = group[column].dropna()
            row[f"{column}_mean"] = values.mean() if len(values) else np.nan
            row[f"{column}_std"] = (
                values.std(ddof=1) if len(values) > 1 else np.nan
            )
            row[f"{column}_median"] = (
                values.median() if len(values) else np.nan
            )
            row[f"{column}_iqr"] = (
                values.quantile(0.75) - values.quantile(0.25)
                if len(values)
                else np.nan
            )
        rows.append(row)
    return pd.DataFrame(rows)
