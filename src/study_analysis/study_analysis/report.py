"""PDF reporting for study analysis."""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.backends.backend_pdf import PdfPages

from .loader import parse_json_array


def _title_page(pdf, attempts, quality):
    figure = plt.figure(figsize=(8.27, 11.69))
    figure.text(0.08, 0.93, "Haply study analysis", fontsize=20, weight="bold")
    figure.text(0.08, 0.88, f"Recorded attempts: {len(attempts)}", fontsize=12)
    figure.text(0.08, 0.85, f"Data-quality findings: {len(quality)}", fontsize=12)
    if quality:
        lines = [
            f"{item.get('file', '')}: {item['issue']} {item.get('value', '')}"
            for item in quality[:35]
        ]
        figure.text(0.08, 0.80, "\n".join(lines), fontsize=8, va="top")
    pdf.savefig(figure)
    plt.close(figure)


def _trajectory_page(pdf, enriched, attempt):
    figure, axes = plt.subplots(2, 2, figsize=(11.69, 8.27))
    position = enriched[["cursor_x", "cursor_y"]]
    axes[0, 0].plot(position.cursor_x, position.cursor_y, linewidth=1)
    axes[0, 0].scatter(
        enriched.start_x.iloc[0], enriched.start_y.iloc[0], label="start"
    )
    axes[0, 0].scatter(
        enriched.end_x.iloc[0], enriched.end_y.iloc[0], marker="x", label="end"
    )
    axes[0, 0].set_aspect("equal", adjustable="box")
    axes[0, 0].set_title("Task-frame trajectory")
    axes[0, 0].set_xlabel("task x [m]")
    axes[0, 0].set_ylabel("task y [m]")
    axes[0, 0].legend()

    axes[0, 1].plot(enriched.elapsed_s, enriched.cross_track_error)
    axes[0, 1].set_title("Cross-track error in temporal order")
    axes[0, 1].set_xlabel("elapsed time [s]")
    axes[0, 1].set_ylabel("absolute error [m]")

    axes[1, 0].plot(enriched.elapsed_s, enriched.progress, label="progress")
    axes[1, 0].axhline(0.0, color="black", linewidth=0.5)
    axes[1, 0].axhline(1.0, color="black", linewidth=0.5)
    axes[1, 0].set_title("Along-path progress")
    axes[1, 0].set_xlabel("elapsed time [s]")

    axes[1, 1].plot(enriched.elapsed_s, enriched.cursor_speed, label="task cursor")
    if {"haply_vel_x", "haply_vel_z"}.issubset(enriched.columns):
        device_speed = np.hypot(enriched.haply_vel_x, enriched.haply_vel_z)
        axes[1, 1].plot(enriched.elapsed_s, device_speed, alpha=0.7, label="Haply x/z")
    axes[1, 1].set_title("Velocity diagnostics")
    axes[1, 1].set_xlabel("elapsed time [s]")
    axes[1, 1].set_ylabel("speed [m/s]")
    axes[1, 1].legend()

    figure.suptitle(
        f"Trial {attempt.key[1]} attempt {attempt.key[2]} | "
        f"{attempt.metadata['controller_family']} / "
        f"{attempt.metadata['study_controller_mode']} / "
        f"{attempt.metadata['study_phase']}"
    )
    figure.tight_layout()
    pdf.savefig(figure)
    plt.close(figure)


def _estimator_control_page(pdf, enriched, attempt):
    figure, axes = plt.subplots(3, 1, figsize=(11.69, 8.27), sharex=True)
    kh = np.vstack([parse_json_array(value, 8)[:8] for value in enriched.K_h])
    for label, index in {"kp_x": 0, "kd_x": 1, "kp_y": 6, "kd_y": 7}.items():
        axes[0].plot(enriched.elapsed_s, kh[:, index], label=label)
    axes[0].set_ylabel("identified coefficient")
    axes[0].set_title("Estimator coefficients")
    axes[0].legend(ncol=4)

    uh = np.vstack([parse_json_array(value, 2)[:2] for value in enriched.u_h])
    ua = np.vstack([parse_json_array(value, 2)[:2] for value in enriched.u_a])
    axes[1].plot(
        enriched.elapsed_s, np.linalg.norm(uh, axis=1), label="estimated interaction"
    )
    axes[1].plot(enriched.elapsed_s, np.linalg.norm(ua, axis=1), label="assistant")
    axes[1].set_ylabel("input magnitude")
    axes[1].set_title("Input magnitudes")
    axes[1].legend()

    denom = np.linalg.norm(uh, axis=1) * np.linalg.norm(ua, axis=1)
    cosine = np.divide(
        np.sum(uh * ua, axis=1),
        denom,
        out=np.full(len(denom), np.nan),
        where=denom > 1e-12,
    )
    axes[2].plot(enriched.elapsed_s, cosine)
    axes[2].axhline(0.0, color="black", linewidth=0.5)
    axes[2].set_ylim(-1.05, 1.05)
    axes[2].set_ylabel("cosine alignment")
    axes[2].set_xlabel("elapsed time [s]")
    axes[2].set_title("Estimated interaction / assistant alignment")
    figure.suptitle(
        f"Estimator and control | trial {attempt.key[1]} attempt {attempt.key[2]}"
    )
    figure.tight_layout()
    pdf.savefig(figure)
    plt.close(figure)


def _condition_page(pdf, trial_metrics):
    figure, axes = plt.subplots(2, 2, figsize=(11.69, 8.27))
    labels = (
        trial_metrics.controller_family.astype(str)
        + "/"
        + trial_metrics.controller_mode.astype(str)
        + "/"
        + trial_metrics.phase.astype(str)
    )
    for axis, column, title in (
        (axes[0, 0], "duration_s", "Completion duration"),
        (axes[0, 1], "cross_track_rmse", "Cross-track RMSE"),
        (axes[1, 0], "assistant_effort", "Assistant effort"),
        (axes[1, 1], "estimated_input_alignment_mean", "Mean input alignment"),
    ):
        positions = np.arange(len(trial_metrics))
        axis.scatter(positions, trial_metrics[column])
        axis.set_title(title)
        axis.set_xticks(positions, labels, rotation=90, fontsize=6)
        axis.grid(axis="y", alpha=0.3)
    figure.suptitle("Descriptive condition overview (one point per attempt)")
    figure.tight_layout()
    pdf.savefig(figure)
    plt.close(figure)


def _outcome_page(pdf, trial_metrics):
    figure, axes = plt.subplots(1, 2, figsize=(11.69, 8.27))
    outcomes = trial_metrics["outcome"].fillna("UNKNOWN").value_counts()
    axes[0].bar(outcomes.index.astype(str), outcomes.values)
    axes[0].set_title("Attempt outcomes")
    axes[0].set_ylabel("attempt count")
    axes[0].tick_params(axis="x", rotation=30)

    durations = [
        group["duration_s"].dropna().to_numpy()
        for _, group in trial_metrics.groupby("outcome", dropna=False)
    ]
    labels = [str(key) for key, _ in trial_metrics.groupby("outcome", dropna=False)]
    nonempty = [
        (label, values)
        for label, values in zip(labels, durations, strict=True)
        if len(values)
    ]
    if nonempty:
        axes[1].boxplot(
            [values for _, values in nonempty],
            labels=[label for label, _ in nonempty],
        )
    axes[1].set_title("Duration by outcome")
    axes[1].set_ylabel("duration [s]")
    axes[1].tick_params(axis="x", rotation=30)
    figure.tight_layout()
    pdf.savefig(figure)
    plt.close(figure)


def _progress_profile_page(pdf, enriched_attempts, attempts):
    """Aggregate after binning each attempt independently by path progress."""
    bin_edges = np.linspace(0.0, 1.0, 21)
    centers = (bin_edges[:-1] + bin_edges[1:]) / 2.0
    profiles = []
    for frame, attempt in zip(enriched_attempts, attempts, strict=True):
        values = frame[["progress", "cross_track_error"]].dropna().copy()
        values = values[(values.progress >= 0.0) & (values.progress <= 1.0)]
        values["progress_bin"] = pd.cut(
            values.progress,
            bin_edges,
            labels=False,
            include_lowest=True,
        )
        profile = values.groupby("progress_bin").cross_track_error.mean()
        for bin_index, error in profile.items():
            profiles.append(
                {
                    "bin": int(bin_index),
                    "error": float(error),
                    "condition": (
                        f"{attempt.metadata['controller_family']}/"
                        f"{attempt.metadata['study_controller_mode']}"
                    ),
                }
            )

    figure, axis = plt.subplots(figsize=(11.69, 8.27))
    profiles = pd.DataFrame(profiles)
    if not profiles.empty:
        for condition, group in profiles.groupby("condition"):
            summary = group.groupby("bin").error.agg(["mean", "std"])
            x = centers[summary.index.to_numpy(dtype=int)]
            axis.plot(x, summary["mean"], label=condition)
            deviation = summary["std"].fillna(0.0).to_numpy()
            axis.fill_between(
                x,
                summary["mean"].to_numpy() - deviation,
                summary["mean"].to_numpy() + deviation,
                alpha=0.15,
            )
    axis.set_title("Cross-track error by per-attempt progress bins")
    axis.set_xlabel("normalized path progress")
    axis.set_ylabel("cross-track error [m]")
    axis.legend()
    figure.tight_layout()
    pdf.savefig(figure)
    plt.close(figure)


def _controller_parameter_page(pdf, trial_metrics):
    columns = [
        column
        for column in trial_metrics.columns
        if column.startswith(("mpc_", "state_feedback_"))
    ]
    if not columns:
        return
    figure, axis = plt.subplots(figsize=(11.69, 8.27))
    positions = np.arange(len(trial_metrics))
    for column in columns:
        axis.plot(positions, trial_metrics[column], marker=".", label=column)
    axis.set_title("Controller-family parameter summaries")
    axis.set_xlabel("attempt row")
    axis.set_ylabel("parameter value")
    axis.legend(fontsize=7, ncol=2)
    axis.grid(axis="y", alpha=0.3)
    figure.tight_layout()
    pdf.savefig(figure)
    plt.close(figure)


def generate_report(path, attempts, enriched_attempts, trial_metrics, quality):
    """Generate a single multipage, publication-friendly PDF."""
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    with PdfPages(output) as pdf:
        _title_page(pdf, attempts, quality)
        _outcome_page(pdf, trial_metrics)
        _condition_page(pdf, trial_metrics)
        _progress_profile_page(pdf, enriched_attempts, attempts)
        _controller_parameter_page(pdf, trial_metrics)
        for attempt, enriched in zip(attempts, enriched_attempts, strict=True):
            _trajectory_page(pdf, enriched, attempt)
            try:
                _estimator_control_page(pdf, enriched, attempt)
            except (ValueError, json.JSONDecodeError):
                # The strict loader reports malformed populated values. Entirely
                # missing early samples are represented as quality gaps instead.
                continue
