"""Deterministic ground-truth and closed-loop engineering benchmarks."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import yaml
from control_node.state_feedback_controller.virtual_fixture_controller import (
    StateFeedbackForceConfig,
    VirtualFixtureStateFeedbackController,
)
from estimator_node.estimator.rls_estimator import RLSEstimator
from matplotlib.backends.backend_pdf import PdfPages

DEFAULT_CONFIG = {
    "seed": 20260721,
    "estimator_samples": 2500,
    "estimator_noise_std": 0.05,
    "estimator_noiseless_relative_error_max": 0.01,
    "estimator_noisy_relative_error_max": 0.15,
    "simulation_dt": 0.01,
    "simulation_duration_s": 20.0,
    "target_radius": 0.01,
    "max_force": 0.5,
}


def estimator_benchmark(seed, samples, noise_std):
    """Identify known coefficients under persistently exciting inputs."""
    true_x = np.array([2.0, -0.4])
    true_y = np.array([1.5, -0.25])
    rows = []
    for label, noise in (("noiseless", 0.0), ("seeded_noise", noise_std)):
        rng = np.random.default_rng(seed)
        estimator = RLSEstimator()
        residuals = []
        for _ in range(samples):
            phi_x = rng.normal(size=2)
            phi_y = rng.normal(size=2)
            ax = float(phi_x @ true_x + rng.normal(scale=noise))
            ay = float(phi_y @ true_y + rng.normal(scale=noise))
            estimator.update(phi_x[0], phi_x[1], phi_y[0], phi_y[1], ax, ay)
            estimate = estimator.get_matrix()
            residuals.extend(
                [ax - estimate[0, [0, 1]] @ phi_x, ay - estimate[1, [2, 3]] @ phi_y]
            )
        matrix = estimator.get_matrix()
        estimated = np.array([matrix[0, 0], matrix[0, 1], matrix[1, 2], matrix[1, 3]])
        truth = np.concatenate([true_x, true_y])
        rows.append(
            {
                "benchmark": "estimator",
                "case": label,
                "relative_coefficient_error": float(
                    np.linalg.norm(estimated - truth) / np.linalg.norm(truth)
                ),
                "prediction_rmse": float(np.sqrt(np.mean(np.square(residuals[-500:])))),
                "final_values": estimated.tolist(),
            }
        )
    return rows


def _controller_factories(maximum):
    config = StateFeedbackForceConfig(max_force_n=maximum)
    return {
        "state_feedback_fixed": lambda: VirtualFixtureStateFeedbackController(
            (-0.08, -0.08), (0.08, 0.08), config=config, adaptive=False
        ),
        "state_feedback_adaptive": lambda: VirtualFixtureStateFeedbackController(
            (-0.08, -0.08), (0.08, 0.08), config=config, adaptive=True
        ),
    }


def controller_benchmark(seed, dt, duration, target_radius, maximum):
    """Exercise all controller variants in the same deterministic plant."""
    rng = np.random.default_rng(seed)
    goal = np.array([0.08, 0.08])
    step_count = int(duration / dt)
    disturbance_noise = rng.normal(scale=0.0005, size=(step_count, 2))
    rows = []
    traces = {}
    for name, factory in _controller_factories(maximum).items():
        controller = factory()
        position = np.array([-0.08, -0.08], dtype=float)
        velocity = np.zeros(2)
        trace = []
        finite = True
        bounded = True
        reached_time = np.nan
        for step in range(step_count):
            if "adaptive" in name:
                controller.update_effective_interaction_model([2.0, 0.2, 2.0, 0.2])
            command = controller.compute_force(position, timestamp_s=(step + 1) * dt)
            finite = finite and bool(np.isfinite(command).all())
            bounded = bounded and bool(np.linalg.norm(command) <= maximum + 1e-9)
            disturbance = 0.02 * np.sin(0.13 * step + np.array([0.0, 1.0]))
            disturbance += disturbance_noise[step]
            acceleration = (command + disturbance - 0.5 * velocity) / 0.5
            velocity = velocity + acceleration * dt
            position = position + velocity * dt
            distance = float(np.linalg.norm(position - goal))
            trace.append((step * dt, *position, *command, distance))
            if distance <= target_radius:
                reached_time = (step + 1) * dt
                break
        traces[name] = np.asarray(trace)
        rows.append(
            {
                "benchmark": "controller",
                "case": name,
                "reached": bool(np.isfinite(reached_time)),
                "reached_time_s": reached_time,
                "final_error": float(trace[-1][-1]),
                "finite_outputs": finite,
                "bounded_outputs": bounded,
            }
        )
    return rows, traces


def adaptation_direction_benchmark(_dt=0.01):
    """Check the state-feedback response to position-dominant effective gains."""
    feedback = VirtualFixtureStateFeedbackController(
        (-0.08, -0.08), (0.08, 0.08), adaptive=True
    )
    feedback.update_effective_interaction_model([10.0, 0.1, 10.0, 0.1])
    return [
        {
            "benchmark": "adaptation",
            "case": "effective_interaction_parameter_direction",
            "state_feedback_direction": feedback.adaptation_scale < 1.0,
        }
    ]


def _write_report(path, results, traces):
    with PdfPages(path) as pdf:
        figure, axis = plt.subplots(figsize=(11.69, 8.27))
        axis.axis("off")
        axis.table(
            cellText=results.fillna("").astype(str).values,
            colLabels=results.columns,
            loc="center",
            cellLoc="center",
        )
        axis.set_title("Deterministic validation results")
        pdf.savefig(figure)
        plt.close(figure)

        figure, axis = plt.subplots(figsize=(8.27, 8.27))
        for name, trace in traces.items():
            axis.plot(trace[:, 1], trace[:, 2], label=name)
        axis.scatter(-0.08, -0.08, label="start")
        axis.scatter(0.08, 0.08, marker="x", label="goal")
        axis.set_aspect("equal", adjustable="box")
        axis.set_xlabel("x [m]")
        axis.set_ylabel("y [m]")
        axis.set_title("Closed-loop state-feedback force benchmark")
        axis.legend()
        pdf.savefig(figure)
        plt.close(figure)


def run(output_directory, seed=None, config=None):
    config_values = dict(DEFAULT_CONFIG)
    if config:
        config_values.update(config)
    if seed is not None:
        config_values["seed"] = int(seed)
    output = Path(output_directory)
    output.mkdir(parents=True, exist_ok=True)

    estimator_rows = estimator_benchmark(
        config_values["seed"],
        config_values["estimator_samples"],
        config_values["estimator_noise_std"],
    )
    controller_rows, traces = controller_benchmark(
        config_values["seed"],
        config_values["simulation_dt"],
        config_values["simulation_duration_s"],
        config_values["target_radius"],
        config_values["max_force"],
    )
    rows = (
        estimator_rows
        + controller_rows
        + adaptation_direction_benchmark(config_values["simulation_dt"])
    )
    results = pd.DataFrame(rows)
    results.to_csv(output / "benchmark_results.csv", index=False)
    _write_report(output / "benchmark_report.pdf", results, traces)

    estimator_pass = (
        estimator_rows[0]["relative_coefficient_error"]
        < config_values["estimator_noiseless_relative_error_max"]
        and estimator_rows[1]["relative_coefficient_error"]
        < config_values["estimator_noisy_relative_error_max"]
    )
    controller_pass = all(
        row["reached"] and row["finite_outputs"] and row["bounded_outputs"]
        for row in controller_rows
    )
    adaptation = rows[-1]
    adaptation_pass = adaptation["state_feedback_direction"]
    return bool(estimator_pass and controller_pass and adaptation_pass), results


def build_parser():
    parser = argparse.ArgumentParser(description="Run deterministic study benchmarks")
    parser.add_argument("--output", required=True)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--config", help="Optional benchmark YAML")
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    config = None
    if args.config:
        config = yaml.safe_load(Path(args.config).read_text(encoding="utf-8")) or {}
    passed, _results = run(args.output, seed=args.seed, config=config)
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
