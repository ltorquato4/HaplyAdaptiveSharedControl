"""Deterministic ground-truth and closed-loop engineering benchmarks."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import yaml
from ament_index_python.packages import get_package_share_directory
from control_node.mpc_controller.adaptive_mpc_controller import (
    AdaptiveMpcController,
)
from control_node.mpc_controller.mpc_controller import MpcController
from control_node.state_feedback_controller.virtual_fixture_controller import (
    StateFeedbackForceConfig,
    VirtualFixtureStateFeedbackController,
)
from estimator_node.estimator.rls_estimator import RLSEstimator
from matplotlib.backends.backend_pdf import PdfPages


def _load_yaml(path):
    with Path(path).open(encoding="utf-8") as stream:
        values = yaml.safe_load(stream)
    if not isinstance(values, dict):
        raise ValueError(f"Configuration must be a YAML mapping: {path}")
    return values


def _installed_config(package_name, filename):
    return Path(get_package_share_directory(package_name)) / "config" / filename


def load_configuration(benchmark_path=None):
    """Load benchmark settings and authoritative controller profiles."""
    benchmark_file = (
        Path(benchmark_path)
        if benchmark_path
        else _installed_config("study_analysis", "benchmark.yaml")
    )
    benchmark = _load_yaml(benchmark_file)
    controller_profiles = {}
    for family, filename in (
        ("state_feedback", "state_feedback.yaml"),
        ("mpc", "mpc.yaml"),
    ):
        profile = _load_yaml(_installed_config("control_node", filename))
        try:
            controller_profiles[family] = profile["control_node"]["ros__parameters"]
        except (KeyError, TypeError) as exc:
            raise ValueError(
                f"{filename} must define control_node.ros__parameters"
            ) from exc
    return benchmark, controller_profiles


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


def _controller_factories(
    start,
    goal,
    state_feedback_parameters,
    mpc_parameters,
):
    state_feedback_config = StateFeedbackForceConfig(
        along_stiffness_n_per_m=state_feedback_parameters[
            "along_stiffness_n_per_m"
        ],
        along_damping_ns_per_m=state_feedback_parameters[
            "along_damping_ns_per_m"
        ],
        fixture_stiffness_n_per_m=state_feedback_parameters[
            "fixture_stiffness_n_per_m"
        ],
        fixture_damping_ns_per_m=state_feedback_parameters[
            "fixture_damping_ns_per_m"
        ],
        max_force_n=state_feedback_parameters["max_force_n"],
        velocity_filter_alpha=state_feedback_parameters[
            "velocity_filter_alpha"
        ],
        docking_enabled=state_feedback_parameters["docking_enabled"],
        docking_start_percent=state_feedback_parameters[
            "docking_start_percent"
        ],
        docking_stiffness_scale=state_feedback_parameters[
            "docking_stiffness_scale"
        ],
        docking_max_cross_track_m=state_feedback_parameters[
            "docking_max_cross_track_m"
        ],
        adaptation_normalization=state_feedback_parameters[
            "adaptation_normalization"
        ],
        adaptation_strength=state_feedback_parameters["adaptation_strength"],
    )
    mpc_common = {
        "prediction_horizon": int(mpc_parameters["prediction_horizon"]),
        "max_control": (
            mpc_parameters["max_control_amplitude"],
            mpc_parameters["max_control_amplitude"],
        ),
        "max_velocity": (
            mpc_parameters["max_velocity_amplitude"],
            mpc_parameters["max_velocity_amplitude"],
        ),
        "x_bounds": (-mpc_parameters["x_bounds"], mpc_parameters["x_bounds"]),
        "y_bounds": (-mpc_parameters["y_bounds"], mpc_parameters["y_bounds"]),
    }
    return {
        "state_feedback_fixed": {
            "factory": lambda: VirtualFixtureStateFeedbackController(
                start,
                goal,
                config=state_feedback_config,
                adaptive=False,
            ),
            "family": "state_feedback",
            "force_conversion": 1.0,
        },
        "state_feedback_adaptive": {
            "factory": lambda: VirtualFixtureStateFeedbackController(
                start,
                goal,
                config=state_feedback_config,
                adaptive=True,
            ),
            "family": "state_feedback",
            "force_conversion": 1.0,
        },
        "mpc_fixed": {
            "factory": lambda: MpcController(
                start,
                goal,
                mpc_parameters["delta_time"],
                **mpc_common,
            ),
            "family": "mpc",
            "force_conversion": mpc_parameters[
                "acceleration_to_force_factor"
            ],
        },
        "mpc_adaptive": {
            "factory": lambda: AdaptiveMpcController(
                start,
                goal,
                mpc_parameters["delta_time"],
                **mpc_common,
                docking_enabled=mpc_parameters["docking_enabled"],
                docking_start_percent=mpc_parameters[
                    "docking_start_percent"
                ],
                docking_comfort_reduction=mpc_parameters[
                    "docking_comfort_reduction"
                ],
                docking_trajectory_weight_scale=mpc_parameters[
                    "docking_trajectory_weight_scale"
                ],
                docking_goal_weight_scale=mpc_parameters[
                    "docking_goal_weight_scale"
                ],
            ),
            "family": "mpc",
            "force_conversion": mpc_parameters[
                "acceleration_to_force_factor"
            ],
        },
    }


def controller_benchmark(
    benchmark_config,
    controller_profiles,
):
    """Exercise all controller variants in the same deterministic plant."""
    seed = benchmark_config["seed"]
    dt = benchmark_config["simulation_dt"]
    duration = benchmark_config["simulation_duration_s"]
    target_radius = benchmark_config["target_radius"]
    start = np.asarray(benchmark_config["start_point"], dtype=float)
    goal = np.asarray(benchmark_config["goal_point"], dtype=float)
    mass = benchmark_config["plant_mass"]
    damping = benchmark_config["plant_damping"]
    disturbance_amplitude = benchmark_config["disturbance_amplitude"]
    disturbance_frequency = benchmark_config["disturbance_frequency"]
    disturbance_phase = np.asarray(
        benchmark_config["disturbance_phase"],
        dtype=float,
    )
    disturbance_noise_std = benchmark_config["disturbance_noise_std"]
    interaction_coefficients = benchmark_config["adaptation_coefficients"]
    state_feedback_parameters = controller_profiles["state_feedback"]
    mpc_parameters = controller_profiles["mpc"]
    state_feedback_maximum = state_feedback_parameters["max_force_n"]
    mpc_max_control = mpc_parameters["max_control_amplitude"]
    mpc_force_conversion = mpc_parameters["acceleration_to_force_factor"]

    rng = np.random.default_rng(seed)
    step_count = int(duration / dt)
    disturbance_noise = rng.normal(
        scale=disturbance_noise_std,
        size=(step_count, 2),
    )
    rows = []
    traces = {}
    controller_specs = _controller_factories(
        start,
        goal,
        state_feedback_parameters,
        mpc_parameters,
    )
    for name, spec in controller_specs.items():
        controller = spec["factory"]()
        family = spec["family"]
        force_conversion = spec["force_conversion"]
        position = start.copy()
        velocity = np.zeros(2)
        trace = []
        finite = True
        bounded = True
        raw_bounded = True
        physical_bounded = True
        reached_time = np.nan
        for step in range(step_count):
            if "adaptive" in name:
                if family == "state_feedback":
                    controller.update_effective_interaction_model(
                        interaction_coefficients
                    )
                else:
                    controller.adapt(
                        [
                            interaction_coefficients[:2],
                            interaction_coefficients[2:],
                        ]
                    )
            if family == "state_feedback":
                command = controller.compute_force(
                    position,
                    timestamp_s=(step + 1) * dt,
                )
                raw_bounded = raw_bounded and bool(
                    np.linalg.norm(command) <= state_feedback_maximum + 1e-9
                )
            else:
                command = np.asarray(
                    controller.compute_control(
                        position,
                        timestamp_s=(step + 1) * dt,
                    )
                )
                raw_bounded = raw_bounded and bool(
                    np.all(np.abs(command) <= mpc_max_control + 1e-9)
                )
            applied_force = command * force_conversion
            finite = finite and bool(np.isfinite(command).all())
            finite = finite and bool(np.isfinite(applied_force).all())
            if family == "state_feedback":
                physical_bounded = physical_bounded and bool(
                    np.linalg.norm(applied_force)
                    <= state_feedback_maximum + 1e-9
                )
            else:
                physical_bounded = physical_bounded and bool(
                    np.all(
                        np.abs(applied_force)
                        <= mpc_max_control * mpc_force_conversion + 1e-9
                    )
                )
            bounded = raw_bounded and physical_bounded
            disturbance = disturbance_amplitude * np.sin(
                disturbance_frequency * step + disturbance_phase
            )
            disturbance += disturbance_noise[step]
            acceleration = (
                applied_force + disturbance - damping * velocity
            ) / mass
            velocity = velocity + acceleration * dt
            position = position + velocity * dt
            distance = float(np.linalg.norm(position - goal))
            trace.append((step * dt, *position, *applied_force, distance))
            if distance <= target_radius:
                reached_time = (step + 1) * dt
                break
        if family == "mpc":
            controller.destroy()
        traces[name] = np.asarray(trace)
        rows.append(
            {
                "benchmark": "controller",
                "case": name,
                "controller_family": family,
                "command_units": (
                    "mpc_control_units" if family == "mpc" else "N"
                ),
                "force_conversion": force_conversion,
                "reached": bool(np.isfinite(reached_time)),
                "reached_time_s": reached_time,
                "final_error": float(trace[-1][-1]),
                "finite_outputs": finite,
                "bounded_outputs": bounded,
                "raw_command_bounded": raw_bounded,
                "physical_force_bounded": physical_bounded,
            }
        )
    return rows, traces


def adaptation_direction_benchmark(benchmark_config, controller_profiles):
    """Check both adaptive controllers against their documented directions."""
    start = np.asarray(benchmark_config["start_point"], dtype=float)
    goal = np.asarray(benchmark_config["goal_point"], dtype=float)
    coefficients = benchmark_config["adaptation_direction_coefficients"]
    factories = _controller_factories(
        start,
        goal,
        controller_profiles["state_feedback"],
        controller_profiles["mpc"],
    )
    feedback = factories["state_feedback_adaptive"]["factory"]()
    feedback.update_effective_interaction_model(coefficients)
    mpc = factories["mpc_adaptive"]["factory"]()
    base_comfort = mpc.cost_function.weight_comfort
    base_trajectory = mpc.cost_function.weight_trajectory
    mpc.adapt([coefficients[:2], coefficients[2:]])
    mpc_direction = (
        mpc.cost_function.weight_comfort > base_comfort
        and mpc.cost_function.weight_trajectory < base_trajectory
    )
    mpc.destroy()
    return [
        {
            "benchmark": "adaptation",
            "case": "effective_interaction_parameter_direction",
            "state_feedback_direction": feedback.adaptation_scale < 1.0,
            "mpc_direction": mpc_direction,
        }
    ]


def _write_report(path, results, traces, benchmark_config):
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
        start = benchmark_config["start_point"]
        goal = benchmark_config["goal_point"]
        axis.scatter(start[0], start[1], label="start")
        axis.scatter(goal[0], goal[1], marker="x", label="goal")
        axis.set_aspect("equal", adjustable="box")
        axis.set_xlabel("x [m]")
        axis.set_ylabel("y [m]")
        axis.set_title("Closed-loop controller benchmark")
        axis.legend()
        pdf.savefig(figure)
        plt.close(figure)


def run(output_directory, seed=None, config_path=None):
    config_values, controller_profiles = load_configuration(config_path)
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
        config_values,
        controller_profiles,
    )
    rows = (
        estimator_rows
        + controller_rows
        + adaptation_direction_benchmark(
            config_values,
            controller_profiles,
        )
    )
    results = pd.DataFrame(rows)
    results.to_csv(output / "benchmark_results.csv", index=False)
    _write_report(
        output / "benchmark_report.pdf",
        results,
        traces,
        config_values,
    )

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
    adaptation_pass = (
        adaptation["state_feedback_direction"] and adaptation["mpc_direction"]
    )
    return bool(estimator_pass and controller_pass and adaptation_pass), results


def build_parser():
    parser = argparse.ArgumentParser(description="Run deterministic study benchmarks")
    parser.add_argument("--output", required=True)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--config", help="Alternative benchmark YAML")
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    passed, _results = run(
        args.output,
        seed=args.seed,
        config_path=args.config,
    )
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
