from study_analysis.benchmark import (
    adaptation_direction_benchmark,
    controller_benchmark,
    estimator_benchmark,
    load_configuration,
)


def test_ground_truth_estimator_meets_accuracy_thresholds():
    benchmark, _profiles = load_configuration()
    rows = estimator_benchmark(
        seed=benchmark["seed"],
        samples=benchmark["estimator_samples"],
        noise_std=benchmark["estimator_noise_std"],
    )
    assert (
        rows[0]["relative_coefficient_error"]
        < benchmark["estimator_noiseless_relative_error_max"]
    )
    assert (
        rows[1]["relative_coefficient_error"]
        < benchmark["estimator_noisy_relative_error_max"]
    )


def test_adaptation_moves_parameters_in_documented_direction():
    benchmark, profiles = load_configuration()
    result = adaptation_direction_benchmark(benchmark, profiles)[0]
    assert result["state_feedback_direction"]
    assert result["mpc_direction"]


def test_all_controller_families_reach_target_with_bounded_outputs():
    benchmark, profiles = load_configuration()
    rows, _traces = controller_benchmark(benchmark, profiles)
    assert {row["case"] for row in rows} == {
        "state_feedback_fixed",
        "state_feedback_adaptive",
        "mpc_fixed",
        "mpc_adaptive",
    }
    assert all(row["reached"] for row in rows)
    assert all(row["finite_outputs"] for row in rows)
    assert all(row["raw_command_bounded"] for row in rows)
    assert all(row["physical_force_bounded"] for row in rows)
