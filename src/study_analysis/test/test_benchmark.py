from study_analysis.benchmark import adaptation_direction_benchmark, estimator_benchmark


def test_ground_truth_estimator_meets_accuracy_thresholds():
    rows = estimator_benchmark(seed=7, samples=2500, noise_std=0.05)
    assert rows[0]["relative_coefficient_error"] < 0.01
    assert rows[1]["relative_coefficient_error"] < 0.15


def test_adaptation_moves_parameters_in_documented_direction():
    result = adaptation_direction_benchmark()[0]
    assert result["state_feedback_direction"]
