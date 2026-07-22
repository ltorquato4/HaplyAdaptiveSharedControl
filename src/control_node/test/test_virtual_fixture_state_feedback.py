import json

import numpy as np
import pytest
from control_node.state_feedback_controller.virtual_fixture_controller import (
    StateFeedbackForceConfig,
    VirtualFixtureStateFeedbackController,
)


def config(**overrides):
    values = {
        "along_stiffness_n_per_m": 1.0,
        "along_damping_ns_per_m": 1.0,
        "fixture_stiffness_n_per_m": 2.0,
        "fixture_damping_ns_per_m": 1.0,
        "max_force_n": 100.0,
        "velocity_filter_alpha": 1.0,
    }
    values.update(overrides)
    return StateFeedbackForceConfig(**values)


def test_force_and_docking_defaults_are_independent():
    defaults = StateFeedbackForceConfig()

    assert defaults.max_force_n == pytest.approx(2.0)
    assert defaults.docking_enabled is False
    assert defaults.docking_start_percent == pytest.approx(85.0)
    assert defaults.docking_stiffness_scale == pytest.approx(2.0)
    assert defaults.docking_max_cross_track_m == pytest.approx(0.02)


def test_projection_separates_progress_from_sideways_error():
    controller = VirtualFixtureStateFeedbackController((0.0, 0.0), (1.0, 0.0), config())

    progress, cross_track = controller.path_coordinates((0.0, 1.0))

    assert progress == pytest.approx(0.0)
    assert cross_track == pytest.approx([0.0, 1.0])


def test_projection_respects_path_direction():
    controller = VirtualFixtureStateFeedbackController(
        (1.0, 1.0), (0.0, 0.0), config()
    )

    progress, cross_track = controller.path_coordinates((0.5, 0.6))

    assert progress == pytest.approx(0.45)
    assert cross_track == pytest.approx([-0.05, 0.05])


def test_virtual_fixture_pushes_toward_line_and_goal():
    controller = VirtualFixtureStateFeedbackController((0.0, 0.0), (1.0, 0.0), config())

    force = controller.compute_force((0.0, 0.1), timestamp_s=1.0)

    assert force == pytest.approx([1.0, -0.2])
    assert controller.last_progress == pytest.approx(0.0)
    assert controller.last_cross_track_m == pytest.approx(0.1)


def test_velocity_uses_cursor_timestamp_instead_of_fixed_dt():
    controller = VirtualFixtureStateFeedbackController((0.0, 0.0), (1.0, 0.0), config())
    controller.compute_force((0.0, 0.0), timestamp_s=1.0)

    controller.compute_force((0.001, 0.0), timestamp_s=1.01)

    assert controller.filtered_velocity == pytest.approx([0.1, 0.0])
    with pytest.raises(ValueError, match="strictly increasing"):
        controller.compute_force((0.002, 0.0), timestamp_s=1.01)


def test_trial_reset_prevents_velocity_from_crossing_stopped_gap():
    controller = VirtualFixtureStateFeedbackController(
        (0.0, 0.0), (1.0, 0.0), config()
    )
    controller.compute_force((0.0, 0.0), timestamp_s=1.0)
    controller.compute_force((0.001, 0.0), timestamp_s=1.01)
    assert controller.filtered_velocity[0] == pytest.approx(0.1)

    controller.reset_kinematics()
    controller.compute_force((0.5, 0.0), timestamp_s=10.0)

    assert controller.filtered_velocity == pytest.approx([0.0, 0.0])


def test_docking_is_disabled_by_default_and_configurable():
    baseline = VirtualFixtureStateFeedbackController(
        (0.0, 0.0), (1.0, 0.0), config(along_stiffness_n_per_m=10.0)
    )
    docked = VirtualFixtureStateFeedbackController(
        (0.0, 0.0),
        (1.0, 0.0),
        config(
            along_stiffness_n_per_m=10.0,
            docking_enabled=True,
            docking_start_percent=80.0,
            docking_stiffness_scale=2.0,
        ),
    )

    baseline_force = baseline.compute_force((0.9, 0.0), timestamp_s=1.0)
    docked_force = docked.compute_force((0.9, 0.0), timestamp_s=1.0)

    assert baseline_force[0] == pytest.approx(1.0)
    assert docked_force[0] > baseline_force[0]
    parameters = json.loads(baseline.parameter_json())
    assert parameters["docking_enabled"] is False


def test_docking_parameters_are_inert_when_docking_is_disabled():
    baseline = VirtualFixtureStateFeedbackController(
        (0.0, 0.0), (1.0, 0.0), config(along_stiffness_n_per_m=10.0)
    )
    disabled = VirtualFixtureStateFeedbackController(
        (0.0, 0.0),
        (1.0, 0.0),
        config(
            along_stiffness_n_per_m=10.0,
            docking_enabled=False,
            docking_start_percent=0.0,
            docking_stiffness_scale=20.0,
        ),
    )

    baseline_force = baseline.compute_force((0.9, 0.0), timestamp_s=1.0)
    disabled_force = disabled.compute_force((0.9, 0.0), timestamp_s=1.0)

    assert disabled_force == pytest.approx(baseline_force)


def test_docking_does_not_activate_away_from_virtual_fixture():
    controller = VirtualFixtureStateFeedbackController(
        (0.0, 0.0),
        (1.0, 0.0),
        config(
            along_stiffness_n_per_m=10.0,
            docking_enabled=True,
            docking_start_percent=80.0,
            docking_stiffness_scale=4.0,
            docking_max_cross_track_m=0.02,
        ),
    )

    force = controller.compute_force((0.9, 0.03), timestamp_s=1.0)

    assert force[0] == pytest.approx(1.0)


def test_adaptation_changes_along_force_but_not_fixture_gains():
    controller = VirtualFixtureStateFeedbackController(
        (0.0, 0.0), (1.0, 0.0), config(), adaptive=True
    )
    fixture_stiffness = controller.config.fixture_stiffness_n_per_m

    controller.update_effective_interaction_model([10.0, 0.1, 10.0, 0.1])

    assert controller.adaptation_scale < 1.0
    assert controller.config.fixture_stiffness_n_per_m == fixture_stiffness


def test_force_is_norm_bounded_and_finite():
    controller = VirtualFixtureStateFeedbackController(
        (0.0, 0.0), (1.0, 0.0), config(max_force_n=0.5)
    )

    force = controller.compute_force((-10.0, 10.0), timestamp_s=1.0)

    assert np.isfinite(force).all()
    assert np.linalg.norm(force) == pytest.approx(0.5)
