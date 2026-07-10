"""Tests for Experiment Mapper pure logic."""

import pytest
from study_orchestration.mapper_logic import (
    AnchoredDeltaMapper,
    MappingConfig,
    TaskPoint,
    map_identity,
)


def test_identity_mapping_preserves_raw_position():
    raw = TaskPoint(0.02, -0.03, 0.1)

    assert map_identity(raw) == raw


def test_anchored_delta_maps_raw_displacement_to_task_start():
    """With use_z_as_y=True (default), raw z drives task-y."""
    mapper = AnchoredDeltaMapper(MappingConfig())
    mapper.capture_anchor(
        raw_position=TaskPoint(1.0, 2.0, 0.3),
        task_start=TaskPoint(-0.08, -0.20, 0.0),
    )

    # x moves +0.02, z moves +0.03 → task_x = -0.06, task_y = -0.17
    mapped = mapper.map_position(TaskPoint(1.02, 2.0, 0.33))

    assert mapped.x == pytest.approx(-0.06)
    assert mapped.y == pytest.approx(-0.17)
    assert mapped.z == pytest.approx(0.0)


def test_anchored_delta_supports_scaling_and_axis_inversion():
    """With use_z_as_y=True (default), scale/inversion applies to the z delta."""
    mapper = AnchoredDeltaMapper(
        MappingConfig(scale_x=2.0, scale_y=0.5, invert_x=True, invert_y=False)
    )
    mapper.capture_anchor(
        raw_position=TaskPoint(1.0, 2.0, 0.0),
        task_start=TaskPoint(0.0, 0.0, 0.0),
    )

    # x moves +0.02, z moves +0.04 → task_x = -0.04 (inverted, ×2), task_y = +0.02 (×0.5)
    mapped = mapper.map_position(TaskPoint(1.02, 2.0, 0.04))

    assert mapped.x == pytest.approx(-0.04)
    assert mapped.y == pytest.approx(0.02)
    assert mapped.z == pytest.approx(0.0)


def test_anchored_delta_use_y_when_z_as_y_disabled():
    """With use_z_as_y=False, raw y drives task-y (legacy behaviour)."""
    mapper = AnchoredDeltaMapper(MappingConfig(use_z_as_y=False))
    mapper.capture_anchor(
        raw_position=TaskPoint(1.0, 2.0, 0.0),
        task_start=TaskPoint(-0.08, -0.20, 0.0),
    )

    # x moves +0.02, y moves +0.03 → task_x = -0.06, task_y = -0.17
    mapped = mapper.map_position(TaskPoint(1.02, 2.03, 0.0))

    assert mapped.x == pytest.approx(-0.06)
    assert mapped.y == pytest.approx(-0.17)
    assert mapped.z == pytest.approx(0.0)


def test_anchored_delta_returns_none_before_anchor():
    mapper = AnchoredDeltaMapper(MappingConfig())

    assert mapper.map_position(TaskPoint(1.0, 2.0, 0.0)) is None


# ---------------------------------------------------------------------------
# Clamping tests
# ---------------------------------------------------------------------------

def _clamped_mapper() -> AnchoredDeltaMapper:
    """Return a mapper anchored at origin with clamping enabled."""
    cfg = MappingConfig(
        clamp_raw=True,
        raw_x_min=-0.10,
        raw_x_max=0.10,
        raw_second_min=0.0,
        raw_second_max=0.15,
    )
    mapper = AnchoredDeltaMapper(cfg)
    mapper.capture_anchor(
        raw_position=TaskPoint(0.0, 0.0, 0.0),
        task_start=TaskPoint(0.0, 0.0, 0.0),
    )
    return mapper


def test_clamp_x_above_max():
    """Moving beyond raw_x_max clamps to raw_x_max."""
    mapper = _clamped_mapper()
    mapped = mapper.map_position(TaskPoint(0.20, 0.0, 0.05))  # x delta = 0.20 > 0.10
    assert mapped.x == pytest.approx(0.10)   # clamped to max


def test_clamp_z_below_min():
    """Moving below the anchor height (negative z delta) clamps to raw_second_min=0."""
    mapper = _clamped_mapper()
    mapped = mapper.map_position(TaskPoint(0.0, 0.0, -0.05))  # z delta negative
    assert mapped.y == pytest.approx(0.0)    # clamped to 0


def test_no_clamp_when_within_bounds():
    """Deltas inside bounds pass through unmodified."""
    mapper = _clamped_mapper()
    mapped = mapper.map_position(TaskPoint(0.05, 0.0, 0.10))  # x=0.05, z=0.10 — both inside
    assert mapped.x == pytest.approx(0.05)
    assert mapped.y == pytest.approx(0.10)
