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
    mapper = AnchoredDeltaMapper(MappingConfig())
    mapper.capture_anchor(
        raw_position=TaskPoint(1.0, 2.0, 0.0),
        task_start=TaskPoint(-0.08, -0.20, 0.0),
    )

    mapped = mapper.map_position(TaskPoint(1.02, 2.03, 0.0))

    assert mapped.x == pytest.approx(-0.06)
    assert mapped.y == pytest.approx(-0.17)
    assert mapped.z == pytest.approx(0.0)


def test_anchored_delta_supports_scaling_and_axis_inversion():
    mapper = AnchoredDeltaMapper(
        MappingConfig(scale_x=2.0, scale_y=0.5, invert_x=True, invert_y=False)
    )
    mapper.capture_anchor(
        raw_position=TaskPoint(1.0, 2.0, 0.0),
        task_start=TaskPoint(0.0, 0.0, 0.0),
    )

    mapped = mapper.map_position(TaskPoint(1.02, 2.04, 0.0))

    assert mapped.x == pytest.approx(-0.04)
    assert mapped.y == pytest.approx(0.02)
    assert mapped.z == pytest.approx(0.0)


def test_anchored_delta_returns_none_before_anchor():
    mapper = AnchoredDeltaMapper(MappingConfig())

    assert mapper.map_position(TaskPoint(1.0, 2.0, 0.0)) is None
