"""Tests for Scenario Generator pure logic."""

import pytest
from study_orchestration.scenario_logic import (
    StudyPoint,
    WorkspaceBounds,
    chained_segment,
    endpoint_reached,
    update_start_gate,
    validate_task_points,
)

DEFAULT_POINTS = [
    StudyPoint(-0.08, -0.20, 0.0),
    StudyPoint(0.08, -0.08, 0.0),
    StudyPoint(0.08, -0.20, 0.0),
]
DEFAULT_BOUNDS = WorkspaceBounds(
    x_min=-0.10,
    x_max=0.10,
    y_min=-0.25,
    y_max=-0.03,
)


def test_default_points_are_valid():
    validate_task_points(DEFAULT_POINTS, DEFAULT_BOUNDS, min_segment_length=0.10)


def test_chained_segments_reuse_previous_endpoint_as_next_start():
    first_start, first_end = chained_segment(DEFAULT_POINTS, 0)
    second_start, second_end = chained_segment(DEFAULT_POINTS, 1)
    third_start, third_end = chained_segment(DEFAULT_POINTS, 2)

    assert first_start == DEFAULT_POINTS[0]
    assert first_end == second_start
    assert second_end == third_start
    assert third_end == first_start


def test_rejects_points_outside_workspace():
    points = [
        StudyPoint(-0.11, -0.20, 0.0),
        StudyPoint(0.08, -0.08, 0.0),
        StudyPoint(0.08, -0.20, 0.0),
    ]

    with pytest.raises(ValueError, match="outside workspace"):
        validate_task_points(points, DEFAULT_BOUNDS, min_segment_length=0.10)


def test_rejects_short_segments():
    points = [
        StudyPoint(-0.08, -0.20, 0.0),
        StudyPoint(-0.07, -0.20, 0.0),
        StudyPoint(0.08, -0.20, 0.0),
    ]

    with pytest.raises(ValueError, match="shorter than"):
        validate_task_points(points, DEFAULT_BOUNDS, min_segment_length=0.10)


def test_endpoint_reached_uses_radius():
    endpoint = StudyPoint(0.08, -0.08, 0.0)

    assert endpoint_reached(StudyPoint(0.081, -0.081, 0.0), endpoint, 0.01)
    assert not endpoint_reached(StudyPoint(0.10, -0.10, 0.0), endpoint, 0.01)


def test_start_gate_latches_after_cursor_reaches_start():
    start = StudyPoint(-0.08, -0.20, 0.0)

    assert not update_start_gate(
        StudyPoint(0.08, -0.08, 0.0),
        start,
        start_gate_reached=False,
        start_reached_radius=0.01,
    )
    assert update_start_gate(
        StudyPoint(-0.081, -0.199, 0.0),
        start,
        start_gate_reached=False,
        start_reached_radius=0.01,
    )
    assert update_start_gate(
        StudyPoint(0.08, -0.08, 0.0),
        start,
        start_gate_reached=True,
        start_reached_radius=0.01,
    )
