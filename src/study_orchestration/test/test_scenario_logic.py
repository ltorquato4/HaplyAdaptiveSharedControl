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
    StudyPoint(-0.08, -0.08, 0.0),
    StudyPoint(0.08, -0.08, 0.0),
    StudyPoint(0.08, 0.08, 0.0),
    StudyPoint(-0.08, 0.08, 0.0),
    StudyPoint(0.0, -0.15, 0.0),
]
DEFAULT_BOUNDS = WorkspaceBounds(
    x_min=-0.12,
    x_max=0.12,
    y_min=-0.15,
    y_max=0.15,
)


def test_default_points_are_valid():
    validate_task_points(DEFAULT_POINTS, DEFAULT_BOUNDS, min_segment_length=0.10)


def test_chained_segments_reuse_previous_endpoint_as_next_start():
    segments = [chained_segment(DEFAULT_POINTS, index) for index in range(5)]

    assert segments[0][0] == DEFAULT_POINTS[0]
    for index in range(4):
        assert segments[index][1] == segments[index + 1][0]
    assert segments[-1][1] == segments[0][0]


def test_rejects_points_outside_workspace():
    points = [
        StudyPoint(-0.13, -0.08, 0.0),
        StudyPoint(0.08, -0.08, 0.0),
        StudyPoint(0.08, 0.08, 0.0),
        StudyPoint(-0.08, 0.08, 0.0),
        StudyPoint(0.0, -0.15, 0.0),
    ]

    with pytest.raises(ValueError, match="outside workspace"):
        validate_task_points(points, DEFAULT_BOUNDS, min_segment_length=0.10)


def test_rejects_short_segments():
    points = [
        StudyPoint(-0.08, -0.08, 0.0),
        StudyPoint(-0.07, -0.08, 0.0),
        StudyPoint(0.08, 0.08, 0.0),
        StudyPoint(-0.08, 0.08, 0.0),
        StudyPoint(0.0, -0.15, 0.0),
    ]

    with pytest.raises(ValueError, match="shorter than"):
        validate_task_points(points, DEFAULT_BOUNDS, min_segment_length=0.10)


def test_endpoint_reached_uses_radius():
    endpoint = StudyPoint(0.08, -0.08, 0.0)

    assert endpoint_reached(StudyPoint(0.081, -0.081, 0.0), endpoint, 0.01)
    assert not endpoint_reached(StudyPoint(0.10, -0.10, 0.0), endpoint, 0.01)


def test_start_gate_latches_after_cursor_reaches_start():
    start = StudyPoint(-0.08, -0.08, 0.0)

    assert not update_start_gate(
        StudyPoint(0.08, -0.08, 0.0),
        start,
        start_gate_reached=False,
        start_reached_radius=0.01,
    )
    assert update_start_gate(
        StudyPoint(-0.081, -0.079, 0.0),
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
