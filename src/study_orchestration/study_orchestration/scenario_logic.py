"""Pure scenario helpers used by the Scenario Generator node."""

from dataclasses import dataclass
from math import sqrt


@dataclass(frozen=True)
class StudyPoint:
    """Task-frame point used for scenario validation and rollout."""

    x: float
    y: float
    z: float = 0.0


@dataclass(frozen=True)
class WorkspaceBounds:
    """Configurable task-frame workspace limits."""

    x_min: float
    x_max: float
    y_min: float
    y_max: float


def distance(first: StudyPoint, second: StudyPoint) -> float:
    """Return planar distance between two task-frame points."""
    dx = first.x - second.x
    dy = first.y - second.y
    return sqrt((dx * dx) + (dy * dy))


def validate_task_points(
    points: list[StudyPoint],
    bounds: WorkspaceBounds,
    expected_count: int = 5,
) -> None:
    """Validate the configured task points.

    Raises:
        ValueError: if the configured points are unsafe for the task frame.
    """
    if len(points) != expected_count:
        raise ValueError(
            f"scenario_generator requires exactly {expected_count} task points"
        )

    for index, point in enumerate(points):
        if not bounds.x_min <= point.x <= bounds.x_max:
            raise ValueError(f"point_{index}_x={point.x} is outside workspace bounds")
        if not bounds.y_min <= point.y <= bounds.y_max:
            raise ValueError(f"point_{index}_y={point.y} is outside workspace bounds")


def validate_equal_segment_lengths(
    segments: list[tuple[StudyPoint, StudyPoint]],
    tolerance: float = 1e-9,
) -> None:
    """Require every configured start/end pair to have equal planar length."""
    if not segments:
        raise ValueError("scenario_generator requires at least one path")

    expected_length = distance(*segments[0])
    for index, segment in enumerate(segments[1:], start=1):
        segment_length = distance(*segment)
        if abs(segment_length - expected_length) > tolerance:
            raise ValueError(
                f"path {index} length {segment_length:.6f} does not match "
                f"path 0 length {expected_length:.6f}"
            )


def chained_segment(
    points: list[StudyPoint], segment_index: int
) -> tuple[StudyPoint, StudyPoint]:
    """Return the start/end pair for a chained point rollout."""
    normalized_index = segment_index % len(points)
    return points[normalized_index], points[(normalized_index + 1) % len(points)]


def endpoint_reached(
    cursor: StudyPoint,
    endpoint: StudyPoint,
    endpoint_reached_radius: float,
) -> bool:
    """Return true when the cursor is close enough to the endpoint."""
    return distance(cursor, endpoint) <= endpoint_reached_radius


def update_start_gate(
    cursor: StudyPoint,
    start: StudyPoint,
    start_gate_reached: bool,
    start_reached_radius: float,
) -> bool:
    """Latch true once the cursor reaches the current start point."""
    return start_gate_reached or endpoint_reached(cursor, start, start_reached_radius)
