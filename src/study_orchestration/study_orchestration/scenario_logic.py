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
    min_segment_length: float,
) -> None:
    """Validate exactly three chained task points.

    Raises:
        ValueError: if the configured points are unsafe for the task frame.
    """
    if len(points) != 3:
        raise ValueError("scenario_generator requires exactly three task points")

    for index, point in enumerate(points):
        if not bounds.x_min <= point.x <= bounds.x_max:
            raise ValueError(f"point_{index}_x={point.x} is outside workspace bounds")
        if not bounds.y_min <= point.y <= bounds.y_max:
            raise ValueError(f"point_{index}_y={point.y} is outside workspace bounds")

    for index in range(3):
        start, end = chained_segment(points, index)
        segment_length = distance(start, end)
        if segment_length < min_segment_length:
            raise ValueError(
                f"segment {index} length {segment_length:.4f} is shorter than "
                f"min_segment_length={min_segment_length:.4f}"
            )


def chained_segment(
    points: list[StudyPoint], segment_index: int
) -> tuple[StudyPoint, StudyPoint]:
    """Return the start/end pair for a chained three-point rollout."""
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
