"""Pure scenario helpers used by the Scenario Generator node."""

from dataclasses import dataclass
from math import sqrt
from pathlib import Path
from typing import Any

import yaml

VALID_PHASES = ("aggressive", "normal", "careful")
VALID_CONTROLLER_MODES = ("adaptive", "fixed")


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


@dataclass(frozen=True)
class ScenarioTask:
    """Explicit task definition loaded from scenario configuration."""

    start_point: StudyPoint
    end_point: StudyPoint
    phase: str
    controller_mode: str


@dataclass(frozen=True)
class ScenarioPath:
    """Path geometry loaded from scenario configuration."""

    start_point: StudyPoint
    end_point: StudyPoint


def distance(first: StudyPoint, second: StudyPoint) -> float:
    """Return planar distance between two task-frame points."""
    dx = first.x - second.x
    dy = first.y - second.y
    return sqrt((dx * dx) + (dy * dy))


def default_scenario_paths() -> list[ScenarioPath]:
    """Return the in-code fallback path sequence."""
    return [
        ScenarioPath(StudyPoint(0.08, -0.08, 0.0), StudyPoint(0.08, 0.08, 0.0)),
        ScenarioPath(StudyPoint(0.08, 0.08, 0.0), StudyPoint(-0.08, 0.08, 0.0)),
        ScenarioPath(StudyPoint(-0.08, 0.08, 0.0), StudyPoint(-0.08, -0.08, 0.0)),
        ScenarioPath(StudyPoint(-0.08, -0.08, 0.0), StudyPoint(0.08, -0.08, 0.0)),
    ]


def load_scenario_tasks(
    task_file: str,
    bounds: WorkspaceBounds,
    min_segment_length: float,
    controller_modes: list[str],
) -> list[ScenarioTask]:
    """Load scenario paths from YAML and expand them into tasks."""
    if not task_file:
        paths = default_scenario_paths()
    else:
        with Path(task_file).open("r", encoding="utf-8") as file:
            payload = yaml.safe_load(file)
        paths = parse_scenario_paths(payload)

    validate_scenario_paths(paths, bounds, min_segment_length)
    return expand_scenario_tasks(paths, controller_modes)


def parse_controller_modes(value: str) -> list[str]:
    """Parse controller mode rollout from a comma-separated parameter."""
    modes = [mode.strip().lower() for mode in value.split(",")]
    modes = [mode for mode in modes if mode]
    invalid_modes = [mode for mode in modes if mode not in VALID_CONTROLLER_MODES]
    if invalid_modes:
        raise ValueError(
            f"controller_modes contains invalid modes {invalid_modes}; "
            f"expected values from {list(VALID_CONTROLLER_MODES)}"
        )
    return modes or ["fixed"]


def parse_scenario_paths(payload: Any) -> list[ScenarioPath]:
    """Parse scenario paths from a loaded YAML payload."""
    if not isinstance(payload, dict) or "paths" not in payload:
        raise ValueError("scenario task file must contain top-level 'paths'")

    raw_paths = payload["paths"]
    if not isinstance(raw_paths, list) or not raw_paths:
        raise ValueError("scenario task file must define at least one path")

    return [_parse_path(raw_path, index) for index, raw_path in enumerate(raw_paths)]


def validate_scenario_paths(
    paths: list[ScenarioPath],
    bounds: WorkspaceBounds,
    min_segment_length: float,
    expected_count: int = 5,
) -> None:
    """Validate scenario path definitions."""
    if not paths:
        raise ValueError("scenario_generator requires at least one path")

    for index, path in enumerate(paths):
        _validate_point(path.start_point, bounds, f"paths[{index}].start_point")
        _validate_point(path.end_point, bounds, f"paths[{index}].end_point")

        segment_length = distance(path.start_point, path.end_point)
        if segment_length < min_segment_length:
            raise ValueError(
                f"path {index} length {segment_length:.4f} is shorter than "
                f"min_segment_length={min_segment_length:.4f}"
            )


def expand_scenario_tasks(
    paths: list[ScenarioPath],
    controller_modes: list[str],
) -> list[ScenarioTask]:
    """Expand paths into phase and controller-mode task blocks."""
    return [
        ScenarioTask(
            start_point=path.start_point,
            end_point=path.end_point,
            phase=phase,
            controller_mode=controller_mode,
        )
        for controller_mode in controller_modes
        for phase in VALID_PHASES
        for path in paths
    ]


def _parse_path(raw_path: Any, index: int) -> ScenarioPath:
    if not isinstance(raw_path, dict):
        raise ValueError(f"paths[{index}] must be a mapping")

    try:
        start_point = _parse_point(
            raw_path["start_point"], f"paths[{index}].start_point"
        )
        end_point = _parse_point(raw_path["end_point"], f"paths[{index}].end_point")
    except KeyError as exc:
        raise ValueError(f"paths[{index}] is missing required field {exc}") from exc

    return ScenarioPath(
        start_point=start_point,
        end_point=end_point,
    )


def _parse_point(value: Any, field_name: str) -> StudyPoint:
    if not isinstance(value, (list, tuple)) or len(value) not in (2, 3):
        raise ValueError(f"{field_name} must contain 2 or 3 numeric values")

    try:
        x = float(value[0])
        y = float(value[1])
        z = float(value[2]) if len(value) == 3 else 0.0
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field_name} must contain numeric values") from exc

    return StudyPoint(x=x, y=y, z=z)


def _validate_point(
    point: StudyPoint,
    bounds: WorkspaceBounds,
    field_name: str,
) -> None:
    if not bounds.x_min <= point.x <= bounds.x_max:
        raise ValueError(f"{field_name}.x={point.x} is outside workspace bounds")
    if not bounds.y_min <= point.y <= bounds.y_max:
        raise ValueError(f"{field_name}.y={point.y} is outside workspace bounds")


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
