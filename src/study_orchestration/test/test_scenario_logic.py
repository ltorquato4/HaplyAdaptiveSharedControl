"""Tests for Scenario Generator pure logic."""

import pytest
from study_orchestration.scenario_logic import (
    ScenarioPath,
    StudyPoint,
    WorkspaceBounds,
    default_scenario_paths,
    endpoint_reached,
    expand_scenario_tasks,
    load_scenario_tasks,
    parse_controller_modes,
    parse_scenario_paths,
    update_start_gate,
    validate_scenario_paths,
)

DEFAULT_BOUNDS = WorkspaceBounds(
    x_min=-0.12,
    x_max=0.12,
    y_min=-0.15,
    y_max=0.15,
)


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


def test_default_scenario_paths_are_chained_rectangle_edges():
    paths = default_scenario_paths()

    assert paths == [
        ScenarioPath(StudyPoint(0.08, -0.08, 0.0), StudyPoint(0.08, 0.08, 0.0)),
        ScenarioPath(StudyPoint(0.08, 0.08, 0.0), StudyPoint(-0.08, 0.08, 0.0)),
        ScenarioPath(StudyPoint(-0.08, 0.08, 0.0), StudyPoint(-0.08, -0.08, 0.0)),
        ScenarioPath(StudyPoint(-0.08, -0.08, 0.0), StudyPoint(0.08, -0.08, 0.0)),
    ]
    assert paths[0].start_point == paths[-1].end_point
    for index in range(1, len(paths)):
        assert paths[index].start_point == paths[index - 1].end_point


def test_parse_scenario_paths_loads_paths_and_defaults_z():
    paths = parse_scenario_paths(
        {
            "paths": [
                {
                    "start_point": [0.08, -0.08],
                    "end_point": [0.08, 0.08],
                },
                {
                    "start_point": [0.08, 0.08, 0.0],
                    "end_point": [-0.08, 0.08, 0.0],
                },
            ]
        }
    )

    validate_scenario_paths(paths, DEFAULT_BOUNDS, min_segment_length=0.10)

    assert len(paths) == 2
    assert paths[0].start_point == StudyPoint(0.08, -0.08, 0.0)
    assert paths[1].end_point == StudyPoint(-0.08, 0.08, 0.0)


def test_load_scenario_tasks_expands_yaml_paths(tmp_path):
    task_file = tmp_path / "paths.yaml"
    task_file.write_text(
        """
paths:
  - start_point: [0.08, -0.08, 0.0]
    end_point: [0.08, 0.08, 0.0]
  - start_point: [0.08, 0.08, 0.0]
    end_point: [-0.08, 0.08, 0.0]
""",
        encoding="utf-8",
    )

    tasks = load_scenario_tasks(
        str(task_file),
        DEFAULT_BOUNDS,
        min_segment_length=0.10,
        controller_modes=["adaptive"],
    )

    assert len(tasks) == 6
    assert [task.phase for task in tasks] == ["aggressive"] * 2 + ["normal"] * 2 + [
        "careful"
    ] * 2
    assert {task.controller_mode for task in tasks} == {"adaptive"}


def test_load_scenario_tasks_uses_default_paths_when_file_is_empty():
    tasks = load_scenario_tasks(
        "",
        DEFAULT_BOUNDS,
        min_segment_length=0.10,
        controller_modes=["fixed"],
    )

    assert len(tasks) == 12
    assert tasks[0].phase == "aggressive"
    assert tasks[4].phase == "normal"
    assert tasks[8].phase == "careful"
    assert {task.controller_mode for task in tasks} == {"fixed"}


def test_expand_scenario_tasks_supports_both_controller_modes():
    paths = default_scenario_paths()

    tasks = expand_scenario_tasks(paths, ["adaptive", "fixed"])

    assert len(tasks) == 24
    assert [task.controller_mode for task in tasks[:12]] == ["adaptive"] * 12
    assert [task.controller_mode for task in tasks[12:]] == ["fixed"] * 12
    assert [task.phase for task in tasks[:4]] == ["aggressive"] * 4
    assert [task.phase for task in tasks[4:8]] == ["normal"] * 4
    assert [task.phase for task in tasks[8:12]] == ["careful"] * 4


def test_expand_scenario_tasks_reflects_yaml_path_count():
    paths = default_scenario_paths()[:3]

    tasks = expand_scenario_tasks(paths, ["adaptive"])

    assert len(tasks) == 9
    assert tasks[3].phase == "normal"
    assert tasks[6].phase == "careful"


def test_parse_controller_modes_accepts_comma_separated_modes():
    assert parse_controller_modes("adaptive,fixed") == ["adaptive", "fixed"]
    assert parse_controller_modes("") == ["fixed"]


def test_parse_controller_modes_rejects_invalid_modes():
    with pytest.raises(ValueError, match="invalid modes"):
        parse_controller_modes("adaptive,manual")


def test_parse_scenario_paths_rejects_missing_paths():
    with pytest.raises(ValueError, match="top-level 'paths'"):
        parse_scenario_paths({})


def test_parse_scenario_paths_rejects_empty_paths():
    with pytest.raises(ValueError, match="at least one path"):
        parse_scenario_paths({"paths": []})


def test_parse_scenario_paths_rejects_malformed_points():
    with pytest.raises(ValueError, match="2 or 3 numeric values"):
        parse_scenario_paths(
            {
                "paths": [
                    {
                        "start_point": [0.08],
                        "end_point": [0.08, 0.08, 0.0],
                    }
                ]
            }
        )


def test_validate_scenario_paths_rejects_out_of_bounds_point():
    paths = [
        ScenarioPath(
            start_point=StudyPoint(-0.13, -0.08, 0.0),
            end_point=StudyPoint(0.08, -0.08, 0.0),
        )
    ]

    with pytest.raises(ValueError, match="outside workspace"):
        validate_scenario_paths(paths, DEFAULT_BOUNDS, min_segment_length=0.10)


def test_validate_scenario_paths_rejects_short_path():
    paths = [
        ScenarioPath(
            start_point=StudyPoint(-0.08, -0.08, 0.0),
            end_point=StudyPoint(-0.07, -0.08, 0.0),
        )
    ]

    with pytest.raises(ValueError, match="shorter than"):
        validate_scenario_paths(paths, DEFAULT_BOUNDS, min_segment_length=0.10)
