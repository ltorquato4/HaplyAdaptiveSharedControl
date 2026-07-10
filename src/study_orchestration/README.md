# Study Orchestration

This package contains the study-owned nodes that sit between the GUI, the raw
Haply input, and the controller-facing experiment topics.

## Nodes

### `scenario_generator`

Owns the task rollout for the experiment.

Publishes:

- `/study_start_point` (`geometry_msgs/Point`)
- `/study_end_point` (`geometry_msgs/Point`)
- `/study_phase` (`std_msgs/String`)
- `/study_controller_mode` (`std_msgs/String`)
- `/study_endpoint_reached` (`std_msgs/Bool`)

Subscribes:

- `/study_is_running` (`std_msgs/Bool`)
- `/experiment_cursor_position` (`geometry_msgs/Point`)

The generator uses exactly three configured task points. By default they are:

| Point | x | y | z |
|---|---:|---:|---:|
| `P0` | `-0.08` | `-0.20` | `0.0` |
| `P1` | `0.08` | `-0.08` | `0.0` |
| `P2` | `0.08` | `-0.20` | `0.0` |

Trials are chained as `P0 -> P1`, `P1 -> P2`, and `P2 -> P0`, so the start
point of each trial is the endpoint of the previous trial.

The default points are validated against the previous GUI dummy-test area:

- `workspace_x_min = -0.10`
- `workspace_x_max = 0.10`
- `workspace_y_min = -0.25`
- `workspace_y_max = -0.03`
- `min_segment_length = 0.10`

These bounds are task-frame defaults only. They are not a guarantee that the
same points are physically comfortable or reachable on the Haply device. Verify
the hardware range before subject testing and override the point parameters if
needed.

### `experiment_mapper`

Owns conversion from raw input-device coordinates to experiment/task-frame
cursor coordinates.

Publishes:

- `/experiment_cursor_position` (`geometry_msgs/Point`)

Subscribes:

- `/haply_state` (`haply_msgs/HaplyState`)
- `/study_start_point` (`geometry_msgs/Point`)
- `/study_is_running` (`std_msgs/Bool`)

Supported modes:

- `identity`: directly republishes raw position as the experiment cursor. This
  is used for mouse simulation because the GUI already generates fake
  `/haply_state` in task-frame coordinates.
- `anchored_delta`: captures the current raw device pose when a trial starts
  and maps subsequent raw displacement onto the current `/study_start_point`.
  This is the default for Haply hardware tests.

Useful parameters:

- `mapping_mode`: `identity` or `anchored_delta`
- `scale_x`, `scale_y`: scale raw displacement before publishing it
- `invert_x`, `invert_y`: flip axis direction if hardware testing shows the
  physical motion is reversed relative to the GUI task frame

## Mouse Test Flow

`ros2 launch haply_study_gui study_gui_mouse.launch.py`

The GUI publishes fake `/haply_state` from the mouse. The mapper converts that
to `/experiment_cursor_position`, and the scenario generator uses the mapped
cursor to detect when the endpoint is reached.

## Haply Test Flow

`ros2 launch haply_study_gui study_gui.launch.py`

The Haply driver publishes real `/haply_state`. The mapper anchors the current
raw Haply pose to the current task start point when the study starts, then
publishes mapped task-frame cursor updates for the GUI, Scenario Generator,
Controller, Estimator, and Logger.
