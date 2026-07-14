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

The generator loads path geometry from YAML. The default file is installed at
`share/study_orchestration/config/default_tasks.yaml` and contains four chained
rectangle-edge paths: lower right to upper right, upper right to upper left,
upper left to lower left, and lower left back to lower right.

The YAML defines only start/end path geometry:

```yaml
paths:
  - start_point: [0.08, -0.08, 0.0]
    end_point: [0.08, 0.08, 0.0]
```

The Scenario Generator expands those paths into task definitions internally.
Behavioral phases are fixed in code as `aggressive`, `normal`, and `careful`.
Controller modes are selected with the `controller_modes` parameter. The default
is `fixed`, which creates 12 tasks: four paths for each of the three phases.
If `controller_modes:=adaptive,fixed` is used, the rollout creates 24 tasks in
mode-first order: all adaptive phase blocks, then all fixed phase blocks.

Use a custom path file by passing `task_file` to the launch:

```bash
ros2 launch haply_study_gui study_gui_mouse.launch.py task_file:=/path/to/paths.yaml
```

Select controller modes without editing the YAML:

```bash
ros2 launch haply_study_gui study_gui_mouse.launch.py controller_modes:=adaptive,fixed
```

Mirrored paths should be added as explicit YAML path entries rather than
generated in Python. Adding or removing paths changes the number of trials per
phase block without code changes.

The task-definition topics (`/study_start_point`, `/study_end_point`,
`/study_phase`, and `/study_controller_mode`) are published when the task
changes and use transient local QoS so late-starting nodes receive the latest
task without requiring continuous republishing.

When the mapped cursor reaches the current endpoint during a running trial, the
generator publishes `/study_endpoint_reached=True`, waits
`inter_trial_delay_s` seconds, then publishes the next start/end task and resets
`/study_endpoint_reached=False`. The default inter-trial delay is `1.0` second.

The default paths are validated against the previous GUI dummy-test area:

- `workspace_x_min = -0.12`
- `workspace_x_max = 0.12`
- `workspace_y_min = -0.15`
- `workspace_y_max = 0.15`
- `min_segment_length = 0.10`

These bounds are task-frame defaults only. They are not a guarantee that the
same points are physically comfortable or reachable on the Haply device. Verify
the hardware range before subject testing and override the path YAML if needed.

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
  This is the default for Haply hardware tests. The physical position where the
  participant starts the trial becomes the raw anchor for that trial, and the
  mapped cursor begins at the task start point.

Useful parameters:

- `mapping_mode`: `identity` or `anchored_delta`
- `scale_x`, `scale_y`: scale raw displacement before publishing it
- `invert_x`, `invert_y`: flip axis direction if hardware testing shows the
  physical motion is reversed relative to the GUI task frame
- `use_z_as_y`: map the Haply vertical `z` axis to the task `y` axis. The
  default Haply launch files set this to `True` for the 2-DoF study.
- `clamp_raw`: clamp raw displacement relative to the captured anchor pose
  before scaling.
- `raw_x_min`, `raw_x_max`: left/right physical displacement limits used when
  `clamp_raw=True`.
- `raw_second_min`, `raw_second_max`: down/up physical displacement limits used
  when `clamp_raw=True`. When `use_z_as_y=True`, these apply to raw `z`.

The default Haply launch files use `scale_x=2.0` and `scale_y=2.0`, so a 10 cm
physical movement maps to 20 cm of task movement. These parameters can be tuned
in `study_gui.launch.py` and `study_gui_haply_test.launch.py` without rebuilding
the workspace.

In hardware mode, the mapper creates a pretrial preview anchor as soon as it has
both the raw Haply pose and the current `/study_start_point`. This lets the GUI
show a mapped cursor before the trial starts. When `/study_is_running` becomes
true, the mapper locks that anchor for the active trial. Releasing and pressing
VerseGrip Button A during the same active trial does not recapture the anchor;
the next task captures a new anchor when the next trial starts.

## Mouse Test Flow

`ros2 launch haply_study_gui study_gui_mouse.launch.py`

The GUI publishes fake `/haply_state` from the mouse. The mapper converts that
to `/experiment_cursor_position`, and the scenario generator uses the mapped
cursor to detect when the endpoint is reached.

## Haply Test Flow

`ros2 launch haply_study_gui study_gui.launch.py`

The Haply driver publishes real `/haply_state`. The mapper publishes pretrial
and active mapped task-frame cursor updates for the GUI, Scenario Generator,
Controller, Estimator, and Logger.
