# Study Orchestration

This package maps raw Haply or mouse input into task coordinates and rolls out
the study scenarios. It owns neither hardware communication nor force-control
calculations.

## `experiment_mapper`

The Mapper is the single owner of raw Button A edge detection and calibration.

Before calibration it records raw position but publishes no experiment cursor.
On the first debounced rising edge it captures the raw neutral pose, maps it to
the fixed task anchor (`task_anchor_x`, `task_anchor_y`, default `(0, 0)`), and
publishes the latched `/study_mapping_ready=True` state. That first press is
not forwarded as a trial-start event. Each later release-and-rising-edge
publishes exactly one task-identified `/study_button_pressed` event.

The calibration anchor persists across scenario rollouts; new scenario start
points never silently recalibrate the device.

### Mapper interfaces

Subscribes to `/haply_state` (`haply_msgs/HaplyState`). Publishes:

- `/experiment_cursor_position` (`geometry_msgs/Point`)
- `/study_cursor` (`haply_msgs/StudyCursor`): timestamped mapped task-frame
  sample with `session_id`, `trial_id`, position, and validity. GUI and
  Scenario reject samples for any other task or samples older than their
  configured `cursor_max_age_s`. `/experiment_cursor_position`
  and `/experiment_input_valid` remain temporary Controller/Estimator/Logger
  compatibility topics.
- `/study_mapping_ready` (`std_msgs/Bool`, reliable transient-local)
- `/study_button_pressed` (`haply_msgs/StudyButtonPress`)

`input_timeout_s` defaults to `0.2` seconds. If raw input stops, the Mapper
marks input invalid and stops publishing cursor samples. A new raw sample marks
input valid again. `button_debounce_s` defaults to `0.05` seconds.

Supported mapping modes are `identity` and `anchored_delta`. The latter applies
the configured axis selection, inversion, scaling, and optional raw-displacement
clamp relative to the first-click anchor. The Haply launch uses `z` as task
`y`; mouse mode uses raw `x/y` with unit scale.

## `scenario_generator`

The Scenario Generator expands every phase, configured path segment, and
controller mode into a finite session schedule. `order_strategy:=seeded_random`
shuffles the behavioral-state order and the path order within each state. Its
resolved seed and complete order are emitted once in the startup log and in the
retained `/study_session` definition; set `order_seed` to a non-negative value
to reproduce a run exactly. `fixed` is available for debugging.

The Scenario Generator is the authoritative trial-state owner. The GUI publishes
an ID-bearing `/study_start_requested`; orchestration validates the current
cursor/input and publishes the typed lifecycle state.

Participant codes are assigned centrally and passed explicitly to hardware
production launches so they remain unique across study computers. The ordinary
mouse launch uses `P00`; controller debug wrappers use `DEBUG_MOUSE` or
`DEBUG_HAPLY`. Scenario includes the resolved label in `/study_session`; the
UUID session identity remains automatic.

Published task and trial topics:

- `/study_session` (`haply_msgs/StudySession`): schema version, UUID session
  identity, pseudonymous participant ID, input/controller family, ordering
  strategy and resolved seed, estimator lifecycle, loop policy, and the
  complete ordered task schedule.
- `/study_task` (`haply_msgs/StudyTask`): atomic task definition, including
  `session_id`, `trial_id`, start/end points, phase, and controller mode.
- `/study_trial_state` (`haply_msgs/StudyTrialState`): `READY`, `RUNNING`,
  `DWELL`, `COMPLETED`, `ABORTED`, or `SESSION_FINISHED`, plus a reason.
- `/study_endpoint_dwell_progress` (`haply_msgs/StudyDwellProgress`):
  `session_id`, `trial_id`, and continuous hold progress in `0.0–1.0`.

Scenario also publishes `/study_start_point`, `/study_end_point`,
`/study_phase`, `/study_controller_mode`, and `/study_endpoint_reached` as a
temporary metadata compatibility layer. `StudyTrialState` is the sole lifecycle
authority for MPC, State Feedback, Estimator, Logger, and GUI.

It subscribes to `/study_start_requested`, `/study_abort_requested`, and the
typed `/study_cursor` topic.

When `require_controller_ready`, `require_estimator_ready`, or
`require_logger_ready` is enabled, Scenario also requires the corresponding
heartbeat before accepting a start. Estimator and Logger become ready only
after applying matching retained `StudySession` and `StudyTask` messages;
Logger additionally has its session manifest and CSV output prepared. Scenario
publishes `/study_system_ready`; a required heartbeat timeout prevents starts
and aborts an active trial.

`task_file` may point to a YAML file containing a `paths` list. Each entry has
an independently defined `start_point` and `end_point`; paths need not form a
closed chain. The shared launches use `config/default_tasks.yaml`.

Endpoint completion requires all of the following:

1. The GUI has issued an accepted `StudyStartRequest`.
2. Input remains valid.
3. The configured minimum trial duration has elapsed.
4. The cursor remains continuously inside the endpoint radius for
   `endpoint_dwell_s` (default `1.0` second).

Leaving the endpoint or losing input resets dwell. Only successful dwell emits
`COMPLETED`. `inter_trial_delay_s` is a separate optional post-completion pause;
the provided GUI launches use `0.0` so the next task appears immediately after
dwell.

## Default task bounds

The default task coordinates are bounded by:

- x: `-0.12` to `0.12`
- y: `-0.15` to `0.15`
- minimum segment length: `0.10`

The nodes validate configured path endpoints and segment lengths at startup.
These task-space bounds must still be verified against the physical Haply
workspace before a participant run.

## Tests

```bash
pytest -q src/study_orchestration/test
```

The test suite covers mapper calibration/edge/debounce behavior, stale input,
continuous endpoint dwell, dwell reset, and scenario rollout.
