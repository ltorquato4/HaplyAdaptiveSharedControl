# Haply Study GUI

This package provides the participant-facing Pygame GUI for the Haply
shared-control study. It renders task targets and the mapped experiment cursor;
it does not communicate with the Haply hardware directly or publish forces.

## Participant flow

Both `source=mouse` and `source=haply` follow the same press interaction:

1. Place the device at its neutral position and press A (left mouse button in
   mouse mode). This calibrates the mapper; no trial starts.
2. Move the mapped cursor to the current start marker and release then press A
   again. A trial starts only on this discrete second press.
3. Trace to the endpoint and remain there continuously for the configured
   dwell duration. The GUI indicates when endpoint dwell has begun.
4. After successful dwell, the next scenario is shown. A held button is never
   reused as a press for the next scenario.

On the first task and each behavioral-mode change, the workspace shows a
two-second fading instruction overlay. It does not require a click; trial start
is enabled automatically when the overlay has faded.

Before calibration the cursor is hidden. The GUI also hides it and prevents
trial start when device input is stale, unavailable, or outside the configured
task workspace. Start and endpoint markers use their configured task-space
radii; acceptance remains in task coordinates. Mouse mode
deliberately uses identity mapping after calibration, so its blue cursor remains
exactly under the operating-system mouse pointer; hardware mode uses
anchored-delta mapping from its neutral calibration pose.

## ROS interfaces

Subscribes to:

- `/experiment_cursor_position` (`geometry_msgs/Point`): legacy mapped cursor
  retained for Controller/Logger compatibility and Estimator debug fallback.
- `/study_cursor` (`haply_msgs/StudyCursor`): timestamped, ID-bearing mapped
  cursor sample. The GUI rejects stale-session/trial samples and invalid input.
- `/study_mapping_ready` (`std_msgs/Bool`): latched calibration state.
- `/experiment_input_valid` (`std_msgs/Bool`): raw device/mouse health, kept
  separate from task-specific cursor validity.
- `/study_button_pressed` (`haply_msgs/StudyButtonPress`): task-identified
  post-calibration press events.
- `/study_task` (`haply_msgs/StudyTask`): atomic task definition, including
  session/trial IDs, start/end points, phase, and controller mode.
- `/study_trial_state` (`haply_msgs/StudyTrialState`): authoritative lifecycle.
- `/study_endpoint_dwell_progress` (`haply_msgs/StudyDwellProgress`):
  ID-bearing endpoint-hold progress.
- `/study_system_ready` (`std_msgs/Bool`): required production components have
  applied the active task and are reporting heartbeats.

Publishes:

- `/study_start_requested` (`haply_msgs/StudyStartRequest`): ID-bearing request
  to start a validated trial.
- `/study_abort_requested` (`haply_msgs/StudyAbortRequest`): ID-bearing request
  to abort the active trial when the GUI closes.
- `/haply_state` (`haply_msgs/HaplyState`) only in mouse mode, to simulate raw
  device position and buttons for the Mapper.

Live state topics use depth-one QoS so consumers receive the current sample
rather than a queue of old cursor positions. Task and calibration state use
reliable transient-local QoS.

## Run

Source ROS and the workspace before launching:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

| Purpose | Command |
| --- | --- |
| Mouse simulation | `ros2 launch haply_study_gui study_gui_mouse.launch.py` |
| Full state-feedback hardware stack (default) | `ros2 launch haply_study_gui study_gui.launch.py participant_id:=P03` |
| Full MPC hardware stack (includes Estimator, Data Logger, and readiness gate) | `ros2 launch haply_study_gui study_gui.launch.py controller:=mpc participant_id:=P03` |
| Start state-feedback controller with hardware GUI | `ros2 launch haply_study_gui study_gui.launch.py controller:=state_feedback participant_id:=P03` |

The hardware launch defaults to state feedback and requires the Haply Inverse SDK Service to be running at
`ws://localhost:10001` before ROS starts.

When launched with `controller:=mpc` or `controller:=state_feedback`, the
hardware GUI waits until Controller has applied its task and Estimator and
Logger have applied matching session/task metadata. Logger must also have its
session manifest ready before the GUI opens. The mouse launch does not wait for
that production gate. When its `controller` is `mpc` or `state_feedback`, it
nevertheless starts Data Logger automatically so the run can be consumed by
`study_analysis`. With `controller:=none`, it keeps the lightweight GUI-only
behavior and does not start Logger.

Participant codes must be assigned centrally because experiments may run on
different computers. The hardware production launch therefore requires an
explicit value such as `participant_id:=P03`. The ordinary mouse launch uses
`P00` for tests. The controller debug wrappers instead use `DEBUG_MOUSE` and
`DEBUG_HAPLY`, producing recognizable folders without requiring identification.
Logger uses the label in names such as `P03_2026-07-23_16-42-08Z`, while the
retained session UUID remains automatic and is stored in the manifest and CSV
metadata.

State feedback uses a dedicated executable and docking is disabled by default.
Enable it with one argument:

```bash
ros2 launch haply_study_gui study_gui.launch.py \
  participant_id:=P03 docking_enabled:=true
```

This activates `docking_start_percent=85`, `docking_stiffness_scale=2.0`, and
`docking_max_cross_track_m=0.02`. These modifiers remain inert when docking is
disabled. The independent global force limit is `max_force_n=2.0` for both
modes. Numeric State Feedback defaults are centralized in
`control_node/config/state_feedback.yaml`; they are not duplicated as launch
arguments. Docking uses directed path projection and is suppressed while the
cursor is outside the configured lateral corridor. The force-norm limit is
stored in session metadata for saturation analysis.

The launch applies configuration in three layers: shared study settings from
`study_base.yaml`, a `study_mouse.yaml` or `study_haply.yaml` source overlay,
and only the selected `state_feedback.yaml` or `mpc.yaml` controller profile.
The only State Feedback numeric override intended for a normal run is therefore
a deliberate edit to its profile; `docking_enabled` remains a run-time switch.

## GUI behavior and parameters

- `debug_controls_enabled` defaults to `false`. Only when enabled do `S` and
  `Space` bypass normal trial controls.
- `auto_start` is ignored unless debug controls are enabled.
- `max_callbacks_per_frame` defaults to `16`, preventing ROS callback backlog
  without unbounded work in one render frame.
- `mode_overlay_duration_s` defaults to `2.0`. It controls the automatic
  behavioral-mode instruction overlay shown on the initial task and mode changes.
- Mouse simulation stops publishing raw state outside the drawing area. A
  mapped hardware cursor outside the task workspace is hidden and reported as
  `cursor outside workspace` until it returns.
- The drawing transform fills the visible workspace in both axes.
- The sidebar distinguishes controller family (`MPC`, `State Feedback`, or
  `None`) from the task mode (`adaptive` or `fixed`).

Run package tests with:

```bash
pytest -q src/haply_study_gui/test
```
