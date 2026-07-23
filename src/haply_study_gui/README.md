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
  retained only for Controller/Estimator/Logger compatibility.
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
| Hardware GUI | `ros2 launch haply_study_gui study_gui.launch.py` |
| Full MPC hardware stack (includes Estimator, Data Logger, and readiness gate) | `ros2 launch haply_study_gui study_gui.launch.py controller:=mpc` |
| Start state-feedback controller with hardware GUI | `ros2 launch haply_study_gui study_gui.launch.py controller:=state_feedback` |

The hardware launch requires the Haply Inverse SDK Service to be running at
`ws://localhost:10001` before ROS starts.

When launched with `controller:=mpc` or `controller:=state_feedback`, the
hardware GUI waits for Controller, Estimator, and Logger readiness before it
opens. The mouse launch is intentionally a lightweight/debug path and does not
wait for that production gate.

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
