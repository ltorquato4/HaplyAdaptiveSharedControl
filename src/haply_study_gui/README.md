# Haply Study GUI

This package contains project-owned GUI code for the Haply shared-control user
study. The copied `haply_ros2_interface` packages remain responsible for the
hardware interface and Haply messages.

## Architecture

The GUI is a visual instruction and experiment feedback publisher:

- subscribes to `haply_state` for live Haply cursor feedback
- subscribes to `study_start_point`, `study_end_point`, `study_phase`, and
  `study_controller_mode` from the future Scenario Generator
- publishes `study_is_running` and `study_endpoint_reached`
- renders the participant-facing Pygame window at 100 Hz by default
- supports `source=mouse` for testing and `source=haply` for hardware input
- includes `dummy_scenario_generator` for local GUI testing until the real
  Scenario Generator exists
- does not publish `haply_target`
- does not own phase rollout, controller mode, or start/end point generation
- does not implement fixed/adaptive controller logic
- does not estimate human parameters or log experiment data

This keeps the study GUI separate from the copied Haply driver code and leaves
force commands to the controller node.

## Haply Connection

The hardware launch starts `haply_interface/haply_driver_node`. That driver
connects to the Haply Inverse SDK Service at `ws://localhost:10001` and
publishes:

- `haply_state` (`haply_msgs/HaplyState`) for combined Inverse3 position,
  velocity, handle orientation, and button state
- `inverse3_state` (`haply_msgs/Inverse3State`) for Inverse3 position and
  velocity
- `handle_state` (`haply_msgs/HandleState`) for VerseGrip orientation and
  buttons

The GUI subscribes to `haply_state` when `source=haply`.

## Run

Use `ros2 launch` for normal testing because the GUI needs other ROS nodes to
publish scenario or hardware data. Use `ros2 run` only when you intentionally
want to start one executable by itself.

### Testing

| Purpose | Command |
| --- | --- |
| Mouse GUI with dummy Scenario Generator | `ros2 launch haply_study_gui study_gui_mouse.launch.py` |
| GUI only with mouse source | `ros2 run haply_study_gui study_gui --ros-args -p source:=mouse` |
| Self-contained `/haply_state` smoke test | `ros2 run haply_study_gui test_haply_state_topic --fake` |

### Hardware

| Purpose | Command |
| --- | --- |
| Haply GUI with dummy Scenario Generator | `ros2 launch haply_study_gui study_gui_haply_test.launch.py` |
| TODO: GUI with Haply driver and real Scenario Generator | `ros2 launch haply_study_gui study_gui.launch.py` |
| GUI only, expecting an existing `/haply_state` publisher | `ros2 run haply_study_gui study_gui --ros-args -p source:=haply` |
| Check live `/haply_state` messages | `ros2 run haply_study_gui test_haply_state_topic` |

The mouse launch starts both `study_gui` and `dummy_scenario_generator`. The
dummy generator publishes start/end points, phase, and controller mode through
the same topics that the future Scenario Generator will use.

The Haply test launch starts `study_gui`, `haply_driver_node`, and
`dummy_scenario_generator`. Use it to test the GUI with the real Haply device
before the real Scenario Generator is implemented.

The TODO hardware launch starts `study_gui` and `haply_driver_node` only. It
does not start `dummy_scenario_generator`; it is reserved for the full
experiment once the real Scenario Generator publishes the study phase,
controller mode, and start/end points.

## Interface

![Haply Study GUI](study_GUI.png)

The left box is the drawing workspace. The right panel shows the Behavioral
State legend and run status. Move the mouse inside the workspace to move the
blue cursor; click and drag from the start marker to the endpoint to draw the
participant path. When the drawn line connects both points, the dummy generator
rolls out the next phase.

The participant-facing GUI does not show start/pause/reset buttons or
coordinate text. Keyboard shortcuts remain available for test runs: `S` sets
`study_is_running=True`, `Space` toggles it, and `Esc`/`Q` closes the window.
