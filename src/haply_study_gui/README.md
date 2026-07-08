# Haply Study GUI

This package contains project-owned GUI code for the Haply shared-control user
study. The copied `haply_ros2_interface` packages remain responsible for the
hardware interface and Haply messages.

## Architecture

The GUI is a visual instruction and experiment feedback publisher:

- subscribes to `experiment_cursor_position` for live experiment cursor
  feedback
- subscribes to `study_start_point`, `study_end_point`, `study_phase`, and
  `study_controller_mode` from the Scenario Generator
- publishes `study_is_running`
- renders the participant-facing Pygame window at 100 Hz by default
- supports `source=mouse` for testing and `source=inverse3` for hardware input
- publishes fake `inverse3_state` in mouse mode so the Experiment Mapper and
  Scenario Generator can be tested without hardware
- does not publish `haply_target`
- hardware runs are started with keyboard input or launch parameters
- does not own phase rollout, controller mode, or start/end point generation
- does not own endpoint detection or trial rollout
- does not implement fixed/adaptive controller logic
- does not estimate human parameters or log experiment data

This keeps the study GUI separate from the copied Haply driver code and leaves
force commands to the controller node.

## Haply Connection

The hardware launch starts `haply_interface/haply_driver_node`. That driver
connects to the Haply Inverse SDK Service at `ws://localhost:10001`. For this
study stack, the grounded hardware input is:

- `inverse3_state` (`haply_msgs/Inverse3State`) for Inverse3 position and
  velocity

The GUI receives the experiment cursor from `experiment_cursor_position`. It
uses `source=inverse3` for hardware mode.

## Run

Use `ros2 launch` for normal testing because the GUI needs other ROS nodes to
publish scenario or hardware data. Use `ros2 run` only when you intentionally
want to start one executable by itself.

### Testing

| Purpose | Command |
| --- | --- |
| Mouse GUI with Mapper and Scenario Generator | `ros2 launch haply_study_gui study_gui_mouse.launch.py` |
| GUI only with mouse source | `ros2 run haply_study_gui study_gui --ros-args -p source:=mouse` |
| Self-contained `/inverse3_state` smoke test | `ros2 run haply_study_gui test_inverse3_state_topic --fake` |

### Hardware

Use the hardware launch only after the Haply Inverse SDK Service is running
and reachable on port `10001`. The ROS driver connects to it as a WebSocket
server at `ws://localhost:10001`; this is not a normal web page, so opening
`http://localhost:10001` in a browser may not show anything useful. Start the
Haply Inverse SDK Service in the same WSL environment that runs ROS so the
copied Haply driver can use its expected `ws://localhost:10001` endpoint.

| Purpose | Command |
| --- | --- |
| Haply GUI with Mapper and Scenario Generator | `ros2 launch haply_study_gui study_gui.launch.py` |
| Haply GUI test launch | `ros2 launch haply_study_gui study_gui_haply_test.launch.py` |
| GUI only, expecting an existing `/inverse3_state` publisher | `ros2 run haply_study_gui study_gui --ros-args -p source:=inverse3` |
| Check live `/inverse3_state` messages | `ros2 run haply_study_gui test_inverse3_state_topic` |

The mouse launch starts `study_gui`, `experiment_mapper`, and
`scenario_generator`. The GUI publishes fake `/inverse3_state`, the mapper
publishes `/experiment_cursor_position`, and the Scenario Generator detects
endpoint completion from the mapped cursor.

The hardware launch starts `study_gui`, `haply_driver_node`,
`experiment_mapper`, and `scenario_generator`. The default task points are
inside the previous GUI dummy-test area, but they still need to be verified on
the physical Haply workspace before subject testing.

## Interface

![Haply Study GUI](study_GUI.png)

The left box is the drawing workspace. The right panel shows the Behavioral
State legend and run status. In mouse mode, move the mouse inside the workspace
to move the blue cursor through the mapper. The Scenario Generator rolls out the
next phase when the mapped cursor reaches the current endpoint.

The participant-facing GUI does not show coordinate text. Keyboard shortcuts
remain available for test runs: `S` sets `study_is_running=True`, `Space`
toggles it, and `Esc`/`Q` closes the window.
