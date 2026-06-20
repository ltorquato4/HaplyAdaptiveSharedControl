# Haply Study GUI

This package contains project-owned GUI code for the Haply shared-control user
study. The copied `haply_ros2_interface` packages remain responsible for the
hardware interface and Haply messages.

## Architecture

The GUI is a visual instruction and experiment-state publisher:

- subscribes to `haply_state` for live Haply cursor feedback
- publishes `study_behavior_state`, `study_trial_state`, `study_start_point`,
  and `study_end_point`
- does not publish `haply_target`
- does not implement fixed/adaptive controller logic
- does not estimate human parameters or log experiment data

This keeps the study GUI separate from the copied Haply driver code and leaves
force commands to the controller node.

## Run

Run only the GUI:

```bash
ros2 run haply_study_gui study_traffic_light_gui
```

Run the GUI with the Haply driver:

```bash
ros2 launch haply_study_gui study_gui.launch.py
```
