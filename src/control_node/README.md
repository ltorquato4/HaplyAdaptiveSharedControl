# Control nodes

This package now exposes two runtime paths with separate executables:

- `state_feedback_control_node` is the production state-feedback implementation.
  It does not import or instantiate MPC classes.
- `control_node` retains the legacy MPC-capable implementation. Selecting MPC
  from the study launch uses this executable with its original MPC behavior.

The shared GUI launch selects the correct executable from `controller`:

```bash
# Default production controller
ros2 launch haply_study_gui study_gui.launch.py controller:=state_feedback

# Legacy MPC path
ros2 launch haply_study_gui study_gui.launch.py controller:=mpc
```

## Configuration profiles

Controller parameters are owned by this package:

- `config/state_feedback.yaml` contains virtual-fixture, timing, adaptation,
  force-limit, and optional docking defaults.
- `config/mpc.yaml` contains the untouched legacy MPC runtime defaults and its
  workspace constraints. It also exposes the legacy adaptive-MPC docking
  constants that were previously hard-coded, without changing their values.

The shared GUI factory loads only the profile matching `controller`. General
Scenario/Mapper/GUI settings remain in `study_orchestration/config`; launch
arguments select the family and log level rather than repeating numerical
controller constants. `docking_enabled` is the one intentional State Feedback
run-time switch, while all docking numbers remain inert in the YAML unless that
switch is true.

Adaptive MPC retains its historical latched terminal zone at 85% progress. Its
profile names the actual MPC weight changes—comfort reduction, trajectory
weight scaling, and goal-weight scaling—rather than calling them State Feedback
stiffness. MPC also retains its historical component-wise physical limit:
`max_control_amplitude=10` multiplied by
`acceleration_to_force_factor=0.2` gives 2 N per device force axis. This is not
the State Feedback controller's independent 2 N vector-norm limit.

## State-feedback force law

State feedback consumes the retained atomic `/study_task`, authoritative
`/study_trial_state`, and each valid timestamped `/study_cursor` sample exactly
once. Velocity is calculated from consecutive cursor timestamps rather than a
fixed assumed period. Derivative history resets at every trial or retry.

The straight path is represented by a unit direction vector. Cursor displacement
and velocity are decomposed into:

- an along-path component, which provides goal-directed assistance; and
- a perpendicular component, which generates the virtual-fixture restoring
  force that keeps the participant near the start-to-end line.

The output is a Cartesian force in newtons:

```text
F = K_goal * remaining_along - D_goal * velocity_along
    - K_fixture * cross_track - D_fixture * velocity_cross
```

The force norm is bounded by `max_force_n` before it is mapped from task X/Y to
Inverse3 X/Z. This follows Haply's documented force-feedback model: applications
calculate spring/damping forces from cursor position and velocity and send
Cartesian force commands in newtons. No participant-applied force measurement
is required for this controller.

References:

- [Haply control commands](https://docs.haply.co/inverseSDK/service/realtime/control-commands/)
- [Haply basic force-feedback tutorial](https://docs.haply.co/inverseSDK/2.1.1/unity/tutorials/basic-force-feedback)

Default state-feedback parameters are intentionally conservative and require
hardware validation before study use:

| Parameter | Default | Meaning |
| --- | ---: | --- |
| `along_stiffness_n_per_m` | 10.0 | Goal stiffness along the path. |
| `along_damping_ns_per_m` | 2.0 | Along-path damping. |
| `fixture_stiffness_n_per_m` | 20.0 | Lateral virtual-fixture stiffness. |
| `fixture_damping_ns_per_m` | 2.0 | Lateral virtual-fixture damping. |
| `max_force_n` | 2.0 | Maximum task-plane force magnitude. Independent of docking. |
| `velocity_filter_alpha` | 0.25 | Timestamped velocity low-pass coefficient. |
| `cursor_timeout_s` | 0.2 | Input age that triggers zero force. |

## Adaptive interpretation

The adaptive state-feedback condition consumes four active coefficients from
`/estimation/K_h`. These coefficients describe an effective closed-loop
interaction model. They include participant action, robot assistance, device
dynamics, and noise; they are not measured human force or isolated human
stiffness.

Adaptation changes the along-path assistance gains. Virtual-fixture gains remain
the same in fixed and adaptive conditions so lateral guidance is not an
additional experimental difference.

## Optional docking

Docking is not required by state feedback and is disabled by default. When
enabled, it smoothly increases along-path stiffness near the endpoint. It does
not alter the lateral virtual fixture and is applied identically to fixed and
adaptive conditions.

```bash
ros2 launch haply_study_gui study_gui.launch.py \
  docking_enabled:=true
```

This activates the defaults `docking_start_percent=85`,
`docking_stiffness_scale=2.0`, and `docking_max_cross_track_m=0.02`.
The independent global force limit remains `max_force_n=2.0` whether docking
is enabled or disabled.

Docking progress is the directed projection onto the start-to-end path, not
Euclidean distance from the start. It activates only when cross-track error is
within `docking_max_cross_track_m`. There is no latch: retreating below the
threshold reduces the docking scale normally.

## Published interfaces

- `/control/U_a` (`geometry_msgs/Vector3`): task-frame assistant force in N for
  state feedback. The legacy MPC executable retains its historical semantics.
- `/control/K_a` (`std_msgs/String`): retained JSON containing physical gains,
  adaptation scale, docking configuration, and estimator interpretation.
- `/haply_target` (`haply_msgs/HaplyControl`): force command mapped to device X/Z.
- `/study_controller_ready` (`std_msgs/Bool`): retained task-readiness heartbeat.

Invalid task geometry, invalid timestamps, non-finite inputs, stale cursor data,
and trial stop all result in a zero-force command from the state-feedback node.

## Validation

Run the focused state-feedback tests:

```bash
pytest -q src/control_node/test/test_virtual_fixture_state_feedback.py
```

For live visualization, the debug entry points now include the production stack
and add only `test_control_node_output` plus debug logging:

```bash
# Production mouse stack + controller-output visualizer
ros2 launch control_node mouse_control_debug_launch.py

# Production hardware stack/readiness gate + controller-output visualizer
ros2 launch control_node haply_control_debug_launch.py
```

These wrappers were originally introduced in commit `fc6ba27` (2026-07-19).
They remain useful for the extra visualization window, but no longer duplicate
production Scenario, Mapper, GUI, Estimator, Logger, or controller parameters.
Both default to State Feedback; pass `controller:=mpc` only when explicitly
testing the legacy MPC path. The hardware wrapper requires the Haply Inverse
Service and device, while the mouse wrapper does not.
