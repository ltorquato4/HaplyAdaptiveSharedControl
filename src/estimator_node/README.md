# Estimator node

The estimator identifies the four active coefficients in

```text
u_h = K_h [goal_error_x, velocity_x, goal_error_y, velocity_y]
```

using recursive least squares (RLS). The active entries in the flattened 2x4
matrix published on `/estimation/K_h` are indices `0`, `1`, `6`, and `7`.
`/estimation/u_h` is the input inferred from this model; it is not a direct
measurement of human force.

The current observation target is cursor acceleration. During assisted trials
that acceleration also contains controller action and unmodelled device or
participant dynamics. Consequently, `K_h` is an effective closed-loop
estimate, not an isolated biomechanical human gain.

The production node learns only while a trial is running. It consumes each
valid, timestamped `/study_cursor` sample once, keeps learned coefficients
between trials in one session, and resets the complete RLS state when a new
`/study_session` ID arrives. Readiness therefore requires both the retained
session definition and the active task. Identified `/study_trial_state`
transitions are authoritative: `RUNNING` and `DWELL` enable learning, while
`READY`, `COMPLETED`, `ABORTED`, and `SESSION_FINISHED` stop it. A retry resets
only timestamp/derivative history, not the learned session coefficients.

Learning needs motion that excites both position-error and velocity terms. A
stationary cursor, a very short trial, repeated timestamps, or motion along
only one axis can leave some coefficients nearly unchanged. Inspect the live
inputs before interpreting a flat coefficient trace:

```bash
ros2 topic hz /study_cursor
ros2 topic echo /study_trial_state
ros2 topic echo /estimation/K_h
```

For a ground-truth check independent of hardware, run the deterministic
benchmark. It tests RLS with known coefficients in noiseless and seeded-noise
cases, then exercises fixed and adaptive state-feedback and MPC controllers:

```bash
ros2 run study_analysis run_benchmark \
  --output analysis_results/benchmark \
  --seed 20260721
```

Participant logs do not contain estimator ground truth, so session analysis is
descriptive. Use the benchmark to establish coefficient-recovery accuracy and
the session report to inspect runtime behavior.
