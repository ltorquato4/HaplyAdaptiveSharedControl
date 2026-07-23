# Study analysis

This package provides two complementary forms of evaluation:

- `analyze_session` produces descriptive metrics and a PDF report from one
  logger session. It does not perform inferential statistics.
- `run_benchmark` evaluates the estimator against known synthetic coefficients
  and runs fixed/adaptive State Feedback and MPC in the same deterministic
  mass-damped planar plant.

```bash
ros2 run study_analysis analyze_session \
  --input logs/<session-folder>

ros2 run study_analysis run_benchmark \
  --output analysis_results/benchmark \
  --seed 20260721
```

`analyze_session` automatically writes to
`analysis_results/<session-folder>/`. Pass `--output` only when a different
destination is needed. The generated directory contains `trial_metrics.csv`,
`condition_summary.csv`, `data_quality.csv`, and `analysis_report.pdf`.

`config/benchmark.yaml` is the single source of truth for simulation and
acceptance settings and is loaded automatically. `--config` selects an
alternative benchmark file, while `--seed` overrides only its seed. Controller
parameters are loaded directly from `control_node/config/state_feedback.yaml`
and `control_node/config/mpc.yaml`; they are not duplicated in analysis code or
benchmark configuration.

Schema-3 logs carry `participant_id` into `trial_metrics.csv`. The UUID
`session_id` remains the machine identity, while participant codes such as
`P03` support human-readable folders and future multi-participant aggregation.

For current-format logs recorded before session metadata was added, pass both
`--controller-family` and `--input-source`. CSVs with legacy component columns
such as `u_h_x` or `U_a_x` are rejected explicitly.

The task frame uses cursor X/Y. Physical Haply diagnostics use device X/Z,
because the hardware mapper maps device Z onto task Y. Cross-track error is the
point-to-line distance in the task frame. Along-path progress is the scalar
projection onto the start-to-end segment; temporal ordering is retained, so
backtracking and overshoot remain visible.

For schema-2 logs, `participant_id` is reported as `unknown`. Attempt duration
and Logger-gap diagnostics use the monotonic clock. Cursor velocity uses unique
`StudyCursor` source timestamps;
repeated Logger rows therefore cannot create artificial zero-time derivatives.
Data quality reports distinguish missed Logger cycles, monotonic scheduling
gaps, wall-clock steps, and Mapper/source-sample gaps. Schema-1 logs continue
to use their original ROS wall timestamps.

State-feedback saturation is evaluated from the Cartesian force-vector norm,
matching its `max_force_n` clamp. MPC saturation uses its per-component
constraint interpretation. The MPC benchmark applies its configured 0.2
conversion before the simulated plant and checks both the raw
±10 command bound and the converted ±2 N per-axis force bound. The benchmark
checks reproducibility, convergence, finiteness, and bounds; it does not claim
that either controller is scientifically superior.

`u_h` is an estimator-derived interaction input, not measured human force.
Participant logs therefore support stability and descriptive comparisons but
do not provide ground truth for estimator accuracy. In assisted trials, cursor
acceleration also reflects controller action and unmodelled dynamics, so the
identified coefficients are effective closed-loop values. Use `run_benchmark`
for known-coefficient validation.
