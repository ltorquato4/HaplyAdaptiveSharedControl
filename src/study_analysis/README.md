# Study analysis

This package provides two complementary forms of evaluation:

- `analyze_session` produces descriptive metrics and a PDF report from one
  logger session. It does not perform inferential statistics.
- `run_benchmark` evaluates the estimator against known synthetic coefficients
  and runs fixed/adaptive state feedback in the same deterministic mass-damped
  planar plant. The legacy MPC implementation is intentionally out of scope.

```bash
ros2 run study_analysis analyze_session \
  --input logs/<session-id> \
  --output analysis_results/<session-id>

ros2 run study_analysis run_benchmark \
  --output analysis_results/benchmark \
  --seed 20260721
```

For current-format logs recorded before session metadata was added, pass both
`--controller-family` and `--input-source`. CSVs with legacy component columns
such as `u_h_x` or `U_a_x` are rejected explicitly.

The task frame uses cursor X/Y. Physical Haply diagnostics use device X/Z,
because the hardware mapper maps device Z onto task Y. Cross-track error is the
point-to-line distance in the task frame. Along-path progress is the scalar
projection onto the start-to-end segment; temporal ordering is retained, so
backtracking and overshoot remain visible.

For schema-2 logs, attempt duration and Logger-gap diagnostics use the
monotonic clock. Cursor velocity uses unique `StudyCursor` source timestamps;
repeated Logger rows therefore cannot create artificial zero-time derivatives.
Data quality reports distinguish missed Logger cycles, monotonic scheduling
gaps, wall-clock steps, and Mapper/source-sample gaps. Schema-1 logs continue
to use their original ROS wall timestamps.

State-feedback saturation is evaluated from the Cartesian force-vector norm,
matching its `max_force_n` clamp. Legacy MPC saturation retains its historical
per-component constraint interpretation.

`u_h` is an estimator-derived interaction input, not measured human force.
Participant logs therefore support stability and descriptive comparisons but
do not provide ground truth for estimator accuracy. In assisted trials, cursor
acceleration also reflects controller action and unmodelled dynamics, so the
identified coefficients are effective closed-loop values. Use `run_benchmark`
for known-coefficient validation.
