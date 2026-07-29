# Study analysis

This package provides two commands:

| Command | Use it for |
| --- | --- |
| `analyze_session` | Creating a PDF report and CSV summaries from one recorded participant session |
| `run_benchmark` | Checking the estimator and controllers in a deterministic simulation |

For normal experiment data, use `analyze_session`. The benchmark is an optional
software-validation tool and does not analyze participant data.

Run the commands from the repository root after building and sourcing the
workspace:

```bash
source install/setup.bash
```

## Analyze a participant session

Pass the directory created by the data logger:

```bash
ros2 run study_analysis analyze_session \
  --input logs/<session-folder>
```

For example:

```bash
ros2 run study_analysis analyze_session \
  --input logs/P03_2026-07-29_06-05-00Z
```

The results are written to `analysis_results/<session-folder>/` by default.
The command creates:

- `analysis_report.pdf`: plots and descriptive summaries;
- `trial_metrics.csv`: one row of metrics for each attempt;
- `condition_summary.csv`: metrics grouped by experimental condition; and
- `data_quality.csv`: missing, malformed, or timing-related data warnings.

To use a different output directory:

```bash
ros2 run study_analysis analyze_session \
  --input logs/<session-folder> \
  --output analysis_results/<result-name>
```

### Older session logs

Recent logs contain their controller family and input source. For current-format
logs recorded before that session metadata was added, provide both values:

```bash
ros2 run study_analysis analyze_session \
  --input logs/<session-folder> \
  --controller-family mpc \
  --input-source haply
```

Accepted controller families are `mpc` and `state_feedback`. Accepted input
sources are `haply` and `mouse`.

## Run the deterministic benchmark

The benchmark tests the estimator against known synthetic coefficients and
runs fixed and adaptive versions of State Feedback and MPC in the same
simulated planar system:

```bash
ros2 run study_analysis run_benchmark \
  --output analysis_results/benchmark \
  --seed 20260721
```

It creates:

- `benchmark_report.pdf`; and
- `benchmark_results.csv`.

The default simulation and acceptance settings are in
[`config/benchmark.yaml`](config/benchmark.yaml). To use another configuration
file, including the seed specified in that file:

```bash
ros2 run study_analysis run_benchmark \
  --output analysis_results/benchmark \
  --config path/to/benchmark.yaml
```

The `--seed` option overrides only the seed from the configuration file.
Controller parameters are always read from
`control_node/config/state_feedback.yaml` and `control_node/config/mpc.yaml`.

## How session metrics are calculated

- The task coordinate system uses cursor X/Y.
- Haply device diagnostics use X/Z because device Z maps to task Y.
- Cross-track error is the perpendicular distance from the cursor to the
  start-to-end line.
- Along-path progress is the cursor's projection onto that line.
- The sample order is preserved, so backtracking and overshooting remain
  visible.
- Attempt duration and logger-gap checks use the monotonic clock.
- Cursor velocity uses unique cursor-source timestamps, avoiding false
  zero-time samples from repeated logger rows.

## Data and interpretation limits

The session analysis is descriptive. It does not perform inferential statistics
or claim that one controller is scientifically superior.

The logged `u_h` value is an estimator output, not a direct measurement of
human force. Participant logs can therefore be used for stability and
descriptive condition comparisons, but not for estimator ground-truth
accuracy. Use `run_benchmark` when known estimator coefficients are required.

During assisted trials, controller action and unmodelled dynamics also affect
cursor acceleration. The identified coefficients should therefore be
interpreted as effective closed-loop values.

## Technical compatibility notes

- Schema-3 logs include `participant_id` in `trial_metrics.csv`. The
  `session_id` UUID remains the machine-readable session identity.
- Schema-2 logs report the participant as `unknown`.
- Schema-1 logs use their original ROS wall-clock timestamps.
- CSV files with legacy component columns such as `u_h_x` or `U_a_x` are
  rejected with an explicit error.
- Data-quality output distinguishes missed logger cycles, scheduling gaps,
  wall-clock changes, and mapper/source-sample gaps.
- State Feedback saturation uses the Cartesian force-vector norm. MPC
  saturation is checked per component.
- The MPC benchmark checks the raw ±10 command bound and the converted ±2 N
  per-axis force bound.
- Benchmark checks cover reproducibility, convergence, finite values, and
  configured bounds.
