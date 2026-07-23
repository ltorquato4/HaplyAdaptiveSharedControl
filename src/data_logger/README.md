# Data logger

The logger waits for matching retained `StudySession` and `StudyTask` messages
before reporting ready. It creates
`logs/<participant-id>_<YYYY-MM-DD_HH-MM-SSZ>/` with:

- `session_manifest.json`: schema, resolved seed, controller/input metadata,
  pseudonymous participant ID, UUID session ID, estimator lifecycle policy,
  and complete task schedule;
- `trial_<trial-id>_attempt_<attempt-id>.csv`: 100 Hz sample data;
- `trial_attempts.csv`: retry-aware timing, final outcome, and reason.

The directory timestamp is UTC (`Z`). If the same participant code is opened
twice within one second, Logger adds a numeric suffix instead of reusing a
directory. Participant ID is also written into every trial row and attempt
summary so combined analyses do not depend on parsing directory names.
Participant codes are assigned centrally and passed to production launches so
they remain unique across study computers. The ordinary mouse launch uses
`P00`, while controller debug wrappers use `DEBUG_MOUSE` or `DEBUG_HAPLY`.

Controller parameters are retained across the sample reset at the start of an
attempt, ensuring the first row can contain the task's initial `K_a`. Retries of
the same trial ID receive increasing attempt IDs and never overwrite prior data.

Recording is driven exclusively by the matching retained `StudyTrialState`:
`RUNNING` starts an attempt, `DWELL` keeps it active, and `COMPLETED` or
`ABORTED` finalizes its outcome. This prevents an unscoped Boolean from opening
or closing the wrong trial. The `study_running` CSV column remains as a schema
field and is true for rows written during either active state; it is no longer a
lifecycle input.

Every sample row contains three timing views: ROS wall time (`timestamp`),
process steady-clock time (`monotonic_timestamp`), and the timestamp of the
latest typed Mapper sample (`cursor_timestamp`). `missed_cycle_count` records
cumulative 100 Hz Logger deadlines that were missed, while
`cursor_sample_sequence` distinguishes new source samples from repeated rows.
This lets analysis separate wall-clock corrections, Logger scheduling stalls,
and missing Mapper input.

Analyze a completed directory with:

```bash
ros2 run study_analysis analyze_session \
  --input logs/<session-folder>
```

Without `--output`, results are written to
`analysis_results/<session-folder>/`.
