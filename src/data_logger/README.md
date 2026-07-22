# Data logger

The logger waits for matching retained `StudySession` and `StudyTask` messages
before reporting ready. It creates `logs/<session-id>/` with:

- `session_manifest.json`: schema, resolved seed, controller/input metadata,
  estimator lifecycle policy, and complete task schedule;
- `trial_<trial-id>_attempt_<attempt-id>.csv`: 100 Hz sample data;
- `trial_attempts.csv`: retry-aware timing, final outcome, and reason.

Controller parameters are retained across the sample reset at the start of an
attempt, ensuring the first row can contain the task's initial `K_a`. Retries of
the same trial ID receive increasing attempt IDs and never overwrite prior data.

Recording is driven exclusively by the matching retained `StudyTrialState`:
`RUNNING` starts an attempt, `DWELL` keeps it active, and `COMPLETED` or
`ABORTED` finalizes its outcome. This prevents an unscoped Boolean from opening
or closing the wrong trial. The `study_running` CSV column remains as a schema
field and is true for rows written during either active state; it is no longer a
subscription to `/study_is_running`.

Every sample row contains three timing views: ROS wall time (`timestamp`),
process steady-clock time (`monotonic_timestamp`), and the timestamp of the
latest typed Mapper sample (`cursor_timestamp`). `missed_cycle_count` records
cumulative 100 Hz Logger deadlines that were missed, while
`cursor_sample_sequence` distinguishes new source samples from repeated rows.
This lets downstream processing separate wall-clock corrections, Logger
scheduling stalls, and missing Mapper input.
