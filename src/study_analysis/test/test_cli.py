import json

import pandas as pd
from study_analysis.cli import run


def test_analysis_cli_writes_tables_and_pdf(tmp_path):
    input_dir = tmp_path / "input"
    output_dir = tmp_path / "output"
    input_dir.mkdir()
    rows = []
    for index in range(3):
        rows.append(
            {
                "timestamp": 10.0 + index * 0.01,
                "session_id": "session",
                "trial_id": 0,
                "attempt_id": 1,
                "study_phase": "normal",
                "study_controller_mode": "fixed",
                "start_x": 0.0,
                "start_y": 0.0,
                "end_x": 1.0,
                "end_y": 0.0,
                "cursor_x": index / 2,
                "cursor_y": 0.0,
                "haply_vel_x": 0.1,
                "haply_vel_z": 0.0,
                "max_control_amplitude": 10.0,
                "u_h": json.dumps([1.0, 0.0, 0.0]),
                "u_a": json.dumps([0.5, 0.0, 0.0]),
                "K_h": json.dumps([2.0, -0.4, 0, 0, 0, 0, 1.5, -0.25]),
                "K_a": json.dumps(
                    {
                        "weight_comfort": 1.0,
                        "weight_trajectory": 2.0,
                        "weight_goal": 3.0,
                    }
                ),
            }
        )
    pd.DataFrame(rows).to_csv(input_dir / "trial_000000_attempt_001.csv", index=False)

    metrics = run(
        input_dir,
        output_dir,
        controller_family="mpc",
        input_source="mouse",
    )

    assert len(metrics) == 1
    for filename in (
        "trial_metrics.csv",
        "condition_summary.csv",
        "data_quality.csv",
        "analysis_report.pdf",
    ):
        path = output_dir / filename
        assert path.exists()
        assert path.stat().st_size > 0
