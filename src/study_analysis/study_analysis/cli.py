"""Command-line entry point for recorded-session analysis."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import pandas as pd

from .loader import SchemaError, load_session
from .metrics import calculate_trial_metrics, summarize_conditions
from .report import generate_report


def build_parser():
    parser = argparse.ArgumentParser(description="Analyze one Haply study session")
    parser.add_argument("--input", required=True, help="Session log directory")
    parser.add_argument("--output", required=True, help="New analysis result directory")
    parser.add_argument("--controller-family", choices=("mpc", "state_feedback"))
    parser.add_argument("--input-source", choices=("mouse", "haply"))
    return parser


def run(input_dir, output_dir, controller_family=None, input_source=None):
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    attempts, quality, _manifest = load_session(
        input_dir, controller_family=controller_family, input_source=input_source
    )
    rows = []
    enriched = []
    for attempt in attempts:
        metrics, frame = calculate_trial_metrics(attempt)
        rows.append(metrics)
        enriched.append(frame)
    trial_metrics = pd.DataFrame(rows)
    condition_summary = summarize_conditions(trial_metrics)
    trial_metrics.to_csv(output / "trial_metrics.csv", index=False)
    condition_summary.to_csv(output / "condition_summary.csv", index=False)
    pd.DataFrame(quality, columns=["file", "severity", "issue", "value"]).to_csv(
        output / "data_quality.csv", index=False
    )
    generate_report(
        output / "analysis_report.pdf", attempts, enriched, trial_metrics, quality
    )
    return trial_metrics


def main(argv=None):
    args = build_parser().parse_args(argv)
    try:
        run(
            args.input,
            args.output,
            controller_family=args.controller_family,
            input_source=args.input_source,
        )
    except SchemaError as exc:
        print(f"analysis failed: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
