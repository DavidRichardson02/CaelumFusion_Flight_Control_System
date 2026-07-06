#!/usr/bin/env python3
"""Run the landing-dispersion export and render regression.

Use --input to point at a real EKF/GPS/wind log.  If no input is provided, this
script emits deterministic simulator-side logs first and uses those as the
regression source.  That keeps the pipeline exercised without widening the
synthesizable visualization bundle.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import List, Optional, Sequence

try:
    from . import emit_landing_sim_log as emitter
    from . import export_landing_telemetry_csv as exporter
    from . import landing_dispersion_envelope as lde
except ImportError:
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import emit_landing_sim_log as emitter
    import export_landing_telemetry_csv as exporter
    import landing_dispersion_envelope as lde


DEFAULT_REAL_LOG_CANDIDATES = [
    Path("logs/landing_dispersion/ekf_gps_wind_log.csv"),
    Path("logs/landing_dispersion/ekf_gps_wind_log.jsonl"),
    Path(".codex_build/landing_dispersion_envelope/real_ekf_gps_wind_log.csv"),
    Path(".codex_build/landing_dispersion_envelope/real_ekf_gps_wind_log.jsonl"),
]


def first_existing(paths: Sequence[Path]) -> Optional[Path]:
    for path in paths:
        if path.exists():
            return path
    return None


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run landing-dispersion telemetry export and SVG render regression.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--input", type=Path, help="real EKF/GPS/wind CSV or JSONL log")
    parser.add_argument("--truth-input", type=Path, help="optional real truth CSV or JSONL log")
    parser.add_argument("--output-dir", type=Path, default=Path(".codex_build/landing_dispersion_envelope/regression"), help="regression output directory")
    parser.add_argument("--require-real-log", action="store_true", help="fail instead of generating simulator logs when no real log is found")
    parser.add_argument("--sample-count", type=int, default=160, help="simulator sample count when no real log is used")
    parser.add_argument("--truth-tolerance-s", type=float, default=0.050, help="truth join tolerance for nearest-time joins")
    parser.add_argument("--allow-missing-uncertainty", action="store_true", help="pass through to exporter")
    parser.add_argument("--allow-missing-wind", action="store_true", help="pass through to exporter")
    parser.add_argument("--allow-missing-wind-sigma", action="store_true", help="pass through to exporter")
    parser.add_argument("--allow-missing-gps", action="store_true", help="pass through to exporter")
    return parser


def run(args: argparse.Namespace) -> int:
    out_dir = args.output_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    source_mode = "real"
    estimate_input = args.input or first_existing(DEFAULT_REAL_LOG_CANDIDATES)
    truth_input = args.truth_input

    if estimate_input is None:
        if args.require_real_log:
            print("error: no real EKF/GPS/wind log found; pass --input or remove --require-real-log", file=sys.stderr)
            return 2
        source_mode = "simulated"
        estimate_input = out_dir / "sim_estimate_log.csv"
        truth_input = out_dir / "sim_truth_log.csv"
        emit_rc = emitter.main(
            [
                "--estimate-out",
                str(estimate_input),
                "--truth-out",
                str(truth_input),
                "--manifest-out",
                str(out_dir / "sim_log_manifest.json"),
                "--sample-count",
                str(args.sample_count),
            ]
        )
        if emit_rc != 0:
            return emit_rc

    canonical = out_dir / "landing_canonical.csv"
    export_manifest = out_dir / "export_manifest.json"
    figure = out_dir / "landing_dispersion.svg"
    summary = out_dir / "landing_summary.json"

    export_args: List[str] = [
        "--input",
        str(estimate_input),
        "--output",
        str(canonical),
        "--manifest-out",
        str(export_manifest),
        "--render-svg",
        str(figure),
        "--render-summary-out",
        str(summary),
        "--truth-tolerance-s",
        str(args.truth_tolerance_s),
    ]
    if truth_input:
        export_args.extend(["--truth-input", str(truth_input)])
    if args.allow_missing_uncertainty:
        export_args.append("--allow-missing-uncertainty")
    if args.allow_missing_wind:
        export_args.append("--allow-missing-wind")
    if args.allow_missing_wind_sigma:
        export_args.append("--allow-missing-wind-sigma")
    if args.allow_missing_gps:
        export_args.append("--allow-missing-gps")

    export_rc = exporter.main(export_args)
    if export_rc != 0:
        return export_rc

    summary_obj = json.loads(summary.read_text(encoding="utf-8"))
    regression_manifest = {
        "source_mode": source_mode,
        "estimate_input": str(estimate_input),
        "truth_input": str(truth_input) if truth_input else None,
        "canonical_csv": str(canonical),
        "figure_svg": str(figure),
        "summary_json": str(summary),
        "row_count": summary_obj.get("row_count"),
        "accepted_gps_samples": summary_obj.get("accepted_gps_samples"),
        "rejected_gps_samples": summary_obj.get("rejected_gps_samples"),
        "truth_position_rmse_m": summary_obj.get("truth_position_rmse_m"),
        "truth_wind_rmse_mps": summary_obj.get("truth_wind_rmse_mps"),
        "note": "Regression uses a real log when provided; otherwise it uses deterministic simulator-side logs.",
    }
    manifest_path = out_dir / "regression_manifest.json"
    manifest_path.write_text(json.dumps(regression_manifest, indent=2) + "\n", encoding="utf-8")

    print(f"Wrote {manifest_path}")
    print(f"Regression source mode: {source_mode}")
    print(f"Figure: {figure}")
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    if args.sample_count < 2:
        parser.error("--sample-count must be at least 2")
    if args.truth_tolerance_s < 0.0:
        parser.error("--truth-tolerance-s must be non-negative")
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
