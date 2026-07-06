#!/usr/bin/env python3
"""Emit simulator-side EKF/GPS/wind logs for landing-dispersion analysis.

The emitted estimate log uses the documented CSV aliases consumed by
export_landing_telemetry_csv.py.  Truth is written separately so the export path
can exercise the same timestamp join that a real simulation run should use.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path
from typing import Dict, Optional, Sequence

try:
    from . import landing_dispersion_envelope as lde
except ImportError:
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import landing_dispersion_envelope as lde


ESTIMATE_FIELDS = [
    "time_s",
    "pos_x_m",
    "pos_y_m",
    "altitude_m",
    "cov_xx_m2",
    "cov_yy_m2",
    "cov_zz_m2",
    "wx_mps",
    "wy_mps",
    "wz_mps",
    "sigma_wind_norm_mps",
    "gps_pos_x_m",
    "gps_pos_y_m",
    "gps_altitude_m",
    "gps_update_used",
    "gps_measurement_rejected",
    "gps_position_innovation_m",
    "gps_velocity_innovation_mps",
    "gps_position_residual_m",
    "classification",
    "audit_rationale",
]

TRUTH_FIELDS = [
    "time_s",
    "true_x_m",
    "true_y_m",
    "true_z_m",
    "true_wind_x_mps",
    "true_wind_y_mps",
    "true_wind_z_mps",
]


def fmt(value: object) -> object:
    if value is None:
        return ""
    if isinstance(value, float):
        return f"{value:.9g}"
    return value


def estimate_row(rec: lde.TelemetryRecord) -> Dict[str, object]:
    # The simulator emits diagonal covariance because that is the most common
    # EKF export contract.  Horizontal sigma is split evenly across X/Y for this
    # deterministic example; real logs should export their actual covariance.
    sigma_xy = rec.sigma_h_m / (2.0 ** 0.5)
    return {
        "time_s": fmt(rec.t_s),
        "pos_x_m": fmt(rec.x_m),
        "pos_y_m": fmt(rec.y_m),
        "altitude_m": fmt(rec.z_m),
        "cov_xx_m2": fmt(sigma_xy * sigma_xy),
        "cov_yy_m2": fmt(sigma_xy * sigma_xy),
        "cov_zz_m2": fmt(rec.sigma_v_m * rec.sigma_v_m),
        "wx_mps": fmt(rec.wind_x_mps),
        "wy_mps": fmt(rec.wind_y_mps),
        "wz_mps": fmt(rec.wind_z_mps),
        "sigma_wind_norm_mps": fmt(rec.wind_sigma_norm_mps),
        "gps_pos_x_m": fmt(rec.gps_x_m),
        "gps_pos_y_m": fmt(rec.gps_y_m),
        "gps_altitude_m": fmt(rec.gps_z_m),
        "gps_update_used": 1 if rec.gps_used else 0,
        "gps_measurement_rejected": 1 if rec.gps_rejected else 0,
        "gps_position_innovation_m": fmt(rec.gps_pos_innovation_m),
        "gps_velocity_innovation_mps": fmt(rec.gps_vel_innovation_mps),
        "gps_position_residual_m": fmt(rec.gps_residual_m),
        "classification": rec.audit_label,
        "audit_rationale": rec.rationale,
    }


def truth_row(rec: lde.TelemetryRecord) -> Dict[str, object]:
    return {
        "time_s": fmt(rec.t_s),
        "true_x_m": fmt(rec.truth_x_m),
        "true_y_m": fmt(rec.truth_y_m),
        "true_z_m": fmt(rec.truth_z_m),
        "true_wind_x_mps": fmt(rec.truth_wind_x_mps),
        "true_wind_y_mps": fmt(rec.truth_wind_y_mps),
        "true_wind_z_mps": fmt(rec.truth_wind_z_mps),
    }


def write_table(path: Path, rows: Sequence[Dict[str, object]], fields: Sequence[str], output_format: str) -> None:
    lde.ensure_parent(path)
    if output_format == "jsonl":
        with path.open("w", encoding="utf-8") as f:
            for row in rows:
                f.write(json.dumps({field: row.get(field, "") for field in fields}, separators=(",", ":")) + "\n")
        return
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in fields})


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Emit deterministic simulator-side EKF/GPS/wind logs using documented landing-dispersion aliases.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--estimate-out", type=Path, default=Path(".codex_build/landing_dispersion_envelope/sim_estimate_log.csv"), help="estimate log output path")
    parser.add_argument("--truth-out", type=Path, default=Path(".codex_build/landing_dispersion_envelope/sim_truth_log.csv"), help="truth log output path")
    parser.add_argument("--manifest-out", type=Path, help="optional manifest JSON path")
    parser.add_argument("--sample-count", type=int, default=160, help="number of simulator samples")
    parser.add_argument("--format", choices=("csv", "jsonl"), default="csv", help="estimate/truth log format")
    return parser


def run(args: argparse.Namespace) -> int:
    if args.sample_count < 2:
        raise ValueError("--sample-count must be at least 2")
    records = lde.generate_synthetic_records(args.sample_count)
    estimate_rows = [estimate_row(rec) for rec in records]
    truth_rows = [truth_row(rec) for rec in records]
    write_table(args.estimate_out, estimate_rows, ESTIMATE_FIELDS, args.format)
    write_table(args.truth_out, truth_rows, TRUTH_FIELDS, args.format)

    manifest = {
        "estimate_out": str(args.estimate_out),
        "truth_out": str(args.truth_out),
        "format": args.format,
        "sample_count": args.sample_count,
        "estimate_fields": ESTIMATE_FIELDS,
        "truth_fields": TRUTH_FIELDS,
        "note": "Simulator-side host log emitter; it does not modify synthesizable HDL.",
    }
    if args.manifest_out:
        lde.ensure_parent(args.manifest_out)
        args.manifest_out.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")

    print(f"Wrote {args.estimate_out}")
    print(f"Wrote {args.truth_out}")
    if args.manifest_out:
        print(f"Wrote {args.manifest_out}")
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    try:
        return run(args)
    except ValueError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
