#!/usr/bin/env python3
"""Normalize EKF/GPS/wind logs into the landing-dispersion CSV schema.

This is the separate host export path for the CaelumFusion landing-dispersion
diagnostic.  It consumes real host-side simulation, EKF, or telemetry logs in
CSV/JSONL form, maps documented aliases into the canonical renderer schema, and
optionally joins truth data by timestamp or row index.  It deliberately does not
touch synthesizable HDL or the VGA visualization bundle.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

try:
    from . import landing_dispersion_envelope as lde
except ImportError:
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import landing_dispersion_envelope as lde


RawRow = Dict[str, object]


def flatten_dict(obj: Dict[str, object], prefix: str = "") -> RawRow:
    flat: RawRow = {}
    for key, value in obj.items():
        name = f"{prefix}.{key}" if prefix else str(key)
        if isinstance(value, dict):
            flat.update(flatten_dict(value, name))
        else:
            flat[name] = value
            flat[str(key)] = value
    return flat


def normalize_keyed_row(row: RawRow) -> Dict[str, object]:
    return {str(key).strip().lower(): value for key, value in row.items()}


def read_rows(path: Path) -> List[Dict[str, object]]:
    suffix = path.suffix.lower()
    rows: List[Dict[str, object]] = []
    if suffix in (".jsonl", ".ndjson"):
        with path.open("r", encoding="utf-8-sig") as f:
            for line_no, line in enumerate(f, start=1):
                line = line.strip()
                if not line:
                    continue
                try:
                    parsed = json.loads(line)
                except json.JSONDecodeError as exc:
                    raise ValueError(f"{path}:{line_no}: invalid JSONL record: {exc}") from exc
                if not isinstance(parsed, dict):
                    raise ValueError(f"{path}:{line_no}: JSONL record must be an object")
                rows.append(normalize_keyed_row(flatten_dict(parsed)))
    else:
        with path.open("r", newline="", encoding="utf-8-sig") as f:
            reader = csv.DictReader(f)
            if not reader.fieldnames:
                raise ValueError(f"{path} has no CSV header")
            for row in reader:
                rows.append(normalize_keyed_row(row))
    if not rows:
        raise ValueError(f"{path} contains no records")
    return rows


def value_for(row: Dict[str, object], field: str) -> Optional[object]:
    for alias in lde.ALIASES[field]:
        key = alias.lower()
        if key in row and row[key] not in ("", None):
            return row[key]
    return None


def float_for(row: Dict[str, object], field: str) -> Optional[float]:
    value = value_for(row, field)
    if value in ("", None):
        return None
    try:
        return float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"field {field} has non-numeric value {value!r}") from exc


def text_for(row: Dict[str, object], field: str) -> str:
    value = value_for(row, field)
    return "" if value in ("", None) else str(value)


def bool_for(row: Dict[str, object], field: str) -> str:
    value = value_for(row, field)
    if value in ("", None):
        return ""
    if isinstance(value, bool):
        return "1" if value else "0"
    return "1" if str(value).strip().lower() in ("1", "true", "yes", "y", "used", "accepted", "accept") else "0"


def present(row: Dict[str, object], field: str) -> bool:
    return value_for(row, field) not in ("", None)


def fmt(value: Optional[float]) -> str:
    if value is None:
        return ""
    if math.isfinite(value):
        return f"{value:.9g}"
    return ""


def sigma_values(row: Dict[str, object]) -> Tuple[str, str, str, bool]:
    sigma_h = float_for(row, "sigma_h_m")
    sigma_v = float_for(row, "sigma_z_m")
    sigma_norm = float_for(row, "sigma_pos_norm_m")

    sx = float_for(row, "sigma_x_m")
    sy = float_for(row, "sigma_y_m")
    sz = float_for(row, "sigma_z_m")
    cov_xx = float_for(row, "cov_xx_m2")
    cov_yy = float_for(row, "cov_yy_m2")
    cov_zz = float_for(row, "cov_zz_m2")

    if sx is None and cov_xx is not None:
        sx = math.sqrt(max(0.0, cov_xx))
    if sy is None and cov_yy is not None:
        sy = math.sqrt(max(0.0, cov_yy))
    if sz is None and cov_zz is not None:
        sz = math.sqrt(max(0.0, cov_zz))

    if sigma_h is None and (sx is not None or sy is not None):
        sigma_h = math.hypot(sx or 0.0, sy or 0.0)
    if sigma_v is None and sz is not None:
        sigma_v = sz
    if sigma_norm is None and (sx is not None or sy is not None or sigma_v is not None):
        sigma_norm = math.sqrt((sx or 0.0) ** 2 + (sy or 0.0) ** 2 + (sigma_v or 0.0) ** 2)

    complete = sigma_h is not None and sigma_v is not None and sigma_norm is not None
    return fmt(sigma_h), fmt(sigma_v), fmt(sigma_norm), complete


def wind_sigma_value(row: Dict[str, object]) -> Tuple[str, bool]:
    direct = float_for(row, "wind_sigma_norm_mps")
    if direct is not None:
        return fmt(direct), True
    sx = float_for(row, "wind_sigma_x_mps")
    sy = float_for(row, "wind_sigma_y_mps")
    sz = float_for(row, "wind_sigma_z_mps")
    if sx is None and sy is None and sz is None:
        return "", False
    return fmt(math.sqrt((sx or 0.0) ** 2 + (sy or 0.0) ** 2 + (sz or 0.0) ** 2)), True


def canonicalize_estimate(row: Dict[str, object], idx: int) -> Tuple[Dict[str, object], Dict[str, bool]]:
    sigma_h, sigma_v, sigma_norm, uncertainty_complete = sigma_values(row)
    wind_sigma, wind_sigma_present = wind_sigma_value(row)
    gps_present = present(row, "gps_x_m") or present(row, "gps_y_m") or present(row, "gps_z_m")
    gps_evidence = (
        gps_present
        or present(row, "gps_used")
        or present(row, "gps_rejected")
        or present(row, "gps_pos_innovation_m")
        or present(row, "gps_vel_innovation_mps")
        or present(row, "gps_residual_m")
    )
    wind_present = present(row, "wind_x_mps") and present(row, "wind_y_mps") and present(row, "wind_z_mps")

    out: Dict[str, object] = {field: "" for field in lde.CANONICAL_FIELDS}
    out["t_s"] = fmt(float_for(row, "t_s") if present(row, "t_s") else float(idx))
    out["x_m"] = fmt(float_for(row, "x_m"))
    out["y_m"] = fmt(float_for(row, "y_m"))
    out["z_m"] = fmt(float_for(row, "z_m"))
    out["sigma_h_m"] = sigma_h
    out["sigma_v_m"] = sigma_v
    out["sigma_pos_norm_m"] = sigma_norm
    out["wind_x_mps"] = fmt(float_for(row, "wind_x_mps"))
    out["wind_y_mps"] = fmt(float_for(row, "wind_y_mps"))
    out["wind_z_mps"] = fmt(float_for(row, "wind_z_mps"))
    out["wind_sigma_norm_mps"] = wind_sigma
    out["gps_x_m"] = fmt(float_for(row, "gps_x_m"))
    out["gps_y_m"] = fmt(float_for(row, "gps_y_m"))
    out["gps_z_m"] = fmt(float_for(row, "gps_z_m"))
    out["gps_used"] = bool_for(row, "gps_used")
    out["gps_rejected"] = bool_for(row, "gps_rejected")
    out["gps_pos_innovation_m"] = fmt(float_for(row, "gps_pos_innovation_m"))
    out["gps_vel_innovation_mps"] = fmt(float_for(row, "gps_vel_innovation_mps"))
    out["gps_residual_m"] = fmt(float_for(row, "gps_residual_m"))
    out["truth_x_m"] = fmt(float_for(row, "truth_x_m"))
    out["truth_y_m"] = fmt(float_for(row, "truth_y_m"))
    out["truth_z_m"] = fmt(float_for(row, "truth_z_m"))
    out["truth_wind_x_mps"] = fmt(float_for(row, "truth_wind_x_mps"))
    out["truth_wind_y_mps"] = fmt(float_for(row, "truth_wind_y_mps"))
    out["truth_wind_z_mps"] = fmt(float_for(row, "truth_wind_z_mps"))
    out["audit_label"] = text_for(row, "audit_label")
    out["rationale"] = text_for(row, "rationale")

    if gps_present and out["gps_used"] == "" and out["gps_rejected"] == "":
        out["gps_used"] = "1"

    status = {
        "position_present": bool(out["x_m"] and out["y_m"] and out["z_m"]),
        "uncertainty_complete": uncertainty_complete,
        "wind_present": wind_present,
        "wind_sigma_present": wind_sigma_present,
        "gps_evidence": gps_evidence,
    }
    return out, status


def canonicalize_truth(row: Dict[str, object]) -> Dict[str, object]:
    out: Dict[str, object] = {}
    for field in (
        "t_s",
        "truth_x_m",
        "truth_y_m",
        "truth_z_m",
        "truth_wind_x_mps",
        "truth_wind_y_mps",
        "truth_wind_z_mps",
    ):
        out[field] = fmt(float_for(row, field))
    return out


def merge_truth_by_index(canonical: List[Dict[str, object]], truth_rows: Sequence[Dict[str, object]]) -> int:
    count = min(len(canonical), len(truth_rows))
    for idx in range(count):
        truth = canonicalize_truth(truth_rows[idx])
        for field in truth:
            if field != "t_s" and truth[field] != "":
                canonical[idx][field] = truth[field]
    return count


def merge_truth_by_time(
    canonical: List[Dict[str, object]],
    truth_rows: Sequence[Dict[str, object]],
    tolerance_s: float,
) -> int:
    truth = [canonicalize_truth(row) for row in truth_rows]
    truth_with_time = [(float(row["t_s"]), row) for row in truth if row.get("t_s") not in ("", None)]
    if not truth_with_time:
        raise ValueError("truth join by time requires t_s/time_s in the truth input")

    joined = 0
    start = 0
    for row in canonical:
        if row.get("t_s") in ("", None):
            continue
        t = float(row["t_s"])
        best_idx = None
        best_dt = None
        for idx in range(start, len(truth_with_time)):
            dt = abs(truth_with_time[idx][0] - t)
            if best_dt is None or dt < best_dt:
                best_idx = idx
                best_dt = dt
            if truth_with_time[idx][0] > t + tolerance_s:
                break
        if best_idx is not None and best_dt is not None and best_dt <= tolerance_s:
            for field, value in truth_with_time[best_idx][1].items():
                if field != "t_s" and value != "":
                    row[field] = value
            joined += 1
            start = best_idx
    return joined


def validate_export(statuses: Sequence[Dict[str, bool]], args: argparse.Namespace) -> Dict[str, object]:
    row_count = len(statuses)
    missing_position = sum(1 for s in statuses if not s["position_present"])
    missing_uncertainty = sum(1 for s in statuses if not s["uncertainty_complete"])
    missing_wind = sum(1 for s in statuses if not s["wind_present"])
    missing_wind_sigma = sum(1 for s in statuses if not s["wind_sigma_present"])
    missing_gps = sum(1 for s in statuses if not s["gps_evidence"])
    gps_evidence_rows = row_count - missing_gps

    errors = []
    if missing_position:
        errors.append(f"{missing_position}/{row_count} rows are missing estimated x/y/z position")
    if missing_uncertainty and not args.allow_missing_uncertainty:
        errors.append(f"{missing_uncertainty}/{row_count} rows are missing position sigma or covariance")
    if missing_wind and not args.allow_missing_wind:
        errors.append(f"{missing_wind}/{row_count} rows are missing wind_x/y/z estimates")
    if missing_wind_sigma and not args.allow_missing_wind_sigma:
        errors.append(f"{missing_wind_sigma}/{row_count} rows are missing wind sigma or wind covariance")
    if gps_evidence_rows == 0 and not args.allow_missing_gps:
        errors.append("no GPS evidence fields were found in the run")
    if errors:
        raise ValueError("; ".join(errors))

    return {
        "row_count": row_count,
        "missing_position_rows": missing_position,
        "missing_uncertainty_rows": missing_uncertainty,
        "missing_wind_rows": missing_wind,
        "missing_wind_sigma_rows": missing_wind_sigma,
        "missing_gps_evidence_rows": missing_gps,
        "gps_evidence_rows": gps_evidence_rows,
    }


def write_canonical_csv(rows: Sequence[Dict[str, object]], path: Path) -> None:
    lde.ensure_parent(path)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=lde.CANONICAL_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in lde.CANONICAL_FIELDS})


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Export real EKF/GPS/wind telemetry into the landing-dispersion CSV schema.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--input", required=True, type=Path, help="estimate telemetry input, CSV or JSONL")
    parser.add_argument("--output", required=True, type=Path, help="canonical CSV output path")
    parser.add_argument("--truth-input", type=Path, help="optional truth CSV/JSONL with truth_x/y/z and truth_wind fields")
    parser.add_argument("--truth-join", choices=("time", "row-index"), default="time", help="truth merge mode")
    parser.add_argument("--truth-tolerance-s", type=float, default=0.050, help="nearest-time truth join tolerance")
    parser.add_argument("--manifest-out", type=Path, help="optional export manifest JSON")
    parser.add_argument("--render-svg", type=Path, help="optional SVG output rendered from the exported CSV")
    parser.add_argument("--render-summary-out", type=Path, help="optional renderer summary JSON path")
    parser.add_argument("--allow-missing-uncertainty", action="store_true", help="allow rows without position sigma/covariance")
    parser.add_argument("--allow-missing-wind", action="store_true", help="allow rows without wind_x/y/z")
    parser.add_argument("--allow-missing-wind-sigma", action="store_true", help="allow rows without wind sigma/covariance")
    parser.add_argument("--allow-missing-gps", action="store_true", help="allow rows without GPS evidence")
    return parser


def run(args: argparse.Namespace) -> int:
    raw_rows = read_rows(args.input)
    canonical: List[Dict[str, object]] = []
    statuses: List[Dict[str, bool]] = []
    for idx, row in enumerate(raw_rows):
        out, status = canonicalize_estimate(row, idx)
        canonical.append(out)
        statuses.append(status)

    truth_joined = 0
    if args.truth_input:
        truth_rows = read_rows(args.truth_input)
        if args.truth_join == "row-index":
            truth_joined = merge_truth_by_index(canonical, truth_rows)
        else:
            truth_joined = merge_truth_by_time(canonical, truth_rows, args.truth_tolerance_s)

    validation = validate_export(statuses, args)
    write_canonical_csv(canonical, args.output)

    manifest = {
        "input": str(args.input),
        "output": str(args.output),
        "truth_input": str(args.truth_input) if args.truth_input else None,
        "truth_join": args.truth_join if args.truth_input else None,
        "truth_rows_joined": truth_joined,
        "validation": validation,
        "canonical_fields": lde.CANONICAL_FIELDS,
        "note": "Host-side export only; no synthesizable HDL or VGA bundle fields are modified.",
    }
    if args.manifest_out:
        lde.ensure_parent(args.manifest_out)
        args.manifest_out.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")

    if args.render_svg:
        records = lde.read_telemetry_csv(args.output)
        summary = lde.summarize(records)
        lde.ensure_parent(args.render_svg)
        args.render_svg.write_text(lde.render_svg(records, lde.RenderConfig(), summary), encoding="utf-8")
        if args.render_summary_out:
            lde.ensure_parent(args.render_summary_out)
            args.render_summary_out.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")

    print(f"Wrote {args.output}")
    if args.manifest_out:
        print(f"Wrote {args.manifest_out}")
    if args.render_svg:
        print(f"Wrote {args.render_svg}")
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    if args.truth_tolerance_s < 0.0:
        parser.error("--truth-tolerance-s must be non-negative")
    try:
        return run(args)
    except ValueError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
