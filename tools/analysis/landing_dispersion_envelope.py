#!/usr/bin/env python3
"""3D Wind-Driven Landing Dispersion Envelope diagnostic.

This is a host-side analysis tool for the CaelumFusion_Flight_Control workflow.
It intentionally does not import or modify synthesizable HDL.  The tool accepts
CSV telemetry when available and can generate a deterministic synthetic descent
run for validation and documentation.

The rendered output is an SVG figure containing:
  - a projected 3D trajectory with a covariance/sigma-derived dispersion tube,
  - GPS accepted/rejected evidence,
  - downsampled wind vectors,
  - wind and uncertainty time-series,
  - GPS fusion evidence indicators,
  - audit/classification labels, and
  - a run-level diagnostic summary.

The SVG renderer is implemented with the Python standard library so it remains
usable in a clean FPGA checkout without installing plotting packages into the
Vivado project environment.
"""

from __future__ import annotations

import argparse
import csv
import html
import json
import math
import os
import statistics
import sys
import webbrowser
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


TITLE = "3D Wind-Driven Landing Dispersion Envelope with Position Uncertainty Bounds"


@dataclass(frozen=True)
class TelemetryRecord:
    """Canonical host-side telemetry sample for the dispersion diagnostic."""

    t_s: float
    x_m: float
    y_m: float
    z_m: float
    sigma_h_m: float
    sigma_v_m: float
    sigma_pos_norm_m: float
    wind_x_mps: float
    wind_y_mps: float
    wind_z_mps: float
    wind_sigma_norm_mps: float
    gps_x_m: Optional[float] = None
    gps_y_m: Optional[float] = None
    gps_z_m: Optional[float] = None
    gps_used: bool = False
    gps_rejected: bool = False
    gps_pos_innovation_m: Optional[float] = None
    gps_vel_innovation_mps: Optional[float] = None
    gps_residual_m: Optional[float] = None
    truth_x_m: Optional[float] = None
    truth_y_m: Optional[float] = None
    truth_z_m: Optional[float] = None
    truth_wind_x_mps: Optional[float] = None
    truth_wind_y_mps: Optional[float] = None
    truth_wind_z_mps: Optional[float] = None
    audit_label: str = "state incomplete"
    rationale: str = ""


@dataclass(frozen=True)
class RenderConfig:
    sigma_scale: float = 2.0
    wind_vector_scale: float = 10.0
    gps_downsample: int = 1
    envelope_downsample: int = 4
    wind_downsample: int = 10
    theme: str = "dark"
    width_px: int = 1600
    height_px: int = 1100


ALIASES: Dict[str, Sequence[str]] = {
    "t_s": ("t_s", "time_s", "time", "t", "timestamp_s", "seconds"),
    "x_m": ("x_m", "pos_x_m", "position_x_m", "est_x_m", "north_m", "east_m"),
    "y_m": ("y_m", "pos_y_m", "position_y_m", "est_y_m", "east_m", "crossrange_m"),
    "z_m": ("z_m", "pos_z_m", "position_z_m", "est_z_m", "altitude_m", "alt_m"),
    "sigma_x_m": ("sigma_x_m", "pos_sigma_x_m", "position_sigma_x_m"),
    "sigma_y_m": ("sigma_y_m", "pos_sigma_y_m", "position_sigma_y_m"),
    "sigma_z_m": ("sigma_z_m", "pos_sigma_z_m", "position_sigma_z_m", "sigma_v_m"),
    "sigma_h_m": ("sigma_h_m", "horizontal_sigma_m", "pos_sigma_h_m"),
    "sigma_pos_norm_m": (
        "sigma_pos_norm_m",
        "position_sigma_norm_m",
        "pos_sigma_norm_m",
        "sigma_total_m",
    ),
    "cov_xx_m2": ("cov_xx_m2", "p_xx_m2", "p_xx", "position_cov_xx_m2"),
    "cov_yy_m2": ("cov_yy_m2", "p_yy_m2", "p_yy", "position_cov_yy_m2"),
    "cov_zz_m2": ("cov_zz_m2", "p_zz_m2", "p_zz", "position_cov_zz_m2"),
    "wind_x_mps": ("wind_x_mps", "wx_mps", "wx", "wind_est_x_mps"),
    "wind_y_mps": ("wind_y_mps", "wy_mps", "wy", "wind_est_y_mps"),
    "wind_z_mps": ("wind_z_mps", "wz_mps", "wz", "wind_est_z_mps"),
    "wind_sigma_x_mps": ("wind_sigma_x_mps", "sigma_wx_mps", "wind_cov_xx_m2_s2"),
    "wind_sigma_y_mps": ("wind_sigma_y_mps", "sigma_wy_mps", "wind_cov_yy_m2_s2"),
    "wind_sigma_z_mps": ("wind_sigma_z_mps", "sigma_wz_mps", "wind_cov_zz_m2_s2"),
    "wind_sigma_norm_mps": ("wind_sigma_norm_mps", "wind_sigma_norm", "sigma_wind_norm_mps"),
    "gps_x_m": ("gps_x_m", "gps_pos_x_m", "meas_gps_x_m"),
    "gps_y_m": ("gps_y_m", "gps_pos_y_m", "meas_gps_y_m"),
    "gps_z_m": ("gps_z_m", "gps_pos_z_m", "gps_altitude_m", "meas_gps_z_m"),
    "gps_used": ("gps_used", "gps_update_used", "gps_fused", "gps_accept"),
    "gps_rejected": ("gps_rejected", "gps_reject", "gps_measurement_rejected"),
    "gps_pos_innovation_m": (
        "gps_pos_innovation_m",
        "gps_position_innovation_m",
        "pos_innovation_m",
    ),
    "gps_vel_innovation_mps": (
        "gps_vel_innovation_mps",
        "gps_velocity_innovation_mps",
        "vel_innovation_mps",
    ),
    "gps_residual_m": ("gps_residual_m", "gps_position_residual_m", "gps_residual"),
    "truth_x_m": ("truth_x_m", "true_x_m"),
    "truth_y_m": ("truth_y_m", "true_y_m"),
    "truth_z_m": ("truth_z_m", "true_z_m", "truth_altitude_m"),
    "truth_wind_x_mps": ("truth_wind_x_mps", "true_wind_x_mps"),
    "truth_wind_y_mps": ("truth_wind_y_mps", "true_wind_y_mps"),
    "truth_wind_z_mps": ("truth_wind_z_mps", "true_wind_z_mps"),
    "audit_label": ("audit_label", "classification", "evidence_label", "nav_label"),
    "rationale": ("rationale", "audit_rationale", "classification_rationale"),
}

CANONICAL_FIELDS = [
    "t_s",
    "x_m",
    "y_m",
    "z_m",
    "sigma_h_m",
    "sigma_v_m",
    "sigma_pos_norm_m",
    "wind_x_mps",
    "wind_y_mps",
    "wind_z_mps",
    "wind_sigma_norm_mps",
    "gps_x_m",
    "gps_y_m",
    "gps_z_m",
    "gps_used",
    "gps_rejected",
    "gps_pos_innovation_m",
    "gps_vel_innovation_mps",
    "gps_residual_m",
    "truth_x_m",
    "truth_y_m",
    "truth_z_m",
    "truth_wind_x_mps",
    "truth_wind_y_mps",
    "truth_wind_z_mps",
    "audit_label",
    "rationale",
]


PALETTE = {
    "dark": {
        "bg": "#071016",
        "panel": "#0d1d26",
        "panel2": "#102935",
        "grid": "#24424d",
        "axis": "#7ea7b5",
        "text": "#e7f2f5",
        "muted": "#93aeb7",
        "trajectory": "#55d6ff",
        "envelope": "#20b7ff",
        "gps_used": "#51f08c",
        "gps_rejected": "#ff5a6b",
        "gps_raw": "#e5d15a",
        "wind": "#c9f76a",
        "wind_x": "#54d9ff",
        "wind_y": "#ffb84d",
        "wind_z": "#b084ff",
        "speed": "#f7f06a",
        "sigma_pos": "#ff6fb1",
        "sigma_h": "#70e0ff",
        "sigma_v": "#ffcf5c",
        "sigma_wind": "#8df28a",
        "warning": "#ffb000",
    },
    "light": {
        "bg": "#f8fbfc",
        "panel": "#ffffff",
        "panel2": "#eef6f8",
        "grid": "#c9d7dc",
        "axis": "#4c6c78",
        "text": "#13242c",
        "muted": "#5b7380",
        "trajectory": "#0077aa",
        "envelope": "#00a3d7",
        "gps_used": "#15883d",
        "gps_rejected": "#d5283d",
        "gps_raw": "#8f7a00",
        "wind": "#668f00",
        "wind_x": "#0077aa",
        "wind_y": "#c86e00",
        "wind_z": "#6b48c8",
        "speed": "#9b8500",
        "sigma_pos": "#bd2a72",
        "sigma_h": "#0087ad",
        "sigma_v": "#b76a00",
        "sigma_wind": "#308b2f",
        "warning": "#d98400",
    },
}


def _normalize_row(row: Dict[str, str]) -> Dict[str, str]:
    return {k.strip().lower(): (v.strip() if isinstance(v, str) else v) for k, v in row.items()}


def _first_value(row: Dict[str, str], name: str) -> Optional[str]:
    for alias in ALIASES[name]:
        if alias.lower() in row and row[alias.lower()] not in ("", None):
            return row[alias.lower()]
    return None


def _float(row: Dict[str, str], name: str, default: Optional[float] = None) -> Optional[float]:
    value = _first_value(row, name)
    if value is None:
        return default
    try:
        return float(value)
    except ValueError as exc:
        raise ValueError(f"column {name} has non-numeric value {value!r}") from exc


def _bool(row: Dict[str, str], name: str, default: bool = False) -> bool:
    value = _first_value(row, name)
    if value is None:
        return default
    return str(value).strip().lower() in ("1", "true", "yes", "y", "used", "accepted", "accept")


def _text(row: Dict[str, str], name: str, default: str = "") -> str:
    value = _first_value(row, name)
    return default if value is None else str(value)


def _safe_sqrt(value: float) -> float:
    return math.sqrt(max(0.0, value))


def _sigma_from_row(row: Dict[str, str]) -> Tuple[float, float, float]:
    sx = _float(row, "sigma_x_m")
    sy = _float(row, "sigma_y_m")
    sz = _float(row, "sigma_z_m")
    if sx is None:
        cov_xx = _float(row, "cov_xx_m2")
        sx = _safe_sqrt(cov_xx) if cov_xx is not None else None
    if sy is None:
        cov_yy = _float(row, "cov_yy_m2")
        sy = _safe_sqrt(cov_yy) if cov_yy is not None else None
    if sz is None:
        cov_zz = _float(row, "cov_zz_m2")
        sz = _safe_sqrt(cov_zz) if cov_zz is not None else None

    sigma_h = _float(row, "sigma_h_m")
    if sigma_h is None:
        sigma_h = math.hypot(sx or 0.0, sy or 0.0)
    sigma_v = sz if sz is not None else 0.0
    sigma_norm = _float(row, "sigma_pos_norm_m")
    if sigma_norm is None:
        sigma_norm = math.sqrt((sx or 0.0) ** 2 + (sy or 0.0) ** 2 + sigma_v**2)
    return sigma_h, sigma_v, sigma_norm


def _wind_sigma_from_row(row: Dict[str, str]) -> float:
    direct = _float(row, "wind_sigma_norm_mps")
    if direct is not None:
        return direct
    sx = _float(row, "wind_sigma_x_mps", 0.0) or 0.0
    sy = _float(row, "wind_sigma_y_mps", 0.0) or 0.0
    sz = _float(row, "wind_sigma_z_mps", 0.0) or 0.0
    return math.sqrt(sx * sx + sy * sy + sz * sz)


def _classify_record(
    gps_used: bool,
    gps_rejected: bool,
    sigma_h: float,
    sigma_v: float,
    wind_sigma: float,
    supplied: str,
) -> str:
    if supplied:
        return supplied
    if gps_rejected:
        return "GPS rejected"
    if gps_used:
        return "GPS update used"
    if sigma_h <= 0.0 and sigma_v <= 0.0:
        return "covariance incomplete"
    if sigma_h > 55.0 or sigma_v > 45.0:
        return "position uncertainty high"
    if wind_sigma > 3.0:
        return "wind uncertainty high"
    return "inertial only"


def read_telemetry_csv(path: Path) -> List[TelemetryRecord]:
    """Read CSV telemetry into the canonical host-side record list."""

    records: List[TelemetryRecord] = []
    with path.open("r", newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        if not reader.fieldnames:
            raise ValueError(f"{path} has no CSV header")

        for idx, raw_row in enumerate(reader):
            row = _normalize_row(raw_row)
            t_s = _float(row, "t_s", float(idx))

            x_m = _float(row, "x_m")
            y_m = _float(row, "y_m")
            z_m = _float(row, "z_m")
            if x_m is None or y_m is None or z_m is None:
                raise ValueError(
                    "CSV input must provide x/y/z position fields. "
                    "Accepted aliases include x_m,y_m,z_m,pos_x_m,pos_y_m,pos_z_m,altitude_m."
                )

            sigma_h, sigma_v, sigma_norm = _sigma_from_row(row)
            wind_sigma = _wind_sigma_from_row(row)
            gps_x = _float(row, "gps_x_m")
            gps_y = _float(row, "gps_y_m")
            gps_z = _float(row, "gps_z_m")
            gps_present = gps_x is not None or gps_y is not None or gps_z is not None
            gps_used = _bool(row, "gps_used", False)
            gps_rejected = _bool(row, "gps_rejected", False)
            if gps_present and not gps_used and not gps_rejected:
                gps_used = True

            supplied_label = _text(row, "audit_label", "")
            label = _classify_record(gps_used, gps_rejected, sigma_h, sigma_v, wind_sigma, supplied_label)

            records.append(
                TelemetryRecord(
                    t_s=t_s if t_s is not None else float(idx),
                    x_m=x_m,
                    y_m=y_m,
                    z_m=z_m,
                    sigma_h_m=sigma_h,
                    sigma_v_m=sigma_v,
                    sigma_pos_norm_m=sigma_norm,
                    wind_x_mps=_float(row, "wind_x_mps", 0.0) or 0.0,
                    wind_y_mps=_float(row, "wind_y_mps", 0.0) or 0.0,
                    wind_z_mps=_float(row, "wind_z_mps", 0.0) or 0.0,
                    wind_sigma_norm_mps=wind_sigma,
                    gps_x_m=gps_x,
                    gps_y_m=gps_y,
                    gps_z_m=gps_z,
                    gps_used=gps_used,
                    gps_rejected=gps_rejected,
                    gps_pos_innovation_m=_float(row, "gps_pos_innovation_m"),
                    gps_vel_innovation_mps=_float(row, "gps_vel_innovation_mps"),
                    gps_residual_m=_float(row, "gps_residual_m"),
                    truth_x_m=_float(row, "truth_x_m"),
                    truth_y_m=_float(row, "truth_y_m"),
                    truth_z_m=_float(row, "truth_z_m"),
                    truth_wind_x_mps=_float(row, "truth_wind_x_mps"),
                    truth_wind_y_mps=_float(row, "truth_wind_y_mps"),
                    truth_wind_z_mps=_float(row, "truth_wind_z_mps"),
                    audit_label=label,
                    rationale=_text(row, "rationale", ""),
                )
            )
    if not records:
        raise ValueError(f"{path} contains no telemetry rows")
    return records


def generate_synthetic_records(count: int = 160) -> List[TelemetryRecord]:
    """Generate deterministic descent data for demo and regression use."""

    records: List[TelemetryRecord] = []
    for i in range(count):
        t = i * 0.5
        progress = i / max(1, count - 1)
        wx = 2.2 + 0.9 * math.sin(t * 0.09)
        wy = -1.1 + 0.7 * math.cos(t * 0.07)
        wz = 0.25 * math.sin(t * 0.13)
        truth_wx = wx - 0.25 * math.sin(t * 0.04)
        truth_wy = wy + 0.20 * math.cos(t * 0.05)
        truth_wz = wz - 0.05 * math.sin(t * 0.08)

        x_truth = 4.8 * t + 18.0 * math.sin(t * 0.04)
        y_truth = 22.0 * math.sin(t * 0.08) + 0.65 * t * wy
        z_truth = max(0.0, 520.0 * (1.0 - progress) ** 1.25 + 8.0 * math.sin(t * 0.10))

        estimator_bias = 6.0 * math.exp(-progress * 3.0)
        x = x_truth + estimator_bias * math.sin(t * 0.11)
        y = y_truth - estimator_bias * math.cos(t * 0.09)
        z = max(0.0, z_truth + 4.0 * math.sin(t * 0.12))

        gps_present = i % 5 == 0
        dropout = 56 <= i <= 72
        residual = 2.0 + 18.0 * abs(math.sin(t * 0.17))
        gps_rejected = gps_present and (dropout or residual > 16.0)
        gps_used = gps_present and not gps_rejected

        sigma_h = 16.0 + 46.0 * (1.0 - progress) ** 1.35
        sigma_v = 8.0 + 24.0 * (1.0 - progress) ** 1.15
        if gps_used:
            sigma_h *= 0.74
            sigma_v *= 0.82
        if dropout:
            sigma_h += 20.0
            sigma_v += 9.0

        wind_sigma = 0.6 + 2.6 * math.exp(-progress * 1.8)
        if dropout:
            wind_sigma += 1.3

        gps_x = gps_y = gps_z = None
        if gps_present:
            gps_x = x_truth + residual * 0.45 * math.sin(t * 0.30)
            gps_y = y_truth - residual * 0.35 * math.cos(t * 0.27)
            gps_z = z_truth + residual * 0.25 * math.sin(t * 0.21)

        if gps_rejected:
            label = "GPS rejected"
            rationale = "residual or dropout gate exceeded"
        elif gps_used:
            label = "GPS update used"
            rationale = "measurement accepted into position estimate"
        elif dropout:
            label = "GPS measurement not used"
            rationale = "synthetic GPS dropout window"
        elif wind_sigma > 3.0:
            label = "wind uncertainty high"
            rationale = "wind covariance is still settling"
        elif sigma_h > 55.0:
            label = "position uncertainty high"
            rationale = "horizontal covariance above descent threshold"
        else:
            label = "inertial only"
            rationale = "propagated estimate without GPS update"

        records.append(
            TelemetryRecord(
                t_s=t,
                x_m=x,
                y_m=y,
                z_m=z,
                sigma_h_m=sigma_h,
                sigma_v_m=sigma_v,
                sigma_pos_norm_m=math.sqrt(sigma_h * sigma_h + sigma_v * sigma_v),
                wind_x_mps=wx,
                wind_y_mps=wy,
                wind_z_mps=wz,
                wind_sigma_norm_mps=wind_sigma,
                gps_x_m=gps_x,
                gps_y_m=gps_y,
                gps_z_m=gps_z,
                gps_used=gps_used,
                gps_rejected=gps_rejected,
                gps_pos_innovation_m=residual if gps_present else None,
                gps_vel_innovation_mps=(0.4 + 2.3 * abs(math.sin(t * 0.14))) if gps_present else None,
                gps_residual_m=residual if gps_present else None,
                truth_x_m=x_truth,
                truth_y_m=y_truth,
                truth_z_m=z_truth,
                truth_wind_x_mps=truth_wx,
                truth_wind_y_mps=truth_wy,
                truth_wind_z_mps=truth_wz,
                audit_label=label,
                rationale=rationale,
            )
        )
    return records


def write_records_csv(records: Sequence[TelemetryRecord], path: Path) -> None:
    ensure_parent(path)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=CANONICAL_FIELDS)
        writer.writeheader()
        for rec in records:
            writer.writerow({field: getattr(rec, field) for field in CANONICAL_FIELDS})


def summarize(records: Sequence[TelemetryRecord]) -> Dict[str, object]:
    duration = records[-1].t_s - records[0].t_s if len(records) > 1 else 0.0
    gps_samples = [r for r in records if r.gps_x_m is not None or r.gps_y_m is not None or r.gps_z_m is not None]
    gps_used = [r for r in records if r.gps_used]
    gps_rejected = [r for r in records if r.gps_rejected]
    labels = list(dict.fromkeys(r.audit_label for r in records))
    final = records[-1]

    truth_pos_pairs = [
        ((r.x_m, r.y_m, r.z_m), (r.truth_x_m, r.truth_y_m, r.truth_z_m))
        for r in records
        if r.truth_x_m is not None and r.truth_y_m is not None and r.truth_z_m is not None
    ]
    truth_wind_pairs = [
        (
            (r.wind_x_mps, r.wind_y_mps, r.wind_z_mps),
            (r.truth_wind_x_mps, r.truth_wind_y_mps, r.truth_wind_z_mps),
        )
        for r in records
        if r.truth_wind_x_mps is not None and r.truth_wind_y_mps is not None and r.truth_wind_z_mps is not None
    ]

    def rmse(pairs: Sequence[Tuple[Tuple[float, float, float], Tuple[Optional[float], Optional[float], Optional[float]]]]) -> Optional[float]:
        if not pairs:
            return None
        vals = []
        for est, truth in pairs:
            vals.append(sum((est[i] - float(truth[i])) ** 2 for i in range(3)))
        return math.sqrt(sum(vals) / len(vals))

    return {
        "row_count": len(records),
        "duration_s": duration,
        "accepted_gps_samples": len(gps_used),
        "rejected_gps_samples": len(gps_rejected),
        "gps_position_samples": len(gps_samples),
        "gps_velocity_samples": sum(1 for r in records if r.gps_vel_innovation_mps is not None),
        "final_position_m": [final.x_m, final.y_m, final.z_m],
        "final_wind_mps": [final.wind_x_mps, final.wind_y_mps, final.wind_z_mps],
        "final_wind_speed_mps": vector_norm((final.wind_x_mps, final.wind_y_mps, final.wind_z_mps)),
        "max_position_sigma_norm_m": max(r.sigma_pos_norm_m for r in records),
        "max_wind_sigma_norm_mps": max(r.wind_sigma_norm_mps for r in records),
        "truth_position_rmse_m": rmse(truth_pos_pairs),
        "truth_wind_rmse_mps": rmse(truth_wind_pairs),
        "evidence_labels_observed": labels,
        "final_label": final.audit_label,
        "final_rationale": final.rationale,
    }


def vector_norm(v: Tuple[float, float, float]) -> float:
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def ensure_parent(path: Path) -> None:
    parent = path.parent
    if parent and str(parent) not in ("", "."):
        parent.mkdir(parents=True, exist_ok=True)


def escape(text: object) -> str:
    return html.escape(str(text), quote=True)


def fmt(value: object, digits: int = 2) -> str:
    if value is None:
        return "n/a"
    if isinstance(value, (int,)):
        return str(value)
    if isinstance(value, float):
        return f"{value:.{digits}f}"
    return str(value)


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class Projector3D:
    def __init__(self, records: Sequence[TelemetryRecord], x: float, y: float, w: float, h: float, sigma_scale: float):
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.yaw = math.radians(-38.0)
        self.pitch = math.radians(24.0)
        self._cy = math.cos(self.yaw)
        self._sy = math.sin(self.yaw)
        self._cp = math.cos(self.pitch)
        self._sp = math.sin(self.pitch)

        points: List[Tuple[float, float, float]] = []
        for rec in records:
            r = sigma_scale * max(rec.sigma_h_m, rec.sigma_v_m)
            points.extend(
                [
                    (rec.x_m, rec.y_m, rec.z_m),
                    (rec.x_m + r, rec.y_m, rec.z_m),
                    (rec.x_m - r, rec.y_m, rec.z_m),
                    (rec.x_m, rec.y_m + r, rec.z_m),
                    (rec.x_m, rec.y_m - r, rec.z_m),
                    (rec.x_m, rec.y_m, rec.z_m + r),
                    (rec.x_m, rec.y_m, max(0.0, rec.z_m - r)),
                ]
            )
            if rec.gps_x_m is not None and rec.gps_y_m is not None and rec.gps_z_m is not None:
                points.append((rec.gps_x_m, rec.gps_y_m, rec.gps_z_m))

        raw = [self._project_raw(*p) for p in points]
        min_x = min(p[0] for p in raw)
        max_x = max(p[0] for p in raw)
        min_y = min(p[1] for p in raw)
        max_y = max(p[1] for p in raw)
        span_x = max(1.0, max_x - min_x)
        span_y = max(1.0, max_y - min_y)
        self.scale = min((w * 0.88) / span_x, (h * 0.84) / span_y)
        self.cx_raw = 0.5 * (min_x + max_x)
        self.cy_raw = 0.5 * (min_y + max_y)
        self.cx_screen = x + w * 0.50
        self.cy_screen = y + h * 0.53

    def _project_raw(self, x: float, y: float, z: float) -> Tuple[float, float, float]:
        xr = x * self._cy - y * self._sy
        yr = x * self._sy + y * self._cy
        screen_y = -(z * self._cp - yr * self._sp)
        depth = yr * self._cp + z * self._sp
        return xr, screen_y, depth

    def project(self, x: float, y: float, z: float) -> Tuple[float, float, float]:
        xr, yr, depth = self._project_raw(x, y, z)
        return (
            self.cx_screen + (xr - self.cx_raw) * self.scale,
            self.cy_screen + (yr - self.cy_raw) * self.scale,
            depth,
        )


def svg_header(width: int, height: int, palette: Dict[str, str]) -> str:
    return (
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {width} {height}" role="img" aria-label="{escape(TITLE)}">\n'
        f'<rect width="100%" height="100%" fill="{palette["bg"]}"/>\n'
        '<defs>\n'
        '<marker id="arrow-wind" markerWidth="8" markerHeight="8" refX="7" refY="3.5" orient="auto">\n'
        f'<polygon points="0 0, 7 3.5, 0 7" fill="{palette["wind"]}"/>\n'
        '</marker>\n'
        '</defs>\n'
    )


def panel_rect(x: float, y: float, w: float, h: float, palette: Dict[str, str], title: str) -> str:
    return (
        f'<rect x="{x:.1f}" y="{y:.1f}" width="{w:.1f}" height="{h:.1f}" '
        f'rx="6" fill="{palette["panel"]}" stroke="{palette["grid"]}" stroke-width="1"/>\n'
        f'<text x="{x + 14:.1f}" y="{y + 24:.1f}" fill="{palette["text"]}" '
        'font-family="Consolas, monospace" font-size="17" font-weight="700">'
        f'{escape(title)}</text>\n'
    )


def points_to_path(points: Sequence[Tuple[float, float]]) -> str:
    if not points:
        return ""
    first = points[0]
    rest = " ".join(f"L {x:.1f} {y:.1f}" for x, y in points[1:])
    return f"M {first[0]:.1f} {first[1]:.1f} {rest}"


def draw_3d_panel(records: Sequence[TelemetryRecord], cfg: RenderConfig, palette: Dict[str, str]) -> str:
    x, y, w, h = 36.0, 68.0, 910.0, 620.0
    p = Projector3D(records, x, y, w, h, cfg.sigma_scale)
    parts = [panel_rect(x, y, w, h, palette, "3D trajectory, GPS evidence, wind vectors, and dispersion envelope")]

    # Ground grid, in world coordinates near z=0.
    xs = [r.x_m for r in records]
    ys = [r.y_m for r in records]
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    xstep = max(25.0, (xmax - xmin) / 6.0)
    ystep = max(20.0, (ymax - ymin) / 6.0)
    for i in range(7):
        gx = xmin + i * xstep
        a = p.project(gx, ymin - ystep, 0.0)
        b = p.project(gx, ymax + ystep, 0.0)
        parts.append(f'<line x1="{a[0]:.1f}" y1="{a[1]:.1f}" x2="{b[0]:.1f}" y2="{b[1]:.1f}" stroke="{palette["grid"]}" stroke-width="0.7" opacity="0.45"/>\n')
    for i in range(7):
        gy = ymin + i * ystep
        a = p.project(xmin - xstep, gy, 0.0)
        b = p.project(xmax + xstep, gy, 0.0)
        parts.append(f'<line x1="{a[0]:.1f}" y1="{a[1]:.1f}" x2="{b[0]:.1f}" y2="{b[1]:.1f}" stroke="{palette["grid"]}" stroke-width="0.7" opacity="0.45"/>\n')

    # Dispersion tube cross-sections, sorted by depth so distant sections draw first.
    envelope = []
    for idx, rec in enumerate(records):
        if idx % max(1, cfg.envelope_downsample) != 0 and idx != len(records) - 1:
            continue
        sx, sy, depth = p.project(rec.x_m, rec.y_m, rec.z_m)
        rx = cfg.sigma_scale * rec.sigma_h_m * p.scale
        ry = cfg.sigma_scale * max(1.0, rec.sigma_v_m) * p.scale * 0.72
        opacity = 0.08 + 0.18 * clamp(rec.sigma_pos_norm_m / max(1.0, max(r.sigma_pos_norm_m for r in records)), 0.0, 1.0)
        envelope.append((depth, f'<ellipse cx="{sx:.1f}" cy="{sy:.1f}" rx="{rx:.1f}" ry="{ry:.1f}" fill="{palette["envelope"]}" fill-opacity="{opacity:.3f}" stroke="{palette["envelope"]}" stroke-opacity="0.38" stroke-width="1"/>\n'))
    for _, svg in sorted(envelope, key=lambda item: item[0]):
        parts.append(svg)

    # Trajectory.
    trajectory = [p.project(r.x_m, r.y_m, r.z_m)[:2] for r in records]
    parts.append(f'<path d="{points_to_path(trajectory)}" fill="none" stroke="{palette["trajectory"]}" stroke-width="3" stroke-linejoin="round" stroke-linecap="round"/>\n')

    # Truth trajectory, if present.
    truth_points = [
        p.project(r.truth_x_m, r.truth_y_m, r.truth_z_m)[:2]
        for r in records
        if r.truth_x_m is not None and r.truth_y_m is not None and r.truth_z_m is not None
    ]
    if len(truth_points) > 1:
        parts.append(f'<path d="{points_to_path(truth_points)}" fill="none" stroke="{palette["muted"]}" stroke-width="1.5" stroke-dasharray="6 6" opacity="0.75"/>\n')

    # GPS evidence.
    gps_count = 0
    for rec in records:
        if rec.gps_x_m is None or rec.gps_y_m is None or rec.gps_z_m is None:
            continue
        gps_count += 1
        if (gps_count - 1) % max(1, cfg.gps_downsample) != 0:
            continue
        sx, sy, _ = p.project(rec.gps_x_m, rec.gps_y_m, rec.gps_z_m)
        color = palette["gps_rejected"] if rec.gps_rejected else palette["gps_used"] if rec.gps_used else palette["gps_raw"]
        shape = "x" if rec.gps_rejected else "circle"
        if shape == "x":
            parts.append(f'<line x1="{sx - 5:.1f}" y1="{sy - 5:.1f}" x2="{sx + 5:.1f}" y2="{sy + 5:.1f}" stroke="{color}" stroke-width="2"/>\n')
            parts.append(f'<line x1="{sx + 5:.1f}" y1="{sy - 5:.1f}" x2="{sx - 5:.1f}" y2="{sy + 5:.1f}" stroke="{color}" stroke-width="2"/>\n')
        else:
            parts.append(f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="4.2" fill="{color}" stroke="{palette["bg"]}" stroke-width="1"/>\n')

    # Wind vectors.
    for idx, rec in enumerate(records):
        if idx % max(1, cfg.wind_downsample) != 0:
            continue
        sx, sy, _ = p.project(rec.x_m, rec.y_m, rec.z_m)
        ex, ey, _ = p.project(
            rec.x_m + rec.wind_x_mps * cfg.wind_vector_scale,
            rec.y_m + rec.wind_y_mps * cfg.wind_vector_scale,
            rec.z_m + rec.wind_z_mps * cfg.wind_vector_scale,
        )
        parts.append(f'<line x1="{sx:.1f}" y1="{sy:.1f}" x2="{ex:.1f}" y2="{ey:.1f}" stroke="{palette["wind"]}" stroke-width="1.6" marker-end="url(#arrow-wind)" opacity="0.82"/>\n')

    # Landing/end marker.
    end = records[-1]
    ex, ey, _ = p.project(end.x_m, end.y_m, end.z_m)
    parts.append(f'<circle cx="{ex:.1f}" cy="{ey:.1f}" r="7" fill="none" stroke="{palette["warning"]}" stroke-width="2.2"/>\n')
    parts.append(f'<text x="{ex + 10:.1f}" y="{ey - 8:.1f}" fill="{palette["text"]}" font-family="Consolas, monospace" font-size="12">final estimate</text>\n')

    legend_x = x + 20
    legend_y = y + h - 72
    legend = [
        (palette["trajectory"], "estimate"),
        (palette["envelope"], f"{cfg.sigma_scale:g}-sigma dispersion envelope"),
        (palette["gps_used"], "GPS accepted"),
        (palette["gps_rejected"], "GPS rejected"),
        (palette["wind"], "wind vector"),
    ]
    for i, (color, label) in enumerate(legend):
        yy = legend_y + i * 14
        parts.append(f'<rect x="{legend_x:.1f}" y="{yy - 8:.1f}" width="10" height="10" fill="{color}" opacity="0.85"/>\n')
        parts.append(f'<text x="{legend_x + 16:.1f}" y="{yy:.1f}" fill="{palette["muted"]}" font-family="Consolas, monospace" font-size="12">{escape(label)}</text>\n')
    return "".join(parts)


def _series_points(records: Sequence[TelemetryRecord], getter) -> List[Tuple[float, Optional[float]]]:
    return [(r.t_s, getter(r)) for r in records]


def draw_time_plot(
    x: float,
    y: float,
    w: float,
    h: float,
    title: str,
    ylabel: str,
    series: Sequence[Tuple[str, Sequence[Tuple[float, Optional[float]]], str, str]],
    palette: Dict[str, str],
) -> str:
    parts = [panel_rect(x, y, w, h, palette, title)]
    plot_x = x + 54
    plot_y = y + 42
    plot_w = w - 78
    plot_h = h - 72
    all_t = [t for _, values, _, _ in series for t, v in values if v is not None]
    all_y = [float(v) for _, values, _, _ in series for _, v in values if v is not None]
    if not all_t or not all_y:
        parts.append(f'<text x="{plot_x:.1f}" y="{plot_y + 30:.1f}" fill="{palette["muted"]}" font-family="Consolas, monospace" font-size="13">No data available</text>\n')
        return "".join(parts)
    tmin, tmax = min(all_t), max(all_t)
    ymin, ymax = min(all_y), max(all_y)
    if abs(ymax - ymin) < 1e-9:
        ymax += 1.0
        ymin -= 1.0
    pad = 0.08 * (ymax - ymin)
    ymin -= pad
    ymax += pad

    def sx(t: float) -> float:
        return plot_x + (t - tmin) / max(1e-9, tmax - tmin) * plot_w

    def sy(v: float) -> float:
        return plot_y + plot_h - (v - ymin) / max(1e-9, ymax - ymin) * plot_h

    parts.append(f'<rect x="{plot_x:.1f}" y="{plot_y:.1f}" width="{plot_w:.1f}" height="{plot_h:.1f}" fill="{palette["panel2"]}" stroke="{palette["grid"]}" stroke-width="1"/>\n')
    for i in range(5):
        yy = plot_y + i * plot_h / 4.0
        val = ymax - i * (ymax - ymin) / 4.0
        parts.append(f'<line x1="{plot_x:.1f}" y1="{yy:.1f}" x2="{plot_x + plot_w:.1f}" y2="{yy:.1f}" stroke="{palette["grid"]}" stroke-width="0.7" opacity="0.55"/>\n')
        parts.append(f'<text x="{plot_x - 8:.1f}" y="{yy + 4:.1f}" text-anchor="end" fill="{palette["muted"]}" font-family="Consolas, monospace" font-size="11">{fmt(val, 1)}</text>\n')
    for i in range(5):
        xx = plot_x + i * plot_w / 4.0
        val = tmin + i * (tmax - tmin) / 4.0
        parts.append(f'<line x1="{xx:.1f}" y1="{plot_y:.1f}" x2="{xx:.1f}" y2="{plot_y + plot_h:.1f}" stroke="{palette["grid"]}" stroke-width="0.5" opacity="0.35"/>\n')
        parts.append(f'<text x="{xx:.1f}" y="{plot_y + plot_h + 17:.1f}" text-anchor="middle" fill="{palette["muted"]}" font-family="Consolas, monospace" font-size="11">{fmt(val, 0)}</text>\n')

    for label, values, color, dash in series:
        pts = [(sx(t), sy(float(v))) for t, v in values if v is not None]
        if len(pts) >= 2:
            dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
            parts.append(f'<path d="{points_to_path(pts)}" fill="none" stroke="{color}" stroke-width="2"{dash_attr}/>\n')

    parts.append(f'<text x="{plot_x - 38:.1f}" y="{plot_y + plot_h / 2:.1f}" transform="rotate(-90 {plot_x - 38:.1f} {plot_y + plot_h / 2:.1f})" text-anchor="middle" fill="{palette["muted"]}" font-family="Consolas, monospace" font-size="11">{escape(ylabel)}</text>\n')
    parts.append(f'<text x="{plot_x + plot_w / 2:.1f}" y="{plot_y + plot_h + 36:.1f}" text-anchor="middle" fill="{palette["muted"]}" font-family="Consolas, monospace" font-size="11">time (s)</text>\n')

    lx = x + w - 210
    ly = y + 24
    for i, (label, _, color, dash) in enumerate(series):
        yy = ly + i * 14
        dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
        parts.append(f'<line x1="{lx:.1f}" y1="{yy:.1f}" x2="{lx + 22:.1f}" y2="{yy:.1f}" stroke="{color}" stroke-width="2"{dash_attr}/>\n')
        parts.append(f'<text x="{lx + 28:.1f}" y="{yy + 4:.1f}" fill="{palette["muted"]}" font-family="Consolas, monospace" font-size="11">{escape(label)}</text>\n')
    return "".join(parts)


def draw_gps_evidence(records: Sequence[TelemetryRecord], palette: Dict[str, str]) -> str:
    x, y, w, h = 970.0, 318.0, 590.0, 214.0
    series = [
        ("pos innovation (m)", _series_points(records, lambda r: r.gps_pos_innovation_m), palette["gps_raw"], ""),
        ("vel innovation (m/s)", _series_points(records, lambda r: r.gps_vel_innovation_mps), palette["wind_y"], "5 4"),
        ("GPS residual (m)", _series_points(records, lambda r: r.gps_residual_m), palette["gps_rejected"], "2 3"),
    ]
    parts = [draw_time_plot(x, y, w, h, "GPS fusion evidence", "innovation / residual", series, palette)]
    plot_x = x + 54
    plot_y = y + h - 34
    plot_w = w - 78
    tmin, tmax = records[0].t_s, records[-1].t_s

    def sx(t: float) -> float:
        return plot_x + (t - tmin) / max(1e-9, tmax - tmin) * plot_w

    for rec in records:
        if rec.gps_used:
            parts.append(f'<rect x="{sx(rec.t_s) - 2:.1f}" y="{plot_y - 10:.1f}" width="4" height="8" fill="{palette["gps_used"]}"/>\n')
        if rec.gps_rejected:
            parts.append(f'<rect x="{sx(rec.t_s) - 2:.1f}" y="{plot_y:.1f}" width="4" height="8" fill="{palette["gps_rejected"]}"/>\n')
    parts.append(f'<text x="{plot_x:.1f}" y="{plot_y + 22:.1f}" fill="{palette["muted"]}" font-family="Consolas, monospace" font-size="11">flags: accepted above baseline, rejected below baseline</text>\n')
    return "".join(parts)


def draw_audit_timeline(records: Sequence[TelemetryRecord], palette: Dict[str, str]) -> str:
    x, y, w, h = 970.0, 552.0, 590.0, 166.0
    parts = [panel_rect(x, y, w, h, palette, "Audit / classification timeline")]
    labels = list(dict.fromkeys(r.audit_label for r in records))
    color_bank = [
        palette["gps_used"],
        palette["gps_rejected"],
        palette["warning"],
        palette["sigma_h"],
        palette["wind"],
        palette["muted"],
        palette["sigma_pos"],
        palette["gps_raw"],
        palette["wind_z"],
    ]
    color_for = {label: color_bank[i % len(color_bank)] for i, label in enumerate(labels)}
    plot_x = x + 24
    plot_y = y + 48
    plot_w = w - 48
    plot_h = 42.0
    tmin, tmax = records[0].t_s, records[-1].t_s
    parts.append(f'<rect x="{plot_x:.1f}" y="{plot_y:.1f}" width="{plot_w:.1f}" height="{plot_h:.1f}" fill="{palette["panel2"]}" stroke="{palette["grid"]}" stroke-width="1"/>\n')
    for i, rec in enumerate(records):
        t0 = rec.t_s
        t1 = records[i + 1].t_s if i + 1 < len(records) else tmax
        sx0 = plot_x + (t0 - tmin) / max(1e-9, tmax - tmin) * plot_w
        sx1 = plot_x + (t1 - tmin) / max(1e-9, tmax - tmin) * plot_w
        parts.append(f'<rect x="{sx0:.1f}" y="{plot_y:.1f}" width="{max(1.0, sx1 - sx0):.1f}" height="{plot_h:.1f}" fill="{color_for[rec.audit_label]}" opacity="0.72"/>\n')
    lx = plot_x
    ly = plot_y + plot_h + 24
    for i, label in enumerate(labels[:7]):
        row = i // 2
        col = i % 2
        xx = lx + col * 270
        yy = ly + row * 17
        parts.append(f'<rect x="{xx:.1f}" y="{yy - 9:.1f}" width="10" height="10" fill="{color_for[label]}"/>\n')
        parts.append(f'<text x="{xx + 16:.1f}" y="{yy:.1f}" fill="{palette["muted"]}" font-family="Consolas, monospace" font-size="11">{escape(label)}</text>\n')
    if len(labels) > 7:
        parts.append(f'<text x="{lx:.1f}" y="{ly + 68:.1f}" fill="{palette["muted"]}" font-family="Consolas, monospace" font-size="11">+ {len(labels) - 7} additional labels in summary JSON</text>\n')
    return "".join(parts)


def draw_summary(summary: Dict[str, object], palette: Dict[str, str]) -> str:
    x, y, w, h = 970.0, 68.0, 590.0, 230.0
    parts = [panel_rect(x, y, w, h, palette, "Run summary")]
    final_pos = summary["final_position_m"]
    final_wind = summary["final_wind_mps"]
    lines = [
        f"rows: {summary['row_count']}   duration: {fmt(summary['duration_s'], 1)} s",
        f"GPS accepted/rejected: {summary['accepted_gps_samples']} / {summary['rejected_gps_samples']}",
        f"GPS pos/vel samples: {summary['gps_position_samples']} / {summary['gps_velocity_samples']}",
        f"final pos: [{fmt(final_pos[0], 1)}, {fmt(final_pos[1], 1)}, {fmt(final_pos[2], 1)}] m",
        f"final wind: [{fmt(final_wind[0], 2)}, {fmt(final_wind[1], 2)}, {fmt(final_wind[2], 2)}] m/s",
        f"final wind speed: {fmt(summary['final_wind_speed_mps'], 2)} m/s",
        f"max pos sigma norm: {fmt(summary['max_position_sigma_norm_m'], 2)} m",
        f"max wind sigma norm: {fmt(summary['max_wind_sigma_norm_mps'], 2)} m/s",
        f"truth pos RMSE: {fmt(summary['truth_position_rmse_m'], 2)} m",
        f"truth wind RMSE: {fmt(summary['truth_wind_rmse_mps'], 2)} m/s",
        f"final label: {summary['final_label']}",
        f"final rationale: {summary['final_rationale'] or 'n/a'}",
    ]
    tx = x + 18
    ty = y + 50
    for i, line in enumerate(lines):
        parts.append(f'<text x="{tx:.1f}" y="{ty + i * 14:.1f}" fill="{palette["text"]}" font-family="Consolas, monospace" font-size="12">{escape(line)}</text>\n')
    return "".join(parts)


def render_svg(records: Sequence[TelemetryRecord], cfg: RenderConfig, summary: Dict[str, object]) -> str:
    if not records:
        raise ValueError("cannot render empty telemetry record list")
    palette = PALETTE.get(cfg.theme, PALETTE["dark"])
    parts = [svg_header(cfg.width_px, cfg.height_px, palette)]
    parts.append(f'<text x="36" y="38" fill="{palette["text"]}" font-family="Consolas, monospace" font-size="24" font-weight="700">{escape(TITLE)}</text>\n')
    parts.append(f'<text x="38" y="58" fill="{palette["muted"]}" font-family="Consolas, monospace" font-size="13">Host-side diagnostic: covariance/sigma envelope, GPS fusion evidence, wind disturbance, and audit labels. Not synthesizable HDL.</text>\n')
    parts.append(draw_3d_panel(records, cfg, palette))
    parts.append(draw_summary(summary, palette))
    parts.append(draw_gps_evidence(records, palette))
    parts.append(draw_audit_timeline(records, palette))

    wind_series = [
        ("wx", _series_points(records, lambda r: r.wind_x_mps), palette["wind_x"], ""),
        ("wy", _series_points(records, lambda r: r.wind_y_mps), palette["wind_y"], ""),
        ("wz", _series_points(records, lambda r: r.wind_z_mps), palette["wind_z"], ""),
        ("|w|", _series_points(records, lambda r: vector_norm((r.wind_x_mps, r.wind_y_mps, r.wind_z_mps))), palette["speed"], "6 4"),
    ]
    parts.append(draw_time_plot(36.0, 730.0, 740.0, 210.0, "Wind components", "m/s", wind_series, palette))

    uncertainty_series = [
        ("position sigma norm", _series_points(records, lambda r: r.sigma_pos_norm_m), palette["sigma_pos"], ""),
        ("horizontal sigma", _series_points(records, lambda r: r.sigma_h_m), palette["sigma_h"], ""),
        ("vertical sigma", _series_points(records, lambda r: r.sigma_v_m), palette["sigma_v"], ""),
        ("wind sigma norm", _series_points(records, lambda r: r.wind_sigma_norm_mps), palette["sigma_wind"], "6 4"),
    ]
    parts.append(draw_time_plot(806.0, 730.0, 754.0, 210.0, "Position and wind uncertainty magnitudes", "m and m/s", uncertainty_series, palette))

    note_y = 982
    note = (
        f"Envelope interpretation: each translucent cross-section is {cfg.sigma_scale:g}-sigma, "
        "using horizontal and vertical position uncertainty. The volume is a representation of possible "
        "landing dispersion from propagated state uncertainty and wind disturbance, not a physical cone."
    )
    parts.append(f'<rect x="36" y="{note_y}" width="1524" height="72" rx="6" fill="{palette["panel"]}" stroke="{palette["grid"]}" stroke-width="1"/>\n')
    parts.append(f'<text x="54" y="{note_y + 28}" fill="{palette["text"]}" font-family="Consolas, monospace" font-size="13">{escape(note[:190])}</text>\n')
    parts.append(f'<text x="54" y="{note_y + 50}" fill="{palette["muted"]}" font-family="Consolas, monospace" font-size="12">CSV schema accepts direct sigma fields or diagonal covariance fields; optional GPS, wind covariance, truth, and audit fields are rendered when present.</text>\n')
    parts.append("</svg>\n")
    return "".join(parts)


def schema_json() -> Dict[str, object]:
    required = {
        "t_s": list(ALIASES["t_s"]),
        "x_m": list(ALIASES["x_m"]),
        "y_m": list(ALIASES["y_m"]),
        "z_m": list(ALIASES["z_m"]),
    }
    optional = {key: list(value) for key, value in ALIASES.items() if key not in required}
    return {
        "title": TITLE,
        "required_position_time_columns": required,
        "optional_columns": optional,
        "units": {
            "position": "meters",
            "velocity": "meters/second",
            "time": "seconds",
            "covariance": "meters^2 for position, (meters/second)^2 for wind",
            "sigma": "1-sigma; plotted envelope uses --sigma-scale",
        },
    }


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=TITLE,
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--input", type=Path, help="CSV telemetry input path")
    source.add_argument("--synthetic", action="store_true", help="use deterministic synthetic descent data")
    parser.add_argument("--synthetic-count", type=int, default=160, help="row count for synthetic data")
    parser.add_argument("--synthetic-out", type=Path, help="optional CSV path for generated synthetic data")
    parser.add_argument("--output", type=Path, default=Path(".codex_build/landing_dispersion_envelope/landing_dispersion_envelope.svg"), help="SVG output path")
    parser.add_argument("--summary-out", type=Path, help="optional JSON summary output path")
    parser.add_argument("--schema-out", type=Path, help="optional JSON schema/alias output path")
    parser.add_argument("--dump-schema", action="store_true", help="print accepted schema aliases and exit")
    parser.add_argument("--sigma-scale", type=float, default=2.0, help="sigma multiplier for the rendered envelope")
    parser.add_argument("--wind-vector-scale", type=float, default=10.0, help="meters of displayed vector per m/s wind")
    parser.add_argument("--gps-downsample", type=int, default=1, help="plot every Nth GPS marker")
    parser.add_argument("--envelope-downsample", type=int, default=4, help="plot every Nth uncertainty cross-section")
    parser.add_argument("--wind-downsample", type=int, default=10, help="plot every Nth wind vector")
    parser.add_argument("--theme", choices=("dark", "light"), default="dark", help="figure theme")
    parser.add_argument("--show", action="store_true", help="open the SVG with the platform default viewer after writing")
    return parser


def run(args: argparse.Namespace) -> int:
    if args.dump_schema:
        print(json.dumps(schema_json(), indent=2))
        return 0

    if args.schema_out:
        ensure_parent(args.schema_out)
        args.schema_out.write_text(json.dumps(schema_json(), indent=2) + "\n", encoding="utf-8")

    if args.input:
        records = read_telemetry_csv(args.input)
    else:
        records = generate_synthetic_records(args.synthetic_count)

    if args.synthetic_out:
        write_records_csv(records, args.synthetic_out)

    cfg = RenderConfig(
        sigma_scale=args.sigma_scale,
        wind_vector_scale=args.wind_vector_scale,
        gps_downsample=max(1, args.gps_downsample),
        envelope_downsample=max(1, args.envelope_downsample),
        wind_downsample=max(1, args.wind_downsample),
        theme=args.theme,
    )
    summary = summarize(records)
    svg = render_svg(records, cfg, summary)
    ensure_parent(args.output)
    args.output.write_text(svg, encoding="utf-8")

    if args.summary_out:
        ensure_parent(args.summary_out)
        args.summary_out.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")

    print(f"Wrote {args.output}")
    if args.synthetic_out:
        print(f"Wrote {args.synthetic_out}")
    if args.summary_out:
        print(f"Wrote {args.summary_out}")
    if args.schema_out:
        print(f"Wrote {args.schema_out}")
    if args.show:
        webbrowser.open(args.output.resolve().as_uri())
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    if not args.input and not args.synthetic:
        # Synthetic data is the safe default for a checkout without exported EKF/GPS logs.
        args.synthetic = True
    if args.sigma_scale <= 0.0:
        parser.error("--sigma-scale must be positive")
    if args.wind_vector_scale < 0.0:
        parser.error("--wind-vector-scale must be non-negative")
    if args.synthetic_count < 2:
        parser.error("--synthetic-count must be at least 2")
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
