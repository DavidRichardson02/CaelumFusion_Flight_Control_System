#!/usr/bin/env python3
"""Regression tests for export_landing_telemetry_csv.py."""

from __future__ import annotations

import csv
import contextlib
import io
import json
import sys
import tempfile
import unittest
from pathlib import Path

try:
    from . import export_landing_telemetry_csv as exporter
    from . import landing_dispersion_envelope as lde
except ImportError:
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import export_landing_telemetry_csv as exporter
    import landing_dispersion_envelope as lde


class ExportLandingTelemetryCsvTest(unittest.TestCase):
    def test_csv_export_merges_truth_by_time_and_renders(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            estimate = root / "estimate.csv"
            truth = root / "truth.csv"
            output = root / "canonical.csv"
            manifest = root / "manifest.json"
            figure = root / "figure.svg"
            summary = root / "summary.json"

            with estimate.open("w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(
                    f,
                    fieldnames=[
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
                        "gps_position_innovation_m",
                    ],
                )
                writer.writeheader()
                for idx in range(6):
                    writer.writerow(
                        {
                            "time_s": idx * 0.25,
                            "pos_x_m": 12.0 + idx,
                            "pos_y_m": -4.0 + idx * 0.5,
                            "altitude_m": 100.0 - idx * 5.0,
                            "cov_xx_m2": 4.0,
                            "cov_yy_m2": 9.0,
                            "cov_zz_m2": 16.0,
                            "wx_mps": 1.0,
                            "wy_mps": -0.5,
                            "wz_mps": 0.1,
                            "sigma_wind_norm_mps": 0.75,
                            "gps_pos_x_m": 12.1 + idx,
                            "gps_pos_y_m": -3.8 + idx * 0.5,
                            "gps_altitude_m": 99.8 - idx * 5.0,
                            "gps_update_used": 1,
                            "gps_position_innovation_m": 1.25,
                        }
                    )

            with truth.open("w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(
                    f,
                    fieldnames=["t_s", "truth_x_m", "truth_y_m", "truth_z_m", "truth_wind_x_mps", "truth_wind_y_mps", "truth_wind_z_mps"],
                )
                writer.writeheader()
                for idx in range(6):
                    writer.writerow(
                        {
                            "t_s": idx * 0.25,
                            "truth_x_m": 12.0 + idx * 0.9,
                            "truth_y_m": -4.2 + idx * 0.5,
                            "truth_z_m": 101.0 - idx * 5.1,
                            "truth_wind_x_mps": 0.9,
                            "truth_wind_y_mps": -0.4,
                            "truth_wind_z_mps": 0.0,
                        }
                    )

            rc = exporter.main(
                [
                    "--input",
                    str(estimate),
                    "--truth-input",
                    str(truth),
                    "--output",
                    str(output),
                    "--manifest-out",
                    str(manifest),
                    "--render-svg",
                    str(figure),
                    "--render-summary-out",
                    str(summary),
                ]
            )
            self.assertEqual(rc, 0)
            self.assertTrue(output.exists())
            self.assertTrue(figure.exists())
            records = lde.read_telemetry_csv(output)
            self.assertEqual(len(records), 6)
            self.assertAlmostEqual(records[0].truth_x_m or 0.0, 12.0)
            self.assertGreater(lde.summarize(records)["truth_position_rmse_m"], 0.0)
            manifest_obj = json.loads(manifest.read_text(encoding="utf-8"))
            self.assertEqual(manifest_obj["truth_rows_joined"], 6)

    def test_strict_export_rejects_missing_wind_and_gps(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "minimal.csv"
            out = Path(tmp) / "out.csv"
            with path.open("w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=["t_s", "x_m", "y_m", "z_m", "cov_xx_m2", "cov_yy_m2", "cov_zz_m2"])
                writer.writeheader()
                writer.writerow({"t_s": 0, "x_m": 1, "y_m": 2, "z_m": 3, "cov_xx_m2": 1, "cov_yy_m2": 1, "cov_zz_m2": 1})

            with contextlib.redirect_stderr(io.StringIO()):
                rc = exporter.main(["--input", str(path), "--output", str(out)])
            self.assertEqual(rc, 2)
            self.assertFalse(out.exists())


if __name__ == "__main__":
    unittest.main()
