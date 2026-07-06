#!/usr/bin/env python3
"""Regression smoke tests for landing_dispersion_envelope.py."""

from __future__ import annotations

import csv
import sys
import tempfile
import unittest
from pathlib import Path

try:
    from . import landing_dispersion_envelope as lde
except ImportError:
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import landing_dispersion_envelope as lde


class LandingDispersionEnvelopeTest(unittest.TestCase):
    def test_synthetic_render_and_summary(self) -> None:
        records = lde.generate_synthetic_records(48)
        summary = lde.summarize(records)
        svg = lde.render_svg(records, lde.RenderConfig(envelope_downsample=6), summary)

        self.assertEqual(summary["row_count"], 48)
        self.assertGreater(summary["accepted_gps_samples"], 0)
        self.assertGreater(summary["rejected_gps_samples"], 0)
        self.assertIn("dispersion envelope", svg.lower())
        self.assertNotIn("uncertainty " + "cone", svg.lower())
        self.assertIn("<svg", svg)

    def test_minimal_csv_with_covariance_and_missing_optional_fields(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "minimal.csv"
            with path.open("w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(
                    f,
                    fieldnames=["t_s", "x_m", "y_m", "z_m", "cov_xx_m2", "cov_yy_m2", "cov_zz_m2"],
                )
                writer.writeheader()
                for i in range(5):
                    writer.writerow(
                        {
                            "t_s": i,
                            "x_m": 10 * i,
                            "y_m": i,
                            "z_m": 100 - 10 * i,
                            "cov_xx_m2": 4.0,
                            "cov_yy_m2": 9.0,
                            "cov_zz_m2": 16.0,
                        }
                    )

            records = lde.read_telemetry_csv(path)
            self.assertEqual(len(records), 5)
            self.assertAlmostEqual(records[0].sigma_h_m, (4.0 + 9.0) ** 0.5)
            self.assertEqual(records[0].wind_sigma_norm_mps, 0.0)
            svg = lde.render_svg(records, lde.RenderConfig(theme="light"), lde.summarize(records))
            self.assertIn("No data", svg)

    def test_cli_writes_outputs(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "figure.svg"
            csv_out = Path(tmp) / "synthetic.csv"
            summary_out = Path(tmp) / "summary.json"
            rc = lde.main(
                [
                    "--synthetic",
                    "--synthetic-count",
                    "24",
                    "--output",
                    str(out),
                    "--synthetic-out",
                    str(csv_out),
                    "--summary-out",
                    str(summary_out),
                ]
            )
            self.assertEqual(rc, 0)
            self.assertTrue(out.exists())
            self.assertTrue(csv_out.exists())
            self.assertTrue(summary_out.exists())


if __name__ == "__main__":
    unittest.main()
