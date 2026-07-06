#!/usr/bin/env python3
"""Regression tests for simulator emission and pipeline wrapper."""

from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

try:
    from . import emit_landing_sim_log as emitter
    from . import export_landing_telemetry_csv as exporter
    from . import landing_dispersion_envelope as lde
    from . import run_landing_dispersion_regression as pipeline
except ImportError:
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import emit_landing_sim_log as emitter
    import export_landing_telemetry_csv as exporter
    import landing_dispersion_envelope as lde
    import run_landing_dispersion_regression as pipeline


class LandingRegressionPipelineTest(unittest.TestCase):
    def test_emitter_writes_alias_logs_exporter_accepts(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            estimate = root / "estimate.csv"
            truth = root / "truth.csv"
            canonical = root / "canonical.csv"
            rc = emitter.main(["--estimate-out", str(estimate), "--truth-out", str(truth), "--sample-count", "32"])
            self.assertEqual(rc, 0)
            export_rc = exporter.main(["--input", str(estimate), "--truth-input", str(truth), "--output", str(canonical)])
            self.assertEqual(export_rc, 0)
            records = lde.read_telemetry_csv(canonical)
            self.assertEqual(len(records), 32)
            self.assertIsNotNone(lde.summarize(records)["truth_position_rmse_m"])

    def test_pipeline_falls_back_to_simulator_logs(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            out_dir = Path(tmp) / "regression"
            rc = pipeline.main(["--output-dir", str(out_dir), "--sample-count", "24"])
            self.assertEqual(rc, 0)
            manifest_path = out_dir / "regression_manifest.json"
            self.assertTrue(manifest_path.exists())
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            self.assertEqual(manifest["source_mode"], "simulated")
            self.assertTrue((out_dir / "landing_canonical.csv").exists())
            self.assertTrue((out_dir / "landing_dispersion.svg").exists())


if __name__ == "__main__":
    unittest.main()
