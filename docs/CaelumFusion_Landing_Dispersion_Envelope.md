# 3D Wind-Driven Landing Dispersion Envelope With Position Uncertainty Bounds

This document describes the host-side CaelumFusion diagnostic implemented in
`tools/analysis/landing_dispersion_envelope.py`. The tool produces an SVG
engineering figure for analyzing a wind-influenced landing trajectory estimate,
position uncertainty bounds, GPS fusion evidence, and audit labels over time.

The diagnostic is software-side only. It does not modify synthesizable FPGA
logic, the Basys-3 top modules, the VGA renderer, or the Vivado build scripts.
It is intended to consume exported EKF/simulation/telemetry CSV data when those
fields are available, and it includes a deterministic synthetic descent
generator so the plotting pipeline can be tested from a clean checkout.

## Why This Is A Dispersion Envelope

The visualization is a trajectory-following uncertainty envelope, not a
fixed-apex geometric spread. Each sample has its own horizontal and vertical
uncertainty radius, derived from direct sigma fields or covariance diagonal
fields. The translucent cross-sections therefore represent the possible position
dispersion around the estimated trajectory at that time, not a physical object
and not a single fixed-shape volume.

## Run Commands

Generate a complete synthetic example, SVG figure, summary JSON, and synthetic
CSV:

```powershell
python tools\analysis\landing_dispersion_envelope.py `
  --synthetic `
  --output .codex_build\landing_dispersion_envelope\example.svg `
  --synthetic-out .codex_build\landing_dispersion_envelope\synthetic_landing.csv `
  --summary-out .codex_build\landing_dispersion_envelope\summary.json `
  --theme dark `
  --sigma-scale 2 `
  --wind-vector-scale 10
```

Normalize a real EKF/GPS/wind log into the canonical renderer CSV. The exporter
accepts CSV or JSONL, maps documented aliases, optionally joins a separate truth
file, writes a manifest, and can render the SVG in the same command:

```powershell
python tools\analysis\export_landing_telemetry_csv.py `
  --input path\to\ekf_gps_wind_log.jsonl `
  --truth-input path\to\simulation_truth.csv `
  --truth-join time `
  --truth-tolerance-s 0.05 `
  --output .codex_build\landing_dispersion_envelope\flight_canonical.csv `
  --manifest-out .codex_build\landing_dispersion_envelope\flight_export_manifest.json `
  --render-svg .codex_build\landing_dispersion_envelope\flight_run.svg `
  --render-summary-out .codex_build\landing_dispersion_envelope\flight_run_summary.json
```

Emit simulator-side logs that already use the documented aliases. This is the
stand-in for the first real EKF/GPS/wind log and is also the recommended shape
for future simulator export code:

```powershell
python tools\analysis\emit_landing_sim_log.py `
  --estimate-out .codex_build\landing_dispersion_envelope\sim_estimate_log.csv `
  --truth-out .codex_build\landing_dispersion_envelope\sim_truth_log.csv `
  --manifest-out .codex_build\landing_dispersion_envelope\sim_log_manifest.json `
  --sample-count 160
```

Run the lightweight export-and-render regression. Pass `--input` when a real log
exists; otherwise the script emits deterministic simulator logs first and uses
those as the regression source:

```powershell
python tools\analysis\run_landing_dispersion_regression.py `
  --input path\to\ekf_gps_wind_log.csv `
  --truth-input path\to\simulation_truth.csv `
  --output-dir .codex_build\landing_dispersion_envelope\regression
```

Without `--input`, the same command exercises the pipeline with simulator-side
logs:

```powershell
python tools\analysis\run_landing_dispersion_regression.py `
  --output-dir .codex_build\landing_dispersion_envelope\regression
```

Render an already canonical telemetry CSV:

```powershell
python tools\analysis\landing_dispersion_envelope.py `
  --input .codex_build\landing_dispersion_envelope\flight_canonical.csv `
  --output .codex_build\landing_dispersion_envelope\flight_run.svg `
  --summary-out .codex_build\landing_dispersion_envelope\flight_run_summary.json `
  --theme light `
  --sigma-scale 3 `
  --gps-downsample 2 `
  --envelope-downsample 4 `
  --wind-downsample 10
```

Print the accepted schema aliases:

```powershell
python tools\analysis\landing_dispersion_envelope.py --dump-schema
```

Run the regression smoke tests:

```powershell
python -m unittest tools.analysis.test_landing_dispersion_envelope tools.analysis.test_export_landing_telemetry_csv
python -m unittest tools.analysis.test_landing_regression_pipeline
```

## Expected CSV Schema

Required columns:

| Quantity | Preferred column | Accepted examples | Units |
| --- | --- | --- | --- |
| Time | `t_s` | `time_s`, `time`, `t`, `timestamp_s` | seconds |
| Estimated X | `x_m` | `pos_x_m`, `position_x_m`, `est_x_m`, `north_m` | meters |
| Estimated Y | `y_m` | `pos_y_m`, `position_y_m`, `est_y_m`, `east_m`, `crossrange_m` | meters |
| Estimated Z | `z_m` | `pos_z_m`, `position_z_m`, `est_z_m`, `altitude_m` | meters |

Position uncertainty may be supplied directly:

| Quantity | Preferred column | Units |
| --- | --- | --- |
| Horizontal sigma | `sigma_h_m` | meters |
| Vertical sigma | `sigma_v_m` | meters |
| Position sigma norm | `sigma_pos_norm_m` | meters |

Alternatively, provide diagonal covariance terms:

| Quantity | Preferred column | Units |
| --- | --- | --- |
| X covariance | `cov_xx_m2` | meters squared |
| Y covariance | `cov_yy_m2` | meters squared |
| Z covariance | `cov_zz_m2` | meters squared |

When covariance is provided, the tool computes:

```text
sigma_x = sqrt(max(cov_xx_m2, 0))
sigma_y = sqrt(max(cov_yy_m2, 0))
sigma_z = sqrt(max(cov_zz_m2, 0))
horizontal_sigma = sqrt(sigma_x^2 + sigma_y^2)
vertical_sigma = sigma_z
position_sigma_norm = sqrt(sigma_x^2 + sigma_y^2 + sigma_z^2)
```

The rendered envelope radius is:

```text
rendered_horizontal_radius = sigma_scale * horizontal_sigma
rendered_vertical_radius   = sigma_scale * vertical_sigma
```

## Optional Fields

Wind estimates:

| Quantity | Preferred column | Units |
| --- | --- | --- |
| Wind X | `wind_x_mps` | meters/second |
| Wind Y | `wind_y_mps` | meters/second |
| Wind Z | `wind_z_mps` | meters/second |
| Wind sigma norm | `wind_sigma_norm_mps` | meters/second |

GPS fusion evidence:

| Quantity | Preferred column | Units |
| --- | --- | --- |
| GPS X/Y/Z | `gps_x_m`, `gps_y_m`, `gps_z_m` | meters |
| Accepted update flag | `gps_used` | boolean |
| Rejected measurement flag | `gps_rejected` | boolean |
| Position innovation | `gps_pos_innovation_m` | meters |
| Velocity innovation | `gps_vel_innovation_mps` | meters/second |
| GPS residual | `gps_residual_m` | meters |

Truth fields, when available:

| Quantity | Preferred column | Units |
| --- | --- | --- |
| Truth X/Y/Z | `truth_x_m`, `truth_y_m`, `truth_z_m` | meters |
| Truth wind X/Y/Z | `truth_wind_x_mps`, `truth_wind_y_mps`, `truth_wind_z_mps` | meters/second |

Audit fields:

| Quantity | Preferred column |
| --- | --- |
| Audit label | `audit_label` |
| Rationale | `rationale` |

If audit labels are absent, the tool derives readable fallback labels such as
`GPS update used`, `GPS rejected`, `wind uncertainty high`, `position
uncertainty high`, `covariance incomplete`, and `inertial only`.

## Exporter Strictness

`export_landing_telemetry_csv.py` is intentionally strict by default because its
purpose is to produce real EKF/GPS/wind telemetry for the dispersion diagnostic.
It fails if estimated position, position uncertainty, wind estimates, wind
uncertainty, or run-level GPS evidence are absent. GPS may still be intermittent
within a valid run. For partial bring-up logs, these gates
can be relaxed explicitly:

```powershell
python tools\analysis\export_landing_telemetry_csv.py `
  --input path\to\partial_log.csv `
  --output .codex_build\landing_dispersion_envelope\partial_canonical.csv `
  --allow-missing-gps `
  --allow-missing-wind `
  --allow-missing-wind-sigma
```

Truth data may be carried in the main input file or supplied separately with
`--truth-input`. Timestamp joins use the nearest truth sample within
`--truth-tolerance-s`; row-aligned simulation exports may use
`--truth-join row-index`.

## Figure Interpretation

The main 3D panel shows the estimated trajectory as a solid path. Translucent
cross-sections along the path show the position uncertainty envelope. GPS
samples are plotted as accepted or rejected evidence. Wind vectors are drawn at
downsampled intervals using the configured vector scale so they remain readable.

The wind subplot shows `wx`, `wy`, `wz`, and wind-speed norm. The uncertainty
subplot shows position sigma norm, horizontal sigma, vertical sigma, and wind
sigma norm. The GPS subplot shows position innovation, velocity innovation when
available, residual, and accepted/rejected flags. The audit timeline summarizes
state-estimation classifications over the run.

The summary block reports row count, duration, GPS accepted/rejected counts,
final position, final wind estimate, maximum position and wind uncertainty, truth
RMSE when truth exists, observed labels, final label, and final rationale.

## Relationship To The FPGA Workflow

The current FPGA-side CaelumFusion visualization bundle is focused on raw sensor
snapshots, derived altitude, vertical speed, attitude, heading, authority phase,
and health metadata. It does not currently expose GPS measurements, EKF
covariance, or wind-state estimates in the synthesizable bundle. This tool is
therefore placed under `tools/analysis/` and accepts exported host-side
simulation or EKF telemetry CSV. If future RTL or firmware exports these fields,
the CSV aliases above should be used as the stable host-side schema. The
separate exporter is the preferred integration boundary: bring new flight
software, simulation, or firmware logs into `flight_canonical.csv` first, then
render the diagnostic from that canonical host-side artifact. Do not widen the
synthesizable VGA bundle just to carry analysis-only covariance, GPS residual,
or truth fields.

Because this feature is host-side only, it has no intended LUT, register, DSP,
BRAM, timing, CDC, or bitstream impact.
