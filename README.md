# CaelumFusion Flight Control System

CaelumFusion Flight Control System is a Vivado RTL project for a Basys 3 class
flight-control visualization, sensor-integration, and bench-instrumentation
stack. The repository is organized around a board-facing VGA top module,
deterministic raw telemetry snapshots, explicit source provenance, focused
testbenches, and documentation that separates implemented behavior from
planned or bench-only evidence.

The active hardware image is not just a display demo. It is an FPGA-side
engineering instrument for observing avionics data quality: validity, status,
freshness, sequence alignment, fault flags, source flags, diagnostic injection,
bus counters, and frame-stable VGA views are first-class outputs.

## Current Baseline

| Item | Current contract |
| --- | --- |
| Board class | Digilent Basys 3 / Artix-7 |
| FPGA part | `xc7a35tcpg236-3` |
| Vivado project | `CaelumFusion_Flight_Control_System.xpr` |
| Canonical top | `caelumfusion_top_vga` |
| Constraint file | `CaelumFusion_Flight_Control_System.srcs/constrs_1/new/Basys-3-Master.xdc` |
| Display output | VGA, 640 x 480 timing |
| Primary RTL tree | `CaelumFusion_Flight_Control_System.srcs/sources_1/new/` |
| Simulation tree | `CaelumFusion_Flight_Control_System.srcs/sim_1/new/` |
| Tooling | Vivado, `xvlog`/`xelab`/`xsim`, PowerShell, Python 3 |
| License | Restrictive all-rights-reserved notice in `LICENSE` |

Tool-version boundary: the checkout path is under a Vivado 2023.2 workspace, but
the `.xpr` currently carries Vivado 2025.1.1 project metadata. Vivado 2023.2 can
compile focused RTL files, but the project file itself should be opened and
validated with a version that understands the current `.xpr` metadata before
making project-level synthesis or timing claims.

## What This Repository Contains

This repository tracks hand-authored project inputs:

- Verilog RTL, headers, and testbenches.
- Basys 3 XDC constraints.
- Vivado project metadata used as a file-set contract.
- Tcl flows for focused synthesis, implementation, and bitstream export.
- Python and WaveForms scripts for bench decoding and analysis.
- External MCU bridge firmware examples.
- Engineering documentation, bring-up guides, release checklists, and LaTeX
  source documents.

This repository intentionally excludes generated Vivado state and heavy build
outputs such as `.runs/`, `.sim/`, `.cache/`, `.hw/`, `.ip_user_files/`,
`.Xil/`, `xsim.dir/`, WDB files, journals, logs, checkpoints, and bitstreams.

## System Architecture

The design is built as a set of explicit evidence producers feeding a single
board-level integration top. Runtime controls select which evidence is live,
which views are shown, and which diagnostics are intentionally injected.

```mermaid
flowchart LR
    subgraph Board["Basys 3 Board Boundary"]
        Switches["Switches and Buttons"]
        VGA["VGA Connector"]
        JA["JA Shared I2C Header"]
        JB["JB SPI / GPIO Header"]
        JXADC["JXADC UART Bridge Pins"]
    end

    subgraph Top["caelumfusion_top_vga"]
        Sync["3-FF Control Synchronizers"]
        I2C["rocket_i2c_suite_top"]
        SPI["rocket_spi_suite_top"]
        UART["teensy_uart_range_bridge"]
        BenchMag["mag1_bench_snapshot_source"]
        Faults["snapshot_fault_injector"]
        Extension["sensor_extension_hub"]
        RenderCtl["caelumfusion_vga_render_control"]
        Science["caelumfusion_science_page_vga"]
        Compass["planar_compass_truth_page_vga"]
        Viz["flight_visualizer_pix"]
    end

    Switches --> Sync
    Sync --> I2C
    Sync --> SPI
    Sync --> UART
    Sync --> BenchMag
    Sync --> Faults
    JA <--> I2C
    JB <--> SPI
    JXADC --> UART
    I2C --> Extension
    SPI --> Extension
    UART --> Extension
    BenchMag --> Extension
    Faults --> Extension
    Extension --> RenderCtl
    RenderCtl --> Viz
    RenderCtl --> Compass
    RenderCtl --> Science
    Viz --> VGA
    Compass --> VGA
    Science --> VGA
```

### Evidence Pipeline

All sensor data is carried as explicit evidence, not as implicit truth. A
snapshot is useful only when its validity, status, age, sequence, payload, and
source provenance are understood together.

```mermaid
flowchart TB
    subgraph Producers["Raw Evidence Producers"]
        BMP["BMP / pressure evidence"]
        ACC["ACC evidence"]
        MAG0["MAG0 heading evidence"]
        PWR["PMON1 power evidence"]
        MAG1["MAG1 physical or bench evidence"]
        RNG["External MCU range evidence"]
        GNSS["Future GNSS snapshot source"]
    end

    subgraph Contracts["Snapshot Contracts"]
        RawBank["valid + status + seq + age + payload"]
        Provenance["real / bridge / replay / synthetic flags"]
        FaultBits["fault flags and counters"]
    end

    subgraph Consumers["Deterministic Consumers"]
        Hub["sensor_extension_hub"]
        Blackbox["blackbox_frame_packer"]
        Pages["VGA HUD, compass, science, integrity pages"]
        Host["Host and bench decoders"]
    end

    BMP --> RawBank
    ACC --> RawBank
    MAG0 --> RawBank
    PWR --> RawBank
    MAG1 --> Provenance
    RNG --> Provenance
    GNSS --> Provenance
    RawBank --> Hub
    Provenance --> Hub
    FaultBits --> Hub
    Hub --> Blackbox
    Hub --> Pages
    Blackbox --> Host
```

### Clocking And CDC Model

The top-level rendering path crosses from system-domain evidence into a pixel
domain. Crossings are modeled explicitly with synchronizers, snapshot registers,
and bundle/toggle CDC modules. Display behavior should remain frame-stable and
active-video gated.

```mermaid
flowchart LR
    subgraph SYS["SYS Domain"]
        SensorJobs["I2C / SPI / UART jobs"]
        SnapRegs["snapshot_regs and derived_snapshot_regs"]
        ExtHub["sensor_extension_hub"]
        ViewCtl["view selection and policy gates"]
    end

    subgraph CDC["CDC Boundary"]
        BitSync["sync_bit_3ff"]
        WordSync["cdc_word_toggle"]
        BundleSync["snapshot_cdc_bundle / telemetry_pix_snapshot_bank"]
    end

    subgraph PIX["PIX Domain"]
        Timing["vga_timing_640x480_60"]
        PageMux["flight_vga_page_mux_pix"]
        Renderers["HUD / compass / science renderers"]
        RGB["hsync, vsync, rgb"]
    end

    SensorJobs --> SnapRegs
    SnapRegs --> ExtHub
    ViewCtl --> BitSync
    ExtHub --> BundleSync
    BitSync --> PageMux
    BundleSync --> Renderers
    Timing --> PageMux
    PageMux --> Renderers
    Renderers --> RGB
```

## Implemented Scope And Non-Scope

| Area | Implemented in this repo | Explicitly not claimed |
| --- | --- | --- |
| VGA instrumentation | Frame-stable VGA views, render control, compact science pages, compass/MAG evidence page, diagnostic overlays | HDMI output from this Basys 3 build |
| Sensor transport | Shared I2C jobs, SPI jobs, external UART packet ingress scaffold | Autonomous sensor fusion or flight authority from these sensors |
| MAG0/MAG1 | MAG0 planar heading evidence; MAG1 physical/bench evidence and disagreement metrics | Tilt-compensated or fused heading |
| PMON1 | SW10-gated raw power-bank evidence | Power-control authority |
| Range bridge | Default-off UART packet ingress and MCU producer example | Physical rangefinder driver or range-to-altitude fusion |
| GNSS/nav/wind | Contract placeholders and guarded snapshot sources | Live EKF, GNSS navigation, or wind estimator unless explicitly wired and validated |
| Black-box logging | Deterministic frame packer scaffold | On-FPGA filesystem or SD-card ownership |
| Release evidence | Source, scripts, benches, docs, and checklists | Current timing closure unless a fresh matching-tool run is cited |

## Repository Map

```text
.
|-- CaelumFusion_Flight_Control_System.xpr
|-- CaelumFusion_Flight_Control_System.srcs/
|   |-- constrs_1/new/
|   |   `-- Basys-3-Master.xdc
|   |-- sources_1/new/
|   |   |-- caelumfusion_top_vga.v
|   |   |-- rocket_i2c_suite_top.v
|   |   |-- rocket_spi_suite_top.v
|   |   |-- sensor_extension_hub.v
|   |   |-- caelumfusion_vga_render_control.v
|   |   |-- caelumfusion_science_page_vga.v
|   |   |-- planar_compass_truth_page_vga.v
|   |   |-- flight_visualizer_pix.v
|   |   `-- telemetry_defs_vh.vh
|   `-- sim_1/new/
|       |-- tb_i2c_suite_regression_all3_real_engine.v
|       |-- tb_sensor_extension_hub.v
|       |-- tb_snapshot_fault_injector.v
|       |-- tb_mag1_bench_snapshot_source.v
|       |-- tb_caelumfusion_science_page_ext_metadata.v
|       `-- tb_caelumfusion_render_control_switch_encoded.v
|-- docs/
|-- firmware/
|-- tools/
|   |-- vivado/
|   |-- waveforms/
|   `-- analysis/
|-- LICENSE
`-- README.md
```

## Primary RTL Entry Points

| File | Role |
| --- | --- |
| `caelumfusion_top_vga.v` | Board-facing integration top and active Vivado top module. |
| `rocket_i2c_suite_top.v` | Shared I2C sensor suite and optional extension-device paths. |
| `rocket_spi_suite_top.v` | SPI sensor suite. |
| `sensor_extension_hub.v` | Extension evidence summarizer, fault flags, MAG0/MAG1 metrics, range/air/env/sun/flow/log fields. |
| `caelumfusion_vga_render_control.v` | Runtime view selection, direct-select handling, page arbitration, diagnostic visibility policy. |
| `flight_visualizer_pix.v` | Main pixel-domain HUD renderer. |
| `caelumfusion_science_page_vga.v` | Compact science, wind, and integrity evidence pages. |
| `planar_compass_truth_page_vga.v` | Compass/MAG evidence page and redundant-magnetometer visualization. |
| `blackbox_frame_packer.v` | Versioned 32-bit black-box evidence stream packer. |
| `teensy_bridge_packet_ingress.v` | Fixed external-MCU packet ingress contract. |
| `teensy_uart_range_bridge.v` | 8N1 UART wrapper for the external-MCU range evidence path. |
| `nav_wind_snapshot_producer.v` | Future explicit nav/wind binding point. |
| `gnss_bridge_snapshot_source.v` | Future GNSS packet-to-snapshot source contract. |

## Runtime Controls

The detailed control contract lives in
`docs/CaelumFusion_Runtime_Control_Map.md`. The condensed board-facing map is:

### Buttons

| Control | RTL signal | Purpose |
| --- | --- | --- |
| BTNC | `rst` | Global reset. |
| BTNU | `btn_page_raw` | Next visualization page. |
| BTND | `btn_prev_raw` | Previous visualization page. |
| BTNR | `btn_direct_compass_raw` | Direct-select request or compass request, depending on build mode. |
| BTNL | unused | Intentionally unassigned. |

### Switches

| Control | RTL signal | Purpose |
| --- | --- | --- |
| SW0 | `sw_arm_raw` | Software arm visualization gate. |
| SW1 | `sw_policy_enable_raw` | Policy-enable visualization gate. |
| SW2 | `sw_selftest_raw` | Self-test stimulus and synthetic extension diagnostics. |
| SW3 | `sw_mag1_bench_raw` | Synthetic MAG1 bench publication gate when compiled in. |
| SW4 | `sw_compass_page_raw` | Compass/MAG evidence page hold. |
| SW5 | `sw_history_freeze_raw` | Freeze history writes for inspection. |
| SW6 | `sw_log_diag_raw` | Black-box diagnostic requests and deliberate fault injection with SW2. |
| SW7 | `sw_lis3dh_i2c_acc_raw` | LIS3DH I2C accelerometer path gate. |
| SW8 | `sw_adxl362_spi_acc_raw` | ADXL362 SPI accelerometer path gate. |
| SW9 | `sw_cmps2_mmc3416_mag_raw` | CMPS2/MMC3416 MAG0 path gate. |
| SW10 | `sw_pmon1_pwr_raw` | PMON1 power telemetry gate. |
| SW11 | `sw_mag1_offset_x_raw` | MAG1 bench offset X or direct-view bit 0. |
| SW12 | `sw_mag1_offset_y_raw` | MAG1 bench offset Y or direct-view bit 1. |
| SW13 | `sw_mag1_offset_z_raw` | MAG1 bench offset Z or direct-view bit 2. |
| SW14 | `sw_compass_default_raw` | Compass/MAG default or hold companion. |
| SW15 | `sw_ext_i2c_raw` | Optional extension I2C group and external-UART bridge gate. |

SW11, SW12, and SW13 have collision arbitration. MAG1 bench mode and deliberate
fault injection own those switches before encoded direct-view selection. See the
runtime control map before using BTNR direct navigation during bench diagnostics.

## Sensor And Evidence Contracts

### I2C

The I2C suite uses the shared JA bus and an open-drain drive model. The engine
releases high and drives low; bench captures should treat high level as pull-up
or released bus state, not as push-pull high drive.

Representative I2C evidence paths include:

- BMP585 pressure/temperature evidence.
- LIS3DH acceleration evidence.
- CMPS2/MMC3416 MAG0 evidence.
- PMON1 power telemetry evidence at `7'h38`.
- Optional LIS2MDL/MAG1 evidence at `7'h1E`.
- Optional HYGRO/GYRO extension evidence.

### SPI

The SPI suite supports accelerometer and related sensor jobs through explicit
mode-0 style engines and job arbitration. SPI paths are selected through the
compiled top-level build and runtime switch gates.

### External MCU UART Bridge

The external-MCU bridge keeps the historical `teensy_*` RTL names, but the
active bench producer in this repository is the EK-TM4C123GXL UART1 firmware
example:

```text
firmware/tm4c123gxl_bridge_range_producer/main.c
```

The first implemented packet target is range/AGL evidence. The bridge remains
default-off unless `USE_TEENSY_UART_RANGE_BRIDGE` is enabled and SW15 gates the
accepted packets.

### MAG1 Provenance

MAG1 has two deliberately separated producers:

- Physical LIS2MDL/MAG1 through the shared I2C suite, tagged as real evidence.
- Synthetic MAG1 bench publication through `mag1_bench_snapshot_source.v`,
  tagged with `EXT_SRC_SYNTHETIC_BIT`.

Neither path changes `der_heading_mdeg`. The live heading remains MAG0 planar
`atan2(MY, MX)` until a separate fusion and tilt-compensation contract exists.

## VGA View Model

The render-control layer chooses among the main HUD, compass evidence, self-test
view, sensor diagnostics, and science pages. The important design rule is that
the view is an engineering instrument: stale, invalid, synthetic, and missing
data must remain visually distinguishable from live validated evidence.

```mermaid
stateDiagram-v2
    [*] --> FlightHUD
    FlightHUD --> CompassTruth: page or direct request
    CompassTruth --> ScienceExplain: page request
    ScienceExplain --> ScienceWind: page request
    ScienceWind --> ScienceIntegrity: page request
    ScienceIntegrity --> SensorDiag: page request
    SensorDiag --> FlightHUD: page request
    FlightHUD --> SelfTestHUD: SW2 self-test
    SelfTestHUD --> SensorDiag: SW2 + SW6 fault injection
    SensorDiag --> FlightHUD: clear diagnostic mode
```

## Verification Strategy

Use focused checks before broad project builds. A good verification sequence is:

1. Check the repository is clean except for the files intentionally under test.
2. Confirm new RTL or benches are present in the Vivado `.xpr` file set.
3. Run narrow `xvlog` syntax checks on changed RTL and its direct dependencies.
4. Elaborate and run focused benches in a fresh simulation work library.
5. Open the Vivado project with the matching project version and validate file
   sets.
6. Only then run synthesis, implementation, timing, and DRC reports.

```mermaid
flowchart TB
    Edit["RTL / XDC / doc edit"]
    FileSet["Project file-set check"]
    Syntax["Focused xvlog"]
    Sim["Focused xelab + xsim benches"]
    Project["Vivado project open and file-set validation"]
    Synth["Synthesis"]
    Impl["Implementation"]
    Release["Release evidence record"]

    Edit --> FileSet
    FileSet --> Syntax
    Syntax --> Sim
    Sim --> Project
    Project --> Synth
    Synth --> Impl
    Impl --> Release
```

### Recently Useful Focused Benches

| Bench | Purpose |
| --- | --- |
| `tb_i2c_suite_regression_all3_real_engine` | Shared I2C engine guardrail for multiple sensor paths. |
| `tb_rocket_i2c_suite_mag1_physical` | LIS2MDL/MAG1 physical path through the I2C suite. |
| `tb_pmon1_i2c_job` | PMON1 transaction sequence and failure behavior. |
| `tb_rocket_i2c_suite_pmon1` | Suite-level PMON1 gating and NACK/error behavior. |
| `tb_sensor_extension_hub` | Extension evidence, MAG1 metadata, source flags, and fault flags. |
| `tb_snapshot_fault_injector` | Deliberate diagnostic fault injection behavior. |
| `tb_mag1_bench_snapshot_source` | Default-off synthetic MAG1 bench publication contract. |
| `tb_caelumfusion_render_control_switch_encoded` | Encoded switch view selection and arbitration. |
| `tb_caelumfusion_science_page_ext_metadata` | Pixel-level science-page extension-metadata smoke test. |
| `tb_teensy_uart_range_bridge` | UART bridge packet handling and diagnostics. |

### Project File-Set Contract

The `.xpr` is part of the build contract. A file existing on disk is not enough
for project-mode builds. When adding an RTL module or bench:

- Add it under the correct `sources_1` or `sim_1` tree.
- Register it in `CaelumFusion_Flight_Control_System.xpr`.
- Confirm it appears in Vivado's file set.
- Re-check that no `UserDisabled` or stale `IS_ENABLED` metadata hides an active
  source.

## Common Commands

Run these from the repository root unless noted otherwise.

### Inspect Repository State

```powershell
git status --short --branch
git log --oneline --decorate --max-count=5
```

### Open The Project

Prefer the Vivado version that matches the current `.xpr` metadata:

```powershell
& "C:\Xilinx\2025.1.1\Vivado\bin\vivado.bat" `
  "CaelumFusion_Flight_Control_System.xpr"
```

For focused Verilog checks that do not need to open the project, Vivado 2023.2
tool executables may still be useful if they are the intended simulator runtime:

```powershell
& "C:\Xilinx\Vivado\2023.2\bin\xvlog.bat" --relax `
  -i "CaelumFusion_Flight_Control_System.srcs\sources_1\new" `
  "CaelumFusion_Flight_Control_System.srcs\sources_1\new\caelumfusion_science_page_vga.v" `
  "CaelumFusion_Flight_Control_System.srcs\sim_1\new\tb_caelumfusion_science_page_ext_metadata.v"
```

### Elaborate And Run A Focused Bench

```powershell
& "C:\Xilinx\Vivado\2023.2\bin\xelab.bat" --relax --debug typical `
  tb_caelumfusion_science_page_ext_metadata `
  -snapshot tb_caelumfusion_science_page_ext_metadata_snap

& "C:\Xilinx\Vivado\2023.2\bin\xsim.bat" `
  tb_caelumfusion_science_page_ext_metadata_snap -runall
```

### Run Scripted Synthesis Or Implementation

Do not treat these commands as evidence unless the run completes and the
generated reports are archived with the tool version, commit hash, and warning
review.

```powershell
& "C:\Xilinx\2025.1.1\Vivado\bin\vivado.bat" -mode batch `
  -source "tools\vivado\synth_caelumfusion_top_vga.tcl"

& "C:\Xilinx\2025.1.1\Vivado\bin\vivado.bat" -mode batch `
  -source "tools\vivado\impl_caelumfusion_top_vga_from_synth.tcl"
```

### Decode WaveForms I2C Captures

```powershell
python tools\decode_waveforms_i2c.py `
  "captures\basys3_ja_i2c_capture.csv" `
  --scl i2c_scl --sda i2c_sda `
  --csv-out "captures\basys3_ja_i2c_decoded.csv" `
  --json-out "captures\basys3_ja_i2c_decoded.json"
```

## Documentation Index

| Document | Use it for |
| --- | --- |
| `docs/CaelumFusion_Runtime_Control_Map.md` | Live Basys 3 button/switch and parameter contract. |
| `docs/CaelumFusion_Basys3_Pmod_Wiring_Guide.md` | Board wiring, Pmod headers, pullups, safety notes, and bench bring-up. |
| `docs/CaelumFusion_Discovery3_Basys3_Pmod_Instrumentation_Guide.md` | Discovery 3 / WaveForms capture workflow. |
| `docs/CaelumFusion_WaveForms_Decoder_Protocol_Scripts.md` | WaveForms custom decoder and script usage. |
| `docs/CaelumFusion_Extension_Milestones.md` | Implemented extension evidence, MAG1, range, GNSS, black-box, and limitations. |
| `docs/CaelumFusion_TM4C123GXL_UART_Bridge_Bringup.md` | TM4C123GXL external bridge bring-up. |
| `docs/CaelumFusion_Teensy_UART_Bridge_Bringup.md` | Historical Teensy bridge bring-up context. |
| `docs/CaelumFusion_Landing_Dispersion_Envelope.md` | Host analysis and landing-dispersion tooling. |
| `docs/CaelumFusion_L1_Avionics_Release_Checklist.md` | Release gate checklist for flight-image evidence. |
| `docs/bench_captures/index.md` | Bench-capture inventory and evidence status. |

## Bench And Hardware Bring-Up Notes

- Treat all Pmod and JXADC wiring as a contract. Confirm pin mapping against the
  XDC and wiring guide before applying power.
- Never connect 5 V UART directly to FPGA inputs.
- For I2C, verify idle-high level, low-drive behavior, pullups, ACK/NACK
  behavior, and no contention before trusting captured data.
- Keep raw capture exports and decoded protocol outputs as separate artifacts.
- When a capture is incomplete or starts mid-frame, label it as partial evidence
  instead of treating decoder errors as sensor failures.
- Distinguish bench evidence from flight evidence. Synthetic MAG1, simulated
  range, self-test stimulus, and deliberate fault injection must remain visibly
  tagged.

## Release Evidence Boundary

Before a bitstream or timing claim is published, record:

- Git commit hash.
- Vivado version and build number.
- Top module and FPGA part.
- Exact Tcl commands.
- Active source and constraint manifest.
- Simulation benches and PASS logs.
- Synthesis reports.
- Implementation reports.
- Timing summary with setup, hold, and pulse-width status.
- DRC report with every warning fixed or explicitly waived.
- Hardware bench evidence for reset, sensor buses, display output, power, and
  integration safety.

The release checklist in `docs/CaelumFusion_L1_Avionics_Release_Checklist.md`
is the source of truth for flight-style signoff. Older baseline report numbers
inside that checklist are historical unless regenerated against the current
commit and matching Vivado version.

## Known Risks And Review Points

| Risk | Current handling |
| --- | --- |
| Vivado version skew | Project metadata is currently newer than the 2023.2 workspace path. Use matching Vivado for project-level validation. |
| Generated artifacts excluded | Bitstreams, checkpoints, logs, WDBs, and run directories are intentionally not committed. Archive them separately for releases. |
| SCL open-drain interpretation | I2C documentation and captures must treat high as released/pulled-high, not push-pull driven high. |
| Synthetic evidence confusion | Synthetic MAG1 and diagnostic data are explicitly source-flagged and must not be cited as physical flight evidence. |
| Navigation/wind binding | Nav/wind fields are guarded until real estimator sources are wired. Do not infer wind or navigation truth from unrelated raw evidence. |
| Freshness/stale data | Every consumer should preserve age/status semantics rather than silently reusing stale-good payloads. |
| Project file-set drift | New files must be registered in the `.xpr`; disabled project entries can break project-mode builds even when files exist on disk. |

## Development Workflow

1. Read the relevant RTL, XDC, docs, and benches before editing.
2. Keep changes small enough to review.
3. Preserve single-writer ownership of snapshot banks and status fields.
4. Keep CDC boundaries explicit.
5. Add or update focused benches when behavior changes.
6. Register source and bench files in the `.xpr`.
7. Run narrow simulation or file-set checks before broad synthesis.
8. Update docs when interfaces, switch maps, packet formats, or evidence fields
   change.
9. Commit source changes and exclude generated Vivado outputs.

## Recommended Next Engineering Steps

1. Version-normalize the Vivado project flow: choose whether the active project
   is 2025.1.1 or needs a 2023.2-compatible project copy.
2. Add a checked-in Tcl file-set validation script so project registration can
   be verified without ad hoc commands.
3. Run fresh-library simulations for the newly registered benches and archive
   PASS logs outside the source tree.
4. Re-run synthesis and implementation with the chosen Vivado version, then
   update the release checklist with current commit-specific evidence.
5. Add a lightweight documentation CI check for Markdown links and Mermaid fence
   integrity once the repository hosting workflow is settled.
