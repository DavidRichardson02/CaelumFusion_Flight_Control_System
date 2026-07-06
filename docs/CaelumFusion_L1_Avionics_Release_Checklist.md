# CaelumFusion L1 Avionics Release Checklist

This checklist is the release gate for the FPGA avionics image intended for the Messier-76 L1 flight article. It is written for the `caelumfusion_top_vga` build on Basys 3 / Artix-7 (`xc7a35tcpg236-3`) and should be completed before generating or flying a bitstream.

## 1. Source And Tool Baseline

- Record the repository commit hash or, if `git` is unavailable, SHA-256 hashes for every modified RTL, XDC, Tcl, and checklist file.
- Record the Vivado version, build number, host, top module, FPGA part, and XDC file used for the build.
- Confirm the active source set contains the intended I2C top implementation: `rocket_i2c_suite_top.v` must declare `module rocket_i2c_suite_top`, not the SPI top.
- Confirm there are no duplicate top modules or shadow copies in the active Vivado source set.
- Archive the exact synthesis and implementation commands. The source-controlled baseline scripts are:
  - `tools/vivado/synth_caelumfusion_top_vga.tcl`
  - `tools/vivado/impl_caelumfusion_top_vga_from_synth.tcl`

## 2. Compile, Simulation, And Elaboration

- Run a clean `xvlog` pass over the active Verilog-2001 source set.
- Run top-level elaboration for `caelumfusion_top_vga` with the same Vivado simulator libraries used by implementation.
- Archive simulator logs and mark each required testbench `PASS`.
- Minimum RTL tests before release:
  - Reset release leaves all validity, freshness, counters, and page/state registers deterministic.
  - I2C job mux asserts only one active sensor job at a time and propagates each busy flag.
  - CMPS2/MMC3416 job initialization, SET, measurement, status polling, and six-byte burst-read contract pass in `tb_mmc3416_i2c_job`.
  - CMPS2/MMC3416 multi-transaction continuation through the shared I2C arbiter/mux passes in `tb_mmc3416_i2c_mux_arbiter_continuation`.
  - Sensor age counters reset on their own snapshot commit and saturate instead of wrapping.
  - SYS-to-PIX visualization publish produces one coherent snapshot update per semantic change.
  - CDC toggle synchronizers do not produce duplicate destination pulses for one source publish event.

## 3. Synthesis And Implementation Reports

- Run:

```tcl
vivado -mode batch -source tools/vivado/synth_caelumfusion_top_vga.tcl
vivado -mode batch -source tools/vivado/impl_caelumfusion_top_vga_from_synth.tcl
```

- Archive these generated artifacts:
  - `.codex_build/synth_baseline/caelumfusion_top_vga_synth.dcp`
  - `.codex_build/synth_baseline/caelumfusion_top_vga_timing_synth.rpt`
  - `.codex_build/synth_baseline/caelumfusion_top_vga_utilization_synth.rpt`
  - `.codex_build/impl_baseline/caelumfusion_top_vga_impl_routed.dcp`
  - `.codex_build/impl_baseline/caelumfusion_top_vga_timing_impl.rpt`
  - `.codex_build/impl_baseline/caelumfusion_top_vga_drc_impl.rpt`
  - `.codex_build/impl_baseline/caelumfusion_top_vga_utilization_impl.rpt`
- Required timing gate:
  - Setup WNS >= 0 ns, setup TNS = 0 ns, and zero setup failing endpoints.
  - Hold WHS >= 0 ns, hold THS = 0 ns, and zero hold failing endpoints.
  - Pulse-width checks have no failing endpoints.
  - No unconstrained functional clock-domain crossing is accepted without a named CDC mechanism and scoped constraint.
- Required DRC gate:
  - Zero errors.
  - Zero critical warnings.
  - Every warning is either fixed or explicitly waived with owner, rationale, affected hierarchy, and flight risk.

## 4. Hardware Bench Evidence

- Power-on reset test: capture the reset waveform or bench log and confirm deterministic boot.
- I2C electrical test: verify SCL/SDA idle high, valid open-drain low drive, no bus contention, and sensor ACK behavior on the assembled avionics harness.
- CMPS2 bench test: verify address `0x30` ACK on Basys 3 JA3/JA4, 10 Hz magnetometer snapshot updates, valid heading freshness, and stale/missing-sensor reporting after a powered-down disconnect test.
- Sensor freshness test: verify BMP, accelerometer, and magnetometer update counters or age fields change as expected and stale data is visibly distinguishable from valid live data.
- Visualization test: record VGA/HUD output after boot, during sensor updates, and during induced stale/error conditions.
- Serial/console test: capture any launch-day operator telemetry output and confirm units, validity flags, and update cadence.
- Power test: verify avionics supply voltage under expected battery, switch, and load conditions.
- Mechanical integration test: verify connector retention, strain relief, mounting, vibration-sensitive wiring, and arming access.

## 5. Flight-Day No-Go Criteria

The flight is no-go if any of the following are true:

- The flown bitstream hash does not match the release record.
- Any required simulation, synthesis, implementation, timing, DRC, or hardware bench artifact is missing.
- Vivado reports any implementation error, critical warning, negative setup slack, negative hold slack, timing violation, or unreviewed DRC warning.
- The active top-level source set does not match the release source manifest.
- Any CDC exception is broad, stale, or not tied to a reviewed CDC circuit.
- SCL/SDA show contention, stuck-low behavior, missing pullups, missing sensor ACK, or stale sensor data that cannot be explained.
- Reset, boot, arming, telemetry, or visualization behavior is not repeatable on the integrated vehicle.
- Battery voltage, switch function, continuity, retention, mounting, or environmental/range safety conditions fail preflight inspection.
- The RSO, mentor, or field safety process rejects any part of the avionics, recovery, propulsion, or vehicle integration configuration.

## 6. Current Engineering Baseline

This snapshot was produced on 2026-06-22 from the scripted flow in `tools/vivado`.

- Tool: Vivado v2023.2 (win64), build 4029153.
- Top: `caelumfusion_top_vga`.
- Part: `xc7a35tcpg236-3`.
- Routed checkpoint: `.codex_build/impl_baseline/caelumfusion_top_vga_impl_routed.dcp`.
- Routed timing: WNS = +2.283 ns, TNS = 0.000 ns, WHS = +0.097 ns, THS = 0.000 ns, zero setup/hold failing endpoints.
- Routed DRC: 42 warnings, 0 errors, 0 critical warnings.
- DRC warning classes: DSP pipeline advisories in `u_viz/u_flight_visualizer_pix`, and `RPBF-3` on `scl`.
- Utilization: 17,779 LUTs (85.48%), 8,313 registers (19.98%), 11 DSPs (12.22%), 0 BRAM tiles.
- Git status was not available because `git` was not on the PowerShell `PATH`; SHA-256 hashes are used below for traceability.

Current design-input SHA-256 hashes:

| File | SHA-256 |
| --- | --- |
| `CaelumFusion_Flight_Control.srcs/sources_1/new/rocket_i2c_suite_top.v` | `1E704EEFA18F83F10D591B423B60F21C4F768574B293E749E4D662B09786425A` |
| `CaelumFusion_Flight_Control.srcs/sources_1/new/flight_viz_suite_top.v` | `2DB81A65FEE26212145C78D7C76F70D11EAA1AE1EF3A1F3A10BABD8DBB9E1FCC` |
| `CaelumFusion_Flight_Control.srcs/sources_1/new/flight_viz_bundle_cdc.v` | `628A0170C213AAD719D0DC730E51288FD640F805BC803F4D52D35761F68E5FD0` |
| `CaelumFusion_Flight_Control.srcs/sources_1/new/flight_viz_model_sys.v` | `0BCC845074499DDCEA9EE62D32E834C851544CD2995C9E31913D8D88AE748F17` |
| `CaelumFusion_Flight_Control.srcs/constrs_1/new/Basys-3-Master.xdc` | `78F4F19AAA03BE0EF2861CD2AE863E0E3A4B105EBF227D9F8221899A5001339B` |
| `tools/vivado/synth_caelumfusion_top_vga.tcl` | `249076C071FA9FFA7F16663AE91AD1DD1E9DA9C4DA0B3780F76DFA8F25671528` |
| `tools/vivado/impl_caelumfusion_top_vga_from_synth.tcl` | `54A69DE2DB68EF8E25C9C8DD2BAA54388DBD03681E0EA216FB6D9DF655082137` |
| `.codex_build/impl_baseline/caelumfusion_top_vga_impl_routed.dcp` | `961D305E69FAF1F9CB1BEC60A481936C584DB85A5A4E82CFFE145C4374A42416` |

Known baseline risks:

- `RPBF-3` remains on `scl`: the RTL declares SCL as `inout`, but the current I2C engine does not sample SCL. Before flight release, either document SCL as open-drain output-only and update the interface consistently, or add tested SCL sampling/clock-stretch behavior.
- DSP pipeline warnings remain in the pixel visualizer. Timing is clean after route, but these warnings should be waived explicitly or cleared if future display features consume more timing margin.
- Simulation PASS logs and hardware bench evidence are not represented by this scripted implementation run and must be added before flight signoff.

## 7. Release Signoff Record

- Release name:
- Date/time:
- Top module:
- FPGA part:
- Vivado version/build:
- Source hash or commit:
- Bitstream hash:
- Simulation log paths:
- Timing report path:
- DRC report path:
- Utilization report path:
- Hardware bench evidence paths:
- Known warnings/waivers:
- Final go/no-go decision:
