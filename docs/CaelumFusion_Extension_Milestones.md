# CaelumFusion Extension Milestones

## Redundant Magnetometer Evidence, First Pass

Implemented scope: raw evidence and visualization scaffolding only. The existing
derived heading contract is unchanged:

```text
der_heading_mdeg = planar atan2(MY, MX)
```

This is not tilt-compensated heading, not fused heading, and not a flight-control
authority input.

### RTL Location

- `lis2mdl_job.v` implements the physical LIS2MDL transaction sequence.
- `rocket_i2c_suite_top.v` exposes that LIS2MDL path as the physical MAG1
  evidence slot on the shared I2C engine.
- `mag1_bench_snapshot_source.v` publishes a default-disabled MAG1 raw snapshot.
- `sensor_extension_hub.v` computes MAG0/MAG1 evidence summary fields.
- `planar_compass_truth_page_vga.v` displays redundant-magnetometer evidence.
- `caelumfusion_top_vga*.v` instantiate the MAG1 producer and extension hub.
- `blackbox_frame_packer.v` remains the deterministic raw-frame scaffold for
  future external-MCU/host logging ownership.

### MAG1 Source

MAG1 now has two deliberately separated producers:

- Physical LIS2MDL/MAG1 evidence from `rocket_i2c_suite_top.v`, compiled by
  `USE_LIS2MDL_MAG1` and runtime-gated by `cfg_ext_i2c_en` / SW15. The path uses
  the shared I2C engine at address `7'h1E`, verifies `WHO_AM_I 8'h4F == 8'h40`,
  writes `CFG_REG_A 8'h60 = 8'h80`, writes `CFG_REG_C 8'h62 = 8'h01`, and reads
  `OUTX/Y/Z 8'h68..8'h6D` into the raw snapshot order `{Z, Y, X}`.
- Synthetic bench MAG1 evidence from `mag1_bench_snapshot_source.v`, compiled by
  `USE_MAG1_BENCH_SOURCE` and runtime-gated by SW3. It is tagged with
  `EXT_SRC_SYNTHETIC_BIT` and remains bench evidence only.

When a MAG1 producer is disabled or not selected, MAG1 reports:

- `mag1_valid = 0`
- `mag1_status = ST_NOT_INITIALIZED`
- `mag1_age_ms = 16'hFFFF`
- zero payload, calibration state, source flags, and checksum

When enabled, the physical source publishes real-source MAG1 evidence with
`EXT_SRC_REAL_BIT`, sequence, age, calibration placeholder, and checksum. The
bench source mirrors MAG0 raw `{MZ, MY, MX}` with optional signed offsets
`MAG1_BENCH_OFFSET_X/Y/Z`, preserves raw units, mirrors the MAG0 sequence by
default, and marks `EXT_SRC_SYNTHETIC_BIT`. Neither source changes
`der_heading_mdeg`; flight-facing heading remains MAG0 planar heading until an
explicit future fusion contract is introduced.

### Evidence Fields

The extension hub computes or carries:

- `ext_mag_delta_l1`: `abs(M0X-M1X)+abs(M0Y-M1Y)+abs(M0Z-M1Z)`
- `ext_mag_norm_primary`: MAG0 L1 norm
- `ext_mag_norm_secondary`: MAG1 L1 norm
- `ext_mag_norm_delta_l1`: absolute MAG0/MAG1 L1 norm difference
- `ext_mag_sequence_aligned`: MAG0 and MAG1 good, fresh, and same sequence
- `ext_mag_sector_delta`: 8-sector planar XY direction disagreement
- `ext_mag_disagreement`: thresholded vector disagreement flag
- `ext_mag_iron_residual`: placeholder equal to norm delta until host ellipse
  or calibrated residual math exists
- `ext_mag_cal_state`: MAG1 calibration-state placeholder
- `ext_mag_source_flags`: real/bridge/replay/synthetic provenance bits
- `ext_mag_bridge_checksum`: source checksum placeholder/checksum field

Thresholds remain parameterized in `sensor_extension_hub`: MAG freshness, norm
band, vector delta, and norm-delta limits.

### Reset, Valid, Stale, and Fault Behavior

- Reset clears all MAG1 and extension evidence fields to safe defaults.
- Disabled MAG1 is explicitly not initialized.
- Valid physical LIS2MDL/MAG1 data is status `ST_OK` only after the WHO/config
  sequence succeeds and a burst read commits.
- Physical LIS2MDL failures use the shared status-code contract:
  `ST_I2C_NACK`, `ST_I2C_TIMEOUT`, `ST_INTERNAL_OVERFLOW`,
  `ST_SENSOR_ID_MISMATCH`, or `ST_CONFIG_ERROR`.
- Valid MAG1 bench data is status `ST_OK` only when MAG0 is valid, status OK,
  and fresh.
- Stale MAG0 propagates `ST_STALE_REJECT` into the synthetic MAG1 bank.
- Missing MAG0 propagates `ST_MISSING_INPUT` and clears MAG1 valid.
- Pair missing, norm out-of-range, norm mismatch, vector disagreement, raw
  status error, and stale future banks are visible in extension fault flags.

### VGA Compass Evidence Page

Enable the optional page with `USE_COMPASS_TRUTH_PAGE != 0`; hold
`btn_page_raw` unless `COMPASS_TRUTH_PAGE_DEFAULT` is also enabled. The page now
shows:

- explicit `PLANAR HEADING ONLY` and `NOT TILT COMPENSATED` warnings
- MAG0 planar heading and raw vector trail
- MAG1 raw X/Y/Z, status, age, sequence, source/cal state, checksum
- MAG0/MAG1 norm bars
- vector L1 delta, norm delta, residual placeholder, sector delta
- sequence-alignment and disagreement indicators
- source freshness/status badges and bus-health counters

### Rangefinder Scaffold

`sensor_extension_hub` already reserves `rng_*` raw snapshot inputs and
publishes `ext_rng_height_cm`. The range bank is the first external-MCU
packet-ingress target. `teensy_uart_range_bridge.v` wraps the fixed packet
ingress with an 8N1 UART receiver and feeds the top-level hub, but the top
parameter `USE_TEENSY_UART_RANGE_BRIDGE` defaults to `0` so disconnected UART
noise cannot become range evidence:

- `rng_valid = 0`
- `rng_status = ST_NOT_INITIALIZED`
- `rng_age_ms = 16'hFFFF`

Do not fuse range into altitude until a physical range source and baro-vs-range
residual validation path are present.

### External MCU Fixed-Packet Range Ingress

`teensy_bridge_packet_ingress.v` is the first fixed binary packet contract for a
coprocessor link. It keeps the historical Teensy name to avoid RTL churn, but
the active producer is now EK-TM4C123GXL UART1. It is intentionally a decoded
packet ingress. `teensy_uart_range_bridge.v` is the first physical transport
wrapper: it accepts 8N1 UART bytes on the Basys-3 `JXADC` connector, verifies
frame sync, and presents one coherent packet per ready/valid handshake.

Implemented packet types:

- `PKT_TEENSY_HEARTBEAT` (`8'h50`): refreshes bridge heartbeat sequence and age.
- `PKT_TEENSY_RANGE_AGL` (`8'h51`): publishes one range/AGL raw snapshot bank.

Range payload contract:

- `pkt_payload[47:32]`: height above ground in centimeters.
- `pkt_payload[31:16]`: confidence / quality scalar.
- `rng_payload[15:0]`: bridge/source provenance flags after checksum validation.

Checksum contract:

```text
checksum = xor(TELEM_PKT_SYNC,
               {pkt_status,pkt_type},
               pkt_seq,
               pkt_timestamp_us[31:16],
               pkt_timestamp_us[15:0],
               pkt_payload[47:32],
               pkt_payload[31:16],
               pkt_payload[15:0],
               pkt_aux,
               pkt_source_flags)
```

The ingress exposes checksum-fault and unsupported-packet counters. Corrupt
range packets commit visible invalid `ST_CONFIG_ERROR` evidence and do not
publish stale-good range data. Missing or stale heartbeat evidence rejects range
updates with `ST_STALE_REJECT` when `REQUIRE_HEARTBEAT_FOR_DATA` is enabled.

Default physical pin contract:

- `teensy_uart_rx_raw`: JXADC pin 1 / XA1_P / FPGA package J3, input from
  TM4C PC5/U1TX/J4.05.
- `teensy_uart_tx`: JXADC pin 2 / XA2_P / FPGA package L3, optional output to
  TM4C PC4/U1RX/J4.04, currently idle high for future ACK/debug.
- JXADC pin 5 or 11: common ground. Do not backfeed power between boards.

`firmware/tm4c123gxl_bridge_range_producer/main.c` is the active LaunchPad-side
producer. It emits heartbeat and simulated range packets over UART1 at 115200
8N1 and includes ICDI virtual COM commands for checksum, stale-heartbeat,
out-of-range, low-confidence, and unsupported-packet validation. It is a bench
truth/transport producer, not a physical lidar or GNSS driver.

### Navigation / Wind Binding Guard

The live VGA nav/wind viewport is now explicitly unbound until real
EKF/GNSS/wind-estimator source signals exist in SYS domain. The compatibility
module `landing_nav_wind_observer.v` no longer derives downrange from altitude,
crossrange from optical flow, or wind from raw air/flow evidence. It reports:

- `nav_valid = 0`
- `nav_status = ST_MISSING_INPUT`
- `nav_age_ms = 16'hFFFF`
- `wind_valid = 0`
- `wind_status = ST_MISSING_INPUT`
- `wind_age_ms = 16'hFFFF`

`nav_wind_snapshot_producer.v` defines the real future binding point. It accepts
explicit EKF/local-navigation, GNSS, and wind-estimator fields, applies
valid/status/freshness gates, and publishes the existing compact `nav_*` and
`wind_*` VGA contract. Do not bind that producer to barometric altitude,
optical-flow deltas, pitot/raw airspeed, or synthetic data as if they were a
navigation or wind estimate.

### GNSS Bridge Snapshot Source

`gnss_bridge_snapshot_source.v` is the first real upstream source contract for
future navigation binding. It accepts one decoded external-MCU/host GNSS packet per
ready/valid handshake and publishes a single-writer `gnss_*` snapshot bank:

- fix metadata: `gnss_fix_type`, `gnss_num_sats`, `gnss_hdop_centi`
- position: `gnss_lat_e7`, `gnss_lon_e7`, `gnss_alt_cm_msl`
- velocity: `gnss_vel_n/e/d_cms`, `gnss_ground_speed_cms`, `gnss_course_mdeg`
- provenance: `gnss_source_flags`, `gnss_checksum`
- timing: `gnss_t_us`, `gnss_seq`, `gnss_age_ms`
- PPS evidence: `gnss_pps_seen`, `gnss_pps_seq`, `gnss_pps_age_ms`
- fault evidence: `gnss_checksum_fault_count`

The module verifies a deterministic XOR checksum over the decoded packet fields.
A checksum fault commits a visible `ST_CONFIG_ERROR` diagnostic snapshot and
increments the checksum-fault counter, but it does not publish corrupt
position/velocity as valid evidence. Stale fixes clear `gnss_valid` and report
`ST_STALE_REJECT`. PPS age is tracked independently so PPS can be made a future
validity requirement without changing the packet contract.

### Black-Box Logging Scaffold

`blackbox_frame_packer.v` emits a deterministic ready/valid 32-bit stream when
`USE_BLACKBOX_LOG` is enabled. Filesystem and SD-card ownership should remain on
the external MCU or host side. The FPGA side exposes raw-bank IDs, sequence, status,
ages, extension summary, log sequence, drop count, and MAG1 metadata. Host
decoders must branch on the version field before assuming frame length or word
meaning.

Current blackbox frame contract:

- Word 0: `{TELEM_PKT_SYNC, PKT_BLACKBOX_WORD, 8'h02}`.
- Word 1: `{frame_seq, 8'd29, 8'd0}`.
- Words 2-5: BMP time, `{seq, valid, status}`, payload high, payload low + age.
- Words 6-9: ACC time, `{seq, valid, status}`, payload high, payload low + age.
- Words 10-13: MAG0 time, `{seq, valid, status}`, payload high, payload low +
  age.
- Words 14-17: PWR time, `{seq, valid, status}`, payload high, payload low +
  age.
- Word 18: `{ext_valid, 7'd0, ext_status, ext_present_flags}`.
- Word 19: `{ext_fault_flags, ext_mag_delta_l1}`.
- Word 20: `{ext_mag_norm_primary, ext_mag_norm_secondary}`.
- Word 21: `{ext_rng_height_cm, ext_air_dp_pa}`.
- Word 22: `{ext_air_speed_cms, ext_env_temp_cdeg}`.
- Word 23: `{ext_env_rh_centi, ext_sun_luma}`.
- Word 24: `{ext_flow_dx, ext_flow_dy}`.
- Word 25: `{drop_count, ext_max_age_ms}`.
- Word 26: compact MAG1 metadata:
  - bits `[7:0]`: `ext_mag_source_flags`
  - bits `[15:8]`: `ext_mag_cal_state`
  - bits `[19:16]`: `ext_mag_sector_delta`
  - bit `20`: `ext_mag_disagreement`
  - bit `21`: `ext_mag_sequence_aligned`
  - bits `[31:22]`: reserved, zero
- Word 27: `{ext_mag_norm_delta_l1, ext_mag_iron_residual}`.
- Word 28: `{ext_mag_bridge_checksum, 16'd0}`.

The previous frame-v1 decoder assumption was a 26-word frame without words
26-28. Decoders should reject unknown versions or route `8'h02` to the 29-word
schema above. Future revisions should add a CRC field only after the frame
schema is frozen.

### Current Limitations

- Physical LIS2MDL/MAG1 is implemented and verified in simulation, but it
  remains a deliberate hardware-validation path: `USE_LIS2MDL_MAG1` controls
  elaboration and SW15 controls live requests/publication. Treat it as bench
  evidence until a physical capture verifies wiring, pullups, orientation, and
  environmental magnetic plausibility on the target board.
- The fixed-packet range ingress, default-off JXADC UART wrapper, and
  simulated-range TM4C123GXL producer are implemented, but no telemetry radio,
  SD sink, physical range sensor driver, or GNSS producer is connected in this
  pass.
- Hard/soft-iron calibration is evidence-only; there is no autonomous ellipse
  fit or correction loop in FPGA.
- `der_heading_mdeg` still uses MAG0 planar `atan2(MY, MX)`.
- No EKF, tilt compensation, pitot/range/GNSS fusion, landing dispersion
  overlay, or autonomous control authority is introduced. The nav/wind VGA
  contract is unavailable-safe until real estimator inputs are wired.

### Verification

Focused checks:

```powershell
& "C:\Xilinx\Vivado\2023.2\bin\xvlog.bat" --relax -i "C:/Xilinx/Vivado/2023.2/Vivado Projects/CaelumFusion_Flight_Control_System/CaelumFusion_Flight_Control_System.srcs/sources_1/new" `
  "C:/Xilinx/Vivado/2023.2/Vivado Projects/CaelumFusion_Flight_Control_System/CaelumFusion_Flight_Control_System.srcs/sources_1/new/blackbox_frame_packer.v" `
  "C:/Xilinx/Vivado/2023.2/Vivado Projects/CaelumFusion_Flight_Control_System/CaelumFusion_Flight_Control_System.srcs/sources_1/new/sensor_extension_hub.v" `
  "C:/Xilinx/Vivado/2023.2/Vivado Projects/CaelumFusion_Flight_Control_System/CaelumFusion_Flight_Control_System.srcs/sources_1/new/snapshot_fault_injector.v" `
  "C:/Xilinx/Vivado/2023.2/Vivado Projects/CaelumFusion_Flight_Control_System/CaelumFusion_Flight_Control_System.srcs/sources_1/new/mag1_bench_snapshot_source.v" `
  "C:/Xilinx/Vivado/2023.2/Vivado Projects/CaelumFusion_Flight_Control_System/CaelumFusion_Flight_Control_System.srcs/sources_1/new/planar_compass_truth_page_vga.v" `
  "C:/Xilinx/Vivado/2023.2/Vivado Projects/CaelumFusion_Flight_Control_System/CaelumFusion_Flight_Control_System.srcs/sim_1/new/tb_sensor_extension_hub.v" `
  "C:/Xilinx/Vivado/2023.2/Vivado Projects/CaelumFusion_Flight_Control_System/CaelumFusion_Flight_Control_System.srcs/sim_1/new/tb_snapshot_fault_injector.v" `
  "C:/Xilinx/Vivado/2023.2/Vivado Projects/CaelumFusion_Flight_Control_System/CaelumFusion_Flight_Control_System.srcs/sim_1/new/tb_mag1_bench_snapshot_source.v" `
  "C:/Xilinx/Vivado/2023.2/Vivado Projects/CaelumFusion_Flight_Control_System/CaelumFusion_Flight_Control_System.srcs/sim_1/new/tb_planar_compass_truth_page_ext.v"
```

Then elaborate and run the three focused testbenches:

```powershell
& "C:\Xilinx\Vivado\2023.2\bin\xelab.bat" --relax --debug typical tb_mag1_bench_snapshot_source -snapshot tb_mag1_bench_snapshot_source_snap
& "C:\Xilinx\Vivado\2023.2\bin\xsim.bat" tb_mag1_bench_snapshot_source_snap -runall

& "C:\Xilinx\Vivado\2023.2\bin\xelab.bat" --relax --debug typical tb_snapshot_fault_injector -snapshot tb_snapshot_fault_injector_snap
& "C:\Xilinx\Vivado\2023.2\bin\xsim.bat" tb_snapshot_fault_injector_snap -runall

& "C:\Xilinx\Vivado\2023.2\bin\xelab.bat" --relax --debug typical tb_sensor_extension_hub -snapshot tb_sensor_extension_hub_redundant_mag_snap
& "C:\Xilinx\Vivado\2023.2\bin\xsim.bat" tb_sensor_extension_hub_redundant_mag_snap -runall

& "C:\Xilinx\Vivado\2023.2\bin\xelab.bat" --relax --debug typical tb_planar_compass_truth_page_ext -snapshot tb_planar_compass_truth_page_ext_redundant_mag_snap
& "C:\Xilinx\Vivado\2023.2\bin\xsim.bat" tb_planar_compass_truth_page_ext_redundant_mag_snap -runall
```

Physical MAG1 focused check added for the LIS2MDL path:

- `tb_rocket_i2c_suite_mag1_physical` proves the physical MAG1 path through the
  shared I2C engine against a register-aware LIS2MDL model. The bench verifies
  SW15 gating, WHO/config/data-register traffic, `{Z,Y,X}` payload packing,
  `EXT_SRC_REAL_BIT`, checksum publication, and that MAG1 evidence does not
  change the flight-facing MAG0 heading reference. Current PASS evidence is
  `PASS: tb_rocket_i2c_suite_mag1_physical`.

```powershell
& "C:\Xilinx\Vivado\2023.2\bin\xelab.bat" --relax --debug typical tb_rocket_i2c_suite_mag1_physical -snapshot tb_rocket_i2c_suite_mag1_physical_snap
& "C:\Xilinx\Vivado\2023.2\bin\xsim.bat" tb_rocket_i2c_suite_mag1_physical_snap -runall
```

PMON1 focused checks added for the power-telemetry milestone:

- `tb_pmon1_i2c_job` proves the PMON1 job command sequence at `7'h38`, the
  DATA/STATUS repeated-start reads, payload packing, and invalid commit on I2C
  failure. Current PASS evidence includes
  `success payload=0x5aa593cf0000 valid=1 status=0x00 init=1`.
- `tb_rocket_i2c_suite_pmon1` proves suite-level SW10 gating, PMON1-present
  publication, and PMON1-missing NACK/error behavior. Current PASS evidence
  includes `ack_pwr_seq=1 ... pmon_addr_hits=4` and
  `nack_status=0xe0 nack_count=9`.
- `tb_i2c_suite_regression_all3_real_engine` remains the guardrail for the
  shared I2C engine and existing LIS3DH/BMP585/LIS2MDL paths; the current
  expected BMP585 payload order is
  `{press_msb, press_lsb, press_xlsb, temp_msb, temp_lsb, temp_xlsb}`.

For a full synthesis attempt:

```powershell
& "C:\Xilinx\Vivado\2023.2\bin\vivado.bat" -mode batch -source "tools/vivado/synth_caelumfusion_top_vga.tcl" -nojournal -log codex_redundant_mag_synth.log
```
