# CaelumFusion Basys-3 Runtime Control Map

This document records the live control contract for `caelumfusion_top_vga`.
Verilog parameters remain elaboration-time hardware inclusion gates. A Basys-3
switch can only enable, disable, or mask logic that is present in the bitstream.

## Buttons

| Board control | RTL signal | Runtime role |
|---|---|---|
| BTNC | `rst` | Global reset. |
| BTNU | `btn_page_raw` | Debounced next visualization page pulse. |
| BTND | `btn_prev_raw` | Debounced previous visualization page pulse. |
| BTNR | `btn_direct_compass_raw` | Debounced direct-select latch. The active encoded-selector build loads the synchronized SW13:SW11 view ID only while SW3 MAG1 bench and SW2+SW6 diagnostic fault injection are inactive; builds with `USE_SWITCH_ENCODED_VIEW_SELECT == 0` request the compass/MAG evidence page. |
| BTNL | unused | Intentionally unassigned because the physical button is missing. |

## Switches

| Board control | RTL signal | Runtime role |
|---|---|---|
| SW0 | `sw_arm_raw` | Software arm gate for authority/status visualization. |
| SW1 | `sw_policy_enable_raw` | Policy-enable gate for authority/status visualization. |
| SW2 | `sw_selftest_raw` | Hold high for self-test HUD stimulus and synthetic extension diagnostic evidence. |
| SW3 | `sw_mag1_bench_raw` | Enable synthetic/bench MAG1 publication if `USE_MAG1_BENCH_SOURCE` is present. |
| SW4 | `sw_compass_page_raw` | Hold high for compass/MAG evidence page. |
| SW5 | `sw_history_freeze_raw` | Freeze altitude/vertical-speed history writes for inspection. |
| SW6 | `sw_log_diag_raw` | Enable black-box/log diagnostic frame requests if `USE_BLACKBOX_LOG` is present; with SW2 high, enable deliberate diagnostic fault injection. |
| SW7 | `sw_lis3dh_i2c_acc_raw` | Enable LIS3DH I2C accelerometer path if `USE_LIS3DH_I2C_ACC` is present; with SW2+SW6, selects ACC as the fault-injection bank. |
| SW8 | `sw_adxl362_spi_acc_raw` | Enable ADXL362 SPI accelerometer path if `USE_ADXL362_SPI_ACC` is present; with SW2+SW6, also selects ACC as the fault-injection bank. |
| SW9 | `sw_cmps2_mmc3416_mag_raw` | Enable CMPS2/MMC34160PJ magnetometer path if `USE_CMPS2_MMC3416_MAG` is present; with SW2+SW6, selects MAG as the fault-injection bank. |
| SW10 | `sw_pmon1_pwr_raw` | Enable PMON1 power telemetry path if `USE_PMON1_PWR` is present; with SW2+SW6, selects PWR as the fault-injection bank. |
| SW11 | `sw_mag1_offset_x_raw` | Apply `MAG1_BENCH_OFFSET_X` while SW3 synthetic MAG1 is enabled; with SW2+SW6, contributes fault-class bit 0; with `USE_SWITCH_ENCODED_VIEW_SELECT != 0`, supplies direct view ID bit 0 only outside those bench/fault ownership modes. |
| SW12 | `sw_mag1_offset_y_raw` | Apply `MAG1_BENCH_OFFSET_Y` while SW3 synthetic MAG1 is enabled; with SW2+SW6, contributes fault-class bit 1; with `USE_SWITCH_ENCODED_VIEW_SELECT != 0`, supplies direct view ID bit 1 only outside those bench/fault ownership modes. |
| SW13 | `sw_mag1_offset_z_raw` | Apply `MAG1_BENCH_OFFSET_Z` while SW3 synthetic MAG1 is enabled; with SW2+SW6, contributes fault-class bit 2; with `USE_SWITCH_ENCODED_VIEW_SELECT != 0`, supplies direct view ID bit 2 only outside those bench/fault ownership modes. |
| SW14 | `sw_compass_default_raw` | Companion hold for the compass/MAG evidence page. |
| SW15 | `sw_ext_i2c_raw` | Optional extension group enable for HYGRO/GYRO/LIS2MDL-MAG1 style shared-I2C paths and, when compiled in, the external-MCU UART range bridge. |

## Parameter-To-Switch Relationship

| Parameter | Live switch behavior |
|---|---|
| `USE_LIS3DH_I2C_ACC` | Includes the LIS3DH path; SW7 gates its live requests/publication. |
| `USE_ADXL362_SPI_ACC` | Includes the ADXL362 path; SW8 gates its live requests/publication. |
| `USE_CMPS2_MMC3416_MAG` | Includes the CMPS2/MMC34160PJ path; SW9 gates its live requests/publication. |
| `USE_PMON1_PWR` | Includes the PMON1 path. The default `caelumfusion_top_vga` build includes this logic; SW10 gates its live requests/publication. |
| `USE_BLACKBOX_LOG` | Includes the black-box frame packer; SW6 gates 10 Hz diagnostic frame requests. The current emitted schema is frame version `8'h02` with 29 32-bit words. |
| `USE_TEENSY_UART_RANGE_BRIDGE` | Includes the JXADC fixed-packet UART range bridge. The generic keeps its historical name, but the active producer is now EK-TM4C123GXL UART1. Default `0` keeps `rng_*` unavailable/not initialized. When set to `1`, SW15 gates accepted UART packets into `sensor_extension_hub.rng_*`. |
| `TEENSY_UART_BAUD` | Sets the UART baud rate for the bridge wrapper; default is 115200 8N1. |
| `USE_MAG1_BENCH_SOURCE` | Includes synthetic MAG1 support when deliberately compiled in; SW3 gates MAG1 evidence publication and the source is tagged with `EXT_SRC_SYNTHETIC_BIT` so it cannot be mistaken for physical MAG1 validation. |
| `MAG1_BENCH_OFFSET_X/Y/Z` | Parameter magnitudes remain compile-time constants; SW11/SW12/SW13 apply each axis live. |
| `USE_COMPASS_TRUTH_PAGE` | Includes the compass/MAG evidence page; BTNU/BTND/BTNR/SW4/SW14 select it live. |
| `USE_SCIENCE_PAGES` | Includes the compact science/evidence overlays. When present, direct view IDs `3'b100`, `3'b101`, and `3'b110` select explanation, wind/dispersion, and integrity pages. |
| `USE_SWITCH_ENCODED_VIEW_SELECT` | Nonzero makes BTNR latch `{SW13, SW12, SW11}` into `view_direct_id_sys[2:0]` only when SW3 MAG1 bench and SW2+SW6 diagnostic fault injection are inactive; legal IDs select views and reserved IDs latch `cfg_invalid_view_sys`. Zero preserves BTNR as direct compass select. |
| `COMPASS_TRUTH_PAGE_DEFAULT` | Sets reset/default view policy; SW14 is the live companion hold. |
| `USE_HYGRO_ENV`, `USE_GYRO_I2C`, `USE_LIS2MDL_MAG1` | Include optional shared-I2C extension paths; SW15 gates them as a group. Physical LIS2MDL/MAG1 uses address `7'h1E`, publishes real-source MAG1 evidence when present, and does not change `der_heading_mdeg`. |

## Direct View IDs

With `USE_SWITCH_ENCODED_VIEW_SELECT != 0`, BTNR samples SW13:SW11 into the
render-control direct-view request only while SW3 MAG1 bench mode and SW2+SW6
diagnostic fault injection are both inactive:

| SW13:SW11 | View ID | Result |
|---:|---|---|
| `000` | `VIEW_FLIGHT_HUD` | Normal live-telemetry HUD. |
| `001` | `VIEW_COMPASS_TRUTH` | Compass/MAG truth page when compiled in; otherwise rejected. |
| `010` | `VIEW_SELFTEST_HUD` | Normal HUD renderer with self-test stimulus. |
| `011` | `VIEW_SENSOR_DIAG` | In-HUD sensor diagnostic page or primary-HUD fallback if page logic is compiled out. |
| `100` | `VIEW_SCIENCE_EXPLAIN` | Compact physics/environment evidence page when science pages are compiled in. |
| `101` | `VIEW_SCIENCE_WIND` | Compact wind triangle / dispersion evidence page when science pages are compiled in. |
| `110` | `VIEW_SCIENCE_INTEGRITY` | Compact sensor-integrity correlation page when science pages are compiled in. |
| `111` | reserved | Rejected; `view_sel_sys` holds and `cfg_invalid_view_sys` latches. |

### SW11-SW13 Collision Arbitration

SW11, SW12, and SW13 have three possible meanings in the Basys-3 build:
direct-view ID bits, MAG1 bench offset enables, and diagnostic fault-class
bits. The RTL treats MAG1 bench and deliberate diagnostic fault injection as
the owners of those switches. If SW3 is high, or if SW2 and SW6 are both high,
pressing BTNR in encoded-selector mode does not navigate. Instead,
`caelumfusion_vga_direct_view_arbiter_sys` converts the request to reserved ID
`3'b111`; `caelumfusion_vga_render_control` holds `view_sel_sys` and latches
`cfg_invalid_view_sys`.

Clean direct navigation procedure:

1. Clear SW3 and clear either SW2 or SW6 so MAG1 bench offsets and fault-class
   selection do not own SW11:SW13.
2. Set SW13:SW11 to the requested direct-view ID and press BTNR.
3. Restore SW3, SW2, SW6, and SW11:SW13 for the intended MAG1 bench or
   diagnostic fault-injection scenario.

The planar heading contract is unchanged: `der_heading_mdeg` remains planar
`atan2(MY, MX)`, not tilt-compensated.

PMON1 is a SW10-gated raw power bank on the shared JA I2C bus at address
`7'h38`. With SW10 low, the PMON path remains intentionally unavailable:
`pwr_valid` is low, `pwr_status` reports not initialized, and `pwr_age_ms`
saturates stale. With SW10 high and a PMON1 present at `7'h38`, the suite
publishes voltage/current/status payloads with normal sequence and age updates.
With SW10 high and PMON1 missing or unplugged, the PMON bank commits invalid I2C
error evidence (`pwr_status` in the `8'hE0` class) and increments bus NACK
evidence; it must not preserve stale-good data.

Physical LIS2MDL/MAG1 is a SW15-gated raw evidence bank on the shared JA I2C bus
at address `7'h1E` when `USE_LIS2MDL_MAG1` is compiled in. With SW15 low, MAG1
remains unavailable/not initialized. With SW15 high and a valid LIS2MDL present,
the suite reads WHO/configures the device, publishes `{Z,Y,X}` raw magnetic
payloads, marks `EXT_SRC_REAL_BIT`, and increments MAG1 sequence/age normally.
MAG1 is evidence for cross-checking MAG0; it is not fused into the live planar
heading output.

SW6 black-box frame requests emit version `8'h02` frames. Word 0 carries
`{TELEM_PKT_SYNC, PKT_BLACKBOX_WORD, 8'h02}` and word 1 carries the 29-word
length. Host decoders must branch on the version before interpreting words 26-28
as compact MAG1 metadata, norm/residual evidence, and checksum evidence.

## External MCU Bridge Packet Contract

`teensy_bridge_packet_ingress` is a fixed-packet SYS-domain ingress block for
external-MCU transport evidence. `teensy_uart_range_bridge` now wraps that
packet ingress with a simple 8N1 UART receiver and feeds
`sensor_extension_hub.rng_*`. The physical connector is Basys-3 `JXADC`, not
`JD`: JXADC pin 1 is FPGA RX from TM4C PC5/U1TX/J4.05, and JXADC pin 2 is FPGA
TX to optional TM4C PC4/U1RX/J4.04. The bridge remains default-off unless
`USE_TEENSY_UART_RANGE_BRIDGE` is overridden to `1`. The first implemented
dispatch target is range/AGL evidence:

The active LaunchPad producer is
`firmware/tm4c123gxl_bridge_range_producer/main.c`; the bring-up checklist is
`docs/CaelumFusion_TM4C123GXL_UART_Bridge_Bringup.md`.

| Packet type | Value | Dispatch |
|---|---:|---|
| `PKT_TEENSY_HEARTBEAT` | `8'h50` | Updates bridge heartbeat age/sequence |
| `PKT_TEENSY_RANGE_AGL` | `8'h51` | Publishes `rng_*` evidence for `ext_rng_height_cm` |

Checksum faults, unsupported packet types, stale range evidence, and missing
heartbeat evidence are explicit diagnostics. This bridge does not bind range to
altitude, navigation, wind, landing dispersion, or control authority.

## Diagnostic Extension Contract

SW2 asserts a synthetic extension-diagnostic source inside
`sensor_extension_hub`. This marks `EXT_PRESENT_DIAG_BIT`, publishes
deterministic placeholder range/air/environment/sun/flow values, and marks the
source as synthetic using `EXT_SRC_SYNTHETIC_BIT`.

SW2 plus SW6 asserts deliberate fault injection. In that mode, the full-HUD
self-test overlay is suppressed so the injected evidence remains visible. The
physical sensor-suite writers are not changed; `snapshot_fault_injector`
creates a deterministic diagnostic view for the evidence/visualization readers.
BTNR encoded direct navigation is blocked while SW2+SW6 are both high because
SW11:SW13 belong to fault-class selection in that mode.

Fault bank selection while SW2+SW6 are high:

| Selection | Faulted bank |
|---|---|
| SW10 high | PWR |
| Else SW9 high | MAG |
| Else SW7 or SW8 high | ACC |
| Else | BMP |

Fault class selection while SW2+SW6 are high:

| SW13:SW12:SW11 | Class |
|---:|---|
| `000` | status/config error |
| `001` | stale |
| `010` | status/config error |
| `011` | stuck sequence |
| `100` | invalid payload / numeric fault |
| `101` | out of range |
| `110`, `111` | status/config error |
