# CaelumFusion CMPS2 Basys 3 Integration

This note records the current Digilent Pmod CMPS2 integration contract for the
Basys 3 `caelumfusion_top_vga` avionics build.

## Hardware Wiring

The active XDC maps the CMPS2 as a standard Type-6 I2C Pmod on Basys 3 Pmod JA.

| CMPS2 signal | CMPS2 pin | Basys 3 Pmod pin | FPGA port | Package pin |
| --- | ---: | --- | --- | --- |
| SCL | J1-3 | JA3 | `scl` | `J2` |
| SDA | J1-4 | JA4 | `sda` | `G2` |
| 3V3 | J1-6 | JA power | board rail | not an FPGA pin |
| GND | J1-5 | JA ground | board ground | not an FPGA pin |

Both `scl` and `sda` are open-drain `inout` ports with LVCMOS33 and pullups in
`CaelumFusion_Flight_Control.srcs/constrs_1/new/Basys-3-Master.xdc`.

## RTL Path

Default `caelumfusion_top_vga` builds, without `CAELUM_SENSOR_SPI`, use this
sensor path:

1. `caelumfusion_top_vga`
2. `rocket_i2c_suite_top`
3. `mmc3416_i2c_job`
4. `snapshot_regs` for the magnetometer publication bank
5. `derived_state_producer`
6. `flight_attitude_math_sys`
7. `flight_viz_suite_top`

The CMPS2 owns the magnetometer slot formerly used by the LIS2MDL I2C job. The
downstream snapshot contract remains a 48-bit payload plus timestamp, sequence,
validity, status, and freshness metadata.

## CMPS2 / MMC3416 Contract

The Pmod CMPS2 manual identifies the module as an MMC34160PJ based I2C compass
at 7-bit address `0x30`, with six magnetic data registers from `0x00` through
`0x05` in low-byte/high-byte axis order. The implemented job uses the following
deterministic sequence:

1. Probe address range `0x30` through `0x37` and accept Product ID `0x06` at
   register `0x20`.
2. Write Control 1 register `0x08 = 0x00` for 16-bit mode.
3. Write Control 0 register `0x07 = 0x80` to refill the SET/RESET capacitor.
4. Wait `REFILL_WAIT_US`.
5. Write Control 0 register `0x07 = 0x20` to perform SET.
6. Wait `SETRESET_WAIT_US`, then publish `init_done`.
7. On each 10 Hz magnetometer epoch, write Control 0 register `0x07 = 0x01` to
   start measurement.
8. Poll status register `0x06` until bit 0 is set or `MEAS_TIMEOUT_US` expires.
9. Burst-read six bytes from `0x00`.

Published payload order is:

```text
{Z_H, Z_L, Y_H, Y_L, X_H, X_L}
```

`derived_state_producer` sets `MAG_PAYLOAD_ZYX = 1`, so heading math decodes
`MX` from payload bits `[15:0]` and `MY` from `[31:16]`.

## Verification Commands

Run these from the project root:

```powershell
New-Item -ItemType Directory -Force -Path .codex_build\cmps2_job_sim | Out-Null
Push-Location .codex_build\cmps2_job_sim
& C:\Xilinx\Vivado\2023.2\bin\xvlog.bat ..\..\CaelumFusion_Flight_Control.srcs\sources_1\new\mmc3416_i2c_job.v ..\..\CaelumFusion_Flight_Control.srcs\sim_1\new\tb_mmc3416_i2c_job.v
& C:\Xilinx\Vivado\2023.2\bin\xelab.bat -debug typical tb_mmc3416_i2c_job -s tb_mmc3416_i2c_job_sim
& C:\Xilinx\Vivado\2023.2\bin\xsim.bat tb_mmc3416_i2c_job_sim -runall
Pop-Location

New-Item -ItemType Directory -Force -Path .codex_build\cmps2_mux_sim | Out-Null
Push-Location .codex_build\cmps2_mux_sim
& C:\Xilinx\Vivado\2023.2\bin\xvlog.bat ..\..\CaelumFusion_Flight_Control.srcs\sources_1\new\mmc3416_i2c_job.v ..\..\CaelumFusion_Flight_Control.srcs\sources_1\new\i2c_job_arbiter.v ..\..\CaelumFusion_Flight_Control.srcs\sources_1\new\i2c_job_mux.v ..\..\CaelumFusion_Flight_Control.srcs\sim_1\new\tb_mmc3416_i2c_mux_arbiter_continuation.v
& C:\Xilinx\Vivado\2023.2\bin\xelab.bat -debug typical tb_mmc3416_i2c_mux_arbiter_continuation -s tb_mmc3416_i2c_mux_arbiter_continuation_sim
& C:\Xilinx\Vivado\2023.2\bin\xsim.bat tb_mmc3416_i2c_mux_arbiter_continuation_sim -runall
Pop-Location
```

Expected results:

```text
PASS: tb_mmc3416_i2c_job
PASS: tb_mmc3416_i2c_mux_arbiter_continuation
```

## Bench Bring-Up

1. With CMPS2 attached to JA, verify SCL/SDA idle high before programming.
2. Program the default `caelumfusion_top_vga` bitstream without
   `CAELUM_SENSOR_SPI`.
3. Confirm the JA SCL line produces 100 kHz traffic and the CMPS2 ACKs address
   `0x30`.
4. Confirm the displayed magnetometer freshness/heading fields update near
   10 Hz.
5. Induce stale data by disconnecting CMPS2 only after power is removed; on the
   next powered run, verify the magnetometer status/freshness path reports the
   missing or stale sensor rather than presenting a valid heading.

## Remaining Integration Risk

The I2C engine does not currently support clock stretching. The CMPS2 path uses
explicit wait and status-poll timing rather than relying on stretching, so this
is acceptable for the present integration. Keep the `RPBF-3`/open-drain SCL
warning reviewed in release signoff until SCL input sampling or the interface
contract is made explicit in implementation reports.
