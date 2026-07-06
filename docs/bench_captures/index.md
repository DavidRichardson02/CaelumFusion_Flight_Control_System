# CaelumFusion Bench Capture Index

This index records WaveForms/Analog Discovery 3 bench evidence for the
CaelumFusion Basys-3 Pmod wiring. Keep this file synchronized with the raw
WaveForms workspace, exported CSVs, screenshots, programmed bitstream, switch
state, and attached Pmods used for each capture.

Use the WaveForms setup and decoder instructions in
`docs/CaelumFusion_WaveForms_Decoder_Protocol_Scripts.md`. This index is the
evidence ledger; it does not replace the raw voltage, raw Logic, or exported CSV
captures.

## Capture Acceptance Rules

1. Record the programmed bitstream hash from the actual Vivado/Hardware Manager
   programming action, not merely from a local candidate file.
2. Ground every measurement: AD3 GND, Basys-3 Pmod GND, and any Scope negative
   lead must share the same reference before signals are interpreted.
3. Keep AD3 DIOs passive inputs when the FPGA or an installed Pmod owns the
   bus.
4. For JB ADXL362/ACL2 SPI, trigger on `CS_N` falling and prove that `CS_N`
   goes low before treating a capture as an SPI frame.
5. Confirm the ADXL362 PARTID read before interpreting acceleration:
   `MOSI = 0B 02 00`, `MISO[2] = F2`.
6. Treat captures with `frames=0`, `bytes=0`, or `SPI STATUS no CS frame` as
   inactive-bus evidence only.
7. Do not interpret XYZ acceleration samples unless the same capture session
   has valid PARTID evidence or references an immediately preceding valid
   PARTID capture using the same wiring, bitstream, and switch state.

## Local Bitstream Candidates

These hashes identify files currently present in the workspace. They are not,
by themselves, proof of the bitstream programmed into the Basys-3.

| File | Last write time | Size | SHA-256 |
| --- | --- | ---: | --- |
| `CaelumFusion_Flight_Control_System.runs/impl_1/caelumfusion_top_vga.bit` | 2026-07-05 19:23:54 | 2,192,146 | `FD7E3898EE72AC0248474E672EEF908144AB4DD5A48E32C32A9D80DD71BA0454` |
| `caelumfusion_top_vga.bit` | 2026-07-05 17:31:21 | 2,192,146 | `D42327DADE1BD682C2C672579CD1261C7D21000DBE9E9966F7BD6A5F951D13AC` |
| `CaelumFusion_Flight_Control_System.runs/caelumfusion_top_vga.bit` | 2026-07-05 17:31:21 | 2,192,146 | `D42327DADE1BD682C2C672579CD1261C7D21000DBE9E9966F7BD6A5F951D13AC` |

## ADXL362 / ACL2 SPI Capture Contract

| Item | Required value |
| --- | --- |
| Basys header | `JB` |
| Attached Pmod | Pmod ACL2 / ADXL362, powered from Basys-3 Pmod `3.3 V` and `GND` |
| Runtime switch | `SW8=1` to enable the ADXL362 path |
| Build requirement | ADXL362 SPI path enabled, for example `USE_ADXL362_SPI_ACC=1` in the selected build variant |
| Logic sample rate | `25 MS/s` minimum, `100 MS/s` preferred |
| Trigger | `DIO2 / JB1 / adxl362_cs_n` falling edge |
| SPI mode | Mode 0, `CPOL=0`, `CPHA=0`, MSB first, 8-bit words, CS active low |
| PARTID transaction | `MOSI: 0x0B 0x02 0x00`; expected `MISO byte 2: 0xF2` |
| Acceleration transaction | Only interpret after PARTID is valid; ADXL362 XYZ burst begins at register `0x0E` |

### JB Probe Map

| AD3 DIO | Basys-3 pin | RTL signal | Direction from FPGA | Capture role |
| --- | --- | --- | --- | --- |
| `DIO2` | `JB1` | `adxl362_cs_n` | Output | SPI CS, active low trigger |
| `DIO3` | `JB2` | `adxl362_mosi` | Output | SPI MOSI command/data |
| `DIO4` | `JB3` | `adxl362_miso` | Input | SPI MISO response; PARTID byte appears here |
| `DIO5` | `JB4` | `adxl362_sclk` | Output | SPI SCLK, mode 0 |
| `DIO6` | `JB8` | `adxl362_int1` | Input | Optional ADXL362 interrupt/data-ready observation |
| `DIO7` | `JB7` | `adxl362_int2` | Input | Optional ADXL362 interrupt observation |
| `AD3 GND` | `JB5` or `JB11` | Pmod ground | Board reference | Required common reference |

## Capture Log

| Capture ID | Date | Status | Programmed bitstream SHA-256 | Switch settings | Attached Pmods | WaveForms workspace | Raw/decoded CSV files | Result |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `2026-07-05_jb_adxl362_partid_pending` | 2026-07-05 | Pending physical AD3 capture | TBD from Hardware Manager/programming action | Required: `SW8=1`; record all other switches before capture | Required: Pmod ACL2 on `JB`; record any JA/JC Pmods also attached | TBD | TBD | Trigger on `CS_N` falling; accept only if the decoder shows `MOSI=0B 02 00` and `PARTID response OK` with `MISO=F2`. |
| `2026-07-05_adxl362_spi_script_193704` | 2026-07-05 | Negative/inactive evidence | Not recorded | Not recorded | Not recorded | Not recorded | `/Users/98dav/caelumfusion_adxl362_spi_capture_20260705_193704.csv` | Script reported `summary frames=0`; not valid PARTID evidence. |
| `2026-07-05_adxl362_spi_script_193800` | 2026-07-05 | Negative/inactive evidence | Not recorded | Not recorded | Not recorded | Not recorded | `/Users/98dav/caelumfusion_adxl362_spi_capture_20260705_193800.csv` | Script reported `frames=0 bytes=0 partIdOk=0`; not valid PARTID or acceleration evidence. |
| `2026-07-05_i2c_script_193339` | 2026-07-05 | I2C side capture reference | Not recorded | Not recorded | Not recorded | Not recorded | `/Users/98dav/caelumfusion_i2c_protocol_capture_20260705_193339.csv` | I2C receiver output only; unrelated to ADXL362 PARTID. |
| `2026-07-05_i2c_script_193439` | 2026-07-05 | I2C side capture reference | Not recorded | Not recorded | Not recorded | Not recorded | `/Users/98dav/caelumfusion_i2c_protocol_capture_20260705_193439.csv` | I2C receiver reported `summary frames=0`; unrelated to ADXL362 PARTID. |

## Per-Capture Entry Template

Copy this section for each reportable bench capture.

### Capture ID: `YYYY-MM-DD_short_description`

| Field | Value |
| --- | --- |
| Date/time |  |
| Operator |  |
| Board | Basys-3 |
| AD3 serial |  |
| Programmed bitstream path |  |
| Programmed bitstream SHA-256 |  |
| Vivado project path | `C:/Xilinx/Vivado/2023.2/Vivado Projects/CaelumFusion_Flight_Control_System` |
| WaveForms version |  |
| WaveForms workspace path |  |
| Switch settings |  |
| Attached Pmods |  |
| Harness/breakout notes |  |
| AD3 DIO map |  |
| Scope channel map |  |
| Logic trigger |  |
| Sample rate / buffer / timebase |  |
| Raw Logic export CSV |  |
| Script/protocol CSV |  |
| Screenshot paths |  |
| Decoder scripts and revisions |  |
| Result summary |  |
| PASS/FAIL |  |

Required ADXL362 notes:

- `CS_N` falling edge observed:
- SCLK toggles only while `CS_N` is low:
- PARTID MOSI bytes observed:
- PARTID MISO byte observed:
- `PARTID=F2` confirmed:
- XYZ samples interpreted in this capture:

## Recommended Next Steps

1. Program the intended ADXL-enabled bitstream and record the exact programmed
   `.bit` path and SHA-256 in the pending capture row.
2. Set `SW8=1`, leave AD3 DIOs as passive inputs, and capture Logic on
   `DIO2..DIO7` with the trigger on `DIO2` falling.
3. Confirm the custom SPI decoder reports `PARTID response OK` before archiving
   any XYZ acceleration interpretation.
4. Update the pending row with the WaveForms workspace path, raw CSV, decoded
   CSV, and screenshot filenames immediately after the capture.
