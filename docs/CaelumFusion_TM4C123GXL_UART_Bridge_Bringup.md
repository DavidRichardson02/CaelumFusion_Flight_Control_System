# CaelumFusion TM4C123GXL UART Bridge Bring-Up

This is the cautious bring-up path for replacing the failed Teensy 4.1 producer
with an EK-TM4C123GXL LaunchPad fixed-packet producer. The FPGA RTL still uses
the historical `teensy_*` signal and module names, but the physical producer is
now the TM4C123GXL LaunchPad.

## Confirmed Facts

Confirmed from local manuals in
`\\Mac\Home\Desktop\Tiva C Series TM4C123G Reference Manuals`:

| Manual | Used for |
|---|---|
| `spmu296.pdf` | EK-TM4C123GXL board identity, USB/debug, BoosterPack headers, UART0 virtual COM, J4 PC4/PC5 pins, GND/3.3 V/5.0 V header pins |
| `spmu298e.pdf` | TivaWare DriverLib UART/GPIO/SysCtl programming model |
| `spmu373a.pdf` | TivaWare SDK/project structure and CCS/GNU-style development assumptions |
| `spma059.pdf` | TM4C123x 3.3 V supply and low-speed UART routing guidance |

Confirmed current FPGA contract:

| Item | Value |
|---|---|
| FPGA bridge generic | `USE_TEENSY_UART_RANGE_BRIDGE`, default `0` |
| TM4C deliberate build script | `tools/vivado/synth_caelumfusion_top_vga_tm4c_bridge.tcl` |
| Legacy bridge build script | `tools/vivado/synth_caelumfusion_top_vga_teensy_bridge.tcl` |
| FPGA UART baud | `TEENSY_UART_BAUD = 115200` |
| UART format | 8 data bits, no parity, 1 stop bit, idle high |
| TM4C firmware | `firmware/tm4c123gxl_bridge_range_producer/main.c` |
| TM4C FPGA UART | UART1, PC5/U1TX on J4.05, PC4/U1RX on J4.04 |
| TM4C console UART | UART0 over ICDI virtual COM, PA0/PA1 |
| FPGA RX pin | Basys-3 JXADC pin 1 / XA1_P / package J3 / `teensy_uart_rx_raw` |
| FPGA TX pin | Basys-3 JXADC pin 2 / XA2_P / package L3 / `teensy_uart_tx`, idle high today |
| Ground | Basys-3 JXADC pin 5 or 11 to LaunchPad J2.01 or J3.02 |

Assumptions that must be verified on the bench:

- The LaunchPad is an EK-TM4C123GXL with the normal TM4C123GH6PM target MCU.
- The board is not modified in a way that repurposes PC4/PC5 or changes header
  voltages.
- No BoosterPack is installed that drives PC4/PC5.
- PC5/U1TX measures near 3.3 V idle-high before any FPGA connection.

## Safety Warnings

- Do not connect LaunchPad J3.01 5.0 V to Basys-3 JXADC or any FPGA I/O.
- Do not use the failed Teensy as a level reference or pass-through device.
- Do not backfeed power between boards. For first bring-up, power TM4C from ICDI
  USB and Basys-3 from its normal supply, with common ground only.
- Do not wire PC4/U1RX to the FPGA TX return path until PC5/U1TX receive-only
  validation works.
- Treat disconnected logic probes as floating. A high or low reading on an
  unconnected lead is not evidence.

## Packet Format

Every frame is 22 UART bytes:

```text
A5 5A
type
status
seq[15:8] seq[7:0]
timestamp_us[31:24] timestamp_us[23:16] timestamp_us[15:8] timestamp_us[7:0]
payload[47:40] payload[39:32] payload[31:24] payload[23:16] payload[15:8] payload[7:0]
aux[15:8] aux[7:0]
source_flags[15:8] source_flags[7:0]
checksum[15:8] checksum[7:0]
```

| Type | Value | Meaning |
|---|---:|---|
| `PKT_BRIDGE_HEARTBEAT` | `0x50` | Bridge heartbeat sequence and age evidence |
| `PKT_BRIDGE_RANGE_AGL` | `0x51` | Range/AGL evidence |
| unsupported test packet | `0x7E` | Negative test for unsupported-packet counter |

Checksum:

```text
checksum = xor(0xA55A,
               {status,type},
               seq,
               timestamp_us[31:16],
               timestamp_us[15:0],
               payload[47:32],
               payload[31:16],
               payload[15:0],
               aux,
               source_flags)
```

## TM4C Firmware Build

The source is in `firmware/tm4c123gxl_bridge_range_producer`.

Required tools:

- TI TivaWare C Series SDK.
- GNU Arm Embedded toolchain, Code Composer Studio, or another TM4C-capable
  compiler/debugger.

GNU Arm example:

```powershell
Set-Location "firmware/tm4c123gxl_bridge_range_producer"
$env:TIVAWARE_ROOT = "C:/ti/TivaWare_C_Series-2.2.0.295"
make
```

This workspace did not have TivaWare or an Arm compiler installed when this
refactor was performed, so the firmware source was not binary-built here.

## Wiring Table

| Source device and pin | Destination device and pin | Signal | Direction | Voltage level | Expected behavior | Verification step |
|---|---|---|---|---|---|---|
| LaunchPad J4.05 / PC5 / U1TX | Basys-3 JXADC pin 1 / XA1_P / J3 / `teensy_uart_rx_raw` | fixed-packet UART | TM4C to FPGA | 3.3 V logic | Idle high, 115200 8N1, `A5 5A` frame sync | First probe PC5 standalone with WaveForms, then probe JXADC pin 1 after wiring |
| LaunchPad J4.04 / PC4 / U1RX | Basys-3 JXADC pin 2 / XA2_P / L3 / `teensy_uart_tx` | optional return UART | FPGA to TM4C | 3.3 V logic | Leave disconnected for first pass; FPGA line currently idle high | Measure JXADC pin 2 high after FPGA configuration before any connection |
| LaunchPad J2.01 or J3.02 | Basys-3 JXADC pin 5 or 11 | GND | shared reference | 0 V | Common reference only | Power-off continuity check |
| LaunchPad ICDI USB | PC | power/debug/console | host to TM4C | USB 5 V input to board regulator | Provides power and virtual COM UART0 | Green power LED on, UART0 console visible |

Do not connect LaunchPad J1.01 3.3 V or J3.01 5.0 V to Basys during first
bring-up. Use those rails only as measured references unless a separate power
plan is reviewed.

## WaveForms Setup

Use Analog Discovery as a passive probe only.

| AD channel | Standalone TM4C probe point | FPGA-connected probe point |
|---:|---|---|
| `DIO8` | LaunchPad J4.05 / PC5 / U1TX | Basys JXADC pin 1 / `teensy_uart_rx_raw` |
| `DIO9` | optional, LaunchPad J4.04 / PC4 / U1RX | Basys JXADC pin 2 / `teensy_uart_tx` |
| `GND` | LaunchPad GND | Common LaunchPad/Basys ground |

| Setting | Value |
|---|---|
| Digital threshold | 1.5 V to 1.7 V |
| Sample rate | 5 MS/s minimum, 10 MS/s preferred |
| Capture duration | 100 ms to 500 ms |
| Trigger | Falling edge on `DIO8` |
| UART decoder | DIO8, 115200 baud, 8N1, LSB first, non-inverted |

Expected decode:

| Scenario | Expected UART evidence |
|---|---|
| Normal producer | Repeating sync `A5 5A`, packet types `50` and `51`, status `00` |
| Command `c` over ICDI COM port | One range frame still decodes, but the final two checksum bytes fail the RTL checksum |
| Command `h`, wait >500 ms | Heartbeat frames stop; range frames continue if range remains enabled |
| Command `u` | One frame has type `7E` |

## FPGA Bridge Image

Build the TM4C-named deliberate bridge image with:

```powershell
& "C:\Xilinx\Vivado\2023.2\bin\vivado.bat" -mode batch `
  -source "tools/vivado/synth_caelumfusion_top_vga_tm4c_bridge.tcl"
```

This still passes `-generic USE_TEENSY_UART_RANGE_BRIDGE=1` because the RTL
generic has not been renamed. The script writes reports/checkpoint under
`.codex_build/synth_tm4c_uart_bridge`.

For first FPGA-connected validation:

1. Keep the failed Teensy completely disconnected.
2. Flash the TM4C firmware.
3. Power the LaunchPad from ICDI USB only.
4. Confirm PC5/U1TX idles near 3.3 V with the FPGA disconnected.
5. Capture PC5/U1TX with WaveForms and verify `A5 5A 50 00` and `A5 5A 51 00`.
6. Power down Basys-3 and the LaunchPad.
7. Wire common ground only; power up and verify no unexpected power path.
8. Power down again.
9. Wire LaunchPad PC5/J4.05 to Basys JXADC pin 1.
10. Leave LaunchPad PC4/J4.04 disconnected.
11. Program the deliberate bridge image.
12. Set SW15 high.
13. Capture JXADC pin 1 with WaveForms and confirm the same `A5 5A` frames.
14. Validate command `c` and command `h` behavior. Check that checksum faults
    and stale heartbeat rejection do not preserve stale-good range data.

Current limitation: the TM4C firmware publishes simulated range evidence only.
It does not yet own a physical lidar/rangefinder driver, GNSS, SD logging,
radio, or FPGA ACK handling.
