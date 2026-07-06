# EK-TM4C123GXL Fixed-Packet Producer

This replaces the failed Teensy 4.1 producer with a TivaWare C program for the
EK-TM4C123GXL LaunchPad. The FPGA packet format is unchanged: 115200 baud, 8N1,
idle high, 22-byte frames beginning with `A5 5A`.

## Confirmed Manual Facts

From the local `spmu296.pdf` LaunchPad manual:

- The EK-TM4C123GXL uses the TM4C123GH6PM target MCU.
- UART0 is connected to the ICDI virtual COM port on PA0/PA1.
- J4.04 exposes PC4 with U1RX.
- J4.05 exposes PC5 with U1TX.
- J2.01 and J3.02 are ground pins.
- J1.01 is 3.3 V and J3.01 is 5.0 V.
- The board is powered from one selected USB source during normal LaunchPad use.

Assumptions to verify before wiring:

- No BoosterPack or jumper modification is already using PC4/PC5.
- The LaunchPad is powered from ICDI USB for this first test.
- PC5/U1TX idles near 3.3 V and decodes as non-inverted UART before it touches
  the FPGA.

## Wiring

| Source | Destination | Signal | Direction | Expected behavior |
|---|---|---|---|---|
| EK-TM4C123GXL J4.05 / PC5 / U1TX | Basys-3 JXADC pin 1 / XA1_P / J3 / `teensy_uart_rx_raw` | fixed-packet UART | TM4C to FPGA | 3.3 V idle-high 115200 8N1 |
| EK-TM4C123GXL J4.04 / PC4 / U1RX | Basys-3 JXADC pin 2 / XA2_P / L3 / `teensy_uart_tx` | optional future return UART | FPGA to TM4C | Leave disconnected for first receive-only bring-up |
| EK-TM4C123GXL J2.01 or J3.02 | Basys-3 JXADC pin 5 or 11 | GND | shared reference | Common ground |

Do not connect LaunchPad J3.01 5.0 V to the Basys-3. Do not power either board
from the other during first bring-up.

## Build

This project expects TI TivaWare plus an Arm GCC toolchain. This workspace did
not have either installed when the source was added, so build verification is
environment-dependent.

Example from this directory:

```powershell
$env:TIVAWARE_ROOT = "C:/ti/TivaWare_C_Series-2.2.0.295"
make
```

If using Code Composer Studio, create or copy an EK-TM4C123GXL TivaWare project
and use `main.c` as the application source. The standalone `startup_gcc.c`,
linker script, and `Makefile` are for a GNU Arm build.

## Runtime

The program uses:

- UART0 over ICDI virtual COM for bench commands at 115200 8N1.
- UART1 on PC5/PC4 for FPGA packets at 115200 8N1.
- PF2 blue LED toggles on heartbeat emission.

Commands on the ICDI virtual COM port:

| Command | Effect |
|---|---|
| `?` | Print help |
| `h` | Toggle heartbeat frames |
| `r` | Toggle range frames |
| `c` | Corrupt next range checksum |
| `b` | Corrupt next heartbeat checksum |
| `o` | Send next range out of FPGA limit |
| `l` | Send next range with low confidence |
| `u` | Send one unsupported packet type |
| `+` / `-` | Adjust simulated range height |

Expected WaveForms decode on PC5/J4.05:

- `A5 5A 50 00 ...` heartbeat frame every 100 ms.
- `A5 5A 51 00 ...` range frame every 50 ms.
- Command `c` produces one frame with a deliberately bad checksum.
- Command `h` stops heartbeat; the FPGA should stale-reject subsequent range
  evidence after the heartbeat age limit expires.
