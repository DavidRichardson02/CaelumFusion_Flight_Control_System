# CaelumFusion WaveForms Decoder And Protocol Scripts

This note defines project-local WaveForms scripts for Analog Discovery 3 probing
of the current CaelumFusion Basys-3 Pmod harness.

The scripts are bench instrumentation artifacts. They do not change RTL, XDC, or
the bitstream. Treat AD3 as a passive probe unless a script name and header say
`isolated` and its explicit active-drive guard has been changed by hand.

## Source Grounding

The script API was checked against the installed WaveForms 3 local documentation:

| Installed documentation | Relevant contract |
|---|---|
| `C:/Program Files (x86)/Digilent/WaveForms3/doc/logic.html` | Logic Custom interpreters use `rgData` input and `rgValue` / `rgFlag` output arrays. |
| `C:/Program Files (x86)/Digilent/WaveForms3/doc/scope.html` | Scope supports probes, Custom Math channels using variables such as `C1`/`C2`, and measurements including rise/fall time. |
| `C:/Program Files (x86)/Digilent/WaveForms3/doc/script.html` | Script tool access to `Protocol.I2C`, `Protocol.SPI`, `File`, `Tool.workspaceDir()`, and instrument data/export APIs. |
| `C:/Program Files (x86)/Digilent/WaveForms3/doc/protocol.html` | Protocol I2C/SPI custom and receiver functions, including passive receiver decode return formats. |

The electrical map follows the active project wiring guide and RTL/XDC:

| AD3 DIO | Basys-3 point | RTL signal | Use |
|---:|---|---|---|
| `DIO0` | `JA3` | `scl` | Shared I2C SCL. |
| `DIO1` | `JA4` | `sda` | Shared I2C SDA. |
| `DIO2` | `JB1` | `adxl362_cs_n` | ADXL362 SPI select, active low. |
| `DIO3` | `JB2` | `adxl362_mosi` | ADXL362 SPI MOSI. |
| `DIO4` | `JB3` | `adxl362_miso` | ADXL362 SPI MISO. |
| `DIO5` | `JB4` | `adxl362_sclk` | ADXL362 SPI clock. |
| `DIO6` | `JB8` | `adxl362_int1` | ADXL362 interrupt 1. |
| `DIO7` | `JB7` | `adxl362_int2` | ADXL362 interrupt 2. |

## Files Added

| File | Paste target | Drives bus? | Purpose |
|---|---|---:|---|
| `tools/waveforms/logic_custom_i2c_decoder.js` | Logic Custom `Decoder` tab | No | Primary I2C evidence path: frame/segment ID, START/RESTART/STOP, raw address byte, 7-bit address, direction, byte index, ACK/NACK, and partial-frame errors. |
| `tools/waveforms/logic_custom_i2c_value_to_text.js` | Logic Custom `Value to text` tab | No | Renders byte-provenance labels for every I2C address/data byte. |
| `tools/waveforms/logic_custom_spi_adxl362_decoder.js` | Logic Custom `Decoder` tab | No | Primary SPI evidence path: CS-frame status, byte index, MOSI/MISO pairs, command/register context, partial-byte errors, and inactive-bus status. |
| `tools/waveforms/logic_custom_spi_adxl362_value_to_text.js` | Logic Custom `Value to text` tab | No | Renders ADXL362 command/register/PARTID/XYZ labels plus no-CS-frame diagnostics. |
| `tools/waveforms/script_protocol_i2c_receiver_caelumfusion.js` | WaveForms `Script` tool | No | Runs passive `Protocol.I2C.Receiver()`, prints semantic I2C frames, and writes a byte-auditable CSV log. |
| `tools/waveforms/script_protocol_spi_adxl362_receiver_caelumfusion.js` | WaveForms `Script` tool | No | Runs passive `Protocol.SPI.Receiver()`, logs MOSI/MISO byte pairs, labels ADXL362 frames, and flags MISO stuck-at symptoms. |
| `tools/waveforms/protocol_custom_i2c_passive_receiver_caelumfusion.js` | Protocol I2C `Custom` | No | Compatibility guard for WaveForms builds where `Receiver()` / `Receive()` are not exposed in the Custom runtime. |
| `tools/waveforms/protocol_custom_i2c_api_probe.js` | Protocol I2C `Custom` | No | Diagnostic-only API probe that reports which I2C Custom functions are available. |
| `tools/waveforms/protocol_custom_i2c_isolated_bus_probe.js` | Protocol I2C `Custom` | Yes, guarded | Address ACK scan for isolated Pmods only. |
| `tools/waveforms/protocol_custom_spi_adxl362_isolated_probe.js` | Protocol SPI `Custom` | Yes, guarded | ADXL362 PARTID/XYZ probe for an isolated ACL2 only. |
| `tools/waveforms/scope_probe_i2c_3v3_pass_input.js` | Scope Probe Custom input script | No | Identity conversion from measured voltage into displayed volts. |
| `tools/waveforms/scope_probe_i2c_3v3_pass_output.js` | Scope Probe Custom trigger script | No | Identity conversion from trigger/display unit back to volts. |
| `tools/waveforms/scope_math_scl_dig_1v65.js` | Scope Math Custom | No | Renders SCL as a 0 V / 3.3 V logic-level view using a 1.65 V threshold. |
| `tools/waveforms/scope_math_sda_dig_1v65.js` | Scope Math Custom | No | Renders SDA as a 0 V / 3.3 V logic-level view using a 1.65 V threshold. |
| `tools/waveforms/scope_math_i2c_bus_min.js` | Scope Math Custom | No | Shows a compact bus-activity envelope that drops when either I2C line is low. |
| `tools/waveforms/scope_math_sda_minus_scl.js` | Scope Math Custom | No | Shows SDA relative to SCL so START, STOP, ACK, and data timing stand out visually. |

## Byte-Validation Artifact Contract

The artifacts are split by WaveForms execution context. Do not move code between
contexts unless the API calls are changed at the same time.

| Artifact | Paste target | Passive? | Required map | Expected output | Failure modes | Verification |
|---|---|---:|---|---|---|---|
| `logic_custom_i2c_decoder.js` plus `logic_custom_i2c_value_to_text.js` | Logic `Custom` Decoder and Value-to-text tabs | Yes | `DIO0=SCL`, `DIO1=SDA` | Frame ID, START/RESTART/STOP, raw address byte, 7-bit address, R/W, ACK/NACK, byte index, data context | Partial frame, capture ended in frame, address NACK | Compare against raw SCL/SDA and Protocol Spy/Slave output. |
| `protocol_custom_i2c_passive_receiver_caelumfusion.js` | `Protocol -> I2C -> Custom` | Yes | `SCL=DIO0`, `SDA=DIO1` | Compatibility check only; reports whether `Receiver()` / `Receive()` exist | `Receiver` unavailable in WaveForms runtime | If unavailable, use built-in `Spy/Slave`, Logic Custom decoders, or the separate Script tool. |
| `protocol_custom_i2c_api_probe.js` | `Protocol -> I2C -> Custom` | Yes | none | Prints available Custom-tab function names | Runtime exposes only active master/slave helpers | Use before trusting Protocol Custom examples from documentation. |
| `script_protocol_i2c_receiver_caelumfusion.js` | Separate WaveForms `Script` tool | Yes | Protocol already set to `I2C`, `SCL=DIO0`, `SDA=DIO1` | Same semantic frame log plus CSV with `address_byte`, `byte_evidence`, and `data_count` | Wrong paste target, no Protocol traffic, malformed capture | Confirm CSV is written and summary address counts match the Logic decode. |
| `logic_custom_spi_adxl362_decoder.js` plus `logic_custom_spi_adxl362_value_to_text.js` | Logic `Custom` Decoder and Value-to-text tabs | Yes | `DIO2=CS_N`, `DIO3=MOSI`, `DIO4=MISO`, `DIO5=SCLK` | CS frames, no-CS-frame status, byte index, MOSI/MISO bytes, command/register labels | No CS frame, partial SPI byte, CS deasserts mid-byte, wrong SPI mode | Confirm PARTID transaction is `0B 02 00` and response byte is `F2` before interpreting XYZ. |
| `script_protocol_spi_adxl362_receiver_caelumfusion.js` | Separate WaveForms `Script` tool | Yes | Protocol already set to SPI spy mode 0, MSB, 8-bit | CSV with MOSI/MISO byte pairs, byte roles, PARTID validation, XYZ recognition, MISO health flags | `miso_all_00`, `miso_all_FF`, `unknown_command`, short reads | Confirm `partIdOk > 0` before interpreting acceleration values. |
| `protocol_custom_i2c_isolated_bus_probe.js` | `Protocol -> I2C -> Custom` | No, active-drive guarded | Isolated I2C Pmod only | Address ACK scan | Refuses to run unless `ALLOW_ACTIVE_DRIVE=true`; bus not free | Use only after FPGA bus owner is disconnected or tri-stated. |
| `protocol_custom_spi_adxl362_isolated_probe.js` | `Protocol -> SPI -> Custom` | No, active-drive guarded | Isolated ADXL362 only | PARTID and XYZ probe controlled by AD3 | Refuses to run unless `ALLOW_ACTIVE_DRIVE=true`; wrong PARTID | Use only off the live FPGA bus. |
| `scope_probe_i2c_3v3_pass_*` and `scope_math_*` | Scope Probe Custom and Scope Math Custom | Yes | Scope `C1=SCL`, `C2=SDA`, common ground | Raw voltage plus semantic threshold/envelope/difference views | Impossible voltage, bad ground, threshold mismatch | Raw channels must show high near `3.3 V` and low near `0 V` before protocol decode. |

Scientific boundary: decoded labels are hypotheses about meaning. Electrical
validity comes from raw Scope/Logic levels, protocol validity comes from frame
and byte checks, and device validity comes from ACK plus expected identity or
register behavior.

## Current Capture Interpretation

The July 5 WaveForms captures show that the Logic instrument is the most useful
evidence path for the present bench state:

| Observation | Engineering meaning | Next action |
|---|---|---|
| Protocol Script I2C receiver prints `summary frames=0` while Logic shows SCL/SDA activity | The separate Script receiver is not acquiring frames in this WaveForms setup, but the Logic capture is seeing real digital transitions. | Use Logic Custom decoders as the primary byte-validation layer. |
| Protocol Spy/Slave shows repeated `h30 [ h18 | WR ] NACK` and `h32 [ h19 | WR ] NACK` | The FPGA is probing LIS3DH 7-bit addresses `0x18` and `0x19`; both are NACKing in the captured state. The raw address bytes are `0x30` and `0x32`. | Treat this as valid negative evidence: the bus is active, but the expected LIS3DH device is not acknowledging. Check Pmod presence, power, address strap, and harness. |
| Logic captures show ADXL362/ACL2 CS_N, SCLK, MOSI, and MISO flat/inactive | The SPI receiver correctly reports zero frames because the FPGA is not selecting or clocking the ADXL362 bus in this capture. | Do not debug MISO or PARTID yet. First enable/compile the ADXL362 path and verify CS_N falls and SCLK toggles. |
| Scope still shows impossible `+/-28.9 V` readouts in some views | Scope probe/reference setup is still suspect for analog voltage interpretation. | Fix `I2C_3V3_PASS` and common ground before using Scope measurements as electrical evidence. Logic digital captures remain useful if thresholds are correct. |

## Full WaveForms Implementation Walkthrough

Use this section as the bench procedure for installing the custom code into a
WaveForms workspace. The passive Logic Custom decoders are the default path for
a live Basys-3 system. The Protocol receiver scripts are secondary text/CSV
capture tools. The isolated Protocol Custom scripts are active-drive tools and
must not be used on a live FPGA-owned bus.

### 1. Create A Clean AD3 Workspace

1. Connect Analog Discovery 3 to USB and open WaveForms.
2. In `Device Manager`, select the Analog Discovery 3.
3. Create a new workspace.
4. Confirm `Supplies`, `Wavegen`, and `Patterns` are stopped or disabled.
5. Save the workspace with a project-specific name such as
   `CaelumFusion_AD3_Basys3_Pmods.dwf3work`.
6. Connect AD3 ground to Basys-3/Pmod ground before connecting any DIO probe.
7. Connect only passive DIO probes for the first capture:
   - `DIO0` to `JA3/scl`.
   - `DIO1` to `JA4/sda`.
   - `DIO2..DIO7` to the JB ADXL362 signals only when debugging ACL2 SPI.
   - `DIO8..DIO15` to JC only when debugging LS1/PIR/DPOT-reserved pins.

Do not enable AD3 output drivers on the live FPGA harness. For normal
CaelumFusion work, AD3 DIO pins are measurement inputs.

### 2. Install The Shared-I2C Logic Custom Decoder

1. Open the `Logic` instrument.
2. Add or enable raw digital channels `DIO0` and `DIO1`.
3. Rename `DIO0` to `scl`.
4. Rename `DIO1` to `sda`.
5. Set the digital threshold for 3.3 V logic. A threshold around `1.5 V` to
   `1.7 V` is appropriate for LVCMOS33 captures.
6. Set an initial capture of about `10 ms` to `100 ms`.
7. Set the sample rate high enough to resolve edges. For standard-mode or
   fast-mode I2C, start at `5 MS/s` or higher.
8. Add a `Custom` interpreter from the Logic add/interpreter menu.
9. Open the Custom interpreter property editor.
10. In the `Decoder` tab, paste the complete contents of
    `tools/waveforms/logic_custom_i2c_decoder.js`.
11. In the `Value to text` tab, paste the complete contents of
    `tools/waveforms/logic_custom_i2c_value_to_text.js`.
12. Apply/accept the interpreter editor.
13. Save the WaveForms workspace.

The decoder reads the physical DIO bit positions in `rgData`, so the constants
at the top of the decoder must continue to match the probe map:

```javascript
var CFG_I2C_SCL_BIT = 0;
var CFG_I2C_SDA_BIT = 1;
```

Only edit these constants if you intentionally move the probes to different AD3
DIO pins. Do not edit address labels just to make a failing device look valid;
the label is metadata, while ACK/NACK is electrical evidence.

### 3. Capture And Interpret Shared-I2C Traffic

1. Power the Basys-3 and attached I2C harness.
2. Confirm `scl` and `sda` idle high before enabling any sensor path.
3. In Logic, trigger on an I2C `START` if available, or use a simple falling
   edge trigger on `sda` while `scl` is high.
4. Run a single capture.
5. Look for `START`, address labels, `ACK`/`NACK`, data bytes, and `STOP`.
6. Treat a device as present only when its address byte is ACKed.
7. Treat repeated NACKs as useful negative evidence, not as a decoder failure.
8. Export the raw Logic acquisition as CSV when the capture is reportable.
9. Run the host decoder on exported CSV for deterministic cross-checks before
   using the data in a report.

Expected first checks with the hardened decoder:

| Observation | Meaning |
|---|---|
| `scl=1`, `sda=1` at idle | Bus pullups and no stuck-low device are plausible. |
| `F# START` / `F# RESTART` / `F# STOP bytes=...` | Decoder found valid I2C framing and reports the segment number plus byte count. |
| `ADDR raw=0x30 addr7=0x18 W NACK LIS3DH SA0=0` | FPGA placed raw address byte `0x30` on the bus, which decodes to 7-bit `0x18` write; no device acknowledged. |
| Address label plus `ACK` | A device electrically responded at that 7-bit address. |
| Address label plus `NACK` | FPGA probed the address, but no device acknowledged; this is valid negative evidence. |
| `DATA 0x.. ACK/NACK ctx=...` | Data byte with address/direction context from the current segment. |
| `ERR capture ended in frame` | Capture stopped before a STOP; increase capture time or pre/post-trigger. |
| No START events | FPGA path may be disabled, wrong switch state, wrong probes, or bus stuck. |

### 4. Install The ADXL362 / ACL2 SPI Logic Custom Decoder

1. Open the `Logic` instrument.
2. Add or enable raw digital channels `DIO2`, `DIO3`, `DIO4`, and `DIO5`.
3. Rename the channels:
   - `DIO2` = `adxl_cs_n`.
   - `DIO3` = `adxl_mosi`.
   - `DIO4` = `adxl_miso`.
   - `DIO5` = `adxl_sclk`.
4. Keep the threshold around `1.5 V` to `1.7 V`.
5. Set the sample rate high enough for SPI. Start at `20 MS/s` if the SCLK rate
   is unknown, then adjust after observing the actual clock.
6. Add a second `Custom` interpreter.
7. In the `Decoder` tab, paste the complete contents of
   `tools/waveforms/logic_custom_spi_adxl362_decoder.js`.
8. In the `Value to text` tab, paste the complete contents of
   `tools/waveforms/logic_custom_spi_adxl362_value_to_text.js`.
9. Trigger on `DIO2/adxl_cs_n` falling.
10. Save the workspace.

The SPI decoder assumes ADXL362 mode 0 behavior: CS active low, data sampled on
SCLK rising edges, MSB first, 8-bit words.

### 5. Capture And Interpret ADXL362 SPI Traffic

1. Attach the ACL2/ADXL362 Pmod to JB with correct power/ground orientation.
2. Attach AD3 DIO probes to JB without disturbing the Pmod connection.
3. Keep `DIO2/adxl_cs_n` high at idle.
4. Load or reset the FPGA design.
5. Enable the compiled ADXL362 path and the relevant runtime switch/gate.
6. Run a Logic capture triggered on CS falling.
7. If the decoder reports `SPI STATUS no CS frame`, the ADXL362 path is inactive
   in that capture. First make CS_N fall and SCLK toggle.
8. Once frames exist, confirm the first expected read is `MOSI=0B 02 00`.
9. Confirm the returned PARTID data byte is `MISO=F2`.
10. After initialization, look for XYZ burst reads from register `0x0E`.
11. If you see `ERR partial SPI byte`, increase capture duration or verify that
    CS is not deasserting mid-byte.

Do not interpret acceleration samples until PARTID is correct. A clean clock and
chip select with a wrong PARTID usually points to wiring, mode, orientation, or
power rather than acceleration physics.

### 6. Run A Passive I2C Protocol Receiver

Use this when you want console output and CSV evidence from WaveForms in
addition to Logic annotations. WaveForms has two similar-looking script
contexts; choose the one that matches the window you actually have open.

#### Option A: Protocol I2C Spy/Slave Window

Use this option when your window shows:

```text
Protocol -> I2C -> Spy/Slave
```

This is the correct passive Protocol-tool path for the live FPGA-owned I2C bus
in WaveForms builds where the Custom editor does not expose `Receiver()`.

Implementation steps:

1. Open `Protocol`.
2. Select `I2C`.
3. Set `SCL=DIO0`.
4. Set `SDA=DIO1`.
5. Set `Frequency=100 kHz` for the receiver configuration.
6. Select the `Spy/Slave` sub-tab.
7. Use the receiver/spy controls in this tab to start passive decode.
8. If available, enable timestamp from the gear menu.
9. Do not use `Master`, `Write`, `Read`, `Clear`, or active bus controls on the
   live FPGA-owned bus.
10. Save or copy the decoded transaction log as text evidence.

Use Logic Custom decoders and Scope raw-voltage measurements as the byte-level
cross-check. The built-in Spy/Slave text area is useful bench evidence, but it
is less controllable than the project-local Logic and Script-tool artifacts.

#### Option B: Protocol I2C Custom Window Compatibility Check

Your screenshot showed:

```text
Protocol -> I2C -> Custom
```

That editor has its own local API. The installed documentation and on-screen
example comments may mention `Receiver()` and `Receive()`, but your WaveForms
runtime reported:

```text
ReferenceError: Can't find variable: Receiver
```

That means this specific Custom runtime cannot run the project passive receiver
from this tab. You can paste this diagnostic instead:

```text
tools/waveforms/protocol_custom_i2c_api_probe.js
```

or:

```text
tools/waveforms/protocol_custom_i2c_passive_receiver_caelumfusion.js
```

Both now act as diagnostics and do not drive the bus. If they report that
`Receiver` and `Receive` are unavailable, use `Spy/Slave`, the Logic Custom
decoder, or the separate Script tool. Do not try to make this Custom tab passive
by calling `Read()`, `Write()`, or `Clear()`; those are active bus operations.

If you paste `script_protocol_i2c_receiver_caelumfusion.js` into this Custom
window, WaveForms reports a different context error:

```text
ReferenceError: Can't find variable: Protocol
```

#### Option C: Separate WaveForms Script Tool

Use this option only when you have opened the separate WaveForms `Script`
instrument/tool, not the Protocol Custom editor.

1. Stop the Logic acquisition if WaveForms reports that Protocol resources are
   busy.
2. Open the `Protocol` instrument.
3. Select `I2C`.
4. Configure passive receiver/spy behavior.
5. Set `SCL=DIO0`.
6. Set `SDA=DIO1`.
7. Set display format to hexadecimal.
8. Do not press any master write/read controls on the live FPGA bus.
9. Open the `Script` instrument.
10. Paste the complete contents of
    `tools/waveforms/script_protocol_i2c_receiver_caelumfusion.js`.
11. Review the header and confirm it says passive receiver.
12. Press `Run`.
13. Let the default `20 s` capture complete, or edit `CAPTURE_SECONDS` before
    running if you need a shorter/longer window.
14. Read the printed summary lines for ACK/NACK counts by address.
15. Use the printed CSV path to locate the generated capture log.

The script uses `Protocol.I2C.Receiver()` and `Protocol.I2C.Receive()`. If no
frames are printed, verify the Protocol tool is actually in I2C mode, the FPGA
is generating bus traffic, and the DIO pins are mapped correctly.

Do not paste this Script-tool version into `Protocol -> I2C -> Custom`.

### 7. Run The Passive ADXL362 SPI Protocol Receiver Script

1. Stop Logic if Protocol resources are busy.
2. Open the `Protocol` instrument.
3. Select `SPI`.
4. Configure spy/receiver behavior:
   - `Select=DIO2`, active low.
   - `Clock=DIO5`.
   - `DQ0/MOSI=DIO3`.
   - `DQ1/MISO=DIO4`.
   - `CPOL=0`.
   - `CPHA=0`.
   - `First bit=MSB`.
   - `Data bits=8`.
   - `Format=Hexadecimal`.
5. Open the `Script` instrument.
6. Paste the complete contents of
   `tools/waveforms/script_protocol_spi_adxl362_receiver_caelumfusion.js`.
7. Press `Run`.
8. Toggle or reset the FPGA path if needed to force ADXL362 initialization.
9. Confirm the script prints `READ PARTID -> 0xF2 OK`.
10. Use the generated CSV as the text evidence file for SPI transactions.

If the script prints frames but every MISO byte is `0x00` or `0xFF`, check MISO
wiring, Pmod orientation, 3.3 V power, and SPI mode before changing RTL.

### 8. Use The Isolated Active-Drive Scripts Only Off The FPGA Bus

The two isolated scripts are intentionally disabled by this line:

```javascript
var ALLOW_ACTIVE_DRIVE = false;
```

Use them only when the target Pmod is electrically isolated from FPGA-driven
pins.

1. Disconnect the Pmod signal pins from the Basys-3 FPGA pins, or guarantee the
   FPGA pins are tri-stated.
2. Provide a verified 3.3 V supply and common ground to the Pmod.
3. Connect AD3 DIO pins to the isolated Pmod signals.
4. Open the correct `Protocol` mode, I2C or SPI.
5. Paste the matching `protocol_custom_*_isolated_*` script into Protocol
   `Custom`.
6. Read the script header.
7. Change `ALLOW_ACTIVE_DRIVE` to `true` only after the wiring is isolated.
8. Run the probe.
9. Change `ALLOW_ACTIVE_DRIVE` back to `false` before saving or reusing the
   script.

Never run these isolated scripts against the live CaelumFusion harness. They are
for board-level sensor proof, not passive FPGA instrumentation.

### 9. Preserve And Review Evidence

1. Save the WaveForms workspace after both Custom interpreters are installed.
2. For each bench capture, record:
   - Date and operator.
   - Bitstream or Vivado build identity.
   - Basys switch settings.
   - Attached Pmods.
   - AD3 DIO map.
   - WaveForms workspace filename.
   - Raw Logic CSV filename.
   - Protocol receiver CSV filename, if used.
3. Keep failed captures when they show meaningful electrical evidence such as
   stuck-low SDA, missing ACK, wrong PARTID, or inactive CS.
4. Do not rename an address label to match an expectation unless the actual
   hardware address strap or RTL probe list changed.

### 10. Safe Script Edits

The most likely safe edits are the configuration constants and label tables:

| File | Safe edit | When to edit |
|---|---|---|
| `logic_custom_i2c_decoder.js` | `CFG_I2C_SCL_BIT`, `CFG_I2C_SDA_BIT` | Only after moving AD3 probes. |
| `logic_custom_i2c_value_to_text.js` | `i2cName()` labels | Only after adding/removing expected I2C addresses. |
| `logic_custom_spi_adxl362_decoder.js` | `CFG_SPI_*_BIT` constants | Only after moving AD3 probes. |
| `logic_custom_spi_adxl362_value_to_text.js` | `regName()` / `byteMeaning()` labels | Only after changing the ADXL362 register job. |
| `protocol_custom_i2c_passive_receiver_caelumfusion.js` | no normal edit | Diagnostic guard for `Protocol -> I2C -> Custom`; use `Spy/Slave` if `Receiver()` is unavailable. |
| `script_protocol_*_receiver_*.js` | `CAPTURE_SECONDS` | When changing capture window length. |
| `protocol_custom_*_isolated_*.js` | `ALLOW_ACTIVE_DRIVE` | Only for isolated, non-FPGA-connected tests. |

After editing a script, paste the whole file into WaveForms again. WaveForms
does not automatically track project-file changes after you have pasted code
into an instrument tab.

## Scope Semantic Probe And Math Layers

This Scope setup is intentionally layered. Keep raw voltage channels visible as
the electrical truth. Add semantic channels above that truth; do not replace the
raw SCL/SDA view with interpreted traces.

Assumptions for the custom math snippets:

| Scope channel | Physical signal | Connection |
|---|---|---|
| `C1` | `SCL` | `1+` to Basys/Pmod SCL, `1-` to Basys/Pmod GND. |
| `C2` | `SDA` | `2+` to Basys/Pmod SDA, `2-` to Basys/Pmod GND. |

AD3 GND, `1-`, and `2-` must all reference the same Basys/Pmod ground. If the
bottom-left readout shows impossible values such as `+/-28.9 V` on a 3.3 V I2C
bus, stop before adding interpretation. Fix probe transform, ground reference,
channel polarity, or wiring first.

### Layer 1: Fix The Probe First

Create one identity probe for passive 3.3 V I2C voltage measurements:

| Field | Value |
|---|---|
| Name | `I2C_3V3_PASS` |
| Unit | `V` |
| Fixed input range | enabled |
| Top | `5 V` |
| Bottom | `-1 V` |
| Voltage input to unit | `Input` |
| Unit to voltage for trigger level | `Output` |

Implementation steps:

1. Open `Scope`.
2. Open the channel/probe configuration dialog for `SCL (1+-)`.
3. Add or duplicate a probe entry.
4. Set the probe name to `I2C_3V3_PASS`.
5. Set `Unit` to `V`.
6. Enable `Fixed input range`.
7. Set `Top` to `5 V`.
8. Set `Bottom` to `-1 V`.
9. Open the `Custom` tab.
10. Replace the example input transform with the exact contents of
    `tools/waveforms/scope_probe_i2c_3v3_pass_input.js`:

```javascript
Input
```

11. Replace the example trigger/output transform with the exact contents of
    `tools/waveforms/scope_probe_i2c_3v3_pass_output.js`:

```javascript
Output
```

12. Click `Evaluate` if available.
13. Click `Apply`.
14. Select `I2C_3V3_PASS` as the probe for `SCL (1+-)`.
15. Select the same `I2C_3V3_PASS` probe for `SDA (2+-)`.
16. Set both raw channels to `DC` coupling.
17. Confirm idle I2C levels are physically plausible: high near `3.3 V`, low
    near `0 V`.

The pass-through probe is not a decoder. Its purpose is to remove the default
example transform from the measurement path so raw volts remain raw volts.

### Layer 2: Add Meaningful Math Channels

Start with only `SCL_DIG_1V65` and `SDA_DIG_1V65`. Once those match the raw
voltage traces, add `I2C_BUS_MIN` and `SDA_MINUS_SCL`.

For each math channel:

1. In `Scope`, click `Add Channel`.
2. Choose `Math`.
3. Set math mode to `Custom`.
4. Set the channel name and unit from the table below.
5. Paste the matching script into the Custom Math editor.
6. Click `Evaluate`.
7. Click `Apply`.
8. Keep the raw `SCL` and `SDA` channels enabled.

| Name | Unit | Paste file | Custom function |
|---|---|---|---|
| `SCL_DIG_1V65` | `logic` | `tools/waveforms/scope_math_scl_dig_1v65.js` | `if(C1 > 1.65) return 3.3;` then `return 0;` |
| `SDA_DIG_1V65` | `logic` | `tools/waveforms/scope_math_sda_dig_1v65.js` | `if(C2 > 1.65) return 3.3;` then `return 0;` |
| `I2C_BUS_MIN` | `V` | `tools/waveforms/scope_math_i2c_bus_min.js` | `return Math.min(C1, C2);` |
| `SDA_MINUS_SCL` | `V` | `tools/waveforms/scope_math_sda_minus_scl.js` | `return C2 - C1;` |

Interpretation:

| Channel | What it proves |
|---|---|
| Raw `SCL` / raw `SDA` | Electrical bus levels, noise, edge shape, and real voltage range. |
| `SCL_DIG_1V65` / `SDA_DIG_1V65` | Whether WaveForms would classify each line as logical high or low at a 1.65 V threshold. |
| `I2C_BUS_MIN` | Compact activity view; it drops whenever either line is pulled low. |
| `SDA_MINUS_SCL` | Relative timing view; near `0 V` means same state, about `-3.3 V` means SCL high/SDA low, about `+3.3 V` means SDA high/SCL low. |

If WaveForms on a specific installation rejects `return` statements in the Math
editor, use the equivalent expression-only forms:

```javascript
C1 > 1.65 ? 3.3 : 0
C2 > 1.65 ? 3.3 : 0
Math.min(C1, C2)
C2 - C1
```

### Layer 3: Add Measurements To Raw Channels

Add measurements to raw `SCL` and raw `SDA`, not to the digitalized math
channels. The math channels are interpretive renderings; the raw channels are
the measurement evidence.

Recommended raw-channel measurements:

| Measurement | Channel | Purpose |
|---|---|---|
| `High` | `SCL`, `SDA` | Confirms high level is near the 3.3 V rail. |
| `Low` | `SCL`, `SDA` | Confirms low level is near ground. |
| `Amplitude` | `SCL`, `SDA` | Confirms usable logic swing. |
| `RiseTime` | `SCL`, `SDA` | Checks pullup strength, bus capacitance, and edge rounding. |
| `FallTime` | `SCL`, `SDA` | Checks low-going edge quality. |
| `Frequency` | `SCL` | Confirms I2C clock rate. |
| `Positive Width` | `SCL` | Checks SCL high time. |
| `Negative Width` | `SCL` | Checks SCL low time. |

Healthy 3.3 V I2C expectations:

| Quantity | Expected result |
|---|---|
| High level | Near `3.3 V`. |
| Low level | Near `0 V`. |
| Trigger threshold | Around `1.6 V`. |
| SCL frequency | At or below the attached device limit, typically `100 kHz` or `400 kHz`. |
| Rise edge | Not excessively slow, rounded, or unable to cross the threshold cleanly. |
| Fall edge | Fast enough to reach a valid low level without bounce-driven false edges. |

Recommended display order:

1. `SCL` raw voltage.
2. `SDA` raw voltage.
3. `SCL_DIG_1V65`.
4. `SDA_DIG_1V65`.
5. `I2C_BUS_MIN`.
6. `SDA_MINUS_SCL`.

Bring this up incrementally. First implement only `I2C_3V3_PASS`,
`SCL_DIG_1V65`, and `SDA_DIG_1V65`. After those agree with the raw voltage
traces, add `I2C_BUS_MIN` and `SDA_MINUS_SCL` as higher-level interpretation
channels.

## Passive Logic Custom Decoders

### Shared I2C

1. Open WaveForms `Logic`.
2. Add raw signals for `DIO0` and `DIO1`; name them `scl` and `sda`.
3. Add `Custom`.
4. In the `Decoder` tab, paste `tools/waveforms/logic_custom_i2c_decoder.js`.
5. In the `Value to text` tab, paste `tools/waveforms/logic_custom_i2c_value_to_text.js`.
6. Capture with enough pretrigger to include a clean START.

Expected useful labels include:

| Label pattern | Meaning |
|---|---|
| `F0 START` | I2C frame or segment began. |
| `F0 B0 ADDR raw=0x30 addr7=0x18 W NACK LIS3DH SA0=0` | Raw byte `0x30` was the address phase; decoded 7-bit address `0x18`; write; NACK. |
| `F0 B1 DATA 0x.. ACK ctx=0x18 W LIS3DH SA0=0` | Data byte with byte index and address/direction context. |
| `F0 STOP bytes=... data=...` | Segment completed; total bytes and data-byte count are shown. |
| `ERR partial before STOP` | Capture ended or STOP occurred inside an incomplete byte. |

Scientific interpretation boundary: an address label is not proof of a working
sensor. A working sensor requires `ACK` on the address byte and plausible
follow-on data transactions. Repeated `0x18` / `0x19` `NACK` traffic means the
FPGA master is probing LIS3DH addresses but has not found an ACKing LIS3DH.

### ADXL362 / ACL2 SPI

1. Open WaveForms `Logic`.
2. Add raw signals for `DIO2..DIO5`; name them `adxl_cs_n`, `adxl_mosi`,
   `adxl_miso`, and `adxl_sclk`.
3. Add `Custom`.
4. In the `Decoder` tab, paste `tools/waveforms/logic_custom_spi_adxl362_decoder.js`.
5. In the `Value to text` tab, paste `tools/waveforms/logic_custom_spi_adxl362_value_to_text.js`.
6. Trigger on `DIO2` falling.

Expected active-job transaction sequence:

| Operation | MOSI bytes | Expected meaning |
|---|---|---|
| No active frame | no CS fall | Decoder emits `SPI STATUS no CS frame`; do not expect PARTID. |
| Read PARTID | `0B 02 00` | MISO byte 2 should be `F2`. |
| Write INTMAP1 | `0A 2A 01` | Map DATA_READY to active-high INT1. |
| Write INTMAP2 | `0A 2B 00` | Leave INT2 unmapped. |
| Write POWER_CTL | `0A 2D 02` | Enter measurement mode. |
| Read XYZ | `0B 0E 00 00 00 00 00 00` | MISO bytes 2..7 are X/Y/Z little-endian raw samples. |

## Passive Protocol Receiver Scripts

The Protocol receiver scripts are useful when you want text and CSV evidence in
addition to waveform annotations.

### I2C Receiver

For the `Protocol -> I2C -> Spy/Slave` tab, use WaveForms' built-in passive
receiver. This is the correct Protocol-window path for the runtime that reports
`ReferenceError: Can't find variable: Receiver` in the Custom tab.

For the `Protocol -> I2C -> Custom` editor, only paste diagnostics such as
`tools/waveforms/protocol_custom_i2c_api_probe.js` or
`tools/waveforms/protocol_custom_i2c_passive_receiver_caelumfusion.js`. In the
observed WaveForms build, these confirm that passive `Receiver()` / `Receive()`
are unavailable there.

For the separate WaveForms `Script` tool, paste and run
`tools/waveforms/script_protocol_i2c_receiver_caelumfusion.js`.

The separate Script-tool version uses `Protocol.I2C.Receiver()` /
`Protocol.I2C.Receive()`. Do not paste it into Protocol Custom.

The script writes a CSV file named like
`caelumfusion_i2c_protocol_capture_YYYYMMDD_HHMMSS.csv` beside the current
WaveForms workspace when the separate Script-tool version is used and
`Tool.workspaceDir()` is available.

### ADXL362 SPI Receiver

1. Open `Protocol`.
2. Select `SPI`.
3. Configure Spy mode:
   - `Select=DIO2`, active low.
   - `Clock=DIO5`.
   - `DQ0/MOSI=DIO3`.
   - `DQ1/MISO=DIO4`.
   - Standard 4-wire, CPOL=0, CPHA=0, MSB first, 8-bit words.
4. Open `Script`.
5. Paste and run `tools/waveforms/script_protocol_spi_adxl362_receiver_caelumfusion.js`.

The script labels PARTID, INTMAP, POWER_CTL, and XYZ burst reads. The displayed
`g_default2g` values assume the ADXL362 default +/-2 g, 1 mg/LSB scaling because
the active job does not program a different range register.

## Active-Drive Scripts

The two `protocol_custom_*_isolated_*` scripts are intentionally disabled by
default:

```javascript
var ALLOW_ACTIVE_DRIVE = false;
```

Only set that to `true` when the FPGA is disconnected from the bus, the FPGA
pins are known tri-stated, or the Pmod is wired as a standalone device under
AD3 control. These scripts are useful for proving a sensor board works
independently from the FPGA design, but they are not passive probe scripts.

## Every-Byte Validation Checklist

Use this checklist before treating a capture as evidence.

1. Verify AD3 ground, Scope `1-`, Scope `2-`, Basys GND, and Pmod GND are the
   same reference.
2. Verify raw Scope voltage first: idle I2C high near `3.3 V`, low near `0 V`,
   and no impossible readings such as `+/-28.9 V`.
3. Verify the raw digital Logic traces agree with the Scope threshold.
4. Verify the expected idle state:
   - I2C: `SCL=1`, `SDA=1`.
   - ADXL362 SPI: `CS_N=1`, `SCLK=0`, MOSI parked by RTL, MISO not interpreted
     until CS is active.
5. Verify protocol framing:
   - I2C has START/RESTART/STOP structure.
   - SPI bytes occur only while CS is active.
6. For I2C, verify every address phase includes:
   - raw address byte,
   - decoded 7-bit address,
   - R/W direction,
   - ACK/NACK.
7. For I2C, verify every data byte includes ACK/NACK. A sensor label without
   ACK is not proof of a working sensor.
8. For SPI, verify every byte includes byte index, MOSI, MISO, and CS-frame
   context.
9. For ADXL362, verify `READ PARTID` returns `0xF2` before interpreting XYZ
   data as acceleration.
10. Export raw captures and decoded CSV logs where available.
11. Cross-check important I2C captures with the host decoder before using them
   in reports.
12. Archive the date, bitstream/build identity, switch settings, attached
   Pmods, harness map, WaveForms workspace, raw CSV, decoded CSV, and script
   revision.

## Troubleshooting Guide

| Symptom | Likely cause | Immediate action |
|---|---|---|
| Scope shows impossible voltages such as `+/-28.9 V` | Bad probe transform, floating differential input, missing ground, or wrong reference | Apply `I2C_3V3_PASS`; tie `1-`, `2-`, and AD3 GND to Basys/Pmod GND before decoding. |
| I2C `SCL` or `SDA` stuck low | Miswired Pmod, unpowered device, too many pullups/loads, FPGA holding line, or bus contention | Remove devices one at a time; verify idle high with only Basys and pullups. |
| Address label appears but always `NACK` | FPGA is probing but no device answered at that address | Check Pmod orientation, address strap, 3.3 V rail, ground, and whether the device is actually attached. |
| `capture began mid-frame` or `capture ended before STOP` | Capture window did not include full transaction | Increase capture time, pretrigger, or trigger on START/CS falling. |
| I2C data bytes decode but ACKs look wrong | Threshold, sample rate, or glitch filtering mismatch | Recheck raw Scope levels and Logic sample rate before changing RTL. |
| SPI `unknown_command` | Wrong pins, wrong mode, wrong bit order, or not actually observing ADXL362 traffic | Confirm `CPOL=0`, `CPHA=0`, MSB first, 8-bit, CS active low, and DIO2-DIO5 map. |
| SPI `miso_all_00` | MISO short/low, unpowered sensor, wrong orientation, or no response | Check 3.3 V, GND, JB orientation, and MISO continuity. |
| SPI `miso_all_FF` | Floating/high MISO, missing sensor, pullup-only response, wrong CS | Check CS activity, Pmod power, and MISO wiring. |
| ADXL362 PARTID not `0xF2` | Wrong SPI mode, wrong device, wrong wiring, or inactive sensor | Do not interpret XYZ data; debug PARTID first. |
| WaveForms reports `Can't find variable: Receiver` | Protocol Custom runtime does not expose passive receiver helpers | Use `Protocol -> I2C -> Spy/Slave`, Logic Custom decoders, or the separate Script tool if `Protocol.I2C` is available. |
| WaveForms reports `Can't find variable: Protocol` | Script-tool code was pasted into Protocol Custom | Do not use the Script-tool file in Protocol Custom; use `Spy/Slave`, Logic Custom decoders, or open the separate Script tool. |
| Active-drive script refuses to run | `ALLOW_ACTIVE_DRIVE=false` guard is working | Only set it true after the FPGA/MCU bus owner is disconnected or tri-stated. |

## Deterministic Cross-Check

For archived I2C captures, export raw Logic CSV and run the existing host
decoder:

```powershell
python tools\decode_waveforms_i2c.py .\capture.csv --expect-addr 0x18,0x19,0x30,0x38,0x40,0x47,0x68,0x69,0x1E --glitch-ns 100
```

The host decoder separates true `NACK` evidence from capture-boundary decode
errors. Use it for reportable results, and use the WaveForms scripts for rapid
bench feedback.

## Recommended Next Development Steps

1. Save a WaveForms workspace with the named DIO channels and both custom
   interpreters already installed.
2. Add the `I2C_3V3_PASS` Scope probe and the first two math channels,
   `SCL_DIG_1V65` and `SDA_DIG_1V65`, then save that workspace as the baseline
   Scope template.
3. Capture a short JA I2C trace with only one known-good I2C device attached and
   archive the raw CSV plus script-generated CSV.
4. Add `I2C_BUS_MIN` and `SDA_MINUS_SCL` only after the raw and digitalized
   Scope traces agree.
5. Capture a JB ADXL362 trace triggered on `CS_N` falling and confirm `PARTID=F2`
   before interpreting acceleration samples.
6. Update `docs/bench_captures/index.md` with the programmed bitstream hash,
   switch settings, attached Pmods, WaveForms workspace path, raw/decoded CSV
   filenames, and PARTID result for each reportable capture.
