/*
 * CaelumFusion isolated ADXL362/Pmod ACL2 SPI probe for WaveForms Protocol SPI
 * Custom mode.
 *
 * WARNING:
 *   This script drives CS_N/SCLK/MOSI. Do not run it while the Basys-3 FPGA is
 *   connected to the same ADXL362 SPI pins. Use only with the Pmod isolated
 *   from the FPGA or with FPGA outputs known tri-stated.
 *
 * WaveForms use:
 *   Protocol -> SPI -> Custom
 *   Select=DIO2 active low, Clock=DIO5, DQ0/MOSI=DIO3, DQ1/MISO=DIO4,
 *   CPOL=0, CPHA=0, First bit=MSB, 8-bit words.
 *   Set ALLOW_ACTIVE_DRIVE to true only after the isolation check is complete.
 */

var ALLOW_ACTIVE_DRIVE = false;

function hex2(v) {
    var s = (v & 0xFF).toString(16).toUpperCase();
    return (s.length < 2) ? ("0" + s) : s;
}

function signed16(lo, hi) {
    var v = ((hi & 0xFF) << 8) | (lo & 0xFF);
    return (v & 0x8000) ? (v - 0x10000) : v;
}

if (!ALLOW_ACTIVE_DRIVE) {
    throw "Refusing to drive SPI. Disconnect/tri-state FPGA pins, then set ALLOW_ACTIVE_DRIVE=true.";
}

Start();
var partid = ReadWrite(8, [0x0B, 0x02, 0x00]);
Stop();
print("ADXL362 PARTID response raw:", partid);
if (partid.length >= 3) {
    print("ADXL362 PARTID=0x" + hex2(partid[2]) + (partid[2] === 0xF2 ? " OK" : " unexpected"));
}

Start();
var xyz = ReadWrite(8, [0x0B, 0x0E, 0, 0, 0, 0, 0, 0]);
Stop();
print("ADXL362 XYZ response raw:", xyz);
if (xyz.length >= 8) {
    var ax = signed16(xyz[2], xyz[3]);
    var ay = signed16(xyz[4], xyz[5]);
    var az = signed16(xyz[6], xyz[7]);
    print("ADXL362 raw ax ay az:", ax, ay, az);
    print("ADXL362 g assuming default +/-2g scale:", ax / 1000.0, ay / 1000.0, az / 1000.0);
}

try {
    DIO.Clear();
} catch (e) {
}
