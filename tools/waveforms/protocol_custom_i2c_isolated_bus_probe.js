/*
 * CaelumFusion isolated-bus I2C address probe for WaveForms Protocol I2C
 * Custom mode.
 *
 * WARNING:
 *   This script drives SCL/SDA. Do not run it on the live Basys-3 FPGA-owned
 *   bus. Use only when the FPGA pins are disconnected, tri-stated, or the
 *   target Pmod is wired as an isolated device-under-test controlled by AD3.
 *
 * WaveForms use:
 *   Protocol -> I2C -> Custom
 *   Set SCL=DIO0, SDA=DIO1, Frequency conservatively, then paste this script.
 *   Set ALLOW_ACTIVE_DRIVE to true only after the isolation check is complete.
 */

var ALLOW_ACTIVE_DRIVE = false;

var addrs = [
    0x18, /* LIS3DH SA0=0 */
    0x19, /* LIS3DH SA0=1 */
    0x1E, /* LIS2MDL */
    0x30, /* CMPS2/MMC3416 */
    0x38, /* PMON1 */
    0x40, /* HYGRO/HDC1080 */
    0x46, /* BMP585 alternate */
    0x47, /* BMP585 preferred */
    0x68, /* GYRO alternate */
    0x69  /* GYRO preferred */
];

function hex2(v) {
    var s = (v & 0xFF).toString(16).toUpperCase();
    return (s.length < 2) ? ("0" + s) : s;
}

if (!ALLOW_ACTIVE_DRIVE) {
    throw "Refusing to drive I2C. Disconnect/tri-state the FPGA bus, then set ALLOW_ACTIVE_DRIVE=true.";
}

if (!Clear()) {
    throw "I2C bus is not free; SDA may be held low.";
}

for (var i = 0; i < addrs.length; i++) {
    var addr = addrs[i];
    var ack = Write(addr);
    print("I2C address 0x" + hex2(addr) + " " + (ack ? "ACK" : "NACK"));
}

try {
    DIO.Clear();
} catch (e) {
}
