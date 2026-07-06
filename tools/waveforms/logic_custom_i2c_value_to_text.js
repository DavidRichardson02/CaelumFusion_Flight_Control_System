/*
 * CaelumFusion shared-I2C custom Logic interpreter Value to text script.
 *
 * WaveForms paste target:
 *   Logic Analyzer -> Add -> Custom -> Value to text
 *
 * This renders the packed values emitted by logic_custom_i2c_decoder.js.
 */

function hex2(v) {
    var s = (v & 0xFF).toString(16).toUpperCase();
    return (s.length < 2) ? ("0" + s) : s;
}

function i2cName(addr) {
    switch (addr & 0x7F) {
        case 0x18: return "LIS3DH SA0=0";
        case 0x19: return "LIS3DH SA0=1";
        case 0x1E: return "LIS2MDL MAG1";
        case 0x30: return "CMPS2/MMC3416 MAG0";
        case 0x38: return "PMON1";
        case 0x40: return "HYGRO/HDC1080";
        case 0x46: return "BMP585 alt";
        case 0x47: return "BMP585";
        case 0x68: return "GYRO alt";
        case 0x69: return "GYRO";
        default: return "unmapped";
    }
}

function frameId(v) {
    return (v >>> 24) & 0x0F;
}

function ackText(f) {
    return ((f === 4) || (f === 6)) ? "ACK" : "NACK";
}

var f = (typeof flag !== "undefined") ? flag : 0;
var v = (typeof value !== "undefined") ? value >>> 0 : 0;
var text = "";

if (f === 1) {
    text = "F" + frameId(v) + " START";
} else if (f === 2) {
    text = "F" + frameId(v) + " RESTART";
} else if (f === 3) {
    var dataBytes = v & 0xFF;
    var totalBytes = (v >>> 8) & 0xFF;
    text = "F" + frameId(v) + " STOP bytes=" + totalBytes +
           " data=" + dataBytes;
} else if ((f === 4) || (f === 5)) {
    var addrByte = v & 0xFF;
    var byteIndex = (v >>> 8) & 0xFF;
    var addr7 = (v >>> 16) & 0x7F;
    var rw = ((v >>> 23) & 1) ? "R" : "W";
    text = "F" + frameId(v) +
           " B" + byteIndex +
           " ADDR raw=0x" + hex2(addrByte) +
           " addr7=0x" + hex2(addr7) +
           " " + rw +
           " " + ackText(f) +
           " " + i2cName(addr7);
} else if ((f === 6) || (f === 7)) {
    var dataByte = v & 0xFF;
    var dataIndex = (v >>> 8) & 0xFF;
    var ctxAddr = (v >>> 16) & 0x7F;
    var ctxRw = ((v >>> 23) & 1) ? "R" : "W";
    text = "F" + frameId(v) +
           " B" + dataIndex +
           " DATA 0x" + hex2(dataByte) +
           " " + ackText(f) +
           " ctx=0x" + hex2(ctxAddr) +
           " " + ctxRw +
           " " + i2cName(ctxAddr);
} else if (f === 15) {
    var bitCount = v & 0xFF;
    var errByteIndex = (v >>> 8) & 0xFF;
    var code = (v >>> 16) & 0xFF;
    if (code === 1) {
        text = "F" + frameId(v) + " ERR partial before START B" +
               errByteIndex + " bits=" + bitCount;
    } else if (code === 2) {
        text = "F" + frameId(v) + " ERR partial before STOP B" +
               errByteIndex + " bits=" + bitCount;
    } else if (code === 3) {
        text = "F" + frameId(v) + " ERR capture ended in frame B" +
               errByteIndex + " bits=" + bitCount;
    } else {
        text = "F" + frameId(v) + " ERR code=" + code +
               " B" + errByteIndex + " bits=" + bitCount;
    }
} else {
    text = "";
}

text;
