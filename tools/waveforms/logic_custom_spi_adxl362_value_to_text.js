/*
 * CaelumFusion ADXL362/Pmod ACL2 SPI custom Logic interpreter Value to text.
 *
 * WaveForms paste target:
 *   Logic Analyzer -> Add -> Custom -> Value to text
 *
 * Renders packed values emitted by logic_custom_spi_adxl362_decoder.js.
 */

function hex2(v) {
    var s = (v & 0xFF).toString(16).toUpperCase();
    return (s.length < 2) ? ("0" + s) : s;
}

function regName(v) {
    switch (v & 0xFF) {
        case 0x02: return "PARTID";
        case 0x0E: return "XDATA_L/XYZ_BURST";
        case 0x0F: return "XDATA_H";
        case 0x10: return "YDATA_L";
        case 0x11: return "YDATA_H";
        case 0x12: return "ZDATA_L";
        case 0x13: return "ZDATA_H";
        case 0x2A: return "INTMAP1";
        case 0x2B: return "INTMAP2";
        case 0x2D: return "POWER_CTL";
        case 0xFF: return "REG_UNKNOWN";
        default: return "REG?";
    }
}

function cmdName(code) {
    if (code === 1) {
        return "WRITE_REG";
    }
    if (code === 2) {
        return "READ_REG";
    }
    return "CMD_UNKNOWN";
}

function byteMeaning(idx, cmdCode, reg, mosi, miso) {
    if (idx === 0) {
        if (mosi === 0x0A) {
            return "CMD WRITE_REG";
        }
        if (mosi === 0x0B) {
            return "CMD READ_REG";
        }
        return "CMD unknown";
    }
    if (idx === 1) {
        return "REG " + regName(mosi);
    }
    if (cmdCode === 1) {
        if (reg === 0x2A && mosi === 0x01) {
            return "WRITE INTMAP1 DATA_READY->INT1";
        }
        if (reg === 0x2B && mosi === 0x00) {
            return "WRITE INTMAP2 unmapped";
        }
        if (reg === 0x2D && mosi === 0x02) {
            return "WRITE POWER_CTL measure";
        }
        return "WRITE data";
    }
    if (cmdCode === 2) {
        if (reg === 0x02 && idx === 2) {
            return "PARTID response " + (miso === 0xF2 ? "OK" : "unexpected");
        }
        if (reg === 0x0E && idx >= 2 && idx <= 7) {
            return "XYZ response byte";
        }
        return "READ response/dummy";
    }
    return "unmapped";
}

var f = (typeof flag !== "undefined") ? flag : 0;
var v = (typeof value !== "undefined") ? value >>> 0 : 0;
var text = "";

if (f === 1) {
    text = "CS low: ADXL362 frame start";
} else if (f === 2) {
    var byteCount = v & 0xFF;
    var cmd = (v >>> 8) & 0xFF;
    var reg = (v >>> 16) & 0xFF;
    text = "CS high: bytes=" + byteCount +
           " cmd=0x" + hex2(cmd) +
           " reg=0x" + hex2(reg) + " " + regName(reg);
} else if (f === 3) {
    var miso = v & 0xFF;
    var mosi = (v >>> 8) & 0xFF;
    var idx = (v >>> 16) & 0x0F;
    var cmdCode = (v >>> 20) & 0x0F;
    var ctxReg = (v >>> 24) & 0xFF;
    text = "#" + idx +
           " " + cmdName(cmdCode) +
           " " + regName(ctxReg) +
           " MOSI=0x" + hex2(mosi) +
           " MISO=0x" + hex2(miso) +
           " " + byteMeaning(idx, cmdCode, ctxReg, mosi, miso);
} else if (f === 4) {
    var levels = v & 0x0F;
    var code = (v >>> 8) & 0xFF;
    var frames = (v >>> 16) & 0xFF;
    var cs = (levels >>> 0) & 1;
    var mosiLevel = (levels >>> 1) & 1;
    var misoLevel = (levels >>> 2) & 1;
    var sclk = (levels >>> 3) & 1;
    if (code === 1) {
        text = "SPI STATUS no CS frame: idle CS=" + cs +
               " SCLK=" + sclk +
               " MOSI=" + mosiLevel +
               " MISO=" + misoLevel +
               " frames=" + frames;
    } else if (code === 2) {
        text = "SPI STATUS capture began with CS low: initial CS=" + cs +
               " SCLK=" + sclk +
               " MOSI=" + mosiLevel +
               " MISO=" + misoLevel;
    } else {
        text = "SPI STATUS code=" + code + " frames=" + frames;
    }
} else if (f === 15) {
    var bitCount = v & 0xFF;
    var errBytes = (v >>> 8) & 0xFF;
    var errCode = (v >>> 16) & 0xFF;
    if (errCode === 1) {
        text = "ERR partial SPI byte at CS high bytes=" + errBytes +
               " bits=" + bitCount;
    } else if (errCode === 2) {
        text = "ERR capture ended with CS low bytes=" + errBytes +
               " bits=" + bitCount;
    } else {
        text = "ERR code=" + errCode + " bytes=" + errBytes +
               " bits=" + bitCount;
    }
} else {
    text = "";
}

text;
