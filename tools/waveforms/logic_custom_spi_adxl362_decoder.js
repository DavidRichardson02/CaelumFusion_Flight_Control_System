/*
 * CaelumFusion ADXL362/Pmod ACL2 SPI custom Logic interpreter decoder.
 *
 * WaveForms paste target:
 *   Logic Analyzer -> Add -> Custom -> Decoder
 *
 * Passive probe map:
 *   DIO2 = JB1 / adxl362_cs_n
 *   DIO3 = JB2 / adxl362_mosi
 *   DIO4 = JB3 / adxl362_miso
 *   DIO5 = JB4 / adxl362_sclk
 *
 * Protocol contract:
 *   ADXL362 SPI mode 0, CS_N active low, MSB first, 8-bit words.
 *   MOSI/MISO are sampled on SCLK rising edges while CS_N is low.
 *
 * Output packing for BYTE events:
 *   bits  0..7   MISO byte
 *   bits  8..15  MOSI byte
 *   bits 16..19  byte index inside CS frame
 *   bits 20..23  command code: 1=WRITE_REG, 2=READ_REG, 15=unknown
 *   bits 24..31  register address context, 0xFF when unknown
 *
 * Output packing for STOP:
 *   bits  0..7   completed byte count
 *   bits  8..15  command byte, or 0xFF
 *   bits 16..23  register byte, or 0xFF
 *
 * Output packing for STATUS:
 *   bits  0..3   idle input levels: CS, MOSI, MISO, SCLK
 *   bits  8..15  status code
 *   bits 16..23  frame count
 */

var CFG_SPI_CS_BIT   = 2;
var CFG_SPI_MOSI_BIT = 3;
var CFG_SPI_MISO_BIT = 4;
var CFG_SPI_SCLK_BIT = 5;

var SPI_FLAG_START  = 1;
var SPI_FLAG_STOP   = 2;
var SPI_FLAG_BYTE   = 3;
var SPI_FLAG_STATUS = 4;
var SPI_FLAG_ERROR  = 15;

var SPI_CMD_WRITE_REG = 1;
var SPI_CMD_READ_REG  = 2;
var SPI_CMD_UNKNOWN   = 15;

var SPI_STATUS_NO_CS_FRAME = 1;
var SPI_STATUS_CAPTURE_STARTS_CS_LOW = 2;

var SPI_ERR_PARTIAL_BYTE_AT_CS_HIGH = 1;
var SPI_ERR_CAPTURE_ENDED_CS_LOW    = 2;

function bitAt(sample, bit) {
    return (sample >>> bit) & 1;
}

function clampIndex(index) {
    if (index < 0) {
        return 0;
    }
    if (index >= rgData.length) {
        return rgData.length - 1;
    }
    return index;
}

function emitSpi(index, flag, value) {
    index = clampIndex(index);
    rgValue[index] = value >>> 0;
    rgFlag[index] = flag;
}

function cmdCode(cmd) {
    if ((cmd & 0xFF) === 0x0A) {
        return SPI_CMD_WRITE_REG;
    }
    if ((cmd & 0xFF) === 0x0B) {
        return SPI_CMD_READ_REG;
    }
    return SPI_CMD_UNKNOWN;
}

function packByte(byteIndex, mosi, miso, cmd, reg) {
    return ((((reg & 0xFF) << 24) >>> 0) |
            ((cmdCode(cmd) & 0x0F) << 20) |
            ((byteIndex & 0x0F) << 16) |
            ((mosi & 0xFF) << 8) |
            (miso & 0xFF)) >>> 0;
}

function packStop(byteCount, cmd, reg) {
    return ((((reg & 0xFF) << 16) >>> 0) |
            ((cmd & 0xFF) << 8) |
            (byteCount & 0xFF)) >>> 0;
}

function packStatus(code, frameCount, cs, mosi, miso, sclk) {
    var levels = ((cs & 1) << 0) | ((mosi & 1) << 1) |
                 ((miso & 1) << 2) | ((sclk & 1) << 3);
    return (((frameCount & 0xFF) << 16) |
            ((code & 0xFF) << 8) |
            levels) >>> 0;
}

function packError(code, bitCount, byteCount) {
    return (((code & 0xFF) << 16) |
            ((byteCount & 0xFF) << 8) |
            (bitCount & 0xFF)) >>> 0;
}

function clearFrame() {
    frameByteIndices = [];
    frameMosi = [];
    frameMiso = [];
    frameBitCount = 0;
    frameMosiByte = 0;
    frameMisoByte = 0;
}

function emitFrameBytes(cmd, reg) {
    for (var k = 0; k < frameMosi.length; k++) {
        emitSpi(frameByteIndices[k], SPI_FLAG_BYTE,
                packByte(k, frameMosi[k], frameMiso[k], cmd, reg));
    }
}

var i;
var n = rgData.length;
for (i = 0; i < n; i++) {
    rgValue[i] = 0;
    rgFlag[i] = 0;
}

var frameByteIndices = [];
var frameMosi = [];
var frameMiso = [];
var frameBitCount = 0;
var frameMosiByte = 0;
var frameMisoByte = 0;

if (n > 1) {
    var initialCs = bitAt(rgData[0], CFG_SPI_CS_BIT);
    var initialMosi = bitAt(rgData[0], CFG_SPI_MOSI_BIT);
    var initialMiso = bitAt(rgData[0], CFG_SPI_MISO_BIT);
    var initialSclk = bitAt(rgData[0], CFG_SPI_SCLK_BIT);
    var prevCs = initialCs;
    var prevSclk = initialSclk;
    var inFrame = (prevCs === 0);
    var frameCount = 0;
    var sawCsFrame = inFrame;

    clearFrame();
    if (inFrame) {
        emitSpi(0, SPI_FLAG_STATUS,
                packStatus(SPI_STATUS_CAPTURE_STARTS_CS_LOW, frameCount,
                           initialCs, initialMosi, initialMiso, initialSclk));
        emitSpi(0, SPI_FLAG_START, 0);
    }

    for (i = 1; i < n; i++) {
        var cs = bitAt(rgData[i], CFG_SPI_CS_BIT);
        var sclk = bitAt(rgData[i], CFG_SPI_SCLK_BIT);
        var mosi = bitAt(rgData[i], CFG_SPI_MOSI_BIT);
        var miso = bitAt(rgData[i], CFG_SPI_MISO_BIT);

        var csFall = (prevCs === 1) && (cs === 0);
        var csRise = (prevCs === 0) && (cs === 1);
        var sclkRise = (prevSclk === 0) && (sclk === 1);

        if (csFall) {
            sawCsFrame = true;
            inFrame = true;
            clearFrame();
            emitSpi(i, SPI_FLAG_START, 0);
        } else if (csRise) {
            if (inFrame) {
                var cmd = frameMosi.length > 0 ? frameMosi[0] : 0xFF;
                var reg = frameMosi.length > 1 ? frameMosi[1] : 0xFF;
                if (frameBitCount !== 0) {
                    emitSpi(i, SPI_FLAG_ERROR,
                            packError(SPI_ERR_PARTIAL_BYTE_AT_CS_HIGH,
                                      frameBitCount, frameMosi.length));
                }
                emitFrameBytes(cmd, reg);
                emitSpi(i, SPI_FLAG_STOP, packStop(frameMosi.length, cmd, reg));
                frameCount++;
            }
            inFrame = false;
            clearFrame();
        } else if (inFrame && sclkRise) {
            frameMosiByte = ((frameMosiByte << 1) | mosi) & 0xFF;
            frameMisoByte = ((frameMisoByte << 1) | miso) & 0xFF;
            frameBitCount++;

            if (frameBitCount === 8) {
                frameByteIndices.push(i);
                frameMosi.push(frameMosiByte);
                frameMiso.push(frameMisoByte);
                frameBitCount = 0;
                frameMosiByte = 0;
                frameMisoByte = 0;
            }
        }

        prevCs = cs;
        prevSclk = sclk;
    }

    if (inFrame) {
        var endCmd = frameMosi.length > 0 ? frameMosi[0] : 0xFF;
        var endReg = frameMosi.length > 1 ? frameMosi[1] : 0xFF;
        emitFrameBytes(endCmd, endReg);
        emitSpi(n - 1, SPI_FLAG_ERROR,
                packError(SPI_ERR_CAPTURE_ENDED_CS_LOW,
                          frameBitCount, frameMosi.length));
    }

    if (!sawCsFrame) {
        emitSpi(0, SPI_FLAG_STATUS,
                packStatus(SPI_STATUS_NO_CS_FRAME, frameCount,
                           initialCs, initialMosi, initialMiso, initialSclk));
    }
}
