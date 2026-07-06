/*
 * CaelumFusion shared-I2C custom Logic interpreter decoder.
 *
 * WaveForms paste target:
 *   Logic Analyzer -> Add -> Custom -> Decoder
 *
 * Passive probe map:
 *   DIO0 = JA3 / scl
 *   DIO1 = JA4 / sda
 *
 * Contract:
 *   This is the primary passive byte-validation path for the live
 *   CaelumFusion FPGA-owned I2C bus. It decodes only captured Logic samples;
 *   it never drives SCL/SDA.
 *
 * Output packing for byte events:
 *   bits  0..7   raw byte on wire
 *   bits  8..15  byte index within current I2C segment
 *   bits 16..22  7-bit address context
 *   bit      23  transfer direction, 0=write, 1=read
 *   bits 24..27  frame/segment counter modulo 16
 *   bits 28..31  reserved
 *
 * Output packing for START/RESTART/STOP:
 *   bits  0..7   data byte count in completed segment for STOP
 *   bits  8..15  total byte count including address for STOP
 *   bits 24..27  frame/segment counter modulo 16
 *
 * Output packing for errors:
 *   bits  0..7   bit count at error
 *   bits  8..15  byte index at error
 *   bits 16..23  error code
 *   bits 24..27  frame/segment counter modulo 16
 */

var CFG_I2C_SCL_BIT = 0;
var CFG_I2C_SDA_BIT = 1;

var I2C_FLAG_START    = 1;
var I2C_FLAG_RESTART  = 2;
var I2C_FLAG_STOP     = 3;
var I2C_FLAG_ADDR_ACK = 4;
var I2C_FLAG_ADDR_NAK = 5;
var I2C_FLAG_DATA_ACK = 6;
var I2C_FLAG_DATA_NAK = 7;
var I2C_FLAG_ERROR    = 15;

var I2C_ERR_PARTIAL_BEFORE_START = 1;
var I2C_ERR_PARTIAL_BEFORE_STOP  = 2;
var I2C_ERR_CAPTURE_ENDED_FRAME  = 3;

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

function emitI2c(index, flag, value) {
    index = clampIndex(index);
    rgValue[index] = value >>> 0;
    rgFlag[index] = flag;
}

function packFrame(frameSeq) {
    return ((frameSeq & 0x0F) << 24) >>> 0;
}

function packStop(frameSeq, byteIndex) {
    var totalBytes = byteIndex & 0xFF;
    var dataBytes = byteIndex > 0 ? ((byteIndex - 1) & 0xFF) : 0;
    return (packFrame(frameSeq) | (totalBytes << 8) | dataBytes) >>> 0;
}

function packByte(frameSeq, byteIndex, byteValue, addr7, rw) {
    return (packFrame(frameSeq) |
            ((rw & 1) << 23) |
            ((addr7 & 0x7F) << 16) |
            ((byteIndex & 0xFF) << 8) |
            (byteValue & 0xFF)) >>> 0;
}

function packError(frameSeq, code, byteIndex, bitCount) {
    return (packFrame(frameSeq) |
            ((code & 0xFF) << 16) |
            ((byteIndex & 0xFF) << 8) |
            (bitCount & 0xFF)) >>> 0;
}

var i;
var n = rgData.length;
for (i = 0; i < n; i++) {
    rgValue[i] = 0;
    rgFlag[i] = 0;
}

if (n > 1) {
    var prevScl = bitAt(rgData[0], CFG_I2C_SCL_BIT);
    var prevSda = bitAt(rgData[0], CFG_I2C_SDA_BIT);
    var inFrame = false;
    var bitCount = 0;
    var byteValue = 0;
    var byteIndex = 0;
    var frameSeq = 0;
    var addr7 = 0;
    var rw = 0;

    for (i = 1; i < n; i++) {
        var scl = bitAt(rgData[i], CFG_I2C_SCL_BIT);
        var sda = bitAt(rgData[i], CFG_I2C_SDA_BIT);

        var sdaFall = (prevSda === 1) && (sda === 0);
        var sdaRise = (prevSda === 0) && (sda === 1);
        var sclRise = (prevScl === 0) && (scl === 1);

        if ((scl === 1) && sdaFall) {
            if (inFrame && bitCount !== 0) {
                emitI2c(i, I2C_FLAG_ERROR,
                         packError(frameSeq, I2C_ERR_PARTIAL_BEFORE_START,
                                   byteIndex, bitCount));
            }
            if (inFrame) {
                frameSeq = (frameSeq + 1) & 0x0F;
                emitI2c(i, I2C_FLAG_RESTART, packFrame(frameSeq));
            } else {
                emitI2c(i, I2C_FLAG_START, packFrame(frameSeq));
            }
            inFrame = true;
            bitCount = 0;
            byteValue = 0;
            byteIndex = 0;
            addr7 = 0;
            rw = 0;
        } else if ((scl === 1) && sdaRise) {
            if (inFrame) {
                if (bitCount !== 0) {
                    emitI2c(i, I2C_FLAG_ERROR,
                             packError(frameSeq, I2C_ERR_PARTIAL_BEFORE_STOP,
                                       byteIndex, bitCount));
                }
                emitI2c(i, I2C_FLAG_STOP, packStop(frameSeq, byteIndex));
                frameSeq = (frameSeq + 1) & 0x0F;
            }
            inFrame = false;
            bitCount = 0;
            byteValue = 0;
            byteIndex = 0;
            addr7 = 0;
            rw = 0;
        } else if (inFrame && sclRise) {
            if (bitCount < 8) {
                byteValue = ((byteValue << 1) | sda) & 0xFF;
                bitCount++;
            } else {
                var ack = (sda === 0);
                var flag;
                if (byteIndex === 0) {
                    addr7 = (byteValue >>> 1) & 0x7F;
                    rw = byteValue & 1;
                    flag = ack ? I2C_FLAG_ADDR_ACK : I2C_FLAG_ADDR_NAK;
                } else {
                    flag = ack ? I2C_FLAG_DATA_ACK : I2C_FLAG_DATA_NAK;
                }
                emitI2c(i, flag, packByte(frameSeq, byteIndex, byteValue, addr7, rw));
                byteIndex++;
                bitCount = 0;
                byteValue = 0;
            }
        }

        prevScl = scl;
        prevSda = sda;
    }

    if (inFrame) {
        emitI2c(n - 1, I2C_FLAG_ERROR,
                 packError(frameSeq, I2C_ERR_CAPTURE_ENDED_FRAME,
                           byteIndex, bitCount));
    }
}
