/*
 * CaelumFusion passive I2C receiver for the separate WaveForms Script tool.
 *
 * Do not paste this file into Protocol -> I2C -> Custom. That editor has a
 * different API and will report:
 *   ReferenceError: Can't find variable: Protocol
 *
 * For Protocol -> I2C -> Custom, use:
 *   protocol_custom_i2c_passive_receiver_caelumfusion.js
 *
 * Use when AD3 probes are attached passively:
 *   DIO0 = JA3 / scl
 *   DIO1 = JA4 / sda
 *
 * Before running:
 *   1. Open Protocol.
 *   2. Select I2C.
 *   3. Set SCL=DIO0, SDA=DIO1, format=Hexadecimal.
 *   4. Use receiver/spy mode. Do not drive SCL/SDA on the live FPGA bus.
 *   5. Open the separate WaveForms Script tool and paste this file there.
 *
 * The script starts Protocol.I2C.Receiver(), decodes returned 9-bit I2C words,
 * prints semantic frames, and writes a byte-auditable CSV log beside the
 * WaveForms workspace.
 */

var CAPTURE_SECONDS = 20.0;
var POLL_SECONDS = 0.10;
var CSV_BASENAME = "caelumfusion_i2c_protocol_capture";

function hex2(v) {
    var s = (v & 0xFF).toString(16).toUpperCase();
    return (s.length < 2) ? ("0" + s) : s;
}

function labelAddr(addr) {
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

function csvEscape(s) {
    s = String(s);
    if (s.indexOf(",") >= 0 || s.indexOf("\"") >= 0) {
        return "\"" + s.replace(/"/g, "\"\"") + "\"";
    }
    return s;
}

function nowTag() {
    var d = new Date();
    function p2(x) { return (x < 10 ? "0" : "") + x; }
    return d.getFullYear() + p2(d.getMonth() + 1) + p2(d.getDate()) + "_" +
           p2(d.getHours()) + p2(d.getMinutes()) + p2(d.getSeconds());
}

function workspacePath(name) {
    var dir = ".";
    try {
        dir = Tool.workspaceDir();
    } catch (e) {
        dir = ".";
    }
    return dir + "/" + name;
}

function ackText(ack) {
    return ack ? "ACK" : "NACK";
}

function newFrame(kind) {
    return {
        kind: kind,
        addr: -1,
        rawAddrByte: -1,
        rw: "",
        addrAck: "",
        data: [],
        dataAck: [],
        byteEvidence: [],
        errors: []
    };
}

function appendByteEvidence(frame, role, byteValue, ack) {
    frame.byteEvidence.push(role + "=0x" + hex2(byteValue) + "/" + ackText(ack));
}

function frameText(frame, index, completed) {
    var addrText = frame.addr < 0 ? "--" : "0x" + hex2(frame.addr);
    var rawAddrText = frame.rawAddrByte < 0 ? "--" : "0x" + hex2(frame.rawAddrByte);
    return "#" + index +
           " complete=" + (completed ? "yes" : "no") +
           " " + frame.kind +
           " addr7=" + addrText +
           " addrByte=" + rawAddrText +
           " " + frame.rw +
           " " + frame.addrAck +
           " " + labelAddr(frame.addr) +
           " dataCount=" + frame.data.length +
           " bytes=[" + frame.byteEvidence.join(" ") + "]" +
           (frame.errors.length ? " errors=[" + frame.errors.join(";") + "]" : "");
}

function writeFrameCsv(file, frame, index, elapsed, completed) {
    var addrText = frame.addr < 0 ? "" : "0x" + hex2(frame.addr);
    var rawAddrText = frame.rawAddrByte < 0 ? "" : "0x" + hex2(frame.rawAddrByte);
    var row = [
        index,
        elapsed.toFixed(3),
        completed ? "yes" : "no",
        frame.kind,
        addrText,
        rawAddrText,
        frame.rw,
        frame.addrAck,
        labelAddr(frame.addr),
        frame.data.length,
        frame.byteEvidence.join(" "),
        frame.data.join(" "),
        frame.dataAck.join(" "),
        frame.errors.join(";")
    ];
    file.appendLine(row.map(csvEscape).join(","));
}

function ensureStats(table, addr) {
    var key = String(addr & 0x7F);
    if (!table[key]) {
        table[key] = {
            frames: 0,
            ack: 0,
            nack: 0,
            reads: 0,
            writes: 0,
            dataBytes: 0
        };
    }
    return table[key];
}

function recordFrameSummary(table, frame) {
    if (frame.addr < 0) {
        return;
    }
    var s = ensureStats(table, frame.addr);
    s.frames++;
    if (frame.addrAck === "ACK") {
        s.ack++;
    } else if (frame.addrAck === "NACK") {
        s.nack++;
    }
    if (frame.rw === "R") {
        s.reads++;
    } else if (frame.rw === "W") {
        s.writes++;
    }
    s.dataBytes += frame.data.length;
}

clear();
try {
    Protocol.Mode.text = "I2C";
} catch (e) {
    print("Protocol.Mode.text was not set by script; verify Protocol is already in I2C mode.");
}

var logPath = workspacePath(CSV_BASENAME + "_" + nowTag() + ".csv");
var logFile = File(logPath);
logFile.writeLine("idx,elapsed_s,complete,kind,address7,address_byte,rw,address_ack,label,data_count,byte_evidence,data,data_ack,errors");

print("CaelumFusion I2C passive receiver for separate WaveForms Script tool");
print("CSV:", logPath);
print("DIO0=SCL, DIO1=SDA. Receiver is passive; do not use master writes on a live FPGA-owned bus.");

Protocol.I2C.Receiver();

var startMs = (new Date()).getTime();
var frame = null;
var frameIndex = 0;
var completedFrames = 0;
var incompleteFrames = 0;
var byteEvents = 0;
var summaryByAddr = {};

function flushFrame(completed) {
    if (frame === null) {
        return;
    }
    var elapsed = (((new Date()).getTime() - startMs) / 1000.0);
    print(frameText(frame, frameIndex, completed));
    writeFrameCsv(logFile, frame, frameIndex, elapsed, completed);
    recordFrameSummary(summaryByAddr, frame);
    if (completed) {
        completedFrames++;
    } else {
        incompleteFrames++;
    }
    frameIndex++;
    frame = null;
}

while ((((new Date()).getTime() - startMs) / 1000.0) < CAPTURE_SECONDS) {
    wait(POLL_SECONDS);
    var words = Protocol.I2C.Receive();
    if (!words || words.length === 0) {
        continue;
    }

    for (var i = 0; i < words.length; i++) {
        var w = words[i];
        if (w === -1 || w === -2) {
            if (frame !== null) {
                frame.errors.push("missing STOP before START");
                flushFrame(false);
            }
            frame = newFrame(w === -1 ? "START" : "RESTART");
        } else if (w === -3) {
            flushFrame(true);
        } else if (w >= 0) {
            if (frame === null) {
                frame = newFrame("MIDCAP");
                frame.errors.push("capture began mid-frame");
            }
            var data8 = (w >> 1) & 0xFF;
            var ack = ((w & 1) === 0);
            byteEvents++;
            if (frame.addr < 0) {
                frame.rawAddrByte = data8;
                frame.addr = (data8 >> 1) & 0x7F;
                frame.rw = (data8 & 1) ? "R" : "W";
                frame.addrAck = ackText(ack);
                appendByteEvidence(frame, "ADDR_" + frame.rw, data8, ack);
            } else {
                frame.data.push("0x" + hex2(data8));
                frame.dataAck.push(ackText(ack));
                appendByteEvidence(frame, "D" + (frame.data.length - 1), data8, ack);
            }
        } else {
            if (frame === null) {
                frame = newFrame("ERROR");
            }
            frame.errors.push("decoder marker " + w);
        }
    }
}

if (frame !== null) {
    frame.errors.push("capture ended before STOP");
    flushFrame(false);
}

print("summary completedFrames=" + completedFrames +
      " incompleteFrames=" + incompleteFrames +
      " byteEvents=" + byteEvents);
for (var a in summaryByAddr) {
    var st = summaryByAddr[a];
    print("0x" + hex2(Number(a)) + " " + labelAddr(Number(a)) +
          " frames=" + st.frames +
          " ACK=" + st.ack +
          " NACK=" + st.nack +
          " reads=" + st.reads +
          " writes=" + st.writes +
          " dataBytes=" + st.dataBytes);
}
