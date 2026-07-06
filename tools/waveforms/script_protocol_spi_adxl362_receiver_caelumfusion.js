/*
 * CaelumFusion passive ADXL362/Pmod ACL2 SPI receiver for the separate
 * WaveForms Script tool.
 *
 * Paste target:
 *   WaveForms Script tool
 *
 * Do not paste this file into Protocol -> SPI -> Custom. This script uses the
 * separate Script-tool Protocol.SPI.* API.
 *
 * Passive probe map:
 *   DIO2 = JB1 / adxl362_cs_n
 *   DIO3 = JB2 / adxl362_mosi
 *   DIO4 = JB3 / adxl362_miso
 *   DIO5 = JB4 / adxl362_sclk
 *
 * Before running:
 *   1. Open Protocol.
 *   2. Select SPI.
 *   3. Configure Spy mode, Select=DIO2 active low, Clock=DIO5,
 *      DQ0/MOSI=DIO3, DQ1/MISO=DIO4, mode Standard, CPOL=0, CPHA=0,
 *      First bit=MSB, Data Bits=8, Format=Hexadecimal.
 *   4. Trigger the FPGA ADXL path with SW8 if that path is compiled/enabled.
 *
 * Byte evidence contract:
 *   Every received CS-framed transfer is logged as MOSI/MISO byte pairs with a
 *   byte index and semantic role. Sensor values are interpreted only after the
 *   ADXL362 PARTID register returns 0xF2.
 */

var CAPTURE_SECONDS = 20.0;
var POLL_SECONDS = 0.10;
var CSV_BASENAME = "caelumfusion_adxl362_spi_capture";

function hex2(v) {
    var s = (v & 0xFF).toString(16).toUpperCase();
    return (s.length < 2) ? ("0" + s) : s;
}

function signed16(lo, hi) {
    var v = ((hi & 0xFF) << 8) | (lo & 0xFF);
    return (v & 0x8000) ? (v - 0x10000) : v;
}

function regName(v) {
    switch (v & 0xFF) {
        case 0x02: return "PARTID";
        case 0x0E: return "XDATA_L";
        case 0x0F: return "XDATA_H";
        case 0x10: return "YDATA_L";
        case 0x11: return "YDATA_H";
        case 0x12: return "ZDATA_L";
        case 0x13: return "ZDATA_H";
        case 0x2A: return "INTMAP1";
        case 0x2B: return "INTMAP2";
        case 0x2D: return "POWER_CTL";
        default: return "REG?";
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

function byteRole(index, mosi) {
    var cmd = mosi.length > 0 ? mosi[0] : -1;
    if (index === 0) {
        if (cmd === 0x0A) {
            return "CMD_WRITE_REG";
        }
        if (cmd === 0x0B) {
            return "CMD_READ_REG";
        }
        return "CMD_UNKNOWN";
    }
    if (index === 1) {
        return "REG_" + regName(mosi[1]);
    }
    if (cmd === 0x0A) {
        return "WRITE_DATA";
    }
    if (cmd === 0x0B) {
        return "READ_DUMMY_RESPONSE";
    }
    return "BYTE";
}

function byteEvidence(mosi, miso) {
    var out = [];
    for (var i = 0; i < mosi.length; i++) {
        out.push("#" + i +
                 ":" + byteRole(i, mosi) +
                 ":MOSI=0x" + hex2(mosi[i]) +
                 "/MISO=0x" + hex2(miso[i]));
    }
    return out;
}

function allEqual(bytes, value) {
    if (bytes.length === 0) {
        return false;
    }
    for (var i = 0; i < bytes.length; i++) {
        if ((bytes[i] & 0xFF) !== value) {
            return false;
        }
    }
    return true;
}

function frameMeaning(mosi, miso) {
    if (mosi.length < 1) {
        return "empty";
    }
    if (mosi[0] === 0x0B && mosi.length >= 3 && mosi[1] === 0x02) {
        return "READ PARTID -> 0x" + hex2(miso[2]) + (miso[2] === 0xF2 ? " OK" : " unexpected");
    }
    if (mosi[0] === 0x0A && mosi.length >= 3) {
        return "WRITE " + regName(mosi[1]) + " = 0x" + hex2(mosi[2]);
    }
    if (mosi[0] === 0x0B && mosi.length >= 8 && mosi[1] === 0x0E) {
        var ax = signed16(miso[2], miso[3]);
        var ay = signed16(miso[4], miso[5]);
        var az = signed16(miso[6], miso[7]);
        return "READ XYZ raw=[" + ax + "," + ay + "," + az +
               "] g_default2g=[" + (ax / 1000.0).toFixed(3) + "," +
               (ay / 1000.0).toFixed(3) + "," + (az / 1000.0).toFixed(3) + "]";
    }
    if (mosi[0] === 0x0B && mosi.length >= 2) {
        return "READ " + regName(mosi[1]);
    }
    return "unmapped";
}

function validationFlags(mosi, miso, oddWordCount) {
    var flags = [];
    if (oddWordCount) {
        flags.push("odd_receive_word_count_missing_miso");
    }
    if (mosi.length === 0) {
        flags.push("empty_frame");
    }
    if (miso.length >= 3 && allEqual(miso, 0x00)) {
        flags.push("miso_all_00_check_power_or_miso");
    }
    if (miso.length >= 3 && allEqual(miso, 0xFF)) {
        flags.push("miso_all_FF_check_power_or_miso_pullup");
    }
    if (mosi.length >= 1 && mosi[0] !== 0x0A && mosi[0] !== 0x0B) {
        flags.push("unknown_command");
    }
    if (mosi.length >= 2 && mosi[0] === 0x0B && mosi[1] === 0x02) {
        if (mosi.length < 3) {
            flags.push("short_partid_read");
        } else if (miso[2] === 0xF2) {
            flags.push("partid_ok");
        } else {
            flags.push("partid_unexpected_0x" + hex2(miso[2]));
        }
    }
    if (mosi.length >= 2 && mosi[0] === 0x0B && mosi[1] === 0x0E) {
        if (mosi.length >= 8) {
            flags.push("xyz_frame");
        } else {
            flags.push("short_xyz_read");
        }
    }
    if (mosi.length >= 1 && mosi[0] === 0x0A && mosi.length < 3) {
        flags.push("short_register_write");
    }
    if (flags.length === 0) {
        flags.push("no_protocol_warning");
    }
    return flags;
}

clear();
try {
    Protocol.Mode.text = "SPI";
} catch (e) {
    print("Protocol.Mode.text was not set by script; verify Protocol is already in SPI mode.");
}

var logPath = workspacePath(CSV_BASENAME + "_" + nowTag() + ".csv");
var logFile = File(logPath);
logFile.writeLine("idx,elapsed_s,bytes,mosi,miso,byte_evidence,meaning,validation");

print("CaelumFusion ADXL362 SPI passive receiver for separate WaveForms Script tool");
print("CSV:", logPath);
print("DIO2=CS_N, DIO3=MOSI, DIO4=MISO, DIO5=SCLK, SPI mode 0, MSB first.");

Protocol.SPI.Receiver();

var startMs = (new Date()).getTime();
var frameIndex = 0;
var totalBytes = 0;
var partIdOk = 0;
var partIdUnexpected = 0;
var xyzFrames = 0;
var misoAll00 = 0;
var misoAllFF = 0;
var warningFrames = 0;

while ((((new Date()).getTime() - startMs) / 1000.0) < CAPTURE_SECONDS) {
    wait(POLL_SECONDS);
    var words = Protocol.SPI.Receive();
    if (!words || words.length === 0) {
        continue;
    }

    var mosi = [];
    var miso = [];
    var oddWordCount = (words.length & 1) !== 0;
    for (var i = 0; i < words.length; i += 2) {
        mosi.push(words[i] & 0xFF);
        if ((i + 1) < words.length) {
            miso.push(words[i + 1] & 0xFF);
        } else {
            miso.push(0);
        }
    }

    var mosiText = [];
    var misoText = [];
    for (var j = 0; j < mosi.length; j++) {
        mosiText.push("0x" + hex2(mosi[j]));
        misoText.push("0x" + hex2(miso[j]));
    }

    var evidence = byteEvidence(mosi, miso);
    var meaning = frameMeaning(mosi, miso);
    var flags = validationFlags(mosi, miso, oddWordCount);
    var elapsed = (((new Date()).getTime() - startMs) / 1000.0);

    totalBytes += mosi.length;
    for (var f = 0; f < flags.length; f++) {
        if (flags[f] === "partid_ok") {
            partIdOk++;
        } else if (flags[f].indexOf("partid_unexpected_") === 0) {
            partIdUnexpected++;
            warningFrames++;
        } else if (flags[f] === "xyz_frame") {
            xyzFrames++;
        } else if (flags[f] === "miso_all_00_check_power_or_miso") {
            misoAll00++;
            warningFrames++;
        } else if (flags[f] === "miso_all_FF_check_power_or_miso_pullup") {
            misoAllFF++;
            warningFrames++;
        } else if (flags[f] !== "no_protocol_warning") {
            warningFrames++;
        }
    }

    print("#" + frameIndex +
          " bytes=" + mosi.length +
          " " + meaning +
          " validation=[" + flags.join(" ") + "]" +
          " evidence=[" + evidence.join(" ") + "]");
    logFile.appendLine([
        frameIndex,
        elapsed.toFixed(3),
        mosi.length,
        csvEscape(mosiText.join(" ")),
        csvEscape(misoText.join(" ")),
        csvEscape(evidence.join(" ")),
        csvEscape(meaning),
        csvEscape(flags.join(" "))
    ].join(","));
    frameIndex++;
}

print("summary frames=" + frameIndex +
      " bytes=" + totalBytes +
      " partIdOk=" + partIdOk +
      " partIdUnexpected=" + partIdUnexpected +
      " xyzFrames=" + xyzFrames +
      " misoAll00=" + misoAll00 +
      " misoAllFF=" + misoAllFF +
      " warningFrames=" + warningFrames);
