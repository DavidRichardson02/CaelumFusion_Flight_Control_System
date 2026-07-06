/*
 * CaelumFusion I2C passive receiver compatibility guard for WaveForms
 * Protocol I2C Custom mode.
 *
 * Paste target:
 *   Protocol -> I2C -> Custom
 *
 * Important runtime finding:
 *   Some WaveForms builds show documentation/comments for Receiver()/Receive()
 *   in this editor, but do not actually expose those functions at runtime.
 *   When that happens, a passive receiver cannot be implemented in this
 *   Protocol Custom tab and the earlier script fails with:
 *
 *     ReferenceError: Can't find variable: Receiver
 *
 * Correct passive alternatives for the live FPGA-owned bus:
 *   1. Use Protocol -> I2C -> Spy/Slave built-in receiver.
 *   2. Use Logic custom decoders.
 *   3. Use script_protocol_i2c_receiver_caelumfusion.js in the separate
 *      WaveForms Script tool, if that tool exposes Protocol.I2C.*.
 *
 * Safety:
 *   This file intentionally does not call Clear(), Read(), Write(),
 *   SlaveConfig(), or SlaveStart(). Those are active bus operations and should
 *   not be used on the live Basys-3 FPGA-owned I2C bus.
 */

print("CaelumFusion I2C Protocol Custom compatibility check");
print("Paste target: Protocol -> I2C -> Custom");
print("SCL should be DIO0 and SDA should be DIO1.");

var hasReceiver = (typeof Receiver === "function");
var hasReceive = (typeof Receive === "function");
var hasClear = (typeof Clear === "function");
var hasRead = (typeof Read === "function");
var hasWrite = (typeof Write === "function");

print("Receiver available:", hasReceiver);
print("Receive available:", hasReceive);
print("Clear available:", hasClear, "(active-drive helper; do not use on live FPGA bus)");
print("Read available:", hasRead, "(active-drive helper; do not use on live FPGA bus)");
print("Write available:", hasWrite, "(active-drive helper; do not use on live FPGA bus)");

if (!hasReceiver || !hasReceive) {
    throw "This WaveForms Protocol Custom runtime cannot passively receive I2C. Use Spy/Slave, Logic Custom decoders, or the separate Script tool instead.";
}

print("Receiver()/Receive() are available in this build. You may run a passive Protocol Custom receiver script here, but verify it calls only Receiver()/Receive().");
