/*
 * WaveForms Protocol I2C Custom API probe.
 *
 * Paste target:
 *   Protocol -> I2C -> Custom
 *
 * Purpose:
 *   Reports which local functions this WaveForms build exposes in the I2C
 *   Custom editor. This is a diagnostic only. It does not drive the bus.
 */

print("Protocol I2C Custom API probe");
print("Expected passive receive helpers, when supported:");
print("Receiver:", typeof Receiver);
print("Receive:", typeof Receive);
print("Active-drive helpers; do not call these on a live FPGA-owned bus:");
print("Clear:", typeof Clear);
print("Read:", typeof Read);
print("Write:", typeof Write);
print("SlaveConfig:", typeof SlaveConfig);
print("SlaveStart:", typeof SlaveStart);
print("SlaveStop:", typeof SlaveStop);
print("SlaveStatus:", typeof SlaveStatus);
print("SlaveReceive:", typeof SlaveReceive);
print("SlaveRespond:", typeof SlaveRespond);
