// DEPRECATED: CaelumFusion Teensy 4.1 -> Basys-3 fixed-packet UART producer.
//
// The Teensy 4.1 board is no longer an active hardware path for this project.
// Do not wire the failed Teensy into the Basys-3 system. The active replacement
// is firmware/tm4c123gxl_bridge_range_producer/main.c for EK-TM4C123GXL UART1.
// This file is intentionally left as a non-buildable historical reference so
// an obsolete Arduino command cannot silently reintroduce the failed hardware
// path.

#error "Deprecated Teensy producer: use firmware/tm4c123gxl_bridge_range_producer/main.c"

#include <Arduino.h>

static HardwareSerial &FPGA_UART = Serial1;

static const uint32_t FPGA_UART_BAUD = 115200UL;
static const uint16_t TELEM_PKT_SYNC = 0xA55A;

static const uint8_t ST_OK = 0x00;
static const uint8_t ST_CONFIG_ERROR = 0x06;

static const uint8_t PKT_TEENSY_HEARTBEAT = 0x50;
static const uint8_t PKT_TEENSY_RANGE_AGL = 0x51;
static const uint8_t PKT_UNSUPPORTED_TEST = 0x7E;

static const uint16_t EXT_SRC_REAL = 1u << 0;
static const uint16_t EXT_SRC_TEENSY_BRIDGE = 1u << 1;

static const uint32_t HEARTBEAT_PERIOD_US = 100000UL;
static const uint32_t RANGE_PERIOD_US = 50000UL;

static uint16_t heartbeat_seq = 0x2000;
static uint16_t range_seq = 0x0100;
static uint32_t next_heartbeat_us = 0;
static uint32_t next_range_us = 0;

static bool heartbeat_enabled = true;
static bool range_enabled = true;
static bool corrupt_next_range = false;
static bool corrupt_next_heartbeat = false;
static bool out_of_range_next = false;
static bool low_confidence_next = false;
static bool unsupported_next = false;

static uint16_t range_height_cm = 185;
static uint16_t range_confidence = 95;

static void write_u16(Stream &s, uint16_t v) {
  s.write(static_cast<uint8_t>(v >> 8));
  s.write(static_cast<uint8_t>(v));
}

static void write_u32(Stream &s, uint32_t v) {
  s.write(static_cast<uint8_t>(v >> 24));
  s.write(static_cast<uint8_t>(v >> 16));
  s.write(static_cast<uint8_t>(v >> 8));
  s.write(static_cast<uint8_t>(v));
}

static void write_u48(Stream &s, uint64_t v) {
  s.write(static_cast<uint8_t>(v >> 40));
  s.write(static_cast<uint8_t>(v >> 32));
  s.write(static_cast<uint8_t>(v >> 24));
  s.write(static_cast<uint8_t>(v >> 16));
  s.write(static_cast<uint8_t>(v >> 8));
  s.write(static_cast<uint8_t>(v));
}

static uint16_t checksum16(uint8_t type,
                           uint8_t status,
                           uint16_t seq,
                           uint32_t timestamp_us,
                           uint64_t payload,
                           uint16_t aux,
                           uint16_t source_flags) {
  return static_cast<uint16_t>(
      TELEM_PKT_SYNC ^
      (static_cast<uint16_t>(status) << 8 | type) ^
      seq ^
      static_cast<uint16_t>(timestamp_us >> 16) ^
      static_cast<uint16_t>(timestamp_us) ^
      static_cast<uint16_t>(payload >> 32) ^
      static_cast<uint16_t>(payload >> 16) ^
      static_cast<uint16_t>(payload) ^
      aux ^
      source_flags);
}

static void send_frame(uint8_t type,
                       uint8_t status,
                       uint16_t seq,
                       uint32_t timestamp_us,
                       uint64_t payload,
                       uint16_t aux,
                       uint16_t source_flags,
                       bool corrupt_checksum) {
  uint16_t sum =
      checksum16(type, status, seq, timestamp_us, payload, aux, source_flags);
  if (corrupt_checksum) {
    sum ^= 0x0001;
  }

  FPGA_UART.write(0xA5);
  FPGA_UART.write(0x5A);
  FPGA_UART.write(type);
  FPGA_UART.write(status);
  write_u16(FPGA_UART, seq);
  write_u32(FPGA_UART, timestamp_us);
  write_u48(FPGA_UART, payload);
  write_u16(FPGA_UART, aux);
  write_u16(FPGA_UART, source_flags);
  write_u16(FPGA_UART, sum);
}

static void send_heartbeat(uint32_t now_us) {
  send_frame(PKT_TEENSY_HEARTBEAT,
             ST_OK,
             heartbeat_seq++,
             now_us,
             0,
             0xCAFE,
             EXT_SRC_TEENSY_BRIDGE,
             corrupt_next_heartbeat);
  corrupt_next_heartbeat = false;
}

static void send_range(uint32_t now_us) {
  const uint16_t height =
      out_of_range_next ? static_cast<uint16_t>(12000) : range_height_cm;
  const uint16_t confidence =
      low_confidence_next ? static_cast<uint16_t>(0) : range_confidence;
  const uint16_t raw_detail =
      static_cast<uint16_t>((now_us >> 8) ^ range_seq);
  const uint64_t payload =
      (static_cast<uint64_t>(height) << 32) |
      (static_cast<uint64_t>(confidence) << 16) |
      raw_detail;

  send_frame(PKT_TEENSY_RANGE_AGL,
             ST_OK,
             range_seq++,
             now_us,
             payload,
             0x3333,
             EXT_SRC_REAL | EXT_SRC_TEENSY_BRIDGE,
             corrupt_next_range);

  corrupt_next_range = false;
  out_of_range_next = false;
  low_confidence_next = false;
}

static void send_unsupported(uint32_t now_us) {
  const uint64_t payload = 0x000100020003ULL;
  send_frame(PKT_UNSUPPORTED_TEST,
             ST_OK,
             range_seq++,
             now_us,
             payload,
             0x7E7E,
             EXT_SRC_TEENSY_BRIDGE,
             false);
}

static bool due(uint32_t now_us, uint32_t *next_us, uint32_t period_us) {
  if (static_cast<int32_t>(now_us - *next_us) >= 0) {
    *next_us += period_us;
    return true;
  }
  return false;
}

static void print_help() {
  Serial.println();
  Serial.println("CaelumFusion Teensy bridge producer");
  Serial.println("Commands:");
  Serial.println("  ?  help");
  Serial.println("  h  toggle heartbeat frames");
  Serial.println("  r  toggle range frames");
  Serial.println("  c  corrupt next range checksum");
  Serial.println("  b  corrupt next heartbeat checksum");
  Serial.println("  o  send next range out of FPGA limit");
  Serial.println("  l  send next range with low confidence");
  Serial.println("  u  send one unsupported packet type");
  Serial.println("  +  increase simulated height");
  Serial.println("  -  decrease simulated height");
}

static void handle_usb_command(int c) {
  switch (c) {
    case '?':
      print_help();
      break;
    case 'h':
      heartbeat_enabled = !heartbeat_enabled;
      Serial.printf("heartbeat_enabled=%u\r\n", heartbeat_enabled ? 1 : 0);
      break;
    case 'r':
      range_enabled = !range_enabled;
      Serial.printf("range_enabled=%u\r\n", range_enabled ? 1 : 0);
      break;
    case 'c':
      corrupt_next_range = true;
      Serial.println("next range checksum will be corrupt");
      break;
    case 'b':
      corrupt_next_heartbeat = true;
      Serial.println("next heartbeat checksum will be corrupt");
      break;
    case 'o':
      out_of_range_next = true;
      Serial.println("next range height will be out of FPGA range");
      break;
    case 'l':
      low_confidence_next = true;
      Serial.println("next range confidence will be low");
      break;
    case 'u':
      unsupported_next = true;
      Serial.println("one unsupported packet will be sent");
      break;
    case '+':
      if (range_height_cm < 9900) {
        range_height_cm += 5;
      }
      Serial.printf("range_height_cm=%u\r\n", range_height_cm);
      break;
    case '-':
      if (range_height_cm >= 5) {
        range_height_cm -= 5;
      }
      Serial.printf("range_height_cm=%u\r\n", range_height_cm);
      break;
    default:
      break;
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(115200);
  FPGA_UART.begin(FPGA_UART_BAUD);

  const uint32_t now_us = micros();
  next_heartbeat_us = now_us + 10000UL;
  next_range_us = now_us + 20000UL;

  delay(50);
  if (Serial) {
    print_help();
  }
}

void loop() {
  while (Serial.available() > 0) {
    handle_usb_command(Serial.read());
  }

  const uint32_t now_us = micros();

  if (unsupported_next) {
    unsupported_next = false;
    send_unsupported(now_us);
  }

  if (heartbeat_enabled &&
      due(now_us, &next_heartbeat_us, HEARTBEAT_PERIOD_US)) {
    send_heartbeat(now_us);
    digitalToggleFast(LED_BUILTIN);
  }

  if (range_enabled && due(now_us, &next_range_us, RANGE_PERIOD_US)) {
    send_range(now_us);
  }
}
