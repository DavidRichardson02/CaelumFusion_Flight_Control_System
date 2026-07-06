`ifndef TELEMETRY_DEFS_VH
`define TELEMETRY_DEFS_VH

//==============================================================================
// telemetry_defs.vh
//------------------------------------------------------------------------------
// Frozen telemetry constants and field semantics for the avionics-style
// publication layer.
//==============================================================================

//------------------------------------------------------------------------------
// Schema / build identity
//------------------------------------------------------------------------------
`define TELEM_SCHEMA_MAJOR       8'h01
`define TELEM_SCHEMA_MINOR       8'h00
`define TELEM_SCHEMA_WORD        16'h0100

// Example build identifier: YYYYMMDD encoded in hex-like decimal form.
// Adjust per build flow if a stronger build stamping method is introduced.
`define TELEM_BUILD_ID           32'h20260321

//------------------------------------------------------------------------------
// Source identifiers
//------------------------------------------------------------------------------
`define SRC_NONE                 8'h00

`define SRC_BMP58X_RAW           8'h01
`define SRC_LIS3DH_RAW           8'h02
`define SRC_LIS2MDL_RAW          8'h03

`define SRC_ALTITUDE_DERIVED     8'h10
`define SRC_VSPD_DERIVED         8'h11
`define SRC_ROLL_DERIVED         8'h12
`define SRC_HEADING_DERIVED      8'h13
`define SRC_DERIVED_STATE        8'h14

`define SRC_I2C_BUS_HEALTH       8'h20
`define SRC_CDC_HEALTH           8'h21
`define SRC_FRAME_HEALTH         8'h22
`define SRC_SYSTEM_HEALTH        8'h23

`define SRC_MAG_REDUNDANCY_EVID  8'h30
`define SRC_RANGE_HEIGHT_RAW     8'h31
`define SRC_PITOT_AIRSPEED_RAW   8'h32
`define SRC_ENVIRONMENT_RAW      8'h33
`define SRC_SUN_HORIZON_RAW      8'h34
`define SRC_OPTICAL_FLOW_RAW     8'h35
`define SRC_BLACKBOX_LOG         8'h36
`define SRC_GNSS_BRIDGE          8'h37
`define SRC_EKF_ESTIMATE         8'h38
`define SRC_WIND_ESTIMATE        8'h39

//------------------------------------------------------------------------------
// Status codes
//------------------------------------------------------------------------------
`define ST_OK                    8'h00
`define ST_NOT_INITIALIZED       8'h01
`define ST_BUSY_NO_NEW_DATA      8'h02
`define ST_I2C_NACK              8'h03
`define ST_I2C_TIMEOUT           8'h04
`define ST_SENSOR_ID_MISMATCH    8'h05
`define ST_CONFIG_ERROR          8'h06
`define ST_DATA_NOT_READY        8'h07
`define ST_RANGE_REJECT          8'h08
`define ST_PLAUSIBILITY_REJECT   8'h09
`define ST_STALE_REJECT          8'h0A
`define ST_MISSING_INPUT         8'h0B
`define ST_NUMERIC_FAULT         8'h0C
`define ST_CDC_MISSED_UPDATE     8'h0D
`define ST_INTERNAL_OVERFLOW     8'h0E
`define ST_FATAL_RESERVED        8'h0F

//------------------------------------------------------------------------------
// Flag bit positions
//------------------------------------------------------------------------------
`define FLG_VALID_BIT            0
`define FLG_FRESH_BIT            1
`define FLG_NEW_SINCE_ACK_BIT    2
`define FLG_SATURATED_BIT        3
`define FLG_DEGRADED_BIT         4
`define FLG_DRDY_SEEN_BIT        5
`define FLG_FIFO_USED_BIT        6

//------------------------------------------------------------------------------
// Canonical display/log telemetry atom
//------------------------------------------------------------------------------
// A compact one-word contract for page and logging summaries.  The atom carries
// observability, not ownership of raw sensor data; producers remain responsible
// for the full snapshot payload and physical-unit decode.
`define TELEM_ATOM_W             64
`define TELEM_ATOM_SEQ_MSB       63
`define TELEM_ATOM_SEQ_LSB       48
`define TELEM_ATOM_AGE_MSB       47
`define TELEM_ATOM_AGE_LSB       32
`define TELEM_ATOM_STATUS_MSB    31
`define TELEM_ATOM_STATUS_LSB    24
`define TELEM_ATOM_SOURCE_MSB    23
`define TELEM_ATOM_SOURCE_LSB    16
`define TELEM_ATOM_FLAGS_MSB     15
`define TELEM_ATOM_FLAGS_LSB     8
`define TELEM_ATOM_TAG_MSB       7
`define TELEM_ATOM_TAG_LSB       0

//------------------------------------------------------------------------------
// Freshness thresholds
//------------------------------------------------------------------------------
`define BMP_FRESH_MAX_MS         16'd200
`define ACC_FRESH_MAX_MS         16'd100
`define MAG_FRESH_MAX_MS         16'd200

`define ALT_FRESH_MAX_MS         16'd200
`define VSPD_FRESH_MAX_MS        16'd200
`define ROLL_FRESH_MAX_MS        16'd100
`define HEAD_FRESH_MAX_MS        16'd200

//------------------------------------------------------------------------------
// Display-oriented formatting / ID constants
//------------------------------------------------------------------------------
`define SENSOR_TAG_BMP           8'h42  // 'B'
`define SENSOR_TAG_ACC           8'h41  // 'A'
`define SENSOR_TAG_MAG           8'h4D  // 'M'
`define SENSOR_TAG_DER           8'h44  // 'D'
`define SENSOR_TAG_PWR           8'h50  // 'P'
`define SENSOR_TAG_EXT           8'h58  // 'X'

//------------------------------------------------------------------------------
// Extension evidence flags
//------------------------------------------------------------------------------
`define EXT_PRESENT_MAG0_BIT     0
`define EXT_PRESENT_MAG1_BIT     1
`define EXT_PRESENT_RANGE_BIT    2
`define EXT_PRESENT_AIR_BIT      3
`define EXT_PRESENT_ENV_BIT      4
`define EXT_PRESENT_SUN_BIT      5
`define EXT_PRESENT_FLOW_BIT     6
`define EXT_PRESENT_BLACKBOX_BIT 7
`define EXT_PRESENT_DIAG_BIT     8

`define EXT_FLG_MAG_PAIR_MISSING_BIT 0
`define EXT_FLG_MAG_DISAGREE_BIT     1
`define EXT_FLG_MAG0_NORM_OOR_BIT    2
`define EXT_FLG_MAG1_NORM_OOR_BIT    3
`define EXT_FLG_MAG_NORM_MISMATCH_BIT 4
`define EXT_FLG_RANGE_STALE_BIT      5
`define EXT_FLG_AIR_STALE_BIT        6
`define EXT_FLG_ENV_STALE_BIT        7
`define EXT_FLG_SUN_STALE_BIT        8
`define EXT_FLG_FLOW_STALE_BIT       9
`define EXT_FLG_RAW_STATUS_ERR_BIT   10
`define EXT_FLG_BLACKBOX_DROP_BIT    11
`define EXT_FLG_DIAG_FAULT_INJECT_BIT 12

// Extension source/provenance bits. These are evidence tags, not estimator
// confidence bits; synthetic/bench sources must remain visibly non-flight.
`define EXT_SRC_REAL_BIT              0
`define EXT_SRC_TEENSY_BRIDGE_BIT     1
`define EXT_SRC_REPLAY_BIT            2
`define EXT_SRC_SYNTHETIC_BIT         3

// Packed redundant-magnetometer metadata used by black-box frames and compact
// display contracts.  The full vector/norm fields remain separate payload words.
`define EXT_MAG_META_W                32
`define EXT_MAG_META_SRC_FLAGS_MSB    7
`define EXT_MAG_META_SRC_FLAGS_LSB    0
`define EXT_MAG_META_CAL_STATE_MSB    15
`define EXT_MAG_META_CAL_STATE_LSB    8
`define EXT_MAG_META_SECTOR_DELTA_MSB 19
`define EXT_MAG_META_SECTOR_DELTA_LSB 16
`define EXT_MAG_META_DISAGREE_BIT     20
`define EXT_MAG_META_SEQ_ALIGNED_BIT  21
`define EXT_MAG_META_RSVD_MSB         31
`define EXT_MAG_META_RSVD_LSB         22

//------------------------------------------------------------------------------
// Diagnostic fault-injection selectors
//------------------------------------------------------------------------------
`define DIAG_BANK_BMP                 2'd0
`define DIAG_BANK_ACC                 2'd1
`define DIAG_BANK_MAG                 2'd2
`define DIAG_BANK_PWR                 2'd3

`define DIAG_FAULT_NONE               3'd0
`define DIAG_FAULT_STALE              3'd1
`define DIAG_FAULT_STATUS             3'd2
`define DIAG_FAULT_STUCK_SEQ          3'd3
`define DIAG_FAULT_INVALID_PAYLOAD    3'd4
`define DIAG_FAULT_OUT_OF_RANGE       3'd5

//------------------------------------------------------------------------------
// Packetization constants
//------------------------------------------------------------------------------
`define TELEM_PKT_SYNC           16'hA55A

`define PKT_RAW_BMP              8'h01
`define PKT_RAW_ACC              8'h02
`define PKT_RAW_MAG              8'h03
`define PKT_RAW_PWR              8'h04
`define PKT_DERIVED_STATE        8'h10
`define PKT_BUS_HEALTH           8'h20
`define PKT_CDC_HEALTH           8'h21
`define PKT_FRAME_HEALTH         8'h22
`define PKT_EVENT_MARKER         8'h30
`define PKT_EXTENSION_SUMMARY    8'h40
`define PKT_BLACKBOX_WORD        8'h41

`define PKT_TEENSY_HEARTBEAT     8'h50
`define PKT_TEENSY_RANGE_AGL     8'h51

`endif
