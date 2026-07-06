`timescale 1ns/1ps
`default_nettype none

`include "telemetry_defs_vh.vh"

//==============================================================================
// sensor_extension_hub
//------------------------------------------------------------------------------
// First-stage extension/evidence contract for the next sensor families:
//
//   - redundant magnetometer and hard/soft-iron evidence
//   - rangefinder/lidar/ultrasonic near-ground height
//   - pitot / differential-pressure airspeed
//   - environmental humidity/temperature
//   - sun/light/horizon cue
//   - optical-flow / camera motion evidence
//   - raw black-box logging stream
//   - deterministic self-test / deliberate fault-injection provenance
//
// The physical leaf drivers are intentionally not embedded here. Future driver
// blocks publish the same snapshot shape used by the existing banks, and this
// hub reduces those snapshots into a compact evidence summary that can cross CDC
// and be logged without changing the visualizer or black-box frame formats.
// Diagnostic self-test values are explicitly marked synthetic and must not be
// treated as flight sensor measurements.
//==============================================================================
module sensor_extension_hub #(
    parameter integer PAYLOAD_W              = 48,
    parameter integer ENABLE_BLACKBOX_LOG    = 0,
    parameter [15:0]  MAG_FRESH_MAX_MS       = 16'd300,
    parameter [15:0]  GENERIC_FRESH_MAX_MS   = 16'd500,
    parameter [15:0]  MAG_NORM_MIN           = 16'd64,
    parameter [15:0]  MAG_NORM_MAX           = 16'd60000,
    parameter [15:0]  MAG_DELTA_L1_MAX       = 16'd2048,
    parameter [15:0]  MAG_NORM_DELTA_MAX     = 16'd2048
)(
    input  wire                 clk,
    input  wire                 rst,

    input  wire [31:0]          bmp_t_us,
    input  wire [15:0]          bmp_seq,
    input  wire                 bmp_valid,
    input  wire [7:0]           bmp_status,
    input  wire [PAYLOAD_W-1:0] bmp_payload,
    input  wire [15:0]          bmp_age_ms,

    input  wire [31:0]          acc_t_us,
    input  wire [15:0]          acc_seq,
    input  wire                 acc_valid,
    input  wire [7:0]           acc_status,
    input  wire [PAYLOAD_W-1:0] acc_payload,
    input  wire [15:0]          acc_age_ms,

    input  wire [31:0]          mag_t_us,
    input  wire [15:0]          mag_seq,
    input  wire                 mag_valid,
    input  wire [7:0]           mag_status,
    input  wire [PAYLOAD_W-1:0] mag_payload,
    input  wire [15:0]          mag_age_ms,

    input  wire [31:0]          pwr_t_us,
    input  wire [15:0]          pwr_seq,
    input  wire                 pwr_valid,
    input  wire [7:0]           pwr_status,
    input  wire [PAYLOAD_W-1:0] pwr_payload,
    input  wire [15:0]          pwr_age_ms,

    input  wire [31:0]          mag1_t_us,
    input  wire [15:0]          mag1_seq,
    input  wire                 mag1_valid,
    input  wire [7:0]           mag1_status,
    input  wire [PAYLOAD_W-1:0] mag1_payload,
    input  wire [15:0]          mag1_age_ms,
    input  wire [7:0]           mag1_cal_state,
    input  wire [7:0]           mag1_source_flags,
    input  wire [15:0]          mag1_bridge_checksum,

    input  wire [31:0]          rng_t_us,
    input  wire [15:0]          rng_seq,
    input  wire                 rng_valid,
    input  wire [7:0]           rng_status,
    input  wire [PAYLOAD_W-1:0] rng_payload,
    input  wire [15:0]          rng_age_ms,

    input  wire [31:0]          air_t_us,
    input  wire [15:0]          air_seq,
    input  wire                 air_valid,
    input  wire [7:0]           air_status,
    input  wire [PAYLOAD_W-1:0] air_payload,
    input  wire [15:0]          air_age_ms,

    input  wire [31:0]          env_t_us,
    input  wire [15:0]          env_seq,
    input  wire                 env_valid,
    input  wire [7:0]           env_status,
    input  wire [PAYLOAD_W-1:0] env_payload,
    input  wire [15:0]          env_age_ms,

    input  wire [31:0]          sun_t_us,
    input  wire [15:0]          sun_seq,
    input  wire                 sun_valid,
    input  wire [7:0]           sun_status,
    input  wire [PAYLOAD_W-1:0] sun_payload,
    input  wire [15:0]          sun_age_ms,

    input  wire [31:0]          flow_t_us,
    input  wire [15:0]          flow_seq,
    input  wire                 flow_valid,
    input  wire [7:0]           flow_status,
    input  wire [PAYLOAD_W-1:0] flow_payload,
    input  wire [15:0]          flow_age_ms,

    input  wire                 diag_selftest_enable,
    input  wire                 diag_fault_inject_enable,
    input  wire [3:0]           diag_fault_mode,

    input  wire                 log_runtime_enable,
    input  wire                 log_emit_req,
    input  wire                 log_stream_ready,
    output wire                 log_stream_valid,
    output wire [31:0]          log_stream_word,
    output wire                 log_stream_last,

    output reg                  ext_valid,
    output reg  [7:0]           ext_status,
    output reg  [15:0]          ext_present_flags,
    output reg  [15:0]          ext_fault_flags,
    output reg  [15:0]          ext_mag_delta_l1,
    output reg  [15:0]          ext_mag_norm_primary,
    output reg  [15:0]          ext_mag_norm_secondary,
    output reg                  ext_mag_sequence_aligned,
    output reg                  ext_mag_disagreement,
    output reg  [3:0]           ext_mag_sector_delta,
    output reg  [15:0]          ext_mag_norm_delta_l1,
    output reg  [15:0]          ext_mag_iron_residual,
    output reg  [7:0]           ext_mag_cal_state,
    output reg  [7:0]           ext_mag_source_flags,
    output reg  [15:0]          ext_mag_bridge_checksum,
    output reg  [15:0]          ext_rng_height_cm,
    output reg  [15:0]          ext_air_dp_pa,
    output reg  [15:0]          ext_air_speed_cms,
    output reg  [15:0]          ext_env_temp_cdeg,
    output reg  [15:0]          ext_env_rh_centi,
    output reg  [15:0]          ext_sun_luma,
    output reg  [15:0]          ext_flow_dx,
    output reg  [15:0]          ext_flow_dy,
    output wire [15:0]          ext_log_seq,
    output wire [15:0]          ext_log_drop_count,
    output reg  [15:0]          ext_max_age_ms
);
    wire _unused_future_times_ok;
    assign _unused_future_times_ok =
        mag1_t_us[0] ^
        rng_t_us[0] ^ rng_seq[0] ^
        air_t_us[0] ^ air_seq[0] ^
        env_t_us[0] ^ env_seq[0] ^
        sun_t_us[0] ^ sun_seq[0] ^
        flow_t_us[0] ^ flow_seq[0];

    localparam [15:0] DIAG_SELFTEST_EXT_PRESENT =
        (16'd1 << `EXT_PRESENT_RANGE_BIT) |
        (16'd1 << `EXT_PRESENT_AIR_BIT) |
        (16'd1 << `EXT_PRESENT_ENV_BIT) |
        (16'd1 << `EXT_PRESENT_SUN_BIT) |
        (16'd1 << `EXT_PRESENT_FLOW_BIT) |
        (16'd1 << `EXT_PRESENT_DIAG_BIT);
    localparam [7:0]  DIAG_SELFTEST_CAL_STATE = 8'hC1;
    localparam [7:0]  DIAG_SYNTHETIC_SOURCE   = (8'd1 << `EXT_SRC_SYNTHETIC_BIT);
    localparam [15:0] DIAG_SELFTEST_CHECKSUM  = 16'hCA1B;
    localparam [15:0] DIAG_SELFTEST_MAX_AGE_MS = 16'd44;

    function [15:0] abs16;
        input signed [15:0] v;
        begin
            if (v == -16'sd32768)
                abs16 = 16'h8000;
            else if (v < 16'sd0)
                abs16 = (~v) + 16'd1;
            else
                abs16 = v[15:0];
        end
    endfunction

    function [15:0] absdiff16;
        input signed [15:0] a;
        input signed [15:0] b;
        reg signed [16:0] d;
        begin
            d = {a[15], a} - {b[15], b};
            if (d < 17'sd0)
                d = -d;
            if (d[16])
                absdiff16 = 16'hFFFF;
            else
                absdiff16 = d[15:0];
        end
    endfunction

    function [15:0] sat_add16;
        input [15:0] a;
        input [15:0] b;
        reg [16:0] s;
        begin
            s = {1'b0, a} + {1'b0, b};
            if (s[16])
                sat_add16 = 16'hFFFF;
            else
                sat_add16 = s[15:0];
        end
    endfunction

    function [15:0] sat_add3_16;
        input [15:0] a;
        input [15:0] b;
        input [15:0] c;
        begin
            sat_add3_16 = sat_add16(sat_add16(a, b), c);
        end
    endfunction

    function [15:0] max16;
        input [15:0] a;
        input [15:0] b;
        begin
            max16 = (a > b) ? a : b;
        end
    endfunction

    function [15:0] age_if_present;
        input present_i;
        input [15:0] age_i;
        begin
            age_if_present = present_i ? age_i : 16'd0;
        end
    endfunction

    function [2:0] raw_sector8;
        input signed [15:0] x;
        input signed [15:0] y;
        reg [15:0] ax;
        reg [15:0] ay;
        begin
            ax = abs16(x);
            ay = abs16(y);
            if ({1'b0, ax} >= ({1'b0, ay} << 1))
                raw_sector8 = (x < 0) ? 3'd4 : 3'd0;
            else if ({1'b0, ay} >= ({1'b0, ax} << 1))
                raw_sector8 = (y < 0) ? 3'd6 : 3'd2;
            else if ((x >= 0) && (y >= 0))
                raw_sector8 = 3'd1;
            else if ((x < 0) && (y >= 0))
                raw_sector8 = 3'd3;
            else if ((x < 0) && (y < 0))
                raw_sector8 = 3'd5;
            else
                raw_sector8 = 3'd7;
        end
    endfunction

    function [3:0] sector_diff8;
        input [2:0] a;
        input [2:0] b;
        reg [3:0] d;
        begin
            if (a >= b)
                d = {1'b0, a} - {1'b0, b};
            else
                d = {1'b0, b} - {1'b0, a};
            if (d > 4'd4)
                sector_diff8 = 4'd8 - d;
            else
                sector_diff8 = d;
        end
    endfunction

    wire signed [15:0] mag0_x_w = mag_payload[15:0];
    wire signed [15:0] mag0_y_w = mag_payload[31:16];
    wire signed [15:0] mag0_z_w = mag_payload[47:32];

    wire signed [15:0] mag1_x_w = mag1_payload[15:0];
    wire signed [15:0] mag1_y_w = mag1_payload[31:16];
    wire signed [15:0] mag1_z_w = mag1_payload[47:32];

    wire [15:0] mag0_norm_w = sat_add3_16(abs16(mag0_x_w), abs16(mag0_y_w), abs16(mag0_z_w));
    wire [15:0] mag1_norm_w = sat_add3_16(abs16(mag1_x_w), abs16(mag1_y_w), abs16(mag1_z_w));
    wire [15:0] mag_delta_w = sat_add3_16(absdiff16(mag0_x_w, mag1_x_w),
                                           absdiff16(mag0_y_w, mag1_y_w),
                                           absdiff16(mag0_z_w, mag1_z_w));
    wire [15:0] mag_norm_delta_w = (mag0_norm_w > mag1_norm_w) ?
                                   (mag0_norm_w - mag1_norm_w) :
                                   (mag1_norm_w - mag0_norm_w);
    wire [2:0]  mag0_sector_w = raw_sector8(mag0_x_w, mag0_y_w);
    wire [2:0]  mag1_sector_w = raw_sector8(mag1_x_w, mag1_y_w);
    wire [3:0]  mag_sector_delta_w = sector_diff8(mag0_sector_w, mag1_sector_w);

    wire mag0_ok_w = mag_valid && (mag_status == `ST_OK) &&
                     (mag_age_ms <= MAG_FRESH_MAX_MS);
    wire mag1_ok_w = mag1_valid && (mag1_status == `ST_OK) &&
                     (mag1_age_ms <= MAG_FRESH_MAX_MS);
    wire rng_ok_w  = rng_valid && (rng_status == `ST_OK) &&
                     (rng_age_ms <= GENERIC_FRESH_MAX_MS);
    wire air_ok_w  = air_valid && (air_status == `ST_OK) &&
                     (air_age_ms <= GENERIC_FRESH_MAX_MS);
    wire env_ok_w  = env_valid && (env_status == `ST_OK) &&
                     (env_age_ms <= GENERIC_FRESH_MAX_MS);
    wire sun_ok_w  = sun_valid && (sun_status == `ST_OK) &&
                     (sun_age_ms <= GENERIC_FRESH_MAX_MS);
    wire flow_ok_w = flow_valid && (flow_status == `ST_OK) &&
                     (flow_age_ms <= GENERIC_FRESH_MAX_MS);

    wire mag_pair_good_w = mag0_ok_w && mag1_ok_w;
    wire mag_pair_present_w =
        (mag_valid || (mag_status != `ST_NOT_INITIALIZED)) &&
        (mag1_valid || (mag1_status != `ST_NOT_INITIALIZED));
    wire mag_sequence_aligned_w = mag_pair_good_w && (mag_seq == mag1_seq);

    wire [15:0] present_flags_w =
        ((mag_valid || mag_status != `ST_NOT_INITIALIZED) ? (16'd1 << `EXT_PRESENT_MAG0_BIT) : 16'd0) |
        ((mag1_valid || mag1_status != `ST_NOT_INITIALIZED) ? (16'd1 << `EXT_PRESENT_MAG1_BIT) : 16'd0) |
        ((rng_valid || rng_status != `ST_NOT_INITIALIZED) ? (16'd1 << `EXT_PRESENT_RANGE_BIT) : 16'd0) |
        ((air_valid || air_status != `ST_NOT_INITIALIZED) ? (16'd1 << `EXT_PRESENT_AIR_BIT) : 16'd0) |
        ((env_valid || env_status != `ST_NOT_INITIALIZED) ? (16'd1 << `EXT_PRESENT_ENV_BIT) : 16'd0) |
        ((sun_valid || sun_status != `ST_NOT_INITIALIZED) ? (16'd1 << `EXT_PRESENT_SUN_BIT) : 16'd0) |
        ((flow_valid || flow_status != `ST_NOT_INITIALIZED) ? (16'd1 << `EXT_PRESENT_FLOW_BIT) : 16'd0);

    wire diag_active_w = diag_selftest_enable || diag_fault_inject_enable;
    wire [15:0] diag_present_flags_w =
        diag_selftest_enable ? DIAG_SELFTEST_EXT_PRESENT :
        (diag_active_w ? (16'd1 << `EXT_PRESENT_DIAG_BIT) : 16'd0);
    wire [15:0] present_flags_with_diag_w =
        present_flags_w | diag_present_flags_w;

    wire [15:0] stale_flags_w =
        ((rng_valid && (rng_age_ms > GENERIC_FRESH_MAX_MS)) ? (16'd1 << `EXT_FLG_RANGE_STALE_BIT) : 16'd0) |
        ((air_valid && (air_age_ms > GENERIC_FRESH_MAX_MS)) ? (16'd1 << `EXT_FLG_AIR_STALE_BIT) : 16'd0) |
        ((env_valid && (env_age_ms > GENERIC_FRESH_MAX_MS)) ? (16'd1 << `EXT_FLG_ENV_STALE_BIT) : 16'd0) |
        ((sun_valid && (sun_age_ms > GENERIC_FRESH_MAX_MS)) ? (16'd1 << `EXT_FLG_SUN_STALE_BIT) : 16'd0) |
        ((flow_valid && (flow_age_ms > GENERIC_FRESH_MAX_MS)) ? (16'd1 << `EXT_FLG_FLOW_STALE_BIT) : 16'd0);

    wire raw_status_err_w =
        ((mag_valid && (mag_status != `ST_OK)) ||
         (mag1_valid && (mag1_status != `ST_OK)) ||
         (rng_valid && (rng_status != `ST_OK)) ||
         (air_valid && (air_status != `ST_OK)) ||
         (env_valid && (env_status != `ST_OK)) ||
         (sun_valid && (sun_status != `ST_OK)) ||
         (flow_valid && (flow_status != `ST_OK)));

    wire [15:0] mag_fault_flags_w =
        ((mag0_ok_w && !mag1_ok_w) ? (16'd1 << `EXT_FLG_MAG_PAIR_MISSING_BIT) : 16'd0) |
        ((mag_pair_good_w && (mag_delta_w > MAG_DELTA_L1_MAX)) ? (16'd1 << `EXT_FLG_MAG_DISAGREE_BIT) : 16'd0) |
        ((mag0_ok_w && ((mag0_norm_w < MAG_NORM_MIN) || (mag0_norm_w > MAG_NORM_MAX))) ?
            (16'd1 << `EXT_FLG_MAG0_NORM_OOR_BIT) : 16'd0) |
        ((mag1_ok_w && ((mag1_norm_w < MAG_NORM_MIN) || (mag1_norm_w > MAG_NORM_MAX))) ?
            (16'd1 << `EXT_FLG_MAG1_NORM_OOR_BIT) : 16'd0) |
        ((mag_pair_good_w && (mag_norm_delta_w > MAG_NORM_DELTA_MAX)) ?
            (16'd1 << `EXT_FLG_MAG_NORM_MISMATCH_BIT) : 16'd0);

    wire blackbox_log_enable_w = (ENABLE_BLACKBOX_LOG != 0) && log_runtime_enable;

    wire [15:0] diag_fault_flags_w =
        diag_fault_inject_enable ?
        ((16'd1 << `EXT_FLG_DIAG_FAULT_INJECT_BIT) |
         (((diag_fault_mode == 4'd0) || diag_fault_mode[0]) ?
            (16'd1 << `EXT_FLG_RAW_STATUS_ERR_BIT) : 16'd0) |
         (diag_fault_mode[1] ?
            ((16'd1 << `EXT_FLG_RANGE_STALE_BIT) |
             (16'd1 << `EXT_FLG_AIR_STALE_BIT)) : 16'd0) |
         (diag_fault_mode[2] ? (16'd1 << `EXT_FLG_MAG_DISAGREE_BIT) : 16'd0) |
         (diag_fault_mode[3] ? (16'd1 << `EXT_FLG_MAG_PAIR_MISSING_BIT) : 16'd0)) :
        16'd0;

    wire [15:0] live_fault_flags_w =
        mag_fault_flags_w |
        stale_flags_w |
        (raw_status_err_w ? (16'd1 << `EXT_FLG_RAW_STATUS_ERR_BIT) : 16'd0) |
        (ext_log_drop_count != 16'd0 ? (16'd1 << `EXT_FLG_BLACKBOX_DROP_BIT) : 16'd0);

    wire [15:0] fault_flags_w =
        diag_selftest_enable ? diag_fault_flags_w :
        (live_fault_flags_w | diag_fault_flags_w);

    wire any_extension_valid_w =
        mag_pair_good_w || rng_ok_w || air_ok_w || env_ok_w || sun_ok_w ||
        flow_ok_w || diag_active_w || (blackbox_log_enable_w && (ext_log_seq != 16'd0));

    wire [7:0] live_status_w =
        (present_flags_with_diag_w == 16'd0) ? `ST_NOT_INITIALIZED :
        ((fault_flags_w & ((16'd1 << `EXT_FLG_MAG_DISAGREE_BIT) |
                           (16'd1 << `EXT_FLG_MAG0_NORM_OOR_BIT) |
                           (16'd1 << `EXT_FLG_MAG1_NORM_OOR_BIT) |
                           (16'd1 << `EXT_FLG_MAG_NORM_MISMATCH_BIT))) != 16'd0) ? `ST_PLAUSIBILITY_REJECT :
        ((fault_flags_w & ((16'd1 << `EXT_FLG_RANGE_STALE_BIT) |
                           (16'd1 << `EXT_FLG_AIR_STALE_BIT) |
                           (16'd1 << `EXT_FLG_ENV_STALE_BIT) |
                           (16'd1 << `EXT_FLG_SUN_STALE_BIT) |
                           (16'd1 << `EXT_FLG_FLOW_STALE_BIT))) != 16'd0) ? `ST_STALE_REJECT :
        ((fault_flags_w & ((16'd1 << `EXT_FLG_RAW_STATUS_ERR_BIT) |
                           (16'd1 << `EXT_FLG_DIAG_FAULT_INJECT_BIT))) != 16'd0) ? `ST_CONFIG_ERROR :
         ((fault_flags_w & (16'd1 << `EXT_FLG_MAG_PAIR_MISSING_BIT)) != 16'd0) ? `ST_MISSING_INPUT :
         `ST_OK;

    wire [7:0] diag_status_w =
        !diag_fault_inject_enable ? `ST_OK :
        diag_fault_mode[2]        ? `ST_PLAUSIBILITY_REJECT :
        diag_fault_mode[1]        ? `ST_STALE_REJECT :
                                    `ST_CONFIG_ERROR;

    wire [7:0] status_w =
        diag_selftest_enable ? diag_status_w : live_status_w;

    wire [15:0] max_sensor_age_w =
        (present_flags_w == 16'd0) ? 16'hFFFF :
        max16(max16(max16(max16(age_if_present(mag_valid, mag_age_ms),
                                age_if_present(mag1_valid, mag1_age_ms)),
                          max16(age_if_present(rng_valid, rng_age_ms),
                                age_if_present(air_valid, air_age_ms))),
                    max16(max16(age_if_present(env_valid, env_age_ms),
                                age_if_present(sun_valid, sun_age_ms)),
                          age_if_present(flow_valid, flow_age_ms))),
              age_if_present(pwr_valid, pwr_age_ms));
    wire [15:0] max_age_w =
        diag_selftest_enable ? DIAG_SELFTEST_MAX_AGE_MS :
        (diag_fault_inject_enable && (present_flags_w == 16'd0)) ? 16'd0 :
        max_sensor_age_w;

    wire [15:0] log_seq_w;
    wire [15:0] log_drop_count_w;

    assign ext_log_seq        = log_seq_w;
    assign ext_log_drop_count = log_drop_count_w;

    blackbox_frame_packer #(
        .PAYLOAD_W(PAYLOAD_W)
    ) u_blackbox_frame_packer (
        .clk                    (clk),
        .rst                    (rst),
        .enable                 (blackbox_log_enable_w),
        .emit_req               (log_emit_req),

        .bmp_t_us               (bmp_t_us),
        .bmp_seq                (bmp_seq),
        .bmp_valid              (bmp_valid),
        .bmp_status             (bmp_status),
        .bmp_payload            (bmp_payload),
        .bmp_age_ms             (bmp_age_ms),

        .acc_t_us               (acc_t_us),
        .acc_seq                (acc_seq),
        .acc_valid              (acc_valid),
        .acc_status             (acc_status),
        .acc_payload            (acc_payload),
        .acc_age_ms             (acc_age_ms),

        .mag_t_us               (mag_t_us),
        .mag_seq                (mag_seq),
        .mag_valid              (mag_valid),
        .mag_status             (mag_status),
        .mag_payload            (mag_payload),
        .mag_age_ms             (mag_age_ms),

        .pwr_t_us               (pwr_t_us),
        .pwr_seq                (pwr_seq),
        .pwr_valid              (pwr_valid),
        .pwr_status             (pwr_status),
        .pwr_payload            (pwr_payload),
        .pwr_age_ms             (pwr_age_ms),

        .ext_valid              (ext_valid),
        .ext_status             (ext_status),
        .ext_present_flags      (ext_present_flags),
        .ext_fault_flags        (ext_fault_flags),
        .ext_mag_delta_l1       (ext_mag_delta_l1),
        .ext_mag_norm_primary   (ext_mag_norm_primary),
        .ext_mag_norm_secondary (ext_mag_norm_secondary),
        .ext_mag_sequence_aligned(ext_mag_sequence_aligned),
        .ext_mag_disagreement   (ext_mag_disagreement),
        .ext_mag_sector_delta   (ext_mag_sector_delta),
        .ext_mag_norm_delta_l1  (ext_mag_norm_delta_l1),
        .ext_mag_iron_residual  (ext_mag_iron_residual),
        .ext_mag_cal_state      (ext_mag_cal_state),
        .ext_mag_source_flags   (ext_mag_source_flags),
        .ext_mag_bridge_checksum(ext_mag_bridge_checksum),
        .ext_rng_height_cm      (ext_rng_height_cm),
        .ext_air_dp_pa          (ext_air_dp_pa),
        .ext_air_speed_cms      (ext_air_speed_cms),
        .ext_env_temp_cdeg      (ext_env_temp_cdeg),
        .ext_env_rh_centi       (ext_env_rh_centi),
        .ext_sun_luma           (ext_sun_luma),
        .ext_flow_dx            (ext_flow_dx),
        .ext_flow_dy            (ext_flow_dy),
        .ext_max_age_ms         (ext_max_age_ms),

        .stream_valid           (log_stream_valid),
        .stream_ready           (log_stream_ready),
        .stream_word            (log_stream_word),
        .stream_last            (log_stream_last),

        .log_seq                (log_seq_w),
        .drop_count             (log_drop_count_w),
        .busy                   ()
    );

    always @(posedge clk) begin
        if (rst) begin
            ext_valid              <= 1'b0;
            ext_status             <= `ST_NOT_INITIALIZED;
            ext_present_flags      <= 16'd0;
            ext_fault_flags        <= 16'd0;
            ext_mag_delta_l1       <= 16'd0;
            ext_mag_norm_primary   <= 16'd0;
            ext_mag_norm_secondary <= 16'd0;
            ext_mag_sequence_aligned <= 1'b0;
            ext_mag_disagreement    <= 1'b0;
            ext_mag_sector_delta    <= 4'd0;
            ext_mag_norm_delta_l1   <= 16'd0;
            ext_mag_iron_residual   <= 16'd0;
            ext_mag_cal_state       <= 8'd0;
            ext_mag_source_flags    <= 8'd0;
            ext_mag_bridge_checksum <= 16'd0;
            ext_rng_height_cm      <= 16'd0;
            ext_air_dp_pa          <= 16'd0;
            ext_air_speed_cms      <= 16'd0;
            ext_env_temp_cdeg      <= 16'd0;
            ext_env_rh_centi       <= 16'd0;
            ext_sun_luma           <= 16'd0;
            ext_flow_dx            <= 16'd0;
            ext_flow_dy            <= 16'd0;
            ext_max_age_ms         <= 16'hFFFF;
        end else begin
            ext_valid              <= any_extension_valid_w;
            ext_status             <= status_w;
            ext_present_flags      <= present_flags_with_diag_w |
                                      ((blackbox_log_enable_w || (log_seq_w != 16'd0)) ?
                                       (16'd1 << `EXT_PRESENT_BLACKBOX_BIT) : 16'd0);
            ext_fault_flags        <= fault_flags_w;
            ext_mag_delta_l1       <= diag_selftest_enable ? 16'd18 : mag_delta_w;
            ext_mag_norm_primary   <= diag_selftest_enable ? 16'd960 : mag0_norm_w;
            ext_mag_norm_secondary <= diag_selftest_enable ? 16'd972 : mag1_norm_w;
            ext_mag_sequence_aligned <= diag_selftest_enable ? 1'b1 : mag_sequence_aligned_w;
            ext_mag_disagreement    <= fault_flags_w[`EXT_FLG_MAG_DISAGREE_BIT];
            ext_mag_sector_delta    <= diag_selftest_enable ? 4'd1 : mag_sector_delta_w;
            ext_mag_norm_delta_l1   <= diag_selftest_enable ? 16'd12 : mag_norm_delta_w;
            // Placeholder hard/soft-iron residual: norm mismatch evidence only.
            // A future host ellipse fit can replace this without changing MAG0.
            ext_mag_iron_residual   <= diag_selftest_enable ? 16'd12 :
                                       (mag_pair_present_w ? mag_norm_delta_w : 16'd0);
            ext_mag_cal_state       <= diag_selftest_enable ? DIAG_SELFTEST_CAL_STATE :
                                       (mag_pair_present_w ? mag1_cal_state : 8'd0);
            ext_mag_source_flags    <= diag_selftest_enable ? DIAG_SYNTHETIC_SOURCE :
                                       (mag_pair_present_w ? mag1_source_flags : 8'd0);
            ext_mag_bridge_checksum <= diag_selftest_enable ? DIAG_SELFTEST_CHECKSUM :
                                       (mag_pair_present_w ? mag1_bridge_checksum : 16'd0);
            ext_rng_height_cm      <= diag_selftest_enable ? 16'd185 : rng_payload[47:32];
            ext_air_dp_pa          <= diag_selftest_enable ? 16'd42 : air_payload[47:32];
            ext_air_speed_cms      <= diag_selftest_enable ? 16'd1250 : air_payload[31:16];
            ext_env_temp_cdeg      <= diag_selftest_enable ? 16'd2345 : env_payload[47:32];
            ext_env_rh_centi       <= diag_selftest_enable ? 16'd4520 : env_payload[31:16];
            ext_sun_luma           <= diag_selftest_enable ? 16'd8192 : sun_payload[47:32];
            ext_flow_dx            <= diag_selftest_enable ? 16'd12 : flow_payload[47:32];
            ext_flow_dy            <= diag_selftest_enable ? 16'hFFF8 : flow_payload[31:16];
            ext_max_age_ms         <= max_age_w;
        end
    end

    wire _unused_payload_ok = &{
        1'b0,
        bmp_payload[0],
        acc_payload[0],
        rng_payload[15:0],
        air_payload[15:0],
        env_payload[15:0],
        sun_payload[31:0],
        flow_payload[15:0]
    };
endmodule

`default_nettype wire
