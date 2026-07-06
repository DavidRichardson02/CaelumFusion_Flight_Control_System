`timescale 1ns/1ps
`default_nettype none

`include "telemetry_defs_vh.vh"

//==============================================================================
// derived_state_producer
//------------------------------------------------------------------------------
// Combines raw sensor snapshots into the derived-state publication bank.
//
// Heading publication path:
//   MMC34160PJ / CMPS2 job -> mag_* snapshot -> flight_attitude_math_sys ->
//   der_heading_mdeg and der_head_fresh.
//
// The module name and port list are preserved. Optional parameters add freshness
// control while remaining backward compatible with existing named-parameter
// instantiations.
//==============================================================================
module derived_state_producer #(
    parameter [31:0] BUILD_ID_CONST      = 32'h44535031,
    parameter [15:0] SCHEMA_WORD_CONST   = 16'h0100,
    parameter integer EXPECT_BMP_DT_US   = 20000,
    parameter integer MIN_BMP_DT_US      = 10000,
    parameter integer MAX_BMP_DT_US      = 40000,
    parameter integer RECIP_SHIFT        = 10,
    parameter integer VSPD_RECIP_Q       = ((1000000 << RECIP_SHIFT) / EXPECT_BMP_DT_US),
    parameter integer MAG_PAYLOAD_ZYX    = 1,
    parameter integer ALT_FRESH_MAX_MS   = 150,
    parameter integer VSPD_FRESH_MAX_MS  = 150,
    parameter integer ROLL_FRESH_MAX_MS  = 80,
    parameter integer HEAD_FRESH_MAX_MS  = 250
)(
    input  wire        clk,
    input  wire        rst,
    input  wire [31:0] now_us,

    input  wire [31:0] bmp_t_us,
    input  wire [15:0] bmp_seq,
    input  wire        bmp_valid,
    input  wire [7:0]  bmp_status,
    input  wire [47:0] bmp_payload,
    input  wire [15:0] bmp_age_ms,

    input  wire [31:0] acc_t_us,
    input  wire [15:0] acc_seq,
    input  wire        acc_valid,
    input  wire [7:0]  acc_status,
    input  wire [47:0] acc_payload,
    input  wire [15:0] acc_age_ms,

    input  wire [31:0] mag_t_us,
    input  wire [15:0] mag_seq,
    input  wire        mag_valid,
    input  wire [7:0]  mag_status,
    input  wire [47:0] mag_payload,
    input  wire [15:0] mag_age_ms,

    input  wire [15:0] i2c_nack_count,
    input  wire [15:0] i2c_timeout_count,
    input  wire [15:0] txn_rate_hz,
    input  wire [31:0] cdc_update_count,
    input  wire [31:0] frame_count,

    output wire [31:0] der_t_us,
    output wire [15:0] der_seq,
    output wire [7:0]  der_source_id,
    output wire [7:0]  der_status,
    output wire        der_valid,
    output wire        der_alt_fresh,
    output wire        der_vspd_fresh,
    output wire        der_roll_fresh,
    output wire        der_head_fresh,
    output wire [15:0] der_bmp_seq_ref,
    output wire [15:0] der_acc_seq_ref,
    output wire [15:0] der_mag_seq_ref,
    output wire [15:0] der_bmp_age_ms,
    output wire [15:0] der_acc_age_ms,
    output wire [15:0] der_mag_age_ms,
    output wire        der_bmp_valid_ref,
    output wire        der_acc_valid_ref,
    output wire        der_mag_valid_ref,
    output wire [31:0] der_altitude_cm,
    output wire [31:0] der_vertical_speed_cms,
    output wire [31:0] der_roll_mdeg,
    output wire [31:0] der_heading_mdeg,
    output wire [15:0] der_i2c_nack_count,
    output wire [15:0] der_i2c_timeout_count,
    output wire [15:0] der_txn_rate_hz,
    output wire [31:0] der_cdc_update_count,
    output wire [31:0] der_frame_count,
    output wire [31:0] der_build_id,
    output wire [15:0] der_schema_word
);
    localparam [15:0] ALT_FRESH_MAX_MS_L  = ALT_FRESH_MAX_MS;
    localparam [15:0] VSPD_FRESH_MAX_MS_L = VSPD_FRESH_MAX_MS;
    localparam [15:0] ROLL_FRESH_MAX_MS_L = ROLL_FRESH_MAX_MS;
    localparam [15:0] HEAD_FRESH_MAX_MS_L = HEAD_FRESH_MAX_MS;
    localparam [31:0] MIN_BMP_DT_US_L     = MIN_BMP_DT_US;
    localparam [31:0] MAX_BMP_DT_US_L     = MAX_BMP_DT_US;

    function signed [31:0] sat_s32_from_s64;
        input signed [63:0] v;
        begin
            if (v > 64'sd2147483647)
                sat_s32_from_s64 = 32'sh7FFF_FFFF;
            else if (v < -64'sd2147483648)
                sat_s32_from_s64 = 32'sh8000_0000;
            else
                sat_s32_from_s64 = v[31:0];
        end
    endfunction

    function [31:0] meters_to_cm_u32;
        input [15:0] meters;
        reg [31:0] ext;
        begin
            ext = {16'd0, meters};
            meters_to_cm_u32 = (ext << 6) + (ext << 5) + (ext << 2);
        end
    endfunction

    wire [7:0]  bmp_alt_lut_idx_w;
    wire [15:0] bmp_alt_m_u16_w;
    wire [31:0] bmp_altitude_cm_w;
    assign bmp_alt_lut_idx_w = bmp_payload[23:16];

    altitude_lut_rom_u8_to_u16 u_bmp_alt_lut (
        .idx       (bmp_alt_lut_idx_w),
        .alt_m_u16 (bmp_alt_m_u16_w)
    );

    assign bmp_altitude_cm_w = meters_to_cm_u32(bmp_alt_m_u16_w);

    wire signed [15:0] att_roll_q12_w;
    wire        [15:0] att_head_q12_u_w;
    wire signed [15:0] att_roll_sin_q15_w;
    wire signed [15:0] att_roll_cos_q15_w;
    wire signed [15:0] att_head_sin_q15_w;
    wire signed [15:0] att_head_cos_q15_w;
    wire               att_roll_valid_w;
    wire               att_head_valid_w;
    wire               att_roll_update_w;
    wire               att_head_update_w;
    wire        [15:0] att_roll_seq_ref_w;
    wire        [15:0] att_head_seq_ref_w;
    wire        [31:0] att_roll_mdeg_w;
    wire        [31:0] att_heading_mdeg_w;
    wire               att_math_busy_w;

    flight_attitude_math_sys #(
        .MAG_PAYLOAD_ZYX(MAG_PAYLOAD_ZYX)
    ) u_flight_attitude_math_sys (
        .sys_clk       (clk),
        .sys_rst       (rst),
        .acc_seq       (acc_seq),
        .acc_valid     (acc_valid),
        .acc_status    (acc_status),
        .acc_payload   (acc_payload),
        .mag_seq       (mag_seq),
        .mag_valid     (mag_valid),
        .mag_status    (mag_status),
        .mag_payload   (mag_payload),
        .roll_q12      (att_roll_q12_w),
        .head_q12_u    (att_head_q12_u_w),
        .roll_sin_q15  (att_roll_sin_q15_w),
        .roll_cos_q15  (att_roll_cos_q15_w),
        .head_sin_q15  (att_head_sin_q15_w),
        .head_cos_q15  (att_head_cos_q15_w),
        .roll_valid    (att_roll_valid_w),
        .head_valid    (att_head_valid_w),
        .roll_update   (att_roll_update_w),
        .head_update   (att_head_update_w),
        .roll_seq_ref  (att_roll_seq_ref_w),
        .head_seq_ref  (att_head_seq_ref_w),
        .roll_mdeg     (att_roll_mdeg_w),
        .heading_mdeg  (att_heading_mdeg_w),
        .busy          (att_math_busy_w)
    );

    wire _unused_att_detail_ok;
    assign _unused_att_detail_ok =
        att_roll_q12_w[0] ^ att_head_q12_u_w[0] ^
        att_roll_sin_q15_w[0] ^ att_roll_cos_q15_w[0] ^
        att_head_sin_q15_w[0] ^ att_head_cos_q15_w[0] ^
        att_math_busy_w;

    wire bmp_ok_w = bmp_valid && (bmp_status == `ST_OK);
    wire acc_ok_w = acc_valid && (acc_status == `ST_OK);
    wire mag_ok_w = mag_valid && (mag_status == `ST_OK);

    wire roll_math_current_w = att_roll_valid_w && (att_roll_seq_ref_w == acc_seq);
    wire head_math_current_w = att_head_valid_w && (att_head_seq_ref_w == mag_seq);

    wire cand_alt_fresh_w  = bmp_ok_w && (bmp_age_ms <= ALT_FRESH_MAX_MS);
    wire cand_vspd_fresh_w = bmp_ok_w && (bmp_age_ms <= VSPD_FRESH_MAX_MS);
    wire cand_roll_fresh_w = acc_ok_w && roll_math_current_w && (acc_age_ms <= ROLL_FRESH_MAX_MS);
    wire cand_head_fresh_w = mag_ok_w && head_math_current_w && (mag_age_ms <= HEAD_FRESH_MAX_MS);

    reg [15:0] bmp_seq_seen_r;
    reg [15:0] acc_seq_seen_r;
    reg [15:0] mag_seq_seen_r;
    reg        bmp_seq_seen_valid_r;
    reg        acc_seq_seen_valid_r;
    reg        mag_seq_seen_valid_r;
    reg [15:0] i2c_nack_seen_r;
    reg [15:0] i2c_timeout_seen_r;
    reg [15:0] txn_rate_seen_r;
    reg [31:0] cdc_update_seen_r;
    reg [31:0] frame_count_seen_r;
    reg [15:0] bmp_age_seen_r;
    reg [15:0] acc_age_seen_r;
    reg [15:0] mag_age_seen_r;

    wire bmp_new_w = !bmp_seq_seen_valid_r || (bmp_seq != bmp_seq_seen_r);
    wire acc_new_w = !acc_seq_seen_valid_r || (acc_seq != acc_seq_seen_r);
    wire mag_new_w = !mag_seq_seen_valid_r || (mag_seq != mag_seq_seen_r);
    wire health_changed_w =
        (i2c_nack_count    != i2c_nack_seen_r) ||
        (i2c_timeout_count != i2c_timeout_seen_r) ||
        (txn_rate_hz       != txn_rate_seen_r) ||
        (cdc_update_count  != cdc_update_seen_r) ||
        (frame_count       != frame_count_seen_r);
    wire age_changed_w =
        (bmp_age_ms != bmp_age_seen_r) ||
        (acc_age_ms != acc_age_seen_r) ||
        (mag_age_ms != mag_age_seen_r);

    wire publish_event_w =
        bmp_new_w || acc_new_w || mag_new_w ||
        att_roll_update_w || att_head_update_w ||
        health_changed_w || age_changed_w;

    reg [31:0] der_t_us_r;
    reg [15:0] der_seq_r;
    reg [7:0]  der_status_r;
    reg        der_valid_r;
    reg        der_alt_fresh_r;
    reg        der_vspd_fresh_r;
    reg        der_roll_fresh_r;
    reg        der_head_fresh_r;
    reg [15:0] der_bmp_seq_ref_r;
    reg [15:0] der_acc_seq_ref_r;
    reg [15:0] der_mag_seq_ref_r;
    reg [15:0] der_bmp_age_ms_r;
    reg [15:0] der_acc_age_ms_r;
    reg [15:0] der_mag_age_ms_r;
    reg        der_bmp_valid_ref_r;
    reg        der_acc_valid_ref_r;
    reg        der_mag_valid_ref_r;
    reg [31:0] der_altitude_cm_r;
    reg signed [31:0] der_vertical_speed_cms_r;
    reg [31:0] der_roll_mdeg_r;
    reg [31:0] der_heading_mdeg_r;

    reg [31:0] altitude_cm_prev_r;
    reg [31:0] bmp_t_us_prev_r;
    reg        prev_alt_valid_r;

    reg [7:0] cand_status_v;
    reg       cand_valid_v;
    reg signed [31:0] dalt_cm_v;
    reg signed [63:0] vspd_mul_v;

    always @(*) begin
        cand_status_v = `ST_OK;
        cand_valid_v  = 1'b0;

        if (!bmp_valid || !acc_valid || !mag_valid)
            cand_status_v = `ST_MISSING_INPUT;
        else if (!bmp_ok_w)
            cand_status_v = bmp_status;
        else if (!acc_ok_w)
            cand_status_v = acc_status;
        else if (!mag_ok_w)
            cand_status_v = mag_status;
        else if (!roll_math_current_w || !head_math_current_w)
            cand_status_v = `ST_STALE_REJECT;
        else if (!cand_alt_fresh_w || !cand_vspd_fresh_w || !cand_roll_fresh_w || !cand_head_fresh_w)
            cand_status_v = `ST_STALE_REJECT;
        else
            cand_valid_v = 1'b1;

        dalt_cm_v  = $signed(bmp_altitude_cm_w) - $signed(altitude_cm_prev_r);
        vspd_mul_v = dalt_cm_v * VSPD_RECIP_Q;
    end

    always @(posedge clk) begin
        if (rst) begin
            bmp_seq_seen_r       <= 16'd0;
            acc_seq_seen_r       <= 16'd0;
            mag_seq_seen_r       <= 16'd0;
            bmp_seq_seen_valid_r <= 1'b0;
            acc_seq_seen_valid_r <= 1'b0;
            mag_seq_seen_valid_r <= 1'b0;
            i2c_nack_seen_r      <= 16'd0;
            i2c_timeout_seen_r   <= 16'd0;
            txn_rate_seen_r      <= 16'd0;
            cdc_update_seen_r    <= 32'd0;
            frame_count_seen_r   <= 32'd0;
            bmp_age_seen_r       <= 16'hFFFF;
            acc_age_seen_r       <= 16'hFFFF;
            mag_age_seen_r       <= 16'hFFFF;

            der_t_us_r             <= 32'd0;
            der_seq_r              <= 16'd0;
            der_status_r           <= `ST_MISSING_INPUT;
            der_valid_r            <= 1'b0;
            der_alt_fresh_r        <= 1'b0;
            der_vspd_fresh_r       <= 1'b0;
            der_roll_fresh_r       <= 1'b0;
            der_head_fresh_r       <= 1'b0;
            der_bmp_seq_ref_r      <= 16'd0;
            der_acc_seq_ref_r      <= 16'd0;
            der_mag_seq_ref_r      <= 16'd0;
            der_bmp_age_ms_r       <= 16'hFFFF;
            der_acc_age_ms_r       <= 16'hFFFF;
            der_mag_age_ms_r       <= 16'hFFFF;
            der_bmp_valid_ref_r    <= 1'b0;
            der_acc_valid_ref_r    <= 1'b0;
            der_mag_valid_ref_r    <= 1'b0;
            der_altitude_cm_r      <= 32'd0;
            der_vertical_speed_cms_r <= 32'sd0;
            der_roll_mdeg_r        <= 32'd0;
            der_heading_mdeg_r     <= 32'd0;
            altitude_cm_prev_r     <= 32'd0;
            bmp_t_us_prev_r        <= 32'd0;
            prev_alt_valid_r       <= 1'b0;
        end else if (publish_event_w) begin
            if (bmp_new_w) begin
                bmp_seq_seen_r       <= bmp_seq;
                bmp_seq_seen_valid_r <= 1'b1;
            end
            if (acc_new_w) begin
                acc_seq_seen_r       <= acc_seq;
                acc_seq_seen_valid_r <= 1'b1;
            end
            if (mag_new_w) begin
                mag_seq_seen_r       <= mag_seq;
                mag_seq_seen_valid_r <= 1'b1;
            end

            i2c_nack_seen_r    <= i2c_nack_count;
            i2c_timeout_seen_r <= i2c_timeout_count;
            txn_rate_seen_r    <= txn_rate_hz;
            cdc_update_seen_r  <= cdc_update_count;
            frame_count_seen_r <= frame_count;
            bmp_age_seen_r     <= bmp_age_ms;
            acc_age_seen_r     <= acc_age_ms;
            mag_age_seen_r     <= mag_age_ms;

            der_t_us_r          <= now_us;
            der_seq_r           <= der_seq_r + 16'd1;
            der_status_r        <= cand_status_v;
            der_valid_r         <= cand_valid_v;
            der_alt_fresh_r     <= cand_alt_fresh_w;
            der_vspd_fresh_r    <= cand_vspd_fresh_w;
            der_roll_fresh_r    <= cand_roll_fresh_w;
            der_head_fresh_r    <= cand_head_fresh_w;
            der_bmp_seq_ref_r   <= bmp_seq;
            der_acc_seq_ref_r   <= acc_seq;
            der_mag_seq_ref_r   <= mag_seq;
            der_bmp_age_ms_r    <= bmp_age_ms;
            der_acc_age_ms_r    <= acc_age_ms;
            der_mag_age_ms_r    <= mag_age_ms;
            der_bmp_valid_ref_r <= bmp_valid;
            der_acc_valid_ref_r <= acc_valid;
            der_mag_valid_ref_r <= mag_valid;

            if (bmp_ok_w) begin
                der_altitude_cm_r <= bmp_altitude_cm_w;
                if (prev_alt_valid_r && (bmp_t_us > bmp_t_us_prev_r) &&
                    ((bmp_t_us - bmp_t_us_prev_r) >= MIN_BMP_DT_US_L) &&
                    ((bmp_t_us - bmp_t_us_prev_r) <= MAX_BMP_DT_US_L)) begin
                    der_vertical_speed_cms_r <= sat_s32_from_s64(vspd_mul_v >>> RECIP_SHIFT);
                end else begin
                    der_vertical_speed_cms_r <= 32'sd0;
                end
                altitude_cm_prev_r <= bmp_altitude_cm_w;
                bmp_t_us_prev_r    <= bmp_t_us;
                prev_alt_valid_r   <= 1'b1;
            end else begin
                prev_alt_valid_r       <= 1'b0;
                der_vertical_speed_cms_r <= 32'sd0;
            end

            if (att_roll_valid_w)
                der_roll_mdeg_r <= att_roll_mdeg_w;

            if (att_head_valid_w)
                der_heading_mdeg_r <= att_heading_mdeg_w;
        end
    end

    assign der_t_us               = der_t_us_r;
    assign der_seq                = der_seq_r;
    assign der_source_id          = `SRC_DERIVED_STATE;
    assign der_status             = der_status_r;
    assign der_valid              = der_valid_r;
    assign der_alt_fresh          = der_alt_fresh_r;
    assign der_vspd_fresh         = der_vspd_fresh_r;
    assign der_roll_fresh         = der_roll_fresh_r;
    assign der_head_fresh         = der_head_fresh_r;
    assign der_bmp_seq_ref        = der_bmp_seq_ref_r;
    assign der_acc_seq_ref        = der_acc_seq_ref_r;
    assign der_mag_seq_ref        = der_mag_seq_ref_r;
    assign der_bmp_age_ms         = der_bmp_age_ms_r;
    assign der_acc_age_ms         = der_acc_age_ms_r;
    assign der_mag_age_ms         = der_mag_age_ms_r;
    assign der_bmp_valid_ref      = der_bmp_valid_ref_r;
    assign der_acc_valid_ref      = der_acc_valid_ref_r;
    assign der_mag_valid_ref      = der_mag_valid_ref_r;
    assign der_altitude_cm        = der_altitude_cm_r;
    assign der_vertical_speed_cms = der_vertical_speed_cms_r;
    assign der_roll_mdeg          = der_roll_mdeg_r;
    assign der_heading_mdeg       = der_heading_mdeg_r;
    assign der_i2c_nack_count     = i2c_nack_count;
    assign der_i2c_timeout_count  = i2c_timeout_count;
    assign der_txn_rate_hz        = txn_rate_hz;
    assign der_cdc_update_count   = cdc_update_count;
    assign der_frame_count        = frame_count;
    assign der_build_id           = BUILD_ID_CONST;
    assign der_schema_word        = SCHEMA_WORD_CONST;
endmodule

`ifdef DERIVED_STATE_PRODUCER_EMBED_LEGACY_MATH
//==============================================================================
// altitude_lut_rom_u8_to_u16
//==============================================================================
module altitude_lut_rom_u8_to_u16 #(
    parameter integer STEP_M = 8
)(
    input  wire [7:0]  idx,
    output reg  [15:0] alt_m_u16
);
    always @(*) begin
        alt_m_u16 = {5'd0, idx, 3'b000};
    end
endmodule

//==============================================================================
// flight_attitude_math_sys
//------------------------------------------------------------------------------
// Roll:    atan2(AY, AZ)
// Heading: atan2(MY, MX), wrapped to [0, 360000) millidegrees.
//
// Payload contracts:
//   acc_payload = {AX, AY, AZ}
//   mag_payload = {MZ, MY, MX} when MAG_PAYLOAD_ZYX != 0
//==============================================================================
module flight_attitude_math_sys #(
    parameter integer PI_Q12          = 12868,
    parameter integer TWO_PI_Q12      = 25736,
    parameter integer MAG_PAYLOAD_ZYX = 1,
    parameter integer ACC_ROLL_Y_SIGN = 1,
    parameter integer ACC_ROLL_X_SIGN = 1,
    parameter integer MAG_HEAD_Y_SIGN = 1,
    parameter integer MAG_HEAD_X_SIGN = 1
)(
    input  wire         sys_clk,
    input  wire         sys_rst,
    input  wire [15:0]  acc_seq,
    input  wire         acc_valid,
    input  wire [7:0]   acc_status,
    input  wire [47:0]  acc_payload,
    input  wire [15:0]  mag_seq,
    input  wire         mag_valid,
    input  wire [7:0]   mag_status,
    input  wire [47:0]  mag_payload,
    output reg  signed [15:0] roll_q12,
    output reg         [15:0] head_q12_u,
    output reg  signed [15:0] roll_sin_q15,
    output reg  signed [15:0] roll_cos_q15,
    output reg  signed [15:0] head_sin_q15,
    output reg  signed [15:0] head_cos_q15,
    output reg                 roll_valid,
    output reg                 head_valid,
    output reg                 roll_update,
    output reg                 head_update,
    output reg          [15:0] roll_seq_ref,
    output reg          [15:0] head_seq_ref,
    output reg          [31:0] roll_mdeg,
    output reg          [31:0] heading_mdeg,
    output wire                busy
);
    localparam [1:0] JOB_NONE = 2'd0;
    localparam [1:0] JOB_ROLL = 2'd1;
    localparam [1:0] JOB_HEAD = 2'd2;
    localparam [31:0] FULL_TURN_MDEG = 32'd360000;

    function signed [15:0] apply_sign16;
        input signed [15:0] value;
        input integer sign_sel;
        begin
            if (sign_sel < 0)
                apply_sign16 = -value;
            else
                apply_sign16 = value;
        end
    endfunction

    function signed [31:0] q12_signed_to_mdeg;
        input signed [15:0] q12_val;
        reg signed [31:0] ext;
        begin
            ext = {{16{q12_val[15]}}, q12_val};
            q12_signed_to_mdeg = (ext <<< 4) - (ext <<< 1);
        end
    endfunction

    function [31:0] q12_unsigned_to_mdeg;
        input [15:0] q12_val;
        reg [31:0] ext;
        reg [31:0] approx;
        begin
            ext = {16'd0, q12_val};
            approx = (ext << 4) - (ext << 1);
            if (approx >= FULL_TURN_MDEG)
                q12_unsigned_to_mdeg = approx - FULL_TURN_MDEG;
            else
                q12_unsigned_to_mdeg = approx;
        end
    endfunction

    wire signed [15:0] acc_ay_raw = acc_payload[31:16];
    wire signed [15:0] acc_az_raw = acc_payload[15:0];
    wire signed [15:0] mag_x_raw  = (MAG_PAYLOAD_ZYX != 0) ? mag_payload[15:0]  : mag_payload[47:32];
    wire signed [15:0] mag_y_raw  = mag_payload[31:16];

    wire signed [15:0] roll_y_w = apply_sign16(acc_ay_raw, ACC_ROLL_Y_SIGN);
    wire signed [15:0] roll_x_w = apply_sign16(acc_az_raw, ACC_ROLL_X_SIGN);
    wire signed [15:0] head_y_w = apply_sign16(mag_y_raw, MAG_HEAD_Y_SIGN);
    wire signed [15:0] head_x_w = apply_sign16(mag_x_raw, MAG_HEAD_X_SIGN);

    wire acc_good_w = acc_valid && (acc_status == `ST_OK);
    wire mag_good_w = mag_valid && (mag_status == `ST_OK);
    wire acc_vec_nonzero_w = (roll_y_w != 16'sd0) || (roll_x_w != 16'sd0);
    wire mag_vec_nonzero_w = (head_y_w != 16'sd0) || (head_x_w != 16'sd0);

    reg        acc_seq_seen_valid;
    reg [15:0] acc_seq_seen;
    reg        mag_seq_seen_valid;
    reg [15:0] mag_seq_seen;

    reg        roll_pending;
    reg [15:0] roll_pending_seq;
    reg signed [15:0] roll_pending_y;
    reg signed [15:0] roll_pending_x;
    reg        head_pending;
    reg [15:0] head_pending_seq;
    reg signed [15:0] head_pending_y;
    reg signed [15:0] head_pending_x;

    wire acc_new_good_w = acc_good_w && acc_vec_nonzero_w &&
                          (!acc_seq_seen_valid || (acc_seq != acc_seq_seen));
    wire mag_new_good_w = mag_good_w && mag_vec_nonzero_w &&
                          (!mag_seq_seen_valid || (mag_seq != mag_seq_seen));

    reg                cordic_start;
    reg signed [15:0] cordic_y_in;
    reg signed [15:0] cordic_x_in;
    wire               cordic_busy;
    wire               cordic_done;
    wire signed [15:0] cordic_angle_q12;
    wire signed [15:0] cordic_sin_q15;
    wire signed [15:0] cordic_cos_q15;
    reg [1:0]          active_job;
    reg [15:0]         active_seq;

    cordic_atan2_q12 #(
        .PI_Q12(PI_Q12)
    ) u_cordic_atan2_q12 (
        .clk       (sys_clk),
        .rst       (sys_rst),
        .start     (cordic_start),
        .y_in      (cordic_y_in),
        .x_in      (cordic_x_in),
        .busy      (cordic_busy),
        .done      (cordic_done),
        .angle_q12 (cordic_angle_q12),
        .sin_q15   (cordic_sin_q15),
        .cos_q15   (cordic_cos_q15)
    );

    wire [15:0] head_wrapped_q12;
    angle_wrap_0_2pi #(
        .PI_Q12    (PI_Q12),
        .TWO_PI_Q12(TWO_PI_Q12)
    ) u_head_wrap_0_2pi (
        .ang_in_q12 (cordic_angle_q12),
        .ang_out_q12(head_wrapped_q12)
    );

    assign busy = cordic_busy | cordic_start | roll_pending | head_pending | (active_job != JOB_NONE);

    always @(posedge sys_clk) begin
        if (sys_rst) begin
            acc_seq_seen_valid <= 1'b0;
            acc_seq_seen       <= 16'd0;
            mag_seq_seen_valid <= 1'b0;
            mag_seq_seen       <= 16'd0;
            roll_pending       <= 1'b0;
            roll_pending_seq   <= 16'd0;
            roll_pending_y     <= 16'sd0;
            roll_pending_x     <= 16'sd0;
            head_pending       <= 1'b0;
            head_pending_seq   <= 16'd0;
            head_pending_y     <= 16'sd0;
            head_pending_x     <= 16'sd0;
            cordic_start       <= 1'b0;
            cordic_y_in        <= 16'sd0;
            cordic_x_in        <= 16'sd0;
            active_job         <= JOB_NONE;
            active_seq         <= 16'd0;
            roll_q12           <= 16'sd0;
            head_q12_u         <= 16'd0;
            roll_sin_q15       <= 16'sd0;
            roll_cos_q15       <= 16'sd0;
            head_sin_q15       <= 16'sd0;
            head_cos_q15       <= 16'sd0;
            roll_valid         <= 1'b0;
            head_valid         <= 1'b0;
            roll_update        <= 1'b0;
            head_update        <= 1'b0;
            roll_seq_ref       <= 16'd0;
            head_seq_ref       <= 16'd0;
            roll_mdeg          <= 32'd0;
            heading_mdeg       <= 32'd0;
        end else begin
            cordic_start <= 1'b0;
            roll_update  <= 1'b0;
            head_update  <= 1'b0;

            if (acc_new_good_w) begin
                acc_seq_seen_valid <= 1'b1;
                acc_seq_seen       <= acc_seq;
                roll_pending       <= 1'b1;
                roll_pending_seq   <= acc_seq;
                roll_pending_y     <= roll_y_w;
                roll_pending_x     <= roll_x_w;
            end

            if (mag_new_good_w) begin
                mag_seq_seen_valid <= 1'b1;
                mag_seq_seen       <= mag_seq;
                head_pending       <= 1'b1;
                head_pending_seq   <= mag_seq;
                head_pending_y     <= head_y_w;
                head_pending_x     <= head_x_w;
            end

            if (cordic_done) begin
                if (active_job == JOB_ROLL) begin
                    roll_q12     <= cordic_angle_q12;
                    roll_sin_q15 <= cordic_sin_q15;
                    roll_cos_q15 <= cordic_cos_q15;
                    roll_seq_ref <= active_seq;
                    roll_mdeg    <= q12_signed_to_mdeg(cordic_angle_q12);
                    roll_valid   <= 1'b1;
                    roll_update  <= 1'b1;
                end else if (active_job == JOB_HEAD) begin
                    head_q12_u     <= head_wrapped_q12;
                    head_sin_q15   <= cordic_sin_q15;
                    head_cos_q15   <= cordic_cos_q15;
                    head_seq_ref   <= active_seq;
                    heading_mdeg   <= q12_unsigned_to_mdeg(head_wrapped_q12);
                    head_valid     <= 1'b1;
                    head_update    <= 1'b1;
                end
                active_job <= JOB_NONE;
            end else if (!cordic_busy && (active_job == JOB_NONE) && !cordic_start) begin
                if (roll_pending) begin
                    cordic_y_in  <= roll_pending_y;
                    cordic_x_in  <= roll_pending_x;
                    active_seq   <= roll_pending_seq;
                    active_job   <= JOB_ROLL;
                    roll_pending <= 1'b0;
                    cordic_start <= 1'b1;
                end else if (head_pending) begin
                    cordic_y_in  <= head_pending_y;
                    cordic_x_in  <= head_pending_x;
                    active_seq   <= head_pending_seq;
                    active_job   <= JOB_HEAD;
                    head_pending <= 1'b0;
                    cordic_start <= 1'b1;
                end
            end
        end
    end
endmodule

//==============================================================================
// cordic_atan2_q12
//------------------------------------------------------------------------------
// Sequential vectoring CORDIC atan2. Input vectors are signed 16-bit values;
// output angle is signed Q4.12 radians in approximately [-pi, +pi].
//==============================================================================
module cordic_atan2_q12 #(
    parameter integer PI_Q12 = 12868,
    parameter integer ITER   = 14
)(
    input  wire               clk,
    input  wire               rst,
    input  wire               start,
    input  wire signed [15:0] y_in,
    input  wire signed [15:0] x_in,
    output reg                busy,
    output reg                done,
    output reg  signed [15:0] angle_q12,
    output reg  signed [15:0] sin_q15,
    output reg  signed [15:0] cos_q15
);
    reg [4:0] iter_r;
    reg signed [31:0] x_r;
    reg signed [31:0] y_r;
    reg signed [31:0] z_r;

    reg signed [31:0] x_next;
    reg signed [31:0] y_next;
    reg signed [31:0] z_next;
    wire signed [15:0] atan_val_w;

    function signed [15:0] atan_lut_q12;
        input [4:0] idx;
        begin
            case (idx)
                5'd0:  atan_lut_q12 = 16'sd3217;
                5'd1:  atan_lut_q12 = 16'sd1899;
                5'd2:  atan_lut_q12 = 16'sd1003;
                5'd3:  atan_lut_q12 = 16'sd509;
                5'd4:  atan_lut_q12 = 16'sd256;
                5'd5:  atan_lut_q12 = 16'sd128;
                5'd6:  atan_lut_q12 = 16'sd64;
                5'd7:  atan_lut_q12 = 16'sd32;
                5'd8:  atan_lut_q12 = 16'sd16;
                5'd9:  atan_lut_q12 = 16'sd8;
                5'd10: atan_lut_q12 = 16'sd4;
                5'd11: atan_lut_q12 = 16'sd2;
                5'd12: atan_lut_q12 = 16'sd1;
                default: atan_lut_q12 = 16'sd0;
            endcase
        end
    endfunction

    assign atan_val_w = atan_lut_q12(iter_r);

    always @(*) begin
        if (y_r >= 0) begin
            x_next = x_r + (y_r >>> iter_r);
            y_next = y_r - (x_r >>> iter_r);
            z_next = z_r + {{16{atan_val_w[15]}}, atan_val_w};
        end else begin
            x_next = x_r - (y_r >>> iter_r);
            y_next = y_r + (x_r >>> iter_r);
            z_next = z_r - {{16{atan_val_w[15]}}, atan_val_w};
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            busy      <= 1'b0;
            done      <= 1'b0;
            iter_r    <= 5'd0;
            x_r       <= 32'sd0;
            y_r       <= 32'sd0;
            z_r       <= 32'sd0;
            angle_q12 <= 16'sd0;
            sin_q15   <= 16'sd0;
            cos_q15   <= 16'sd0;
        end else begin
            done <= 1'b0;

            if (start && !busy) begin
                busy   <= 1'b1;
                iter_r <= 5'd0;
                if (x_in < 0) begin
                    x_r <= -($signed({{16{x_in[15]}}, x_in}) <<< 8);
                    y_r <= -($signed({{16{y_in[15]}}, y_in}) <<< 8);
                    if (y_in >= 0)
                        z_r <= PI_Q12;
                    else
                        z_r <= -PI_Q12;
                end else begin
                    x_r <= $signed({{16{x_in[15]}}, x_in}) <<< 8;
                    y_r <= $signed({{16{y_in[15]}}, y_in}) <<< 8;
                    z_r <= 32'sd0;
                end
            end else if (busy) begin
                x_r <= x_next;
                y_r <= y_next;
                z_r <= z_next;

                if (iter_r == (ITER - 1)) begin
                    busy      <= 1'b0;
                    done      <= 1'b1;
                    angle_q12 <= z_next[15:0];
                    sin_q15   <= 16'sd0;
                    cos_q15   <= 16'sd0;
                end else begin
                    iter_r <= iter_r + 5'd1;
                end
            end
        end
    end
endmodule

//==============================================================================
// angle_wrap_0_2pi
//==============================================================================
module angle_wrap_0_2pi #(
    parameter integer PI_Q12     = 12868,
    parameter integer TWO_PI_Q12 = 25736
)(
    input  wire signed [15:0] ang_in_q12,
    output wire        [15:0] ang_out_q12
);
    wire signed [31:0] ext_in;
    wire signed [31:0] wrapped;

    assign ext_in  = {{16{ang_in_q12[15]}}, ang_in_q12};
    assign wrapped = (ang_in_q12 < 0) ? (ext_in + TWO_PI_Q12) : ext_in;
    assign ang_out_q12 = wrapped[15:0];
endmodule

`endif
`default_nettype wire
