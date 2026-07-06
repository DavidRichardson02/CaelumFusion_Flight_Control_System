`timescale 1ns/1ps
`default_nettype none

`include "flight_viz_bundle_defs.vh"
`include "telemetry_defs_vh.vh"

//==============================================================================
// flight_visualizer_pix
//------------------------------------------------------------------------------
// ROLE
//   Unified PIX-domain telemetry renderer.
//
// PURPOSE
//   Canonical merged renderer covering:
//
//     1) schema-faithful widened packed bundle consumption
//     2) frozen telemetry text overlay via textgen + compositor
//     3) top health strip with color-coded freshness / status blocks
//     4) altitude and vertical-speed tapes
//     5) artificial horizon driven from derived roll angle
//     6) roll-plane vertical vector instrument driven from derived roll angle
//     7) compass rose / needle driven from derived heading angle
//     8) apogee authority physics overlay from published drag-servo evidence
//     9) altitude and vertical-speed strip charts from BRAM history with
//        explicit newest-sample cursors
//
// PIPELINE
//   1) Latch committed PIX bundle
//   2) Precompute geometry products from the q0 pixel stage
//   3) Build base scene in the q1-aligned pixel stage
//   4) Generate telemetry text from committed bundle
//   5) Composite telemetry text at frozen coordinates
//   6) Register text/base/sync into one render-aligned output stage
//==============================================================================
module flight_visualizer_pix #(
    parameter integer H_ACTIVE  = 640,
    parameter integer H_FP      = 16,
    parameter integer H_SYNC    = 96,
    parameter integer H_BP      = 48,
    parameter integer V_ACTIVE  = 480,
    parameter integer V_FP      = 10,
    parameter integer V_SYNC    = 2,
    parameter integer V_BP      = 33,
    parameter integer HSYNC_POL = 0,
    parameter integer VSYNC_POL = 0,

    // The secondary sensor-diagnostic page is useful during bring-up but costs
    // substantial pixel-domain LUTs. Keep it compiled out in the default
    // Basys-3 build; the primary graphical HUD remains active.
    parameter integer ENABLE_SENSOR_DIAG_PAGE = 0,

    // Text labels are a diagnostic overlay, not part of the minimum graphical
    // flight HUD. Disable them in resource-constrained Basys-3 builds so the
    // text generator and glyph compositor synthesize away.
    parameter integer ENABLE_TELEMETRY_TEXT_OVERLAY = 1
)(
    input  wire                      pix_clk,
    input  wire                      pix_rst,

    input  wire [`VIZ_BUNDLE_W-1:0]  viz_bundle_pix,
    input  wire                      viz_update_pix,
    input  wire [1:0]                vga_page_select_pix,

    input  wire [9:0]                wr_ptr_pix,
    input  wire                      wr_ptr_update_pix,

    output reg  [9:0]                alt_rd_addr,
    input  wire [15:0]               alt_rd_data,

    output reg  [9:0]                vspd_rd_addr,
    input  wire [15:0]               vspd_rd_data,

    output wire                      vga_hsync,
    output wire                      vga_vsync,
    output wire [11:0]               vga_rgb
);

    //==========================================================================
    // VGA timing
    //==========================================================================
    wire        active;
    wire        vga_hsync_raw;
    wire        vga_vsync_raw;
    wire [10:0] x;
    wire [10:0] y;
    wire        vsync_edge;

    localparam HSYNC_IDLE = (HSYNC_POL ? 1'b0 : 1'b1);
    localparam VSYNC_IDLE = (VSYNC_POL ? 1'b0 : 1'b1);

    vga_timing_640x480_60 #(
        .H_ACTIVE (H_ACTIVE),
        .H_FP     (H_FP),
        .H_SYNC   (H_SYNC),
        .H_BP     (H_BP),
        .V_ACTIVE (V_ACTIVE),
        .V_FP     (V_FP),
        .V_SYNC   (V_SYNC),
        .V_BP     (V_BP),
        .HSYNC_POL(HSYNC_POL),
        .VSYNC_POL(VSYNC_POL)
    ) u_vga (
        .clk       (pix_clk),
        .rst       (pix_rst),
        .hsync     (vga_hsync_raw),
        .vsync     (vga_vsync_raw),
        .active    (active),
        .x         (x),
        .y         (y),
        .vsync_edge(vsync_edge)
    );

    //==========================================================================
    // Frame counter
    //==========================================================================
    reg [31:0] frame_ctr;
    always @(posedge pix_clk) begin
        if (pix_rst)
            frame_ctr <= 32'd0;
        else if (vsync_edge)
            frame_ctr <= frame_ctr + 32'd1;
    end

    //==========================================================================
    // Latched committed visualization bundle
    //==========================================================================
    reg [`VIZ_BUNDLE_W-1:0] viz_q;
    reg [`VIZ_BUNDLE_W-1:0] viz_pending_q;
    reg                     viz_pending_valid_q;

    always @(posedge pix_clk) begin
        if (pix_rst) begin
            viz_q <= {`VIZ_BUNDLE_W{1'b0}};
            viz_pending_q <= {`VIZ_BUNDLE_W{1'b0}};
            viz_pending_valid_q <= 1'b0;
        end else begin
            if (viz_update_pix) begin
                viz_pending_q       <= viz_bundle_pix;
                viz_pending_valid_q <= 1'b1;
            end

            if (vsync_edge) begin
                if (viz_update_pix) begin
                    viz_q               <= viz_bundle_pix;
                    viz_pending_valid_q <= 1'b0;
                end else if (viz_pending_valid_q) begin
                    viz_q               <= viz_pending_q;
                    viz_pending_valid_q <= 1'b0;
                end
            end
        end
    end

    //==========================================================================
    // Canonical widened packed-bundle unpack
    //==========================================================================
    wire        auth_valid          = viz_q[`VIZ_AUTH_VALID_BIT];
    wire [7:0]  auth_status         = viz_q[`VIZ_AUTH_STATUS_MSB:`VIZ_AUTH_STATUS_LSB];
    wire [7:0]  auth_flags          = viz_q[`VIZ_AUTH_FLAGS_MSB:`VIZ_AUTH_FLAGS_LSB];
    wire [31:0] auth_target_cm      = viz_q[`VIZ_AUTH_TARGET_CM_MSB:`VIZ_AUTH_TARGET_CM_LSB];
    wire [31:0] auth_pred_no_cm     = viz_q[`VIZ_AUTH_PRED_NO_CM_MSB:`VIZ_AUTH_PRED_NO_CM_LSB];
    wire [31:0] auth_pred_full_cm   = viz_q[`VIZ_AUTH_PRED_FULL_CM_MSB:`VIZ_AUTH_PRED_FULL_CM_LSB];
    wire [15:0] auth_uncertainty_cm = viz_q[`VIZ_AUTH_UNC_CM_MSB:`VIZ_AUTH_UNC_CM_LSB];
    wire [7:0]  auth_brake_cmd_u8   = viz_q[`VIZ_AUTH_CMD_U8_MSB:`VIZ_AUTH_CMD_U8_LSB];
    wire [11:0] auth_servo_us       = viz_q[`VIZ_AUTH_SERVO_US_MSB:`VIZ_AUTH_SERVO_US_LSB];
    wire [3:0]  auth_phase_code     = viz_q[`VIZ_AUTH_PHASE_MSB:`VIZ_AUTH_PHASE_LSB];
    wire [6:0]  auth_gate_flags     = viz_q[`VIZ_AUTH_GATE_MSB:`VIZ_AUTH_GATE_LSB];

    wire [15:0] bmp_seq      = viz_q[`VIZ_BMP_SEQ_MSB:`VIZ_BMP_SEQ_LSB];
    wire        bmp_vld      = viz_q[`VIZ_BMP_VALID_BIT];
    wire [7:0]  bmp_st       = viz_q[`VIZ_BMP_STATUS_MSB:`VIZ_BMP_STATUS_LSB];
    wire [15:0] bmp_age_ms   = viz_q[`VIZ_BMP_AGE_MS_MSB:`VIZ_BMP_AGE_MS_LSB];

    wire [15:0] acc_seq      = viz_q[`VIZ_ACC_SEQ_MSB:`VIZ_ACC_SEQ_LSB];
    wire        acc_vld      = viz_q[`VIZ_ACC_VALID_BIT];
    wire [7:0]  acc_st       = viz_q[`VIZ_ACC_STATUS_MSB:`VIZ_ACC_STATUS_LSB];
    wire [15:0] acc_age_ms   = viz_q[`VIZ_ACC_AGE_MS_MSB:`VIZ_ACC_AGE_MS_LSB];

    wire [15:0] mag_seq      = viz_q[`VIZ_MAG_SEQ_MSB:`VIZ_MAG_SEQ_LSB];
    wire        mag_vld      = viz_q[`VIZ_MAG_VALID_BIT];
    wire [7:0]  mag_st       = viz_q[`VIZ_MAG_STATUS_MSB:`VIZ_MAG_STATUS_LSB];
    wire [15:0] mag_age_ms   = viz_q[`VIZ_MAG_AGE_MS_MSB:`VIZ_MAG_AGE_MS_LSB];

    wire [15:0] pwr_seq       = viz_q[`VIZ_PWR_SEQ_MSB:`VIZ_PWR_SEQ_LSB];
    wire        pwr_valid     = viz_q[`VIZ_PWR_VALID_BIT];
    wire [7:0]  pwr_status    = viz_q[`VIZ_PWR_STATUS_MSB:`VIZ_PWR_STATUS_LSB];
    wire [15:0] pwr_age_ms    = viz_q[`VIZ_PWR_AGE_MS_MSB:`VIZ_PWR_AGE_MS_LSB];
    wire [11:0] pwr_volt_code = viz_q[`VIZ_PWR_VOLT_CODE_MSB:`VIZ_PWR_VOLT_CODE_LSB];
    wire [11:0] pwr_curr_code = viz_q[`VIZ_PWR_CURR_CODE_MSB:`VIZ_PWR_CURR_CODE_LSB];
    wire [7:0]  pwr_alert     = viz_q[`VIZ_PWR_ALERT_MSB:`VIZ_PWR_ALERT_LSB];

    wire        ext_valid      = viz_q[`VIZ_EXT_VALID_BIT];
    wire [7:0]  ext_status     = viz_q[`VIZ_EXT_STATUS_MSB:`VIZ_EXT_STATUS_LSB];
    wire [15:0] ext_present_flags = viz_q[`VIZ_EXT_PRESENT_MSB:`VIZ_EXT_PRESENT_LSB];
    wire [15:0] ext_fault_flags   = viz_q[`VIZ_EXT_FAULT_MSB:`VIZ_EXT_FAULT_LSB];
    wire [15:0] ext_mag_delta_l1  =
        viz_q[`VIZ_EXT_MAG_DELTA_L1_MSB:`VIZ_EXT_MAG_DELTA_L1_LSB];
    wire [15:0] ext_mag_norm_primary =
        viz_q[`VIZ_EXT_MAG_NORM0_MSB:`VIZ_EXT_MAG_NORM0_LSB];
    wire [15:0] ext_mag_norm_secondary =
        viz_q[`VIZ_EXT_MAG_NORM1_MSB:`VIZ_EXT_MAG_NORM1_LSB];
    wire [15:0] ext_rng_height_cm =
        viz_q[`VIZ_EXT_RNG_HEIGHT_CM_MSB:`VIZ_EXT_RNG_HEIGHT_CM_LSB];
    wire [15:0] ext_air_dp_pa =
        viz_q[`VIZ_EXT_AIR_DP_PA_MSB:`VIZ_EXT_AIR_DP_PA_LSB];
    wire [15:0] ext_air_speed_cms =
        viz_q[`VIZ_EXT_AIR_SPEED_CMS_MSB:`VIZ_EXT_AIR_SPEED_CMS_LSB];
    wire [15:0] ext_env_temp_cdeg =
        viz_q[`VIZ_EXT_ENV_TEMP_CDEG_MSB:`VIZ_EXT_ENV_TEMP_CDEG_LSB];
    wire [15:0] ext_env_rh_centi =
        viz_q[`VIZ_EXT_ENV_RH_CENTI_MSB:`VIZ_EXT_ENV_RH_CENTI_LSB];
    wire [15:0] ext_sun_luma =
        viz_q[`VIZ_EXT_SUN_LUMA_MSB:`VIZ_EXT_SUN_LUMA_LSB];
    wire signed [15:0] ext_flow_dx =
        viz_q[`VIZ_EXT_FLOW_DX_MSB:`VIZ_EXT_FLOW_DX_LSB];
    wire signed [15:0] ext_flow_dy =
        viz_q[`VIZ_EXT_FLOW_DY_MSB:`VIZ_EXT_FLOW_DY_LSB];
    wire [15:0] ext_log_seq =
        viz_q[`VIZ_EXT_LOG_SEQ_MSB:`VIZ_EXT_LOG_SEQ_LSB];
    wire [15:0] ext_log_drop_count =
        viz_q[`VIZ_EXT_LOG_DROP_MSB:`VIZ_EXT_LOG_DROP_LSB];
    wire [15:0] ext_max_age_ms =
        viz_q[`VIZ_EXT_MAX_AGE_MS_MSB:`VIZ_EXT_MAX_AGE_MS_LSB];

    wire        der_valid    = viz_q[`VIZ_DER_VALID_BIT];
    wire [7:0]  der_status   = viz_q[`VIZ_DER_STATUS_MSB:`VIZ_DER_STATUS_LSB];

    wire        der_alt_fresh  = viz_q[`VIZ_DER_ALT_FRESH_BIT];
    wire        der_vspd_fresh = viz_q[`VIZ_DER_VSPD_FRESH_BIT];
    wire        der_roll_fresh = viz_q[`VIZ_DER_ROLL_FRESH_BIT];
    wire        der_head_fresh = viz_q[`VIZ_DER_HEAD_FRESH_BIT];

    wire [15:0] der_bmp_seq_ref = viz_q[`VIZ_DER_BMP_SEQ_REF_MSB:`VIZ_DER_BMP_SEQ_REF_LSB];
    wire [15:0] der_acc_seq_ref = viz_q[`VIZ_DER_ACC_SEQ_REF_MSB:`VIZ_DER_ACC_SEQ_REF_LSB];
    wire [15:0] der_mag_seq_ref = viz_q[`VIZ_DER_MAG_SEQ_REF_MSB:`VIZ_DER_MAG_SEQ_REF_LSB];

    wire [15:0] der_bmp_age_ms  = viz_q[`VIZ_DER_BMP_AGE_MS_MSB:`VIZ_DER_BMP_AGE_MS_LSB];
    wire [15:0] der_acc_age_ms  = viz_q[`VIZ_DER_ACC_AGE_MS_MSB:`VIZ_DER_ACC_AGE_MS_LSB];
    wire [15:0] der_mag_age_ms  = viz_q[`VIZ_DER_MAG_AGE_MS_MSB:`VIZ_DER_MAG_AGE_MS_LSB];

    wire        der_bmp_valid_ref = viz_q[`VIZ_DER_BMP_VALID_REF_BIT];
    wire        der_acc_valid_ref = viz_q[`VIZ_DER_ACC_VALID_REF_BIT];
    wire        der_mag_valid_ref = viz_q[`VIZ_DER_MAG_VALID_REF_BIT];

    wire [31:0] der_altitude_cm        = viz_q[`VIZ_DER_ALT_CM_MSB:`VIZ_DER_ALT_CM_LSB];
    wire [31:0] der_vertical_speed_cms = viz_q[`VIZ_DER_VSPD_CMS_MSB:`VIZ_DER_VSPD_CMS_LSB];
    wire [31:0] der_roll_mdeg          = viz_q[`VIZ_DER_ROLL_MDEG_MSB:`VIZ_DER_ROLL_MDEG_LSB];
    wire [31:0] der_heading_mdeg       = viz_q[`VIZ_DER_HEAD_MDEG_MSB:`VIZ_DER_HEAD_MDEG_LSB];

    wire        nav_valid          = viz_q[`VIZ_NAV_VALID_BIT];
    wire [7:0]  nav_status         = viz_q[`VIZ_NAV_STATUS_MSB:`VIZ_NAV_STATUS_LSB];
    wire [7:0]  nav_flags          = viz_q[`VIZ_NAV_FLAGS_MSB:`VIZ_NAV_FLAGS_LSB];
    wire signed [15:0] nav_downrange_m =
        viz_q[`VIZ_NAV_DOWNRANGE_M_MSB:`VIZ_NAV_DOWNRANGE_M_LSB];
    wire signed [15:0] nav_crossrange_m =
        viz_q[`VIZ_NAV_CROSSRANGE_M_MSB:`VIZ_NAV_CROSSRANGE_M_LSB];
    wire [15:0] nav_age_ms         = viz_q[`VIZ_NAV_AGE_MS_MSB:`VIZ_NAV_AGE_MS_LSB];

    wire        wind_valid         = viz_q[`VIZ_WIND_VALID_BIT];
    wire [7:0]  wind_status        = viz_q[`VIZ_WIND_STATUS_MSB:`VIZ_WIND_STATUS_LSB];
    wire signed [15:0] wind_x_cms  = viz_q[`VIZ_WIND_X_CMS_MSB:`VIZ_WIND_X_CMS_LSB];
    wire signed [15:0] wind_y_cms  = viz_q[`VIZ_WIND_Y_CMS_MSB:`VIZ_WIND_Y_CMS_LSB];
    wire signed [15:0] wind_z_cms  = viz_q[`VIZ_WIND_Z_CMS_MSB:`VIZ_WIND_Z_CMS_LSB];
    wire [15:0] wind_age_ms        = viz_q[`VIZ_WIND_AGE_MS_MSB:`VIZ_WIND_AGE_MS_LSB];

    wire [15:0] i2c_nack_count    = viz_q[`VIZ_I2C_NACK_MSB:`VIZ_I2C_NACK_LSB];
    wire [15:0] i2c_timeout_count = viz_q[`VIZ_I2C_TMO_MSB:`VIZ_I2C_TMO_LSB];
    wire [15:0] txn_rate_hz       = viz_q[`VIZ_TXN_RATE_MSB:`VIZ_TXN_RATE_LSB];
    wire [31:0] cdc_update_count  = viz_q[`VIZ_CDC_UPD_MSB:`VIZ_CDC_UPD_LSB];
    wire [31:0] build_id          = viz_q[`VIZ_BUILD_ID_MSB:`VIZ_BUILD_ID_LSB];
    wire [15:0] schema_word       = viz_q[`VIZ_SCHEMA_MSB:`VIZ_SCHEMA_LSB];

    //==========================================================================
    // Write pointer latch
    //==========================================================================
    reg [9:0] wr_ptr_q;
    reg [9:0] wr_ptr_pending_q;
    reg       wr_ptr_pending_valid_q;
    always @(posedge pix_clk) begin
        if (pix_rst) begin
            wr_ptr_q <= 10'd0;
            wr_ptr_pending_q <= 10'd0;
            wr_ptr_pending_valid_q <= 1'b0;
        end else begin
            if (wr_ptr_update_pix) begin
                wr_ptr_pending_q       <= wr_ptr_pix;
                wr_ptr_pending_valid_q <= 1'b1;
            end

            if (vsync_edge) begin
                if (wr_ptr_update_pix) begin
                    wr_ptr_q               <= wr_ptr_pix;
                    wr_ptr_pending_valid_q <= 1'b0;
                end else if (wr_ptr_pending_valid_q) begin
                    wr_ptr_q               <= wr_ptr_pending_q;
                    wr_ptr_pending_valid_q <= 1'b0;
                end
            end
        end
    end

    //==========================================================================
    // BRAM chart addressing: right edge = newest sample
    //==========================================================================
    wire [9:0] k         = 10'd639 - x[9:0];
    wire [9:0] samp_addr = wr_ptr_q - 10'd1 - k;

    always @(posedge pix_clk) begin
        if (pix_rst) begin
            alt_rd_addr  <= 10'd0;
            vspd_rd_addr <= 10'd0;
        end else begin
            alt_rd_addr  <= samp_addr;
            vspd_rd_addr <= samp_addr;
        end
    end

    //==========================================================================
    // Coordinate pipeline for BRAM latency alignment
    //==========================================================================
    reg [10:0] x_q0, y_q0;
    reg [10:0] x_q1, y_q1;
    reg        active_q0, active_q1;
    reg        hsync_q0, hsync_q1;
    reg        vsync_q0, vsync_q1;

    always @(posedge pix_clk) begin
        if (pix_rst) begin
            x_q0      <= 11'd0;
            y_q0      <= 11'd0;
            active_q0 <= 1'b0;
            hsync_q0  <= HSYNC_IDLE;
            vsync_q0  <= VSYNC_IDLE;

            x_q1      <= 11'd0;
            y_q1      <= 11'd0;
            active_q1 <= 1'b0;
            hsync_q1  <= HSYNC_IDLE;
            vsync_q1  <= VSYNC_IDLE;
        end else begin
            x_q0      <= x;
            y_q0      <= y;
            active_q0 <= active;
            hsync_q0  <= vga_hsync_raw;
            vsync_q0  <= vga_vsync_raw;

            x_q1      <= x_q0;
            y_q1      <= y_q0;
            active_q1 <= active_q0;
            hsync_q1  <= hsync_q0;
            vsync_q1  <= vsync_q0;
        end
    end

    //==========================================================================
    // Helper functions
    //==========================================================================
    function [11:0] rgb;
        input [3:0] r;
        input [3:0] g;
        input [3:0] b;
        begin
            rgb = {r, g, b};
        end
    endfunction

    function [15:0] sat_u16_from_u32;
        input [31:0] v;
        begin
            if (v[31:16] != 16'd0)
                sat_u16_from_u32 = 16'hFFFF;
            else
                sat_u16_from_u32 = v[15:0];
        end
    endfunction

    function [15:0] sat_s16_from_s32;
        input [31:0] v;
        begin
            if (v[31] == 1'b0) begin
                if (v[30:15] != 16'd0)
                    sat_s16_from_s32 = 16'h7FFF;
                else
                    sat_s16_from_s32 = v[15:0];
            end else begin
                if (v[30:15] != 16'hFFFF)
                    sat_s16_from_s32 = 16'h8000;
                else
                    sat_s16_from_s32 = v[15:0];
            end
        end
    endfunction

    function [10:0] clamp_y;
        input integer v;
        input integer y0;
        input integer y1;
        begin
            if (v < y0)
                clamp_y = y0[10:0];
            else if (v > y1)
                clamp_y = y1[10:0];
            else
                clamp_y = v[10:0];
        end
    endfunction

    function [15:0] abs16u;
        input signed [15:0] v;
        begin
            if (v[15])
                abs16u = (v == -16'sd32768) ? 16'h7FFF : (16'd0 - v);
            else
                abs16u = v[15:0];
        end
    endfunction

    function [12:0] abs13u;
        input signed [12:0] v;
        begin
            if (v[12])
                abs13u = (v == -13'sd4096) ? 13'h0FFF : (13'd0 - v);
            else
                abs13u = v[12:0];
        end
    endfunction

    function signed [12:0] clip_s13;
        input signed [15:0] v;
        input signed [15:0] lo;
        input signed [15:0] hi;
        begin
            if (v < lo)
                clip_s13 = lo[12:0];
            else if (v > hi)
                clip_s13 = hi[12:0];
            else
                clip_s13 = v[12:0];
        end
    endfunction

    function between_s13;
        input signed [12:0] v;
        input signed [12:0] a;
        input signed [12:0] b;
        begin
            if (a <= b)
                between_s13 = (v >= a) && (v <= b);
            else
                between_s13 = (v >= b) && (v <= a);
        end
    endfunction

    function [11:0] health_color_ms;
        input        v_ok;
        input [7:0]  st;
        input [15:0] age_ms;
        input [15:0] stale_ms;
        reg bad;
        reg stale;
        begin
            bad   = (st != 8'h00);
            stale = (age_ms >= stale_ms);

            if (!v_ok)
                health_color_ms = rgb(4'h6, 4'h6, 4'h7);
            else if (bad)
                health_color_ms = rgb(4'hD, 4'h3, 4'h3);
            else if (stale)
                health_color_ms = rgb(4'hD, 4'h9, 4'h3);
            else
                health_color_ms = rgb(4'h2, 4'hB, 4'h5);
        end
    endfunction

    function [11:0] derived_color;
        input        der_ok;
        input [7:0]  der_st;
        input        fresh_ok;
        input        ref_ok;
        input [11:0] good_rgb;
        begin
            if (!der_ok || !ref_ok)
                derived_color = rgb(4'h7, 4'h7, 4'h7);
            else if (der_st != 8'h00)
                derived_color = rgb(4'hD, 4'h3, 4'h3);
            else if (!fresh_ok)
                derived_color = rgb(4'hD, 4'h9, 4'h3);
            else
                derived_color = good_rgb;
        end
    endfunction

    function [10:0] age_bar_width_px;
        input [15:0] age_ms;
        input [15:0] stale_ms;
        integer scaled;
        begin
            if (stale_ms == 16'd0) begin
                scaled = 176;
            end else if (age_ms >= stale_ms) begin
                scaled = 176;
            end else begin
                scaled = (age_ms * 176) / stale_ms;
            end

            if (scaled < 0)
                age_bar_width_px = 11'd0;
            else if (scaled > 176)
                age_bar_width_px = 11'd176;
            else
                age_bar_width_px = scaled[10:0];
        end
    endfunction

    function [10:0] bar_width_u16;
        input [15:0] value;
        input [15:0] full_scale;
        input [10:0] width_px;
        integer scaled;
        begin
            if (full_scale == 16'd0)
                scaled = width_px;
            else if (value >= full_scale)
                scaled = width_px;
            else
                scaled = (value * width_px) / full_scale;

            if (scaled < 0)
                bar_width_u16 = 11'd0;
            else if (scaled > width_px)
                bar_width_u16 = width_px;
            else
                bar_width_u16 = scaled[10:0];
        end
    endfunction

    function [10:0] offset_coord_s13;
        input integer base;
        input signed [12:0] delta;
        input integer lo;
        input integer hi;
        integer v;
        begin
            v = base + delta;
            if (v < lo)
                offset_coord_s13 = lo[10:0];
            else if (v > hi)
                offset_coord_s13 = hi[10:0];
            else
                offset_coord_s13 = v[10:0];
        end
    endfunction

    //--------------------------------------------------------------------------
    // Angle -> octant mapping
    //--------------------------------------------------------------------------
    function [2:0] octant_from_u_mdeg;
        input [31:0] ang_mdeg;
        begin
            if (ang_mdeg < 32'd22500)
                octant_from_u_mdeg = 3'd0;
            else if (ang_mdeg < 32'd67500)
                octant_from_u_mdeg = 3'd1;
            else if (ang_mdeg < 32'd112500)
                octant_from_u_mdeg = 3'd2;
            else if (ang_mdeg < 32'd157500)
                octant_from_u_mdeg = 3'd3;
            else if (ang_mdeg < 32'd202500)
                octant_from_u_mdeg = 3'd4;
            else if (ang_mdeg < 32'd247500)
                octant_from_u_mdeg = 3'd5;
            else if (ang_mdeg < 32'd292500)
                octant_from_u_mdeg = 3'd6;
            else if (ang_mdeg < 32'd337500)
                octant_from_u_mdeg = 3'd7;
            else
                octant_from_u_mdeg = 3'd0;
        end
    endfunction

    function [2:0] octant_from_s_mdeg;
        input signed [31:0] ang_mdeg_s;
        reg signed [31:0] a;
        begin
            a = ang_mdeg_s;
            if (a < 0)
                a = a + 32'sd360000;
            if (a < 0)
                a = 32'sd0;

            if (a < 32'sd22500)
                octant_from_s_mdeg = 3'd0;
            else if (a < 32'sd67500)
                octant_from_s_mdeg = 3'd1;
            else if (a < 32'sd112500)
                octant_from_s_mdeg = 3'd2;
            else if (a < 32'sd157500)
                octant_from_s_mdeg = 3'd3;
            else if (a < 32'sd202500)
                octant_from_s_mdeg = 3'd4;
            else if (a < 32'sd247500)
                octant_from_s_mdeg = 3'd5;
            else if (a < 32'sd292500)
                octant_from_s_mdeg = 3'd6;
            else if (a < 32'sd337500)
                octant_from_s_mdeg = 3'd7;
            else
                octant_from_s_mdeg = 3'd0;
        end
    endfunction

    function signed [2:0] dir_x_from_oct;
        input [2:0] oct;
        begin
            case (oct)
                3'd0: dir_x_from_oct =  3'sd1;
                3'd1: dir_x_from_oct =  3'sd1;
                3'd2: dir_x_from_oct =  3'sd0;
                3'd3: dir_x_from_oct = -3'sd1;
                3'd4: dir_x_from_oct = -3'sd1;
                3'd5: dir_x_from_oct = -3'sd1;
                3'd6: dir_x_from_oct =  3'sd0;
                3'd7: dir_x_from_oct =  3'sd1;
                default: dir_x_from_oct = 3'sd1;
            endcase
        end
    endfunction

    function signed [2:0] dir_y_from_oct;
        input [2:0] oct;
        begin
            case (oct)
                3'd0: dir_y_from_oct =  3'sd0;
                3'd1: dir_y_from_oct =  3'sd1;
                3'd2: dir_y_from_oct =  3'sd1;
                3'd3: dir_y_from_oct =  3'sd1;
                3'd4: dir_y_from_oct =  3'sd0;
                3'd5: dir_y_from_oct = -3'sd1;
                3'd6: dir_y_from_oct = -3'sd1;
                3'd7: dir_y_from_oct = -3'sd1;
                default: dir_y_from_oct = 3'sd0;
            endcase
        end
    endfunction

    function signed [15:0] dot_from_oct;
        input [2:0] oct;
        input signed [12:0] dx;
        input signed [12:0] dy;
        begin
            case (oct)
                3'd0: dot_from_oct = dx;
                3'd1: dot_from_oct = dx + dy;
                3'd2: dot_from_oct = dy;
                3'd3: dot_from_oct = -dx + dy;
                3'd4: dot_from_oct = -dx;
                3'd5: dot_from_oct = -dx - dy;
                3'd6: dot_from_oct = -dy;
                3'd7: dot_from_oct = dx - dy;
                default: dot_from_oct = dx;
            endcase
        end
    endfunction

    function signed [15:0] cross_from_oct;
        input [2:0] oct;
        input signed [12:0] dx;
        input signed [12:0] dy;
        begin
            case (oct)
                3'd0: cross_from_oct = dy;
                3'd1: cross_from_oct = dy - dx;
                3'd2: cross_from_oct = -dx;
                3'd3: cross_from_oct = -dy - dx;
                3'd4: cross_from_oct = -dy;
                3'd5: cross_from_oct = dx - dy;
                3'd6: cross_from_oct = dx;
                3'd7: cross_from_oct = dx + dy;
                default: cross_from_oct = dy;
            endcase
        end
    endfunction

    //==========================================================================
    // Transitional plotting numerics from true derived-state fields
    //==========================================================================
    wire [15:0] alt16_plot  = sat_u16_from_u32(der_altitude_cm);
    wire [15:0] vspd16_plot = sat_s16_from_s32(der_vertical_speed_cms);
    wire        vspd_negative = vspd16_plot[15];

    //==========================================================================
    // Layout contract
    //---------------------------------------------------------------------------
    // 640x480 Basys-3 VGA presentation layout:
    //   0..27    compact health/status strip
    //   28..105  fixed title + telemetry bands, owned by the text compositor
    //   106..359 primary instruments
    //   360..467 engineering charts
    //   468..479 muted debug strip
    //==========================================================================
    localparam [15:0] BMP_STALE_MS = 16'd150;
    localparam [15:0] ACC_STALE_MS = 16'd80;
    localparam [15:0] MAG_STALE_MS = 16'd250;

    localparam integer TOP_H = 28;

    localparam integer MAIN_Y0 = 28;
    localparam integer MAIN_Y1 = 359;

    localparam integer CHART_ALT_Y0  = 360;
    localparam integer CHART_ALT_Y1  = 415;
    localparam integer CHART_VSPD_Y0 = 416;
    localparam integer CHART_VSPD_Y1 = 467;

    localparam integer COL0_X0 = 0;
    localparam integer COL0_X1 = 212;
    localparam integer COL1_X0 = 213;
    localparam integer COL1_X1 = 425;
    localparam integer COL2_X0 = 426;
    localparam integer COL2_X1 = 639;

    localparam integer TAPE_PAD_X = 8;
    localparam integer TAPE_W     = 72;
    localparam integer TAPE_GAP   = 10;

    localparam integer ALT_TAPE_X0  = COL0_X0 + TAPE_PAD_X;
    localparam integer ALT_TAPE_X1  = ALT_TAPE_X0 + TAPE_W - 1;
    localparam integer VSPD_TAPE_X0 = ALT_TAPE_X1 + 1 + TAPE_GAP;
    localparam integer VSPD_TAPE_X1 = VSPD_TAPE_X0 + TAPE_W - 1;

    localparam integer TAPE_Y0 = 108;
    localparam integer TAPE_Y1 = MAIN_Y1 - 12;
    localparam [10:0] TAPE_CY = ((TAPE_Y0 + TAPE_Y1) / 2);

    localparam integer AUTH_X0 = COL0_X1 - 44;
    localparam integer AUTH_X1 = COL0_X1 - 8;
    localparam integer AUTH_Y0 = TAPE_Y0;
    localparam integer AUTH_Y1 = TAPE_Y1;
    localparam integer AUTH_CX = (AUTH_X0 + AUTH_X1) / 2;
    localparam integer AUTH_SPAN = AUTH_Y1 - AUTH_Y0;
    localparam [31:0]  AUTH_SCALE_CM  = AUTH_SPAN * 32'd2048;
    localparam integer AUTH_SCALE_SHIFT = 11;
    localparam integer AUTH_CMD_X0 = AUTH_X1 - 6;
    localparam integer AUTH_CMD_X1 = AUTH_X1 - 3;
    localparam integer AUTH_CMD_L1_Y = AUTH_Y1 - (AUTH_SPAN / 4);
    localparam integer AUTH_CMD_L2_Y = AUTH_Y1 - (AUTH_SPAN / 2);
    localparam integer AUTH_CMD_L3_Y = AUTH_Y1 - ((AUTH_SPAN * 3) / 4);

    localparam integer HZN_CX = (COL1_X0 + COL1_X1) / 2;
    localparam integer HZN_CY = 205;
    localparam integer HZN_R  = 96;

    localparam integer VEC_X0 = COL1_X0 + 26;
    localparam integer VEC_X1 = COL1_X1 - 26;
    localparam integer VEC_Y0 = 312;
    localparam integer VEC_Y1 = 352;
    localparam integer VEC_CX = (VEC_X0 + VEC_X1) / 2;
    localparam integer VEC_CY = (VEC_Y0 + VEC_Y1) / 2;

    localparam integer CMP_CX = (COL2_X0 + COL2_X1) / 2;
    localparam integer CMP_CY = 205;
    localparam integer CMP_R  = 86;

    localparam integer LAND_X0 = COL2_X0 + 12;
    localparam integer LAND_X1 = COL2_X1 - 12;
    localparam integer LAND_Y0 = 306;
    localparam integer LAND_Y1 = 352;
    localparam integer LAND_CX = (LAND_X0 + LAND_X1) / 2;
    localparam integer LAND_CY = (LAND_Y0 + LAND_Y1) / 2;

    localparam [1:0] VGA_PAGE_HUD         = 2'd0;
    localparam [1:0] VGA_PAGE_SENSOR_DIAG = 2'd1;

    localparam integer DIAG_MATRIX_X0 = 8;
    localparam integer DIAG_MATRIX_X1 = 314;
    localparam integer DIAG_MATRIX_Y0 = 36;
    localparam integer DIAG_MATRIX_Y1 = 284;
    localparam integer DIAG_ROW_H     = 28;

    localparam integer DIAG_SCALE_X0 = 330;
    localparam integer DIAG_SCALE_X1 = 628;
    localparam integer DIAG_SCALE_Y0 = 36;
    localparam integer DIAG_SCALE_Y1 = 222;
    localparam integer DIAG_BAR_X0   = 430;
    localparam integer DIAG_BAR_X1   = 616;
    localparam integer DIAG_BAR_W    = DIAG_BAR_X1 - DIAG_BAR_X0;
    localparam [10:0]  DIAG_BAR_W_U11 = DIAG_BAR_W;
    localparam integer DIAG_BAR_H    = 12;

    localparam integer DIAG_TRAIL_X0 = 330;
    localparam integer DIAG_TRAIL_X1 = 628;
    localparam integer DIAG_TRAIL_Y0 = 246;
    localparam integer DIAG_TRAIL_Y1 = 452;
    localparam integer DIAG_TRAIL_CX = (DIAG_TRAIL_X0 + DIAG_TRAIL_X1) / 2;
    localparam integer DIAG_TRAIL_CY = (DIAG_TRAIL_Y0 + DIAG_TRAIL_Y1) / 2;
    localparam signed [12:0] DIAG_TRAIL_CX_S = DIAG_TRAIL_CX;
    localparam signed [12:0] DIAG_TRAIL_CY_S = DIAG_TRAIL_CY;
    localparam integer DIAG_TRAIL_DEPTH = 16;

    localparam signed [12:0] HZN_CX_S   = HZN_CX;
    localparam signed [12:0] HZN_CY_S   = HZN_CY;
    localparam signed [12:0] VEC_CX_S   = VEC_CX;
    localparam signed [12:0] VEC_CY_S   = VEC_CY;
    localparam signed [12:0] CMP_CX_S   = CMP_CX;
    localparam signed [12:0] CMP_CY_S   = CMP_CY;
    localparam signed [12:0] LAND_CX_S  = LAND_CX;
    localparam signed [12:0] LAND_CY_S  = LAND_CY;
    localparam signed [12:0] HZN_R_S    = HZN_R;
    localparam signed [12:0] HZN_RM12_S = HZN_R - 12;
    localparam signed [15:0] VEC_DOT_MAX_S    = 16'sd22;
    localparam signed [15:0] VEC_DOT_TIP_LO_S = 16'sd17;
    localparam signed [15:0] VEC_DOT_TIP_HI_S = 16'sd24;

    localparam [10:0] CHART_ALT_Q1_Y  = CHART_ALT_Y0 + ((CHART_ALT_Y1 - CHART_ALT_Y0) / 4);
    localparam [10:0] CHART_ALT_MID_Y = CHART_ALT_Y0 + ((CHART_ALT_Y1 - CHART_ALT_Y0) / 2);
    localparam [10:0] CHART_ALT_Q3_Y  = CHART_ALT_Y0 + (((CHART_ALT_Y1 - CHART_ALT_Y0) * 3) / 4);
    localparam [10:0] CHART_VSPD_CY   = ((CHART_VSPD_Y0 + CHART_VSPD_Y1) / 2);

    //==========================================================================
    // Apogee-authority display mapping
    //==========================================================================
    function [10:0] auth_cm_to_y;
        input [31:0] cm;
        reg [31:0] scaled_px;
        integer yv;
        begin
            if (cm >= AUTH_SCALE_CM) begin
                yv = AUTH_Y0;
            end else begin
                scaled_px = cm >> AUTH_SCALE_SHIFT;
                yv = AUTH_Y1 - scaled_px;
            end

            auth_cm_to_y = clamp_y(yv, AUTH_Y0, AUTH_Y1);
        end
    endfunction

    //==========================================================================
    // Tape mappings
    //==========================================================================
    function [10:0] alt_to_y;
        input [15:0] a;
        integer span;
        integer y0;
        integer y1;
        integer h;
        integer yv;
        begin
            y0   = TAPE_Y0;
            y1   = TAPE_Y1;
            span = (y1 - y0);
            h    = a;
            if (h > 2048)
                h = 2048;
            yv = y1 - (h * span) / 2048;
            alt_to_y = clamp_y(yv, y0, y1);
        end
    endfunction

    function [10:0] vspd_to_y;
        input signed [15:0] v;
        integer span;
        integer y0;
        integer y1;
        integer vv;
        integer yv;
        begin
            y0   = TAPE_Y0;
            y1   = TAPE_Y1;
            span = (y1 - y0);
            vv   = v;
            if (vv > 127)
                vv = 127;
            if (vv < -128)
                vv = -128;
            yv = (y0 + y1)/2 - (vv * (span/2)) / 128;
            vspd_to_y = clamp_y(yv, y0, y1);
        end
    endfunction

    function [10:0] chart_y_alt;
        input [15:0] a;
        integer y0, y1, span, h, yv;
        begin
            y0   = CHART_ALT_Y0;
            y1   = CHART_ALT_Y1;
            span = (y1 - y0);
            h    = a;
            if (h > 2048)
                h = 2048;
            yv = y1 - (h * span) / 2048;
            chart_y_alt = clamp_y(yv, y0, y1);
        end
    endfunction

    function [10:0] chart_y_vspd;
        input [15:0] v_u16;
        integer y0, y1, span, vv, yv;
        reg signed [15:0] v_s16;
        begin
            y0   = CHART_VSPD_Y0;
            y1   = CHART_VSPD_Y1;
            span = (y1 - y0);
            v_s16 = v_u16;
            vv   = v_s16;
            if (vv > 127)
                vv = 127;
            if (vv < -128)
                vv = -128;
            yv = (y0 + y1)/2 - (vv * (span/2)) / 128;
            chart_y_vspd = clamp_y(yv, y0, y1);
        end
    endfunction

    wire [10:0] alt_y       = alt_to_y(alt16_plot);
    wire [10:0] vspd_y      = vspd_to_y(vspd16_plot);
    wire [10:0] plot_alt_y  = chart_y_alt(alt_rd_data);
    wire [10:0] plot_vspd_y = chart_y_vspd(vspd_rd_data);

    //==========================================================================
    // Apogee authority envelope
    //---------------------------------------------------------------------------
    // The ladder consumes the SYS-domain authority record from the visualization
    // bundle. Predictor and servo policy ownership therefore stays before the
    // SYS->PIX CDC boundary; this renderer only maps the already-published
    // evidence to pixels.
    //==========================================================================
    wire auth_data_ok =
        auth_valid &&
        (auth_status == 8'h00) &&
        auth_flags[`VIZ_AUTH_FLG_INPUT_OK_BIT];

    wire [31:0] auth_unc_cm = {16'd0, auth_uncertainty_cm};

    wire [31:0] auth_target_low_cm =
        (auth_target_cm > auth_unc_cm) ? (auth_target_cm - auth_unc_cm) : 32'd0;
    wire [32:0] auth_target_hi_sum_cm =
        {1'b0, auth_target_cm} + {1'b0, auth_unc_cm};
    wire [31:0] auth_target_hi_cm =
        auth_target_hi_sum_cm[32] ? 32'hFFFF_FFFF : auth_target_hi_sum_cm[31:0];

    reg [2:0] auth_cmd_level;
    always @(*) begin
        if (!auth_data_ok || (auth_brake_cmd_u8 == 8'd0))
            auth_cmd_level = 3'd0;
        else if (auth_brake_cmd_u8 <= 8'd64)
            auth_cmd_level = 3'd1;
        else if (auth_brake_cmd_u8 <= 8'd128)
            auth_cmd_level = 3'd2;
        else if (auth_brake_cmd_u8 <= 8'd192)
            auth_cmd_level = 3'd3;
        else
            auth_cmd_level = 3'd4;
    end

    reg [10:0] auth_cmd_top_y;
    always @(*) begin
        case (auth_cmd_level)
            3'd1: auth_cmd_top_y = AUTH_CMD_L1_Y[10:0];
            3'd2: auth_cmd_top_y = AUTH_CMD_L2_Y[10:0];
            3'd3: auth_cmd_top_y = AUTH_CMD_L3_Y[10:0];
            3'd4: auth_cmd_top_y = AUTH_Y0[10:0];
            default: auth_cmd_top_y = AUTH_Y1[10:0];
        endcase
    end

    wire auth_target_reachable =
        auth_data_ok &&
        auth_flags[`VIZ_AUTH_FLG_REACHABLE_BIT];

    wire [10:0] auth_current_y = auth_cm_to_y(der_altitude_cm);
    wire [10:0] auth_target_y  = auth_cm_to_y(auth_target_cm);
    wire [10:0] auth_no_y      = auth_cm_to_y(auth_pred_no_cm);
    wire [10:0] auth_full_y    = auth_cm_to_y(auth_pred_full_cm);
    wire [10:0] auth_unc_hi_y  = auth_cm_to_y(auth_target_hi_cm);
    wire [10:0] auth_unc_lo_y  = auth_cm_to_y(auth_target_low_cm);

    wire [10:0] auth_env_y_top = (auth_no_y < auth_full_y) ? auth_no_y : auth_full_y;
    wire [10:0] auth_env_y_bot = (auth_no_y < auth_full_y) ? auth_full_y : auth_no_y;

    wire [11:0] auth_envelope_color =
        (!auth_data_ok)          ? rgb(4'h7, 4'h7, 4'h7) :
        auth_target_reachable    ? rgb(4'h2, 4'hF, 4'h6) :
                                   rgb(4'hF, 4'h4, 4'h2);
    wire [11:0] auth_command_color =
        (auth_cmd_level == 3'd0) ? rgb(4'h2, 4'h4, 4'h3) :
        auth_target_reachable    ? rgb(4'h2, 4'hF, 4'h6) :
                                   rgb(4'hF, 4'h8, 4'h0);

    wire auth_safety_runtime_ok =
        auth_gate_flags[`VIZ_AUTH_GATE_SAFETY_RUNTIME_OK_BIT];
    wire auth_safety_allows_actuation =
        auth_gate_flags[`VIZ_AUTH_GATE_SAFETY_ALLOWS_BIT];
    wire auth_policy_runtime_enable =
        auth_gate_flags[`VIZ_AUTH_GATE_POLICY_ENABLE_BIT];
    wire auth_software_armed =
        auth_gate_flags[`VIZ_AUTH_GATE_SOFTWARE_ARMED_BIT];
    wire auth_actuator_active =
        auth_gate_flags[`VIZ_AUTH_GATE_ACTUATOR_ACTIVE_BIT];
    wire auth_external_phase_valid =
        auth_gate_flags[`VIZ_AUTH_GATE_EXTERNAL_PHASE_BIT];
    wire auth_local_phase_used =
        auth_gate_flags[`VIZ_AUTH_GATE_LOCAL_PHASE_BIT];

    wire auth_actuation_allowed =
        auth_safety_runtime_ok &&
        auth_safety_allows_actuation &&
        auth_policy_runtime_enable &&
        auth_software_armed &&
        auth_flags[`VIZ_AUTH_FLG_ACT_SAFE_BIT];

    wire auth_policy_demands_actuation = (auth_brake_cmd_u8 != 8'd0);

    reg [11:0] auth_phase_color;
    always @(*) begin
        case (auth_phase_code)
            `VIZ_AUTH_PHASE_IDLE:    auth_phase_color = rgb(4'h2, 4'h5, 4'h9);
            `VIZ_AUTH_PHASE_BOOST:   auth_phase_color = rgb(4'hF, 4'h8, 4'h0);
            `VIZ_AUTH_PHASE_COAST:   auth_phase_color = rgb(4'h2, 4'hC, 4'hF);
            `VIZ_AUTH_PHASE_BRAKE:   auth_phase_color = rgb(4'hF, 4'h0, 4'hF);
            `VIZ_AUTH_PHASE_DESCENT: auth_phase_color = rgb(4'hF, 4'h4, 4'h2);
            default:                 auth_phase_color = rgb(4'h6, 4'h6, 4'h6);
        endcase
    end

    wire [11:0] auth_gate_color =
        (!auth_data_ok) ? rgb(4'h7, 4'h7, 4'h7) :
        auth_actuator_active ? rgb(4'h2, 4'hF, 4'h6) :
        (auth_policy_demands_actuation && !auth_actuation_allowed) ?
            rgb(4'hF, 4'h2, 4'h0) :
        auth_actuation_allowed ? rgb(4'h2, 4'h8, 4'h6) :
                                 rgb(4'hF, 4'h8, 4'h0);

    //==========================================================================
    // Regions and shared style
    //==========================================================================
    wire in_top        = (y_q1 < TOP_H[10:0]);
    wire in_main       = (y_q1 >= MAIN_Y0[10:0]) && (y_q1 <= MAIN_Y1[10:0]);
    wire in_col0       = in_main && (x_q1 <= COL0_X1[10:0]);
    wire in_col1       = in_main && (x_q1 >= COL1_X0[10:0]) && (x_q1 <= COL1_X1[10:0]);
    wire in_col2       = in_main && (x_q1 >= COL2_X0[10:0]) && (x_q1 <= COL2_X1[10:0]);
    wire in_chart_alt  = (y_q1 >= CHART_ALT_Y0[10:0])  && (y_q1 <= CHART_ALT_Y1[10:0]);
    wire in_chart_vspd = (y_q1 >= CHART_VSPD_Y0[10:0]) && (y_q1 <= CHART_VSPD_Y1[10:0]);

    wire [11:0] hc_bmp = health_color_ms(bmp_vld, bmp_st, bmp_age_ms, BMP_STALE_MS);
    wire [11:0] hc_acc = health_color_ms(acc_vld, acc_st, acc_age_ms, ACC_STALE_MS);
    wire [11:0] hc_mag = health_color_ms(mag_vld, mag_st, mag_age_ms, MAG_STALE_MS);

    wire [11:0] alt_color  = derived_color(der_valid, der_status,
                                            der_alt_fresh, der_bmp_valid_ref,
                                            rgb(4'h2, 4'hF, 4'h9));
    wire [11:0] vspd_good_color = vspd_negative ? rgb(4'hF, 4'h7, 4'h2)
                                                 : rgb(4'h2, 4'hC, 4'hF);
    wire [11:0] vspd_color = derived_color(der_valid, der_status,
                                            der_vspd_fresh, der_bmp_valid_ref,
                                            vspd_good_color);
    wire [11:0] roll_color = derived_color(der_valid, der_status,
                                            der_roll_fresh, der_acc_valid_ref,
                                            rgb(4'hF, 4'hF, 4'hF));
    wire [11:0] vert_vec_color = derived_color(der_valid, der_status,
                                               der_roll_fresh, der_acc_valid_ref,
                                               rgb(4'h3, 4'hF, 4'hF));
    wire [11:0] head_color = derived_color(der_valid, der_status,
                                            der_head_fresh, der_mag_valid_ref,
                                            rgb(4'hF, 4'h2, 4'h2));

    wire [10:0] bmp_age_w = age_bar_width_px(bmp_age_ms, BMP_STALE_MS);
    wire [10:0] acc_age_w = age_bar_width_px(acc_age_ms, ACC_STALE_MS);
    wire [10:0] mag_age_w = age_bar_width_px(mag_age_ms, MAG_STALE_MS);

    wire grid = (x_q1[5:0] == 6'd0) || (y_q1[5:0] == 6'd0);
    wire roll_quality_ok =
        der_valid &&
        (der_status == 8'h00) &&
        der_roll_fresh &&
        der_acc_valid_ref;

    //--------------------------------------------------------------------------
    // Top health strip regions
    //--------------------------------------------------------------------------
    wire bmp_blk = in_top && (x_q1 >= 11'd12)  && (x_q1 < 11'd208);
    wire acc_blk = in_top && (x_q1 >= 11'd218) && (x_q1 < 11'd416);
    wire mag_blk = in_top && (x_q1 >= 11'd424) && (x_q1 < 11'd624);

    wire bmp_bar =
        bmp_blk &&
        (x_q1 >= 11'd20) &&
        (x_q1 < (11'd20 + bmp_age_w)) &&
        (y_q1 >= 11'd21) && (y_q1 < 11'd25);

    wire acc_bar =
        acc_blk &&
        (x_q1 >= 11'd226) &&
        (x_q1 < (11'd226 + acc_age_w)) &&
        (y_q1 >= 11'd21) && (y_q1 < 11'd25);

    wire mag_bar =
        mag_blk &&
        (x_q1 >= 11'd432) &&
        (x_q1 < (11'd432 + mag_age_w)) &&
        (y_q1 >= 11'd21) && (y_q1 < 11'd25);

    //--------------------------------------------------------------------------
    // Tapes
    //--------------------------------------------------------------------------
    wire in_alt_tape =
        in_col0 &&
        (x_q1 >= ALT_TAPE_X0[10:0]) && (x_q1 <= ALT_TAPE_X1[10:0]) &&
        (y_q1 >= TAPE_Y0[10:0])     && (y_q1 <= TAPE_Y1[10:0]);

    wire in_vspd_tape =
        in_col0 &&
        (x_q1 >= VSPD_TAPE_X0[10:0]) && (x_q1 <= VSPD_TAPE_X1[10:0]) &&
        (y_q1 >= TAPE_Y0[10:0])      && (y_q1 <= TAPE_Y1[10:0]);

    wire tape_tick = (y_q1[3:0] == 4'd0) && (x_q1[2:0] == 3'd0);

    wire alt_tape_border =
        in_alt_tape &&
        ((x_q1 == ALT_TAPE_X0[10:0]) || (x_q1 == ALT_TAPE_X1[10:0]) ||
         (y_q1 == TAPE_Y0[10:0])     || (y_q1 == TAPE_Y1[10:0]));

    wire vspd_tape_border =
        in_vspd_tape &&
        ((x_q1 == VSPD_TAPE_X0[10:0]) || (x_q1 == VSPD_TAPE_X1[10:0]) ||
         (y_q1 == TAPE_Y0[10:0])      || (y_q1 == TAPE_Y1[10:0]));

    wire vspd_zero_line = in_vspd_tape && (y_q1 == TAPE_CY);

    wire alt_line =
        in_alt_tape &&
        ((y_q1 == alt_y) || (y_q1 + 11'd1 == alt_y) || (y_q1 == alt_y + 11'd1));

    wire vspd_line =
        in_vspd_tape &&
        ((y_q1 == vspd_y) || (y_q1 + 11'd1 == vspd_y) || (y_q1 == vspd_y + 11'd1));

    //---------------------------------------------------------------------------
    // Column 0: apogee authority envelope
    //---------------------------------------------------------------------------
    wire in_auth_panel =
        in_col0 &&
        (x_q1 >= AUTH_X0[10:0]) && (x_q1 <= AUTH_X1[10:0]) &&
        (y_q1 >= AUTH_Y0[10:0]) && (y_q1 <= AUTH_Y1[10:0]);

    wire auth_panel_border =
        in_auth_panel &&
        ((x_q1 == AUTH_X0[10:0]) || (x_q1 == AUTH_X1[10:0]) ||
         (y_q1 == AUTH_Y0[10:0]) || (y_q1 == AUTH_Y1[10:0]));

    wire auth_axis =
        in_auth_panel &&
        (x_q1 == AUTH_CX[10:0]);

    wire auth_tick =
        in_auth_panel &&
        (y_q1[4:0] == 5'd0) &&
        (x_q1 >= (AUTH_CX - 3)) &&
        (x_q1 <= (AUTH_CX + 3));

    wire auth_unc_band =
        in_auth_panel &&
        auth_data_ok &&
        (auth_uncertainty_cm != 16'd0) &&
        (x_q1 >= (AUTH_X0 + 2)) &&
        (x_q1 <= (AUTH_X1 - 8)) &&
        (y_q1 >= auth_unc_hi_y) &&
        (y_q1 <= auth_unc_lo_y) &&
        (x_q1[1] == y_q1[1]);

    wire auth_unc_edge =
        in_auth_panel &&
        auth_data_ok &&
        (auth_uncertainty_cm != 16'd0) &&
        (x_q1 >= (AUTH_X0 + 3)) &&
        (x_q1 <= (AUTH_X1 - 8)) &&
        ((y_q1 == auth_unc_hi_y) || (y_q1 == auth_unc_lo_y));

    wire auth_envelope_span =
        in_auth_panel &&
        auth_data_ok &&
        (x_q1 >= (AUTH_CX + 1)) &&
        (x_q1 <= (AUTH_CX + 3)) &&
        (y_q1 >= auth_env_y_top) &&
        (y_q1 <= auth_env_y_bot);

    wire auth_prediction_corridor =
        in_auth_panel &&
        auth_data_ok &&
        (x_q1 >= (AUTH_CX + 5)) &&
        (x_q1 <= (AUTH_X1 - 9)) &&
        (y_q1 >= auth_env_y_top) &&
        (y_q1 <= auth_env_y_bot) &&
        ((x_q1[2:0] == y_q1[2:0]) || (y_q1[4:0] == 5'd0));

    wire auth_target_marker =
        in_auth_panel &&
        (x_q1 >= (AUTH_X0 + 3)) &&
        (x_q1 <= (AUTH_X1 - 8)) &&
        (y_q1 == auth_target_y);

    wire auth_current_marker =
        in_auth_panel &&
        auth_data_ok &&
        (x_q1 >= (AUTH_X0 + 4)) &&
        (x_q1 <= (AUTH_CX - 2)) &&
        ((y_q1 == auth_current_y) ||
         (y_q1 + 11'd1 == auth_current_y) ||
         (y_q1 == auth_current_y + 11'd1));

    wire auth_no_marker =
        in_auth_panel &&
        auth_data_ok &&
        (x_q1 >= (AUTH_CX + 4)) &&
        (x_q1 <= (AUTH_X1 - 9)) &&
        ((y_q1 == auth_no_y) ||
         (y_q1 + 11'd1 == auth_no_y) ||
         (y_q1 == auth_no_y + 11'd1));

    wire auth_full_marker =
        in_auth_panel &&
        auth_data_ok &&
        (x_q1 >= (AUTH_X0 + 5)) &&
        (x_q1 <= (AUTH_CX - 2)) &&
        ((y_q1 == auth_full_y) ||
         (y_q1 + 11'd1 == auth_full_y) ||
         (y_q1 == auth_full_y + 11'd1));

    wire auth_cmd_frame =
        in_auth_panel &&
        (x_q1 >= AUTH_CMD_X0[10:0]) &&
        (x_q1 <= AUTH_CMD_X1[10:0]) &&
        ((x_q1 == AUTH_CMD_X0[10:0]) ||
         (x_q1 == AUTH_CMD_X1[10:0]) ||
         (y_q1 == AUTH_Y0[10:0]) ||
         (y_q1 == AUTH_Y1[10:0]));

    wire auth_cmd_fill =
        in_auth_panel &&
        auth_data_ok &&
        (auth_cmd_level != 3'd0) &&
        (x_q1 > AUTH_CMD_X0[10:0]) &&
        (x_q1 < AUTH_CMD_X1[10:0]) &&
        (y_q1 >= auth_cmd_top_y) &&
        (y_q1 < AUTH_Y1[10:0]);

    wire auth_quality_hatch =
        in_auth_panel &&
        (!auth_data_ok) &&
        (x_q1[3:0] == y_q1[3:0]);

    wire auth_phase_strip =
        in_auth_panel &&
        (x_q1 >= (AUTH_X0 + 2)) &&
        (x_q1 <= (AUTH_X1 - 9)) &&
        (y_q1 >= (AUTH_Y0 + 2)) &&
        (y_q1 <= (AUTH_Y0 + 5));

    wire auth_phase_source_mark =
        auth_phase_strip &&
        ((auth_external_phase_valid && (x_q1[1:0] == 2'd0)) ||
         (auth_local_phase_used && (x_q1[1:0] == y_q1[1:0])));

    wire auth_gate_strip =
        in_auth_panel &&
        (x_q1 >= (AUTH_X0 + 2)) &&
        (x_q1 <= (AUTH_X0 + 18)) &&
        (y_q1 >= (AUTH_Y1 - 7)) &&
        (y_q1 <= (AUTH_Y1 - 3));

    wire auth_gate_cell_region =
        in_auth_panel &&
        (x_q1 >= (AUTH_X0 + 2)) &&
        (x_q1 <= (AUTH_X0 + 20)) &&
        (y_q1 >= (AUTH_Y1 - 8)) &&
        (y_q1 <= (AUTH_Y1 - 2));

    wire [10:0] auth_gate_cell_rel_x = x_q1 - (AUTH_X0 + 2);
    wire [2:0]  auth_gate_cell_idx   = auth_gate_cell_rel_x[4:2];
    wire        auth_gate_cell_gap   = (auth_gate_cell_rel_x[1:0] == 2'd3);
    wire        auth_gate_cell =
        auth_gate_cell_region &&
        (auth_gate_cell_idx <= 3'd4) &&
        !auth_gate_cell_gap;

    wire auth_gate_cell_enabled =
        (auth_gate_cell_idx == 3'd0) ? auth_safety_runtime_ok :
        (auth_gate_cell_idx == 3'd1) ? auth_safety_allows_actuation :
        (auth_gate_cell_idx == 3'd2) ? auth_policy_runtime_enable :
        (auth_gate_cell_idx == 3'd3) ? auth_software_armed :
        (auth_gate_cell_idx == 3'd4) ? auth_flags[`VIZ_AUTH_FLG_ACT_SAFE_BIT] :
                                       1'b0;

    wire [11:0] auth_gate_cell_color =
        (!auth_data_ok) ? rgb(4'h7, 4'h7, 4'h7) :
        auth_gate_cell_enabled ? rgb(4'h2, 4'hF, 4'h6) :
        auth_policy_demands_actuation ? rgb(4'hF, 4'h2, 4'h0) :
                                        rgb(4'hF, 4'h8, 4'h0);

    //==========================================================================
    // Geometric instrumentation reconstructed from derived angles
    //--------------------------------------------------------------------------
    // The mask, dot-product, and cross-product terms are registered from the
    // q0 pixel stage and consumed in q1. Horizon/compass containment uses a
    // fixed octagonal metric instead of per-pixel radius squares so the Basys-3
    // build does not spend LUT fabric on four small multipliers.
    //==========================================================================
    wire [2:0] roll_oct = octant_from_s_mdeg(der_roll_mdeg);
    wire [2:0] head_oct = octant_from_u_mdeg(der_heading_mdeg);

    wire signed [2:0] hzn_nx = dir_x_from_oct(roll_oct);
    wire signed [2:0] hzn_ny = dir_y_from_oct(roll_oct);

    wire signed [2:0] cmp_fx = dir_x_from_oct(head_oct);
    wire signed [2:0] cmp_fy = dir_y_from_oct(head_oct);

    wire signed [12:0] hzn_dx_q0_w = $signed({2'b00, x_q0}) - HZN_CX_S;
    wire signed [12:0] hzn_dy_q0_w = $signed({2'b00, y_q0}) - HZN_CY_S;
    wire signed [12:0] cmp_dx_q0_w = $signed({2'b00, x_q0}) - CMP_CX_S;
    wire signed [12:0] cmp_dy_q0_w = $signed({2'b00, y_q0}) - CMP_CY_S;

    reg signed [12:0] hzn_dx;
    reg signed [12:0] hzn_dy;
    reg signed [12:0] cmp_dx;
    reg signed [12:0] cmp_dy;

    reg [12:0] hzn_abs_dx_q;
    reg [12:0] hzn_abs_dy_q;
    reg [12:0] cmp_abs_dx_q;
    reg [12:0] cmp_abs_dy_q;

    reg signed [15:0] hzn_eval_x_q;
    reg signed [15:0] hzn_eval_y_q;
    reg signed [15:0] cmp_cross_x_q;
    reg signed [15:0] cmp_cross_y_q;
    reg signed [15:0] cmp_dot_x_q;
    reg signed [15:0] cmp_dot_y_q;

    always @(posedge pix_clk) begin
        if (pix_rst) begin
            hzn_dx        <= 13'sd0;
            hzn_dy        <= 13'sd0;
            cmp_dx        <= 13'sd0;
            cmp_dy        <= 13'sd0;
            hzn_abs_dx_q  <= 13'd0;
            hzn_abs_dy_q  <= 13'd0;
            cmp_abs_dx_q  <= 13'd0;
            cmp_abs_dy_q  <= 13'd0;
            hzn_eval_x_q  <= 16'sd0;
            hzn_eval_y_q  <= 16'sd0;
            cmp_cross_x_q <= 16'sd0;
            cmp_cross_y_q <= 16'sd0;
            cmp_dot_x_q   <= 16'sd0;
            cmp_dot_y_q   <= 16'sd0;
        end else begin
            hzn_dx        <= hzn_dx_q0_w;
            hzn_dy        <= hzn_dy_q0_w;
            cmp_dx        <= cmp_dx_q0_w;
            cmp_dy        <= cmp_dy_q0_w;
            hzn_abs_dx_q  <= abs13u(hzn_dx_q0_w);
            hzn_abs_dy_q  <= abs13u(hzn_dy_q0_w);
            cmp_abs_dx_q  <= abs13u(cmp_dx_q0_w);
            cmp_abs_dy_q  <= abs13u(cmp_dy_q0_w);
            hzn_eval_x_q  <= hzn_nx * hzn_dx_q0_w;
            hzn_eval_y_q  <= hzn_ny * hzn_dy_q0_w;
            cmp_cross_x_q <= cmp_fx * cmp_dy_q0_w;
            cmp_cross_y_q <= cmp_fy * cmp_dx_q0_w;
            cmp_dot_x_q   <= cmp_fx * cmp_dx_q0_w;
            cmp_dot_y_q   <= cmp_fy * cmp_dy_q0_w;
        end
    end

    wire [13:0] hzn_oct_x = {1'b0, hzn_abs_dx_q} + {2'b00, hzn_abs_dy_q[12:1]};
    wire [13:0] hzn_oct_y = {1'b0, hzn_abs_dy_q} + {2'b00, hzn_abs_dx_q[12:1]};
    wire [13:0] cmp_oct_x = {1'b0, cmp_abs_dx_q} + {2'b00, cmp_abs_dy_q[12:1]};
    wire [13:0] cmp_oct_y = {1'b0, cmp_abs_dy_q} + {2'b00, cmp_abs_dx_q[12:1]};

    wire hzn_in_disc =
        in_col1 &&
        (hzn_oct_x <= HZN_R) &&
        (hzn_oct_y <= HZN_R);

    wire cmp_in_disc =
        in_col2 &&
        (cmp_oct_x <= CMP_R) &&
        (cmp_oct_y <= CMP_R);

    wire hzn_ring =
        hzn_in_disc &&
        ((hzn_oct_x >= (HZN_R-2)) ||
         (hzn_oct_y >= (HZN_R-2)));

    wire cmp_ring =
        cmp_in_disc &&
        ((cmp_oct_x >= (CMP_R-2)) ||
         (cmp_oct_y >= (CMP_R-2)));

    wire signed [15:0] hzn_eval = hzn_eval_x_q + hzn_eval_y_q;
    wire [15:0] hzn_eval_abs = abs16u(hzn_eval);

    wire hzn_line_on  = hzn_in_disc && (hzn_eval_abs <= 16'd2);
    wire hzn_sky_side = hzn_eval[15];

    wire roll_ref_tick =
        hzn_in_disc &&
        (abs16u(hzn_dx) <= 16'd2) &&
        (hzn_dy >= -HZN_R_S) &&
        (hzn_dy <= -HZN_RM12_S);

    wire hzn_aircraft_wings =
        hzn_in_disc &&
        (abs16u(hzn_dy) <= 16'd1) &&
        (((hzn_dx >= -13'sd46) && (hzn_dx <= -13'sd10)) ||
         ((hzn_dx >=  13'sd10) && (hzn_dx <=  13'sd46)));

    wire hzn_aircraft_stem =
        hzn_in_disc &&
        (abs16u(hzn_dx) <= 16'd1) &&
        (hzn_dy >= -13'sd8) &&
        (hzn_dy <=  13'sd14);

    wire hzn_center_dot =
        hzn_in_disc &&
        (abs16u(hzn_dx) <= 16'd3) &&
        (abs16u(hzn_dy) <= 16'd3);

    wire hzn_quality_hatch =
        hzn_in_disc &&
        (!roll_quality_ok) &&
        (x_q1[4:0] == y_q1[4:0]);

    //--------------------------------------------------------------------------
    // Column 1: explicit roll-plane vertical vector instrument
    //
    // This panel uses the same committed roll octant as the artificial horizon,
    // but renders the inferred body/world vertical projection as a direct vector
    // rather than only as a horizon line. It is intentionally pitch-neutral
    // because the current visualization bundle publishes roll and heading, not a
    // full Euler attitude.
    //--------------------------------------------------------------------------
    wire in_vec_panel =
        in_col1 &&
        (x_q1 >= VEC_X0[10:0]) && (x_q1 <= VEC_X1[10:0]) &&
        (y_q1 >= VEC_Y0[10:0]) && (y_q1 <= VEC_Y1[10:0]);

    wire vec_panel_border =
        in_vec_panel &&
        ((x_q1 == VEC_X0[10:0]) || (x_q1 == VEC_X1[10:0]) ||
         (y_q1 == VEC_Y0[10:0]) || (y_q1 == VEC_Y1[10:0]));

    wire signed [12:0] vec_dx = $signed({2'b00, x_q1}) - VEC_CX_S;
    wire signed [12:0] vec_dy = $signed({2'b00, y_q1}) - VEC_CY_S;
    wire signed [15:0] vec_dot = dot_from_oct(roll_oct, vec_dx, vec_dy);
    wire signed [15:0] vec_cross = cross_from_oct(roll_oct, vec_dx, vec_dy);
    wire [15:0] vec_cross_abs = abs16u(vec_cross);

    wire vec_axis =
        in_vec_panel &&
        ((abs16u(vec_dx) <= 16'd1) || (abs16u(vec_dy) <= 16'd1));

    wire vec_vector_tail =
        in_vec_panel &&
        (vec_dot >= -16'sd7) &&
        (vec_dot <=  16'sd0) &&
        (vec_cross_abs <= 16'd1);

    wire vec_vector_core =
        in_vec_panel &&
        (vec_dot >= 16'sd0) &&
        (vec_dot <= VEC_DOT_MAX_S) &&
        (vec_cross_abs <= 16'd2);

    wire vec_vector_tip =
        in_vec_panel &&
        (vec_dot >= VEC_DOT_TIP_LO_S) &&
        (vec_dot <= VEC_DOT_TIP_HI_S) &&
        (vec_cross_abs <= 16'd5);

    wire vec_center_dot =
        in_vec_panel &&
        (abs16u(vec_dx) <= 16'd2) &&
        (abs16u(vec_dy) <= 16'd2);

    wire vec_quality_hatch =
        in_vec_panel &&
        (!roll_quality_ok) &&
        (x_q1[3:0] == y_q1[3:0]);

    wire signed [15:0] cmp_cross = cmp_cross_x_q - cmp_cross_y_q;
    wire signed [15:0] cmp_dot   = cmp_dot_x_q + cmp_dot_y_q;

    wire [15:0] cmp_cross_abs = abs16u(cmp_cross);

    wire cmp_needle_core =
        cmp_in_disc &&
        (cmp_cross_abs <= 16'd2) &&
        (cmp_dot >= -16'sd80) &&
        (cmp_dot <=  16'sd80);

    wire cmp_needle_tip =
        cmp_in_disc &&
        (cmp_cross_abs <= 16'd4) &&
        (cmp_dot >= 16'sd68);

    wire cmp_ns_axis = cmp_in_disc && (abs16u(cmp_dx) <= 16'd1);
    wire cmp_ew_axis = cmp_in_disc && (abs16u(cmp_dy) <= 16'd1);

    wire cmp_cardinal_tick =
        cmp_in_disc &&
        (((abs16u(cmp_dx) <= 16'd2) &&
          ((cmp_dy <= -13'sd68) || (cmp_dy >= 13'sd68))) ||
         ((abs16u(cmp_dy) <= 16'd2) &&
          ((cmp_dx <= -13'sd68) || (cmp_dx >= 13'sd68))));

    wire cmp_center_dot =
        cmp_in_disc &&
        (abs16u(cmp_dx) <= 16'd2) &&
        (abs16u(cmp_dy) <= 16'd2);

    wire cmp_quality_hatch =
        cmp_in_disc &&
        (!(der_valid && (der_status == 8'h00) && der_head_fresh && der_mag_valid_ref)) &&
        (x_q1[4:0] == y_q1[4:0]);

    //--------------------------------------------------------------------------
    // Column 2: landing dispersion / wind estimator viewport
    //
    // This panel deliberately consumes only committed bundle fields. If the
    // top-level design has not wired a real navigation or wind producer yet, the
    // viewport renders a degraded hatch instead of silently reusing fixed demo
    // vectors or assuming zero crossrange.
    //--------------------------------------------------------------------------
    wire nav_data_ok =
        nav_valid &&
        (nav_status == 8'h00) &&
        (nav_age_ms < 16'd1000);

    wire wind_data_ok =
        wind_valid &&
        (wind_status == 8'h00) &&
        (wind_age_ms < 16'd1000);

    wire in_landing_panel =
        in_col2 &&
        (x_q1 >= LAND_X0[10:0]) && (x_q1 <= LAND_X1[10:0]) &&
        (y_q1 >= LAND_Y0[10:0]) && (y_q1 <= LAND_Y1[10:0]);

    wire landing_panel_border =
        in_landing_panel &&
        ((x_q1 == LAND_X0[10:0]) || (x_q1 == LAND_X1[10:0]) ||
         (y_q1 == LAND_Y0[10:0]) || (y_q1 == LAND_Y1[10:0]));

    wire signed [12:0] landing_dx = $signed({2'b00, x_q1}) - LAND_CX_S;
    wire signed [12:0] landing_dy = $signed({2'b00, y_q1}) - LAND_CY_S;

    wire signed [12:0] nav_cross_px =
        clip_s13(nav_crossrange_m >>> 1, -16'sd90, 16'sd90);
    wire signed [12:0] nav_down_px =
        clip_s13(-(nav_downrange_m >>> 2), -16'sd20, 16'sd20);

    wire signed [12:0] wind_x_px =
        clip_s13(wind_x_cms >>> 5, -16'sd30, 16'sd30);
    wire signed [12:0] wind_y_px =
        clip_s13(-(wind_y_cms >>> 5), -16'sd20, 16'sd20);
    wire signed [12:0] wind_z_px =
        clip_s13(-(wind_z_cms >>> 5), -16'sd20, 16'sd20);

    wire signed [12:0] landing_nav_err_x = landing_dx - nav_cross_px;
    wire signed [12:0] landing_nav_err_y = landing_dy - nav_down_px;
    wire signed [12:0] landing_wind_x_err = landing_dx - wind_x_px;
    wire signed [12:0] landing_wind_y_err = landing_dy - wind_y_px;

    wire landing_axis =
        in_landing_panel &&
        ((abs13u(landing_dx) <= 13'd1) || (abs13u(landing_dy) <= 13'd1));

    wire landing_nav_uncertainty =
        in_landing_panel &&
        nav_data_ok &&
        (abs13u(landing_nav_err_x) <= 13'd10) &&
        (abs13u(landing_nav_err_y) <= 13'd5) &&
        (x_q1[0] ^ y_q1[0]);

    wire landing_nav_point =
        in_landing_panel &&
        nav_data_ok &&
        (abs13u(landing_nav_err_x) <= 13'd2) &&
        (abs13u(landing_nav_err_y) <= 13'd2);

    wire landing_wind_x_leg =
        in_landing_panel &&
        wind_data_ok &&
        (abs13u(landing_dy) <= 13'd1) &&
        between_s13(landing_dx, 13'sd0, wind_x_px);

    wire landing_wind_y_leg =
        in_landing_panel &&
        wind_data_ok &&
        (abs13u(landing_dx - wind_x_px) <= 13'd1) &&
        between_s13(landing_dy, 13'sd0, wind_y_px);

    wire landing_wind_tip =
        in_landing_panel &&
        wind_data_ok &&
        (abs13u(landing_wind_x_err) <= 13'd2) &&
        (abs13u(landing_wind_y_err) <= 13'd2);

    wire landing_wind_z_bar =
        in_landing_panel &&
        wind_data_ok &&
        (x_q1 >= (LAND_X1 - 6)) &&
        (x_q1 <= (LAND_X1 - 4)) &&
        between_s13(landing_dy, 13'sd0, wind_z_px);

    wire landing_quality_hatch =
        in_landing_panel &&
        (!nav_data_ok || !wind_data_ok) &&
        (x_q1[3:0] == y_q1[3:0]);

    //--------------------------------------------------------------------------
    // Page 1: sensor evidence, extension scales, and multi-frame nav/wind trail
    //--------------------------------------------------------------------------
    wire signed [12:0] diag_trail_dx =
        $signed({2'b00, x_q1}) - DIAG_TRAIL_CX_S;
    wire signed [12:0] diag_trail_dy =
        $signed({2'b00, y_q1}) - DIAG_TRAIL_CY_S;

    wire signed [12:0] diag_nav_cross_px =
        clip_s13(nav_crossrange_m, -16'sd130, 16'sd130);
    wire signed [12:0] diag_nav_down_px =
        clip_s13(-(nav_downrange_m >>> 1), -16'sd84, 16'sd84);
    wire signed [12:0] diag_wind_x_px =
        clip_s13(wind_x_cms >>> 4, -16'sd120, 16'sd120);
    wire signed [12:0] diag_wind_y_px =
        clip_s13(-(wind_y_cms >>> 4), -16'sd84, 16'sd84);

    wire [10:0] diag_nav_x_now =
        offset_coord_s13(DIAG_TRAIL_CX, diag_nav_cross_px,
                         DIAG_TRAIL_X0 + 4, DIAG_TRAIL_X1 - 4);
    wire [10:0] diag_nav_y_now =
        offset_coord_s13(DIAG_TRAIL_CY, diag_nav_down_px,
                         DIAG_TRAIL_Y0 + 4, DIAG_TRAIL_Y1 - 4);
    wire [10:0] diag_wind_x_now =
        offset_coord_s13(DIAG_TRAIL_CX, diag_wind_x_px,
                         DIAG_TRAIL_X0 + 4, DIAG_TRAIL_X1 - 4);
    wire [10:0] diag_wind_y_now =
        offset_coord_s13(DIAG_TRAIL_CY, diag_wind_y_px,
                         DIAG_TRAIL_Y0 + 4, DIAG_TRAIL_Y1 - 4);

    reg [10:0] diag_nav_x_hist [0:DIAG_TRAIL_DEPTH-1];
    reg [10:0] diag_nav_y_hist [0:DIAG_TRAIL_DEPTH-1];
    reg [10:0] diag_wind_x_hist [0:DIAG_TRAIL_DEPTH-1];
    reg [10:0] diag_wind_y_hist [0:DIAG_TRAIL_DEPTH-1];
    reg [3:0]  diag_trail_wr_ptr;
    integer diag_trail_i;

    always @(posedge pix_clk) begin
        if (pix_rst) begin
            diag_trail_wr_ptr <= 4'd0;
            for (diag_trail_i = 0; diag_trail_i < DIAG_TRAIL_DEPTH;
                 diag_trail_i = diag_trail_i + 1) begin
                diag_nav_x_hist[diag_trail_i]  <= DIAG_TRAIL_CX[10:0];
                diag_nav_y_hist[diag_trail_i]  <= DIAG_TRAIL_CY[10:0];
                diag_wind_x_hist[diag_trail_i] <= DIAG_TRAIL_CX[10:0];
                diag_wind_y_hist[diag_trail_i] <= DIAG_TRAIL_CY[10:0];
            end
        end else if (vsync_edge && (nav_data_ok || wind_data_ok)) begin
            diag_nav_x_hist[diag_trail_wr_ptr] <=
                nav_data_ok ? diag_nav_x_now : DIAG_TRAIL_CX[10:0];
            diag_nav_y_hist[diag_trail_wr_ptr] <=
                nav_data_ok ? diag_nav_y_now : DIAG_TRAIL_CY[10:0];
            diag_wind_x_hist[diag_trail_wr_ptr] <=
                wind_data_ok ? diag_wind_x_now : DIAG_TRAIL_CX[10:0];
            diag_wind_y_hist[diag_trail_wr_ptr] <=
                wind_data_ok ? diag_wind_y_now : DIAG_TRAIL_CY[10:0];
            diag_trail_wr_ptr <= diag_trail_wr_ptr + 4'd1;
        end
    end

    reg diag_nav_trail_hit;
    reg diag_wind_trail_hit;
    integer diag_trail_j;
    always @(*) begin
        diag_nav_trail_hit = 1'b0;
        diag_wind_trail_hit = 1'b0;
        for (diag_trail_j = 0; diag_trail_j < DIAG_TRAIL_DEPTH;
             diag_trail_j = diag_trail_j + 1) begin
            if ((x_q1 + 11'd1 >= diag_nav_x_hist[diag_trail_j]) &&
                (x_q1 <= diag_nav_x_hist[diag_trail_j] + 11'd1) &&
                (y_q1 + 11'd1 >= diag_nav_y_hist[diag_trail_j]) &&
                (y_q1 <= diag_nav_y_hist[diag_trail_j] + 11'd1))
                diag_nav_trail_hit = 1'b1;

            if ((x_q1 + 11'd1 >= diag_wind_x_hist[diag_trail_j]) &&
                (x_q1 <= diag_wind_x_hist[diag_trail_j] + 11'd1) &&
                (y_q1 + 11'd1 >= diag_wind_y_hist[diag_trail_j]) &&
                (y_q1 <= diag_wind_y_hist[diag_trail_j] + 11'd1))
                diag_wind_trail_hit = 1'b1;
        end
    end

    wire [16:0] ext_flow_mag_sum =
        {1'b0, abs16u(ext_flow_dx)} + {1'b0, abs16u(ext_flow_dy)};
    wire [15:0] ext_flow_mag =
        ext_flow_mag_sum[16] ? 16'hFFFF : ext_flow_mag_sum[15:0];

    reg [3:0]  diag_row_id;
    reg        diag_row_hit;
    reg        diag_row_valid;
    reg        diag_row_fresh;
    reg        diag_row_fault;
    reg [7:0]  diag_row_status;
    reg [15:0] diag_row_age;
    reg [15:0] diag_row_stale_ms;
    reg [10:0] diag_row_age_w;

    always @(*) begin
        diag_row_id       = 4'hF;
        diag_row_hit      = 1'b0;
        diag_row_valid    = 1'b0;
        diag_row_fresh    = 1'b0;
        diag_row_fault    = 1'b0;
        diag_row_status   = 8'hFF;
        diag_row_age      = 16'hFFFF;
        diag_row_stale_ms = 16'd1000;

        if ((y_q1 >= DIAG_MATRIX_Y0[10:0]) &&
            (y_q1 <= (DIAG_MATRIX_Y0 + DIAG_ROW_H - 1))) begin
            diag_row_id       = 4'd0;
            diag_row_hit      = 1'b1;
            diag_row_valid    = bmp_vld;
            diag_row_status   = bmp_st;
            diag_row_age      = bmp_age_ms;
            diag_row_stale_ms = BMP_STALE_MS;
            diag_row_fresh    = (bmp_age_ms < BMP_STALE_MS);
        end else if ((y_q1 >= (DIAG_MATRIX_Y0 + DIAG_ROW_H)) &&
                     (y_q1 <= (DIAG_MATRIX_Y0 + (DIAG_ROW_H * 2) - 1))) begin
            diag_row_id       = 4'd1;
            diag_row_hit      = 1'b1;
            diag_row_valid    = acc_vld;
            diag_row_status   = acc_st;
            diag_row_age      = acc_age_ms;
            diag_row_stale_ms = ACC_STALE_MS;
            diag_row_fresh    = (acc_age_ms < ACC_STALE_MS);
        end else if ((y_q1 >= (DIAG_MATRIX_Y0 + (DIAG_ROW_H * 2))) &&
                     (y_q1 <= (DIAG_MATRIX_Y0 + (DIAG_ROW_H * 3) - 1))) begin
            diag_row_id       = 4'd2;
            diag_row_hit      = 1'b1;
            diag_row_valid    = mag_vld;
            diag_row_status   = mag_st;
            diag_row_age      = mag_age_ms;
            diag_row_stale_ms = MAG_STALE_MS;
            diag_row_fresh    = (mag_age_ms < MAG_STALE_MS);
        end else if ((y_q1 >= (DIAG_MATRIX_Y0 + (DIAG_ROW_H * 3))) &&
                     (y_q1 <= (DIAG_MATRIX_Y0 + (DIAG_ROW_H * 4) - 1))) begin
            diag_row_id       = 4'd3;
            diag_row_hit      = 1'b1;
            diag_row_valid    = pwr_valid;
            diag_row_status   = pwr_status;
            diag_row_age      = pwr_age_ms;
            diag_row_stale_ms = 16'd1000;
            diag_row_fresh    = (pwr_age_ms < 16'd1000);
            diag_row_fault    = (pwr_alert != 8'd0);
        end else if ((y_q1 >= (DIAG_MATRIX_Y0 + (DIAG_ROW_H * 4))) &&
                     (y_q1 <= (DIAG_MATRIX_Y0 + (DIAG_ROW_H * 5) - 1))) begin
            diag_row_id       = 4'd4;
            diag_row_hit      = 1'b1;
            diag_row_valid    = der_valid;
            diag_row_status   = der_status;
            diag_row_age      = der_bmp_age_ms;
            diag_row_stale_ms = 16'd1000;
            diag_row_fresh    = der_alt_fresh && der_vspd_fresh &&
                                der_roll_fresh && der_head_fresh;
            diag_row_fault    = !(der_bmp_valid_ref && der_acc_valid_ref &&
                                  der_mag_valid_ref);
        end else if ((y_q1 >= (DIAG_MATRIX_Y0 + (DIAG_ROW_H * 5))) &&
                     (y_q1 <= (DIAG_MATRIX_Y0 + (DIAG_ROW_H * 6) - 1))) begin
            diag_row_id       = 4'd5;
            diag_row_hit      = 1'b1;
            diag_row_valid    = ext_valid;
            diag_row_status   = ext_status;
            diag_row_age      = ext_max_age_ms;
            diag_row_stale_ms = 16'd1000;
            diag_row_fresh    = (ext_max_age_ms < 16'd1000);
            diag_row_fault    = (ext_fault_flags != 16'd0);
        end else if ((y_q1 >= (DIAG_MATRIX_Y0 + (DIAG_ROW_H * 6))) &&
                     (y_q1 <= (DIAG_MATRIX_Y0 + (DIAG_ROW_H * 7) - 1))) begin
            diag_row_id       = 4'd6;
            diag_row_hit      = 1'b1;
            diag_row_valid    = nav_valid;
            diag_row_status   = nav_status;
            diag_row_age      = nav_age_ms;
            diag_row_stale_ms = 16'd1000;
            diag_row_fresh    = (nav_age_ms < 16'd1000);
        end else if ((y_q1 >= (DIAG_MATRIX_Y0 + (DIAG_ROW_H * 7))) &&
                     (y_q1 <= (DIAG_MATRIX_Y0 + (DIAG_ROW_H * 8) - 1))) begin
            diag_row_id       = 4'd7;
            diag_row_hit      = 1'b1;
            diag_row_valid    = wind_valid;
            diag_row_status   = wind_status;
            diag_row_age      = wind_age_ms;
            diag_row_stale_ms = 16'd1000;
            diag_row_fresh    = (wind_age_ms < 16'd1000);
        end

        diag_row_age_w = bar_width_u16(diag_row_age, diag_row_stale_ms, 11'd116);
    end

    wire diag_row_ok =
        diag_row_valid && diag_row_fresh &&
        (diag_row_status == 8'h00) && !diag_row_fault;
    wire [11:0] diag_row_status_color =
        !diag_row_valid ? 12'h667 :
        (diag_row_status != 8'h00) || diag_row_fault ? 12'hF22 :
        !diag_row_fresh ? 12'hF80 :
                          12'h2F6;

    reg [11:0] diag_rgb;
    always @(*) begin
        diag_rgb = grid ? 12'h111 : 12'h000;

        if (y_q1 < TOP_H[10:0])
            diag_rgb = 12'h112;

        if ((x_q1 >= DIAG_MATRIX_X0[10:0]) &&
            (x_q1 <= DIAG_MATRIX_X1[10:0]) &&
            (y_q1 >= DIAG_MATRIX_Y0[10:0]) &&
            (y_q1 <= DIAG_MATRIX_Y1[10:0])) begin
            diag_rgb = 12'h011;
            if (diag_row_hit && y_q1[0])
                diag_rgb = diag_row_ok ? 12'h021 : 12'h110;
            if ((x_q1 == DIAG_MATRIX_X0[10:0]) ||
                (x_q1 == DIAG_MATRIX_X1[10:0]) ||
                (y_q1 == DIAG_MATRIX_Y0[10:0]) ||
                (y_q1 == DIAG_MATRIX_Y1[10:0]))
                diag_rgb = 12'h79A;
            if (diag_row_hit &&
                (x_q1 >= 11'd18) && (x_q1 <= 11'd58) &&
                (y_q1[2:0] != 3'd0))
                diag_rgb = diag_row_status_color;
            if (diag_row_hit &&
                (x_q1 >= 11'd76) && (x_q1 <= 11'd106) &&
                (y_q1[2:0] != 3'd0))
                diag_rgb = diag_row_valid ? 12'h2F6 : 12'h667;
            if (diag_row_hit &&
                (x_q1 >= 11'd116) && (x_q1 <= 11'd146) &&
                (y_q1[2:0] != 3'd0))
                diag_rgb = (diag_row_status == 8'h00) ? 12'h2F6 : 12'hF22;
            if (diag_row_hit &&
                (x_q1 >= 11'd158) && (x_q1 <= 11'd274) &&
                (y_q1[2:0] != 3'd0))
                diag_rgb = 12'h123;
            if (diag_row_hit &&
                (x_q1 >= 11'd158) &&
                (x_q1 < (11'd158 + diag_row_age_w)) &&
                (y_q1[2:0] != 3'd0))
                diag_rgb = diag_row_fresh ? 12'h2AF : 12'hF80;
            if (diag_row_hit &&
                (x_q1 >= 11'd284) && (x_q1 <= 11'd304) &&
                (y_q1[2:0] != 3'd0))
                diag_rgb = diag_row_fault ? 12'hF22 : 12'h2F6;
        end

        if ((x_q1 >= DIAG_SCALE_X0[10:0]) &&
            (x_q1 <= DIAG_SCALE_X1[10:0]) &&
            (y_q1 >= DIAG_SCALE_Y0[10:0]) &&
            (y_q1 <= DIAG_SCALE_Y1[10:0])) begin
            diag_rgb = 12'h010;
            if ((x_q1 == DIAG_SCALE_X0[10:0]) ||
                (x_q1 == DIAG_SCALE_X1[10:0]) ||
                (y_q1 == DIAG_SCALE_Y0[10:0]) ||
                (y_q1 == DIAG_SCALE_Y1[10:0]))
                diag_rgb = 12'h79A;
        end

        if ((x_q1 >= DIAG_BAR_X0[10:0]) &&
            (x_q1 <= DIAG_BAR_X1[10:0])) begin
            if ((y_q1 >= 11'd56) && (y_q1 < (11'd56 + DIAG_BAR_H))) begin
                diag_rgb = 12'h123;
                if (x_q1 < (DIAG_BAR_X0 + bar_width_u16(ext_rng_height_cm, 16'd500, DIAG_BAR_W_U11)))
                    diag_rgb = (ext_present_flags[`EXT_PRESENT_RANGE_BIT] &&
                                !ext_fault_flags[`EXT_FLG_RANGE_STALE_BIT]) ? 12'h2F6 : 12'hF80;
            end
            if ((y_q1 >= 11'd84) && (y_q1 < (11'd84 + DIAG_BAR_H))) begin
                diag_rgb = 12'h123;
                if (x_q1 < (DIAG_BAR_X0 + bar_width_u16(ext_air_speed_cms, 16'd5000, DIAG_BAR_W_U11)))
                    diag_rgb = (ext_present_flags[`EXT_PRESENT_AIR_BIT] &&
                                !ext_fault_flags[`EXT_FLG_AIR_STALE_BIT]) ? 12'h2AF : 12'hF80;
            end
            if ((y_q1 >= 11'd112) && (y_q1 < (11'd112 + DIAG_BAR_H))) begin
                diag_rgb = 12'h123;
                if (x_q1 < (DIAG_BAR_X0 + bar_width_u16(ext_env_temp_cdeg, 16'd6000, DIAG_BAR_W_U11)))
                    diag_rgb = (ext_present_flags[`EXT_PRESENT_ENV_BIT] &&
                                !ext_fault_flags[`EXT_FLG_ENV_STALE_BIT]) ? 12'hFC2 : 12'hF80;
            end
            if ((y_q1 >= 11'd140) && (y_q1 < (11'd140 + DIAG_BAR_H))) begin
                diag_rgb = 12'h123;
                if (x_q1 < (DIAG_BAR_X0 + bar_width_u16(ext_env_rh_centi, 16'd10000, DIAG_BAR_W_U11)))
                    diag_rgb = (ext_present_flags[`EXT_PRESENT_ENV_BIT] &&
                                !ext_fault_flags[`EXT_FLG_ENV_STALE_BIT]) ? 12'h3CF : 12'hF80;
            end
            if ((y_q1 >= 11'd168) && (y_q1 < (11'd168 + DIAG_BAR_H))) begin
                diag_rgb = 12'h123;
                if (x_q1 < (DIAG_BAR_X0 + bar_width_u16(ext_sun_luma, 16'hFFFF, DIAG_BAR_W_U11)))
                    diag_rgb = (ext_present_flags[`EXT_PRESENT_SUN_BIT] &&
                                !ext_fault_flags[`EXT_FLG_SUN_STALE_BIT]) ? 12'hFF2 : 12'hF80;
            end
            if ((y_q1 >= 11'd196) && (y_q1 < (11'd196 + DIAG_BAR_H))) begin
                diag_rgb = 12'h123;
                if (x_q1 < (DIAG_BAR_X0 + bar_width_u16(ext_flow_mag, 16'd2048, DIAG_BAR_W_U11)))
                    diag_rgb = (ext_present_flags[`EXT_PRESENT_FLOW_BIT] &&
                                !ext_fault_flags[`EXT_FLG_FLOW_STALE_BIT]) ? 12'hC4F : 12'hF80;
            end
        end

        if ((x_q1 >= DIAG_TRAIL_X0[10:0]) &&
            (x_q1 <= DIAG_TRAIL_X1[10:0]) &&
            (y_q1 >= DIAG_TRAIL_Y0[10:0]) &&
            (y_q1 <= DIAG_TRAIL_Y1[10:0])) begin
            diag_rgb = 12'h011;
            if ((abs13u(diag_trail_dx) <= 13'd1) ||
                (abs13u(diag_trail_dy) <= 13'd1))
                diag_rgb = 12'h234;
            if (diag_nav_trail_hit)
                diag_rgb = 12'h154;
            if (diag_wind_trail_hit)
                diag_rgb = 12'hFC2;
            if ((x_q1 + 11'd2 >= diag_nav_x_now) &&
                (x_q1 <= diag_nav_x_now + 11'd2) &&
                (y_q1 + 11'd2 >= diag_nav_y_now) &&
                (y_q1 <= diag_nav_y_now + 11'd2) &&
                nav_data_ok)
                diag_rgb = 12'h2F6;
            if ((x_q1 + 11'd2 >= diag_wind_x_now) &&
                (x_q1 <= diag_wind_x_now + 11'd2) &&
                (y_q1 + 11'd2 >= diag_wind_y_now) &&
                (y_q1 <= diag_wind_y_now + 11'd2) &&
                wind_data_ok)
                diag_rgb = 12'hFFF;
            if ((!nav_data_ok || !wind_data_ok) && (x_q1[3:0] == y_q1[3:0]))
                diag_rgb = !nav_data_ok ? 12'hF50 : 12'hD93;
            if ((x_q1 == DIAG_TRAIL_X0[10:0]) ||
                (x_q1 == DIAG_TRAIL_X1[10:0]) ||
                (y_q1 == DIAG_TRAIL_Y0[10:0]) ||
                (y_q1 == DIAG_TRAIL_Y1[10:0]))
                diag_rgb = 12'h79A;
        end
    end

    //==========================================================================
    // Charts
    //==========================================================================
    wire chart_alt_point  = in_chart_alt  && (y_q1 == plot_alt_y);
    wire chart_vspd_point = in_chart_vspd && (y_q1 == plot_vspd_y);
    wire chart_newest_x   = (x_q1 == 11'd639);

    wire chart_alt_ref =
        in_chart_alt &&
        ((y_q1 == CHART_ALT_Q1_Y) ||
         (y_q1 == CHART_ALT_MID_Y) ||
         (y_q1 == CHART_ALT_Q3_Y));

    wire chart_vspd_zero = in_chart_vspd && (y_q1 == CHART_VSPD_CY);

    wire chart_alt_cursor  = in_chart_alt  && chart_newest_x;
    wire chart_vspd_cursor = in_chart_vspd && chart_newest_x;

    wire chart_alt_now_halo =
        chart_alt_cursor &&
        ((y_q1 + 11'd1 == plot_alt_y) || (y_q1 == plot_alt_y + 11'd1));

    wire chart_vspd_now_halo =
        chart_vspd_cursor &&
        ((y_q1 + 11'd1 == plot_vspd_y) || (y_q1 == plot_vspd_y + 11'd1));

    //==========================================================================
    // Telemetry text overlay infrastructure
    //==========================================================================
    reg  [11:0] rgb_base_q;
    reg  [11:0] rgb_sensor_diag_q;

    wire [119:0] top_bmp_line;
    wire [119:0] top_acc_line;
    wire [119:0] top_mag_line;
    wire [23:0]  top_bmp_rgb_24;
    wire [23:0]  top_acc_rgb_24;
    wire [23:0]  top_mag_rgb_24;

    wire [143:0] left_line0;
    wire [143:0] left_line1;
    wire [143:0] left_line2;
    wire [143:0] left_line3;
    wire [143:0] left_line4;

    wire [143:0] mid_line0;
    wire [143:0] mid_line1;
    wire [143:0] mid_line2;
    wire [143:0] mid_line3;
    wire [143:0] mid_line4;

    wire [143:0] right_line0;
    wire [143:0] right_line1;
    wire [143:0] right_line2;
    wire [143:0] right_line3;
    wire [143:0] right_line4;

    wire [23:0]  left_rgb_24;
    wire [23:0]  mid_rgb_24;
    wire [23:0]  right_rgb_24;

    wire [383:0] bottom_line;
    wire [23:0]  bottom_rgb_24;

    wire         telemetry_overlay_on;
    wire [23:0]  telemetry_overlay_rgb_24;
    wire [11:0]  telemetry_overlay_rgb_12;

    reg          render_active_q;
    reg          render_hsync_q;
    reg          render_vsync_q;
    reg          render_overlay_on_q;
    reg  [1:0]   render_page_select_q;
    reg  [11:0]  render_overlay_rgb_q;

    //==========================================================================
    // Telemetry text generation/composition
    //==========================================================================
    generate
        if (ENABLE_TELEMETRY_TEXT_OVERLAY != 0) begin : gen_telem_text_overlay
            flight_viz_telemetry_textgen u_telem_text (
                .bmp_valid_pix              (bmp_vld),
                .bmp_status_pix             (bmp_st),
                .bmp_age_ms_pix             (bmp_age_ms),
                .bmp_seq_pix                (bmp_seq),

                .acc_valid_pix              (acc_vld),
                .acc_status_pix             (acc_st),
                .acc_age_ms_pix             (acc_age_ms),
                .acc_seq_pix                (acc_seq),

                .mag_valid_pix              (mag_vld),
                .mag_status_pix             (mag_st),
                .mag_age_ms_pix             (mag_age_ms),
                .mag_seq_pix                (mag_seq),

                .pwr_valid_pix              (pwr_valid),
                .pwr_status_pix             (pwr_status),
                .pwr_age_ms_pix             (pwr_age_ms),
                .pwr_seq_pix                (pwr_seq),
                .pwr_voltage_code_pix       (pwr_volt_code),
                .pwr_current_code_pix       (pwr_curr_code),
                .pwr_alert_status_pix       (pwr_alert),

                .der_valid_pix              (der_valid),
                .der_status_pix             (der_status),

                .der_alt_fresh_pix          (der_alt_fresh),
                .der_vspd_fresh_pix         (der_vspd_fresh),
                .der_roll_fresh_pix         (der_roll_fresh),
                .der_head_fresh_pix         (der_head_fresh),

                .der_bmp_seq_ref_pix        (der_bmp_seq_ref),
                .der_acc_seq_ref_pix        (der_acc_seq_ref),
                .der_mag_seq_ref_pix        (der_mag_seq_ref),

                .der_bmp_age_ms_pix         (der_bmp_age_ms),
                .der_acc_age_ms_pix         (der_acc_age_ms),
                .der_mag_age_ms_pix         (der_mag_age_ms),

                .der_bmp_valid_ref_pix      (der_bmp_valid_ref),
                .der_acc_valid_ref_pix      (der_acc_valid_ref),
                .der_mag_valid_ref_pix      (der_mag_valid_ref),

                .der_altitude_cm_pix        (der_altitude_cm),
                .der_vertical_speed_cms_pix (der_vertical_speed_cms),
                .der_roll_mdeg_pix          (der_roll_mdeg),
                .der_heading_mdeg_pix       (der_heading_mdeg),

                .i2c_nack_count_pix         (i2c_nack_count),
                .i2c_timeout_count_pix      (i2c_timeout_count),
                .txn_rate_hz_pix            (txn_rate_hz),
                .cdc_update_count_pix       (cdc_update_count),
                .frame_count_pix            (frame_ctr),
                .build_id_pix               (build_id),
                .schema_word_pix            (schema_word),

                .top_bmp_line               (top_bmp_line),
                .top_acc_line               (top_acc_line),
                .top_mag_line               (top_mag_line),

                .top_bmp_rgb                (top_bmp_rgb_24),
                .top_acc_rgb                (top_acc_rgb_24),
                .top_mag_rgb                (top_mag_rgb_24),

                .left_line0                 (left_line0),
                .left_line1                 (left_line1),
                .left_line2                 (left_line2),
                .left_line3                 (left_line3),
                .left_line4                 (left_line4),

                .mid_line0                  (mid_line0),
                .mid_line1                  (mid_line1),
                .mid_line2                  (mid_line2),
                .mid_line3                  (mid_line3),
                .mid_line4                  (mid_line4),

                .right_line0                (right_line0),
                .right_line1                (right_line1),
                .right_line2                (right_line2),
                .right_line3                (right_line3),
                .right_line4                (right_line4),

                .left_rgb                   (left_rgb_24),
                .mid_rgb                    (mid_rgb_24),
                .right_rgb                  (right_rgb_24),

                .bottom_line                (bottom_line),
                .bottom_rgb                 (bottom_rgb_24)
            );

            flight_viz_telemetry_compositor #(
                .GLYPH_SCALE(2),
                .CHAR_ADV_X (7)
            ) u_telem_comp (
                .clk_pix      (pix_clk),
                .rst_pix      (pix_rst),
                .hcount       (x_q1[9:0]),
                .vcount       (y_q1[9:0]),
                .active_video (active_q1),

                .top_bmp_line (top_bmp_line),
                .top_acc_line (top_acc_line),
                .top_mag_line (top_mag_line),

                .top_bmp_rgb  (top_bmp_rgb_24),
                .top_acc_rgb  (top_acc_rgb_24),
                .top_mag_rgb  (top_mag_rgb_24),

                .left_line0   (left_line0),
                .left_line1   (left_line1),
                .left_line2   (left_line2),
                .left_line3   (left_line3),
                .left_line4   (left_line4),

                .mid_line0    (mid_line0),
                .mid_line1    (mid_line1),
                .mid_line2    (mid_line2),
                .mid_line3    (mid_line3),
                .mid_line4    (mid_line4),

                .right_line0  (right_line0),
                .right_line1  (right_line1),
                .right_line2  (right_line2),
                .right_line3  (right_line3),
                .right_line4  (right_line4),

                .left_rgb     (left_rgb_24),
                .mid_rgb      (mid_rgb_24),
                .right_rgb    (right_rgb_24),

                .bottom_line  (bottom_line),
                .bottom_rgb   (bottom_rgb_24),

                .overlay_on   (telemetry_overlay_on),
                .overlay_rgb  (telemetry_overlay_rgb_24)
            );
        end else begin : gen_no_telem_text_overlay
            assign top_bmp_line            = 120'd0;
            assign top_acc_line            = 120'd0;
            assign top_mag_line            = 120'd0;
            assign top_bmp_rgb_24          = 24'd0;
            assign top_acc_rgb_24          = 24'd0;
            assign top_mag_rgb_24          = 24'd0;
            assign left_line0              = 144'd0;
            assign left_line1              = 144'd0;
            assign left_line2              = 144'd0;
            assign left_line3              = 144'd0;
            assign left_line4              = 144'd0;
            assign mid_line0               = 144'd0;
            assign mid_line1               = 144'd0;
            assign mid_line2               = 144'd0;
            assign mid_line3               = 144'd0;
            assign mid_line4               = 144'd0;
            assign right_line0             = 144'd0;
            assign right_line1             = 144'd0;
            assign right_line2             = 144'd0;
            assign right_line3             = 144'd0;
            assign right_line4             = 144'd0;
            assign left_rgb_24             = 24'd0;
            assign mid_rgb_24              = 24'd0;
            assign right_rgb_24            = 24'd0;
            assign bottom_line             = 384'd0;
            assign bottom_rgb_24           = 24'd0;
            assign telemetry_overlay_on    = 1'b0;
            assign telemetry_overlay_rgb_24 = 24'd0;
        end
    endgenerate

    //--------------------------------------------------------------------------
    // Frozen 24-bit -> 12-bit VGA packing
    //--------------------------------------------------------------------------
    assign telemetry_overlay_rgb_12 = {
        telemetry_overlay_rgb_24[23:20],
        telemetry_overlay_rgb_24[15:12],
        telemetry_overlay_rgb_24[7:4]
    };

    //==========================================================================
    // Base scene renderer
    //==========================================================================
    always @(posedge pix_clk) begin
        if (pix_rst) begin
            rgb_base_q           <= 12'h000;
            rgb_sensor_diag_q    <= 12'h000;
            render_active_q      <= 1'b0;
            render_hsync_q       <= HSYNC_IDLE;
            render_vsync_q       <= VSYNC_IDLE;
            render_overlay_on_q  <= 1'b0;
            render_page_select_q <= VGA_PAGE_HUD;
            render_overlay_rgb_q <= 12'h000;
        end else begin
            render_active_q      <= active_q1;
            render_hsync_q       <= hsync_q1;
            render_vsync_q       <= vsync_q1;
            render_overlay_on_q  <= telemetry_overlay_on;
            render_page_select_q <= (ENABLE_SENSOR_DIAG_PAGE != 0) ?
                                    vga_page_select_pix : VGA_PAGE_HUD;
            render_overlay_rgb_q <= telemetry_overlay_rgb_12;

            if (!active_q1) begin
                rgb_base_q        <= 12'h000;
                rgb_sensor_diag_q <= 12'h000;
            end else begin
                rgb_base_q        <= grid ? 12'h111 : 12'h000;
                rgb_sensor_diag_q <= (ENABLE_SENSOR_DIAG_PAGE != 0) ?
                                     diag_rgb : 12'h000;

                //==============================================================
                // Top health band
                //==============================================================
                if (in_top)
                    rgb_base_q <= 12'h111;

                if (bmp_blk &&
                    (x_q1 > 11'd16) && (x_q1 < 11'd204) &&
                    (y_q1 > 11'd3)  && (y_q1 < 11'd18))
                    rgb_base_q <= hc_bmp;

                if (acc_blk &&
                    (x_q1 > 11'd222) && (x_q1 < 11'd412) &&
                    (y_q1 > 11'd3)   && (y_q1 < 11'd18))
                    rgb_base_q <= hc_acc;

                if (mag_blk &&
                    (x_q1 > 11'd428) && (x_q1 < 11'd620) &&
                    (y_q1 > 11'd3)   && (y_q1 < 11'd18))
                    rgb_base_q <= hc_mag;

                if (bmp_bar || acc_bar || mag_bar)
                    rgb_base_q <= 12'hFFF;

                //==============================================================
                // Column 0: tapes
                //==============================================================
                if (in_alt_tape || in_vspd_tape) begin
                    rgb_base_q <= 12'h011;
                    if (tape_tick)
                        rgb_base_q <= 12'h033;
                end

                if (alt_tape_border || vspd_tape_border)
                    rgb_base_q <= 12'h566;

                if (vspd_zero_line)
                    rgb_base_q <= 12'hAAA;

                if (alt_line)
                    rgb_base_q <= alt_color;

                if (vspd_line)
                    rgb_base_q <= vspd_color;

                //==============================================================
                // Column 0: apogee authority envelope
                //==============================================================
                if (in_auth_panel) begin
                    rgb_base_q <= 12'h010;
                    if (auth_axis || auth_tick)
                        rgb_base_q <= 12'h153;
                end

                if (auth_unc_band)
                    rgb_base_q <= 12'h426;

                if (auth_unc_edge)
                    rgb_base_q <= 12'h86F;

                if (auth_prediction_corridor)
                    rgb_base_q <= auth_target_reachable ? 12'h143 : 12'h421;

                if (auth_envelope_span)
                    rgb_base_q <= auth_envelope_color;

                if (auth_target_marker)
                    rgb_base_q <= 12'hF0F;

                if (auth_current_marker)
                    rgb_base_q <= 12'hFFF;

                if (auth_full_marker)
                    rgb_base_q <= 12'h3FF;

                if (auth_no_marker)
                    rgb_base_q <= 12'hFD0;

                if (auth_cmd_frame)
                    rgb_base_q <= 12'h686;

                if (auth_cmd_fill)
                    rgb_base_q <= auth_command_color;

                if (auth_quality_hatch)
                    rgb_base_q <= 12'hF80;

                if (auth_phase_strip)
                    rgb_base_q <= auth_phase_color;

                if (auth_phase_source_mark)
                    rgb_base_q <= auth_external_phase_valid ? 12'hFFF : 12'h999;

                if (auth_gate_strip)
                    rgb_base_q <= auth_gate_color;

                if (auth_gate_cell)
                    rgb_base_q <= auth_gate_cell_color;

                if (auth_panel_border)
                    rgb_base_q <= 12'h898;

                //==============================================================
                // Column 1: artificial horizon
                //==============================================================
                if (hzn_in_disc) begin
                    if (hzn_sky_side)
                        rgb_base_q <= rgb(4'h1, 4'h4, 4'h8);
                    else
                        rgb_base_q <= rgb(4'h7, 4'h4, 4'h1);
                end

                if (hzn_ring)
                    rgb_base_q <= 12'hCCC;

                if (hzn_line_on)
                    rgb_base_q <= roll_color;

                if (roll_ref_tick)
                    rgb_base_q <= 12'hFD0;

                if (hzn_aircraft_wings || hzn_aircraft_stem || hzn_center_dot)
                    rgb_base_q <= 12'hFFF;

                if (hzn_quality_hatch)
                    rgb_base_q <= roll_color;

                //==============================================================
                // Column 1: roll-plane vertical vector instrument
                //==============================================================
                if (in_vec_panel) begin
                    rgb_base_q <= 12'h012;
                    if (vec_axis)
                        rgb_base_q <= 12'h034;
                end

                if (vec_quality_hatch)
                    rgb_base_q <= vert_vec_color;

                if (vec_panel_border)
                    rgb_base_q <= 12'h789;

                if (vec_vector_tail || vec_vector_core)
                    rgb_base_q <= vert_vec_color;

                if (vec_vector_tip || vec_center_dot)
                    rgb_base_q <= 12'hFFF;

                //==============================================================
                // Column 2: compass
                //==============================================================
                if (cmp_in_disc)
                    rgb_base_q <= 12'h121;

                if (cmp_ns_axis || cmp_ew_axis)
                    rgb_base_q <= 12'h243;

                if (cmp_ring)
                    rgb_base_q <= 12'hCCC;

                if (cmp_cardinal_tick)
                    rgb_base_q <= 12'hFFF;

                if (cmp_needle_core)
                    rgb_base_q <= head_color;

                if (cmp_needle_tip)
                    rgb_base_q <= 12'hFFF;

                if (cmp_center_dot)
                    rgb_base_q <= 12'hFFF;

                if (cmp_quality_hatch)
                    rgb_base_q <= head_color;

                //==============================================================
                // Column 2: landing dispersion / wind estimator viewport
                //==============================================================
                if (in_landing_panel) begin
                    rgb_base_q <= 12'h011;
                    if (landing_axis)
                        rgb_base_q <= 12'h234;
                end

                if (landing_nav_uncertainty)
                    rgb_base_q <= nav_flags[0] ? 12'h154 : 12'h143;

                if (landing_wind_x_leg || landing_wind_y_leg || landing_wind_z_bar)
                    rgb_base_q <= 12'hFC2;

                if (landing_wind_tip)
                    rgb_base_q <= 12'hFFF;

                if (landing_nav_point)
                    rgb_base_q <= 12'h2F6;

                if (landing_quality_hatch)
                    rgb_base_q <= (!nav_data_ok) ? 12'hF50 : 12'hD93;

                if (landing_panel_border)
                    rgb_base_q <= 12'h79A;

                //==============================================================
                // Charts
                //==============================================================
                if (in_chart_alt) begin
                    rgb_base_q <= 12'h011;
                    if (grid)
                        rgb_base_q <= 12'h022;
                    if (chart_alt_ref)
                        rgb_base_q <= 12'h244;
                    if (chart_alt_cursor)
                        rgb_base_q <= 12'h666;
                    if (chart_alt_now_halo)
                        rgb_base_q <= 12'hFFF;
                    if (chart_alt_point)
                        rgb_base_q <= alt_color;
                end

                if (in_chart_vspd) begin
                    if (y_q1 < CHART_VSPD_CY)
                        rgb_base_q <= 12'h012;
                    else
                        rgb_base_q <= 12'h210;
                    if (grid)
                        rgb_base_q <= 12'h020;
                    if (chart_vspd_zero)
                        rgb_base_q <= 12'hAAA;
                    if (chart_vspd_cursor)
                        rgb_base_q <= 12'h666;
                    if (chart_vspd_now_halo)
                        rgb_base_q <= 12'hFFF;
                    if (chart_vspd_point)
                        rgb_base_q <= vspd_color;
                end

            end
        end
    end

    //==========================================================================
    // Final overlay mux
    //
    // Text overlay has highest priority. Base scene, overlay, active-video, and
    // sync are all taken from the same registered render stage.
    //==========================================================================
    assign vga_hsync = render_hsync_q;
    assign vga_vsync = render_vsync_q;

    flight_vga_page_mux_pix #(
        .PAGE_HUD        (VGA_PAGE_HUD),
        .PAGE_SENSOR_DIAG(VGA_PAGE_SENSOR_DIAG)
    ) u_flight_vga_page_mux_pix (
        .active_pix           (render_active_q),
        .page_select_pix      (render_page_select_q),
        .hud_rgb              (rgb_base_q),
        .sensor_diag_rgb      (rgb_sensor_diag_q),
        .telemetry_overlay_on (render_overlay_on_q),
        .telemetry_overlay_rgb(render_overlay_rgb_q),
        .vga_rgb              (vga_rgb)
    );

endmodule

`default_nettype wire
