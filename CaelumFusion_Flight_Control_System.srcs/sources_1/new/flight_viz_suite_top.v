`timescale 1ns/1ps
`default_nettype none

`include "flight_viz_bundle_defs.vh"
`include "telemetry_defs_vh.vh"

//==============================================================================
// flight_viz_suite_top
//------------------------------------------------------------------------------
// Complete visualization integration shell for the CaelumFusion flight HUD.
//
// SYS-domain raw snapshots and the coherent derived-state publication are first
// normalized by flight_viz_model_sys into the canonical visualization bundle.
// The widened bundle then crosses into PIX through an explicit CDC before the
// renderer consumes it.
//==============================================================================
module flight_viz_suite_top #(
    parameter integer PAYLOAD_W  = 48,

    parameter integer H_ACTIVE   = 640,
    parameter integer H_FP       = 16,
    parameter integer H_SYNC     = 96,
    parameter integer H_BP       = 48,
    parameter integer V_ACTIVE   = 480,
    parameter integer V_FP       = 10,
    parameter integer V_SYNC     = 2,
    parameter integer V_BP       = 33,
    parameter integer HSYNC_POL  = 0,
    parameter integer VSYNC_POL  = 0,

    parameter integer HIST_DEPTH = 1024,

    // Compile-time option for the secondary VGA sensor-diagnostic page.
    // Default-off keeps the xc7a35t/Basys-3 build within its LUT budget.
    parameter integer ENABLE_SENSOR_DIAG_PAGE = 1,

    // Compile-time option for frozen telemetry text. Disable in the default
    // Basys-3 image to keep the graphical HUD while recovering LUT margin.
    parameter integer ENABLE_TELEMETRY_TEXT_OVERLAY = 1
)(
    //==========================================================================
    // SYS domain
    //==========================================================================
    input  wire                    sys_clk,
    input  wire                    sys_rst,

    //--------------------------------------------------------------------------
    // Raw published snapshots
    //--------------------------------------------------------------------------
    input  wire [31:0]             bmp_t_us,
    input  wire [15:0]             bmp_seq,
    input  wire                    bmp_valid,
    input  wire [7:0]              bmp_status,
    input  wire [PAYLOAD_W-1:0]    bmp_payload,
    input  wire [15:0]             bmp_age_ms,

    input  wire [31:0]             acc_t_us,
    input  wire [15:0]             acc_seq,
    input  wire                    acc_valid,
    input  wire [7:0]              acc_status,
    input  wire [PAYLOAD_W-1:0]    acc_payload,
    input  wire [15:0]             acc_age_ms,

    input  wire [31:0]             mag_t_us,
    input  wire [15:0]             mag_seq,
    input  wire                    mag_valid,
    input  wire [7:0]              mag_status,
    input  wire [PAYLOAD_W-1:0]    mag_payload,
    input  wire [15:0]             mag_age_ms,

    input  wire [31:0]             pwr_t_us,
    input  wire [15:0]             pwr_seq,
    input  wire                    pwr_valid,
    input  wire [7:0]              pwr_status,
    input  wire [PAYLOAD_W-1:0]    pwr_payload,
    input  wire [15:0]             pwr_age_ms,

    //--------------------------------------------------------------------------
    // Future sensor-extension evidence summary
    //--------------------------------------------------------------------------
    input  wire                    ext_valid,
    input  wire [7:0]              ext_status,
    input  wire [15:0]             ext_present_flags,
    input  wire [15:0]             ext_fault_flags,
    input  wire [15:0]             ext_mag_delta_l1,
    input  wire [15:0]             ext_mag_norm_primary,
    input  wire [15:0]             ext_mag_norm_secondary,
    input  wire                    ext_mag_sequence_aligned,
    input  wire                    ext_mag_disagreement,
    input  wire [3:0]              ext_mag_sector_delta,
    input  wire [7:0]              ext_mag_source_flags,
    input  wire [15:0]             ext_rng_height_cm,
    input  wire [15:0]             ext_air_dp_pa,
    input  wire [15:0]             ext_air_speed_cms,
    input  wire [15:0]             ext_env_temp_cdeg,
    input  wire [15:0]             ext_env_rh_centi,
    input  wire [15:0]             ext_sun_luma,
    input  wire [15:0]             ext_flow_dx,
    input  wire [15:0]             ext_flow_dy,
    input  wire [15:0]             ext_log_seq,
    input  wire [15:0]             ext_log_drop_count,
    input  wire [15:0]             ext_max_age_ms,

    //--------------------------------------------------------------------------
    // Derived-state publication
    //--------------------------------------------------------------------------
    input  wire                    der_valid,
    input  wire [7:0]              der_status,

    input  wire                    der_alt_fresh,
    input  wire                    der_vspd_fresh,
    input  wire                    der_roll_fresh,
    input  wire                    der_head_fresh,

    input  wire [15:0]             der_bmp_seq_ref,
    input  wire [15:0]             der_acc_seq_ref,
    input  wire [15:0]             der_mag_seq_ref,

    input  wire [15:0]             der_bmp_age_ms,
    input  wire [15:0]             der_acc_age_ms,
    input  wire [15:0]             der_mag_age_ms,

    input  wire                    der_bmp_valid_ref,
    input  wire                    der_acc_valid_ref,
    input  wire                    der_mag_valid_ref,

    input  wire [31:0]             der_altitude_cm,
    input  wire [31:0]             der_vertical_speed_cms,
    input  wire [31:0]             der_roll_mdeg,
    input  wire [31:0]             der_heading_mdeg,

    //--------------------------------------------------------------------------
    // Navigation / wind estimator publication
    //--------------------------------------------------------------------------
    input  wire                    nav_valid,
    input  wire [7:0]              nav_status,
    input  wire [7:0]              nav_flags,
    input  wire [15:0]             nav_downrange_m,
    input  wire [15:0]             nav_crossrange_m,
    input  wire [15:0]             nav_age_ms,

    input  wire                    wind_valid,
    input  wire [7:0]              wind_status,
    input  wire [15:0]             wind_x_cms,
    input  wire [15:0]             wind_y_cms,
    input  wire [15:0]             wind_z_cms,
    input  wire [15:0]             wind_age_ms,

    //--------------------------------------------------------------------------
    // Authority phase / actuation gates
    //--------------------------------------------------------------------------
    input  wire [3:0]              auth_phase_code_sys,
    input  wire                    auth_phase_valid_sys,
    input  wire                    safety_runtime_ok_sys,
    input  wire                    safety_allows_actuation_sys,
    input  wire                    policy_runtime_enable_sys,
    input  wire                    software_armed_sys,

    //--------------------------------------------------------------------------
    // Platform metadata / observability
    //--------------------------------------------------------------------------
    input  wire [15:0]             i2c_nack_count,
    input  wire [15:0]             i2c_timeout_count,
    input  wire [15:0]             txn_rate_hz,
    input  wire [31:0]             cdc_update_count_sys,
    input  wire [31:0]             build_id,
    input  wire [15:0]             schema_word,

    //--------------------------------------------------------------------------
    // Bring-up / debug control
    //--------------------------------------------------------------------------
    input  wire                    viz_selftest_en_sys,
    input  wire [1:0]              vga_page_select_sys,
    input  wire                    history_freeze_sys,

    //==========================================================================
    // PIX domain
    //==========================================================================
    input  wire                    pix_clk,
    input  wire                    pix_rst,

    //==========================================================================
    // VGA outputs
    //==========================================================================
    output wire                    vga_hsync,
    output wire                    vga_vsync,
    output wire [11:0]             vga_rgb
);

    //==========================================================================
    // Helper Functions
    //==========================================================================
    function [15:0] sat_u16_from_u32;
        input [31:0] v;
        begin
            if (v > 32'd65535)
                sat_u16_from_u32 = 16'hFFFF;
            else
                sat_u16_from_u32 = v[15:0];
        end
    endfunction

    function [15:0] sat_s16_from_s32;
        input signed [31:0] v;
        begin
            if (v > 32'sd32767)
                sat_s16_from_s32 = 16'h7FFF;
            else if (v < -32'sd32768)
                sat_s16_from_s32 = 16'h8000;
            else
                sat_s16_from_s32 = v[15:0];
        end
    endfunction

    //==========================================================================
    // Self-test constants
    //==========================================================================
    localparam [7:0]  SELFTEST_BMP_STATUS = `ST_OK;
    localparam [7:0]  SELFTEST_ACC_STATUS = `ST_OK;
    localparam [7:0]  SELFTEST_MAG_STATUS = `ST_OK;
    localparam [7:0]  SELFTEST_PWR_STATUS = `ST_OK;

    localparam [15:0] SELFTEST_BMP_SEQ    = 16'h1234;
    localparam [15:0] SELFTEST_ACC_SEQ    = 16'h5678;
    localparam [15:0] SELFTEST_MAG_SEQ    = 16'h9ABC;
    localparam [15:0] SELFTEST_PWR_SEQ    = 16'h0F5A;

    localparam [15:0] SELFTEST_BMP_AGE    = 16'd111;
    localparam [15:0] SELFTEST_ACC_AGE    = 16'd222;
    localparam [15:0] SELFTEST_MAG_AGE    = 16'd333;
    localparam [15:0] SELFTEST_PWR_AGE    = 16'd44;
    localparam [47:0] SELFTEST_PWR_PAYLOAD = {8'h00, 12'h5A0, 12'h120, 16'h0000};
    localparam [15:0] SELFTEST_EXT_PRESENT = (16'd1 << `EXT_PRESENT_MAG0_BIT) |
                                             (16'd1 << `EXT_PRESENT_MAG1_BIT) |
                                             (16'd1 << `EXT_PRESENT_RANGE_BIT) |
                                             (16'd1 << `EXT_PRESENT_AIR_BIT) |
                                             (16'd1 << `EXT_PRESENT_ENV_BIT) |
                                             (16'd1 << `EXT_PRESENT_SUN_BIT) |
                                             (16'd1 << `EXT_PRESENT_FLOW_BIT) |
                                             (16'd1 << `EXT_PRESENT_BLACKBOX_BIT);
    localparam [7:0] SELFTEST_EXT_MAG_SOURCE =
        (8'd1 << `EXT_SRC_SYNTHETIC_BIT);

    localparam [31:0] SELFTEST_ALT_CM     = 32'd12345;
    localparam [31:0] SELFTEST_VSPD_CMS   = 32'd42;
    localparam [31:0] SELFTEST_ROLL_MDEG  = 32'd12000;
    localparam [31:0] SELFTEST_HEAD_MDEG  = 32'd123000;
    localparam [3:0]  SELFTEST_AUTH_PHASE = `VIZ_AUTH_PHASE_COAST;
    localparam [15:0] SELFTEST_NAV_DOWN_M = 16'd240;
    localparam [15:0] SELFTEST_NAV_CROSS_M = 16'd28;
    localparam [15:0] SELFTEST_WIND_X_CMS = 16'd120;
    localparam [15:0] SELFTEST_WIND_Y_CMS = 16'hFFD8;
    localparam [15:0] SELFTEST_WIND_Z_CMS = 16'd16;

    localparam [31:0] SELFTEST_BUILD_ID   = 32'h5E1F7E57;
    localparam [15:0] SELFTEST_SCHEMA     = 16'hD06E;

    //==========================================================================
    // SYS-domain self-test liveness generator
    //==========================================================================
    reg [19:0] selftest_div_sys;
    reg [15:0] selftest_seq_sys;

    always @(posedge sys_clk) begin
        if (sys_rst) begin
            selftest_div_sys <= 20'd0;
            selftest_seq_sys <= 16'd0;
        end else begin
            selftest_div_sys <= selftest_div_sys + 20'd1;
            if (viz_selftest_en_sys && (selftest_div_sys == 20'd0))
                selftest_seq_sys <= selftest_seq_sys + 16'd1;
        end
    end

    wire selftest_tick_sys = (selftest_div_sys == 20'd0);

    //==========================================================================
    // SYS-domain selected semantic image
    //==========================================================================
    wire [31:0] bmp_t_us_sel_sys;
    wire [15:0] bmp_seq_sel_sys;
    wire        bmp_valid_sel_sys;
    wire [7:0]  bmp_status_sel_sys;
    wire [PAYLOAD_W-1:0] bmp_payload_sel_sys;
    wire [15:0] bmp_age_ms_sel_sys;

    wire [31:0] acc_t_us_sel_sys;
    wire [15:0] acc_seq_sel_sys;
    wire        acc_valid_sel_sys;
    wire [7:0]  acc_status_sel_sys;
    wire [PAYLOAD_W-1:0] acc_payload_sel_sys;
    wire [15:0] acc_age_ms_sel_sys;

    wire [31:0] mag_t_us_sel_sys;
    wire [15:0] mag_seq_sel_sys;
    wire        mag_valid_sel_sys;
    wire [7:0]  mag_status_sel_sys;
    wire [PAYLOAD_W-1:0] mag_payload_sel_sys;
    wire [15:0] mag_age_ms_sel_sys;

    wire [31:0] pwr_t_us_sel_sys;
    wire [15:0] pwr_seq_sel_sys;
    wire        pwr_valid_sel_sys;
    wire [7:0]  pwr_status_sel_sys;
    wire [PAYLOAD_W-1:0] pwr_payload_sel_sys;
    wire [15:0] pwr_age_ms_sel_sys;

    wire        ext_valid_sel_sys;
    wire [7:0]  ext_status_sel_sys;
    wire [15:0] ext_present_flags_sel_sys;
    wire [15:0] ext_fault_flags_sel_sys;
    wire [15:0] ext_mag_delta_l1_sel_sys;
    wire [15:0] ext_mag_norm_primary_sel_sys;
    wire [15:0] ext_mag_norm_secondary_sel_sys;
    wire        ext_mag_sequence_aligned_sel_sys;
    wire        ext_mag_disagreement_sel_sys;
    wire [3:0]  ext_mag_sector_delta_sel_sys;
    wire [7:0]  ext_mag_source_flags_sel_sys;
    wire [15:0] ext_rng_height_cm_sel_sys;
    wire [15:0] ext_air_dp_pa_sel_sys;
    wire [15:0] ext_air_speed_cms_sel_sys;
    wire [15:0] ext_env_temp_cdeg_sel_sys;
    wire [15:0] ext_env_rh_centi_sel_sys;
    wire [15:0] ext_sun_luma_sel_sys;
    wire [15:0] ext_flow_dx_sel_sys;
    wire [15:0] ext_flow_dy_sel_sys;
    wire [15:0] ext_log_seq_sel_sys;
    wire [15:0] ext_log_drop_count_sel_sys;
    wire [15:0] ext_max_age_ms_sel_sys;

    wire        der_valid_sel_sys;
    wire [7:0]  der_status_sel_sys;
    wire        der_alt_fresh_sel_sys;
    wire        der_vspd_fresh_sel_sys;
    wire        der_roll_fresh_sel_sys;
    wire        der_head_fresh_sel_sys;

    wire [15:0] der_bmp_seq_ref_sel_sys;
    wire [15:0] der_acc_seq_ref_sel_sys;
    wire [15:0] der_mag_seq_ref_sel_sys;

    wire [15:0] der_bmp_age_ms_sel_sys;
    wire [15:0] der_acc_age_ms_sel_sys;
    wire [15:0] der_mag_age_ms_sel_sys;

    wire        der_bmp_valid_ref_sel_sys;
    wire        der_acc_valid_ref_sel_sys;
    wire        der_mag_valid_ref_sel_sys;

    wire [31:0] der_altitude_cm_sel_sys;
    wire [31:0] der_vertical_speed_cms_sel_sys;
    wire [31:0] der_roll_mdeg_sel_sys;
    wire [31:0] der_heading_mdeg_sel_sys;

    wire        nav_valid_sel_sys;
    wire [7:0]  nav_status_sel_sys;
    wire [7:0]  nav_flags_sel_sys;
    wire [15:0] nav_downrange_m_sel_sys;
    wire [15:0] nav_crossrange_m_sel_sys;
    wire [15:0] nav_age_ms_sel_sys;

    wire        wind_valid_sel_sys;
    wire [7:0]  wind_status_sel_sys;
    wire [15:0] wind_x_cms_sel_sys;
    wire [15:0] wind_y_cms_sel_sys;
    wire [15:0] wind_z_cms_sel_sys;
    wire [15:0] wind_age_ms_sel_sys;

    wire [3:0]  auth_phase_code_sel_sys;
    wire        auth_phase_valid_sel_sys;
    wire        safety_runtime_ok_sel_sys;
    wire        safety_allows_actuation_sel_sys;
    wire        policy_runtime_enable_sel_sys;
    wire        software_armed_sel_sys;

    wire [15:0] i2c_nack_count_sel_sys;
    wire [15:0] i2c_timeout_count_sel_sys;
    wire [15:0] txn_rate_hz_sel_sys;
    wire [31:0] cdc_update_count_sel_sys;
    wire [31:0] build_id_sel_sys;
    wire [15:0] schema_word_sel_sys;

    assign bmp_t_us_sel_sys      = bmp_t_us;
    assign bmp_seq_sel_sys       = viz_selftest_en_sys ? (SELFTEST_BMP_SEQ ^ selftest_seq_sys) : bmp_seq;
    assign bmp_valid_sel_sys     = viz_selftest_en_sys ? 1'b1                                    : bmp_valid;
    assign bmp_status_sel_sys    = viz_selftest_en_sys ? SELFTEST_BMP_STATUS                     : bmp_status;
    assign bmp_payload_sel_sys   = bmp_payload;
    assign bmp_age_ms_sel_sys    = viz_selftest_en_sys ? SELFTEST_BMP_AGE                        : bmp_age_ms;

    assign acc_t_us_sel_sys      = acc_t_us;
    assign acc_seq_sel_sys       = viz_selftest_en_sys ? (SELFTEST_ACC_SEQ ^ selftest_seq_sys) : acc_seq;
    assign acc_valid_sel_sys     = viz_selftest_en_sys ? 1'b1                                   : acc_valid;
    assign acc_status_sel_sys    = viz_selftest_en_sys ? SELFTEST_ACC_STATUS                    : acc_status;
    assign acc_payload_sel_sys   = acc_payload;
    assign acc_age_ms_sel_sys    = viz_selftest_en_sys ? SELFTEST_ACC_AGE                       : acc_age_ms;

    assign mag_t_us_sel_sys      = mag_t_us;
    assign mag_seq_sel_sys       = viz_selftest_en_sys ? (SELFTEST_MAG_SEQ ^ selftest_seq_sys) : mag_seq;
    assign mag_valid_sel_sys     = viz_selftest_en_sys ? 1'b1                                   : mag_valid;
    assign mag_status_sel_sys    = viz_selftest_en_sys ? SELFTEST_MAG_STATUS                    : mag_status;
    assign mag_payload_sel_sys   = mag_payload;
    assign mag_age_ms_sel_sys    = viz_selftest_en_sys ? SELFTEST_MAG_AGE                       : mag_age_ms;

    assign pwr_t_us_sel_sys      = pwr_t_us;
    assign pwr_seq_sel_sys       = viz_selftest_en_sys ? (SELFTEST_PWR_SEQ ^ selftest_seq_sys) : pwr_seq;
    assign pwr_valid_sel_sys     = viz_selftest_en_sys ? 1'b1                                   : pwr_valid;
    assign pwr_status_sel_sys    = viz_selftest_en_sys ? SELFTEST_PWR_STATUS                    : pwr_status;
    assign pwr_payload_sel_sys   = viz_selftest_en_sys ? SELFTEST_PWR_PAYLOAD                   : pwr_payload;
    assign pwr_age_ms_sel_sys    = viz_selftest_en_sys ? SELFTEST_PWR_AGE                       : pwr_age_ms;

    assign ext_valid_sel_sys =
        viz_selftest_en_sys ? 1'b1 : ext_valid;
    assign ext_status_sel_sys =
        viz_selftest_en_sys ? `ST_OK : ext_status;
    assign ext_present_flags_sel_sys =
        viz_selftest_en_sys ? SELFTEST_EXT_PRESENT : ext_present_flags;
    assign ext_fault_flags_sel_sys =
        viz_selftest_en_sys ? 16'd0 : ext_fault_flags;
    assign ext_mag_delta_l1_sel_sys =
        viz_selftest_en_sys ? 16'd18 : ext_mag_delta_l1;
    assign ext_mag_norm_primary_sel_sys =
        viz_selftest_en_sys ? 16'd960 : ext_mag_norm_primary;
    assign ext_mag_norm_secondary_sel_sys =
        viz_selftest_en_sys ? 16'd972 : ext_mag_norm_secondary;
    assign ext_mag_sequence_aligned_sel_sys =
        viz_selftest_en_sys ? 1'b1 : ext_mag_sequence_aligned;
    assign ext_mag_disagreement_sel_sys =
        viz_selftest_en_sys ? 1'b0 : ext_mag_disagreement;
    assign ext_mag_sector_delta_sel_sys =
        viz_selftest_en_sys ? 4'd1 : ext_mag_sector_delta;
    assign ext_mag_source_flags_sel_sys =
        viz_selftest_en_sys ? SELFTEST_EXT_MAG_SOURCE : ext_mag_source_flags;
    assign ext_rng_height_cm_sel_sys =
        viz_selftest_en_sys ? 16'd185 : ext_rng_height_cm;
    assign ext_air_dp_pa_sel_sys =
        viz_selftest_en_sys ? 16'd42 : ext_air_dp_pa;
    assign ext_air_speed_cms_sel_sys =
        viz_selftest_en_sys ? 16'd1250 : ext_air_speed_cms;
    assign ext_env_temp_cdeg_sel_sys =
        viz_selftest_en_sys ? 16'd2345 : ext_env_temp_cdeg;
    assign ext_env_rh_centi_sel_sys =
        viz_selftest_en_sys ? 16'd4520 : ext_env_rh_centi;
    assign ext_sun_luma_sel_sys =
        viz_selftest_en_sys ? 16'd8192 : ext_sun_luma;
    assign ext_flow_dx_sel_sys =
        viz_selftest_en_sys ? 16'd12 : ext_flow_dx;
    assign ext_flow_dy_sel_sys =
        viz_selftest_en_sys ? 16'hFFF8 : ext_flow_dy;
    assign ext_log_seq_sel_sys =
        viz_selftest_en_sys ? (16'h4000 ^ selftest_seq_sys) : ext_log_seq;
    assign ext_log_drop_count_sel_sys =
        viz_selftest_en_sys ? 16'd0 : ext_log_drop_count;
    assign ext_max_age_ms_sel_sys =
        viz_selftest_en_sys ? 16'd44 : ext_max_age_ms;

    assign der_valid_sel_sys      = viz_selftest_en_sys ? 1'b1  : der_valid;
    assign der_status_sel_sys     = viz_selftest_en_sys ? `ST_OK : der_status;
    assign der_alt_fresh_sel_sys  = viz_selftest_en_sys ? 1'b1  : der_alt_fresh;
    assign der_vspd_fresh_sel_sys = viz_selftest_en_sys ? 1'b1  : der_vspd_fresh;
    assign der_roll_fresh_sel_sys = viz_selftest_en_sys ? 1'b1  : der_roll_fresh;
    assign der_head_fresh_sel_sys = viz_selftest_en_sys ? 1'b1  : der_head_fresh;

    assign der_bmp_seq_ref_sel_sys = viz_selftest_en_sys ? bmp_seq_sel_sys : der_bmp_seq_ref;
    assign der_acc_seq_ref_sel_sys = viz_selftest_en_sys ? acc_seq_sel_sys : der_acc_seq_ref;
    assign der_mag_seq_ref_sel_sys = viz_selftest_en_sys ? mag_seq_sel_sys : der_mag_seq_ref;

    assign der_bmp_age_ms_sel_sys = viz_selftest_en_sys ? bmp_age_ms_sel_sys : der_bmp_age_ms;
    assign der_acc_age_ms_sel_sys = viz_selftest_en_sys ? acc_age_ms_sel_sys : der_acc_age_ms;
    assign der_mag_age_ms_sel_sys = viz_selftest_en_sys ? mag_age_ms_sel_sys : der_mag_age_ms;

    assign der_bmp_valid_ref_sel_sys = viz_selftest_en_sys ? 1'b1 : der_bmp_valid_ref;
    assign der_acc_valid_ref_sel_sys = viz_selftest_en_sys ? 1'b1 : der_acc_valid_ref;
    assign der_mag_valid_ref_sel_sys = viz_selftest_en_sys ? 1'b1 : der_mag_valid_ref;

    assign der_altitude_cm_sel_sys =
        viz_selftest_en_sys ? SELFTEST_ALT_CM : der_altitude_cm;
    assign der_vertical_speed_cms_sel_sys =
        viz_selftest_en_sys ? SELFTEST_VSPD_CMS : der_vertical_speed_cms;
    assign der_roll_mdeg_sel_sys =
        viz_selftest_en_sys ? SELFTEST_ROLL_MDEG : der_roll_mdeg;
    assign der_heading_mdeg_sel_sys =
        viz_selftest_en_sys ? SELFTEST_HEAD_MDEG : der_heading_mdeg;

    assign nav_valid_sel_sys =
        viz_selftest_en_sys ? 1'b1 : nav_valid;
    assign nav_status_sel_sys =
        viz_selftest_en_sys ? `ST_OK : nav_status;
    assign nav_flags_sel_sys =
        viz_selftest_en_sys ? 8'h03 : nav_flags;
    assign nav_downrange_m_sel_sys =
        viz_selftest_en_sys ? SELFTEST_NAV_DOWN_M : nav_downrange_m;
    assign nav_crossrange_m_sel_sys =
        viz_selftest_en_sys ? SELFTEST_NAV_CROSS_M : nav_crossrange_m;
    assign nav_age_ms_sel_sys =
        viz_selftest_en_sys ? 16'd20 : nav_age_ms;

    assign wind_valid_sel_sys =
        viz_selftest_en_sys ? 1'b1 : wind_valid;
    assign wind_status_sel_sys =
        viz_selftest_en_sys ? `ST_OK : wind_status;
    assign wind_x_cms_sel_sys =
        viz_selftest_en_sys ? SELFTEST_WIND_X_CMS : wind_x_cms;
    assign wind_y_cms_sel_sys =
        viz_selftest_en_sys ? SELFTEST_WIND_Y_CMS : wind_y_cms;
    assign wind_z_cms_sel_sys =
        viz_selftest_en_sys ? SELFTEST_WIND_Z_CMS : wind_z_cms;
    assign wind_age_ms_sel_sys =
        viz_selftest_en_sys ? 16'd20 : wind_age_ms;

    assign auth_phase_code_sel_sys =
        viz_selftest_en_sys ? SELFTEST_AUTH_PHASE : auth_phase_code_sys;
    assign auth_phase_valid_sel_sys =
        viz_selftest_en_sys ? 1'b1 : auth_phase_valid_sys;
    assign safety_runtime_ok_sel_sys =
        viz_selftest_en_sys ? 1'b1 : safety_runtime_ok_sys;
    assign safety_allows_actuation_sel_sys =
        viz_selftest_en_sys ? 1'b1 : safety_allows_actuation_sys;
    assign policy_runtime_enable_sel_sys =
        viz_selftest_en_sys ? 1'b1 : policy_runtime_enable_sys;
    assign software_armed_sel_sys =
        viz_selftest_en_sys ? 1'b1 : software_armed_sys;

    assign i2c_nack_count_sel_sys    = viz_selftest_en_sys ? 16'd0 : i2c_nack_count;
    assign i2c_timeout_count_sel_sys = viz_selftest_en_sys ? 16'd0 : i2c_timeout_count;
    assign txn_rate_hz_sel_sys       = viz_selftest_en_sys ? 16'd50 : txn_rate_hz;
    assign cdc_update_count_sel_sys  = viz_selftest_en_sys ? {16'd0, selftest_seq_sys}
                                                           : cdc_update_count_sys;
    assign build_id_sel_sys          = viz_selftest_en_sys ? SELFTEST_BUILD_ID : build_id;
    assign schema_word_sel_sys       = viz_selftest_en_sys ? SELFTEST_SCHEMA   : schema_word;

    //==========================================================================
    // SYS-domain visualization model
    //==========================================================================
    wire [`VIZ_BUNDLE_W-1:0] viz_bundle_model_sys;
    wire [15:0]              viz_key_model_sys;
    wire [9:0]               wr_ptr_model_sys;
    wire                     wr_ptr_pulse_model_sys;
    wire                     alt_we_model_sys;
    wire [9:0]               alt_addr_model_sys;
    wire [15:0]              alt_data_model_sys;
    wire                     vspd_we_model_sys;
    wire [9:0]               vspd_addr_model_sys;
    wire [15:0]              vspd_data_model_sys;

    flight_viz_model_sys #(
        .PAYLOAD_W(PAYLOAD_W)
    ) u_flight_viz_model_sys (
        .sys_clk                 (sys_clk),
        .sys_rst                 (sys_rst),

        .bmp_t_us                (bmp_t_us_sel_sys),
        .bmp_seq                 (bmp_seq_sel_sys),
        .bmp_valid               (bmp_valid_sel_sys),
        .bmp_status              (bmp_status_sel_sys),
        .bmp_payload             (bmp_payload_sel_sys),
        .bmp_age_ms              (bmp_age_ms_sel_sys),

        .acc_t_us                (acc_t_us_sel_sys),
        .acc_seq                 (acc_seq_sel_sys),
        .acc_valid               (acc_valid_sel_sys),
        .acc_status              (acc_status_sel_sys),
        .acc_payload             (acc_payload_sel_sys),
        .acc_age_ms              (acc_age_ms_sel_sys),

        .mag_t_us                (mag_t_us_sel_sys),
        .mag_seq                 (mag_seq_sel_sys),
        .mag_valid               (mag_valid_sel_sys),
        .mag_status              (mag_status_sel_sys),
        .mag_payload             (mag_payload_sel_sys),
        .mag_age_ms              (mag_age_ms_sel_sys),

        .pwr_t_us                (pwr_t_us_sel_sys),
        .pwr_seq                 (pwr_seq_sel_sys),
        .pwr_valid               (pwr_valid_sel_sys),
        .pwr_status              (pwr_status_sel_sys),
        .pwr_payload             (pwr_payload_sel_sys),
        .pwr_age_ms              (pwr_age_ms_sel_sys),

        .ext_valid               (ext_valid_sel_sys),
        .ext_status              (ext_status_sel_sys),
        .ext_present_flags       (ext_present_flags_sel_sys),
        .ext_fault_flags         (ext_fault_flags_sel_sys),
        .ext_mag_delta_l1        (ext_mag_delta_l1_sel_sys),
        .ext_mag_norm_primary    (ext_mag_norm_primary_sel_sys),
        .ext_mag_norm_secondary  (ext_mag_norm_secondary_sel_sys),
        .ext_mag_sequence_aligned(ext_mag_sequence_aligned_sel_sys),
        .ext_mag_disagreement    (ext_mag_disagreement_sel_sys),
        .ext_mag_sector_delta    (ext_mag_sector_delta_sel_sys),
        .ext_mag_source_flags    (ext_mag_source_flags_sel_sys),
        .ext_rng_height_cm       (ext_rng_height_cm_sel_sys),
        .ext_air_dp_pa           (ext_air_dp_pa_sel_sys),
        .ext_air_speed_cms       (ext_air_speed_cms_sel_sys),
        .ext_env_temp_cdeg       (ext_env_temp_cdeg_sel_sys),
        .ext_env_rh_centi        (ext_env_rh_centi_sel_sys),
        .ext_sun_luma            (ext_sun_luma_sel_sys),
        .ext_flow_dx             (ext_flow_dx_sel_sys),
        .ext_flow_dy             (ext_flow_dy_sel_sys),
        .ext_log_seq             (ext_log_seq_sel_sys),
        .ext_log_drop_count      (ext_log_drop_count_sel_sys),
        .ext_max_age_ms          (ext_max_age_ms_sel_sys),

        .der_valid               (der_valid_sel_sys),
        .der_status              (der_status_sel_sys),
        .der_alt_fresh           (der_alt_fresh_sel_sys),
        .der_vspd_fresh          (der_vspd_fresh_sel_sys),
        .der_roll_fresh          (der_roll_fresh_sel_sys),
        .der_head_fresh          (der_head_fresh_sel_sys),

        .der_bmp_seq_ref         (der_bmp_seq_ref_sel_sys),
        .der_acc_seq_ref         (der_acc_seq_ref_sel_sys),
        .der_mag_seq_ref         (der_mag_seq_ref_sel_sys),

        .der_bmp_age_ms          (der_bmp_age_ms_sel_sys),
        .der_acc_age_ms          (der_acc_age_ms_sel_sys),
        .der_mag_age_ms          (der_mag_age_ms_sel_sys),

        .der_bmp_valid_ref       (der_bmp_valid_ref_sel_sys),
        .der_acc_valid_ref       (der_acc_valid_ref_sel_sys),
        .der_mag_valid_ref       (der_mag_valid_ref_sel_sys),

        .der_altitude_cm         (der_altitude_cm_sel_sys),
        .der_vertical_speed_cms  (der_vertical_speed_cms_sel_sys),
        .der_roll_mdeg           (der_roll_mdeg_sel_sys),
        .der_heading_mdeg        (der_heading_mdeg_sel_sys),

        .nav_valid               (nav_valid_sel_sys),
        .nav_status              (nav_status_sel_sys),
        .nav_flags               (nav_flags_sel_sys),
        .nav_downrange_m         (nav_downrange_m_sel_sys),
        .nav_crossrange_m        (nav_crossrange_m_sel_sys),
        .nav_age_ms              (nav_age_ms_sel_sys),

        .wind_valid              (wind_valid_sel_sys),
        .wind_status             (wind_status_sel_sys),
        .wind_x_cms              (wind_x_cms_sel_sys),
        .wind_y_cms              (wind_y_cms_sel_sys),
        .wind_z_cms              (wind_z_cms_sel_sys),
        .wind_age_ms             (wind_age_ms_sel_sys),

        .auth_phase_code         (auth_phase_code_sel_sys),
        .auth_phase_valid        (auth_phase_valid_sel_sys),
        .safety_runtime_ok       (safety_runtime_ok_sel_sys),
        .safety_allows_actuation (safety_allows_actuation_sel_sys),
        .policy_runtime_enable   (policy_runtime_enable_sel_sys),
        .software_armed          (software_armed_sel_sys),

        .i2c_nack_count          (i2c_nack_count_sel_sys),
        .i2c_timeout_count       (i2c_timeout_count_sel_sys),
        .txn_rate_hz             (txn_rate_hz_sel_sys),
        .cdc_update_count        (cdc_update_count_sel_sys),
        .build_id                (build_id_sel_sys),
        .schema_word             (schema_word_sel_sys),

        .alt_we                  (alt_we_model_sys),
        .alt_addr                (alt_addr_model_sys),
        .alt_data                (alt_data_model_sys),
        .vspd_we                 (vspd_we_model_sys),
        .vspd_addr               (vspd_addr_model_sys),
        .vspd_data               (vspd_data_model_sys),

        .viz_bundle_sys          (viz_bundle_model_sys),
        .viz_key_sys             (viz_key_model_sys),
        .wr_ptr_sys              (wr_ptr_model_sys),
        .wr_ptr_pulse_sys        (wr_ptr_pulse_model_sys)
    );

    wire _unused_model_history_ok;
    assign _unused_model_history_ok =
        alt_we_model_sys ^
        alt_addr_model_sys[0] ^
        alt_data_model_sys[0] ^
        vspd_we_model_sys ^
        vspd_addr_model_sys[0] ^
        vspd_data_model_sys[0] ^
        wr_ptr_model_sys[0] ^
        wr_ptr_pulse_model_sys;

    //==========================================================================
    // SYS-domain CDC publication pipeline
    //==========================================================================
    reg [`VIZ_BUNDLE_W-1:0] viz_bundle_publish_sys;
    reg [15:0]              viz_key_prev_sys;
    reg                     viz_publish_pulse_sys;

    wire viz_model_changed_sys = (viz_key_model_sys != viz_key_prev_sys);
    wire viz_publish_request_sys =
        viz_model_changed_sys | (viz_selftest_en_sys & selftest_tick_sys);

    always @(posedge sys_clk) begin
        if (sys_rst) begin
            viz_bundle_publish_sys <= {`VIZ_BUNDLE_W{1'b0}};
            viz_key_prev_sys       <= 16'd0;
            viz_publish_pulse_sys  <= 1'b0;
        end else begin
            viz_publish_pulse_sys <= viz_publish_request_sys;

            if (viz_publish_request_sys)
                viz_bundle_publish_sys <= viz_bundle_model_sys;

            viz_key_prev_sys <= viz_key_model_sys;
        end
    end

    //==========================================================================
    // SYS -> PIX visualization bundle CDC
    //==========================================================================
    wire [`VIZ_BUNDLE_W-1:0] viz_bundle_pix_shadow;
    wire                     viz_bundle_update_pix;

    flight_viz_bundle_cdc #(
        .BUNDLE_W(`VIZ_BUNDLE_W)
    ) u_viz_bundle_cdc (
        .src_clk           (sys_clk),
        .src_rst           (sys_rst),
        .src_bundle        (viz_bundle_publish_sys),
        .src_publish_pulse (viz_publish_pulse_sys),

        .dst_clk           (pix_clk),
        .dst_rst           (pix_rst),
        .dst_bundle_shadow (viz_bundle_pix_shadow),
        .dst_update_pulse  (viz_bundle_update_pix)
    );

    //==========================================================================
    // PIX-domain history rings
    //==========================================================================
    reg [9:0]  wr_ptr_pix_r;
    reg        wr_ptr_update_pix_r;
    reg [15:0] hist_bmp_seq_pix_r;
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg [2:0] history_freeze_pix_ff;

    wire [31:0] alt_shadow_pix_w;
    wire [31:0] vspd_shadow_pix_w;
    wire [15:0] bmp_seq_shadow_pix_w;
    wire        bmp_good_shadow_pix_w;

    assign alt_shadow_pix_w =
        viz_bundle_pix_shadow[`VIZ_DER_ALT_CM_MSB:`VIZ_DER_ALT_CM_LSB];
    assign vspd_shadow_pix_w =
        viz_bundle_pix_shadow[`VIZ_DER_VSPD_CMS_MSB:`VIZ_DER_VSPD_CMS_LSB];
    assign bmp_seq_shadow_pix_w =
        viz_bundle_pix_shadow[`VIZ_BMP_SEQ_MSB:`VIZ_BMP_SEQ_LSB];
    assign bmp_good_shadow_pix_w =
        viz_bundle_pix_shadow[`VIZ_BMP_VALID_BIT] &&
        (viz_bundle_pix_shadow[`VIZ_BMP_STATUS_MSB:`VIZ_BMP_STATUS_LSB] == `ST_OK);
    wire history_freeze_pix_w = history_freeze_pix_ff[2];
    wire hist_write_pix_w =
        !history_freeze_pix_w &&
        viz_bundle_update_pix &&
        bmp_good_shadow_pix_w &&
        (bmp_seq_shadow_pix_w != hist_bmp_seq_pix_r);

    wire [15:0] hist_alt_din_pix_w  = sat_u16_from_u32(alt_shadow_pix_w);
    wire [15:0] hist_vspd_din_pix_w = sat_s16_from_s32($signed(vspd_shadow_pix_w));

    always @(posedge pix_clk) begin
        if (pix_rst) begin
            wr_ptr_pix_r        <= 10'd0;
            wr_ptr_update_pix_r <= 1'b0;
            hist_bmp_seq_pix_r  <= 16'd0;
            history_freeze_pix_ff <= 3'b000;
        end else begin
            wr_ptr_update_pix_r <= 1'b0;
            history_freeze_pix_ff <= {history_freeze_pix_ff[1:0], history_freeze_sys};

            if (hist_write_pix_w) begin
                wr_ptr_pix_r                <= wr_ptr_pix_r + 10'd1;
                wr_ptr_update_pix_r         <= 1'b1;
                hist_bmp_seq_pix_r          <= bmp_seq_shadow_pix_w;
            end
        end
    end

    //--------------------------------------------------------------------------
    // Local synchronous read ports for chart history
    //--------------------------------------------------------------------------
    wire [9:0] alt_rd_addr_w;
    wire [15:0] alt_rd_data_w;

    wire [9:0] vspd_rd_addr_w;
    wire [15:0] vspd_rd_data_w;

    generate
        if (HIST_DEPTH == 1024) begin : gen_history_bram

        //
        /*
            dp_bram_1024x16 u_alt_hist_bram (
                .a_clk  (pix_clk),
                .a_we   (hist_write_pix_w),
                .a_addr (wr_ptr_pix_r),
                .a_din  (hist_alt_din_pix_w),
                .a_dout (),

                .b_clk  (pix_clk),
                .b_addr (alt_rd_addr_w),
                .b_dout (alt_rd_data_w)
            );

            dp_bram_1024x16 u_vspd_hist_bram (
                .a_clk  (pix_clk),
                .a_we   (hist_write_pix_w),
                .a_addr (wr_ptr_pix_r),
                .a_din  (hist_vspd_din_pix_w),
                .a_dout (),

                .b_clk  (pix_clk),
                .b_addr (vspd_rd_addr_w),
                .b_dout (vspd_rd_data_w)
            );

            //*/
        end else begin : gen_history_reg_fallback
            reg [15:0] alt_hist_mem  [0:HIST_DEPTH-1];
            reg [15:0] vspd_hist_mem [0:HIST_DEPTH-1];
            reg [15:0] alt_rd_data_r;
            reg [15:0] vspd_rd_data_r;
            integer idx;

            always @(posedge pix_clk) begin
                if (pix_rst) begin
                    alt_rd_data_r  <= 16'd0;
                    vspd_rd_data_r <= 16'd0;
                    for (idx = 0; idx < HIST_DEPTH; idx = idx + 1) begin
                        alt_hist_mem[idx]  <= 16'd0;
                        vspd_hist_mem[idx] <= 16'd0;
                    end
                end else begin
                    if (hist_write_pix_w) begin
                        alt_hist_mem[wr_ptr_pix_r]  <= hist_alt_din_pix_w;
                        vspd_hist_mem[wr_ptr_pix_r] <= hist_vspd_din_pix_w;
                    end
                    alt_rd_data_r  <= alt_hist_mem[alt_rd_addr_w];
                    vspd_rd_data_r <= vspd_hist_mem[vspd_rd_addr_w];
                end
            end

            assign alt_rd_data_w  = alt_rd_data_r;
            assign vspd_rd_data_w = vspd_rd_data_r;
        end
    endgenerate

    reg vga_sensor_diag_page_sys_r;

    always @(posedge sys_clk) begin
        if (sys_rst) begin
            vga_sensor_diag_page_sys_r <= 1'b0;
        end else begin
            vga_sensor_diag_page_sys_r <= (vga_page_select_sys == 2'd1);
        end
    end

    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg [2:0] vga_diag_page_pix_ff;
    reg [1:0] vga_page_select_pix_r;

    always @(posedge pix_clk) begin
        if (pix_rst) begin
            vga_diag_page_pix_ff    <= 3'b000;
            vga_page_select_pix_r   <= 2'd0;
        end else begin
            vga_diag_page_pix_ff  <= {vga_diag_page_pix_ff[1:0],
                                      vga_sensor_diag_page_sys_r};
            vga_page_select_pix_r <= vga_diag_page_pix_ff[2] ? 2'd1 : 2'd0;
        end
    end

    //==========================================================================
    // PIX-domain visualizer
    //==========================================================================
    flight_visualizer_pix #(
        .H_ACTIVE  (H_ACTIVE),
        .H_FP      (H_FP),
        .H_SYNC    (H_SYNC),
        .H_BP      (H_BP),
        .V_ACTIVE  (V_ACTIVE),
        .V_FP      (V_FP),
        .V_SYNC    (V_SYNC),
        .V_BP      (V_BP),
        .HSYNC_POL (HSYNC_POL),
        .VSYNC_POL (VSYNC_POL),
        .ENABLE_SENSOR_DIAG_PAGE(ENABLE_SENSOR_DIAG_PAGE),
        .ENABLE_TELEMETRY_TEXT_OVERLAY(ENABLE_TELEMETRY_TEXT_OVERLAY)
    ) u_flight_visualizer_pix (
        .pix_clk              (pix_clk),
        .pix_rst              (pix_rst),

        .viz_bundle_pix       (viz_bundle_pix_shadow),
        .viz_update_pix       (viz_bundle_update_pix),
        .vga_page_select_pix  (vga_page_select_pix_r),

        .wr_ptr_pix           (wr_ptr_pix_r),
        .wr_ptr_update_pix    (wr_ptr_update_pix_r),

        .alt_rd_addr          (alt_rd_addr_w),
        .alt_rd_data          (alt_rd_data_w),

        .vspd_rd_addr         (vspd_rd_addr_w),
        .vspd_rd_data         (vspd_rd_data_w),

        .vga_hsync            (vga_hsync),
        .vga_vsync            (vga_vsync),
        .vga_rgb              (vga_rgb)
    );

endmodule

`default_nettype wire
