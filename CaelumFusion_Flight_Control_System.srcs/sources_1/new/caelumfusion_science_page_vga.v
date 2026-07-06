`timescale 1ns / 1ps
`default_nettype none

`include "telemetry_defs_vh.vh"

// -----------------------------------------------------------------------------
// CaelumFusion compact science evidence pages for the 640x480 visible path
// -----------------------------------------------------------------------------
// This block is intentionally an overlay stage after the existing flight HUD,
// sensor diagnostic, and compass renderers. It owns only the science page IDs,
// so legacy pages pass through unchanged. SYS-domain evidence is copied into a
// held snapshot and then transferred to pix_clk with a toggle handshake; the
// wide bus is held stable for many pixel clocks before the synchronized toggle
// is observed.
//
// Page IDs:
//   4 = physics/environment explanation.
//   5 = wind triangle / dispersion evidence.
//   6 = sensor integrity correlation.
//
// The first RTL version uses deterministic geometric panels rather than a font
// ROM. Unsupported planned families render as explicit unavailable hatching;
// zeros are never presented as live measurements.
// -----------------------------------------------------------------------------
module caelumfusion_science_page_vga #(
    parameter integer H_ACTIVE  = 640,
    parameter integer H_FP      = 16,
    parameter integer H_SYNC    = 96,
    parameter integer H_BP      = 48,
    parameter integer V_ACTIVE  = 480,
    parameter integer V_FP      = 10,
    parameter integer V_SYNC    = 2,
    parameter integer V_BP      = 33
) (
    input  wire        sys_clk,
    input  wire        sys_rst,
    input  wire [2:0]  page_id_sys,

    input  wire        bmp_valid,
    input  wire [7:0]  bmp_status,
    input  wire [15:0] bmp_age_ms,

    input  wire        acc_valid,
    input  wire [7:0]  acc_status,
    input  wire [15:0] acc_age_ms,

    input  wire        mag_valid,
    input  wire [7:0]  mag_status,
    input  wire [47:0] mag_payload,
    input  wire [15:0] mag_age_ms,

    input  wire        pwr_valid,
    input  wire [7:0]  pwr_status,
    input  wire [47:0] pwr_payload,
    input  wire [15:0] pwr_age_ms,

    input  wire        ext_valid,
    input  wire [7:0]  ext_status,
    input  wire [15:0] ext_present_flags,
    input  wire [15:0] ext_fault_flags,
    input  wire [15:0] ext_mag_delta_l1,
    input  wire [15:0] ext_mag_norm_primary,
    input  wire [15:0] ext_mag_norm_secondary,
    input  wire        ext_mag_sequence_aligned,
    input  wire        ext_mag_disagreement,
    input  wire [3:0]  ext_mag_sector_delta,
    input  wire [15:0] ext_mag_norm_delta_l1,
    input  wire [15:0] ext_mag_iron_residual,
    input  wire [7:0]  ext_mag_cal_state,
    input  wire [7:0]  ext_mag_source_flags,
    input  wire [15:0] ext_mag_bridge_checksum,
    input  wire [15:0] ext_rng_height_cm,
    input  wire [15:0] ext_air_dp_pa,
    input  wire [15:0] ext_air_speed_cms,
    input  wire [15:0] ext_env_temp_cdeg,
    input  wire [15:0] ext_env_rh_centi,
    input  wire [15:0] ext_sun_luma,
    input  wire [15:0] ext_flow_dx,
    input  wire [15:0] ext_flow_dy,
    input  wire [15:0] ext_log_seq,
    input  wire [15:0] ext_log_drop_count,
    input  wire [15:0] ext_max_age_ms,

    input  wire        der_valid,
    input  wire [7:0]  der_status,
    input  wire        der_alt_fresh,
    input  wire        der_vspd_fresh,
    input  wire [15:0] der_bmp_age_ms,
    input  wire [15:0] der_acc_age_ms,
    input  wire [15:0] der_mag_age_ms,
    input  wire [31:0] der_altitude_cm,
    input  wire [31:0] der_vertical_speed_cms,

    input  wire        nav_valid,
    input  wire [7:0]  nav_status,
    input  wire [15:0] nav_downrange_m,
    input  wire [15:0] nav_crossrange_m,
    input  wire [15:0] nav_age_ms,

    input  wire        wind_valid,
    input  wire [7:0]  wind_status,
    input  wire [15:0] wind_x_cms,
    input  wire [15:0] wind_y_cms,
    input  wire [15:0] wind_age_ms,

    input  wire [3:0]  auth_phase_code_sys,
    input  wire        auth_phase_valid_sys,
    input  wire        safety_runtime_ok_sys,
    input  wire        safety_allows_actuation_sys,
    input  wire        policy_runtime_enable_sys,
    input  wire        software_armed_sys,

    input  wire        pix_clk,
    input  wire        pix_rst,
    input  wire        vga_hsync_in,
    input  wire        vga_vsync_in,
    input  wire [11:0] vga_rgb_in,
    output wire        vga_hsync_out,
    output wire        vga_vsync_out,
    output wire [11:0] vga_rgb_out
);

    localparam [2:0] VIEW_SCIENCE_EXPLAIN   = 3'd4;
    localparam [2:0] VIEW_SCIENCE_WIND      = 3'd5;
    localparam [2:0] VIEW_SCIENCE_INTEGRITY = 3'd6;

    localparam integer H_TOTAL = H_ACTIVE + H_FP + H_SYNC + H_BP;
    localparam integer V_TOTAL = V_ACTIVE + V_FP + V_SYNC + V_BP;
    localparam [11:0] H_ACTIVE_12 = H_ACTIVE;
    localparam [11:0] V_ACTIVE_12 = V_ACTIVE;
    localparam integer SNAP_W = 716;

    reg [15:0] snap_div_sys;
    reg [SNAP_W-1:0] snap_bus_sys;
    reg snap_toggle_sys;

    always @(posedge sys_clk) begin
        if (sys_rst) begin
            snap_div_sys    <= 16'd0;
            snap_bus_sys    <= {SNAP_W{1'b0}};
            snap_toggle_sys <= 1'b0;
        end else begin
            snap_div_sys <= snap_div_sys + 16'd1;
            if (snap_div_sys == 16'd0) begin
                snap_bus_sys <= {
                    page_id_sys,
                    bmp_valid, bmp_status, bmp_age_ms,
                    acc_valid, acc_status, acc_age_ms,
                    mag_valid, mag_status, mag_age_ms, mag_payload,
                    pwr_valid, pwr_status, pwr_age_ms, pwr_payload[15:0],
                    ext_valid, ext_status, ext_present_flags, ext_fault_flags,
                    ext_mag_delta_l1, ext_mag_norm_primary,
                    ext_mag_norm_secondary, ext_mag_sequence_aligned,
                    ext_mag_disagreement, ext_mag_sector_delta,
                    ext_mag_norm_delta_l1, ext_mag_iron_residual,
                    ext_mag_cal_state, ext_mag_source_flags,
                    ext_mag_bridge_checksum,
                    ext_rng_height_cm, ext_air_dp_pa, ext_air_speed_cms,
                    ext_env_temp_cdeg, ext_env_rh_centi, ext_sun_luma,
                    ext_flow_dx, ext_flow_dy,
                    ext_log_seq, ext_log_drop_count, ext_max_age_ms,
                    der_valid, der_status, der_alt_fresh, der_vspd_fresh,
                    der_altitude_cm[15:0], der_vertical_speed_cms[15:0],
                    der_bmp_age_ms, der_acc_age_ms, der_mag_age_ms,
                    nav_valid, nav_status, nav_downrange_m, nav_crossrange_m,
                    nav_age_ms,
                    wind_valid, wind_status, wind_x_cms, wind_y_cms,
                    wind_age_ms,
                    auth_phase_code_sys, auth_phase_valid_sys,
                    safety_runtime_ok_sys, safety_allows_actuation_sys,
                    policy_runtime_enable_sys, software_armed_sys
                };
                snap_toggle_sys <= ~snap_toggle_sys;
            end
        end
    end

    reg [2:0] snap_toggle_pix;
    reg [SNAP_W-1:0] snap_bus_pix;

    always @(posedge pix_clk) begin
        if (pix_rst) begin
            snap_toggle_pix <= 3'b000;
            snap_bus_pix    <= {SNAP_W{1'b0}};
        end else begin
            snap_toggle_pix <= {snap_toggle_pix[1:0], snap_toggle_sys};
            if (snap_toggle_pix[2] ^ snap_toggle_pix[1]) begin
                snap_bus_pix <= snap_bus_sys;
            end
        end
    end

    wire [2:0]  page_id_pix;
    wire        bmp_valid_pix;
    wire [7:0]  bmp_status_pix;
    wire [15:0] bmp_age_ms_pix;
    wire        acc_valid_pix;
    wire [7:0]  acc_status_pix;
    wire [15:0] acc_age_ms_pix;
    wire        mag_valid_pix;
    wire [7:0]  mag_status_pix;
    wire [15:0] mag_age_ms_pix;
    wire [47:0] mag_payload_pix;
    wire        pwr_valid_pix;
    wire [7:0]  pwr_status_pix;
    wire [15:0] pwr_age_ms_pix;
    wire [15:0] pwr_payload16_pix;
    wire        ext_valid_pix;
    wire [7:0]  ext_status_pix;
    wire [15:0] ext_present_flags_pix;
    wire [15:0] ext_fault_flags_pix;
    wire [15:0] ext_mag_delta_l1_pix;
    wire [15:0] ext_mag_norm_primary_pix;
    wire [15:0] ext_mag_norm_secondary_pix;
    wire        ext_mag_sequence_aligned_pix;
    wire        ext_mag_disagreement_pix;
    wire [3:0]  ext_mag_sector_delta_pix;
    wire [15:0] ext_mag_norm_delta_l1_pix;
    wire [15:0] ext_mag_iron_residual_pix;
    wire [7:0]  ext_mag_cal_state_pix;
    wire [7:0]  ext_mag_source_flags_pix;
    wire [15:0] ext_mag_bridge_checksum_pix;
    wire [15:0] ext_rng_height_cm_pix;
    wire [15:0] ext_air_dp_pa_pix;
    wire [15:0] ext_air_speed_cms_pix;
    wire [15:0] ext_env_temp_cdeg_pix;
    wire [15:0] ext_env_rh_centi_pix;
    wire [15:0] ext_sun_luma_pix;
    wire [15:0] ext_flow_dx_pix;
    wire [15:0] ext_flow_dy_pix;
    wire [15:0] ext_log_seq_pix;
    wire [15:0] ext_log_drop_count_pix;
    wire [15:0] ext_max_age_ms_pix;
    wire        der_valid_pix;
    wire [7:0]  der_status_pix;
    wire        der_alt_fresh_pix;
    wire        der_vspd_fresh_pix;
    wire [15:0] der_altitude_cm_pix;
    wire [15:0] der_vertical_speed_cms_pix;
    wire [15:0] der_bmp_age_ms_pix;
    wire [15:0] der_acc_age_ms_pix;
    wire [15:0] der_mag_age_ms_pix;
    wire        nav_valid_pix;
    wire [7:0]  nav_status_pix;
    wire [15:0] nav_downrange_m_pix;
    wire [15:0] nav_crossrange_m_pix;
    wire [15:0] nav_age_ms_pix;
    wire        wind_valid_pix;
    wire [7:0]  wind_status_pix;
    wire [15:0] wind_x_cms_pix;
    wire [15:0] wind_y_cms_pix;
    wire [15:0] wind_age_ms_pix;
    wire [3:0]  auth_phase_code_pix;
    wire        auth_phase_valid_pix;
    wire        safety_runtime_ok_pix;
    wire        safety_allows_actuation_pix;
    wire        policy_runtime_enable_pix;
    wire        software_armed_pix;

    assign {
        page_id_pix,
        bmp_valid_pix, bmp_status_pix, bmp_age_ms_pix,
        acc_valid_pix, acc_status_pix, acc_age_ms_pix,
        mag_valid_pix, mag_status_pix, mag_age_ms_pix, mag_payload_pix,
        pwr_valid_pix, pwr_status_pix, pwr_age_ms_pix, pwr_payload16_pix,
        ext_valid_pix, ext_status_pix, ext_present_flags_pix, ext_fault_flags_pix,
        ext_mag_delta_l1_pix, ext_mag_norm_primary_pix,
        ext_mag_norm_secondary_pix, ext_mag_sequence_aligned_pix,
        ext_mag_disagreement_pix, ext_mag_sector_delta_pix,
        ext_mag_norm_delta_l1_pix, ext_mag_iron_residual_pix,
        ext_mag_cal_state_pix, ext_mag_source_flags_pix,
        ext_mag_bridge_checksum_pix,
        ext_rng_height_cm_pix, ext_air_dp_pa_pix, ext_air_speed_cms_pix,
        ext_env_temp_cdeg_pix, ext_env_rh_centi_pix, ext_sun_luma_pix,
        ext_flow_dx_pix, ext_flow_dy_pix,
        ext_log_seq_pix, ext_log_drop_count_pix, ext_max_age_ms_pix,
        der_valid_pix, der_status_pix, der_alt_fresh_pix, der_vspd_fresh_pix,
        der_altitude_cm_pix, der_vertical_speed_cms_pix,
        der_bmp_age_ms_pix, der_acc_age_ms_pix, der_mag_age_ms_pix,
        nav_valid_pix, nav_status_pix, nav_downrange_m_pix,
        nav_crossrange_m_pix, nav_age_ms_pix,
        wind_valid_pix, wind_status_pix, wind_x_cms_pix, wind_y_cms_pix,
        wind_age_ms_pix,
        auth_phase_code_pix, auth_phase_valid_pix, safety_runtime_ok_pix,
        safety_allows_actuation_pix, policy_runtime_enable_pix,
        software_armed_pix
    } = snap_bus_pix;

    reg [11:0] h_count;
    reg [11:0] v_count;

    always @(posedge pix_clk) begin
        if (pix_rst) begin
            h_count <= 12'd0;
            v_count <= 12'd0;
        end else if (h_count == (H_TOTAL - 1)) begin
            h_count <= 12'd0;
            if (v_count == (V_TOTAL - 1)) begin
                v_count <= 12'd0;
            end else begin
                v_count <= v_count + 12'd1;
            end
        end else begin
            h_count <= h_count + 12'd1;
        end
    end

    wire active_pix = (h_count < H_ACTIVE_12) && (v_count < V_ACTIVE_12);
    wire science_page_active = (page_id_pix == VIEW_SCIENCE_EXPLAIN) ||
                               (page_id_pix == VIEW_SCIENCE_WIND) ||
                               (page_id_pix == VIEW_SCIENCE_INTEGRITY);

    function in_box;
        input [11:0] px;
        input [11:0] py;
        input [11:0] x0;
        input [11:0] x1;
        input [11:0] y0;
        input [11:0] y1;
        begin
            in_box = (px >= x0) && (px < x1) && (py >= y0) && (py < y1);
        end
    endfunction

    function [11:0] status_rgb;
        input valid;
        input [7:0] status;
        begin
            if (valid == 1'b0) begin
                status_rgb = 12'h555;
            end else if (status != `ST_OK) begin
                status_rgb = 12'hfb2;
            end else begin
                status_rgb = 12'h2d5;
            end
        end
    endfunction

    function [11:0] ext_status_rgb;
        input present;
        input faulted;
        begin
            if (!present)
                ext_status_rgb = 12'h555;
            else if (!ext_valid_pix || (ext_status_pix != `ST_OK))
                ext_status_rgb = 12'hf42;
            else if (faulted)
                ext_status_rgb = 12'hfb2;
            else
                ext_status_rgb = 12'h2d5;
        end
    endfunction

    function [15:0] abs16;
        input [15:0] v;
        begin
            abs16 = v[15] ? ((~v) + 16'd1) : v;
        end
    endfunction

    function [9:0] bar10_shift4;
        input [15:0] v;
        reg [11:0] scaled;
        begin
            scaled = v[15:4];
            if (scaled > 12'd220)
                bar10_shift4 = 10'd220;
            else
                bar10_shift4 = scaled[9:0];
        end
    endfunction

    function [9:0] bar10_shift5;
        input [15:0] v;
        reg [10:0] scaled;
        begin
            scaled = v[15:5];
            if (scaled > 11'd180)
                bar10_shift5 = 10'd180;
            else
                bar10_shift5 = scaled[9:0];
        end
    endfunction

    wire range_present_pix = ext_present_flags_pix[`EXT_PRESENT_RANGE_BIT];
    wire air_present_pix   = ext_present_flags_pix[`EXT_PRESENT_AIR_BIT];
    wire env_present_pix   = ext_present_flags_pix[`EXT_PRESENT_ENV_BIT];
    wire sun_present_pix   = ext_present_flags_pix[`EXT_PRESENT_SUN_BIT];
    wire flow_present_pix  = ext_present_flags_pix[`EXT_PRESENT_FLOW_BIT];
    wire mag1_present_pix  = ext_present_flags_pix[`EXT_PRESENT_MAG1_BIT];
    wire blackbox_present_pix = ext_present_flags_pix[`EXT_PRESENT_BLACKBOX_BIT];

    wire range_fault_pix = ext_fault_flags_pix[`EXT_FLG_RANGE_STALE_BIT];
    wire air_fault_pix   = ext_fault_flags_pix[`EXT_FLG_AIR_STALE_BIT];
    wire env_fault_pix   = ext_fault_flags_pix[`EXT_FLG_ENV_STALE_BIT];
    wire sun_fault_pix   = ext_fault_flags_pix[`EXT_FLG_SUN_STALE_BIT];
    wire flow_fault_pix  = ext_fault_flags_pix[`EXT_FLG_FLOW_STALE_BIT];
    wire mag_fault_pix   = ext_fault_flags_pix[`EXT_FLG_MAG_PAIR_MISSING_BIT] |
                           ext_fault_flags_pix[`EXT_FLG_MAG_DISAGREE_BIT] |
                           ext_fault_flags_pix[`EXT_FLG_MAG0_NORM_OOR_BIT] |
                           ext_fault_flags_pix[`EXT_FLG_MAG1_NORM_OOR_BIT] |
                           ext_fault_flags_pix[`EXT_FLG_MAG_NORM_MISMATCH_BIT];

    wire [9:0] range_bar_px = bar10_shift4(ext_rng_height_cm_pix);
    wire [9:0] air_bar_px   = bar10_shift4(ext_air_speed_cms_pix);
    wire [9:0] dp_bar_px    = bar10_shift4(ext_air_dp_pa_pix);
    wire [9:0] env_bar_px   = bar10_shift5(abs16(ext_env_temp_cdeg_pix));
    wire [9:0] rh_bar_px    = bar10_shift5(ext_env_rh_centi_pix);
    wire [9:0] sun_bar_px   = bar10_shift4(ext_sun_luma_pix);
    wire [9:0] flow_dx_bar_px = bar10_shift4(abs16(ext_flow_dx_pix));
    wire [9:0] flow_dy_bar_px = bar10_shift4(abs16(ext_flow_dy_pix));
    wire [9:0] ext_age_bar_px = (ext_max_age_ms_pix[9:0] > 10'd220) ?
                                10'd220 : ext_max_age_ms_pix[9:0];

    wire [9:0] nav_dx_px = (nav_downrange_m_pix[9:0] > 10'd160) ? 10'd160 : nav_downrange_m_pix[9:0];
    wire [9:0] nav_dy_px = (nav_crossrange_m_pix[9:0] > 10'd120) ? 10'd120 : nav_crossrange_m_pix[9:0];

    wire [15:0] wind_x_abs_pix = wind_x_cms_pix[15] ? ((~wind_x_cms_pix) + 16'd1) : wind_x_cms_pix;
    wire [15:0] wind_y_abs_pix = wind_y_cms_pix[15] ? ((~wind_y_cms_pix) + 16'd1) : wind_y_cms_pix;
    wire [9:0] wind_dx_px = (wind_x_abs_pix[15:8] > 8'd150) ? 10'd150 : {2'b00, wind_x_abs_pix[15:8]};
    wire [9:0] wind_dy_px = (wind_y_abs_pix[15:8] > 8'd110) ? 10'd110 : {2'b00, wind_y_abs_pix[15:8]};
    wire wind_x_neg = wind_x_cms_pix[15];
    wire wind_y_neg = wind_y_cms_pix[15];

    wire [9:0] mag_norm0_bar_px = bar10_shift4(ext_mag_norm_primary_pix);
    wire [9:0] mag_norm1_bar_px = bar10_shift4(ext_mag_norm_secondary_pix);
    wire [9:0] mag_delta_bar_px = bar10_shift4(ext_mag_delta_l1_pix);
    wire [9:0] mag_resid_bar_px = bar10_shift4(ext_mag_iron_residual_pix);
    wire [11:0] mag_sector_x = 12'd38 + {4'd0, ext_mag_sector_delta_pix, 4'd0};
    wire ext_mag_source_synth_pix = ext_mag_source_flags_pix[`EXT_SRC_SYNTHETIC_BIT];
    wire ext_mag_source_real_pix  = ext_mag_source_flags_pix[`EXT_SRC_REAL_BIT];

    reg [11:0] science_rgb;

    always @* begin
        science_rgb = 12'h000;

        case (page_id_pix)
            VIEW_SCIENCE_EXPLAIN: begin
                science_rgb = 12'h013;
                if (in_box(h_count, v_count, 12'd0, 12'd640, 12'd0, 12'd30)) begin
                    science_rgb = 12'h17f;
                end
                if (in_box(h_count, v_count, 12'd20, 12'd150, 12'd46, 12'd62)) begin
                    science_rgb = ext_status_rgb(range_present_pix, range_fault_pix);
                end
                if (in_box(h_count, v_count, 12'd160, 12'd290, 12'd46, 12'd62)) begin
                    science_rgb = ext_status_rgb(air_present_pix, air_fault_pix);
                end
                if (in_box(h_count, v_count, 12'd300, 12'd430, 12'd46, 12'd62)) begin
                    science_rgb = ext_status_rgb(env_present_pix, env_fault_pix);
                end
                if (in_box(h_count, v_count, 12'd40, 12'd120, 12'd120, 12'd430)) begin
                    science_rgb = 12'h124;
                end
                if (in_box(h_count, v_count, 12'd62, 12'd96, 12'd420 - {2'b00, range_bar_px}, 12'd420)) begin
                    science_rgb = ext_status_rgb(range_present_pix, range_fault_pix);
                end
                if (in_box(h_count, v_count, 12'd150, 12'd150 + {2'b00, air_bar_px}, 12'd210, 12'd226)) begin
                    science_rgb = ext_status_rgb(air_present_pix, air_fault_pix);
                end
                if (in_box(h_count, v_count, 12'd150, 12'd150 + {2'b00, dp_bar_px}, 12'd190, 12'd246)) begin
                    if (v_count[2] == 1'b0) begin
                        science_rgb = 12'h953;
                    end
                end
                if (in_box(h_count, v_count, 12'd340, 12'd610, 12'd110, 12'd230)) begin
                    science_rgb = ext_status_rgb(env_present_pix, env_fault_pix);
                    if ((h_count[4] ^ v_count[4]) == 1'b1) begin
                        science_rgb = 12'h243;
                    end
                end
                if (in_box(h_count, v_count, 12'd360, 12'd360 + {2'b00, env_bar_px}, 12'd150, 12'd166)) begin
                    science_rgb = 12'h4df;
                end
                if (in_box(h_count, v_count, 12'd360, 12'd360 + {2'b00, rh_bar_px}, 12'd184, 12'd200)) begin
                    science_rgb = 12'h2d5;
                end
                if (in_box(h_count, v_count, 12'd340, 12'd610, 12'd260, 12'd410)) begin
                    science_rgb = ext_status_rgb(sun_present_pix, sun_fault_pix);
                    if ((h_count[5] ^ v_count[5]) == 1'b1) begin
                        science_rgb = 12'h245;
                    end
                end
                if (in_box(h_count, v_count, 12'd360, 12'd360 + {2'b00, sun_bar_px}, 12'd305, 12'd321)) begin
                    science_rgb = 12'hfd2;
                end
                if (in_box(h_count, v_count, 12'd360, 12'd360 + {2'b00, flow_dx_bar_px}, 12'd342, 12'd354)) begin
                    science_rgb = ext_status_rgb(flow_present_pix, flow_fault_pix);
                end
                if (in_box(h_count, v_count, 12'd360, 12'd360 + {2'b00, flow_dy_bar_px}, 12'd362, 12'd374)) begin
                    science_rgb = 12'h6af;
                end
            end

            VIEW_SCIENCE_WIND: begin
                science_rgb = 12'h021;
                if (in_box(h_count, v_count, 12'd0, 12'd640, 12'd0, 12'd30)) begin
                    science_rgb = 12'h0a6;
                end
                if (in_box(h_count, v_count, 12'd315, 12'd325, 12'd90, 12'd390) ||
                    in_box(h_count, v_count, 12'd170, 12'd470, 12'd235, 12'd245)) begin
                    science_rgb = 12'h588;
                end
                if (air_present_pix && in_box(h_count, v_count, 12'd320, 12'd320 + {2'b00, air_bar_px}, 12'd210, 12'd222)) begin
                    science_rgb = ext_status_rgb(air_present_pix, air_fault_pix);
                end
                if (flow_present_pix && in_box(h_count, v_count, 12'd320, 12'd320 + {2'b00, flow_dx_bar_px}, 12'd224, 12'd236)) begin
                    science_rgb = ext_status_rgb(flow_present_pix, flow_fault_pix);
                end
                if (nav_valid_pix && in_box(h_count, v_count, 12'd320, 12'd320 + {2'b00, nav_dx_px}, 12'd224, 12'd236)) begin
                    science_rgb = status_rgb(nav_valid_pix, nav_status_pix);
                end
                if (nav_valid_pix && in_box(h_count, v_count, 12'd308, 12'd320, 12'd240, 12'd240 + {2'b00, nav_dy_px})) begin
                    science_rgb = 12'h3d7;
                end
                if (flow_present_pix && in_box(h_count, v_count, 12'd330, 12'd342, 12'd240, 12'd240 + {2'b00, flow_dy_bar_px})) begin
                    science_rgb = 12'h6af;
                end
                if (wind_valid_pix && (wind_x_neg == 1'b0) && in_box(h_count, v_count, 12'd320, 12'd320 + {2'b00, wind_dx_px}, 12'd252, 12'd264)) begin
                    science_rgb = status_rgb(wind_valid_pix, wind_status_pix);
                end
                if (wind_valid_pix && (wind_x_neg == 1'b1) && in_box(h_count, v_count, 12'd320 - {2'b00, wind_dx_px}, 12'd320, 12'd252, 12'd264)) begin
                    science_rgb = status_rgb(wind_valid_pix, wind_status_pix);
                end
                if (wind_valid_pix && (wind_y_neg == 1'b0) && in_box(h_count, v_count, 12'd330, 12'd342, 12'd240, 12'd240 + {2'b00, wind_dy_px})) begin
                    science_rgb = 12'he82;
                end
                if (wind_valid_pix && (wind_y_neg == 1'b1) && in_box(h_count, v_count, 12'd330, 12'd342, 12'd240 - {2'b00, wind_dy_px}, 12'd240)) begin
                    science_rgb = 12'he82;
                end
                if (in_box(h_count, v_count, 12'd28, 12'd180, 12'd70, 12'd105)) begin
                    science_rgb = ext_status_rgb(air_present_pix, air_fault_pix);
                end
                if (in_box(h_count, v_count, 12'd28, 12'd180, 12'd115, 12'd150)) begin
                    science_rgb = ext_status_rgb(flow_present_pix, flow_fault_pix);
                end
                if (in_box(h_count, v_count, 12'd28, 12'd180, 12'd160, 12'd195)) begin
                    science_rgb = ext_status_rgb(range_present_pix, range_fault_pix);
                end
                if (in_box(h_count, v_count, 12'd470, 12'd615, 12'd70, 12'd205)) begin
                    science_rgb = ext_status_rgb(blackbox_present_pix,
                                                 ext_fault_flags_pix[`EXT_FLG_BLACKBOX_DROP_BIT]);
                    if ((h_count[4] ^ v_count[4]) == 1'b1) begin
                        science_rgb = 12'h333;
                    end
                end
                if (in_box(h_count, v_count, 12'd470, 12'd615, 12'd230, 12'd365)) begin
                    science_rgb = ext_status_rgb(sun_present_pix, sun_fault_pix);
                    if ((h_count[5] ^ v_count[5]) == 1'b1) begin
                        science_rgb = 12'h244;
                    end
                end
            end

            VIEW_SCIENCE_INTEGRITY: begin
                science_rgb = 12'h210;
                if (in_box(h_count, v_count, 12'd0, 12'd640, 12'd0, 12'd30)) begin
                    science_rgb = 12'hd60;
                end
                if (in_box(h_count, v_count, 12'd30, 12'd270, 12'd70, 12'd235)) begin
                    science_rgb = ext_status_rgb(mag1_present_pix, mag_fault_pix);
                    if ((h_count[5] ^ v_count[5]) == 1'b1) begin
                        science_rgb = 12'h342;
                    end
                end
                if (in_box(h_count, v_count, 12'd34, 12'd34 + {2'b00, mag_norm0_bar_px}, 12'd100, 12'd114)) begin
                    science_rgb = 12'h2d5;
                end
                if (in_box(h_count, v_count, 12'd34, 12'd34 + {2'b00, mag_norm1_bar_px}, 12'd126, 12'd140)) begin
                    science_rgb = mag1_present_pix ? 12'h4df : 12'h666;
                end
                if (in_box(h_count, v_count, 12'd34, 12'd34 + {2'b00, mag_delta_bar_px}, 12'd152, 12'd166)) begin
                    science_rgb = ext_mag_disagreement_pix ? 12'hf42 : 12'hfb2;
                end
                if (in_box(h_count, v_count, mag_sector_x, mag_sector_x + 12'd10, 12'd188, 12'd208)) begin
                    science_rgb = ext_mag_sequence_aligned_pix ? 12'hfff : 12'hfb2;
                end
                if (in_box(h_count, v_count, 12'd34, 12'd34 + {2'b00, mag_resid_bar_px}, 12'd250, 12'd266)) begin
                    science_rgb = (ext_mag_cal_state_pix == 8'h00) ? 12'hfb2 : 12'h2d5;
                end
                if (in_box(h_count, v_count, 12'd320, 12'd610, 12'd70, 12'd145)) begin
                    science_rgb = ext_status_rgb(range_present_pix, range_fault_pix);
                    if ((h_count[4] ^ v_count[4]) == 1'b1) begin
                        science_rgb = 12'h333;
                    end
                end
                if (in_box(h_count, v_count, 12'd320, 12'd610, 12'd165, 12'd240)) begin
                    science_rgb = ext_status_rgb(air_present_pix, air_fault_pix);
                    if ((h_count[5] ^ v_count[5]) == 1'b1) begin
                        science_rgb = 12'h533;
                    end
                end
                if (in_box(h_count, v_count, 12'd320, 12'd610, 12'd260, 12'd335)) begin
                    science_rgb = status_rgb(pwr_valid_pix, pwr_status_pix);
                    if ((h_count[4] ^ v_count[4]) == 1'b1) begin
                        science_rgb = 12'h853;
                    end
                end
                if (in_box(h_count, v_count, 12'd320, 12'd610, 12'd355, 12'd430)) begin
                    science_rgb = ext_status_rgb(blackbox_present_pix,
                                                 ext_fault_flags_pix[`EXT_FLG_BLACKBOX_DROP_BIT]);
                    if ((h_count[4] ^ v_count[4]) == 1'b1) begin
                        science_rgb = 12'h333;
                    end
                end
                if (in_box(h_count, v_count, 12'd330, 12'd330 + {2'b00, range_bar_px}, 12'd96, 12'd108)) begin
                    science_rgb = 12'h4df;
                end
                if (in_box(h_count, v_count, 12'd330, 12'd330 + {2'b00, air_bar_px}, 12'd190, 12'd202)) begin
                    science_rgb = 12'h2d5;
                end
                if (in_box(h_count, v_count, 12'd330, 12'd330 + {2'b00, ext_age_bar_px}, 12'd386, 12'd398)) begin
                    science_rgb = (ext_log_drop_count_pix == 16'd0) ? 12'h2d5 : 12'hf42;
                end
                if (in_box(h_count, v_count, 12'd38, 12'd220, 12'd300, 12'd314)) begin
                    science_rgb = (safety_runtime_ok_pix && safety_allows_actuation_pix) ? 12'h2d5 : 12'hd42;
                end
                if (in_box(h_count, v_count, 12'd38, 12'd220, 12'd322, 12'd336)) begin
                    science_rgb = policy_runtime_enable_pix ? 12'h2d5 : 12'h555;
                end
                if (in_box(h_count, v_count, 12'd38, 12'd220, 12'd344, 12'd358)) begin
                    science_rgb = software_armed_pix ? 12'h2d5 : 12'h555;
                end
                if (in_box(h_count, v_count, 12'd38, 12'd38 + {8'b00000000, auth_phase_code_pix}, 12'd380, 12'd396)) begin
                    science_rgb = auth_phase_valid_pix ? 12'h6af : 12'h555;
                end
            end

            default: begin
                science_rgb = 12'h000;
            end
        endcase

        if (page_id_pix == VIEW_SCIENCE_EXPLAIN) begin
            if (in_box(h_count, v_count, 12'd20, 12'd150, 12'd438, 12'd452)) begin
                science_rgb = ext_status_rgb(range_present_pix, range_fault_pix);
            end
            if (in_box(h_count, v_count, 12'd160, 12'd290, 12'd438, 12'd452)) begin
                science_rgb = ext_status_rgb(air_present_pix, air_fault_pix);
            end
            if (in_box(h_count, v_count, 12'd300, 12'd430, 12'd438, 12'd452)) begin
                science_rgb = (ext_max_age_ms_pix[15:10] == 6'd0) ? 12'h2d5 : 12'hfb2;
            end
        end

        if (page_id_pix == VIEW_SCIENCE_WIND) begin
            if (in_box(h_count, v_count, 12'd28, 12'd28 + {2'b00, ext_age_bar_px}, 12'd410, 12'd422)) begin
                science_rgb = (ext_status_pix == `ST_OK) ? 12'h4df : 12'hf42;
            end
            if (in_box(h_count, v_count, 12'd28, 12'd28 + {2'b00, flow_dx_bar_px}, 12'd432, 12'd444)) begin
                science_rgb = ext_status_rgb(flow_present_pix, flow_fault_pix);
            end
        end

        if (page_id_pix == VIEW_SCIENCE_INTEGRITY) begin
            if (in_box(h_count, v_count, 12'd38, 12'd38 + {2'b00, ext_age_bar_px}, 12'd414, 12'd426)) begin
                science_rgb = ext_status_rgb(mag1_present_pix, mag_fault_pix);
            end
            if (in_box(h_count, v_count, 12'd38, 12'd38 + {2'b00, pwr_age_ms_pix[9:0]}, 12'd436, 12'd448)) begin
                science_rgb = 12'hd83;
            end
        end
    end

    wire _unused_science_inputs_ok;
    assign _unused_science_inputs_ok =
        bmp_valid_pix ^ bmp_status_pix[0] ^ bmp_age_ms_pix[0] ^
        acc_valid_pix ^ acc_status_pix[0] ^ acc_age_ms_pix[0] ^
        mag_valid_pix ^ mag_status_pix[0] ^ mag_age_ms_pix[0] ^
        mag_payload_pix[0] ^ pwr_payload16_pix[0] ^
        der_valid_pix ^ der_alt_fresh_pix ^ der_vspd_fresh_pix ^
        der_altitude_cm_pix[0] ^ der_vertical_speed_cms_pix[0] ^
        der_bmp_age_ms_pix[0] ^ der_acc_age_ms_pix[0] ^
        der_mag_age_ms_pix[0] ^ nav_age_ms_pix[0] ^
        wind_age_ms_pix[0] ^ ext_log_seq_pix[0] ^
        ext_mag_bridge_checksum_pix[0] ^ ext_mag_source_real_pix ^
        ext_mag_source_synth_pix;

    assign vga_hsync_out = vga_hsync_in;
    assign vga_vsync_out = vga_vsync_in;
    assign vga_rgb_out = (active_pix && science_page_active) ? science_rgb : vga_rgb_in;

endmodule

`default_nettype wire
