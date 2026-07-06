`timescale 1ns / 1ps
`default_nettype none

`include "telemetry_defs_vh.vh"

// -----------------------------------------------------------------------------
// CaelumFusion VGA render-control unification layer
// -----------------------------------------------------------------------------
// This module is the single integration point for the Phase-6 VGA-visible
// rendering path.  It keeps the existing flight HUD renderer and compass-truth
// diagnostic page intact, but removes the duplicated top-level selection wiring
// that previously allowed one raw button to enable both HUD self-test data and
// the full-screen compass page at the same time.
//
// Clocking contract:
//   * All view-control inputs are synchronous one-cycle pulses or stable levels
//     in sys_clk.  Debounce and asynchronous button synchronization belong above
//     this module.
//   * Pixel timing and RGB generation remain in the existing render blocks.
//   * The compass page block performs its existing SYS-to-PIX bundle transfer.
//
// View contract:
//   VIEW_FLIGHT_HUD     = normal telemetry HUD.
//   VIEW_COMPASS_TRUTH  = full-screen planar compass / magnetometer truth page.
//   VIEW_SELFTEST_HUD   = normal HUD renderer driven by its self-test stimulus.
//   VIEW_SENSOR_DIAG    = page-selectable HUD renderer diagnostics page.
//   VIEW_SCIENCE_*      = compact full-screen science/evidence overlays.
//   Other direct-view values are rejected and reported on cfg_invalid_view_sys.
//
// Migration intent:
//   Replace the pair of top-level instantiations of flight_viz_suite_top and
//   planar_compass_truth_page_vga with this module.  Existing VGA pins and sensor
//   data contracts are unchanged.
// -----------------------------------------------------------------------------
module caelumfusion_vga_render_control #(
    parameter integer SYS_CLK_HZ    = 100_000_000,
    parameter integer H_ACTIVE      = 640,
    parameter integer H_FP          = 16,
    parameter integer H_SYNC        = 96,
    parameter integer H_BP          = 48,
    parameter integer V_ACTIVE      = 480,
    parameter integer V_FP          = 10,
    parameter integer V_SYNC        = 2,
    parameter integer V_BP          = 33,
    parameter integer HSYNC_POL     = 0,
    parameter integer VSYNC_POL     = 0,
    parameter integer HIST_DEPTH    = 1024,
    parameter integer MAG_PLOT_SHIFT = 8,
    parameter integer ENABLE_SENSOR_DIAG_PAGE = 1,
    parameter integer ENABLE_COMPASS_TRUTH_PAGE = 0,
    parameter integer ENABLE_SCIENCE_PAGES = 1,
    parameter integer ENABLE_TELEMETRY_TEXT_OVERLAY = 1,
    parameter integer ENABLE_RENDER_STATUS_STRIP = 1,
    parameter integer COMPASS_TRUTH_PAGE_DEFAULT = 0,
    parameter [2:0]   RESET_VIEW_ID = 3'd0
) (
    input  wire        sys_clk,
    input  wire        sys_rst,

    // SYS-domain render-view controls.  Pulses must already be debounced and
    // synchronized to sys_clk.  Legacy level inputs preserve simple button-hold
    // behavior while allowing self-test and compass page selection to be split.
    input  wire        view_next_pulse_sys,
    input  wire        view_prev_pulse_sys,
    input  wire        view_direct_valid_sys,
    input  wire [2:0]  view_direct_id_sys,
    input  wire        cfg_invalid_view_clear_sys,
    input  wire        legacy_compass_page_hold_sys,
    input  wire        legacy_selftest_hold_sys,
    input  wire        history_freeze_sys,
    input  wire        direct_selector_collision_sys,

    output reg  [2:0]  view_sel_sys,
    output wire [2:0]  view_effective_sys,
    output reg         view_changed_pulse_sys,
    output reg         cfg_invalid_view_sys,
    output wire        flight_selftest_en_sys,
    output wire        compass_page_enable_sys,

    // Raw sensor snapshots in SYS domain.
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

    input  wire [15:0] mag1_seq,
    input  wire        mag1_valid,
    input  wire [7:0]  mag1_status,
    input  wire [47:0] mag1_payload,
    input  wire [15:0] mag1_age_ms,

    input  wire [31:0] pwr_t_us,
    input  wire [15:0] pwr_seq,
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

    // Derived estimator products in SYS domain.
    input  wire        der_valid,
    input  wire [7:0]  der_status,
    input  wire        der_alt_fresh,
    input  wire        der_vspd_fresh,
    input  wire        der_roll_fresh,
    input  wire        der_head_fresh,
    input  wire [15:0] der_bmp_seq_ref,
    input  wire [15:0] der_acc_seq_ref,
    input  wire [15:0] der_mag_seq_ref,
    input  wire [15:0] der_bmp_age_ms,
    input  wire [15:0] der_acc_age_ms,
    input  wire [15:0] der_mag_age_ms,
    input  wire        der_bmp_valid_ref,
    input  wire        der_acc_valid_ref,
    input  wire        der_mag_valid_ref,
    input  wire [31:0] der_altitude_cm,
    input  wire [31:0] der_vertical_speed_cms,
    input  wire [31:0] der_roll_mdeg,
    input  wire [31:0] der_heading_mdeg,

    // Navigation and wind context in SYS domain.
    input  wire        nav_valid,
    input  wire [7:0]  nav_status,
    input  wire [7:0]  nav_flags,
    input  wire [15:0] nav_downrange_m,
    input  wire [15:0] nav_crossrange_m,
    input  wire [15:0] nav_age_ms,

    input  wire        wind_valid,
    input  wire [7:0]  wind_status,
    input  wire [15:0] wind_x_cms,
    input  wire [15:0] wind_y_cms,
    input  wire [15:0] wind_z_cms,
    input  wire [15:0] wind_age_ms,

    // Authority, safety, and build metadata in SYS domain.
    input  wire [3:0]  auth_phase_code_sys,
    input  wire        auth_phase_valid_sys,
    input  wire        safety_runtime_ok_sys,
    input  wire        safety_allows_actuation_sys,
    input  wire        policy_runtime_enable_sys,
    input  wire        software_armed_sys,
    input  wire [15:0] i2c_nack_count,
    input  wire [15:0] i2c_timeout_count,
    input  wire [15:0] txn_rate_hz,
    input  wire [31:0] cdc_update_count_sys,
    input  wire [31:0] build_id,
    input  wire [15:0] schema_word,

    input  wire        pix_clk,
    input  wire        pix_rst,
    output wire        vga_hsync,
    output wire        vga_vsync,
    output wire [11:0] vga_rgb
);

    localparam [2:0] VIEW_FLIGHT_HUD    = 3'd0;
    localparam [2:0] VIEW_COMPASS_TRUTH = 3'd1;
    localparam [2:0] VIEW_SELFTEST_HUD  = 3'd2;
    localparam [2:0] VIEW_SENSOR_DIAG   = 3'd3;
    localparam [2:0] VIEW_SCIENCE_EXPLAIN   = 3'd4;
    localparam [2:0] VIEW_SCIENCE_WIND      = 3'd5;
    localparam [2:0] VIEW_SCIENCE_INTEGRITY = 3'd6;

    localparam [2:0] RESET_VIEW_SAFE =
        ((ENABLE_COMPASS_TRUTH_PAGE != 0) && (COMPASS_TRUTH_PAGE_DEFAULT != 0)) ? VIEW_COMPASS_TRUTH :
        ((RESET_VIEW_ID == VIEW_FLIGHT_HUD) ||
         ((RESET_VIEW_ID == VIEW_COMPASS_TRUTH) && (ENABLE_COMPASS_TRUTH_PAGE != 0)) ||
         (RESET_VIEW_ID == VIEW_SELFTEST_HUD) ||
         (RESET_VIEW_ID == VIEW_SENSOR_DIAG) ||
         (((RESET_VIEW_ID == VIEW_SCIENCE_EXPLAIN) ||
           (RESET_VIEW_ID == VIEW_SCIENCE_WIND) ||
           (RESET_VIEW_ID == VIEW_SCIENCE_INTEGRITY)) &&
          (ENABLE_SCIENCE_PAGES != 0))) ? RESET_VIEW_ID : VIEW_FLIGHT_HUD;

    function is_legal_view;
        input [2:0] view_id;
        begin
            case (view_id)
                VIEW_FLIGHT_HUD,
                VIEW_SELFTEST_HUD,
                VIEW_SENSOR_DIAG:  is_legal_view = 1'b1;
                VIEW_COMPASS_TRUTH: is_legal_view = (ENABLE_COMPASS_TRUTH_PAGE != 0) ? 1'b1 : 1'b0;
                VIEW_SCIENCE_EXPLAIN,
                VIEW_SCIENCE_WIND,
                VIEW_SCIENCE_INTEGRITY:
                                    is_legal_view = (ENABLE_SCIENCE_PAGES != 0) ? 1'b1 : 1'b0;
                default:           is_legal_view = 1'b0;
            endcase
        end
    endfunction

    function [2:0] next_view;
        input [2:0] view_id;
        begin
            case (view_id)
                VIEW_FLIGHT_HUD: next_view = VIEW_SENSOR_DIAG;
                VIEW_SENSOR_DIAG:
                    next_view = (ENABLE_SCIENCE_PAGES != 0) ? VIEW_SCIENCE_EXPLAIN :
                                ((ENABLE_COMPASS_TRUTH_PAGE != 0) ? VIEW_COMPASS_TRUTH : VIEW_SELFTEST_HUD);
                VIEW_SCIENCE_EXPLAIN:
                    next_view = (ENABLE_SCIENCE_PAGES != 0) ? VIEW_SCIENCE_WIND :
                                ((ENABLE_COMPASS_TRUTH_PAGE != 0) ? VIEW_COMPASS_TRUTH : VIEW_SELFTEST_HUD);
                VIEW_SCIENCE_WIND:
                    next_view = (ENABLE_SCIENCE_PAGES != 0) ? VIEW_SCIENCE_INTEGRITY :
                                ((ENABLE_COMPASS_TRUTH_PAGE != 0) ? VIEW_COMPASS_TRUTH : VIEW_SELFTEST_HUD);
                VIEW_SCIENCE_INTEGRITY:
                    next_view = (ENABLE_COMPASS_TRUTH_PAGE != 0) ? VIEW_COMPASS_TRUTH : VIEW_SELFTEST_HUD;
                VIEW_COMPASS_TRUTH: next_view = VIEW_SELFTEST_HUD;
                default:            next_view = VIEW_FLIGHT_HUD;
            endcase
        end
    endfunction

    function [2:0] prev_view;
        input [2:0] view_id;
        begin
            case (view_id)
                VIEW_SELFTEST_HUD:
                    prev_view = (ENABLE_COMPASS_TRUTH_PAGE != 0) ? VIEW_COMPASS_TRUTH :
                                ((ENABLE_SCIENCE_PAGES != 0) ? VIEW_SCIENCE_INTEGRITY : VIEW_SENSOR_DIAG);
                VIEW_COMPASS_TRUTH:
                    prev_view = (ENABLE_SCIENCE_PAGES != 0) ? VIEW_SCIENCE_INTEGRITY : VIEW_SENSOR_DIAG;
                VIEW_SCIENCE_INTEGRITY:
                    prev_view = (ENABLE_SCIENCE_PAGES != 0) ? VIEW_SCIENCE_WIND : VIEW_SENSOR_DIAG;
                VIEW_SCIENCE_WIND:
                    prev_view = (ENABLE_SCIENCE_PAGES != 0) ? VIEW_SCIENCE_EXPLAIN : VIEW_SENSOR_DIAG;
                VIEW_SCIENCE_EXPLAIN: prev_view = VIEW_SENSOR_DIAG;
                VIEW_SENSOR_DIAG:     prev_view = VIEW_FLIGHT_HUD;
                default:              prev_view = VIEW_SELFTEST_HUD;
            endcase
        end
    endfunction

    reg [2:0] requested_view;
    reg       request_valid;
    reg       request_rejected;

    always @* begin
        requested_view   = view_sel_sys;
        request_valid    = 1'b0;
        request_rejected = 1'b0;

        if (view_direct_valid_sys) begin
            request_valid = 1'b1;
            if (is_legal_view(view_direct_id_sys)) begin
                requested_view = view_direct_id_sys;
            end else begin
                requested_view   = view_sel_sys;
                request_rejected = 1'b1;
            end
        end else if (view_next_pulse_sys) begin
            request_valid  = 1'b1;
            requested_view = next_view(view_sel_sys);
        end else if (view_prev_pulse_sys) begin
            request_valid  = 1'b1;
            requested_view = prev_view(view_sel_sys);
        end
    end

    always @(posedge sys_clk) begin
        if (sys_rst) begin
            view_sel_sys            <= RESET_VIEW_SAFE;
            view_changed_pulse_sys  <= 1'b0;
            cfg_invalid_view_sys    <= 1'b0;
        end else begin
            view_changed_pulse_sys <= 1'b0;

            if (cfg_invalid_view_clear_sys) begin
                cfg_invalid_view_sys <= 1'b0;
            end

            if (request_rejected) begin
                cfg_invalid_view_sys <= 1'b1;
            end else if (request_valid && (requested_view != view_sel_sys)) begin
                view_sel_sys           <= requested_view;
                view_changed_pulse_sys <= 1'b1;
            end
        end
    end

    assign view_effective_sys = (legacy_compass_page_hold_sys && (ENABLE_COMPASS_TRUTH_PAGE != 0)) ? VIEW_COMPASS_TRUTH :
                                legacy_selftest_hold_sys     ? VIEW_SELFTEST_HUD  :
                                                              view_sel_sys;

    assign flight_selftest_en_sys  = (view_effective_sys == VIEW_SELFTEST_HUD);
    assign compass_page_enable_sys = (ENABLE_COMPASS_TRUTH_PAGE != 0) &&
                                     (view_effective_sys == VIEW_COMPASS_TRUTH);

    wire        hud_vga_hsync;
    wire        hud_vga_vsync;
    wire [11:0] hud_vga_rgb;

    flight_viz_suite_top #(
        .PAYLOAD_W (48),
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
        .HIST_DEPTH(HIST_DEPTH),
        .ENABLE_SENSOR_DIAG_PAGE(ENABLE_SENSOR_DIAG_PAGE),
        .ENABLE_TELEMETRY_TEXT_OVERLAY(ENABLE_TELEMETRY_TEXT_OVERLAY)
    ) u_flight_viz_suite_top (
        .sys_clk(sys_clk),
        .sys_rst(sys_rst),
        .bmp_t_us(bmp_t_us),
        .bmp_seq(bmp_seq),
        .bmp_valid(bmp_valid),
        .bmp_status(bmp_status),
        .bmp_payload(bmp_payload),
        .bmp_age_ms(bmp_age_ms),
        .acc_t_us(acc_t_us),
        .acc_seq(acc_seq),
        .acc_valid(acc_valid),
        .acc_status(acc_status),
        .acc_payload(acc_payload),
        .acc_age_ms(acc_age_ms),
        .mag_t_us(mag_t_us),
        .mag_seq(mag_seq),
        .mag_valid(mag_valid),
        .mag_status(mag_status),
        .mag_payload(mag_payload),
        .mag_age_ms(mag_age_ms),
        .pwr_t_us(pwr_t_us),
        .pwr_seq(pwr_seq),
        .pwr_valid(pwr_valid),
        .pwr_status(pwr_status),
        .pwr_payload(pwr_payload),
        .pwr_age_ms(pwr_age_ms),
        .ext_valid(ext_valid),
        .ext_status(ext_status),
        .ext_present_flags(ext_present_flags),
        .ext_fault_flags(ext_fault_flags),
        .ext_mag_delta_l1(ext_mag_delta_l1),
        .ext_mag_norm_primary(ext_mag_norm_primary),
        .ext_mag_norm_secondary(ext_mag_norm_secondary),
        .ext_mag_sequence_aligned(ext_mag_sequence_aligned),
        .ext_mag_disagreement(ext_mag_disagreement),
        .ext_mag_sector_delta(ext_mag_sector_delta),
        .ext_mag_source_flags(ext_mag_source_flags),
        .ext_rng_height_cm(ext_rng_height_cm),
        .ext_air_dp_pa(ext_air_dp_pa),
        .ext_air_speed_cms(ext_air_speed_cms),
        .ext_env_temp_cdeg(ext_env_temp_cdeg),
        .ext_env_rh_centi(ext_env_rh_centi),
        .ext_sun_luma(ext_sun_luma),
        .ext_flow_dx(ext_flow_dx),
        .ext_flow_dy(ext_flow_dy),
        .ext_log_seq(ext_log_seq),
        .ext_log_drop_count(ext_log_drop_count),
        .ext_max_age_ms(ext_max_age_ms),
        .der_valid(der_valid),
        .der_status(der_status),
        .der_alt_fresh(der_alt_fresh),
        .der_vspd_fresh(der_vspd_fresh),
        .der_roll_fresh(der_roll_fresh),
        .der_head_fresh(der_head_fresh),
        .der_bmp_seq_ref(der_bmp_seq_ref),
        .der_acc_seq_ref(der_acc_seq_ref),
        .der_mag_seq_ref(der_mag_seq_ref),
        .der_bmp_age_ms(der_bmp_age_ms),
        .der_acc_age_ms(der_acc_age_ms),
        .der_mag_age_ms(der_mag_age_ms),
        .der_bmp_valid_ref(der_bmp_valid_ref),
        .der_acc_valid_ref(der_acc_valid_ref),
        .der_mag_valid_ref(der_mag_valid_ref),
        .der_altitude_cm(der_altitude_cm),
        .der_vertical_speed_cms(der_vertical_speed_cms),
        .der_roll_mdeg(der_roll_mdeg),
        .der_heading_mdeg(der_heading_mdeg),
        .nav_valid(nav_valid),
        .nav_status(nav_status),
        .nav_flags(nav_flags),
        .nav_downrange_m(nav_downrange_m),
        .nav_crossrange_m(nav_crossrange_m),
        .nav_age_ms(nav_age_ms),
        .wind_valid(wind_valid),
        .wind_status(wind_status),
        .wind_x_cms(wind_x_cms),
        .wind_y_cms(wind_y_cms),
        .wind_z_cms(wind_z_cms),
        .wind_age_ms(wind_age_ms),
        .auth_phase_code_sys(auth_phase_code_sys),
        .auth_phase_valid_sys(auth_phase_valid_sys),
        .safety_runtime_ok_sys(safety_runtime_ok_sys),
        .safety_allows_actuation_sys(safety_allows_actuation_sys),
        .policy_runtime_enable_sys(policy_runtime_enable_sys),
        .software_armed_sys(software_armed_sys),
        .viz_selftest_en_sys(flight_selftest_en_sys),
        .vga_page_select_sys((view_effective_sys == VIEW_SENSOR_DIAG) ? 2'd1 : 2'd0),
        .history_freeze_sys(history_freeze_sys),
        .i2c_nack_count(i2c_nack_count),
        .i2c_timeout_count(i2c_timeout_count),
        .txn_rate_hz(txn_rate_hz),
        .cdc_update_count_sys(cdc_update_count_sys),
        .build_id(build_id),
        .schema_word(schema_word),
        .pix_clk(pix_clk),
        .pix_rst(pix_rst),
        .vga_hsync(hud_vga_hsync),
        .vga_vsync(hud_vga_vsync),
        .vga_rgb(hud_vga_rgb)
    );

    wire        compass_vga_hsync;
    wire        compass_vga_vsync;
    wire [11:0] compass_vga_rgb;
    wire        page_vga_hsync;
    wire        page_vga_vsync;
    wire [11:0] page_vga_rgb;

    generate
        if (ENABLE_COMPASS_TRUTH_PAGE != 0) begin : gen_compass_truth_page
            planar_compass_truth_page_vga #(
                .SYS_CLK_HZ(SYS_CLK_HZ),
                .UI_UPDATE_HZ(1000),
                .H_ACTIVE(H_ACTIVE),
                .H_FP(H_FP),
                .H_SYNC(H_SYNC),
                .H_BP(H_BP),
                .V_ACTIVE(V_ACTIVE),
                .V_FP(V_FP),
                .V_SYNC(V_SYNC),
                .V_BP(V_BP),
                .MAG_PLOT_SHIFT(MAG_PLOT_SHIFT),
                // The wrapper owns default view policy.  Keeping the child
                // default at 0 prevents it from overriding the HUD output.
                .COMPASS_TRUTH_PAGE_DEFAULT(0)
            ) u_planar_compass_truth_page_vga (
                .sys_clk(sys_clk),
                .sys_rst(sys_rst),
                .page_enable_sys(compass_page_enable_sys),
                .mag_seq(mag_seq),
                .mag_valid(mag_valid),
                .mag_status(mag_status),
                .mag_payload(mag_payload),
                .mag_age_ms(mag_age_ms),
                .mag1_seq(mag1_seq),
                .mag1_valid(mag1_valid),
                .mag1_status(mag1_status),
                .mag1_payload(mag1_payload),
                .mag1_age_ms(mag1_age_ms),
                .der_valid(der_valid),
                .der_status(der_status),
                .der_head_fresh(der_head_fresh),
                .der_mag_seq_ref(der_mag_seq_ref),
                .der_heading_mdeg(der_heading_mdeg),
                .ext_valid(ext_valid),
                .ext_status(ext_status),
                .ext_present_flags(ext_present_flags),
                .ext_fault_flags(ext_fault_flags),
                .ext_mag_delta_l1(ext_mag_delta_l1),
                .ext_mag_norm_primary(ext_mag_norm_primary),
                .ext_mag_norm_secondary(ext_mag_norm_secondary),
                .ext_mag_sequence_aligned(ext_mag_sequence_aligned),
                .ext_mag_disagreement(ext_mag_disagreement),
                .ext_mag_sector_delta(ext_mag_sector_delta),
                .ext_mag_norm_delta_l1(ext_mag_norm_delta_l1),
                .ext_mag_iron_residual(ext_mag_iron_residual),
                .ext_mag_cal_state(ext_mag_cal_state),
                .ext_mag_source_flags(ext_mag_source_flags),
                .ext_mag_bridge_checksum(ext_mag_bridge_checksum),
                .ext_max_age_ms(ext_max_age_ms),
                .i2c_nack_count(i2c_nack_count),
                .i2c_timeout_count(i2c_timeout_count),
                .txn_rate_hz(txn_rate_hz),
                .pix_clk(pix_clk),
                .pix_rst(pix_rst),
                .vga_hsync_in(hud_vga_hsync),
                .vga_vsync_in(hud_vga_vsync),
                .vga_rgb_in(hud_vga_rgb),
                .vga_hsync_out(compass_vga_hsync),
                .vga_vsync_out(compass_vga_vsync),
                .vga_rgb_out(compass_vga_rgb)
            );
        end else begin : gen_no_compass_truth_page
            assign compass_vga_hsync = hud_vga_hsync;
            assign compass_vga_vsync = hud_vga_vsync;
            assign compass_vga_rgb   = hud_vga_rgb;
        end
    endgenerate

    generate
        if (ENABLE_SCIENCE_PAGES != 0) begin : gen_science_pages
            caelumfusion_science_page_vga #(
                .H_ACTIVE(H_ACTIVE),
                .H_FP(H_FP),
                .H_SYNC(H_SYNC),
                .H_BP(H_BP),
                .V_ACTIVE(V_ACTIVE),
                .V_FP(V_FP),
                .V_SYNC(V_SYNC),
                .V_BP(V_BP)
            ) u_caelumfusion_science_page_vga (
                .sys_clk(sys_clk),
                .sys_rst(sys_rst),
                .page_id_sys(view_effective_sys),
                .bmp_valid(bmp_valid),
                .bmp_status(bmp_status),
                .bmp_age_ms(bmp_age_ms),
                .acc_valid(acc_valid),
                .acc_status(acc_status),
                .acc_age_ms(acc_age_ms),
                .mag_valid(mag_valid),
                .mag_status(mag_status),
                .mag_payload(mag_payload),
                .mag_age_ms(mag_age_ms),
                .pwr_valid(pwr_valid),
                .pwr_status(pwr_status),
                .pwr_payload(pwr_payload),
                .pwr_age_ms(pwr_age_ms),
                .ext_valid(ext_valid),
                .ext_status(ext_status),
                .ext_present_flags(ext_present_flags),
                .ext_fault_flags(ext_fault_flags),
                .ext_mag_delta_l1(ext_mag_delta_l1),
                .ext_mag_norm_primary(ext_mag_norm_primary),
                .ext_mag_norm_secondary(ext_mag_norm_secondary),
                .ext_mag_sequence_aligned(ext_mag_sequence_aligned),
                .ext_mag_disagreement(ext_mag_disagreement),
                .ext_mag_sector_delta(ext_mag_sector_delta),
                .ext_mag_norm_delta_l1(ext_mag_norm_delta_l1),
                .ext_mag_iron_residual(ext_mag_iron_residual),
                .ext_mag_cal_state(ext_mag_cal_state),
                .ext_mag_source_flags(ext_mag_source_flags),
                .ext_mag_bridge_checksum(ext_mag_bridge_checksum),
                .ext_rng_height_cm(ext_rng_height_cm),
                .ext_air_dp_pa(ext_air_dp_pa),
                .ext_air_speed_cms(ext_air_speed_cms),
                .ext_env_temp_cdeg(ext_env_temp_cdeg),
                .ext_env_rh_centi(ext_env_rh_centi),
                .ext_sun_luma(ext_sun_luma),
                .ext_flow_dx(ext_flow_dx),
                .ext_flow_dy(ext_flow_dy),
                .ext_log_seq(ext_log_seq),
                .ext_log_drop_count(ext_log_drop_count),
                .ext_max_age_ms(ext_max_age_ms),
                .der_valid(der_valid),
                .der_status(der_status),
                .der_alt_fresh(der_alt_fresh),
                .der_vspd_fresh(der_vspd_fresh),
                .der_bmp_age_ms(der_bmp_age_ms),
                .der_acc_age_ms(der_acc_age_ms),
                .der_mag_age_ms(der_mag_age_ms),
                .der_altitude_cm(der_altitude_cm),
                .der_vertical_speed_cms(der_vertical_speed_cms),
                .nav_valid(nav_valid),
                .nav_status(nav_status),
                .nav_downrange_m(nav_downrange_m),
                .nav_crossrange_m(nav_crossrange_m),
                .nav_age_ms(nav_age_ms),
                .wind_valid(wind_valid),
                .wind_status(wind_status),
                .wind_x_cms(wind_x_cms),
                .wind_y_cms(wind_y_cms),
                .wind_age_ms(wind_age_ms),
                .auth_phase_code_sys(auth_phase_code_sys),
                .auth_phase_valid_sys(auth_phase_valid_sys),
                .safety_runtime_ok_sys(safety_runtime_ok_sys),
                .safety_allows_actuation_sys(safety_allows_actuation_sys),
                .policy_runtime_enable_sys(policy_runtime_enable_sys),
                .software_armed_sys(software_armed_sys),
                .pix_clk(pix_clk),
                .pix_rst(pix_rst),
                .vga_hsync_in(compass_vga_hsync),
                .vga_vsync_in(compass_vga_vsync),
                .vga_rgb_in(compass_vga_rgb),
                .vga_hsync_out(page_vga_hsync),
                .vga_vsync_out(page_vga_vsync),
                .vga_rgb_out(page_vga_rgb)
            );
        end else begin : gen_no_science_pages
            assign page_vga_hsync = compass_vga_hsync;
            assign page_vga_vsync = compass_vga_vsync;
            assign page_vga_rgb   = compass_vga_rgb;
        end
    endgenerate

    // All-pages health strip. Each source cell is a canonical 64-bit telemetry
    // atom: {seq, age_ms, status, source_id, flags, tag}. The strip is drawn
    // after HUD, compass, and science pages, so it remains visible on every
    // renderable page without duplicating page-local health logic.
    localparam [15:0] PWR_FRESH_MAX_MS  = 16'd500;
    localparam [15:0] EXT_FRESH_MAX_MS  = 16'd1000;
    localparam [15:0] NAV_FRESH_MAX_MS  = 16'd1000;
    localparam [15:0] WIND_FRESH_MAX_MS = 16'd1000;

    localparam integer STATUS_CTRL_W = 5;
    localparam integer STATUS_ATOM_COUNT = 8;
    localparam integer STATUS_BUNDLE_W =
        STATUS_CTRL_W + (STATUS_ATOM_COUNT * `TELEM_ATOM_W);

    localparam integer STATUS_BMP_LSB  = STATUS_CTRL_W;
    localparam integer STATUS_BMP_MSB  = STATUS_BMP_LSB + `TELEM_ATOM_W - 1;
    localparam integer STATUS_ACC_LSB  = STATUS_BMP_MSB + 1;
    localparam integer STATUS_ACC_MSB  = STATUS_ACC_LSB + `TELEM_ATOM_W - 1;
    localparam integer STATUS_MAG_LSB  = STATUS_ACC_MSB + 1;
    localparam integer STATUS_MAG_MSB  = STATUS_MAG_LSB + `TELEM_ATOM_W - 1;
    localparam integer STATUS_DER_LSB  = STATUS_MAG_MSB + 1;
    localparam integer STATUS_DER_MSB  = STATUS_DER_LSB + `TELEM_ATOM_W - 1;
    localparam integer STATUS_PWR_LSB  = STATUS_DER_MSB + 1;
    localparam integer STATUS_PWR_MSB  = STATUS_PWR_LSB + `TELEM_ATOM_W - 1;
    localparam integer STATUS_EXT_LSB  = STATUS_PWR_MSB + 1;
    localparam integer STATUS_EXT_MSB  = STATUS_EXT_LSB + `TELEM_ATOM_W - 1;
    localparam integer STATUS_NAV_LSB  = STATUS_EXT_MSB + 1;
    localparam integer STATUS_NAV_MSB  = STATUS_NAV_LSB + `TELEM_ATOM_W - 1;
    localparam integer STATUS_WIND_LSB = STATUS_NAV_MSB + 1;
    localparam integer STATUS_WIND_MSB = STATUS_WIND_LSB + `TELEM_ATOM_W - 1;

    wire [STATUS_BUNDLE_W-1:0] render_status_bundle_pix;
    function [15:0] max3_u16;
        input [15:0] a;
        input [15:0] b;
        input [15:0] c;
        reg [15:0] ab;
        begin
            ab = (a > b) ? a : b;
            max3_u16 = (ab > c) ? ab : c;
        end
    endfunction

    function [7:0] atom_flags;
        input       valid;
        input [7:0] status;
        input       fresh;
        input       saturated;
        input       degraded_extra;
        begin
            atom_flags = 8'd0;
            atom_flags[`FLG_VALID_BIT]     = valid;
            atom_flags[`FLG_FRESH_BIT]     = fresh;
            atom_flags[`FLG_SATURATED_BIT] = saturated;
            atom_flags[`FLG_DEGRADED_BIT]  =
                (!valid) || (status != `ST_OK) || (!fresh) || degraded_extra;
        end
    endfunction

    function [`TELEM_ATOM_W-1:0] pack_atom;
        input [15:0] seq;
        input [15:0] age_ms;
        input [7:0]  status;
        input [7:0]  source;
        input [7:0]  flags;
        input [7:0]  tag;
        begin
            pack_atom = {`TELEM_ATOM_W{1'b0}};
            pack_atom[`TELEM_ATOM_SEQ_MSB:`TELEM_ATOM_SEQ_LSB] = seq;
            pack_atom[`TELEM_ATOM_AGE_MSB:`TELEM_ATOM_AGE_LSB] = age_ms;
            pack_atom[`TELEM_ATOM_STATUS_MSB:`TELEM_ATOM_STATUS_LSB] = status;
            pack_atom[`TELEM_ATOM_SOURCE_MSB:`TELEM_ATOM_SOURCE_LSB] = source;
            pack_atom[`TELEM_ATOM_FLAGS_MSB:`TELEM_ATOM_FLAGS_LSB] = flags;
            pack_atom[`TELEM_ATOM_TAG_MSB:`TELEM_ATOM_TAG_LSB] = tag;
        end
    endfunction

    wire [15:0] der_max_age_ms_sys =
        max3_u16(der_bmp_age_ms, der_acc_age_ms, der_mag_age_ms);
    wire        der_all_fresh_sys =
        der_alt_fresh && der_vspd_fresh && der_roll_fresh && der_head_fresh;

    wire [`TELEM_ATOM_W-1:0] bmp_atom_sys =
        pack_atom(bmp_seq, bmp_age_ms, bmp_status, `SRC_BMP58X_RAW,
                  atom_flags(bmp_valid, bmp_status,
                             bmp_valid && (bmp_status == `ST_OK) &&
                             (bmp_age_ms < `BMP_FRESH_MAX_MS),
                             (bmp_age_ms == 16'hFFFF), 1'b0),
                  `SENSOR_TAG_BMP);
    wire [`TELEM_ATOM_W-1:0] acc_atom_sys =
        pack_atom(acc_seq, acc_age_ms, acc_status, `SRC_LIS3DH_RAW,
                  atom_flags(acc_valid, acc_status,
                             acc_valid && (acc_status == `ST_OK) &&
                             (acc_age_ms < `ACC_FRESH_MAX_MS),
                             (acc_age_ms == 16'hFFFF), 1'b0),
                  `SENSOR_TAG_ACC);
    wire [`TELEM_ATOM_W-1:0] mag_atom_sys =
        pack_atom(mag_seq, mag_age_ms, mag_status, `SRC_LIS2MDL_RAW,
                  atom_flags(mag_valid, mag_status,
                             mag_valid && (mag_status == `ST_OK) &&
                             (mag_age_ms < `MAG_FRESH_MAX_MS),
                             (mag_age_ms == 16'hFFFF), 1'b0),
                  `SENSOR_TAG_MAG);
    wire [`TELEM_ATOM_W-1:0] der_atom_sys =
        pack_atom(der_bmp_seq_ref ^ der_acc_seq_ref ^ der_mag_seq_ref,
                  der_max_age_ms_sys, der_status, `SRC_DERIVED_STATE,
                  atom_flags(der_valid, der_status,
                             der_valid && (der_status == `ST_OK) &&
                             der_all_fresh_sys,
                             (der_max_age_ms_sys == 16'hFFFF), 1'b0),
                  `SENSOR_TAG_DER);
    wire [`TELEM_ATOM_W-1:0] pwr_atom_sys =
        pack_atom(pwr_seq, pwr_age_ms, pwr_status, `SRC_SYSTEM_HEALTH,
                  atom_flags(pwr_valid, pwr_status,
                             pwr_valid && (pwr_status == `ST_OK) &&
                             (pwr_age_ms < PWR_FRESH_MAX_MS),
                             (pwr_age_ms == 16'hFFFF), 1'b0),
                  `SENSOR_TAG_PWR);
    wire [`TELEM_ATOM_W-1:0] ext_atom_sys =
        pack_atom(ext_log_seq, ext_max_age_ms, ext_status,
                  `SRC_MAG_REDUNDANCY_EVID,
                  atom_flags(ext_valid, ext_status,
                             ext_valid && (ext_status == `ST_OK) &&
                             (ext_max_age_ms < EXT_FRESH_MAX_MS),
                             (ext_max_age_ms == 16'hFFFF),
                             (ext_fault_flags != 16'd0)),
                  `SENSOR_TAG_EXT);
    wire [`TELEM_ATOM_W-1:0] nav_atom_sys =
        pack_atom(nav_downrange_m ^ nav_crossrange_m, nav_age_ms, nav_status,
                  `SRC_EKF_ESTIMATE,
                  atom_flags(nav_valid, nav_status,
                             nav_valid && (nav_status == `ST_OK) &&
                             (nav_age_ms < NAV_FRESH_MAX_MS),
                             (nav_age_ms == 16'hFFFF), 1'b0),
                  8'h4E);
    wire [`TELEM_ATOM_W-1:0] wind_atom_sys =
        pack_atom(wind_x_cms ^ wind_y_cms ^ wind_z_cms, wind_age_ms,
                  wind_status, `SRC_WIND_ESTIMATE,
                  atom_flags(wind_valid, wind_status,
                             wind_valid && (wind_status == `ST_OK) &&
                             (wind_age_ms < WIND_FRESH_MAX_MS),
                             (wind_age_ms == 16'hFFFF), 1'b0),
                  8'h57);

    wire [4:0] render_status_ctrl_sys = {
        view_effective_sys,
        cfg_invalid_view_sys,
        direct_selector_collision_sys
    };

    wire [STATUS_BUNDLE_W-1:0] render_status_bundle_sys = {
        wind_atom_sys,
        nav_atom_sys,
        ext_atom_sys,
        pwr_atom_sys,
        der_atom_sys,
        mag_atom_sys,
        acc_atom_sys,
        bmp_atom_sys,
        render_status_ctrl_sys
    };

    reg [STATUS_BUNDLE_W-1:0] render_status_bundle_hold_sys;
    reg [31:0]                render_status_key_sys;

    always @(posedge sys_clk) begin
        if (sys_rst) begin
            render_status_bundle_hold_sys <= {STATUS_BUNDLE_W{1'b0}};
            render_status_key_sys         <= 32'd0;
        end else if (render_status_bundle_sys != render_status_bundle_hold_sys) begin
            render_status_bundle_hold_sys <= render_status_bundle_sys;
            render_status_key_sys         <= render_status_key_sys + 32'd1;
        end
    end

    wire       render_status_update_pix;

    cdc_bundle_toggle_2way #(
        .W(STATUS_BUNDLE_W)
    ) u_render_status_cdc (
        .src_clk    (sys_clk),
        .src_rst    (sys_rst),
        .src_bundle (render_status_bundle_hold_sys),
        .src_key    (render_status_key_sys),
        .dst_clk    (pix_clk),
        .dst_rst    (pix_rst),
        .dst_bundle (render_status_bundle_pix),
        .dst_update (render_status_update_pix)
    );

    wire        status_timing_hsync;
    wire        status_timing_vsync;
    wire        status_active_pix;
    wire [10:0] status_x_pix;
    wire [10:0] status_y_pix;
    wire        status_vsync_edge;

    vga_timing_640x480_60 #(
        .H_ACTIVE  (H_ACTIVE),
        .H_FP      (H_FP),
        .H_SYNC    (H_SYNC),
        .H_BP      (H_BP),
        .V_ACTIVE  (V_ACTIVE),
        .V_FP      (V_FP),
        .V_SYNC    (V_SYNC),
        .V_BP      (V_BP),
        .HSYNC_POL (HSYNC_POL),
        .VSYNC_POL (VSYNC_POL)
    ) u_render_status_timing (
        .clk        (pix_clk),
        .rst        (pix_rst),
        .hsync      (status_timing_hsync),
        .vsync      (status_timing_vsync),
        .active     (status_active_pix),
        .x          (status_x_pix),
        .y          (status_y_pix),
        .vsync_edge (status_vsync_edge)
    );

    wire _unused_render_status_timing_ok =
        status_timing_hsync ^ status_timing_vsync ^
        status_vsync_edge ^ render_status_update_pix;

    function status_in_box;
        input [10:0] px;
        input [10:0] py;
        input integer x0;
        input integer x1;
        input integer y0;
        input integer y1;
        begin
            status_in_box = (px >= x0[10:0]) && (px < x1[10:0]) &&
                            (py >= y0[10:0]) && (py < y1[10:0]);
        end
    endfunction

    wire [4:0] render_status_ctrl_pix =
        render_status_bundle_pix[STATUS_CTRL_W-1:0];
    wire [`TELEM_ATOM_W-1:0] bmp_atom_pix =
        render_status_bundle_pix[STATUS_BMP_MSB:STATUS_BMP_LSB];
    wire [`TELEM_ATOM_W-1:0] acc_atom_pix =
        render_status_bundle_pix[STATUS_ACC_MSB:STATUS_ACC_LSB];
    wire [`TELEM_ATOM_W-1:0] mag_atom_pix =
        render_status_bundle_pix[STATUS_MAG_MSB:STATUS_MAG_LSB];
    wire [`TELEM_ATOM_W-1:0] der_atom_pix =
        render_status_bundle_pix[STATUS_DER_MSB:STATUS_DER_LSB];
    wire [`TELEM_ATOM_W-1:0] pwr_atom_pix =
        render_status_bundle_pix[STATUS_PWR_MSB:STATUS_PWR_LSB];
    wire [`TELEM_ATOM_W-1:0] ext_atom_pix =
        render_status_bundle_pix[STATUS_EXT_MSB:STATUS_EXT_LSB];
    wire [`TELEM_ATOM_W-1:0] nav_atom_pix =
        render_status_bundle_pix[STATUS_NAV_MSB:STATUS_NAV_LSB];
    wire [`TELEM_ATOM_W-1:0] wind_atom_pix =
        render_status_bundle_pix[STATUS_WIND_MSB:STATUS_WIND_LSB];

    function [11:0] atom_rgb;
        input [`TELEM_ATOM_W-1:0] atom;
        reg [7:0] status;
        reg [7:0] flags;
        begin
            status = atom[`TELEM_ATOM_STATUS_MSB:`TELEM_ATOM_STATUS_LSB];
            flags  = atom[`TELEM_ATOM_FLAGS_MSB:`TELEM_ATOM_FLAGS_LSB];

            if (!flags[`FLG_VALID_BIT])
                atom_rgb = 12'h555;
            else if (status != `ST_OK)
                atom_rgb = 12'hf42;
            else if (flags[`FLG_DEGRADED_BIT])
                atom_rgb = 12'hfb2;
            else if (!flags[`FLG_FRESH_BIT])
                atom_rgb = 12'hd83;
            else
                atom_rgb = 12'h2d5;
        end
    endfunction

    localparam integer STATUS_X0 = H_ACTIVE - 252;
    localparam integer STATUS_X1 = H_ACTIVE - 4;
    localparam integer STATUS_Y0 = V_ACTIVE - 14;
    localparam integer STATUS_Y1 = V_ACTIVE - 2;

    localparam integer STATUS_CELL_W = 10;
    localparam integer STATUS_CELL_G = 4;
    localparam integer STATUS_CELL_0 = STATUS_X0 + 6;
    localparam integer STATUS_CELL_1 = STATUS_CELL_0 + STATUS_CELL_W + STATUS_CELL_G;
    localparam integer STATUS_CELL_2 = STATUS_CELL_1 + STATUS_CELL_W + STATUS_CELL_G;
    localparam integer STATUS_CELL_3 = STATUS_CELL_2 + STATUS_CELL_W + STATUS_CELL_G;
    localparam integer STATUS_CELL_4 = STATUS_CELL_3 + STATUS_CELL_W + STATUS_CELL_G;
    localparam integer STATUS_CELL_5 = STATUS_CELL_4 + STATUS_CELL_W + STATUS_CELL_G;
    localparam integer STATUS_CELL_6 = STATUS_CELL_5 + STATUS_CELL_W + STATUS_CELL_G;
    localparam integer STATUS_CELL_7 = STATUS_CELL_6 + STATUS_CELL_W + STATUS_CELL_G;
    localparam integer STATUS_CTRL_0 = STATUS_CELL_7 + STATUS_CELL_W + 14;
    localparam integer STATUS_CTRL_1 = STATUS_CTRL_0 + STATUS_CELL_W + STATUS_CELL_G;
    localparam integer STATUS_CTRL_2 = STATUS_CTRL_1 + STATUS_CELL_W + STATUS_CELL_G;
    localparam integer STATUS_CTRL_3 = STATUS_CTRL_2 + STATUS_CELL_W + 10;
    localparam integer STATUS_CTRL_4 = STATUS_CTRL_3 + 12 + 6;

    reg        status_overlay_en;
    reg [11:0] status_overlay_rgb;

    always @* begin
        status_overlay_en  = 1'b0;
        status_overlay_rgb = page_vga_rgb;

        if ((ENABLE_RENDER_STATUS_STRIP != 0) && status_active_pix) begin
            if (status_in_box(status_x_pix, status_y_pix,
                              STATUS_X0, STATUS_X1, STATUS_Y0, STATUS_Y1)) begin
                status_overlay_en  = 1'b1;
                status_overlay_rgb = 12'h111;
            end

            if (status_in_box(status_x_pix, status_y_pix,
                              STATUS_X0, STATUS_X1, STATUS_Y0, STATUS_Y0 + 1) ||
                status_in_box(status_x_pix, status_y_pix,
                              STATUS_X0, STATUS_X1, STATUS_Y1 - 1, STATUS_Y1) ||
                status_in_box(status_x_pix, status_y_pix,
                              STATUS_X0, STATUS_X0 + 1, STATUS_Y0, STATUS_Y1) ||
                status_in_box(status_x_pix, status_y_pix,
                              STATUS_X1 - 1, STATUS_X1, STATUS_Y0, STATUS_Y1)) begin
                status_overlay_en  = 1'b1;
                status_overlay_rgb = 12'h888;
            end

            if (status_in_box(status_x_pix, status_y_pix,
                              STATUS_CELL_0, STATUS_CELL_0 + STATUS_CELL_W,
                              STATUS_Y0 + 3, STATUS_Y1 - 3)) begin
                status_overlay_en  = 1'b1;
                status_overlay_rgb = atom_rgb(bmp_atom_pix);
            end
            if (status_in_box(status_x_pix, status_y_pix,
                              STATUS_CELL_1, STATUS_CELL_1 + STATUS_CELL_W,
                              STATUS_Y0 + 3, STATUS_Y1 - 3)) begin
                status_overlay_en  = 1'b1;
                status_overlay_rgb = atom_rgb(acc_atom_pix);
            end
            if (status_in_box(status_x_pix, status_y_pix,
                              STATUS_CELL_2, STATUS_CELL_2 + STATUS_CELL_W,
                              STATUS_Y0 + 3, STATUS_Y1 - 3)) begin
                status_overlay_en  = 1'b1;
                status_overlay_rgb = atom_rgb(mag_atom_pix);
            end
            if (status_in_box(status_x_pix, status_y_pix,
                              STATUS_CELL_3, STATUS_CELL_3 + STATUS_CELL_W,
                              STATUS_Y0 + 3, STATUS_Y1 - 3)) begin
                status_overlay_en  = 1'b1;
                status_overlay_rgb = atom_rgb(der_atom_pix);
            end
            if (status_in_box(status_x_pix, status_y_pix,
                              STATUS_CELL_4, STATUS_CELL_4 + STATUS_CELL_W,
                              STATUS_Y0 + 3, STATUS_Y1 - 3)) begin
                status_overlay_en  = 1'b1;
                status_overlay_rgb = atom_rgb(pwr_atom_pix);
            end
            if (status_in_box(status_x_pix, status_y_pix,
                              STATUS_CELL_5, STATUS_CELL_5 + STATUS_CELL_W,
                              STATUS_Y0 + 3, STATUS_Y1 - 3)) begin
                status_overlay_en  = 1'b1;
                status_overlay_rgb = atom_rgb(ext_atom_pix);
            end
            if (status_in_box(status_x_pix, status_y_pix,
                              STATUS_CELL_6, STATUS_CELL_6 + STATUS_CELL_W,
                              STATUS_Y0 + 3, STATUS_Y1 - 3)) begin
                status_overlay_en  = 1'b1;
                status_overlay_rgb = atom_rgb(nav_atom_pix);
            end
            if (status_in_box(status_x_pix, status_y_pix,
                              STATUS_CELL_7, STATUS_CELL_7 + STATUS_CELL_W,
                              STATUS_Y0 + 3, STATUS_Y1 - 3)) begin
                status_overlay_en  = 1'b1;
                status_overlay_rgb = atom_rgb(wind_atom_pix);
            end

            if (status_in_box(status_x_pix, status_y_pix,
                              STATUS_CTRL_0, STATUS_CTRL_0 + STATUS_CELL_W,
                              STATUS_Y0 + 3, STATUS_Y1 - 3)) begin
                status_overlay_en  = 1'b1;
                status_overlay_rgb = render_status_ctrl_pix[4] ? 12'h2d5 : 12'h333;
            end
            if (status_in_box(status_x_pix, status_y_pix,
                              STATUS_CTRL_1, STATUS_CTRL_1 + STATUS_CELL_W,
                              STATUS_Y0 + 3, STATUS_Y1 - 3)) begin
                status_overlay_en  = 1'b1;
                status_overlay_rgb = render_status_ctrl_pix[3] ? 12'h2d5 : 12'h333;
            end
            if (status_in_box(status_x_pix, status_y_pix,
                              STATUS_CTRL_2, STATUS_CTRL_2 + STATUS_CELL_W,
                              STATUS_Y0 + 3, STATUS_Y1 - 3)) begin
                status_overlay_en  = 1'b1;
                status_overlay_rgb = render_status_ctrl_pix[2] ? 12'h2d5 : 12'h333;
            end
            if (status_in_box(status_x_pix, status_y_pix,
                              STATUS_CTRL_3, STATUS_CTRL_3 + 12,
                              STATUS_Y0 + 3, STATUS_Y1 - 3)) begin
                status_overlay_en  = 1'b1;
                status_overlay_rgb = render_status_ctrl_pix[1] ? 12'hf22 : 12'h311;
            end
            if (status_in_box(status_x_pix, status_y_pix,
                              STATUS_CTRL_4, STATUS_CTRL_4 + 12,
                              STATUS_Y0 + 3, STATUS_Y1 - 3)) begin
                status_overlay_en  = 1'b1;
                status_overlay_rgb = render_status_ctrl_pix[0] ? 12'hfb0 : 12'h331;
            end
        end
    end

    assign vga_hsync = page_vga_hsync;
    assign vga_vsync = page_vga_vsync;
    assign vga_rgb   = status_overlay_en ? status_overlay_rgb : page_vga_rgb;

endmodule

//==============================================================================
// caelumfusion_vga_direct_view_arbiter_sys
//------------------------------------------------------------------------------
// Resolves the board-level SW11-SW13 control collision before requests reach the
// registered render-control wrapper.  In encoded mode, BTNR may sample the
// switch triplet only when MAG1 bench publication and deliberate diagnostic
// fault injection are both inactive.  During either bench/diagnostic ownership
// mode the BTNR pulse is preserved, but the requested ID is forced to the
// reserved invalid code so the wrapper holds view_sel_sys and latches its
// existing cfg_invalid_view_sys diagnostic.
//==============================================================================
module caelumfusion_vga_direct_view_arbiter_sys #(
    parameter integer USE_SWITCH_ENCODED_VIEW_SELECT = 1
)(
    input  wire       direct_button_pulse_sys,
    input  wire       sw_mag1_bench_level_sys,
    input  wire       diag_fault_inject_sys,
    input  wire       sw_view_id0_level_sys,
    input  wire       sw_view_id1_level_sys,
    input  wire       sw_view_id2_level_sys,
    output wire       view_direct_valid_sys,
    output wire [2:0] view_direct_id_sys,
    output wire       selector_collision_sys
);
    localparam [2:0] VIEW_COMPASS_TRUTH    = 3'd1;
    localparam [2:0] VIEW_RESERVED_INVALID = 3'd7;

    wire encoded_select_enabled_w =
        (USE_SWITCH_ENCODED_VIEW_SELECT != 0) ? 1'b1 : 1'b0;

    assign selector_collision_sys =
        encoded_select_enabled_w &&
        (sw_mag1_bench_level_sys || diag_fault_inject_sys);

    assign view_direct_valid_sys = direct_button_pulse_sys;
    assign view_direct_id_sys =
        !encoded_select_enabled_w ? VIEW_COMPASS_TRUTH :
        selector_collision_sys    ? VIEW_RESERVED_INVALID :
                                    {sw_view_id2_level_sys,
                                     sw_view_id1_level_sys,
                                     sw_view_id0_level_sys};
endmodule

`default_nettype wire
