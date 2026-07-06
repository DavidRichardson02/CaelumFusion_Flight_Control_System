`timescale 1ns / 1ps
`default_nettype none

`include "telemetry_defs_vh.vh"

//==============================================================================
// nav_wind_snapshot_producer
//------------------------------------------------------------------------------
// Real SYS-domain producer for the VGA navigation/wind contract.
//
// This module is intentionally not an EKF, GNSS parser, wind estimator, or proxy
// generator. It accepts already-produced EKF/GNSS/wind estimator evidence in the
// SYS clock domain, applies validity/status/freshness gates, and publishes the
// compact navigation/wind snapshot consumed by flight_viz_suite_top.
//
// Bind this producer only when the source signals are real:
//   * ekf_* is a navigation estimator/local-frame solution, not baro altitude.
//   * gnss_* is an explicit GPS/GNSS fix/provenance signal when required.
//   * wind_est_* is an explicit wind estimate, not raw pitot or optical flow.
//
// All output fields are reset-safe. When disabled, the producer reports
// ST_NOT_INITIALIZED. When enabled but a required source is missing, stale, or
// faulted, it reports that state and keeps nav_valid/wind_valid low.
//==============================================================================
module nav_wind_snapshot_producer #(
    parameter [15:0] NAV_FRESH_MAX_MS       = 16'd1000,
    parameter [15:0] WIND_FRESH_MAX_MS      = 16'd1000,
    parameter integer REQUIRE_GNSS_FOR_NAV  = 1
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        enable,
    input  wire        sample_event,
    input  wire [31:0] now_us,

    // EKF/local-navigation estimate in the display frame. Values are signed
    // two's-complement meters, passed as raw 16-bit fields to preserve the
    // existing VGA bundle contract.
    input  wire        ekf_valid,
    input  wire [7:0]  ekf_status,
    input  wire [15:0] ekf_seq,
    input  wire [15:0] ekf_age_ms,
    input  wire [15:0] ekf_downrange_m,
    input  wire [15:0] ekf_crossrange_m,
    input  wire [15:0] ekf_source_flags,

    // GNSS fix/provenance evidence. The producer does not interpret latitude or
    // longitude; the EKF input must already have converted position into the
    // local navigation frame.
    input  wire        gnss_valid,
    input  wire [7:0]  gnss_status,
    input  wire [15:0] gnss_seq,
    input  wire [15:0] gnss_age_ms,
    input  wire [7:0]  gnss_fix_type,
    input  wire [15:0] gnss_source_flags,

    // Wind-estimator output in cm/s. This must be an estimator product; raw
    // pitot, optical-flow, or barometric evidence should stay in ext_* banks.
    input  wire        wind_est_valid,
    input  wire [7:0]  wind_est_status,
    input  wire [15:0] wind_est_seq,
    input  wire [15:0] wind_est_age_ms,
    input  wire [15:0] wind_est_x_cms,
    input  wire [15:0] wind_est_y_cms,
    input  wire [15:0] wind_est_z_cms,
    input  wire [15:0] wind_est_source_flags,

    output reg  [31:0] nav_t_us,
    output reg  [15:0] nav_seq,
    output reg         nav_valid,
    output reg  [7:0]  nav_status,
    output reg  [7:0]  nav_flags,
    output reg  [15:0] nav_downrange_m,
    output reg  [15:0] nav_crossrange_m,
    output reg  [15:0] nav_age_ms,

    output reg  [31:0] wind_t_us,
    output reg  [15:0] wind_seq,
    output reg         wind_valid,
    output reg  [7:0]  wind_status,
    output reg  [7:0]  wind_flags,
    output reg  [15:0] wind_x_cms,
    output reg  [15:0] wind_y_cms,
    output reg  [15:0] wind_z_cms,
    output reg  [15:0] wind_age_ms
);

    function [15:0] max16;
        input [15:0] a;
        input [15:0] b;
        begin
            max16 = (a > b) ? a : b;
        end
    endfunction

    wire gnss_required_w = (REQUIRE_GNSS_FOR_NAV != 0) ? 1'b1 : 1'b0;

    wire ekf_present_w =
        ekf_valid || (ekf_status != `ST_NOT_INITIALIZED);
    wire gnss_present_w =
        gnss_valid || (gnss_status != `ST_NOT_INITIALIZED);
    wire wind_present_w =
        wind_est_valid || (wind_est_status != `ST_NOT_INITIALIZED);

    wire ekf_status_ok_w  = (ekf_status == `ST_OK);
    wire gnss_status_ok_w = (gnss_status == `ST_OK);
    wire wind_status_ok_w = (wind_est_status == `ST_OK);

    wire ekf_age_ok_w  = (ekf_age_ms <= NAV_FRESH_MAX_MS);
    wire gnss_age_ok_w = (gnss_age_ms <= NAV_FRESH_MAX_MS);
    wire wind_age_ok_w = (wind_est_age_ms <= WIND_FRESH_MAX_MS);

    wire ekf_ready_w =
        enable && ekf_valid && ekf_status_ok_w && ekf_age_ok_w;
    wire gnss_ready_w =
        !gnss_required_w ||
        (enable && gnss_valid && gnss_status_ok_w && gnss_age_ok_w);
    wire wind_ready_w =
        enable && wind_est_valid && wind_status_ok_w && wind_age_ok_w;

    wire nav_missing_w =
        !ekf_present_w || (gnss_required_w && !gnss_present_w);
    wire nav_stale_w =
        (ekf_present_w && ekf_status_ok_w && !ekf_age_ok_w) ||
        (gnss_required_w && gnss_present_w && gnss_status_ok_w && !gnss_age_ok_w);
    wire nav_fault_w =
        (ekf_present_w && !ekf_status_ok_w) ||
        (gnss_required_w && gnss_present_w && !gnss_status_ok_w);
    wire nav_ready_w =
        enable && ekf_ready_w && gnss_ready_w;

    wire wind_missing_w = !wind_present_w;
    wire wind_stale_w =
        wind_present_w && wind_status_ok_w && !wind_age_ok_w;
    wire wind_fault_w =
        wind_present_w && !wind_status_ok_w;

    wire [7:0] nav_status_next_w =
        !enable       ? `ST_NOT_INITIALIZED :
        nav_ready_w   ? `ST_OK :
        nav_missing_w ? `ST_MISSING_INPUT :
        nav_fault_w   ? (!ekf_status_ok_w ? ekf_status : gnss_status) :
        nav_stale_w   ? `ST_STALE_REJECT :
                         `ST_MISSING_INPUT;

    wire [7:0] wind_status_next_w =
        !enable        ? `ST_NOT_INITIALIZED :
        wind_ready_w   ? `ST_OK :
        wind_missing_w ? `ST_MISSING_INPUT :
        wind_fault_w   ? wind_est_status :
        wind_stale_w   ? `ST_STALE_REJECT :
                          `ST_MISSING_INPUT;

    wire [15:0] nav_age_next_w =
        !nav_ready_w ? 16'hFFFF :
        gnss_required_w ? max16(ekf_age_ms, gnss_age_ms) : ekf_age_ms;

    wire real_nav_source_bound_w =
        enable && (ekf_present_w || gnss_present_w);
    wire real_wind_source_bound_w =
        enable && wind_present_w;

    // nav_flags bit contract:
    // [0] EKF/local-navigation estimate fresh
    // [1] GNSS requirement satisfied, or GNSS not required by parameter
    // [2] wind estimator fresh
    // [3] real producer bound, not proxy-derived
    // [4] degraded / incomplete navigation publication
    // [5] navigation ready for landing viewport rendering
    // [6] GNSS required but missing/not fresh
    // [7] source fault or stale required source observed
    wire [7:0] nav_flags_next_w = enable ? {
        (nav_fault_w || nav_stale_w),
        (gnss_required_w && !gnss_ready_w),
        nav_ready_w,
        !nav_ready_w,
        real_nav_source_bound_w,
        wind_ready_w,
        gnss_ready_w,
        ekf_ready_w
    } : 8'd0;

    // wind_flags bit contract:
    // [0] wind estimator fresh
    // [1] reserved
    // [2] reserved
    // [3] real producer bound, not raw pitot/flow proxy
    // [4] degraded / incomplete wind publication
    // [5] wind ready for rendering
    // [6] reserved
    // [7] wind source fault or stale source observed
    wire [7:0] wind_flags_next_w = enable ? {
        (wind_fault_w || wind_stale_w),
        1'b0,
        wind_ready_w,
        !wind_ready_w,
        real_wind_source_bound_w,
        1'b0,
        1'b0,
        wind_ready_w
    } : 8'd0;

    wire _unused_source_metadata_ok;
    assign _unused_source_metadata_ok =
        ekf_seq[0] ^ ekf_source_flags[0] ^
        gnss_seq[0] ^ gnss_fix_type[0] ^ gnss_source_flags[0] ^
        wind_est_seq[0] ^ wind_est_source_flags[0];

    always @(posedge clk) begin
        if (rst) begin
            nav_t_us        <= 32'd0;
            nav_seq         <= 16'd0;
            nav_valid       <= 1'b0;
            nav_status      <= `ST_NOT_INITIALIZED;
            nav_flags       <= 8'd0;
            nav_downrange_m <= 16'd0;
            nav_crossrange_m<= 16'd0;
            nav_age_ms      <= 16'hFFFF;

            wind_t_us       <= 32'd0;
            wind_seq        <= 16'd0;
            wind_valid      <= 1'b0;
            wind_status     <= `ST_NOT_INITIALIZED;
            wind_flags      <= 8'd0;
            wind_x_cms      <= 16'd0;
            wind_y_cms      <= 16'd0;
            wind_z_cms      <= 16'd0;
            wind_age_ms     <= 16'hFFFF;
        end else if (!enable) begin
            nav_valid       <= 1'b0;
            nav_status      <= `ST_NOT_INITIALIZED;
            nav_flags       <= 8'd0;
            nav_downrange_m <= 16'd0;
            nav_crossrange_m<= 16'd0;
            nav_age_ms      <= 16'hFFFF;

            wind_valid      <= 1'b0;
            wind_status     <= `ST_NOT_INITIALIZED;
            wind_flags      <= 8'd0;
            wind_x_cms      <= 16'd0;
            wind_y_cms      <= 16'd0;
            wind_z_cms      <= 16'd0;
            wind_age_ms     <= 16'hFFFF;
        end else if (sample_event) begin
            nav_t_us        <= now_us;
            nav_seq         <= nav_seq + 16'd1;
            nav_valid       <= nav_ready_w;
            nav_status      <= nav_status_next_w;
            nav_flags       <= nav_flags_next_w;
            nav_downrange_m <= nav_ready_w ? ekf_downrange_m  : 16'd0;
            nav_crossrange_m<= nav_ready_w ? ekf_crossrange_m : 16'd0;
            nav_age_ms      <= nav_age_next_w;

            wind_t_us       <= now_us;
            wind_seq        <= wind_seq + 16'd1;
            wind_valid      <= wind_ready_w;
            wind_status     <= wind_status_next_w;
            wind_flags      <= wind_flags_next_w;
            wind_x_cms      <= wind_ready_w ? wind_est_x_cms : 16'd0;
            wind_y_cms      <= wind_ready_w ? wind_est_y_cms : 16'd0;
            wind_z_cms      <= wind_ready_w ? wind_est_z_cms : 16'd0;
            wind_age_ms     <= wind_ready_w ? wind_est_age_ms : 16'hFFFF;
        end
    end

endmodule

`default_nettype wire
