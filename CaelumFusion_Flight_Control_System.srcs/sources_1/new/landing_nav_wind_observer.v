`timescale 1ns / 1ps
`default_nettype none

`include "telemetry_defs_vh.vh"

// Compatibility shim for the landing-page navigation/wind contract consumed by
// flight_viz_suite_top.
//
// This module deliberately does not derive navigation or wind from altitude,
// optical-flow, or airspeed proxy evidence. A real navigation/wind publication
// must come from nav_wind_snapshot_producer after explicit EKF/GNSS/wind source
// signals are present in SYS domain. Until then, the live top publishes a clear
// missing-input status so the VGA renderer degrades instead of implying a valid
// zero-wind, zero-crossrange, or altitude-as-downrange solution.
module landing_nav_wind_observer #(
    parameter [15:0] NAV_FRESH_MAX_MS  = 16'd1000,
    parameter [15:0] WIND_FRESH_MAX_MS = 16'd1000
) (
    input  wire        der_valid,
    input  wire [7:0]  der_status,
    input  wire        der_alt_fresh,
    input  wire        der_vspd_fresh,
    input  wire [15:0] der_bmp_age_ms,
    input  wire [31:0] der_altitude_cm,

    input  wire        ext_valid,
    input  wire [7:0]  ext_status,
    input  wire [15:0] ext_present_flags,
    input  wire [15:0] ext_fault_flags,
    input  wire [15:0] ext_air_speed_cms,
    input  wire [15:0] ext_flow_dx,
    input  wire [15:0] ext_flow_dy,
    input  wire [15:0] ext_max_age_ms,

    output wire        nav_valid,
    output wire [7:0]  nav_status,
    output wire [7:0]  nav_flags,
    output wire [15:0] nav_downrange_m,
    output wire [15:0] nav_crossrange_m,
    output wire [15:0] nav_age_ms,

    output wire        wind_valid,
    output wire [7:0]  wind_status,
    output wire [15:0] wind_x_cms,
    output wire [15:0] wind_y_cms,
    output wire [15:0] wind_z_cms,
    output wire [15:0] wind_age_ms
);

    // Keep all legacy inputs consumed so synthesis warnings stay focused on
    // real integration issues. These signals are intentionally not used to form
    // nav/wind outputs because they are not EKF/GNSS/wind-estimator products.
    wire _unused_legacy_inputs_ok;
    assign _unused_legacy_inputs_ok =
        NAV_FRESH_MAX_MS[0] ^ WIND_FRESH_MAX_MS[0] ^
        der_valid ^ der_status[0] ^ der_alt_fresh ^ der_vspd_fresh ^
        der_bmp_age_ms[0] ^ der_altitude_cm[0] ^
        ext_valid ^ ext_status[0] ^ ext_present_flags[0] ^
        ext_fault_flags[0] ^ ext_air_speed_cms[0] ^
        ext_flow_dx[0] ^ ext_flow_dy[0] ^ ext_max_age_ms[0];

    assign nav_valid        = 1'b0;
    assign nav_status       = `ST_MISSING_INPUT;
    assign nav_flags        = 8'h10;          // degraded/unbound contract
    assign nav_downrange_m  = 16'd0;
    assign nav_crossrange_m = 16'd0;
    assign nav_age_ms       = 16'hFFFF;

    assign wind_valid  = 1'b0;
    assign wind_status = `ST_MISSING_INPUT;
    assign wind_x_cms  = 16'd0;
    assign wind_y_cms  = 16'd0;
    assign wind_z_cms  = 16'd0;
    assign wind_age_ms = 16'hFFFF;

endmodule

`default_nettype wire
