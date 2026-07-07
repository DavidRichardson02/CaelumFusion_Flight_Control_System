`timescale 1ns/1ps
`default_nettype none

`include "telemetry_defs_vh.vh"

//==============================================================================
// kalman_alt2_fixed_sys
//------------------------------------------------------------------------------
// Synthesizable fixed-point two-state altitude Kalman core.
//
// This module begins the RTL implementation of the verified CaelumSufflamen
// KfAlt2State contract.  It uses integer project units rather than floating
// point:
//
//   h_cm      altitude state, centimeters
//   v_cms     vertical-speed state, centimeters/second
//   a_cms2    vertical acceleration input, centimeters/second^2
//   P00       altitude variance, centimeters^2
//   P01/P10   altitude/velocity covariance, centimeters^2/second
//   P11       velocity variance, centimeters^2/second^2
//
// One command is consumed per clock in this priority order:
//   reset > seed > predict > update.
//
// The update step implements a scalar Joseph-form covariance correction using
// Q16.16 Kalman gains.  It is bounded and synthesizable, but intentionally not
// yet pipelined for a 100 MHz timing claim.  Integrating it into the live
// CaelumFusion derived-state path should add pipeline staging or a multicycle
// microsequencer if timing/resource reports require it.
//==============================================================================
module kalman_alt2_fixed_sys #(
    parameter signed [31:0] INITIAL_P00_CM2  = 32'sd10000,
    parameter signed [31:0] INITIAL_P11_CM2S2 = 32'sd10000,
    parameter signed [31:0] MEAS_R_CM2       = 32'sd571,
    parameter signed [31:0] PROCESS_Q00_MIN_CM2 = 32'sd1,
    parameter signed [31:0] PROCESS_Q11_MIN_CM2S2 = 32'sd1,
    parameter [15:0]        MAX_DT_MS        = 16'd100
)(
    input  wire                 clk,
    input  wire                 rst,

    input  wire                 seed_valid,
    input  wire signed [31:0]   seed_h_cm,

    input  wire                 predict_valid,
    input  wire signed [31:0]   predict_a_cms2,
    input  wire [15:0]          predict_dt_ms,

    input  wire                 update_valid,
    input  wire signed [31:0]   update_z_cm,

    output reg                  est_valid,
    output reg                  est_seeded,
    output reg [7:0]            est_status,
    output reg signed [31:0]    est_h_cm,
    output reg signed [31:0]    est_v_cms,
    output reg signed [31:0]    est_a_cms2,
    output reg signed [31:0]    est_P00_cm2,
    output reg signed [31:0]    est_P01_cm2s,
    output reg signed [31:0]    est_P10_cm2s,
    output reg signed [31:0]    est_P11_cm2s2
);

    localparam signed [63:0] Q16_ONE = 64'sd65536;

    function signed [31:0] sat_s32;
        input signed [63:0] v;
        begin
            if (v > 64'sd2147483647)
                sat_s32 = 32'sh7FFF_FFFF;
            else if (v < -64'sd2147483648)
                sat_s32 = 32'sh8000_0000;
            else
                sat_s32 = v[31:0];
        end
    endfunction

    function signed [31:0] floor_nonneg_s32;
        input signed [63:0] v;
        begin
            if (v <= 64'sd0)
                floor_nonneg_s32 = 32'sd0;
            else if (v > 64'sd2147483647)
                floor_nonneg_s32 = 32'sh7FFF_FFFF;
            else
                floor_nonneg_s32 = v[31:0];
        end
    endfunction

    wire dt_ok_w = (predict_dt_ms != 16'd0) && (predict_dt_ms <= MAX_DT_MS);

    // Kinematic prediction in integer centimeters.
    wire signed [63:0] dt_ms_s64 = {48'd0, predict_dt_ms};
    wire signed [63:0] dt2_ms2_s64 = dt_ms_s64 * dt_ms_s64;

    wire signed [63:0] dh_v_cm_s64 =
        ($signed(est_v_cms) * dt_ms_s64) / 64'sd1000;
    wire signed [63:0] dh_a_cm_s64 =
        ($signed(predict_a_cms2) * dt2_ms2_s64) / 64'sd2000000;
    wire signed [63:0] dv_cms_s64 =
        ($signed(predict_a_cms2) * dt_ms_s64) / 64'sd1000;

    // Covariance prediction for F=[1 dt;0 1].  Q is deliberately conservative
    // and floored so covariance never becomes mathematically frozen.
    wire signed [63:0] P01_plus_P10_s64 =
        $signed(est_P01_cm2s) + $signed(est_P10_cm2s);
    wire signed [63:0] FPFT_P00_s64 =
        $signed(est_P00_cm2) +
        ((P01_plus_P10_s64 * dt_ms_s64) / 64'sd1000) +
        (($signed(est_P11_cm2s2) * dt2_ms2_s64) / 64'sd1000000);
    wire signed [63:0] FPFT_P01_s64 =
        $signed(est_P01_cm2s) +
        (($signed(est_P11_cm2s2) * dt_ms_s64) / 64'sd1000);
    wire signed [63:0] FPFT_P11_s64 = $signed(est_P11_cm2s2);

    wire signed [63:0] Q00_s64 =
        (PROCESS_Q00_MIN_CM2 > 0) ? PROCESS_Q00_MIN_CM2 : 64'sd1;
    wire signed [63:0] Q11_s64 =
        (PROCESS_Q11_MIN_CM2S2 > 0) ? PROCESS_Q11_MIN_CM2S2 : 64'sd1;

    // Scalar measurement update with H=[1 0].
    wire signed [63:0] innov_y_s64 = $signed(update_z_cm) - $signed(est_h_cm);
    wire signed [63:0] innov_S_s64 = $signed(est_P00_cm2) + $signed(MEAS_R_CM2);
    wire update_numeric_ok_w = update_valid && est_seeded && (innov_S_s64 > 64'sd0);

    wire signed [63:0] K0_q16_s64 = update_numeric_ok_w ?
        (($signed(est_P00_cm2) <<< 16) / innov_S_s64) : 64'sd0;
    wire signed [63:0] K1_q16_s64 = update_numeric_ok_w ?
        (($signed(est_P10_cm2s) <<< 16) / innov_S_s64) : 64'sd0;

    wire signed [63:0] h_update_s64 =
        $signed(est_h_cm) + ((K0_q16_s64 * innov_y_s64) >>> 16);
    wire signed [63:0] v_update_s64 =
        $signed(est_v_cms) + ((K1_q16_s64 * innov_y_s64) >>> 16);

    wire signed [63:0] a00_q16_s64 = Q16_ONE - K0_q16_s64;
    wire signed [63:0] a10_q16_s64 = -K1_q16_s64;

    wire signed [63:0] b00_s64 = (a00_q16_s64 * $signed(est_P00_cm2)) >>> 16;
    wire signed [63:0] b01_s64 = (a00_q16_s64 * $signed(est_P01_cm2s)) >>> 16;
    wire signed [63:0] b10_s64 = ((a10_q16_s64 * $signed(est_P00_cm2)) >>> 16) +
                                  $signed(est_P10_cm2s);
    wire signed [63:0] b11_s64 = ((a10_q16_s64 * $signed(est_P01_cm2s)) >>> 16) +
                                  $signed(est_P11_cm2s2);

    wire signed [63:0] k0r_s64 = K0_q16_s64 * $signed(MEAS_R_CM2);
    wire signed [63:0] k1r_s64 = K1_q16_s64 * $signed(MEAS_R_CM2);

    wire signed [63:0] nP00_joseph_s64 =
        ((b00_s64 * a00_q16_s64) >>> 16) + ((k0r_s64 * K0_q16_s64) >>> 32);
    wire signed [63:0] nP01_joseph_s64 =
        ((b00_s64 * a10_q16_s64) >>> 16) + b01_s64 +
        ((k0r_s64 * K1_q16_s64) >>> 32);
    wire signed [63:0] nP10_joseph_s64 =
        ((b10_s64 * a00_q16_s64) >>> 16) +
        ((k1r_s64 * K0_q16_s64) >>> 32);
    wire signed [63:0] nP11_joseph_s64 =
        ((b10_s64 * a10_q16_s64) >>> 16) + b11_s64 +
        ((k1r_s64 * K1_q16_s64) >>> 32);

    wire signed [63:0] sym01_s64 = (nP01_joseph_s64 + nP10_joseph_s64) >>> 1;

    always @(posedge clk) begin
        if (rst) begin
            est_valid    <= 1'b0;
            est_seeded   <= 1'b0;
            est_status   <= `ST_NOT_INITIALIZED;
            est_h_cm     <= 32'sd0;
            est_v_cms    <= 32'sd0;
            est_a_cms2   <= 32'sd0;
            est_P00_cm2  <= INITIAL_P00_CM2;
            est_P01_cm2s <= 32'sd0;
            est_P10_cm2s <= 32'sd0;
            est_P11_cm2s2 <= INITIAL_P11_CM2S2;
        end else if (seed_valid) begin
            est_valid    <= 1'b1;
            est_seeded   <= 1'b1;
            est_status   <= `ST_OK;
            est_h_cm     <= seed_h_cm;
            est_v_cms    <= 32'sd0;
            est_a_cms2   <= 32'sd0;
            est_P00_cm2  <= INITIAL_P00_CM2;
            est_P01_cm2s <= 32'sd0;
            est_P10_cm2s <= 32'sd0;
            est_P11_cm2s2 <= INITIAL_P11_CM2S2;
        end else if (predict_valid) begin
            if (est_seeded && dt_ok_w) begin
                est_valid    <= 1'b1;
                est_status   <= `ST_OK;
                est_h_cm     <= sat_s32($signed(est_h_cm) + dh_v_cm_s64 + dh_a_cm_s64);
                est_v_cms    <= sat_s32($signed(est_v_cms) + dv_cms_s64);
                est_a_cms2   <= predict_a_cms2;
                est_P00_cm2  <= floor_nonneg_s32(FPFT_P00_s64 + Q00_s64);
                est_P01_cm2s <= sat_s32(FPFT_P01_s64);
                est_P10_cm2s <= sat_s32(FPFT_P01_s64);
                est_P11_cm2s2 <= floor_nonneg_s32(FPFT_P11_s64 + Q11_s64);
            end else begin
                est_valid  <= 1'b0;
                est_status <= est_seeded ? `ST_STALE_REJECT : `ST_NOT_INITIALIZED;
            end
        end else if (update_valid) begin
            if (update_numeric_ok_w) begin
                est_valid    <= 1'b1;
                est_status   <= `ST_OK;
                est_h_cm     <= sat_s32(h_update_s64);
                est_v_cms    <= sat_s32(v_update_s64);
                est_P00_cm2  <= floor_nonneg_s32(nP00_joseph_s64);
                est_P01_cm2s <= sat_s32(sym01_s64);
                est_P10_cm2s <= sat_s32(sym01_s64);
                est_P11_cm2s2 <= floor_nonneg_s32(nP11_joseph_s64);
            end else begin
                est_valid  <= 1'b0;
                est_status <= est_seeded ? `ST_NUMERIC_FAULT : `ST_NOT_INITIALIZED;
            end
        end
    end

endmodule

`default_nettype wire
