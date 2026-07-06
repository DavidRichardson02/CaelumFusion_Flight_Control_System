`timescale 1ns/1ps
`default_nettype none

`include "telemetry_defs_vh.vh"
`include "flight_viz_bundle_defs.vh"

//==============================================================================
// apogee_authority_policy_sys
//------------------------------------------------------------------------------
// SYS-domain apogee authority and drag-servo policy model.
//
// CONTRACT
//   Inputs are the coherent derived-state values already qualified by the
//   derived-state producer. Outputs are a bounded, bundle-ready authority record:
//
//     - no-brake ballistic apogee estimate, cm
//     - full-brake drag apogee estimate, cm
//     - target apogee, cm
//     - uncertainty band, cm
//     - quantized brake command demand, u8
//     - safety-gated hobby-servo pulse width, us
//     - validity/status/flags for display and debugging
//
// PHYSICS / POLICY MODEL
//   The no-brake estimate uses h + v^2/(2g) with v in cm/s and g ~= 981 cm/s^2.
//   To keep the RTL deterministic and small, v^2/(2g) is approximated as
//   v^2 / 2^11, equivalent to a divisor of 2048. This is about 3.1% lower than
//   the prior v^2 * 33 / 2^16 approximation and avoids a wide post-square
//   multiply-by-33 datapath in LUT fabric.
//
//   The full-brake estimate models the airbrake as removing half of the kinetic
//   height. This is deliberately parameter-light: it is a real policy signal and
//   a stable hardware contract, but it is not yet a calibrated aerodynamic model.
//
//   The servo command is intentionally quantized to five positions. That avoids a
//   variable divider in the first hardware contract while still exposing a real
//   actuation demand:
//
//       0, 64, 128, 192, 255 -> 1000, 1250, 1500, 1750, 2000 us
//
//   Demand generation is inhibited unless derived altitude/vertical-speed
//   evidence is valid, fresh, and ascending. The servo pulse is additionally
//   gated by explicit safety/arming inputs; a denied gate retracts to 1000 us
//   while preserving the raw demand in auth_brake_cmd_u8 for observability.
//==============================================================================
module apogee_authority_policy_sys #(
    parameter [31:0] TARGET_APOGEE_CM = 32'd304800, // 10,000 ft
    parameter [15:0] MAX_BMP_AGE_MS   = `ALT_FRESH_MAX_MS,
    parameter [31:0] UNC_BASE_CM      = 32'd1500,
    parameter [31:0] UNC_MAX_CM       = 32'd50000
)(
    input  wire        sys_clk,
    input  wire        sys_rst,
    input  wire        der_valid,
    input  wire [7:0]  der_status,
    input  wire        der_alt_fresh,
    input  wire        der_vspd_fresh,
    input  wire        der_bmp_valid_ref,
    input  wire [15:0] der_bmp_age_ms,
    input  wire [31:0] altitude_cm,
    input  wire signed [31:0] vertical_speed_cms,
    input  wire        safety_runtime_ok,
    input  wire        safety_allows_actuation,
    input  wire        policy_runtime_enable,
    input  wire        software_armed,

    output reg         auth_valid,
    output reg  [7:0]  auth_status,
    output reg  [7:0]  auth_flags,
    output reg  [31:0] auth_target_cm,
    output reg  [31:0] auth_pred_no_cm,
    output reg  [31:0] auth_pred_full_cm,
    output reg  [15:0] auth_uncertainty_cm,
    output reg  [7:0]  auth_brake_cmd_u8,
    output reg  [11:0] auth_servo_us
);

    //==========================================================================
    // Helper Functions
    //==========================================================================
    function [31:0] sat_add_u32;
        input [31:0] a;
        input [31:0] b;
        reg [32:0] sum;
        begin
            sum = {1'b0, a} + {1'b0, b};
            sat_add_u32 = sum[32] ? 32'hFFFF_FFFF : sum[31:0];
        end
    endfunction

    wire der_status_ok = (der_status == `ST_OK);
    wire input_ok_w =
        der_valid &&
        der_status_ok &&
        der_alt_fresh &&
        der_vspd_fresh &&
        der_bmp_valid_ref &&
        (der_bmp_age_ms <= MAX_BMP_AGE_MS);

    wire [7:0] auth_status_w =
        (!der_valid && der_status_ok) ? `ST_MISSING_INPUT :
        (!der_status_ok)             ? der_status :
        (!der_alt_fresh ||
         !der_vspd_fresh ||
         !der_bmp_valid_ref ||
         (der_bmp_age_ms > MAX_BMP_AGE_MS)) ? `ST_STALE_REJECT :
                                              `ST_OK;

    wire velocity_negative = vertical_speed_cms[31];
    wire velocity_over_cap =
        (!velocity_negative) && (vertical_speed_cms > 32'sd65535);

    wire [15:0] vpos_cms_w =
        velocity_negative ? 16'd0 :
        velocity_over_cap ? 16'hFFFF :
                            vertical_speed_cms[15:0];

    wire [31:0] v2_cms2_w = {16'd0, vpos_cms_w} * {16'd0, vpos_cms_w};

    reg        input_ok_s1;
    reg [7:0]  auth_status_s1;
    reg [31:0] altitude_cm_s1;
    reg [15:0] bmp_age_ms_s1;
    reg        vpos_nonzero_s1;
    reg        safety_runtime_ok_s1;
    reg        safety_allows_actuation_s1;
    reg        policy_runtime_enable_s1;
    reg        software_armed_s1;
    reg [31:0] v2_cms2_s1;

    // v^2 / 2^11 ~= v^2 / (2 * 981). This resource-safe approximation removes
    // the previous wide v^2*33 shift-add while preserving monotonic coast gain.
    wire [31:0] coast_gain_cm_s2_w = {11'd0, v2_cms2_s1[31:11]};

    // 20 cm/ms as shift-add.
    wire [31:0] unc_age_cm_s2_w =
        ({16'd0, bmp_age_ms_s1} << 4) +
        ({16'd0, bmp_age_ms_s1} << 2);

    wire [31:0] unc_raw_cm_s2_w = UNC_BASE_CM + unc_age_cm_s2_w;
    wire        unc_capped_s2_w = (!input_ok_s1) || (unc_raw_cm_s2_w > UNC_MAX_CM);
    wire [31:0] unc_cm32_s2_w  = unc_capped_s2_w ? UNC_MAX_CM : unc_raw_cm_s2_w;

    reg        input_ok_s2;
    reg [7:0]  auth_status_s2;
    reg [31:0] altitude_cm_s2;
    reg        vpos_nonzero_s2;
    reg        safety_runtime_ok_s2;
    reg        safety_allows_actuation_s2;
    reg        policy_runtime_enable_s2;
    reg        software_armed_s2;
    reg [31:0] coast_gain_cm_s2;
    reg [31:0] full_brake_gain_cm_s2;
    reg [31:0] unc_cm32_s2;
    reg        unc_capped_s2;

    wire [31:0] pred_no_cm_w   = sat_add_u32(altitude_cm_s2, coast_gain_cm_s2);
    wire [31:0] pred_full_cm_w = sat_add_u32(altitude_cm_s2, full_brake_gain_cm_s2);

    wire [31:0] target_low_cm_s2_w =
        (TARGET_APOGEE_CM > unc_cm32_s2) ? (TARGET_APOGEE_CM - unc_cm32_s2) : 32'd0;
    wire [31:0] target_hi_cm_s2_w = sat_add_u32(TARGET_APOGEE_CM, unc_cm32_s2);

    wire ascending_s2_w = input_ok_s2 && vpos_nonzero_s2;

    reg        input_ok_s3;
    reg [7:0]  auth_status_s3;
    reg [31:0] pred_no_cm_s3;
    reg [31:0] pred_full_cm_s3;
    reg [31:0] unc_cm32_s3;
    reg        unc_capped_s3;
    reg [31:0] target_low_cm_s3;
    reg [31:0] target_hi_cm_s3;
    reg        ascending_s3;
    reg        actuation_allowed_s3;

    wire no_brake_above_high_s3_w = pred_no_cm_s3 > target_hi_cm_s3;
    wire target_reachable_s3_w =
        input_ok_s3 &&
        ascending_s3 &&
        (pred_no_cm_s3 >= target_low_cm_s3) &&
        (pred_full_cm_s3 <= target_hi_cm_s3);

    wire [31:0] overshoot_cm_s3_w =
        no_brake_above_high_s3_w ? (pred_no_cm_s3 - target_hi_cm_s3) : 32'd0;

    wire [31:0] authority_gap_cm_s3_w =
        (pred_no_cm_s3 > pred_full_cm_s3) ?
            (pred_no_cm_s3 - pred_full_cm_s3) :
            32'd0;

    wire [31:0] gap_q1_cm_s3_w = authority_gap_cm_s3_w >> 2;
    wire [31:0] gap_q2_cm_s3_w = authority_gap_cm_s3_w >> 1;
    wire [31:0] gap_q3_cm_s3_w = (authority_gap_cm_s3_w >> 1) + (authority_gap_cm_s3_w >> 2);

    reg [2:0] cmd_level_w;
    always @(*) begin
        if (!ascending_s3 || (overshoot_cm_s3_w == 32'd0))
            cmd_level_w = 3'd0;
        else if (authority_gap_cm_s3_w == 32'd0)
            cmd_level_w = 3'd4;
        else if (overshoot_cm_s3_w <= gap_q1_cm_s3_w)
            cmd_level_w = 3'd1;
        else if (overshoot_cm_s3_w <= gap_q2_cm_s3_w)
            cmd_level_w = 3'd2;
        else if (overshoot_cm_s3_w <= gap_q3_cm_s3_w)
            cmd_level_w = 3'd3;
        else
            cmd_level_w = 3'd4;
    end

    reg [7:0]  auth_brake_cmd_u8_w;
    reg [11:0] auth_policy_servo_us_w;
    always @(*) begin
        case (cmd_level_w)
            3'd1: begin
                auth_brake_cmd_u8_w   = 8'd64;
                auth_policy_servo_us_w = 12'd1250;
            end
            3'd2: begin
                auth_brake_cmd_u8_w   = 8'd128;
                auth_policy_servo_us_w = 12'd1500;
            end
            3'd3: begin
                auth_brake_cmd_u8_w   = 8'd192;
                auth_policy_servo_us_w = 12'd1750;
            end
            3'd4: begin
                auth_brake_cmd_u8_w   = 8'd255;
                auth_policy_servo_us_w = 12'd2000;
            end
            default: begin
                auth_brake_cmd_u8_w   = 8'd0;
                auth_policy_servo_us_w = 12'd1000;
            end
        endcase
    end

    wire actuator_active_w = actuation_allowed_s3 && (auth_brake_cmd_u8_w != 8'd0);

    always @(posedge sys_clk) begin
        if (sys_rst) begin
            input_ok_s1              <= 1'b0;
            auth_status_s1           <= `ST_NOT_INITIALIZED;
            altitude_cm_s1           <= 32'd0;
            bmp_age_ms_s1            <= 16'hFFFF;
            vpos_nonzero_s1          <= 1'b0;
            safety_runtime_ok_s1     <= 1'b0;
            safety_allows_actuation_s1 <= 1'b0;
            policy_runtime_enable_s1 <= 1'b0;
            software_armed_s1        <= 1'b0;
            v2_cms2_s1               <= 32'd0;

            input_ok_s2              <= 1'b0;
            auth_status_s2           <= `ST_NOT_INITIALIZED;
            altitude_cm_s2           <= 32'd0;
            vpos_nonzero_s2          <= 1'b0;
            safety_runtime_ok_s2     <= 1'b0;
            safety_allows_actuation_s2 <= 1'b0;
            policy_runtime_enable_s2 <= 1'b0;
            software_armed_s2        <= 1'b0;
            coast_gain_cm_s2         <= 32'd0;
            full_brake_gain_cm_s2    <= 32'd0;
            unc_cm32_s2              <= UNC_MAX_CM;
            unc_capped_s2            <= 1'b1;

            input_ok_s3              <= 1'b0;
            auth_status_s3           <= `ST_NOT_INITIALIZED;
            pred_no_cm_s3            <= 32'd0;
            pred_full_cm_s3          <= 32'd0;
            unc_cm32_s3              <= UNC_MAX_CM;
            unc_capped_s3            <= 1'b1;
            target_low_cm_s3         <= 32'd0;
            target_hi_cm_s3          <= TARGET_APOGEE_CM;
            ascending_s3             <= 1'b0;
            actuation_allowed_s3     <= 1'b0;

            auth_valid               <= 1'b0;
            auth_status              <= `ST_NOT_INITIALIZED;
            auth_flags               <= 8'd0;
            auth_target_cm           <= TARGET_APOGEE_CM;
            auth_pred_no_cm          <= 32'd0;
            auth_pred_full_cm        <= 32'd0;
            auth_uncertainty_cm      <= UNC_MAX_CM[15:0];
            auth_brake_cmd_u8        <= 8'd0;
            auth_servo_us            <= 12'd1000;
        end else begin
            input_ok_s1              <= input_ok_w;
            auth_status_s1           <= auth_status_w;
            altitude_cm_s1           <= altitude_cm;
            bmp_age_ms_s1            <= der_bmp_age_ms;
            vpos_nonzero_s1          <= (vpos_cms_w != 16'd0);
            safety_runtime_ok_s1     <= safety_runtime_ok;
            safety_allows_actuation_s1 <= safety_allows_actuation;
            policy_runtime_enable_s1 <= policy_runtime_enable;
            software_armed_s1        <= software_armed;
            v2_cms2_s1               <= v2_cms2_w;

            input_ok_s2              <= input_ok_s1;
            auth_status_s2           <= auth_status_s1;
            altitude_cm_s2           <= altitude_cm_s1;
            vpos_nonzero_s2          <= vpos_nonzero_s1;
            safety_runtime_ok_s2     <= safety_runtime_ok_s1;
            safety_allows_actuation_s2 <= safety_allows_actuation_s1;
            policy_runtime_enable_s2 <= policy_runtime_enable_s1;
            software_armed_s2        <= software_armed_s1;
            coast_gain_cm_s2         <= coast_gain_cm_s2_w;
            full_brake_gain_cm_s2    <= coast_gain_cm_s2_w >> 1;
            unc_cm32_s2              <= unc_cm32_s2_w;
            unc_capped_s2            <= unc_capped_s2_w;

            input_ok_s3              <= input_ok_s2;
            auth_status_s3           <= auth_status_s2;
            pred_no_cm_s3            <= pred_no_cm_w;
            pred_full_cm_s3          <= pred_full_cm_w;
            unc_cm32_s3              <= unc_cm32_s2;
            unc_capped_s3            <= unc_capped_s2;
            target_low_cm_s3         <= target_low_cm_s2_w;
            target_hi_cm_s3          <= target_hi_cm_s2_w;
            ascending_s3             <= ascending_s2_w;
            actuation_allowed_s3     <= input_ok_s2 &&
                                        safety_runtime_ok_s2 &&
                                        safety_allows_actuation_s2 &&
                                        policy_runtime_enable_s2 &&
                                        software_armed_s2;

            auth_valid               <= input_ok_s3;
            auth_status              <= auth_status_s3;
            auth_target_cm           <= TARGET_APOGEE_CM;
            auth_pred_no_cm          <= pred_no_cm_s3;
            auth_pred_full_cm        <= pred_full_cm_s3;
            auth_uncertainty_cm      <= unc_cm32_s3[15:0];
            auth_brake_cmd_u8        <= auth_brake_cmd_u8_w;
            auth_servo_us            <= actuator_active_w ? auth_policy_servo_us_w : 12'd1000;

            auth_flags[`VIZ_AUTH_FLG_INPUT_OK_BIT]    <= input_ok_s3;
            auth_flags[`VIZ_AUTH_FLG_ASCENDING_BIT]   <= ascending_s3;
            auth_flags[`VIZ_AUTH_FLG_NO_HIGH_BIT]     <= no_brake_above_high_s3_w;
            auth_flags[`VIZ_AUTH_FLG_REACHABLE_BIT]   <= target_reachable_s3_w;
            auth_flags[`VIZ_AUTH_FLG_CMD_NONZERO_BIT] <= (auth_brake_cmd_u8_w != 8'd0);
            auth_flags[`VIZ_AUTH_FLG_CMD_SAT_BIT]     <= (auth_brake_cmd_u8_w == 8'd255);
            auth_flags[`VIZ_AUTH_FLG_UNC_CAP_BIT]     <= unc_capped_s3;
            auth_flags[`VIZ_AUTH_FLG_ACT_SAFE_BIT]    <= actuation_allowed_s3;
        end
    end

endmodule

`default_nettype wire
