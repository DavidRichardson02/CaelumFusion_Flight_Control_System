`timescale 1ns/1ps
`default_nettype none

`include "flight_viz_bundle_defs.vh"
`include "telemetry_defs_vh.vh"

//==============================================================================
// authority_gate_phase_sys
//------------------------------------------------------------------------------
// SYS-domain authority phase and gate producer.
//
// This module turns the current sensor/derived-state health into the explicit
// phase and gate signals consumed by flight_viz_model_sys. The phase output is
// stateful, not a direct one-cycle classifier:
//   - IDLE tracks the local ground altitude baseline.
//   - BOOST latches only after altitude-above-ground and vertical-speed
//     evidence agree that launch is underway.
//   - COAST follows BOOST after a bounded dwell.
//   - BRAKE is entered only in a healthy armed/policy-enabled coast window.
//   - DESCENT latches after a downward vertical-speed threshold.
//
// This is still an evidence-level FSM, not a calibrated flight computer. The
// final servo pulse remains owned by apogee_authority_policy_sys; this module
// owns only phase validity and safety/policy gate observability.
//
// The synchronized software-arm level remains observable even when runtime
// health denies policy enable or actuation. SW1 is the independent runtime
// policy-enable source. No physical actuator output is driven here; future
// actuator pins should be sourced from a dedicated non-self-test SYS-domain
// gate path.
//==============================================================================
module authority_gate_phase_sys #(
    parameter integer ASCENT_THRESH_CMS  = 100,
    parameter integer DESCENT_THRESH_CMS = -100,
    parameter integer LAUNCH_VSPD_THRESH_CMS = 250,
    parameter integer LAUNCH_ALT_DELTA_CM    = 100,
    parameter integer BRAKE_MIN_ALT_AGL_CM   = 500,
    parameter integer BOOST_MIN_CYCLES       = 5_000_000,

    parameter [15:0] BMP_MAX_AGE_MS = `BMP_FRESH_MAX_MS,
    parameter [15:0] ACC_MAX_AGE_MS = `ACC_FRESH_MAX_MS,
    parameter [15:0] MAG_MAX_AGE_MS = `MAG_FRESH_MAX_MS
)(
    input  wire        clk,
    input  wire        rst,

    input  wire        sw_arm_raw,
    input  wire        sw_policy_enable_raw,

    input  wire        bmp_valid,
    input  wire [7:0]  bmp_status,
    input  wire [15:0] bmp_age_ms,

    input  wire        acc_valid,
    input  wire [7:0]  acc_status,
    input  wire [15:0] acc_age_ms,

    input  wire        mag_valid,
    input  wire [7:0]  mag_status,
    input  wire [15:0] mag_age_ms,

    input  wire        der_valid,
    input  wire [7:0]  der_status,
    input  wire        der_alt_fresh,
    input  wire        der_vspd_fresh,
    input  wire        der_roll_fresh,
    input  wire        der_head_fresh,

    input  wire [15:0] der_bmp_age_ms,
    input  wire [15:0] der_acc_age_ms,
    input  wire [15:0] der_mag_age_ms,

    input  wire        der_bmp_valid_ref,
    input  wire        der_acc_valid_ref,
    input  wire        der_mag_valid_ref,

    input  wire signed [31:0] der_vertical_speed_cms,
    input  wire [31:0]        der_altitude_cm,

    output reg  [3:0]  auth_phase_code,
    output reg         auth_phase_valid,
    output reg         safety_runtime_ok,
    output reg         safety_allows_actuation,
    output reg         policy_runtime_enable,
    output reg         software_armed
);
    wire sw_arm_level;
    wire sw_policy_enable_level;

    sync_bit_3ff u_sw_arm_sync (
        .clk          (clk),
        .rst          (rst),
        .async_in     (sw_arm_raw),
        .sync_level   (sw_arm_level),
        .rise_pulse   (),
        .fall_pulse   (),
        .toggle_pulse ()
    );

    sync_bit_3ff u_sw_policy_enable_sync (
        .clk          (clk),
        .rst          (rst),
        .async_in     (sw_policy_enable_raw),
        .sync_level   (sw_policy_enable_level),
        .rise_pulse   (),
        .fall_pulse   (),
        .toggle_pulse ()
    );

    wire bmp_ok =
        bmp_valid &&
        (bmp_status == `ST_OK) &&
        (bmp_age_ms <= BMP_MAX_AGE_MS);

    wire acc_ok =
        acc_valid &&
        (acc_status == `ST_OK) &&
        (acc_age_ms <= ACC_MAX_AGE_MS);

    wire mag_ok =
        mag_valid &&
        (mag_status == `ST_OK) &&
        (mag_age_ms <= MAG_MAX_AGE_MS);

    wire der_refs_ok =
        der_bmp_valid_ref &&
        der_acc_valid_ref &&
        der_mag_valid_ref &&
        (der_bmp_age_ms <= BMP_MAX_AGE_MS) &&
        (der_acc_age_ms <= ACC_MAX_AGE_MS) &&
        (der_mag_age_ms <= MAG_MAX_AGE_MS);

    wire der_fresh_ok =
        der_alt_fresh &&
        der_vspd_fresh &&
        der_roll_fresh &&
        der_head_fresh;

    wire runtime_ok_w =
        bmp_ok &&
        acc_ok &&
        mag_ok &&
        der_valid &&
        (der_status == `ST_OK) &&
        der_fresh_ok &&
        der_refs_ok;

    localparam signed [31:0] ASCENT_THRESH_S32      = ASCENT_THRESH_CMS;
    localparam signed [31:0] DESCENT_THRESH_S32     = DESCENT_THRESH_CMS;
    localparam signed [31:0] LAUNCH_VSPD_THRESH_S32 = LAUNCH_VSPD_THRESH_CMS;
    localparam [31:0]        LAUNCH_ALT_DELTA_U32   = LAUNCH_ALT_DELTA_CM;
    localparam [31:0]        BRAKE_MIN_ALT_AGL_U32  = BRAKE_MIN_ALT_AGL_CM;
    localparam [31:0]        BOOST_MIN_CYCLES_U32   = BOOST_MIN_CYCLES;

    reg [3:0]  phase_state_r;
    reg [31:0] ground_altitude_cm_r;
    reg        ground_altitude_valid_r;
    reg [31:0] boost_cycle_count_r;

    wire [31:0] altitude_agl_cm_w =
        (!ground_altitude_valid_r || (der_altitude_cm <= ground_altitude_cm_r)) ?
            32'd0 :
            (der_altitude_cm - ground_altitude_cm_r);

    wire ascent_evidence_w =
        der_vertical_speed_cms > ASCENT_THRESH_S32;
    wire descent_evidence_w =
        der_vertical_speed_cms < DESCENT_THRESH_S32;
    wire launch_evidence_w =
        (der_vertical_speed_cms > LAUNCH_VSPD_THRESH_S32) &&
        (altitude_agl_cm_w >= LAUNCH_ALT_DELTA_U32);
    wire brake_window_w =
        ascent_evidence_w &&
        (altitude_agl_cm_w >= BRAKE_MIN_ALT_AGL_U32);
    wire near_ground_w =
        (altitude_agl_cm_w <= LAUNCH_ALT_DELTA_U32) &&
        !ascent_evidence_w;

    wire policy_enable_w = runtime_ok_w && sw_policy_enable_level;

    reg [3:0]  phase_state_next;
    reg [31:0] boost_cycle_count_next;
    reg        boost_dwell_done_w;

    always @(*) begin
        phase_state_next       = phase_state_r;
        boost_cycle_count_next = boost_cycle_count_r;
        boost_dwell_done_w     =
            (BOOST_MIN_CYCLES_U32 == 32'd0) ||
            (boost_cycle_count_r >= BOOST_MIN_CYCLES_U32);

        if (!runtime_ok_w) begin
            phase_state_next       = `VIZ_AUTH_PHASE_UNKNOWN;
            boost_cycle_count_next = 32'd0;
        end else begin
            case (phase_state_r)
                `VIZ_AUTH_PHASE_UNKNOWN: begin
                    boost_cycle_count_next = 32'd0;
                    if (launch_evidence_w)
                        phase_state_next = `VIZ_AUTH_PHASE_BOOST;
                    else
                        phase_state_next = `VIZ_AUTH_PHASE_IDLE;
                end

                `VIZ_AUTH_PHASE_IDLE: begin
                    boost_cycle_count_next = 32'd0;
                    if (launch_evidence_w)
                        phase_state_next = `VIZ_AUTH_PHASE_BOOST;
                    else
                        phase_state_next = `VIZ_AUTH_PHASE_IDLE;
                end

                `VIZ_AUTH_PHASE_BOOST: begin
                    if (!boost_dwell_done_w)
                        boost_cycle_count_next = boost_cycle_count_r + 32'd1;

                    if (descent_evidence_w) begin
                        phase_state_next       = `VIZ_AUTH_PHASE_DESCENT;
                        boost_cycle_count_next = 32'd0;
                    end else if (boost_dwell_done_w) begin
                        if (ascent_evidence_w)
                            phase_state_next = `VIZ_AUTH_PHASE_COAST;
                        else if (near_ground_w)
                            phase_state_next = `VIZ_AUTH_PHASE_IDLE;
                        else
                            phase_state_next = `VIZ_AUTH_PHASE_COAST;
                        boost_cycle_count_next = 32'd0;
                    end else begin
                        phase_state_next = `VIZ_AUTH_PHASE_BOOST;
                    end
                end

                `VIZ_AUTH_PHASE_COAST: begin
                    boost_cycle_count_next = 32'd0;
                    if (descent_evidence_w)
                        phase_state_next = `VIZ_AUTH_PHASE_DESCENT;
                    else if (policy_enable_w && sw_arm_level && brake_window_w)
                        phase_state_next = `VIZ_AUTH_PHASE_BRAKE;
                    else
                        phase_state_next = `VIZ_AUTH_PHASE_COAST;
                end

                `VIZ_AUTH_PHASE_BRAKE: begin
                    boost_cycle_count_next = 32'd0;
                    if (descent_evidence_w)
                        phase_state_next = `VIZ_AUTH_PHASE_DESCENT;
                    else if (!policy_enable_w || !sw_arm_level || !brake_window_w)
                        phase_state_next = `VIZ_AUTH_PHASE_COAST;
                    else
                        phase_state_next = `VIZ_AUTH_PHASE_BRAKE;
                end

                `VIZ_AUTH_PHASE_DESCENT: begin
                    boost_cycle_count_next = 32'd0;
                    if (near_ground_w)
                        phase_state_next = `VIZ_AUTH_PHASE_IDLE;
                    else
                        phase_state_next = `VIZ_AUTH_PHASE_DESCENT;
                end

                default: begin
                    phase_state_next       = `VIZ_AUTH_PHASE_UNKNOWN;
                    boost_cycle_count_next = 32'd0;
                end
            endcase
        end
    end

    wire phase_valid_w = runtime_ok_w;
    wire phase_allows_actuation_w =
        phase_valid_w &&
        ((phase_state_next == `VIZ_AUTH_PHASE_COAST) ||
         (phase_state_next == `VIZ_AUTH_PHASE_BRAKE));
    wire safety_allows_w =
        runtime_ok_w &&
        policy_enable_w &&
        sw_arm_level &&
        phase_allows_actuation_w;

    always @(posedge clk) begin
        if (rst) begin
            auth_phase_code        <= `VIZ_AUTH_PHASE_UNKNOWN;
            auth_phase_valid       <= 1'b0;
            safety_runtime_ok      <= 1'b0;
            safety_allows_actuation <= 1'b0;
            policy_runtime_enable  <= 1'b0;
            software_armed         <= 1'b0;
            phase_state_r          <= `VIZ_AUTH_PHASE_UNKNOWN;
            ground_altitude_cm_r   <= 32'd0;
            ground_altitude_valid_r <= 1'b0;
            boost_cycle_count_r    <= 32'd0;
        end else begin
            phase_state_r          <= phase_state_next;
            boost_cycle_count_r    <= boost_cycle_count_next;

            if (!runtime_ok_w) begin
                ground_altitude_valid_r <= 1'b0;
            end else if (!ground_altitude_valid_r ||
                         (phase_state_next == `VIZ_AUTH_PHASE_IDLE)) begin
                ground_altitude_cm_r    <= der_altitude_cm;
                ground_altitude_valid_r <= 1'b1;
            end

            auth_phase_code        <= runtime_ok_w ? phase_state_next :
                                                       `VIZ_AUTH_PHASE_UNKNOWN;
            auth_phase_valid       <= phase_valid_w;
            safety_runtime_ok      <= runtime_ok_w;
            safety_allows_actuation <= safety_allows_w;
            policy_runtime_enable  <= policy_enable_w;
            software_armed         <= sw_arm_level;
        end
    end

endmodule

`default_nettype wire
