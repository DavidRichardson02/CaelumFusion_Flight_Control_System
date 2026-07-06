`timescale 1ns/1ps
`default_nettype none

`include "flight_viz_bundle_defs.vh"
`include "telemetry_defs_vh.vh"

//==============================================================================
// flight_viz_model_sys
//------------------------------------------------------------------------------
// SYS-domain visualization model.
//
// This block is the semantic boundary between telemetry/derived-state producers
// and the SYS->PIX visualization CDC. It consumes raw source summaries plus the
// coherent derived-state publication and produces:
//
//   1) canonical packed visualization bundle
//   2) semantic publication key
//   3) SYS-domain history write intent for altitude and vertical speed
//
// The actual chart memories remain in the PIX domain in flight_viz_suite_top.
// The history write intent is still produced here so tests and future bridges
// can verify that the model's write policy matches the bundle semantics.
//==============================================================================
module flight_viz_model_sys #(
    parameter integer PAYLOAD_W = 48
)(
    input  wire                       sys_clk,
    input  wire                       sys_rst,

    //--------------------------------------------------------------------------
    // Raw BMP snapshot
    //--------------------------------------------------------------------------
    input  wire [31:0]                bmp_t_us,
    input  wire [15:0]                bmp_seq,
    input  wire                       bmp_valid,
    input  wire [7:0]                 bmp_status,
    input  wire [PAYLOAD_W-1:0]       bmp_payload,
    input  wire [15:0]                bmp_age_ms,

    //--------------------------------------------------------------------------
    // Raw ACC snapshot
    //--------------------------------------------------------------------------
    input  wire [31:0]                acc_t_us,
    input  wire [15:0]                acc_seq,
    input  wire                       acc_valid,
    input  wire [7:0]                 acc_status,
    input  wire [PAYLOAD_W-1:0]       acc_payload,
    input  wire [15:0]                acc_age_ms,

    //--------------------------------------------------------------------------
    // Raw MAG snapshot
    //--------------------------------------------------------------------------
    input  wire [31:0]                mag_t_us,
    input  wire [15:0]                mag_seq,
    input  wire                       mag_valid,
    input  wire [7:0]                 mag_status,
    input  wire [PAYLOAD_W-1:0]       mag_payload,
    input  wire [15:0]                mag_age_ms,

    //--------------------------------------------------------------------------
    // Raw PMON1 power snapshot
    //--------------------------------------------------------------------------
    input  wire [31:0]                pwr_t_us,
    input  wire [15:0]                pwr_seq,
    input  wire                       pwr_valid,
    input  wire [7:0]                 pwr_status,
    input  wire [PAYLOAD_W-1:0]       pwr_payload,
    input  wire [15:0]                pwr_age_ms,

    //--------------------------------------------------------------------------
    // Future sensor-extension evidence summary
    //--------------------------------------------------------------------------
    input  wire                       ext_valid,
    input  wire [7:0]                 ext_status,
    input  wire [15:0]                ext_present_flags,
    input  wire [15:0]                ext_fault_flags,
    input  wire [15:0]                ext_mag_delta_l1,
    input  wire [15:0]                ext_mag_norm_primary,
    input  wire [15:0]                ext_mag_norm_secondary,
    input  wire                       ext_mag_sequence_aligned,
    input  wire                       ext_mag_disagreement,
    input  wire [3:0]                 ext_mag_sector_delta,
    input  wire [7:0]                 ext_mag_source_flags,
    input  wire [15:0]                ext_rng_height_cm,
    input  wire [15:0]                ext_air_dp_pa,
    input  wire [15:0]                ext_air_speed_cms,
    input  wire [15:0]                ext_env_temp_cdeg,
    input  wire [15:0]                ext_env_rh_centi,
    input  wire [15:0]                ext_sun_luma,
    input  wire [15:0]                ext_flow_dx,
    input  wire [15:0]                ext_flow_dy,
    input  wire [15:0]                ext_log_seq,
    input  wire [15:0]                ext_log_drop_count,
    input  wire [15:0]                ext_max_age_ms,

    //--------------------------------------------------------------------------
    // Coherent derived-state publication
    //--------------------------------------------------------------------------
    input  wire                       der_valid,
    input  wire [7:0]                 der_status,
    input  wire                       der_alt_fresh,
    input  wire                       der_vspd_fresh,
    input  wire                       der_roll_fresh,
    input  wire                       der_head_fresh,

    input  wire [15:0]                der_bmp_seq_ref,
    input  wire [15:0]                der_acc_seq_ref,
    input  wire [15:0]                der_mag_seq_ref,

    input  wire [15:0]                der_bmp_age_ms,
    input  wire [15:0]                der_acc_age_ms,
    input  wire [15:0]                der_mag_age_ms,

    input  wire                       der_bmp_valid_ref,
    input  wire                       der_acc_valid_ref,
    input  wire                       der_mag_valid_ref,

    input  wire [31:0]                der_altitude_cm,
    input  wire [31:0]                der_vertical_speed_cms,
    input  wire [31:0]                der_roll_mdeg,
    input  wire [31:0]                der_heading_mdeg,

    //--------------------------------------------------------------------------
    // Navigation / wind estimator publication
    //--------------------------------------------------------------------------
    input  wire                       nav_valid,
    input  wire [7:0]                 nav_status,
    input  wire [7:0]                 nav_flags,
    input  wire [15:0]                nav_downrange_m,
    input  wire [15:0]                nav_crossrange_m,
    input  wire [15:0]                nav_age_ms,

    input  wire                       wind_valid,
    input  wire [7:0]                 wind_status,
    input  wire [15:0]                wind_x_cms,
    input  wire [15:0]                wind_y_cms,
    input  wire [15:0]                wind_z_cms,
    input  wire [15:0]                wind_age_ms,

    //--------------------------------------------------------------------------
    // Authority phase / actuation gates
    //--------------------------------------------------------------------------
    input  wire [3:0]                 auth_phase_code,
    input  wire                       auth_phase_valid,
    input  wire                       safety_runtime_ok,
    input  wire                       safety_allows_actuation,
    input  wire                       policy_runtime_enable,
    input  wire                       software_armed,

    //--------------------------------------------------------------------------
    // Health / metadata counters
    //--------------------------------------------------------------------------
    input  wire [15:0]                i2c_nack_count,
    input  wire [15:0]                i2c_timeout_count,
    input  wire [15:0]                txn_rate_hz,
    input  wire [31:0]                cdc_update_count,
    input  wire [31:0]                build_id,
    input  wire [15:0]                schema_word,

    //--------------------------------------------------------------------------
    // History ring write ports
    //--------------------------------------------------------------------------
    output reg                        alt_we,
    output reg  [9:0]                 alt_addr,
    output reg  [15:0]                alt_data,

    output reg                        vspd_we,
    output reg  [9:0]                 vspd_addr,
    output reg  [15:0]                vspd_data,

    //--------------------------------------------------------------------------
    // Canonical packed visualization bundle
    //--------------------------------------------------------------------------
    output wire [`VIZ_BUNDLE_W-1:0]   viz_bundle_sys,
    output wire [15:0]                viz_key_sys,

    //--------------------------------------------------------------------------
    // Rolling history write pointer publication
    //--------------------------------------------------------------------------
    output wire [9:0]                 wr_ptr_sys,
    output wire                       wr_ptr_pulse_sys
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
    // Change Detection
    //==========================================================================
    reg [15:0] bmp_seq_seen;
    reg [15:0] acc_seq_seen;
    reg [15:0] mag_seq_seen;
    reg [15:0] pwr_seq_seen;
    reg        bmp_seq_seen_valid;
    reg        acc_seq_seen_valid;
    reg        mag_seq_seen_valid;
    reg        pwr_seq_seen_valid;

    wire bmp_seq_new = !bmp_seq_seen_valid || (bmp_seq != bmp_seq_seen);
    wire acc_seq_new = !acc_seq_seen_valid || (acc_seq != acc_seq_seen);
    wire mag_seq_new = !mag_seq_seen_valid || (mag_seq != mag_seq_seen);
    wire pwr_seq_new = !pwr_seq_seen_valid || (pwr_seq != pwr_seq_seen);

    reg [15:0] viz_key_q;
    reg [9:0]  wr_ptr_q;
    reg        wr_ptr_pulse_q;
    reg [3:0]  semantic_changed_pipe_q;

    reg [31:0] bmp_t_us_q;
    reg [31:0] acc_t_us_q;
    reg [31:0] mag_t_us_q;
    reg [31:0] pwr_t_us_q;

    reg [15:0] bmp_seq_q;
    reg        bmp_valid_q;
    reg [7:0]  bmp_status_q;
    reg [15:0] bmp_age_ms_q;

    reg [15:0] acc_seq_q;
    reg        acc_valid_q;
    reg [7:0]  acc_status_q;
    reg [15:0] acc_age_ms_q;

    reg [15:0] mag_seq_q;
    reg        mag_valid_q;
    reg [7:0]  mag_status_q;
    reg [15:0] mag_age_ms_q;

    reg [15:0] pwr_seq_q;
    reg        pwr_valid_q;
    reg [7:0]  pwr_status_q;
    reg [15:0] pwr_age_ms_q;
    reg [PAYLOAD_W-1:0] pwr_payload_q;

    reg        ext_valid_q;
    reg [7:0]  ext_status_q;
    reg [15:0] ext_present_flags_q;
    reg [15:0] ext_fault_flags_q;
    reg [15:0] ext_mag_delta_l1_q;
    reg [15:0] ext_mag_norm_primary_q;
    reg [15:0] ext_mag_norm_secondary_q;
    reg        ext_mag_sequence_aligned_q;
    reg        ext_mag_disagreement_q;
    reg [3:0]  ext_mag_sector_delta_q;
    reg [7:0]  ext_mag_source_flags_q;
    reg [15:0] ext_rng_height_cm_q;
    reg [15:0] ext_air_dp_pa_q;
    reg [15:0] ext_air_speed_cms_q;
    reg [15:0] ext_env_temp_cdeg_q;
    reg [15:0] ext_env_rh_centi_q;
    reg [15:0] ext_sun_luma_q;
    reg [15:0] ext_flow_dx_q;
    reg [15:0] ext_flow_dy_q;
    reg [15:0] ext_log_seq_q;
    reg [15:0] ext_log_drop_count_q;
    reg [15:0] ext_max_age_ms_q;

    reg        der_valid_q;
    reg [7:0]  der_status_q;
    reg        der_alt_fresh_q;
    reg        der_vspd_fresh_q;
    reg        der_roll_fresh_q;
    reg        der_head_fresh_q;

    reg [15:0] der_bmp_seq_ref_q;
    reg [15:0] der_acc_seq_ref_q;
    reg [15:0] der_mag_seq_ref_q;

    reg [15:0] der_bmp_age_ms_q;
    reg [15:0] der_acc_age_ms_q;
    reg [15:0] der_mag_age_ms_q;

    reg        der_bmp_valid_ref_q;
    reg        der_acc_valid_ref_q;
    reg        der_mag_valid_ref_q;

    reg [31:0] der_altitude_cm_q;
    reg [31:0] der_vertical_speed_cms_q;
    reg [31:0] der_roll_mdeg_q;
    reg [31:0] der_heading_mdeg_q;

    reg        nav_valid_q;
    reg [7:0]  nav_status_q;
    reg [7:0]  nav_flags_q;
    reg [15:0] nav_downrange_m_q;
    reg [15:0] nav_crossrange_m_q;
    reg [15:0] nav_age_ms_q;

    reg        wind_valid_q;
    reg [7:0]  wind_status_q;
    reg [15:0] wind_x_cms_q;
    reg [15:0] wind_y_cms_q;
    reg [15:0] wind_z_cms_q;
    reg [15:0] wind_age_ms_q;

    reg [3:0]  auth_phase_code_q;
    reg        auth_phase_valid_q;
    reg        safety_runtime_ok_q;
    reg        safety_allows_actuation_q;
    reg        policy_runtime_enable_q;
    reg        software_armed_q;

    reg [15:0] i2c_nack_count_q;
    reg [15:0] i2c_timeout_count_q;
    reg [15:0] txn_rate_hz_q;
    reg [31:0] cdc_update_count_q;
    reg [31:0] build_id_q;
    reg [15:0] schema_word_q;

    reg [`VIZ_BUNDLE_W-1:0] viz_bundle_r;

    //==========================================================================
    // SYS-domain Apogee Authority / Drag-Servo Policy
    //==========================================================================
    wire        auth_valid_w;
    wire [7:0]  auth_status_w;
    wire [7:0]  auth_flags_w;
    wire [31:0] auth_target_cm_w;
    wire [31:0] auth_pred_no_cm_w;
    wire [31:0] auth_pred_full_cm_w;
    wire [15:0] auth_uncertainty_cm_w;
    wire [7:0]  auth_brake_cmd_u8_w;
    wire [11:0] auth_servo_us_w;
    wire [3:0]  auth_phase_code_w;
    wire [6:0]  auth_gate_flags_w;

    apogee_authority_policy_sys u_apogee_authority_policy_sys (
        .sys_clk             (sys_clk),
        .sys_rst             (sys_rst),
        .der_valid           (der_valid_q),
        .der_status          (der_status_q),
        .der_alt_fresh       (der_alt_fresh_q),
        .der_vspd_fresh      (der_vspd_fresh_q),
        .der_bmp_valid_ref   (der_bmp_valid_ref_q),
        .der_bmp_age_ms      (der_bmp_age_ms_q),
        .altitude_cm         (der_altitude_cm_q),
        .vertical_speed_cms  ($signed(der_vertical_speed_cms_q)),
        .safety_runtime_ok   (safety_runtime_ok_q),
        .safety_allows_actuation(safety_allows_actuation_q),
        .policy_runtime_enable(policy_runtime_enable_q),
        .software_armed      (software_armed_q),

        .auth_valid          (auth_valid_w),
        .auth_status         (auth_status_w),
        .auth_flags          (auth_flags_w),
        .auth_target_cm      (auth_target_cm_w),
        .auth_pred_no_cm     (auth_pred_no_cm_w),
        .auth_pred_full_cm   (auth_pred_full_cm_w),
        .auth_uncertainty_cm (auth_uncertainty_cm_w),
        .auth_brake_cmd_u8   (auth_brake_cmd_u8_w),
        .auth_servo_us       (auth_servo_us_w)
    );

    reg [3:0] local_phase_code_r;
    always @(*) begin
        if (!auth_valid_w)
            local_phase_code_r = `VIZ_AUTH_PHASE_UNKNOWN;
        else if ($signed(der_vertical_speed_cms_q) < 32'sd0)
            local_phase_code_r = `VIZ_AUTH_PHASE_DESCENT;
        else if (auth_brake_cmd_u8_w != 8'd0)
            local_phase_code_r = `VIZ_AUTH_PHASE_BRAKE;
        else if (auth_flags_w[`VIZ_AUTH_FLG_ASCENDING_BIT])
            local_phase_code_r = `VIZ_AUTH_PHASE_COAST;
        else
            local_phase_code_r = `VIZ_AUTH_PHASE_IDLE;
    end

    assign auth_phase_code_w = auth_phase_valid_q ? auth_phase_code_q :
                                                    local_phase_code_r;

    assign auth_gate_flags_w[`VIZ_AUTH_GATE_SAFETY_RUNTIME_OK_BIT] = safety_runtime_ok_q;
    assign auth_gate_flags_w[`VIZ_AUTH_GATE_SAFETY_ALLOWS_BIT]     = safety_allows_actuation_q;
    assign auth_gate_flags_w[`VIZ_AUTH_GATE_POLICY_ENABLE_BIT]     = policy_runtime_enable_q;
    assign auth_gate_flags_w[`VIZ_AUTH_GATE_SOFTWARE_ARMED_BIT]    = software_armed_q;
    assign auth_gate_flags_w[`VIZ_AUTH_GATE_ACTUATOR_ACTIVE_BIT]   = (auth_servo_us_w != 12'd1000);
    assign auth_gate_flags_w[`VIZ_AUTH_GATE_EXTERNAL_PHASE_BIT]    = auth_phase_valid_q;
    assign auth_gate_flags_w[`VIZ_AUTH_GATE_LOCAL_PHASE_BIT]       = !auth_phase_valid_q;

    wire semantic_changed_now =
        bmp_seq_new ||
        acc_seq_new ||
        mag_seq_new ||
        pwr_seq_new ||
        (bmp_valid      != bmp_valid_q) ||
        (acc_valid      != acc_valid_q) ||
        (mag_valid      != mag_valid_q) ||
        (pwr_valid      != pwr_valid_q) ||
        (bmp_status     != bmp_status_q) ||
        (acc_status     != acc_status_q) ||
        (mag_status     != mag_status_q) ||
        (pwr_status     != pwr_status_q) ||
        (bmp_age_ms     != bmp_age_ms_q) ||
        (acc_age_ms     != acc_age_ms_q) ||
        (mag_age_ms     != mag_age_ms_q) ||
        (pwr_age_ms     != pwr_age_ms_q) ||
        (pwr_payload    != pwr_payload_q) ||
        (ext_valid              != ext_valid_q) ||
        (ext_status             != ext_status_q) ||
        (ext_present_flags      != ext_present_flags_q) ||
        (ext_fault_flags        != ext_fault_flags_q) ||
        (ext_mag_delta_l1       != ext_mag_delta_l1_q) ||
        (ext_mag_norm_primary   != ext_mag_norm_primary_q) ||
        (ext_mag_norm_secondary != ext_mag_norm_secondary_q) ||
        (ext_mag_sequence_aligned != ext_mag_sequence_aligned_q) ||
        (ext_mag_disagreement    != ext_mag_disagreement_q) ||
        (ext_mag_sector_delta    != ext_mag_sector_delta_q) ||
        (ext_mag_source_flags    != ext_mag_source_flags_q) ||
        (ext_rng_height_cm      != ext_rng_height_cm_q) ||
        (ext_air_dp_pa          != ext_air_dp_pa_q) ||
        (ext_air_speed_cms      != ext_air_speed_cms_q) ||
        (ext_env_temp_cdeg      != ext_env_temp_cdeg_q) ||
        (ext_env_rh_centi       != ext_env_rh_centi_q) ||
        (ext_sun_luma           != ext_sun_luma_q) ||
        (ext_flow_dx            != ext_flow_dx_q) ||
        (ext_flow_dy            != ext_flow_dy_q) ||
        (ext_log_seq            != ext_log_seq_q) ||
        (ext_log_drop_count     != ext_log_drop_count_q) ||
        (ext_max_age_ms         != ext_max_age_ms_q) ||
        (der_valid      != der_valid_q) ||
        (der_status     != der_status_q) ||
        (der_alt_fresh  != der_alt_fresh_q) ||
        (der_vspd_fresh != der_vspd_fresh_q) ||
        (der_roll_fresh != der_roll_fresh_q) ||
        (der_head_fresh != der_head_fresh_q) ||
        (der_bmp_seq_ref != der_bmp_seq_ref_q) ||
        (der_acc_seq_ref != der_acc_seq_ref_q) ||
        (der_mag_seq_ref != der_mag_seq_ref_q) ||
        (der_bmp_age_ms  != der_bmp_age_ms_q) ||
        (der_acc_age_ms  != der_acc_age_ms_q) ||
        (der_mag_age_ms  != der_mag_age_ms_q) ||
        (der_bmp_valid_ref != der_bmp_valid_ref_q) ||
        (der_acc_valid_ref != der_acc_valid_ref_q) ||
        (der_mag_valid_ref != der_mag_valid_ref_q) ||
        (der_altitude_cm        != der_altitude_cm_q) ||
        (der_vertical_speed_cms != der_vertical_speed_cms_q) ||
        (der_roll_mdeg         != der_roll_mdeg_q) ||
        (der_heading_mdeg      != der_heading_mdeg_q) ||
        (nav_valid             != nav_valid_q) ||
        (nav_status            != nav_status_q) ||
        (nav_flags             != nav_flags_q) ||
        (nav_downrange_m       != nav_downrange_m_q) ||
        (nav_crossrange_m      != nav_crossrange_m_q) ||
        (nav_age_ms            != nav_age_ms_q) ||
        (wind_valid            != wind_valid_q) ||
        (wind_status           != wind_status_q) ||
        (wind_x_cms            != wind_x_cms_q) ||
        (wind_y_cms            != wind_y_cms_q) ||
        (wind_z_cms            != wind_z_cms_q) ||
        (wind_age_ms           != wind_age_ms_q) ||
        (auth_phase_code       != auth_phase_code_q) ||
        (auth_phase_valid      != auth_phase_valid_q) ||
        (safety_runtime_ok     != safety_runtime_ok_q) ||
        (safety_allows_actuation != safety_allows_actuation_q) ||
        (policy_runtime_enable != policy_runtime_enable_q) ||
        (software_armed        != software_armed_q) ||
        (i2c_nack_count    != i2c_nack_count_q) ||
        (i2c_timeout_count != i2c_timeout_count_q) ||
        (txn_rate_hz       != txn_rate_hz_q) ||
        (cdc_update_count  != cdc_update_count_q) ||
        (build_id          != build_id_q) ||
        (schema_word       != schema_word_q);

    //==========================================================================
    // Outputs
    //==========================================================================
    assign viz_bundle_sys   = viz_bundle_r;
    assign viz_key_sys      = viz_key_q;
    assign wr_ptr_sys       = wr_ptr_q;
    assign wr_ptr_pulse_sys = wr_ptr_pulse_q;

    //==========================================================================
    // Sequential Logic
    //==========================================================================
    always @(posedge sys_clk) begin
        if (sys_rst) begin
            bmp_seq_seen       <= 16'd0;
            acc_seq_seen       <= 16'd0;
            mag_seq_seen       <= 16'd0;
            pwr_seq_seen       <= 16'd0;
            bmp_seq_seen_valid <= 1'b0;
            acc_seq_seen_valid <= 1'b0;
            mag_seq_seen_valid <= 1'b0;
            pwr_seq_seen_valid <= 1'b0;

            viz_key_q      <= 16'd0;
            wr_ptr_q       <= 10'd0;
            wr_ptr_pulse_q <= 1'b0;
            semantic_changed_pipe_q <= 4'd0;

            bmp_t_us_q    <= 32'd0;
            acc_t_us_q    <= 32'd0;
            mag_t_us_q    <= 32'd0;
            pwr_t_us_q    <= 32'd0;

            bmp_seq_q     <= 16'd0;
            bmp_valid_q   <= 1'b0;
            bmp_status_q  <= `ST_NOT_INITIALIZED;
            bmp_age_ms_q  <= 16'hFFFF;

            acc_seq_q     <= 16'd0;
            acc_valid_q   <= 1'b0;
            acc_status_q  <= `ST_NOT_INITIALIZED;
            acc_age_ms_q  <= 16'hFFFF;

            mag_seq_q     <= 16'd0;
            mag_valid_q   <= 1'b0;
            mag_status_q  <= `ST_NOT_INITIALIZED;
            mag_age_ms_q  <= 16'hFFFF;

            pwr_seq_q     <= 16'd0;
            pwr_valid_q   <= 1'b0;
            pwr_status_q  <= `ST_NOT_INITIALIZED;
            pwr_age_ms_q  <= 16'hFFFF;
            pwr_payload_q <= {PAYLOAD_W{1'b0}};

            ext_valid_q              <= 1'b0;
            ext_status_q             <= `ST_NOT_INITIALIZED;
            ext_present_flags_q      <= 16'd0;
            ext_fault_flags_q        <= 16'd0;
            ext_mag_delta_l1_q       <= 16'd0;
            ext_mag_norm_primary_q   <= 16'd0;
            ext_mag_norm_secondary_q <= 16'd0;
            ext_mag_sequence_aligned_q <= 1'b0;
            ext_mag_disagreement_q    <= 1'b0;
            ext_mag_sector_delta_q    <= 4'd0;
            ext_mag_source_flags_q    <= 8'd0;
            ext_rng_height_cm_q      <= 16'd0;
            ext_air_dp_pa_q          <= 16'd0;
            ext_air_speed_cms_q      <= 16'd0;
            ext_env_temp_cdeg_q      <= 16'd0;
            ext_env_rh_centi_q       <= 16'd0;
            ext_sun_luma_q           <= 16'd0;
            ext_flow_dx_q            <= 16'd0;
            ext_flow_dy_q            <= 16'd0;
            ext_log_seq_q            <= 16'd0;
            ext_log_drop_count_q     <= 16'd0;
            ext_max_age_ms_q         <= 16'hFFFF;

            der_valid_q      <= 1'b0;
            der_status_q     <= `ST_NOT_INITIALIZED;
            der_alt_fresh_q  <= 1'b0;
            der_vspd_fresh_q <= 1'b0;
            der_roll_fresh_q <= 1'b0;
            der_head_fresh_q <= 1'b0;

            der_bmp_seq_ref_q <= 16'd0;
            der_acc_seq_ref_q <= 16'd0;
            der_mag_seq_ref_q <= 16'd0;

            der_bmp_age_ms_q <= 16'hFFFF;
            der_acc_age_ms_q <= 16'hFFFF;
            der_mag_age_ms_q <= 16'hFFFF;

            der_bmp_valid_ref_q <= 1'b0;
            der_acc_valid_ref_q <= 1'b0;
            der_mag_valid_ref_q <= 1'b0;

            der_altitude_cm_q        <= 32'd0;
            der_vertical_speed_cms_q <= 32'd0;
            der_roll_mdeg_q          <= 32'd0;
            der_heading_mdeg_q       <= 32'd0;

            nav_valid_q        <= 1'b0;
            nav_status_q       <= `ST_NOT_INITIALIZED;
            nav_flags_q        <= 8'd0;
            nav_downrange_m_q  <= 16'd0;
            nav_crossrange_m_q <= 16'd0;
            nav_age_ms_q       <= 16'hFFFF;

            wind_valid_q       <= 1'b0;
            wind_status_q      <= `ST_NOT_INITIALIZED;
            wind_x_cms_q       <= 16'd0;
            wind_y_cms_q       <= 16'd0;
            wind_z_cms_q       <= 16'd0;
            wind_age_ms_q      <= 16'hFFFF;

            auth_phase_code_q       <= `VIZ_AUTH_PHASE_UNKNOWN;
            auth_phase_valid_q      <= 1'b0;
            safety_runtime_ok_q     <= 1'b0;
            safety_allows_actuation_q <= 1'b0;
            policy_runtime_enable_q <= 1'b0;
            software_armed_q        <= 1'b0;

            i2c_nack_count_q    <= 16'd0;
            i2c_timeout_count_q <= 16'd0;
            txn_rate_hz_q       <= 16'd0;
            cdc_update_count_q  <= 32'd0;
            build_id_q          <= 32'd0;
            schema_word_q       <= 16'd0;

            alt_we    <= 1'b0;
            alt_addr  <= 10'd0;
            alt_data  <= 16'd0;
            vspd_we   <= 1'b0;
            vspd_addr <= 10'd0;
            vspd_data <= 16'd0;
        end else begin
            alt_we         <= 1'b0;
            vspd_we        <= 1'b0;
            wr_ptr_pulse_q <= 1'b0;
            semantic_changed_pipe_q <= {semantic_changed_pipe_q[2:0],
                                        semantic_changed_now};

            if (semantic_changed_pipe_q[3])
                viz_key_q <= viz_key_q + 16'd1;

            if (bmp_seq_new) begin
                bmp_seq_seen       <= bmp_seq;
                bmp_seq_seen_valid <= 1'b1;
            end

            if (acc_seq_new) begin
                acc_seq_seen       <= acc_seq;
                acc_seq_seen_valid <= 1'b1;
            end

            if (mag_seq_new) begin
                mag_seq_seen       <= mag_seq;
                mag_seq_seen_valid <= 1'b1;
            end

            if (pwr_seq_new) begin
                pwr_seq_seen       <= pwr_seq;
                pwr_seq_seen_valid <= 1'b1;
            end

            bmp_t_us_q   <= bmp_t_us;
            acc_t_us_q   <= acc_t_us;
            mag_t_us_q   <= mag_t_us;
            pwr_t_us_q   <= pwr_t_us;

            bmp_seq_q    <= bmp_seq;
            bmp_valid_q  <= bmp_valid;
            bmp_status_q <= bmp_status;
            bmp_age_ms_q <= bmp_age_ms;

            acc_seq_q    <= acc_seq;
            acc_valid_q  <= acc_valid;
            acc_status_q <= acc_status;
            acc_age_ms_q <= acc_age_ms;

            mag_seq_q    <= mag_seq;
            mag_valid_q  <= mag_valid;
            mag_status_q <= mag_status;
            mag_age_ms_q <= mag_age_ms;

            pwr_seq_q     <= pwr_seq;
            pwr_valid_q   <= pwr_valid;
            pwr_status_q  <= pwr_status;
            pwr_age_ms_q  <= pwr_age_ms;
            pwr_payload_q <= pwr_payload;

            ext_valid_q              <= ext_valid;
            ext_status_q             <= ext_status;
            ext_present_flags_q      <= ext_present_flags;
            ext_fault_flags_q        <= ext_fault_flags;
            ext_mag_delta_l1_q       <= ext_mag_delta_l1;
            ext_mag_norm_primary_q   <= ext_mag_norm_primary;
            ext_mag_norm_secondary_q <= ext_mag_norm_secondary;
            ext_mag_sequence_aligned_q <= ext_mag_sequence_aligned;
            ext_mag_disagreement_q    <= ext_mag_disagreement;
            ext_mag_sector_delta_q    <= ext_mag_sector_delta;
            ext_mag_source_flags_q    <= ext_mag_source_flags;
            ext_rng_height_cm_q      <= ext_rng_height_cm;
            ext_air_dp_pa_q          <= ext_air_dp_pa;
            ext_air_speed_cms_q      <= ext_air_speed_cms;
            ext_env_temp_cdeg_q      <= ext_env_temp_cdeg;
            ext_env_rh_centi_q       <= ext_env_rh_centi;
            ext_sun_luma_q           <= ext_sun_luma;
            ext_flow_dx_q            <= ext_flow_dx;
            ext_flow_dy_q            <= ext_flow_dy;
            ext_log_seq_q            <= ext_log_seq;
            ext_log_drop_count_q     <= ext_log_drop_count;
            ext_max_age_ms_q         <= ext_max_age_ms;

            der_valid_q      <= der_valid;
            der_status_q     <= der_status;
            der_alt_fresh_q  <= der_alt_fresh;
            der_vspd_fresh_q <= der_vspd_fresh;
            der_roll_fresh_q <= der_roll_fresh;
            der_head_fresh_q <= der_head_fresh;

            der_bmp_seq_ref_q <= der_bmp_seq_ref;
            der_acc_seq_ref_q <= der_acc_seq_ref;
            der_mag_seq_ref_q <= der_mag_seq_ref;

            der_bmp_age_ms_q <= der_bmp_age_ms;
            der_acc_age_ms_q <= der_acc_age_ms;
            der_mag_age_ms_q <= der_mag_age_ms;

            der_bmp_valid_ref_q <= der_bmp_valid_ref;
            der_acc_valid_ref_q <= der_acc_valid_ref;
            der_mag_valid_ref_q <= der_mag_valid_ref;

            der_altitude_cm_q        <= der_altitude_cm;
            der_vertical_speed_cms_q <= der_vertical_speed_cms;
            der_roll_mdeg_q          <= der_roll_mdeg;
            der_heading_mdeg_q       <= der_heading_mdeg;

            nav_valid_q        <= nav_valid;
            nav_status_q       <= nav_status;
            nav_flags_q        <= nav_flags;
            nav_downrange_m_q  <= nav_downrange_m;
            nav_crossrange_m_q <= nav_crossrange_m;
            nav_age_ms_q       <= nav_age_ms;

            wind_valid_q       <= wind_valid;
            wind_status_q      <= wind_status;
            wind_x_cms_q       <= wind_x_cms;
            wind_y_cms_q       <= wind_y_cms;
            wind_z_cms_q       <= wind_z_cms;
            wind_age_ms_q      <= wind_age_ms;

            auth_phase_code_q       <= auth_phase_code;
            auth_phase_valid_q      <= auth_phase_valid;
            safety_runtime_ok_q     <= safety_runtime_ok;
            safety_allows_actuation_q <= safety_allows_actuation;
            policy_runtime_enable_q <= policy_runtime_enable;
            software_armed_q        <= software_armed;

            i2c_nack_count_q    <= i2c_nack_count;
            i2c_timeout_count_q <= i2c_timeout_count;
            txn_rate_hz_q       <= txn_rate_hz;
            cdc_update_count_q  <= cdc_update_count;
            build_id_q          <= build_id;
            schema_word_q       <= schema_word;

            if (bmp_seq_new && bmp_valid && (bmp_status == `ST_OK)) begin
                alt_we    <= 1'b1;
                alt_addr  <= wr_ptr_q;
                alt_data  <= sat_u16_from_u32(der_altitude_cm);

                vspd_we   <= 1'b1;
                vspd_addr <= wr_ptr_q;
                vspd_data <= sat_s16_from_s32($signed(der_vertical_speed_cms));

                wr_ptr_q       <= wr_ptr_q + 10'd1;
                wr_ptr_pulse_q <= 1'b1;
            end
        end
    end

    //==========================================================================
    // Canonical Packed Bundle
    //==========================================================================
    always @(*) begin
        viz_bundle_r = {`VIZ_BUNDLE_W{1'b0}};

        viz_bundle_r[`VIZ_EXT_VALID_BIT] = ext_valid_q;
        viz_bundle_r[`VIZ_EXT_STATUS_MSB:`VIZ_EXT_STATUS_LSB] = ext_status_q;
        viz_bundle_r[`VIZ_EXT_PRESENT_MSB:`VIZ_EXT_PRESENT_LSB] = ext_present_flags_q;
        viz_bundle_r[`VIZ_EXT_FAULT_MSB:`VIZ_EXT_FAULT_LSB] = ext_fault_flags_q;
        viz_bundle_r[`VIZ_EXT_MAG_DELTA_L1_MSB:`VIZ_EXT_MAG_DELTA_L1_LSB] = ext_mag_delta_l1_q;
        viz_bundle_r[`VIZ_EXT_MAG_NORM0_MSB:`VIZ_EXT_MAG_NORM0_LSB] = ext_mag_norm_primary_q;
        viz_bundle_r[`VIZ_EXT_MAG_NORM1_MSB:`VIZ_EXT_MAG_NORM1_LSB] = ext_mag_norm_secondary_q;
        viz_bundle_r[`VIZ_EXT_MAG_SEQ_ALIGNED_BIT] = ext_mag_sequence_aligned_q;
        viz_bundle_r[`VIZ_EXT_MAG_DISAGREE_BIT] = ext_mag_disagreement_q;
        viz_bundle_r[`VIZ_EXT_MAG_SECTOR_DELTA_MSB:`VIZ_EXT_MAG_SECTOR_DELTA_LSB] =
            ext_mag_sector_delta_q;
        viz_bundle_r[`VIZ_EXT_MAG_SOURCE_SYNTH_BIT] =
            ext_mag_source_flags_q[`EXT_SRC_SYNTHETIC_BIT];
        viz_bundle_r[`VIZ_EXT_RNG_HEIGHT_CM_MSB:`VIZ_EXT_RNG_HEIGHT_CM_LSB] = ext_rng_height_cm_q;
        viz_bundle_r[`VIZ_EXT_AIR_DP_PA_MSB:`VIZ_EXT_AIR_DP_PA_LSB] = ext_air_dp_pa_q;
        viz_bundle_r[`VIZ_EXT_AIR_SPEED_CMS_MSB:`VIZ_EXT_AIR_SPEED_CMS_LSB] = ext_air_speed_cms_q;
        viz_bundle_r[`VIZ_EXT_ENV_TEMP_CDEG_MSB:`VIZ_EXT_ENV_TEMP_CDEG_LSB] = ext_env_temp_cdeg_q;
        viz_bundle_r[`VIZ_EXT_ENV_RH_CENTI_MSB:`VIZ_EXT_ENV_RH_CENTI_LSB] = ext_env_rh_centi_q;
        viz_bundle_r[`VIZ_EXT_SUN_LUMA_MSB:`VIZ_EXT_SUN_LUMA_LSB] = ext_sun_luma_q;
        viz_bundle_r[`VIZ_EXT_FLOW_DX_MSB:`VIZ_EXT_FLOW_DX_LSB] = ext_flow_dx_q;
        viz_bundle_r[`VIZ_EXT_FLOW_DY_MSB:`VIZ_EXT_FLOW_DY_LSB] = ext_flow_dy_q;
        viz_bundle_r[`VIZ_EXT_LOG_SEQ_MSB:`VIZ_EXT_LOG_SEQ_LSB] = ext_log_seq_q;
        viz_bundle_r[`VIZ_EXT_LOG_DROP_MSB:`VIZ_EXT_LOG_DROP_LSB] = ext_log_drop_count_q;
        viz_bundle_r[`VIZ_EXT_MAX_AGE_MS_MSB:`VIZ_EXT_MAX_AGE_MS_LSB] = ext_max_age_ms_q;

        viz_bundle_r[`VIZ_PWR_VALID_BIT]                         = pwr_valid_q;
        viz_bundle_r[`VIZ_PWR_STATUS_MSB:`VIZ_PWR_STATUS_LSB]    = pwr_status_q;
        viz_bundle_r[`VIZ_PWR_SEQ_MSB:`VIZ_PWR_SEQ_LSB]          = pwr_seq_q;
        viz_bundle_r[`VIZ_PWR_AGE_MS_MSB:`VIZ_PWR_AGE_MS_LSB]    = pwr_age_ms_q;
        viz_bundle_r[`VIZ_PWR_VOLT_CODE_MSB:`VIZ_PWR_VOLT_CODE_LSB] = pwr_payload_q[39:28];
        viz_bundle_r[`VIZ_PWR_CURR_CODE_MSB:`VIZ_PWR_CURR_CODE_LSB] = pwr_payload_q[27:16];
        viz_bundle_r[`VIZ_PWR_ALERT_MSB:`VIZ_PWR_ALERT_LSB]      = pwr_payload_q[47:40];
        viz_bundle_r[`VIZ_PWR_RSVD_MSB:`VIZ_PWR_RSVD_LSB]        = 7'd0;

        viz_bundle_r[`VIZ_NAV_VALID_BIT]                         = nav_valid_q;
        viz_bundle_r[`VIZ_NAV_STATUS_MSB:`VIZ_NAV_STATUS_LSB]    = nav_status_q;
        viz_bundle_r[`VIZ_NAV_FLAGS_MSB:`VIZ_NAV_FLAGS_LSB]      = nav_flags_q;
        viz_bundle_r[`VIZ_NAV_DOWNRANGE_M_MSB:`VIZ_NAV_DOWNRANGE_M_LSB] = nav_downrange_m_q;
        viz_bundle_r[`VIZ_NAV_CROSSRANGE_M_MSB:`VIZ_NAV_CROSSRANGE_M_LSB] = nav_crossrange_m_q;
        viz_bundle_r[`VIZ_WIND_VALID_BIT]                        = wind_valid_q;
        viz_bundle_r[`VIZ_WIND_STATUS_MSB:`VIZ_WIND_STATUS_LSB]  = wind_status_q;
        viz_bundle_r[`VIZ_WIND_X_CMS_MSB:`VIZ_WIND_X_CMS_LSB]    = wind_x_cms_q;
        viz_bundle_r[`VIZ_WIND_Y_CMS_MSB:`VIZ_WIND_Y_CMS_LSB]    = wind_y_cms_q;
        viz_bundle_r[`VIZ_WIND_Z_CMS_MSB:`VIZ_WIND_Z_CMS_LSB]    = wind_z_cms_q;
        viz_bundle_r[`VIZ_NAV_AGE_MS_MSB:`VIZ_NAV_AGE_MS_LSB]    = nav_age_ms_q;
        viz_bundle_r[`VIZ_WIND_AGE_MS_MSB:`VIZ_WIND_AGE_MS_LSB]  = wind_age_ms_q;

        viz_bundle_r[`VIZ_AUTH_VALID_BIT]                        = auth_valid_w;
        viz_bundle_r[`VIZ_AUTH_STATUS_MSB:`VIZ_AUTH_STATUS_LSB]  = auth_status_w;
        viz_bundle_r[`VIZ_AUTH_FLAGS_MSB:`VIZ_AUTH_FLAGS_LSB]    = auth_flags_w;
        viz_bundle_r[`VIZ_AUTH_TARGET_CM_MSB:`VIZ_AUTH_TARGET_CM_LSB] = auth_target_cm_w;
        viz_bundle_r[`VIZ_AUTH_PRED_NO_CM_MSB:`VIZ_AUTH_PRED_NO_CM_LSB] = auth_pred_no_cm_w;
        viz_bundle_r[`VIZ_AUTH_PRED_FULL_CM_MSB:`VIZ_AUTH_PRED_FULL_CM_LSB] = auth_pred_full_cm_w;
        viz_bundle_r[`VIZ_AUTH_UNC_CM_MSB:`VIZ_AUTH_UNC_CM_LSB]  = auth_uncertainty_cm_w;
        viz_bundle_r[`VIZ_AUTH_CMD_U8_MSB:`VIZ_AUTH_CMD_U8_LSB]  = auth_brake_cmd_u8_w;
        viz_bundle_r[`VIZ_AUTH_SERVO_US_MSB:`VIZ_AUTH_SERVO_US_LSB] = auth_servo_us_w;
        viz_bundle_r[`VIZ_AUTH_PHASE_MSB:`VIZ_AUTH_PHASE_LSB]    = auth_phase_code_w;
        viz_bundle_r[`VIZ_AUTH_GATE_MSB:`VIZ_AUTH_GATE_LSB]      = auth_gate_flags_w;

        viz_bundle_r[`VIZ_BMP_SEQ_MSB:`VIZ_BMP_SEQ_LSB]       = bmp_seq_q;
        viz_bundle_r[`VIZ_BMP_VALID_BIT]                      = bmp_valid_q;
        viz_bundle_r[`VIZ_BMP_STATUS_MSB:`VIZ_BMP_STATUS_LSB] = bmp_status_q;
        viz_bundle_r[`VIZ_BMP_AGE_MS_MSB:`VIZ_BMP_AGE_MS_LSB] = bmp_age_ms_q;

        viz_bundle_r[`VIZ_ACC_SEQ_MSB:`VIZ_ACC_SEQ_LSB]       = acc_seq_q;
        viz_bundle_r[`VIZ_ACC_VALID_BIT]                      = acc_valid_q;
        viz_bundle_r[`VIZ_ACC_STATUS_MSB:`VIZ_ACC_STATUS_LSB] = acc_status_q;
        viz_bundle_r[`VIZ_ACC_AGE_MS_MSB:`VIZ_ACC_AGE_MS_LSB] = acc_age_ms_q;

        viz_bundle_r[`VIZ_MAG_SEQ_MSB:`VIZ_MAG_SEQ_LSB]       = mag_seq_q;
        viz_bundle_r[`VIZ_MAG_VALID_BIT]                      = mag_valid_q;
        viz_bundle_r[`VIZ_MAG_STATUS_MSB:`VIZ_MAG_STATUS_LSB] = mag_status_q;
        viz_bundle_r[`VIZ_MAG_AGE_MS_MSB:`VIZ_MAG_AGE_MS_LSB] = mag_age_ms_q;

        viz_bundle_r[`VIZ_DER_VALID_BIT]                      = der_valid_q;
        viz_bundle_r[`VIZ_DER_STATUS_MSB:`VIZ_DER_STATUS_LSB] = der_status_q;
        viz_bundle_r[`VIZ_DER_ALT_FRESH_BIT]                  = der_alt_fresh_q;
        viz_bundle_r[`VIZ_DER_VSPD_FRESH_BIT]                 = der_vspd_fresh_q;
        viz_bundle_r[`VIZ_DER_ROLL_FRESH_BIT]                 = der_roll_fresh_q;
        viz_bundle_r[`VIZ_DER_HEAD_FRESH_BIT]                 = der_head_fresh_q;

        viz_bundle_r[`VIZ_DER_BMP_SEQ_REF_MSB:`VIZ_DER_BMP_SEQ_REF_LSB] = der_bmp_seq_ref_q;
        viz_bundle_r[`VIZ_DER_ACC_SEQ_REF_MSB:`VIZ_DER_ACC_SEQ_REF_LSB] = der_acc_seq_ref_q;
        viz_bundle_r[`VIZ_DER_MAG_SEQ_REF_MSB:`VIZ_DER_MAG_SEQ_REF_LSB] = der_mag_seq_ref_q;

        viz_bundle_r[`VIZ_DER_BMP_AGE_MS_MSB:`VIZ_DER_BMP_AGE_MS_LSB] = der_bmp_age_ms_q;
        viz_bundle_r[`VIZ_DER_ACC_AGE_MS_MSB:`VIZ_DER_ACC_AGE_MS_LSB] = der_acc_age_ms_q;
        viz_bundle_r[`VIZ_DER_MAG_AGE_MS_MSB:`VIZ_DER_MAG_AGE_MS_LSB] = der_mag_age_ms_q;

        viz_bundle_r[`VIZ_DER_BMP_VALID_REF_BIT] = der_bmp_valid_ref_q;
        viz_bundle_r[`VIZ_DER_ACC_VALID_REF_BIT] = der_acc_valid_ref_q;
        viz_bundle_r[`VIZ_DER_MAG_VALID_REF_BIT] = der_mag_valid_ref_q;

        viz_bundle_r[`VIZ_DER_ALT_CM_MSB:`VIZ_DER_ALT_CM_LSB]       = der_altitude_cm_q;
        viz_bundle_r[`VIZ_DER_VSPD_CMS_MSB:`VIZ_DER_VSPD_CMS_LSB]   = der_vertical_speed_cms_q;
        viz_bundle_r[`VIZ_DER_ROLL_MDEG_MSB:`VIZ_DER_ROLL_MDEG_LSB] = der_roll_mdeg_q;
        viz_bundle_r[`VIZ_DER_HEAD_MDEG_MSB:`VIZ_DER_HEAD_MDEG_LSB] = der_heading_mdeg_q;

        viz_bundle_r[`VIZ_I2C_NACK_MSB:`VIZ_I2C_NACK_LSB] = i2c_nack_count_q;
        viz_bundle_r[`VIZ_I2C_TMO_MSB:`VIZ_I2C_TMO_LSB]   = i2c_timeout_count_q;
        viz_bundle_r[`VIZ_TXN_RATE_MSB:`VIZ_TXN_RATE_LSB] = txn_rate_hz_q;
        viz_bundle_r[`VIZ_CDC_UPD_MSB:`VIZ_CDC_UPD_LSB]   = cdc_update_count_q;
        viz_bundle_r[`VIZ_BUILD_ID_MSB:`VIZ_BUILD_ID_LSB] = build_id_q;
        viz_bundle_r[`VIZ_SCHEMA_MSB:`VIZ_SCHEMA_LSB]     = schema_word_q[3:0];
        viz_bundle_r[`VIZ_RSVD_MSB:`VIZ_RSVD_LSB]         = 1'b0;
    end

    wire _unused_raw_ok = &{
        1'b0,
        bmp_t_us_q[0], acc_t_us_q[0], mag_t_us_q[0], pwr_t_us_q[0],
        bmp_payload[0], acc_payload[0], mag_payload[0]
    };

endmodule

`default_nettype wire
