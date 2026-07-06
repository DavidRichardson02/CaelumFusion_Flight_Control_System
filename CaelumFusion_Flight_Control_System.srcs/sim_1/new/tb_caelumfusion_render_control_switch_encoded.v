`timescale 1ns/1ps
`default_nettype none

`ifdef CAELUMFUSION_RENDER_CONTROL_STUB_COMPASS_PAGE
// The control contract does not depend on the compass renderer's pixels.  This
// opt-in stub keeps focused CLI runs small while normal project simulations can
// elaborate the real renderer by leaving the macro undefined.
module planar_compass_truth_page_vga #(
    parameter integer SYS_CLK_HZ = 100_000_000,
    parameter integer UI_UPDATE_HZ = 1000,
    parameter integer H_ACTIVE = 640,
    parameter integer H_FP     = 16,
    parameter integer H_SYNC   = 96,
    parameter integer H_BP     = 48,
    parameter integer V_ACTIVE = 480,
    parameter integer V_FP     = 10,
    parameter integer V_SYNC   = 2,
    parameter integer V_BP     = 33,
    parameter integer MAG_PLOT_SHIFT = 8,
    parameter integer COMPASS_TRUTH_PAGE_DEFAULT = 0
)(
    input  wire        sys_clk,
    input  wire        sys_rst,
    input  wire        page_enable_sys,
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
    input  wire        der_valid,
    input  wire [7:0]  der_status,
    input  wire        der_head_fresh,
    input  wire [15:0] der_mag_seq_ref,
    input  wire [31:0] der_heading_mdeg,
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
    input  wire [15:0] ext_max_age_ms,
    input  wire [15:0] i2c_nack_count,
    input  wire [15:0] i2c_timeout_count,
    input  wire [15:0] txn_rate_hz,
    input  wire        pix_clk,
    input  wire        pix_rst,
    input  wire        vga_hsync_in,
    input  wire        vga_vsync_in,
    input  wire [11:0] vga_rgb_in,
    output wire        vga_hsync_out,
    output wire        vga_vsync_out,
    output wire [11:0] vga_rgb_out
);
    assign vga_hsync_out = vga_hsync_in;
    assign vga_vsync_out = vga_vsync_in;
    assign vga_rgb_out   = page_enable_sys ? 12'h123 : vga_rgb_in;
endmodule
`endif

//==============================================================================
// tb_caelumfusion_render_control_switch_encoded
//------------------------------------------------------------------------------
// Focused render-control contract bench for the Basys-3 switch-encoded direct
// view selector.
//
// Proven properties:
//   1) USE_SWITCH_ENCODED_VIEW_SELECT=1 maps SW13:SW11 into the direct view ID.
//   2) Next/previous page pulses traverse the compiled legal view sequence.
//   3) Direct encodings 000..110 are exercised with compass page disabled.
//   4) Disabled compass-page requests and reserved encoding 111 are
//      rejected without changing the registered view.
//   5) Legacy SW2/SW4/SW14 hold priority is preserved, including SW4/SW14
//      overriding SW2 when the compass page exists.
//   6) SW3 MAG1-bench mode and SW2+SW6 diagnostic-fault mode convert BTNR
//      direct requests into reserved ID 111, preserving the registered view.
//   7) Reset clears latched invalid-view diagnostics.
//==============================================================================
module tb_caelumfusion_render_control_switch_encoded;

    localparam [2:0] VIEW_FLIGHT_HUD    = 3'd0;
    localparam [2:0] VIEW_COMPASS_TRUTH = 3'd1;
    localparam [2:0] VIEW_SELFTEST_HUD  = 3'd2;
    localparam [2:0] VIEW_SENSOR_DIAG   = 3'd3;
    localparam [2:0] VIEW_SCIENCE_EXPLAIN   = 3'd4;
    localparam [2:0] VIEW_SCIENCE_WIND      = 3'd5;
    localparam [2:0] VIEW_SCIENCE_INTEGRITY = 3'd6;

    localparam integer USE_SWITCH_ENCODED_VIEW_SELECT = 1;

    reg sys_clk;
    reg sys_rst;

    reg btn_direct_disabled;
    reg btn_direct_enabled;
    reg btn_next_disabled;
    reg btn_prev_disabled;
    reg btn_next_enabled;
    reg btn_prev_enabled;
    reg cfg_clear_disabled;
    reg cfg_clear_enabled;

    reg sw11_direct_bit0;
    reg sw12_direct_bit1;
    reg sw13_direct_bit2;

    reg sw2_selftest_hold;
    reg sw3_mag1_bench_mode;
    reg sw4_compass_hold;
    reg sw6_log_diag_mode;
    reg sw14_compass_hold;
    reg sw5_history_freeze;

    wire [2:0] encoded_view_id_sys =
        (USE_SWITCH_ENCODED_VIEW_SELECT != 0) ?
        {sw13_direct_bit2, sw12_direct_bit1, sw11_direct_bit0} :
        VIEW_COMPASS_TRUTH;

    wire legacy_compass_hold_sys = sw4_compass_hold | sw14_compass_hold;
    wire diag_fault_inject_sys = sw2_selftest_hold & sw6_log_diag_mode;
    wire legacy_selftest_hold_sys = sw2_selftest_hold & !diag_fault_inject_sys;

    wire       direct_valid_disabled;
    wire [2:0] direct_view_id_disabled;
    wire       direct_collision_disabled;
    wire       direct_valid_enabled;
    wire [2:0] direct_view_id_enabled;
    wire       direct_collision_enabled;

    wire [2:0] view_sel_disabled;
    wire [2:0] view_effective_disabled;
    wire       view_changed_disabled;
    wire       invalid_disabled;
    wire       selftest_disabled;
    wire       compass_disabled;
    wire       hsync_disabled;
    wire       vsync_disabled;
    wire [11:0] rgb_disabled;

    wire [2:0] view_sel_enabled;
    wire [2:0] view_effective_enabled;
    wire       view_changed_enabled;
    wire       invalid_enabled;
    wire       selftest_enabled;
    wire       compass_enabled;
    wire       hsync_enabled;
    wire       vsync_enabled;
    wire [11:0] rgb_enabled;

    integer errors;

    initial begin
        sys_clk = 1'b0;
        forever #5 sys_clk = ~sys_clk;
    end

    caelumfusion_vga_direct_view_arbiter_sys #(
        .USE_SWITCH_ENCODED_VIEW_SELECT(USE_SWITCH_ENCODED_VIEW_SELECT)
    ) direct_arbiter_disabled (
        .direct_button_pulse_sys(btn_direct_disabled),
        .sw_mag1_bench_level_sys(sw3_mag1_bench_mode),
        .diag_fault_inject_sys(diag_fault_inject_sys),
        .sw_view_id0_level_sys(sw11_direct_bit0),
        .sw_view_id1_level_sys(sw12_direct_bit1),
        .sw_view_id2_level_sys(sw13_direct_bit2),
        .view_direct_valid_sys(direct_valid_disabled),
        .view_direct_id_sys(direct_view_id_disabled),
        .selector_collision_sys(direct_collision_disabled)
    );

    caelumfusion_vga_direct_view_arbiter_sys #(
        .USE_SWITCH_ENCODED_VIEW_SELECT(USE_SWITCH_ENCODED_VIEW_SELECT)
    ) direct_arbiter_enabled (
        .direct_button_pulse_sys(btn_direct_enabled),
        .sw_mag1_bench_level_sys(sw3_mag1_bench_mode),
        .diag_fault_inject_sys(diag_fault_inject_sys),
        .sw_view_id0_level_sys(sw11_direct_bit0),
        .sw_view_id1_level_sys(sw12_direct_bit1),
        .sw_view_id2_level_sys(sw13_direct_bit2),
        .view_direct_valid_sys(direct_valid_enabled),
        .view_direct_id_sys(direct_view_id_enabled),
        .selector_collision_sys(direct_collision_enabled)
    );

    //--------------------------------------------------------------------------
    // DUT with the compass page compiled out. This is the default resource-safe
    // Basys-3 profile and is where direct ID 001 must be rejected.
    //--------------------------------------------------------------------------
    caelumfusion_vga_render_control #(
        .ENABLE_SENSOR_DIAG_PAGE(0),
        .ENABLE_COMPASS_TRUTH_PAGE(0),
        .ENABLE_SCIENCE_PAGES(1),
        .ENABLE_TELEMETRY_TEXT_OVERLAY(0),
        .COMPASS_TRUTH_PAGE_DEFAULT(0),
        .RESET_VIEW_ID(VIEW_FLIGHT_HUD)
    ) dut_compass_disabled (
        .sys_clk(sys_clk),
        .sys_rst(sys_rst),
        .view_next_pulse_sys(btn_next_disabled),
        .view_prev_pulse_sys(btn_prev_disabled),
        .view_direct_valid_sys(direct_valid_disabled),
        .view_direct_id_sys(direct_view_id_disabled),
        .cfg_invalid_view_clear_sys(cfg_clear_disabled),
        .legacy_compass_page_hold_sys(legacy_compass_hold_sys),
        .legacy_selftest_hold_sys(legacy_selftest_hold_sys),
        .history_freeze_sys(sw5_history_freeze),
        .direct_selector_collision_sys(direct_collision_disabled),
        .view_sel_sys(view_sel_disabled),
        .view_effective_sys(view_effective_disabled),
        .view_changed_pulse_sys(view_changed_disabled),
        .cfg_invalid_view_sys(invalid_disabled),
        .flight_selftest_en_sys(selftest_disabled),
        .compass_page_enable_sys(compass_disabled),
        .bmp_t_us(32'd0),
        .bmp_seq(16'd0),
        .bmp_valid(1'b0),
        .bmp_status(8'd0),
        .bmp_payload(48'd0),
        .bmp_age_ms(16'd0),
        .acc_t_us(32'd0),
        .acc_seq(16'd0),
        .acc_valid(1'b0),
        .acc_status(8'd0),
        .acc_payload(48'd0),
        .acc_age_ms(16'd0),
        .mag_t_us(32'd0),
        .mag_seq(16'd0),
        .mag_valid(1'b0),
        .mag_status(8'd0),
        .mag_payload(48'd0),
        .mag_age_ms(16'd0),
        .mag1_seq(16'd0),
        .mag1_valid(1'b0),
        .mag1_status(8'd0),
        .mag1_payload(48'd0),
        .mag1_age_ms(16'd0),
        .pwr_t_us(32'd0),
        .pwr_seq(16'd0),
        .pwr_valid(1'b0),
        .pwr_status(8'd0),
        .pwr_payload(48'd0),
        .pwr_age_ms(16'd0),
        .ext_valid(1'b0),
        .ext_status(8'd0),
        .ext_present_flags(16'd0),
        .ext_fault_flags(16'd0),
        .ext_mag_delta_l1(16'd0),
        .ext_mag_norm_primary(16'd0),
        .ext_mag_norm_secondary(16'd0),
        .ext_mag_sequence_aligned(1'b0),
        .ext_mag_disagreement(1'b0),
        .ext_mag_sector_delta(4'd0),
        .ext_mag_norm_delta_l1(16'd0),
        .ext_mag_iron_residual(16'd0),
        .ext_mag_cal_state(8'd0),
        .ext_mag_source_flags(8'd0),
        .ext_mag_bridge_checksum(16'd0),
        .ext_rng_height_cm(16'd0),
        .ext_air_dp_pa(16'd0),
        .ext_air_speed_cms(16'd0),
        .ext_env_temp_cdeg(16'd0),
        .ext_env_rh_centi(16'd0),
        .ext_sun_luma(16'd0),
        .ext_flow_dx(16'd0),
        .ext_flow_dy(16'd0),
        .ext_log_seq(16'd0),
        .ext_log_drop_count(16'd0),
        .ext_max_age_ms(16'd0),
        .der_valid(1'b0),
        .der_status(8'd0),
        .der_alt_fresh(1'b0),
        .der_vspd_fresh(1'b0),
        .der_roll_fresh(1'b0),
        .der_head_fresh(1'b0),
        .der_bmp_seq_ref(16'd0),
        .der_acc_seq_ref(16'd0),
        .der_mag_seq_ref(16'd0),
        .der_bmp_age_ms(16'd0),
        .der_acc_age_ms(16'd0),
        .der_mag_age_ms(16'd0),
        .der_bmp_valid_ref(1'b0),
        .der_acc_valid_ref(1'b0),
        .der_mag_valid_ref(1'b0),
        .der_altitude_cm(32'd0),
        .der_vertical_speed_cms(32'd0),
        .der_roll_mdeg(32'd0),
        .der_heading_mdeg(32'd0),
        .nav_valid(1'b0),
        .nav_status(8'd0),
        .nav_flags(8'd0),
        .nav_downrange_m(16'd0),
        .nav_crossrange_m(16'd0),
        .nav_age_ms(16'd0),
        .wind_valid(1'b0),
        .wind_status(8'd0),
        .wind_x_cms(16'd0),
        .wind_y_cms(16'd0),
        .wind_z_cms(16'd0),
        .wind_age_ms(16'd0),
        .auth_phase_code_sys(4'd0),
        .auth_phase_valid_sys(1'b0),
        .safety_runtime_ok_sys(1'b0),
        .safety_allows_actuation_sys(1'b0),
        .policy_runtime_enable_sys(1'b0),
        .software_armed_sys(1'b0),
        .i2c_nack_count(16'd0),
        .i2c_timeout_count(16'd0),
        .txn_rate_hz(16'd0),
        .cdc_update_count_sys(32'd0),
        .build_id(32'd0),
        .schema_word(16'd0),
        .pix_clk(sys_clk),
        .pix_rst(sys_rst),
        .vga_hsync(hsync_disabled),
        .vga_vsync(vsync_disabled),
        .vga_rgb(rgb_disabled)
    );

    //--------------------------------------------------------------------------
    // DUT with the compass page compiled in. This isolates legacy hold priority
    // without depending on the direct-selection disabled-compass rejection case.
    //--------------------------------------------------------------------------
    caelumfusion_vga_render_control #(
        .ENABLE_SENSOR_DIAG_PAGE(0),
        .ENABLE_COMPASS_TRUTH_PAGE(1),
        .ENABLE_SCIENCE_PAGES(1),
        .ENABLE_TELEMETRY_TEXT_OVERLAY(0),
        .COMPASS_TRUTH_PAGE_DEFAULT(0),
        .RESET_VIEW_ID(VIEW_FLIGHT_HUD)
    ) dut_compass_enabled (
        .sys_clk(sys_clk),
        .sys_rst(sys_rst),
        .view_next_pulse_sys(btn_next_enabled),
        .view_prev_pulse_sys(btn_prev_enabled),
        .view_direct_valid_sys(direct_valid_enabled),
        .view_direct_id_sys(direct_view_id_enabled),
        .cfg_invalid_view_clear_sys(cfg_clear_enabled),
        .legacy_compass_page_hold_sys(legacy_compass_hold_sys),
        .legacy_selftest_hold_sys(legacy_selftest_hold_sys),
        .history_freeze_sys(sw5_history_freeze),
        .direct_selector_collision_sys(direct_collision_enabled),
        .view_sel_sys(view_sel_enabled),
        .view_effective_sys(view_effective_enabled),
        .view_changed_pulse_sys(view_changed_enabled),
        .cfg_invalid_view_sys(invalid_enabled),
        .flight_selftest_en_sys(selftest_enabled),
        .compass_page_enable_sys(compass_enabled),
        .bmp_t_us(32'd0),
        .bmp_seq(16'd0),
        .bmp_valid(1'b0),
        .bmp_status(8'd0),
        .bmp_payload(48'd0),
        .bmp_age_ms(16'd0),
        .acc_t_us(32'd0),
        .acc_seq(16'd0),
        .acc_valid(1'b0),
        .acc_status(8'd0),
        .acc_payload(48'd0),
        .acc_age_ms(16'd0),
        .mag_t_us(32'd0),
        .mag_seq(16'd0),
        .mag_valid(1'b0),
        .mag_status(8'd0),
        .mag_payload(48'd0),
        .mag_age_ms(16'd0),
        .mag1_seq(16'd0),
        .mag1_valid(1'b0),
        .mag1_status(8'd0),
        .mag1_payload(48'd0),
        .mag1_age_ms(16'd0),
        .pwr_t_us(32'd0),
        .pwr_seq(16'd0),
        .pwr_valid(1'b0),
        .pwr_status(8'd0),
        .pwr_payload(48'd0),
        .pwr_age_ms(16'd0),
        .ext_valid(1'b0),
        .ext_status(8'd0),
        .ext_present_flags(16'd0),
        .ext_fault_flags(16'd0),
        .ext_mag_delta_l1(16'd0),
        .ext_mag_norm_primary(16'd0),
        .ext_mag_norm_secondary(16'd0),
        .ext_mag_sequence_aligned(1'b0),
        .ext_mag_disagreement(1'b0),
        .ext_mag_sector_delta(4'd0),
        .ext_mag_norm_delta_l1(16'd0),
        .ext_mag_iron_residual(16'd0),
        .ext_mag_cal_state(8'd0),
        .ext_mag_source_flags(8'd0),
        .ext_mag_bridge_checksum(16'd0),
        .ext_rng_height_cm(16'd0),
        .ext_air_dp_pa(16'd0),
        .ext_air_speed_cms(16'd0),
        .ext_env_temp_cdeg(16'd0),
        .ext_env_rh_centi(16'd0),
        .ext_sun_luma(16'd0),
        .ext_flow_dx(16'd0),
        .ext_flow_dy(16'd0),
        .ext_log_seq(16'd0),
        .ext_log_drop_count(16'd0),
        .ext_max_age_ms(16'd0),
        .der_valid(1'b0),
        .der_status(8'd0),
        .der_alt_fresh(1'b0),
        .der_vspd_fresh(1'b0),
        .der_roll_fresh(1'b0),
        .der_head_fresh(1'b0),
        .der_bmp_seq_ref(16'd0),
        .der_acc_seq_ref(16'd0),
        .der_mag_seq_ref(16'd0),
        .der_bmp_age_ms(16'd0),
        .der_acc_age_ms(16'd0),
        .der_mag_age_ms(16'd0),
        .der_bmp_valid_ref(1'b0),
        .der_acc_valid_ref(1'b0),
        .der_mag_valid_ref(1'b0),
        .der_altitude_cm(32'd0),
        .der_vertical_speed_cms(32'd0),
        .der_roll_mdeg(32'd0),
        .der_heading_mdeg(32'd0),
        .nav_valid(1'b0),
        .nav_status(8'd0),
        .nav_flags(8'd0),
        .nav_downrange_m(16'd0),
        .nav_crossrange_m(16'd0),
        .nav_age_ms(16'd0),
        .wind_valid(1'b0),
        .wind_status(8'd0),
        .wind_x_cms(16'd0),
        .wind_y_cms(16'd0),
        .wind_z_cms(16'd0),
        .wind_age_ms(16'd0),
        .auth_phase_code_sys(4'd0),
        .auth_phase_valid_sys(1'b0),
        .safety_runtime_ok_sys(1'b0),
        .safety_allows_actuation_sys(1'b0),
        .policy_runtime_enable_sys(1'b0),
        .software_armed_sys(1'b0),
        .i2c_nack_count(16'd0),
        .i2c_timeout_count(16'd0),
        .txn_rate_hz(16'd0),
        .cdc_update_count_sys(32'd0),
        .build_id(32'd0),
        .schema_word(16'd0),
        .pix_clk(sys_clk),
        .pix_rst(sys_rst),
        .vga_hsync(hsync_enabled),
        .vga_vsync(vsync_enabled),
        .vga_rgb(rgb_enabled)
    );

    task fail;
        input [8*96-1:0] msg;
        begin
            errors = errors + 1;
            $display("FAIL: %0s at %0t", msg, $time);
        end
    endtask

    task check3;
        input [8*96-1:0] msg;
        input [2:0] actual;
        input [2:0] expected;
        begin
            if (actual !== expected) begin
                $display("FAIL: %0s actual=%0d expected=%0d at %0t",
                         msg, actual, expected, $time);
                errors = errors + 1;
            end
        end
    endtask

    task check1;
        input [8*96-1:0] msg;
        input actual;
        input expected;
        begin
            if (actual !== expected) begin
                $display("FAIL: %0s actual=%0b expected=%0b at %0t",
                         msg, actual, expected, $time);
                errors = errors + 1;
            end
        end
    endtask

    task set_encoded_switches;
        input [2:0] encoding;
        begin
            sw11_direct_bit0 = encoding[0];
            sw12_direct_bit1 = encoding[1];
            sw13_direct_bit2 = encoding[2];
            #1;
            check3("SW13:SW11 encoded direct ID", encoded_view_id_sys, encoding);
        end
    endtask

    task clear_invalid_disabled;
        begin
            @(negedge sys_clk);
            cfg_clear_disabled = 1'b1;
            @(posedge sys_clk);
            #1;
            check1("disabled invalid flag clears", invalid_disabled, 1'b0);
            @(negedge sys_clk);
            cfg_clear_disabled = 1'b0;
        end
    endtask

    task next_disabled_expect;
        input [2:0] expected_view;
        begin
            @(negedge sys_clk);
            btn_next_disabled = 1'b1;
            btn_prev_disabled = 1'b0;
            @(posedge sys_clk);
            #1;
            check3("disabled next registered view",
                   view_sel_disabled, expected_view);
            check3("disabled next effective view",
                   view_effective_disabled, expected_view);
            check1("disabled next changed pulse",
                   view_changed_disabled, 1'b1);
            check1("disabled next leaves invalid clear",
                   invalid_disabled, 1'b0);
            @(negedge sys_clk);
            btn_next_disabled = 1'b0;
            @(posedge sys_clk);
            #1;
            check1("disabled next changed pulse is one cycle",
                   view_changed_disabled, 1'b0);
        end
    endtask

    task prev_disabled_expect;
        input [2:0] expected_view;
        begin
            @(negedge sys_clk);
            btn_next_disabled = 1'b0;
            btn_prev_disabled = 1'b1;
            @(posedge sys_clk);
            #1;
            check3("disabled previous registered view",
                   view_sel_disabled, expected_view);
            check3("disabled previous effective view",
                   view_effective_disabled, expected_view);
            check1("disabled previous changed pulse",
                   view_changed_disabled, 1'b1);
            check1("disabled previous leaves invalid clear",
                   invalid_disabled, 1'b0);
            @(negedge sys_clk);
            btn_prev_disabled = 1'b0;
            @(posedge sys_clk);
            #1;
            check1("disabled previous changed pulse is one cycle",
                   view_changed_disabled, 1'b0);
        end
    endtask

    task reset_clears_invalid_expect;
        begin
            set_encoded_switches(3'b111);
            @(negedge sys_clk);
            btn_direct_disabled = 1'b1;
            btn_direct_enabled = 1'b1;
            @(posedge sys_clk);
            #1;
            check1("disabled invalid set before reset",
                   invalid_disabled, 1'b1);
            check1("enabled invalid set before reset",
                   invalid_enabled, 1'b1);

            @(negedge sys_clk);
            btn_direct_disabled = 1'b0;
            btn_direct_enabled = 1'b0;
            sys_rst = 1'b1;
            @(posedge sys_clk);
            #1;
            check1("disabled invalid clears on reset",
                   invalid_disabled, 1'b0);
            check1("enabled invalid clears on reset",
                   invalid_enabled, 1'b0);
            check3("disabled reset view after invalid",
                   view_sel_disabled, VIEW_FLIGHT_HUD);
            check3("enabled reset view after invalid",
                   view_sel_enabled, VIEW_FLIGHT_HUD);
            @(negedge sys_clk);
            sys_rst = 1'b0;
            @(posedge sys_clk);
            #1;
        end
    endtask

    task direct_disabled_expect;
        input [2:0] encoding;
        input [2:0] expected_view;
        input expected_changed;
        input expected_invalid;
        begin
            set_encoded_switches(encoding);
            check1("disabled direct selector collision inactive",
                   direct_collision_disabled, 1'b0);
            check3("disabled direct arbiter passes encoded ID",
                   direct_view_id_disabled, encoding);
            @(negedge sys_clk);
            btn_direct_disabled = 1'b1;
            @(posedge sys_clk);
            #1;
            check1("disabled direct valid follows button",
                   direct_valid_disabled, 1'b1);
            check3("disabled registered view after direct request",
                   view_sel_disabled, expected_view);
            check3("disabled effective view after direct request",
                   view_effective_disabled, expected_view);
            check1("disabled changed pulse after direct request",
                   view_changed_disabled, expected_changed);
            check1("disabled invalid flag after direct request",
                   invalid_disabled, expected_invalid);
            @(negedge sys_clk);
            btn_direct_disabled = 1'b0;
            @(posedge sys_clk);
            #1;
            check1("disabled changed pulse is one cycle",
                   view_changed_disabled, 1'b0);
        end
    endtask

    task direct_disabled_collision_expect;
        input [2:0] encoding;
        input mag1_bench_active;
        input fault_inject_active;
        input [2:0] expected_hold_view;
        begin
            set_encoded_switches(encoding);
            @(negedge sys_clk);
            sw3_mag1_bench_mode = mag1_bench_active;
            sw2_selftest_hold = fault_inject_active;
            sw6_log_diag_mode = fault_inject_active;
            btn_direct_disabled = 1'b1;
            @(posedge sys_clk);
            #1;
            check1("collision direct valid follows button",
                   direct_valid_disabled, 1'b1);
            check1("collision arbiter flags SW11:SW13 ownership",
                   direct_collision_disabled, 1'b1);
            check3("collision arbiter emits reserved direct ID",
                   direct_view_id_disabled, 3'b111);
            check3("collision preserves registered view",
                   view_sel_disabled, expected_hold_view);
            check3("collision preserves effective view",
                   view_effective_disabled, expected_hold_view);
            check1("collision direct request has no change pulse",
                   view_changed_disabled, 1'b0);
            check1("collision direct request is rejected",
                   invalid_disabled, 1'b1);
            @(negedge sys_clk);
            btn_direct_disabled = 1'b0;
            sw3_mag1_bench_mode = 1'b0;
            sw2_selftest_hold = 1'b0;
            sw6_log_diag_mode = 1'b0;
            @(posedge sys_clk);
            #1;
            check1("collision changed pulse remains clear after release",
                   view_changed_disabled, 1'b0);
            check1("collision flag clears after owner switches clear",
                   direct_collision_disabled, 1'b0);
        end
    endtask

    task direct_enabled_expect;
        input [2:0] encoding;
        input [2:0] expected_view;
        begin
            set_encoded_switches(encoding);
            check1("enabled direct selector collision inactive",
                   direct_collision_enabled, 1'b0);
            check3("enabled direct arbiter passes encoded ID",
                   direct_view_id_enabled, encoding);
            @(negedge sys_clk);
            btn_direct_enabled = 1'b1;
            @(posedge sys_clk);
            #1;
            check1("enabled direct valid follows button",
                   direct_valid_enabled, 1'b1);
            check3("enabled registered view after direct request",
                   view_sel_enabled, expected_view);
            check1("enabled changed pulse after direct request",
                   view_changed_enabled, 1'b1);
            @(negedge sys_clk);
            btn_direct_enabled = 1'b0;
            @(posedge sys_clk);
            #1;
            check1("enabled changed pulse is one cycle",
                   view_changed_enabled, 1'b0);
        end
    endtask

    task check_disabled_hold_state;
        input [2:0] expected_effective;
        input expected_selftest;
        begin
            #1;
            check3("disabled hold effective view",
                   view_effective_disabled, expected_effective);
            check1("disabled hold selftest enable",
                   selftest_disabled, expected_selftest);
            check1("disabled hold compass enable",
                   compass_disabled, 1'b0);
        end
    endtask

    task check_enabled_hold_state;
        input [2:0] expected_effective;
        input expected_selftest;
        input expected_compass;
        begin
            #1;
            check3("enabled hold effective view",
                   view_effective_enabled, expected_effective);
            check1("enabled hold selftest enable",
                   selftest_enabled, expected_selftest);
            check1("enabled hold compass enable",
                   compass_enabled, expected_compass);
        end
    endtask

    initial begin
        errors = 0;
        sys_rst = 1'b1;
        btn_direct_disabled = 1'b0;
        btn_direct_enabled = 1'b0;
        btn_next_disabled = 1'b0;
        btn_prev_disabled = 1'b0;
        btn_next_enabled = 1'b0;
        btn_prev_enabled = 1'b0;
        cfg_clear_disabled = 1'b0;
        cfg_clear_enabled = 1'b0;
        sw11_direct_bit0 = 1'b0;
        sw12_direct_bit1 = 1'b0;
        sw13_direct_bit2 = 1'b0;
        sw2_selftest_hold = 1'b0;
        sw3_mag1_bench_mode = 1'b0;
        sw4_compass_hold = 1'b0;
        sw6_log_diag_mode = 1'b0;
        sw14_compass_hold = 1'b0;
        sw5_history_freeze = 1'b0;

        repeat (4) @(posedge sys_clk);
        sys_rst = 1'b0;
        repeat (2) @(posedge sys_clk);
        #1;

        check3("disabled reset registered view",
               view_sel_disabled, VIEW_FLIGHT_HUD);
        check3("enabled reset registered view",
               view_sel_enabled, VIEW_FLIGHT_HUD);

        // Cyclic navigation with the default Basys-3 compass-disabled profile.
        next_disabled_expect(VIEW_SENSOR_DIAG);
        next_disabled_expect(VIEW_SCIENCE_EXPLAIN);
        next_disabled_expect(VIEW_SCIENCE_WIND);
        next_disabled_expect(VIEW_SCIENCE_INTEGRITY);
        next_disabled_expect(VIEW_SELFTEST_HUD);
        next_disabled_expect(VIEW_FLIGHT_HUD);

        prev_disabled_expect(VIEW_SELFTEST_HUD);
        prev_disabled_expect(VIEW_SCIENCE_INTEGRITY);
        prev_disabled_expect(VIEW_SCIENCE_WIND);
        prev_disabled_expect(VIEW_SCIENCE_EXPLAIN);
        prev_disabled_expect(VIEW_SENSOR_DIAG);
        prev_disabled_expect(VIEW_FLIGHT_HUD);

        // Encodings 000..110 with the compass page disabled. 001 is the
        // disabled compass page and must be rejected without changing view_sel.
        direct_disabled_expect(3'b000, VIEW_FLIGHT_HUD, 1'b0, 1'b0);
        direct_disabled_expect(3'b001, VIEW_FLIGHT_HUD, 1'b0, 1'b1);
        clear_invalid_disabled();
        direct_disabled_expect(3'b010, VIEW_SELFTEST_HUD, 1'b1, 1'b0);
        direct_disabled_expect(3'b011, VIEW_SENSOR_DIAG, 1'b1, 1'b0);
        direct_disabled_expect(3'b100, VIEW_SCIENCE_EXPLAIN, 1'b1, 1'b0);
        direct_disabled_expect(3'b101, VIEW_SCIENCE_WIND, 1'b1, 1'b0);
        direct_disabled_expect(3'b110, VIEW_SCIENCE_INTEGRITY, 1'b1, 1'b0);

        // Reserved direct IDs must reject and preserve the last legal view.
        direct_disabled_expect(3'b111, VIEW_SCIENCE_INTEGRITY, 1'b0, 1'b1);
        clear_invalid_disabled();

        // When SW11:SW13 are owned by MAG1 bench or diagnostic fault injection,
        // BTNR produces a deterministic reserved request instead of navigating.
        direct_disabled_collision_expect(3'b010, 1'b1, 1'b0,
                                         VIEW_SCIENCE_INTEGRITY);
        clear_invalid_disabled();
        direct_disabled_collision_expect(3'b101, 1'b0, 1'b1,
                                         VIEW_SCIENCE_INTEGRITY);
        clear_invalid_disabled();

        // Disabled compass page: SW4/SW14 do not override the registered view.
        sw4_compass_hold = 1'b1;
        sw14_compass_hold = 1'b0;
        sw2_selftest_hold = 1'b0;
        check_disabled_hold_state(VIEW_SCIENCE_INTEGRITY, 1'b0);

        sw2_selftest_hold = 1'b1;
        check_disabled_hold_state(VIEW_SELFTEST_HUD, 1'b1);

        sw4_compass_hold = 1'b0;
        sw14_compass_hold = 1'b1;
        check_disabled_hold_state(VIEW_SELFTEST_HUD, 1'b1);

        sw2_selftest_hold = 1'b0;
        check_disabled_hold_state(VIEW_SCIENCE_INTEGRITY, 1'b0);
        sw14_compass_hold = 1'b0;

        // Compass-enabled hold priority: registered view < SW2 < SW4/SW14.
        sw2_selftest_hold = 1'b0;
        sw4_compass_hold = 1'b0;
        sw14_compass_hold = 1'b0;
        direct_enabled_expect(3'b011, VIEW_SENSOR_DIAG);
        check_enabled_hold_state(VIEW_SENSOR_DIAG, 1'b0, 1'b0);

        sw2_selftest_hold = 1'b1;
        check_enabled_hold_state(VIEW_SELFTEST_HUD, 1'b1, 1'b0);

        sw4_compass_hold = 1'b1;
        check_enabled_hold_state(VIEW_COMPASS_TRUTH, 1'b0, 1'b1);

        sw4_compass_hold = 1'b0;
        sw14_compass_hold = 1'b1;
        check_enabled_hold_state(VIEW_COMPASS_TRUTH, 1'b0, 1'b1);

        sw14_compass_hold = 1'b0;
        check_enabled_hold_state(VIEW_SELFTEST_HUD, 1'b1, 1'b0);

        sw2_selftest_hold = 1'b0;
        check_enabled_hold_state(VIEW_SENSOR_DIAG, 1'b0, 1'b0);

        reset_clears_invalid_expect();

        if (errors == 0) begin
            $display("PASS: tb_caelumfusion_render_control_switch_encoded");
            $finish;
        end else begin
            $display("FAIL: tb_caelumfusion_render_control_switch_encoded errors=%0d",
                     errors);
            $finish;
        end
    end

endmodule

`default_nettype wire
