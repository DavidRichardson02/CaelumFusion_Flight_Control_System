`timescale 1ns/1ps
`default_nettype none

`include "flight_viz_bundle_defs.vh"
`include "telemetry_defs_vh.vh"

//==============================================================================
// tb_flight_viz_suite_top
//------------------------------------------------------------------------------
// ROLE
//   Self-checking transport-and-render testbench for flight_viz_suite_top.
//
// PROVEN PROPERTIES
//   1) SYS semantic update causes a packed bundle publication.
//   2) The published bundle crosses into PIX and appears in the PIX shadow bus.
//   3) The PIX history rings store the transferred altitude / vertical speed.
//   4) End-to-end VGA output responds with the expected telemetry text pixel.
//
// TEST METHOD
//   - Drive one nominal semantic image in SYS.
//   - Wait for internal publish pulse.
//   - Wait for PIX update pulse from the CDC output.
//   - Check transferred shadow fields.
//   - Check history ring write at slot 0.
//   - Wait for a known active glyph pixel in the wrapped visualizer.
//   - Check final VGA RGB equals packed green text color.
//
// IMPORTANT
//   This is a proof-oriented testbench and therefore intentionally uses
//   hierarchical references into the DUT.
//==============================================================================
module tb_flight_viz_suite_top;

    //==========================================================================
    // Clocks / reset
    //==========================================================================
    reg sys_clk;
    reg sys_rst;

    reg pix_clk;
    reg pix_rst;

    //==========================================================================
    // DUT inputs
    //==========================================================================
    reg  [31:0]          bmp_t_us;
    reg  [15:0]          bmp_seq;
    reg                  bmp_valid;
    reg  [7:0]           bmp_status;
    reg  [47:0]          bmp_payload;
    reg  [15:0]          bmp_age_ms;

    reg  [31:0]          acc_t_us;
    reg  [15:0]          acc_seq;
    reg                  acc_valid;
    reg  [7:0]           acc_status;
    reg  [47:0]          acc_payload;
    reg  [15:0]          acc_age_ms;

    reg  [31:0]          mag_t_us;
    reg  [15:0]          mag_seq;
    reg                  mag_valid;
    reg  [7:0]           mag_status;
    reg  [47:0]          mag_payload;
    reg  [15:0]          mag_age_ms;

    reg  [31:0]          pwr_t_us;
    reg  [15:0]          pwr_seq;
    reg                  pwr_valid;
    reg  [7:0]           pwr_status;
    reg  [47:0]          pwr_payload;
    reg  [15:0]          pwr_age_ms;

    reg                  ext_valid;
    reg  [7:0]           ext_status;
    reg  [15:0]          ext_present_flags;
    reg  [15:0]          ext_fault_flags;
    reg  [15:0]          ext_mag_delta_l1;
    reg  [15:0]          ext_mag_norm_primary;
    reg  [15:0]          ext_mag_norm_secondary;
    reg                  ext_mag_sequence_aligned;
    reg                  ext_mag_disagreement;
    reg  [3:0]           ext_mag_sector_delta;
    reg  [7:0]           ext_mag_source_flags;
    reg  [15:0]          ext_rng_height_cm;
    reg  [15:0]          ext_air_dp_pa;
    reg  [15:0]          ext_air_speed_cms;
    reg  [15:0]          ext_env_temp_cdeg;
    reg  [15:0]          ext_env_rh_centi;
    reg  [15:0]          ext_sun_luma;
    reg  [15:0]          ext_flow_dx;
    reg  [15:0]          ext_flow_dy;
    reg  [15:0]          ext_log_seq;
    reg  [15:0]          ext_log_drop_count;
    reg  [15:0]          ext_max_age_ms;

    reg                  der_valid;
    reg  [7:0]           der_status;

    reg                  der_alt_fresh;
    reg                  der_vspd_fresh;
    reg                  der_roll_fresh;
    reg                  der_head_fresh;

    reg  [15:0]          der_bmp_seq_ref;
    reg  [15:0]          der_acc_seq_ref;
    reg  [15:0]          der_mag_seq_ref;

    reg  [15:0]          der_bmp_age_ms;
    reg  [15:0]          der_acc_age_ms;
    reg  [15:0]          der_mag_age_ms;

    reg                  der_bmp_valid_ref;
    reg                  der_acc_valid_ref;
    reg                  der_mag_valid_ref;

    reg  [31:0]          der_altitude_cm;
    reg  [31:0]          der_vertical_speed_cms;
    reg  [31:0]          der_roll_mdeg;
    reg  [31:0]          der_heading_mdeg;

    reg                  nav_valid;
    reg  [7:0]           nav_status;
    reg  [7:0]           nav_flags;
    reg  [15:0]          nav_downrange_m;
    reg  [15:0]          nav_crossrange_m;
    reg  [15:0]          nav_age_ms;

    reg                  wind_valid;
    reg  [7:0]           wind_status;
    reg  [15:0]          wind_x_cms;
    reg  [15:0]          wind_y_cms;
    reg  [15:0]          wind_z_cms;
    reg  [15:0]          wind_age_ms;

    reg  [3:0]           auth_phase_code_sys;
    reg                  auth_phase_valid_sys;
    reg                  safety_runtime_ok_sys;
    reg                  safety_allows_actuation_sys;
    reg                  policy_runtime_enable_sys;
    reg                  software_armed_sys;

    reg  [15:0]          i2c_nack_count;
    reg  [15:0]          i2c_timeout_count;
    reg  [15:0]          txn_rate_hz;
    reg  [31:0]          cdc_update_count_sys;
    reg  [31:0]          build_id;
    reg  [15:0]          schema_word;

    reg                  viz_selftest_en_sys;
    reg  [1:0]           vga_page_select_sys;

    //==========================================================================
    // DUT outputs
    //==========================================================================
    wire                 vga_hsync;
    wire                 vga_vsync;
    wire [11:0]          vga_rgb;

    //==========================================================================
    // DUT
    //==========================================================================
    flight_viz_suite_top #(
        .HIST_DEPTH(16)
    ) u_dut (
        .sys_clk                 (sys_clk),
        .sys_rst                 (sys_rst),

        .bmp_t_us                (bmp_t_us),
        .bmp_seq                 (bmp_seq),
        .bmp_valid               (bmp_valid),
        .bmp_status              (bmp_status),
        .bmp_payload             (bmp_payload),
        .bmp_age_ms              (bmp_age_ms),

        .acc_t_us                (acc_t_us),
        .acc_seq                 (acc_seq),
        .acc_valid               (acc_valid),
        .acc_status              (acc_status),
        .acc_payload             (acc_payload),
        .acc_age_ms              (acc_age_ms),

        .mag_t_us                (mag_t_us),
        .mag_seq                 (mag_seq),
        .mag_valid               (mag_valid),
        .mag_status              (mag_status),
        .mag_payload             (mag_payload),
        .mag_age_ms              (mag_age_ms),

        .pwr_t_us                (pwr_t_us),
        .pwr_seq                 (pwr_seq),
        .pwr_valid               (pwr_valid),
        .pwr_status              (pwr_status),
        .pwr_payload             (pwr_payload),
        .pwr_age_ms              (pwr_age_ms),

        .ext_valid               (ext_valid),
        .ext_status              (ext_status),
        .ext_present_flags       (ext_present_flags),
        .ext_fault_flags         (ext_fault_flags),
        .ext_mag_delta_l1        (ext_mag_delta_l1),
        .ext_mag_norm_primary    (ext_mag_norm_primary),
        .ext_mag_norm_secondary  (ext_mag_norm_secondary),
        .ext_mag_sequence_aligned(ext_mag_sequence_aligned),
        .ext_mag_disagreement    (ext_mag_disagreement),
        .ext_mag_sector_delta    (ext_mag_sector_delta),
        .ext_mag_source_flags    (ext_mag_source_flags),
        .ext_rng_height_cm       (ext_rng_height_cm),
        .ext_air_dp_pa           (ext_air_dp_pa),
        .ext_air_speed_cms       (ext_air_speed_cms),
        .ext_env_temp_cdeg       (ext_env_temp_cdeg),
        .ext_env_rh_centi        (ext_env_rh_centi),
        .ext_sun_luma            (ext_sun_luma),
        .ext_flow_dx             (ext_flow_dx),
        .ext_flow_dy             (ext_flow_dy),
        .ext_log_seq             (ext_log_seq),
        .ext_log_drop_count      (ext_log_drop_count),
        .ext_max_age_ms          (ext_max_age_ms),

        .der_valid               (der_valid),
        .der_status              (der_status),

        .der_alt_fresh           (der_alt_fresh),
        .der_vspd_fresh          (der_vspd_fresh),
        .der_roll_fresh          (der_roll_fresh),
        .der_head_fresh          (der_head_fresh),

        .der_bmp_seq_ref         (der_bmp_seq_ref),
        .der_acc_seq_ref         (der_acc_seq_ref),
        .der_mag_seq_ref         (der_mag_seq_ref),

        .der_bmp_age_ms          (der_bmp_age_ms),
        .der_acc_age_ms          (der_acc_age_ms),
        .der_mag_age_ms          (der_mag_age_ms),

        .der_bmp_valid_ref       (der_bmp_valid_ref),
        .der_acc_valid_ref       (der_acc_valid_ref),
        .der_mag_valid_ref       (der_mag_valid_ref),

        .der_altitude_cm         (der_altitude_cm),
        .der_vertical_speed_cms  (der_vertical_speed_cms),
        .der_roll_mdeg           (der_roll_mdeg),
        .der_heading_mdeg        (der_heading_mdeg),

        .nav_valid               (nav_valid),
        .nav_status              (nav_status),
        .nav_flags               (nav_flags),
        .nav_downrange_m         (nav_downrange_m),
        .nav_crossrange_m        (nav_crossrange_m),
        .nav_age_ms              (nav_age_ms),

        .wind_valid              (wind_valid),
        .wind_status             (wind_status),
        .wind_x_cms              (wind_x_cms),
        .wind_y_cms              (wind_y_cms),
        .wind_z_cms              (wind_z_cms),
        .wind_age_ms             (wind_age_ms),

        .auth_phase_code_sys     (auth_phase_code_sys),
        .auth_phase_valid_sys    (auth_phase_valid_sys),
        .safety_runtime_ok_sys   (safety_runtime_ok_sys),
        .safety_allows_actuation_sys(safety_allows_actuation_sys),
        .policy_runtime_enable_sys(policy_runtime_enable_sys),
        .software_armed_sys      (software_armed_sys),

        .i2c_nack_count          (i2c_nack_count),
        .i2c_timeout_count       (i2c_timeout_count),
        .txn_rate_hz             (txn_rate_hz),
        .cdc_update_count_sys    (cdc_update_count_sys),
        .build_id                (build_id),
        .schema_word             (schema_word),

        .viz_selftest_en_sys     (viz_selftest_en_sys),
        .vga_page_select_sys     (vga_page_select_sys),
        .history_freeze_sys      (1'b0),

        .pix_clk                 (pix_clk),
        .pix_rst                 (pix_rst),

        .vga_hsync               (vga_hsync),
        .vga_vsync               (vga_vsync),
        .vga_rgb                 (vga_rgb)
    );

    //==========================================================================
    // Clock generation
    //--------------------------------------------------------------------------
    // Different clock periods intentionally exercise the CDC path.
    //==========================================================================
    initial begin
        sys_clk = 1'b0;
        forever #5 sys_clk = ~sys_clk;   // 100 MHz
    end

    initial begin
        pix_clk = 1'b0;
        forever #7 pix_clk = ~pix_clk;   // ~71.4 MHz
    end

    //==========================================================================
    // Simple pass/fail bookkeeping
    //==========================================================================
    integer fail_count;

    task tb_fail;
        input [255:0] msg;
        begin
            fail_count = fail_count + 1;
            $display("TB FAIL: %0s  t=%0t", msg, $time);
            $finish;
        end
    endtask

    task tb_pass;
        input [255:0] msg;
        begin
            $display("TB PASS: %0s  t=%0t", msg, $time);
        end
    endtask

    task check_eq1;
        input got;
        input exp;
        input [255:0] msg;
        begin
            if (got !== exp)
                tb_fail(msg);
            else
                tb_pass(msg);
        end
    endtask

    task check_eq8;
        input [7:0] got;
        input [7:0] exp;
        input [255:0] msg;
        begin
            if (got !== exp) begin
                $display("TB DETAIL: got=%02h exp=%02h", got, exp);
                tb_fail(msg);
            end else
                tb_pass(msg);
        end
    endtask

    task check_eq12;
        input [11:0] got;
        input [11:0] exp;
        input [255:0] msg;
        begin
            if (got !== exp) begin
                $display("TB DETAIL: got=%03h exp=%03h", got, exp);
                tb_fail(msg);
            end else
                tb_pass(msg);
        end
    endtask

    task check_eq16;
        input [15:0] got;
        input [15:0] exp;
        input [255:0] msg;
        begin
            if (got !== exp) begin
                $display("TB DETAIL: got=%04h exp=%04h", got, exp);
                tb_fail(msg);
            end else
                tb_pass(msg);
        end
    endtask

    task check_eq32;
        input [31:0] got;
        input [31:0] exp;
        input [255:0] msg;
        begin
            if (got !== exp) begin
                $display("TB DETAIL: got=%08h exp=%08h", got, exp);
                tb_fail(msg);
            end else
                tb_pass(msg);
        end
    endtask

    //==========================================================================
    // Wait helpers
    //==========================================================================
    task wait_sys_publish;
        integer watchdog;
        begin
            watchdog = 0;
            while (u_dut.viz_publish_pulse_sys !== 1'b1) begin
                @(posedge sys_clk);
                watchdog = watchdog + 1;
                if (watchdog > 100000)
                    tb_fail("timeout waiting for SYS publish pulse");
            end
            tb_pass("observed SYS publish pulse");
        end
    endtask

    task wait_pix_update;
        integer watchdog;
        begin
            watchdog = 0;
            while (u_dut.viz_bundle_update_pix !== 1'b1) begin
                @(posedge pix_clk);
                watchdog = watchdog + 1;
                if (watchdog > 100000)
                    tb_fail("timeout waiting for PIX bundle update pulse");
            end
            tb_pass("observed PIX bundle update pulse");
        end
    endtask

    task wait_visualizer_q1_pixel;
        input [10:0] x_tgt;
        input [10:0] y_tgt;
        input        active_tgt;
        integer      watchdog;
        begin
            watchdog = 0;
            while (!((u_dut.u_flight_visualizer_pix.x_q1 == x_tgt) &&
                     (u_dut.u_flight_visualizer_pix.y_q1 == y_tgt) &&
                     (u_dut.u_flight_visualizer_pix.active_q1 == active_tgt))) begin
                @(posedge pix_clk);
                watchdog = watchdog + 1;
                if (watchdog > 2000000)
                    tb_fail("timeout waiting for wrapped visualizer q1 pixel");
            end
            @(posedge pix_clk);
        end
    endtask

    task wait_committed_visualizer_bundle;
        integer watchdog;
        begin
            watchdog = 0;
            while (u_dut.u_flight_visualizer_pix.viz_q[`VIZ_BMP_SEQ_MSB:`VIZ_BMP_SEQ_LSB] !== 16'h007A) begin
                @(posedge pix_clk);
                watchdog = watchdog + 1;
                if (watchdog > 1000000)
                    tb_fail("timeout waiting for wrapped visualizer committed bundle");
            end

            // The renderer commits semantic inputs only at the frame boundary.
            // Wait a few PIX clocks so downstream text/color combinational
            // paths are sampled after the committed bundle is stable.
            repeat (4) @(posedge pix_clk);
            tb_pass("wrapped visualizer committed nominal bundle");
        end
    endtask

    //==========================================================================
    // Stimulus helpers
    //==========================================================================
    task drive_defaults;
        begin
            bmp_t_us               = 32'd0;
            bmp_seq                = 16'd0;
            bmp_valid              = 1'b0;
            bmp_status             = 8'h00;
            bmp_payload            = 48'd0;
            bmp_age_ms             = 16'd0;

            acc_t_us               = 32'd0;
            acc_seq                = 16'd0;
            acc_valid              = 1'b0;
            acc_status             = 8'h00;
            acc_payload            = 48'd0;
            acc_age_ms             = 16'd0;

            mag_t_us               = 32'd0;
            mag_seq                = 16'd0;
            mag_valid              = 1'b0;
            mag_status             = 8'h00;
            mag_payload            = 48'd0;
            mag_age_ms             = 16'd0;

            pwr_t_us               = 32'd0;
            pwr_seq                = 16'd0;
            pwr_valid              = 1'b0;
            pwr_status             = 8'h00;
            pwr_payload            = 48'd0;
            pwr_age_ms             = 16'd0;

            ext_valid              = 1'b0;
            ext_status             = 8'h01;
            ext_present_flags      = 16'd0;
            ext_fault_flags        = 16'd0;
            ext_mag_delta_l1       = 16'd0;
            ext_mag_norm_primary   = 16'd0;
            ext_mag_norm_secondary = 16'd0;
            ext_mag_sequence_aligned = 1'b0;
            ext_mag_disagreement    = 1'b0;
            ext_mag_sector_delta    = 4'd0;
            ext_mag_source_flags    = 8'd0;
            ext_rng_height_cm      = 16'd0;
            ext_air_dp_pa          = 16'd0;
            ext_air_speed_cms      = 16'd0;
            ext_env_temp_cdeg      = 16'd0;
            ext_env_rh_centi       = 16'd0;
            ext_sun_luma           = 16'd0;
            ext_flow_dx            = 16'd0;
            ext_flow_dy            = 16'd0;
            ext_log_seq            = 16'd0;
            ext_log_drop_count     = 16'd0;
            ext_max_age_ms         = 16'hFFFF;

            der_valid              = 1'b0;
            der_status             = 8'h00;

            der_alt_fresh          = 1'b0;
            der_vspd_fresh         = 1'b0;
            der_roll_fresh         = 1'b0;
            der_head_fresh         = 1'b0;

            der_bmp_seq_ref        = 16'd0;
            der_acc_seq_ref        = 16'd0;
            der_mag_seq_ref        = 16'd0;

            der_bmp_age_ms         = 16'd0;
            der_acc_age_ms         = 16'd0;
            der_mag_age_ms         = 16'd0;

            der_bmp_valid_ref      = 1'b0;
            der_acc_valid_ref      = 1'b0;
            der_mag_valid_ref      = 1'b0;

            der_altitude_cm        = 32'd0;
            der_vertical_speed_cms = 32'd0;
            der_roll_mdeg          = 32'd0;
            der_heading_mdeg       = 32'd0;

            nav_valid              = 1'b0;
            nav_status             = 8'h01;
            nav_flags              = 8'd0;
            nav_downrange_m        = 16'd0;
            nav_crossrange_m       = 16'd0;
            nav_age_ms             = 16'hFFFF;

            wind_valid             = 1'b0;
            wind_status            = 8'h01;
            wind_x_cms             = 16'd0;
            wind_y_cms             = 16'd0;
            wind_z_cms             = 16'd0;
            wind_age_ms            = 16'hFFFF;

            auth_phase_code_sys    = `VIZ_AUTH_PHASE_UNKNOWN;
            auth_phase_valid_sys   = 1'b0;
            safety_runtime_ok_sys  = 1'b0;
            safety_allows_actuation_sys = 1'b0;
            policy_runtime_enable_sys = 1'b0;
            software_armed_sys     = 1'b0;

            i2c_nack_count         = 16'd0;
            i2c_timeout_count      = 16'd0;
            txn_rate_hz            = 16'd0;
            cdc_update_count_sys   = 32'd0;
            build_id               = 32'd0;
            schema_word            = 16'd0;

            viz_selftest_en_sys    = 1'b0;
            vga_page_select_sys    = 2'd0;
        end
    endtask

    task inject_nominal_semantic_update;
        begin
            // Raw summaries
            bmp_seq      = 16'h007A;
            bmp_valid    = 1'b1;
            bmp_status   = 8'h00;
            bmp_age_ms   = 16'd14;

            acc_seq      = 16'h012C;
            acc_valid    = 1'b1;
            acc_status   = 8'h00;
            acc_age_ms   = 16'd4;

            mag_seq      = 16'h0051;
            mag_valid    = 1'b1;
            mag_status   = 8'h00;
            mag_age_ms   = 16'd98;

            pwr_t_us     = 32'd1_000_000;
            pwr_seq      = 16'h0F5A;
            pwr_valid    = 1'b1;
            pwr_status   = 8'h00;
            pwr_payload  = {8'h00, 12'h5A0, 12'h120, 16'h0000};
            pwr_age_ms   = 16'd44;

            ext_valid              = 1'b1;
            ext_status             = 8'h00;
            ext_present_flags      = 16'h00FF;
            ext_fault_flags        = 16'h0000;
            ext_mag_delta_l1       = 16'd18;
            ext_mag_norm_primary   = 16'd960;
            ext_mag_norm_secondary = 16'd972;
            ext_mag_sequence_aligned = 1'b1;
            ext_mag_disagreement    = 1'b0;
            ext_mag_sector_delta    = 4'd1;
            ext_mag_source_flags    = (8'd1 << `EXT_SRC_SYNTHETIC_BIT);
            ext_rng_height_cm      = 16'd185;
            ext_air_dp_pa          = 16'd42;
            ext_air_speed_cms      = 16'd1250;
            ext_env_temp_cdeg      = 16'd2345;
            ext_env_rh_centi       = 16'd4520;
            ext_sun_luma           = 16'd8192;
            ext_flow_dx            = 16'd12;
            ext_flow_dy            = 16'hFFF8;
            ext_log_seq            = 16'h4001;
            ext_log_drop_count     = 16'd0;
            ext_max_age_ms         = 16'd44;

            // Derived state
            der_valid              = 1'b1;
            der_status             = 8'h00;

            der_alt_fresh          = 1'b1;
            der_vspd_fresh         = 1'b1;
            der_roll_fresh         = 1'b1;
            der_head_fresh         = 1'b1;

            der_bmp_seq_ref        = 16'h007A;
            der_acc_seq_ref        = 16'h012C;
            der_mag_seq_ref        = 16'h0051;

            der_bmp_age_ms         = 16'd14;
            der_acc_age_ms         = 16'd4;
            der_mag_age_ms         = 16'd98;

            der_bmp_valid_ref      = 1'b1;
            der_acc_valid_ref      = 1'b1;
            der_mag_valid_ref      = 1'b1;

            der_altitude_cm        = 32'd12345;
            der_vertical_speed_cms = 32'd42;
            der_roll_mdeg          = 32'd12000;
            der_heading_mdeg       = 32'd123000;

            // Navigation/wind state for the landing-dispersion viewport.
            nav_valid              = 1'b1;
            nav_status             = 8'h00;
            nav_flags              = 8'h03;
            nav_downrange_m        = 16'd40;
            nav_crossrange_m       = 16'd8;
            nav_age_ms             = 16'd20;

            wind_valid             = 1'b1;
            wind_status            = 8'h00;
            wind_x_cms             = 16'd160;
            wind_y_cms             = 16'hFFC0;
            wind_z_cms             = 16'd64;
            wind_age_ms            = 16'd18;

            auth_phase_code_sys    = `VIZ_AUTH_PHASE_COAST;
            auth_phase_valid_sys   = 1'b1;
            safety_runtime_ok_sys  = 1'b1;
            safety_allows_actuation_sys = 1'b1;
            policy_runtime_enable_sys = 1'b1;
            software_armed_sys     = 1'b1;

            // Platform fields
            i2c_nack_count         = 16'd0;
            i2c_timeout_count      = 16'd0;
            txn_rate_hz            = 16'd50;
            cdc_update_count_sys   = 32'd12345;
            build_id               = 32'h00003F7C;
            schema_word            = 16'h1A2B;
        end
    endtask

    //==========================================================================
    // Main proof sequence
    //==========================================================================
    initial begin
        fail_count = 0;

        sys_rst = 1'b1;
        pix_rst = 1'b1;

        drive_defaults();

        repeat (8) @(posedge sys_clk);
        repeat (8) @(posedge pix_clk);

        sys_rst = 1'b0;
        pix_rst = 1'b0;

        // Allow both domains to settle after reset.
        repeat (8) @(posedge sys_clk);
        repeat (8) @(posedge pix_clk);

        //--------------------------------------------------------------------------
        // Inject one semantic update in SYS.
        //--------------------------------------------------------------------------
        @(posedge sys_clk);
        inject_nominal_semantic_update();

        //--------------------------------------------------------------------------
        // 1) Prove SYS publication occurred.
        //--------------------------------------------------------------------------
        wait_sys_publish();

        //--------------------------------------------------------------------------
        // 2) Prove CDC delivered the packed image into PIX shadow.
        //--------------------------------------------------------------------------
        wait_pix_update();

        check_eq16(
            u_dut.viz_bundle_pix_shadow[`VIZ_BMP_SEQ_MSB:`VIZ_BMP_SEQ_LSB],
            16'h007A,
            "PIX shadow bundle carries BMP sequence"
        );

        check_eq1(
            u_dut.viz_bundle_pix_shadow[`VIZ_BMP_VALID_BIT],
            1'b1,
            "PIX shadow bundle carries BMP valid"
        );

        check_eq8(
            u_dut.viz_bundle_pix_shadow[`VIZ_BMP_STATUS_MSB:`VIZ_BMP_STATUS_LSB],
            8'h00,
            "PIX shadow bundle carries BMP status"
        );

        check_eq16(
            u_dut.viz_bundle_pix_shadow[`VIZ_ACC_SEQ_MSB:`VIZ_ACC_SEQ_LSB],
            16'h012C,
            "PIX shadow bundle carries ACC sequence"
        );

        check_eq16(
            u_dut.viz_bundle_pix_shadow[`VIZ_MAG_SEQ_MSB:`VIZ_MAG_SEQ_LSB],
            16'h0051,
            "PIX shadow bundle carries MAG sequence"
        );

        check_eq1(
            u_dut.viz_bundle_pix_shadow[`VIZ_PWR_VALID_BIT],
            1'b1,
            "PIX shadow bundle carries PMON1 valid"
        );

        check_eq16(
            u_dut.viz_bundle_pix_shadow[`VIZ_PWR_SEQ_MSB:`VIZ_PWR_SEQ_LSB],
            16'h0F5A,
            "PIX shadow bundle carries PMON1 sequence"
        );

        check_eq16(
            u_dut.viz_bundle_pix_shadow[`VIZ_PWR_VOLT_CODE_MSB:`VIZ_PWR_VOLT_CODE_LSB],
            16'h05A0,
            "PIX shadow bundle carries PMON1 voltage code"
        );

        check_eq16(
            u_dut.viz_bundle_pix_shadow[`VIZ_PWR_CURR_CODE_MSB:`VIZ_PWR_CURR_CODE_LSB],
            16'h0120,
            "PIX shadow bundle carries PMON1 current code"
        );

        check_eq1(
            u_dut.viz_bundle_pix_shadow[`VIZ_EXT_VALID_BIT],
            1'b1,
            "PIX shadow bundle carries extension valid"
        );

        check_eq16(
            u_dut.viz_bundle_pix_shadow[`VIZ_EXT_PRESENT_MSB:`VIZ_EXT_PRESENT_LSB],
            16'h00FF,
            "PIX shadow bundle carries extension present flags"
        );

        check_eq16(
            u_dut.viz_bundle_pix_shadow[`VIZ_EXT_MAG_DELTA_L1_MSB:`VIZ_EXT_MAG_DELTA_L1_LSB],
            16'd18,
            "PIX shadow bundle carries redundant-mag delta evidence"
        );

        check_eq16(
            u_dut.viz_bundle_pix_shadow[`VIZ_EXT_RNG_HEIGHT_CM_MSB:`VIZ_EXT_RNG_HEIGHT_CM_LSB],
            16'd185,
            "PIX shadow bundle carries near-ground range height"
        );

        check_eq16(
            u_dut.viz_bundle_pix_shadow[`VIZ_EXT_AIR_SPEED_CMS_MSB:`VIZ_EXT_AIR_SPEED_CMS_LSB],
            16'd1250,
            "PIX shadow bundle carries pitot airspeed"
        );

        check_eq16(
            u_dut.viz_bundle_pix_shadow[`VIZ_EXT_ENV_RH_CENTI_MSB:`VIZ_EXT_ENV_RH_CENTI_LSB],
            16'd4520,
            "PIX shadow bundle carries environmental humidity"
        );

        check_eq32(
            u_dut.viz_bundle_pix_shadow[`VIZ_DER_ALT_CM_MSB:`VIZ_DER_ALT_CM_LSB],
            32'd12345,
            "PIX shadow bundle carries derived altitude"
        );

        check_eq32(
            u_dut.viz_bundle_pix_shadow[`VIZ_DER_VSPD_CMS_MSB:`VIZ_DER_VSPD_CMS_LSB],
            32'd42,
            "PIX shadow bundle carries derived vertical speed"
        );

        check_eq1(
            u_dut.viz_bundle_pix_shadow[`VIZ_AUTH_VALID_BIT],
            1'b1,
            "PIX shadow bundle carries authority valid"
        );

        check_eq8(
            u_dut.viz_bundle_pix_shadow[`VIZ_AUTH_STATUS_MSB:`VIZ_AUTH_STATUS_LSB],
            8'h00,
            "PIX shadow bundle carries authority OK status"
        );

        check_eq8(
            u_dut.viz_bundle_pix_shadow[`VIZ_AUTH_FLAGS_MSB:`VIZ_AUTH_FLAGS_LSB],
            8'h83,
            "PIX shadow bundle carries authority policy flags"
        );

        check_eq32(
            u_dut.viz_bundle_pix_shadow[`VIZ_AUTH_TARGET_CM_MSB:`VIZ_AUTH_TARGET_CM_LSB],
            32'd304800,
            "PIX shadow bundle carries authority target"
        );

        check_eq32(
            u_dut.viz_bundle_pix_shadow[`VIZ_AUTH_PRED_NO_CM_MSB:`VIZ_AUTH_PRED_NO_CM_LSB],
            32'd12345,
            "PIX shadow bundle carries authority no-brake apogee"
        );

        check_eq32(
            u_dut.viz_bundle_pix_shadow[`VIZ_AUTH_PRED_FULL_CM_MSB:`VIZ_AUTH_PRED_FULL_CM_LSB],
            32'd12345,
            "PIX shadow bundle carries authority full-brake apogee"
        );

        check_eq16(
            u_dut.viz_bundle_pix_shadow[`VIZ_AUTH_UNC_CM_MSB:`VIZ_AUTH_UNC_CM_LSB],
            16'd1780,
            "PIX shadow bundle carries authority uncertainty"
        );

        check_eq8(
            u_dut.viz_bundle_pix_shadow[`VIZ_AUTH_CMD_U8_MSB:`VIZ_AUTH_CMD_U8_LSB],
            8'd0,
            "PIX shadow bundle carries retracted authority command"
        );

        check_eq16(
            {4'd0, u_dut.viz_bundle_pix_shadow[`VIZ_AUTH_SERVO_US_MSB:`VIZ_AUTH_SERVO_US_LSB]},
            16'd1000,
            "PIX shadow bundle carries retracted servo pulse"
        );

        check_eq8(
            {4'd0, u_dut.viz_bundle_pix_shadow[`VIZ_AUTH_PHASE_MSB:`VIZ_AUTH_PHASE_LSB]},
            {4'd0, `VIZ_AUTH_PHASE_COAST},
            "PIX shadow bundle carries explicit coast phase"
        );

        check_eq8(
            {1'b0, u_dut.viz_bundle_pix_shadow[`VIZ_AUTH_GATE_MSB:`VIZ_AUTH_GATE_LSB]},
            8'h2F,
            "PIX shadow bundle carries explicit safety gate flags"
        );

        check_eq1(
            u_dut.viz_bundle_pix_shadow[`VIZ_NAV_VALID_BIT],
            1'b1,
            "PIX shadow bundle carries nav valid"
        );

        check_eq8(
            u_dut.viz_bundle_pix_shadow[`VIZ_NAV_STATUS_MSB:`VIZ_NAV_STATUS_LSB],
            8'h00,
            "PIX shadow bundle carries nav OK status"
        );

        check_eq8(
            u_dut.viz_bundle_pix_shadow[`VIZ_NAV_FLAGS_MSB:`VIZ_NAV_FLAGS_LSB],
            8'h03,
            "PIX shadow bundle carries nav quality flags"
        );

        check_eq16(
            u_dut.viz_bundle_pix_shadow[`VIZ_NAV_DOWNRANGE_M_MSB:`VIZ_NAV_DOWNRANGE_M_LSB],
            16'd40,
            "PIX shadow bundle carries nav downrange"
        );

        check_eq16(
            u_dut.viz_bundle_pix_shadow[`VIZ_NAV_CROSSRANGE_M_MSB:`VIZ_NAV_CROSSRANGE_M_LSB],
            16'd8,
            "PIX shadow bundle carries nav crossrange"
        );

        check_eq16(
            u_dut.viz_bundle_pix_shadow[`VIZ_NAV_AGE_MS_MSB:`VIZ_NAV_AGE_MS_LSB],
            16'd20,
            "PIX shadow bundle carries nav age"
        );

        check_eq1(
            u_dut.viz_bundle_pix_shadow[`VIZ_WIND_VALID_BIT],
            1'b1,
            "PIX shadow bundle carries wind valid"
        );

        check_eq8(
            u_dut.viz_bundle_pix_shadow[`VIZ_WIND_STATUS_MSB:`VIZ_WIND_STATUS_LSB],
            8'h00,
            "PIX shadow bundle carries wind OK status"
        );

        check_eq16(
            u_dut.viz_bundle_pix_shadow[`VIZ_WIND_X_CMS_MSB:`VIZ_WIND_X_CMS_LSB],
            16'd160,
            "PIX shadow bundle carries wind x"
        );

        check_eq16(
            u_dut.viz_bundle_pix_shadow[`VIZ_WIND_Y_CMS_MSB:`VIZ_WIND_Y_CMS_LSB],
            16'hFFC0,
            "PIX shadow bundle carries wind y"
        );

        check_eq16(
            u_dut.viz_bundle_pix_shadow[`VIZ_WIND_Z_CMS_MSB:`VIZ_WIND_Z_CMS_LSB],
            16'd64,
            "PIX shadow bundle carries wind z"
        );

        check_eq16(
            u_dut.viz_bundle_pix_shadow[`VIZ_WIND_AGE_MS_MSB:`VIZ_WIND_AGE_MS_LSB],
            16'd18,
            "PIX shadow bundle carries wind age"
        );

        //--------------------------------------------------------------------------
        // 3) Prove the history rings captured the update.
        //--------------------------------------------------------------------------
        // The history write occurs on the PIX clock edge where viz_bundle_update_pix
        // is observed. Wait two PIX edges to move beyond the write edge and the
        // transient wr_ptr_update pulse.
        //--------------------------------------------------------------------------
        repeat (2) @(posedge pix_clk);

        check_eq16(
            u_dut.gen_history_reg_fallback.alt_hist_mem[0],
            16'd12345,
            "altitude history ring slot 0 stores transferred altitude"
        );

        check_eq16(
            u_dut.gen_history_reg_fallback.vspd_hist_mem[0],
            16'd42,
            "vertical-speed history ring slot 0 stores transferred speed"
        );

        check_eq16(
            {6'd0, u_dut.wr_ptr_pix_r},
            16'd1,
            "history write pointer increments after one PIX update"
        );

        //--------------------------------------------------------------------------
        // 4) Prove end-to-end VGA response.
        //
        // Known glyph target:
        //   compact top BMP lane origin = (20,6)
        //   first character = 'B'
        //   row 0 / col 0 is an inked pixel
        //
        // Nominal BMP health => muted green text => 24'h70D070 => packed 12'h7D7
        //--------------------------------------------------------------------------
        wait_committed_visualizer_bundle();
        wait_visualizer_q1_pixel(11'd20, 11'd6, 1'b1);

        check_eq1(
            u_dut.u_flight_visualizer_pix.telemetry_overlay_on,
            1'b1,
            "wrapped visualizer asserts telemetry overlay on known glyph pixel"
        );

        check_eq12(
            u_dut.u_flight_visualizer_pix.telemetry_overlay_rgb_12,
            12'h7D7,
            "wrapped visualizer packs muted telemetry green to 12'h7D7"
        );

        check_eq12(
            vga_rgb,
            12'h7D7,
            "suite-top VGA output shows end-to-end telemetry response"
        );

        // The landing viewport lives under the same renderer and final compositor.
        // With nav_crossrange_m=8 and nav_downrange_m=40, the viewport maps the
        // navigation point to x=536, y=319. Sampling vga_rgb here verifies that
        // the packed nav/wind contract reaches a scientifically meaningful pixel.
        wait_visualizer_q1_pixel(11'd536, 11'd319, 1'b1);

        check_eq1(
            u_dut.u_flight_visualizer_pix.telemetry_overlay_on,
            1'b0,
            "landing viewport sample is not hidden by telemetry text"
        );

        check_eq12(
            vga_rgb,
            12'h2F6,
            "suite-top VGA output shows landing viewport navigation point"
        );

        $display("TB PASS: flight_viz_suite_top transport-and-render proof completed.");
        $finish;
    end

endmodule

`default_nettype wire
