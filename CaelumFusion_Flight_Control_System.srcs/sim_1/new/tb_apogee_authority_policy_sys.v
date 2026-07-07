`timescale 1ns/1ps
`default_nettype none

`include "telemetry_defs_vh.vh"
`include "flight_viz_bundle_defs.vh"

module tb_apogee_authority_policy_sys;
    reg clk = 1'b0;
    always #5 clk = ~clk;

    reg rst;
    reg der_valid;
    reg [7:0] der_status;
    reg der_alt_fresh;
    reg der_vspd_fresh;
    reg der_bmp_valid_ref;
    reg [15:0] der_bmp_age_ms;
    reg [31:0] altitude_cm;
    reg signed [31:0] vertical_speed_cms;
    reg safety_runtime_ok;
    reg safety_allows_actuation;
    reg policy_runtime_enable;
    reg software_armed;

    wire auth_valid;
    wire [7:0] auth_status;
    wire [7:0] auth_flags;
    wire [31:0] auth_target_cm;
    wire [31:0] auth_pred_no_cm;
    wire [31:0] auth_pred_full_cm;
    wire [15:0] auth_uncertainty_cm;
    wire [7:0] auth_brake_cmd_u8;
    wire [11:0] auth_servo_us;

    apogee_authority_policy_sys #(
        .TARGET_APOGEE_CM(32'd304800),
        .MAX_BMP_AGE_MS(16'd200),
        .SYS_CLK_HZ(100),
        .POLICY_UPDATE_HZ(50),
        .POLICY_MIN_ALT_CM(32'd3000),
        .POLICY_MIN_VSPD_CMS(32'sd1500),
        .POLICY_DEADBAND_CM(32'd500),
        .POLICY_SLEW_U8_PER_SEC(16'd383),
        .POLICY_P00_FALLBACK_CM2(32'd10000),
        .POLICY_SIGMA_MARGIN_Q8(16'd256),
        .UNC_MAX_CM(32'd2000)
    ) dut (
        .sys_clk(clk),
        .sys_rst(rst),
        .der_valid(der_valid),
        .der_status(der_status),
        .der_alt_fresh(der_alt_fresh),
        .der_vspd_fresh(der_vspd_fresh),
        .der_bmp_valid_ref(der_bmp_valid_ref),
        .der_bmp_age_ms(der_bmp_age_ms),
        .altitude_cm(altitude_cm),
        .vertical_speed_cms(vertical_speed_cms),
        .safety_runtime_ok(safety_runtime_ok),
        .safety_allows_actuation(safety_allows_actuation),
        .policy_runtime_enable(policy_runtime_enable),
        .software_armed(software_armed),
        .auth_valid(auth_valid),
        .auth_status(auth_status),
        .auth_flags(auth_flags),
        .auth_target_cm(auth_target_cm),
        .auth_pred_no_cm(auth_pred_no_cm),
        .auth_pred_full_cm(auth_pred_full_cm),
        .auth_uncertainty_cm(auth_uncertainty_cm),
        .auth_brake_cmd_u8(auth_brake_cmd_u8),
        .auth_servo_us(auth_servo_us)
    );

    task settle;
        begin
            repeat (12) @(posedge clk);
        end
    endtask

    task expect_idle;
        input [255:0] name;
        begin
            if (auth_valid !== 1'b0) begin
                $display("FAIL %0s: auth_valid asserted", name);
                $fatal;
            end
            if (auth_brake_cmd_u8 !== 8'd0) begin
                $display("FAIL %0s: command not zero: %0d", name, auth_brake_cmd_u8);
                $fatal;
            end
            if (auth_servo_us !== 12'd1000) begin
                $display("FAIL %0s: servo not idle: %0d", name, auth_servo_us);
                $fatal;
            end
        end
    endtask

    task expect_command;
        input [255:0] name;
        begin
            if (auth_valid !== 1'b1) begin
                $display("FAIL %0s: auth_valid not asserted", name);
                $fatal;
            end
            if (auth_brake_cmd_u8 == 8'd0) begin
                $display("FAIL %0s: command is zero", name);
                $fatal;
            end
            if (auth_servo_us == 12'd1000) begin
                $display("FAIL %0s: servo still idle", name);
                $fatal;
            end
            if (auth_uncertainty_cm !== 16'd100) begin
                $display("FAIL %0s: expected sqrt(P00)=100 cm margin, got %0d", name, auth_uncertainty_cm);
                $fatal;
            end
            if (auth_target_cm !== 32'd304700) begin
                $display("FAIL %0s: expected effective target 304700 cm, got %0d", name, auth_target_cm);
                $fatal;
            end
        end
    endtask

    task set_nominal;
        begin
            der_valid = 1'b1;
            der_status = `ST_OK;
            der_alt_fresh = 1'b1;
            der_vspd_fresh = 1'b1;
            der_bmp_valid_ref = 1'b1;
            der_bmp_age_ms = 16'd0;
            altitude_cm = 32'd300000;
            vertical_speed_cms = 32'sd15000;
            safety_runtime_ok = 1'b1;
            safety_allows_actuation = 1'b1;
            policy_runtime_enable = 1'b1;
            software_armed = 1'b1;
        end
    endtask

    reg [7:0] first_cmd;

    initial begin
        rst = 1'b1;
        der_valid = 1'b0;
        der_status = `ST_OK;
        der_alt_fresh = 1'b0;
        der_vspd_fresh = 1'b0;
        der_bmp_valid_ref = 1'b0;
        der_bmp_age_ms = 16'hFFFF;
        altitude_cm = 32'd0;
        vertical_speed_cms = 32'sd0;
        safety_runtime_ok = 1'b0;
        safety_allows_actuation = 1'b0;
        policy_runtime_enable = 1'b0;
        software_armed = 1'b0;

        repeat (4) @(posedge clk);
        rst = 1'b0;
        settle();
        expect_idle("reset_idle");

        set_nominal();
        policy_runtime_enable = 1'b0;
        settle();
        expect_idle("policy_disabled");

        set_nominal();
        software_armed = 1'b0;
        settle();
        expect_idle("missing_software_arm");

        set_nominal();
        safety_allows_actuation = 1'b0;
        settle();
        expect_idle("phase_or_safety_denied");

        set_nominal();
        altitude_cm = 32'd1000;
        settle();
        expect_idle("below_altitude_gate");

        set_nominal();
        vertical_speed_cms = 32'sd500;
        settle();
        expect_idle("below_vertical_speed_gate");

        set_nominal();
        der_bmp_age_ms = 16'd201;
        settle();
        expect_idle("stale_estimator");

        set_nominal();
        settle();
        expect_command("valid_coast_command");
        first_cmd = auth_brake_cmd_u8;

        // Gate failure must reset private slew memory to zero rather than
        // resuming from a stale pre-denial command.
        software_armed = 1'b0;
        settle();
        expect_idle("gate_failure_resets_slew_memory");

        set_nominal();
        repeat (6) @(posedge clk);
        if (auth_brake_cmd_u8 >= first_cmd) begin
            $display("FAIL slew restart: command did not restart from a lower value, old=%0d new=%0d", first_cmd, auth_brake_cmd_u8);
            $fatal;
        end

        $display("PASS tb_apogee_authority_policy_sys");
        $finish;
    end
endmodule

`default_nettype wire
