`timescale 1ns/1ps
`default_nettype none

`include "telemetry_defs_vh.vh"

module tb_kalman_alt2_fixed_sys;
    reg clk = 1'b0;
    always #5 clk = ~clk;

    reg rst;
    reg seed_valid;
    reg signed [31:0] seed_h_cm;
    reg predict_valid;
    reg signed [31:0] predict_a_cms2;
    reg [15:0] predict_dt_ms;
    reg update_valid;
    reg signed [31:0] update_z_cm;

    wire est_valid;
    wire est_seeded;
    wire [7:0] est_status;
    wire signed [31:0] est_h_cm;
    wire signed [31:0] est_v_cms;
    wire signed [31:0] est_a_cms2;
    wire signed [31:0] est_P00_cm2;
    wire signed [31:0] est_P01_cm2s;
    wire signed [31:0] est_P10_cm2s;
    wire signed [31:0] est_P11_cm2s2;

    kalman_alt2_fixed_sys #(
        .INITIAL_P00_CM2(32'sd10000),
        .INITIAL_P11_CM2S2(32'sd10000),
        .MEAS_R_CM2(32'sd571),
        .PROCESS_Q00_MIN_CM2(32'sd1),
        .PROCESS_Q11_MIN_CM2S2(32'sd1),
        .MAX_DT_MS(16'd100)
    ) dut (
        .clk(clk),
        .rst(rst),
        .seed_valid(seed_valid),
        .seed_h_cm(seed_h_cm),
        .predict_valid(predict_valid),
        .predict_a_cms2(predict_a_cms2),
        .predict_dt_ms(predict_dt_ms),
        .update_valid(update_valid),
        .update_z_cm(update_z_cm),
        .est_valid(est_valid),
        .est_seeded(est_seeded),
        .est_status(est_status),
        .est_h_cm(est_h_cm),
        .est_v_cms(est_v_cms),
        .est_a_cms2(est_a_cms2),
        .est_P00_cm2(est_P00_cm2),
        .est_P01_cm2s(est_P01_cm2s),
        .est_P10_cm2s(est_P10_cm2s),
        .est_P11_cm2s2(est_P11_cm2s2)
    );

    task clear_cmds;
        begin
            seed_valid = 1'b0;
            predict_valid = 1'b0;
            update_valid = 1'b0;
            seed_h_cm = 32'sd0;
            predict_a_cms2 = 32'sd0;
            predict_dt_ms = 16'd0;
            update_z_cm = 32'sd0;
        end
    endtask

    initial begin
        rst = 1'b1;
        clear_cmds();
        repeat (4) @(posedge clk);
        rst = 1'b0;
        @(posedge clk);

        if (est_valid !== 1'b0 || est_seeded !== 1'b0) begin
            $display("FAIL reset: valid/seeded asserted");
            $fatal;
        end

        seed_h_cm = 32'sd1000;
        seed_valid = 1'b1;
        @(posedge clk);
        clear_cmds();
        @(posedge clk);

        if (est_valid !== 1'b1 || est_seeded !== 1'b1 || est_h_cm !== 32'sd1000 || est_v_cms !== 32'sd0) begin
            $display("FAIL seed: valid=%0d seeded=%0d h=%0d v=%0d", est_valid, est_seeded, est_h_cm, est_v_cms);
            $fatal;
        end

        predict_a_cms2 = 32'sd981;
        predict_dt_ms = 16'd100;
        predict_valid = 1'b1;
        @(posedge clk);
        clear_cmds();
        @(posedge clk);

        if (est_valid !== 1'b1 || est_status !== `ST_OK) begin
            $display("FAIL predict status: valid=%0d status=%0h", est_valid, est_status);
            $fatal;
        end
        if (est_v_cms <= 32'sd0 || est_h_cm <= 32'sd1000) begin
            $display("FAIL predict motion: h=%0d v=%0d", est_h_cm, est_v_cms);
            $fatal;
        end
        if (est_P00_cm2 <= 32'sd0 || est_P11_cm2s2 <= 32'sd0 || est_P01_cm2s !== est_P10_cm2s) begin
            $display("FAIL predict covariance: P00=%0d P01=%0d P10=%0d P11=%0d", est_P00_cm2, est_P01_cm2s, est_P10_cm2s, est_P11_cm2s2);
            $fatal;
        end

        update_z_cm = 32'sd1000;
        update_valid = 1'b1;
        @(posedge clk);
        clear_cmds();
        @(posedge clk);

        if (est_valid !== 1'b1 || est_status !== `ST_OK) begin
            $display("FAIL update status: valid=%0d status=%0h", est_valid, est_status);
            $fatal;
        end
        if (est_P00_cm2 <= 32'sd0 || est_P11_cm2s2 <= 32'sd0 || est_P01_cm2s !== est_P10_cm2s) begin
            $display("FAIL update covariance: P00=%0d P01=%0d P10=%0d P11=%0d", est_P00_cm2, est_P01_cm2s, est_P10_cm2s, est_P11_cm2s2);
            $fatal;
        end

        predict_dt_ms = 16'd0;
        predict_a_cms2 = 32'sd981;
        predict_valid = 1'b1;
        @(posedge clk);
        clear_cmds();
        @(posedge clk);

        if (est_valid !== 1'b0 || est_status !== `ST_STALE_REJECT) begin
            $display("FAIL bad dt rejection: valid=%0d status=%0h", est_valid, est_status);
            $fatal;
        end

        $display("PASS tb_kalman_alt2_fixed_sys");
        $finish;
    end
endmodule

`default_nettype wire
