`timescale 1ns/1ps
`default_nettype none

`include "telemetry_defs_vh.vh"

module tb_mag1_bench_snapshot_source;
    reg clk = 1'b0;
    always #5 clk = ~clk;

    reg rst;
    reg enable;
    reg [31:0] mag0_t_us;
    reg [15:0] mag0_seq;
    reg        mag0_valid;
    reg [7:0]  mag0_status;
    reg [47:0] mag0_payload;
    reg [15:0] mag0_age_ms;

    wire [31:0] mag1_t_us;
    wire [15:0] mag1_seq;
    wire        mag1_valid;
    wire [7:0]  mag1_status;
    wire [47:0] mag1_payload;
    wire [15:0] mag1_age_ms;
    wire [7:0]  mag1_cal_state;
    wire [7:0]  mag1_source_flags;
    wire [15:0] mag1_bridge_checksum;

    mag1_bench_snapshot_source #(
        .MAG1_BENCH_OFFSET_X(16'd2),
        .MAG1_BENCH_OFFSET_Y(16'hFFFD),
        .MAG1_BENCH_OFFSET_Z(16'd5),
        .MAG_FRESH_MAX_MS(16'd200),
        .MAG1_SEQUENCE_OFFSET(16'd0)
    ) dut (
        .clk(clk),
        .rst(rst),
        .enable(enable),
        .cfg_offset_x_en(1'b1),
        .cfg_offset_y_en(1'b1),
        .cfg_offset_z_en(1'b1),
        .mag0_t_us(mag0_t_us),
        .mag0_seq(mag0_seq),
        .mag0_valid(mag0_valid),
        .mag0_status(mag0_status),
        .mag0_payload(mag0_payload),
        .mag0_age_ms(mag0_age_ms),
        .mag1_t_us(mag1_t_us),
        .mag1_seq(mag1_seq),
        .mag1_valid(mag1_valid),
        .mag1_status(mag1_status),
        .mag1_payload(mag1_payload),
        .mag1_age_ms(mag1_age_ms),
        .mag1_cal_state(mag1_cal_state),
        .mag1_source_flags(mag1_source_flags),
        .mag1_bridge_checksum(mag1_bridge_checksum)
    );

    task fail;
        input [8*96-1:0] msg;
        begin
            $display("FAIL: %0s", msg);
            $finish;
        end
    endtask

    task expect_true;
        input condition;
        input [8*96-1:0] msg;
        begin
            if (!condition)
                fail(msg);
            else
                $display("PASS: %0s", msg);
        end
    endtask

    initial begin
        rst = 1'b1;
        enable = 1'b0;
        mag0_t_us = 32'd1000;
        mag0_seq = 16'h0042;
        mag0_valid = 1'b1;
        mag0_status = `ST_OK;
        mag0_payload = {16'd300, 16'd200, 16'd100};
        mag0_age_ms = 16'd12;

        repeat (4) @(posedge clk);
        rst = 1'b0;
        repeat (2) @(posedge clk);
        #1;
        expect_true(!mag1_valid, "disabled bench source does not publish MAG1 valid");
        expect_true(mag1_status == `ST_NOT_INITIALIZED, "disabled bench source reports not initialized");
        expect_true(mag1_age_ms == 16'hFFFF, "disabled bench source age is stale sentinel");

        enable = 1'b1;
        repeat (2) @(posedge clk);
        #1;
        expect_true(mag1_valid, "enabled bench source publishes MAG1 valid from MAG0");
        expect_true(mag1_status == `ST_OK, "fresh MAG0 drives OK synthetic MAG1");
        expect_true(mag1_seq == mag0_seq, "synthetic MAG1 sequence mirrors MAG0");
        expect_true(mag1_payload == {16'd305, 16'd197, 16'd102}, "synthetic offsets are applied in raw units");
        expect_true(mag1_cal_state == 8'h80, "synthetic MAG1 calibration state is explicitly uncalibrated");
        expect_true(mag1_source_flags[`EXT_SRC_SYNTHETIC_BIT], "synthetic MAG1 source bit is set");
        expect_true(mag1_bridge_checksum != 16'd0, "synthetic MAG1 checksum is populated");

        mag0_age_ms = 16'd250;
        repeat (2) @(posedge clk);
        #1;
        expect_true(mag1_valid, "stale synthetic source remains a snapshot");
        expect_true(mag1_status == `ST_STALE_REJECT, "stale MAG0 propagates stale MAG1 status");

        mag0_valid = 1'b0;
        repeat (2) @(posedge clk);
        #1;
        expect_true(!mag1_valid, "missing MAG0 clears synthetic MAG1 valid");
        expect_true(mag1_status == `ST_MISSING_INPUT, "missing MAG0 reports missing input");
        expect_true(mag1_age_ms == 16'hFFFF, "missing MAG0 reports stale sentinel age");

        $display("PASS: tb_mag1_bench_snapshot_source");
        $finish;
    end
endmodule

`default_nettype wire
