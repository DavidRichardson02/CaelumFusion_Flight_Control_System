`timescale 1ns/1ps
`default_nettype none

`include "telemetry_defs_vh.vh"

module tb_snapshot_fault_injector;
    reg clk = 1'b0;
    always #5 clk = ~clk;

    reg rst;
    reg enable;
    reg [2:0] fault_class;
    reg [31:0] in_t_us;
    reg [15:0] in_seq;
    reg        in_valid;
    reg [7:0]  in_status;
    reg [47:0] in_payload;
    reg [15:0] in_age_ms;

    wire [31:0] out_t_us;
    wire [15:0] out_seq;
    wire        out_valid;
    wire [7:0]  out_status;
    wire [47:0] out_payload;
    wire [15:0] out_age_ms;
    wire        injected;

    snapshot_fault_injector #(
        .INVALID_PAYLOAD(48'hBAD0_1234_5678),
        .OUT_OF_RANGE_PAYLOAD(48'h7FFF_8000_FFFF)
    ) dut (
        .clk(clk),
        .rst(rst),
        .enable(enable),
        .fault_class(fault_class),
        .in_t_us(in_t_us),
        .in_seq(in_seq),
        .in_valid(in_valid),
        .in_status(in_status),
        .in_payload(in_payload),
        .in_age_ms(in_age_ms),
        .out_t_us(out_t_us),
        .out_seq(out_seq),
        .out_valid(out_valid),
        .out_status(out_status),
        .out_payload(out_payload),
        .out_age_ms(out_age_ms),
        .injected(injected)
    );

    task fail;
        input [8*128-1:0] msg;
        begin
            $display("FAIL: %0s", msg);
            $finish;
        end
    endtask

    task expect;
        input condition;
        input [8*128-1:0] msg;
        begin
            if (!condition)
                fail(msg);
        end
    endtask

    task tick;
        begin
            @(posedge clk);
            #1;
        end
    endtask

    initial begin
        rst = 1'b1;
        enable = 1'b0;
        fault_class = `DIAG_FAULT_NONE;
        in_t_us = 32'd100;
        in_seq = 16'h0100;
        in_valid = 1'b1;
        in_status = `ST_OK;
        in_payload = 48'h0001_0002_0003;
        in_age_ms = 16'd12;
        repeat (3) tick();

        rst = 1'b0;
        tick();
        expect(!injected, "disabled pass-through is not injected");
        expect(out_seq == in_seq, "disabled pass-through preserves sequence");
        expect(out_valid == in_valid, "disabled pass-through preserves valid");
        expect(out_status == in_status, "disabled pass-through preserves status");
        expect(out_payload == in_payload, "disabled pass-through preserves payload");
        expect(out_age_ms == in_age_ms, "disabled pass-through preserves age");

        enable = 1'b1;
        fault_class = `DIAG_FAULT_STALE;
        tick();
        expect(injected, "stale mode is injected");
        expect(!out_valid, "stale mode clears valid");
        expect(out_status == `ST_STALE_REJECT, "stale mode reports stale status");
        expect(out_age_ms == 16'hFFFF, "stale mode saturates age");

        fault_class = `DIAG_FAULT_STATUS;
        tick();
        expect(out_valid, "status mode keeps a visible snapshot");
        expect(out_status == `ST_CONFIG_ERROR, "status mode reports config error");
        expect(out_payload == in_payload, "status mode preserves payload");

        enable = 1'b0;
        fault_class = `DIAG_FAULT_NONE;
        in_seq = 16'h0200;
        tick();
        enable = 1'b1;
        fault_class = `DIAG_FAULT_STUCK_SEQ;
        tick();
        expect(out_seq == 16'h0200, "stuck sequence captures initial sequence");
        in_seq = 16'h0201;
        tick();
        expect(out_seq == 16'h0200, "stuck sequence holds captured sequence");
        enable = 1'b0;
        tick();
        expect(out_seq == in_seq, "disabling stuck sequence releases sequence");

        enable = 1'b1;
        fault_class = `DIAG_FAULT_INVALID_PAYLOAD;
        tick();
        expect(out_valid, "invalid-payload mode keeps visible snapshot");
        expect(out_status == `ST_NUMERIC_FAULT, "invalid-payload mode reports numeric fault");
        expect(out_payload == 48'hBAD0_1234_5678, "invalid-payload mode uses deterministic payload");

        fault_class = `DIAG_FAULT_OUT_OF_RANGE;
        tick();
        expect(out_valid, "out-of-range mode keeps visible snapshot");
        expect(out_status == `ST_RANGE_REJECT, "out-of-range mode reports range reject");
        expect(out_payload == 48'h7FFF_8000_FFFF, "out-of-range mode uses deterministic payload");

        $display("PASS: tb_snapshot_fault_injector");
        $finish;
    end
endmodule

`default_nettype wire
