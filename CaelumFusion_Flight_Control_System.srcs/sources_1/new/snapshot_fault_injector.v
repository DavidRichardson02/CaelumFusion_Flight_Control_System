`timescale 1ns/1ps
`default_nettype none

`include "telemetry_defs_vh.vh"

//==============================================================================
// snapshot_fault_injector
//------------------------------------------------------------------------------
// Deterministic diagnostic overlay for one published snapshot-shaped bank.
//
// This block does not own the physical sensor snapshot. It creates a gated
// diagnostic view for validation, visualization, and logging paths while the
// upstream sensor-suite writer remains single-owner of the real bank.
//==============================================================================
module snapshot_fault_injector #(
    parameter [47:0] INVALID_PAYLOAD      = 48'hBAD0_0000_0000,
    parameter [47:0] OUT_OF_RANGE_PAYLOAD = 48'h7FFF_8000_FFFF
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        enable,
    input  wire [2:0]  fault_class,

    input  wire [31:0] in_t_us,
    input  wire [15:0] in_seq,
    input  wire        in_valid,
    input  wire [7:0]  in_status,
    input  wire [47:0] in_payload,
    input  wire [15:0] in_age_ms,

    output wire [31:0] out_t_us,
    output wire [15:0] out_seq,
    output wire        out_valid,
    output wire [7:0]  out_status,
    output wire [47:0] out_payload,
    output wire [15:0] out_age_ms,
    output wire        injected
);
    reg [15:0] stuck_seq_r;
    reg        stuck_active_r;

    wire stuck_mode_w = enable && (fault_class == `DIAG_FAULT_STUCK_SEQ);

    always @(posedge clk) begin
        if (rst) begin
            stuck_seq_r    <= 16'd0;
            stuck_active_r <= 1'b0;
        end else if (!stuck_mode_w) begin
            stuck_seq_r    <= in_seq;
            stuck_active_r <= 1'b0;
        end else if (!stuck_active_r) begin
            stuck_seq_r    <= in_seq;
            stuck_active_r <= 1'b1;
        end
    end

    assign injected = enable && (fault_class != `DIAG_FAULT_NONE);

    assign out_t_us = in_t_us;
    assign out_seq =
        (enable && (fault_class == `DIAG_FAULT_STUCK_SEQ)) ? stuck_seq_r :
        in_seq;

    assign out_valid =
        (enable && (fault_class == `DIAG_FAULT_STALE)) ? 1'b0 :
        (enable && (fault_class == `DIAG_FAULT_STATUS)) ? 1'b1 :
        (enable && (fault_class == `DIAG_FAULT_INVALID_PAYLOAD)) ? 1'b1 :
        (enable && (fault_class == `DIAG_FAULT_OUT_OF_RANGE)) ? 1'b1 :
        in_valid;

    assign out_status =
        (enable && (fault_class == `DIAG_FAULT_STALE)) ? `ST_STALE_REJECT :
        (enable && (fault_class == `DIAG_FAULT_STATUS)) ? `ST_CONFIG_ERROR :
        (enable && (fault_class == `DIAG_FAULT_INVALID_PAYLOAD)) ? `ST_NUMERIC_FAULT :
        (enable && (fault_class == `DIAG_FAULT_OUT_OF_RANGE)) ? `ST_RANGE_REJECT :
        in_status;

    assign out_payload =
        (enable && (fault_class == `DIAG_FAULT_INVALID_PAYLOAD)) ? INVALID_PAYLOAD :
        (enable && (fault_class == `DIAG_FAULT_OUT_OF_RANGE)) ? OUT_OF_RANGE_PAYLOAD :
        in_payload;

    assign out_age_ms =
        (enable && (fault_class == `DIAG_FAULT_STALE)) ? 16'hFFFF :
        in_age_ms;
endmodule

`default_nettype wire
