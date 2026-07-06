
`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// snapshot_regs
//------------------------------------------------------------------------------
// Atomic commit register bank: t_us, seq, valid, status, payload.
//==============================================================================
module snapshot_regs #(
    parameter integer PAYLOAD_W = 48
)(
    input  wire                 clk,
    input  wire                 rst,
    input  wire [31:0]          time_us,

    input  wire                 commit,
    input  wire                 valid_in,
    input  wire [7:0]           status_in,
    input  wire [PAYLOAD_W-1:0] payload_in,

    output reg  [31:0]          snap_t_us,
    output reg  [15:0]          snap_seq,
    output reg                  snap_valid,
    output reg  [7:0]           snap_status,
    output reg  [PAYLOAD_W-1:0] snap_payload
);
    always @(posedge clk) begin
        if (rst) begin
            snap_t_us     <= 32'd0;
            snap_seq      <= 16'd0;
            snap_valid    <= 1'b0;
            snap_status   <= 8'h01;
            snap_payload  <= {PAYLOAD_W{1'b0}};
        end else begin
            if (commit) begin
                snap_t_us    <= time_us;
                snap_seq     <= snap_seq + 16'd1;
                snap_valid   <= valid_in;
                snap_status  <= status_in;
                snap_payload <= payload_in;
            end
        end
    end
endmodule

`default_nettype wire
