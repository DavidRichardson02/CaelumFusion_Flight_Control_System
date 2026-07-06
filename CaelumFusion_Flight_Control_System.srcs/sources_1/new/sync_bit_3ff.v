`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// sync_bit_3ff
//------------------------------------------------------------------------------
// Synchronize one asynchronous single-bit input into clk domain and derive
// one-cycle edge pulses from the synchronized level.
//
// This module is for single-bit status, interrupt, and event inputs only. Do not
// use it to transfer a multi-bit bus; use a handshake, FIFO, or snapshot CDC for
// those cases.
//==============================================================================
module sync_bit_3ff (
    input  wire clk,
    input  wire rst,
    input  wire async_in,

    output wire sync_level,
    output wire rise_pulse,
    output wire fall_pulse,
    output wire toggle_pulse
);
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg [2:0] sync_ff;
    reg sync_level_q;

    always @(posedge clk) begin
        if (rst) begin
            sync_ff      <= 3'b000;
            sync_level_q <= 1'b0;
        end else begin
            sync_ff[0]   <= async_in;
            sync_ff[1]   <= sync_ff[0];
            sync_ff[2]   <= sync_ff[1];
            sync_level_q <= sync_ff[2];
        end
    end

    assign sync_level  = sync_ff[2];
    assign rise_pulse  =  sync_ff[2] & ~sync_level_q;
    assign fall_pulse  = ~sync_ff[2] &  sync_level_q;
    assign toggle_pulse = sync_ff[2] ^  sync_level_q;
endmodule

`default_nettype wire
