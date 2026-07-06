`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// simple_dp_ram
//------------------------------------------------------------------------------
// Simple dual-port RAM:
//   - Port A: write-only
//   - Port B: read-only
//
// NOTES
//   - Independent clocks are supported.
//   - Read port is synchronous.
//   - This template is inference-friendly for FPGA block/distributed RAM.
//   - No byte enables, no reset of memory contents.
//==============================================================================
module simple_dp_ram #(
    parameter integer AW = 10,
    parameter integer DW = 16
)(
    //--------------------------------------------------------------------------
    // Port A: write
    //--------------------------------------------------------------------------
    input  wire              clka,
    input  wire              wea,
    input  wire [AW-1:0]     addra,
    input  wire [DW-1:0]     dina,

    //--------------------------------------------------------------------------
    // Port B: read
    //--------------------------------------------------------------------------
    input  wire              clkb,
    input  wire [AW-1:0]     addrb,
    output reg  [DW-1:0]     doutb
);

    localparam integer DEPTH = (1 << AW);

    reg [DW-1:0] mem [0:DEPTH-1];

    //--------------------------------------------------------------------------
    // Write port
    //--------------------------------------------------------------------------
    always @(posedge clka) begin
        if (wea)
            mem[addra] <= dina;
    end

    //--------------------------------------------------------------------------
    // Read port
    //--------------------------------------------------------------------------
    always @(posedge clkb) begin
        doutb <= mem[addrb];
    end

endmodule

`default_nettype wire