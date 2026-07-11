`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// dp_bram_1024xsixteen
//------------------------------------------------------------------------------
// True dual-port RAM, independent clocks.
//  - Port A: SYS write + registered SYS read
//  - Port B: PIX read with TWO-STAGE registered output
//
// Timing contract:
//  - a_dout reflects mem[a_addr] with 1 a_clk latency
//  - b_dout reflects mem[b_addr] with 2 b_clk latency
//
// Intent:
//  - Keep BRAM inference clean
//  - Add one extra PIX-domain pipeline stage to shorten the
//    BRAM-to-renderer critical path
//==============================================================================
module dp_bram_1024xsixteen (
    // Port A (SYS)
    input  wire        a_clk,
    input  wire        a_we,
    input  wire [9:0]  a_addr,
    input  wire [15:0] a_din,
    output reg  [15:0] a_dout,

    // Port B (PIX)
    input  wire        b_clk,
    input  wire [9:0]  b_addr,
    output reg  [15:0] b_dout
);

    (* ram_style = "block" *) reg [15:0] mem [0:1023];

    // Internal Port-B pipeline register
    reg [15:0] b_dout_stage0;

    //--------------------------------------------------------------------------
    // Port A
    //--------------------------------------------------------------------------
    always @(posedge a_clk) begin
        if (a_we) begin
            mem[a_addr] <= a_din;
        end
        a_dout <= mem[a_addr];
    end

    //--------------------------------------------------------------------------
    // Port B
    //--------------------------------------------------------------------------
    always @(posedge b_clk) begin
        b_dout_stage0 <= mem[b_addr];
        b_dout        <= b_dout_stage0;
    end

endmodule

`default_nettype wire
