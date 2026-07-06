`timescale 1ns/1ps
`default_nettype none

// ============================================================================
// clk_div_4.v
//------------------------------------------------------------------------------
// Role   : Divide clk_in by 4 with clean 50% duty cycle.
// Method : Two cascaded divide-by-2 toggle flops.
// Notes  :
//   - rst_in is synchronous to clk_in (as written).
//   - For best clock routing on Xilinx, feed clk_out into a BUFG.
// ============================================================================
module clk_div_4 (
    input  wire clk_in,
    input  wire rst_in,   // synchronous reset
    output wire clk_out
);

    reg div2;
    reg div4;

    always @(posedge clk_in) begin
        if (rst_in) begin
            div2 <= 1'b0;
            div4 <= 1'b0;
        end else begin
            div2 <= ~div2;        // /2
            if (div2)             // toggle /4 on every other clk_in edge
                div4 <= ~div4;    // /4
        end
    end

    assign clk_out = div4;

endmodule

`default_nettype wire
