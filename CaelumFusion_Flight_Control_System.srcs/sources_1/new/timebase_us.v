`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// timebase_us
//------------------------------------------------------------------------------
// ROLE
//   Generate a 1 microsecond tick and a free-running microsecond timebase from
//   the system clock.
//
// CONTRACT
//   - tick_1us is one clk cycle wide.
//   - time_us increments exactly when tick_1us is asserted.
//   - CLK_HZ must be an integer multiple of 1 MHz for this Stage-A version.
//==============================================================================
module timebase_us #(
    parameter integer CLK_HZ = 100_000_000
)(
    input  wire        clk,
    input  wire        rst,
    output reg  [31:0] time_us,
    output reg         tick_1us
);

    localparam integer CYCLES_PER_US = CLK_HZ / 1_000_000;
    localparam integer CTR_W         = 32;

    reg [CTR_W-1:0] div_ctr;

    always @(posedge clk) begin
        if (rst) begin
            div_ctr  <= {CTR_W{1'b0}};
            time_us  <= 32'd0;
            tick_1us <= 1'b0;
        end else begin
            tick_1us <= 1'b0;

            if (div_ctr == (CYCLES_PER_US - 1)) begin
                div_ctr  <= {CTR_W{1'b0}};
                time_us  <= time_us + 32'd1;
                tick_1us <= 1'b1;
            end else begin
                div_ctr <= div_ctr + {{(CTR_W-1){1'b0}},1'b1};
            end
        end
    end

endmodule

`default_nettype wire