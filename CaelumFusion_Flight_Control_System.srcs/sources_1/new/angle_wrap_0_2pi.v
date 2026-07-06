`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// angle_wrap_0_2pi
//------------------------------------------------------------------------------
// Wrap signed Q4.12 radians angle in [-pi, +pi) to unsigned [0, 2*pi).
// Q format: Q4.12
//  - PI_Q12    ≈ round(pi * 4096)     = 12868
//  - TWO_PI_Q12≈ round(2*pi * 4096)   = 25736
//==============================================================================
module angle_wrap_0_2pi #(
    parameter integer PI_Q12     = 12868,
    parameter integer TWO_PI_Q12 = 25736
)(
    input  wire signed [15:0] ang_in_q12,
    output reg        [15:0]  ang_out_q12
);
    always @(*) begin
        if (ang_in_q12 < 16'sd0) begin
            ang_out_q12 = $unsigned(ang_in_q12 + $signed(TWO_PI_Q12[15:0]));
        end else begin
            ang_out_q12 = $unsigned(ang_in_q12);
        end
    end
endmodule

`default_nettype wire