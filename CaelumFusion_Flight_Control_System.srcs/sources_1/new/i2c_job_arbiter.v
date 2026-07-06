`timescale 1ns/1ps
`default_nettype none


//==============================================================================
// i2c_job_arbiter
//------------------------------------------------------------------------------
// Deterministic fixed-priority grant.
// Priority: LIS3DH > BMP585 > magnetometer slot > PMON1 power monitor.
// Only one grant asserted at a time.
//==============================================================================
module i2c_job_arbiter (
    input  wire clk,
    input  wire rst,

    input  wire want_lis3dh,
    input  wire want_bmp585,
    input  wire want_lis2mdl,
    input  wire want_pmon1,

    input  wire engine_busy,

    output reg  grant_lis3dh,
    output reg  grant_bmp585,
    output reg  grant_lis2mdl,
    output reg  grant_pmon1
);
    always @(posedge clk) begin
        if (rst) begin
            grant_lis3dh  <= 1'b0;
            grant_bmp585  <= 1'b0;
            grant_lis2mdl <= 1'b0;
            grant_pmon1   <= 1'b0;
        end else begin
            // Hold grants only for one cycle; jobs latch by observing grant while launching cmd.
            grant_lis3dh  <= 1'b0;
            grant_bmp585  <= 1'b0;
            grant_lis2mdl <= 1'b0;
            grant_pmon1   <= 1'b0;

            if (!engine_busy) begin
                if (want_lis3dh) begin
                    grant_lis3dh <= 1'b1;
                end else if (want_bmp585) begin
                    grant_bmp585 <= 1'b1;
                end else if (want_lis2mdl) begin
                    grant_lis2mdl <= 1'b1;
                end else if (want_pmon1) begin
                    grant_pmon1 <= 1'b1;
                end
            end
        end
    end
endmodule

`default_nettype wire
