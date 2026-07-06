`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// spi_job_arbiter
//------------------------------------------------------------------------------
// Fixed-priority scheduler for the shared SPI engine.
//
// Priority: LIS3DH > BMP5xx > LIS2MDL.
//------------------------------------------------------------------------------
module spi_job_arbiter (
    input  wire clk,
    input  wire rst,

    input  wire want_lis3dh,
    input  wire want_bmp5xx,
    input  wire want_lis2mdl,
    input  wire engine_busy,

    output reg  grant_lis3dh,
    output reg  grant_bmp5xx,
    output reg  grant_lis2mdl
);

    always @(posedge clk) begin
        if (rst) begin
            grant_lis3dh  <= 1'b0;
            grant_bmp5xx  <= 1'b0;
            grant_lis2mdl <= 1'b0;
        end else begin
            grant_lis3dh  <= 1'b0;
            grant_bmp5xx  <= 1'b0;
            grant_lis2mdl <= 1'b0;

            if (!engine_busy) begin
                if (want_lis3dh) begin
                    grant_lis3dh <= 1'b1;
                end else if (want_bmp5xx) begin
                    grant_bmp5xx <= 1'b1;
                end else if (want_lis2mdl) begin
                    grant_lis2mdl <= 1'b1;
                end
            end
        end
    end

endmodule

`default_nettype wire
