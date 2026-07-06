`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// i2c_job_arbiter7
//------------------------------------------------------------------------------
// Deterministic fixed-priority grant for the expanded shared-I2C suite.
//
// Priority preserves existing flight-critical ordering first:
//   LIS3DH > BMP585 > CMPS2/MMC3416 > PMON1 > HYGRO > GYRO > LIS2MDL/MAG1
//
// Only one grant is asserted for one clk cycle. Jobs that need multi-transfer
// progress keep their want_* asserted through their busy output and reacquire
// the shared engine between transactions.
//==============================================================================
module i2c_job_arbiter7 (
    input  wire clk,
    input  wire rst,

    input  wire want_lis3dh,
    input  wire want_bmp585,
    input  wire want_cmps2,
    input  wire want_pmon1,
    input  wire want_hygro,
    input  wire want_gyro,
    input  wire want_lis2mdl_mag1,

    input  wire engine_busy,

    output reg  grant_lis3dh,
    output reg  grant_bmp585,
    output reg  grant_cmps2,
    output reg  grant_pmon1,
    output reg  grant_hygro,
    output reg  grant_gyro,
    output reg  grant_lis2mdl_mag1
);
    always @(posedge clk) begin
        if (rst) begin
            grant_lis3dh      <= 1'b0;
            grant_bmp585      <= 1'b0;
            grant_cmps2       <= 1'b0;
            grant_pmon1       <= 1'b0;
            grant_hygro       <= 1'b0;
            grant_gyro        <= 1'b0;
            grant_lis2mdl_mag1 <= 1'b0;
        end else begin
            grant_lis3dh      <= 1'b0;
            grant_bmp585      <= 1'b0;
            grant_cmps2       <= 1'b0;
            grant_pmon1       <= 1'b0;
            grant_hygro       <= 1'b0;
            grant_gyro        <= 1'b0;
            grant_lis2mdl_mag1 <= 1'b0;

            if (!engine_busy) begin
                if (want_lis3dh)
                    grant_lis3dh <= 1'b1;
                else if (want_bmp585)
                    grant_bmp585 <= 1'b1;
                else if (want_cmps2)
                    grant_cmps2 <= 1'b1;
                else if (want_pmon1)
                    grant_pmon1 <= 1'b1;
                else if (want_hygro)
                    grant_hygro <= 1'b1;
                else if (want_gyro)
                    grant_gyro <= 1'b1;
                else if (want_lis2mdl_mag1)
                    grant_lis2mdl_mag1 <= 1'b1;
            end
        end
    end
endmodule

`default_nettype wire
