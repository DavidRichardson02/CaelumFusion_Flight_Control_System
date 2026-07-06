`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// epoch_scheduler
//------------------------------------------------------------------------------
// Converts tick_1us into deterministic epoch pulses.
// Pulses are one clk cycle wide.
//==============================================================================
module epoch_scheduler #(
    parameter integer RATE_100HZ_US = 10_000,
    parameter integer RATE_50HZ_US  = 20_000,
    parameter integer RATE_10HZ_US  = 100_000
)(
    input  wire clk,
    input  wire rst,
    input  wire tick_1us,
    output wire epoch_100hz,
    output wire epoch_50hz,
    output wire epoch_10hz
);

    reg [31:0] ctr_100;
    reg [31:0] ctr_50;
    reg [31:0] ctr_10;

    reg epoch_100hz_r;
    reg epoch_50hz_r;
    reg epoch_10hz_r;

    assign epoch_100hz = epoch_100hz_r;
    assign epoch_50hz  = epoch_50hz_r;
    assign epoch_10hz  = epoch_10hz_r;

    always @(posedge clk) begin
        if (rst) begin
            ctr_100      <= 32'd0;
            ctr_50       <= 32'd0;
            ctr_10       <= 32'd0;
            epoch_100hz_r <= 1'b0;
            epoch_50hz_r  <= 1'b0;
            epoch_10hz_r  <= 1'b0;
        end else begin
            epoch_100hz_r <= 1'b0;
            epoch_50hz_r  <= 1'b0;
            epoch_10hz_r  <= 1'b0;

            if (tick_1us) begin
                if (ctr_100 == (RATE_100HZ_US - 1)) begin
                    ctr_100       <= 32'd0;
                    epoch_100hz_r <= 1'b1;
                end else begin
                    ctr_100 <= ctr_100 + 32'd1;
                end

                if (ctr_50 == (RATE_50HZ_US - 1)) begin
                    ctr_50      <= 32'd0;
                    epoch_50hz_r <= 1'b1;
                end else begin
                    ctr_50 <= ctr_50 + 32'd1;
                end

                if (ctr_10 == (RATE_10HZ_US - 1)) begin
                    ctr_10      <= 32'd0;
                    epoch_10hz_r <= 1'b1;
                end else begin
                    ctr_10 <= ctr_10 + 32'd1;
                end
            end
        end
    end

endmodule

`default_nettype wire