`timescale 1ns/1ps
`default_nettype none

// ============================================================================
// vga_timing_640x480_60.v
//------------------------------------------------------------------------------
// Role   : Generate 640x480@60Hz timing from pixel clock.
//
// Key properties:
//   - All outputs are registered (glitch-free).
//   - Sync/active derived from NEXT counter values (wrap-safe).
//   - Explicit sync start/end parameters (easy audit).
//
// Standard 640x480@60 (nominal):
//   Pixel clock: 25.175 MHz (often approximated as 25.0 MHz in labs)
//   H: visible 640, front porch 16, sync 96, back porch 48 => total 800
//   V: visible 480, front porch 10, sync 2,  back porch 33 => total 525
// Sync polarity: negative (active-low)
// ============================================================================

module vga_timing_640x480_60 #(
    parameter integer H_ACTIVE  = 640,
    parameter integer H_FP      = 16,
    parameter integer H_SYNC    = 96,
    parameter integer H_BP      = 48,
    parameter integer V_ACTIVE  = 480,
    parameter integer V_FP      = 10,
    parameter integer V_SYNC    = 2,
    parameter integer V_BP      = 33,
    parameter integer HSYNC_POL = 0,
    parameter integer VSYNC_POL = 0
)(
    input  wire        clk,
    input  wire        rst,
    output reg         hsync,
    output reg         vsync,
    output reg         active,
    output reg [10:0]  x,
    output reg [10:0]  y,
    output reg         vsync_edge
);

    // Integer totals
    localparam integer H_TOTAL = H_ACTIVE + H_FP + H_SYNC + H_BP;
    localparam integer V_TOTAL = V_ACTIVE + V_FP + V_SYNC + V_BP;

    // 11-bit thresholds (avoid slicing expressions)
    localparam [10:0] H_ACT_END   = H_ACTIVE;
    localparam [10:0] H_SYNC_BEG  = H_ACTIVE + H_FP;
    localparam [10:0] H_SYNC_END  = H_ACTIVE + H_FP + H_SYNC;

    localparam [10:0] V_ACT_END   = V_ACTIVE;
    localparam [10:0] V_SYNC_BEG  = V_ACTIVE + V_FP;
    localparam [10:0] V_SYNC_END  = V_ACTIVE + V_FP + V_SYNC;

    reg [10:0] h_ctr;
    reg [10:0] v_ctr;

    wire h_act = (h_ctr < H_ACT_END);
    wire v_act = (v_ctr < V_ACT_END);

    wire h_sync_region = (h_ctr >= H_SYNC_BEG) && (h_ctr < H_SYNC_END);
    wire v_sync_region = (v_ctr >= V_SYNC_BEG) && (v_ctr < V_SYNC_END);

    reg vsync_prev;

    always @(posedge clk) begin
        if (rst) begin
            h_ctr      <= 11'd0;
            v_ctr      <= 11'd0;

            hsync      <= (HSYNC_POL ? 1'b0 : 1'b1);
            vsync      <= (VSYNC_POL ? 1'b0 : 1'b1);

            active     <= 1'b0;
            x          <= 11'd0;
            y          <= 11'd0;

            vsync_prev <= 1'b0;
            vsync_edge <= 1'b0;
        end else begin
            // advance counters
            if (h_ctr == (H_TOTAL-1)) begin
                h_ctr <= 11'd0;
                if (v_ctr == (V_TOTAL-1))
                    v_ctr <= 11'd0;
                else
                    v_ctr <= v_ctr + 11'd1;
            end else begin
                h_ctr <= h_ctr + 11'd1;
            end

            // compute active and coordinates
            active <= (h_act && v_act);
            x      <= h_ctr;
            y      <= v_ctr;

            // sync pulses
            if (h_sync_region) hsync <= (HSYNC_POL ? 1'b1 : 1'b0);
            else               hsync <= (HSYNC_POL ? 1'b0 : 1'b1);

            if (v_sync_region) vsync <= (VSYNC_POL ? 1'b1 : 1'b0);
            else               vsync <= (VSYNC_POL ? 1'b0 : 1'b1);

            // vsync edge pulse (rising edge of the *output* vsync signal)
            vsync_edge <= 1'b0;
            if (!vsync_prev && vsync)
                vsync_edge <= 1'b1;
            vsync_prev <= vsync;
        end
    end

endmodule

`default_nettype wire
