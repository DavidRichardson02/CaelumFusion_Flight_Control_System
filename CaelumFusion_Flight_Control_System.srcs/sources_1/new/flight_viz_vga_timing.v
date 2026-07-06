`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// flight_viz_vga_timing
//------------------------------------------------------------------------------
// ROLE
//   Parameterized VGA timing generator for the flight visualization pipeline.
//
// FUNCTION
//   Generates:
//     - hcount       : horizontal pixel position across the full line period
//     - vcount       : vertical line position across the full frame period
//     - active_video : asserted only during the visible image rectangle
//     - vga_hsync    : horizontal sync with selectable polarity
//     - vga_vsync    : vertical sync with selectable polarity
//     - frame_tick   : one-cycle pulse at the first pixel of each new frame
//
// TIMING MODEL
//   A raster frame is divided into:
//
//     Horizontal:
//       active region
//       front porch
//       sync pulse
//       back porch
//
//     Vertical:
//       active region
//       front porch
//       sync pulse
//       back porch
//
// COUNTER CONVENTION
//   - hcount runs from 0 to H_TOTAL-1
//   - vcount runs from 0 to V_TOTAL-1
//   - (hcount,vcount) = (0,0) denotes the first pixel time of a frame
//
// FRAME TICK CONVENTION
//   frame_tick is asserted for one clk_pix cycle exactly when the counters
//   advance into the first pixel of the new frame:
//
//       hcount == 0 and vcount == 0
//
//   after rollover from the previous frame.
//
// DETERMINISM POLICY
//   - single clocked counter pair
//   - no inferred latches
//   - no dependence on external enables
//   - sync and active-video derived purely from registered counters
//
// RESET POLICY
//   - synchronous to clk_pix
//   - reset places counters at origin and drives outputs to the corresponding
//     first-frame values
//==============================================================================
module flight_viz_vga_timing #(
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
    input  wire       clk_pix,
    input  wire       rst_pix,

    output reg        active_video,
    output reg [9:0]  hcount,
    output reg [9:0]  vcount,
    output reg        frame_tick,
    output reg        vga_hsync,
    output reg        vga_vsync
);

    //==========================================================================
    // Derived totals
    //--------------------------------------------------------------------------
    // H_TOTAL and V_TOTAL define the complete raster period including visible
    // area and blanking intervals.
    //==========================================================================
    localparam integer H_TOTAL = H_ACTIVE + H_FP + H_SYNC + H_BP;
    localparam integer V_TOTAL = V_ACTIVE + V_FP + V_SYNC + V_BP;

    //--------------------------------------------------------------------------
    // Sync interval boundaries
    //--------------------------------------------------------------------------
    // Horizontal sync is asserted during:
    //   [H_ACTIVE + H_FP, H_ACTIVE + H_FP + H_SYNC)
    //
    // Vertical sync is asserted during:
    //   [V_ACTIVE + V_FP, V_ACTIVE + V_FP + V_SYNC)
    //--------------------------------------------------------------------------
    localparam integer H_SYNC_START = H_ACTIVE + H_FP;
    localparam integer H_SYNC_END   = H_ACTIVE + H_FP + H_SYNC;

    localparam integer V_SYNC_START = V_ACTIVE + V_FP;
    localparam integer V_SYNC_END   = V_ACTIVE + V_FP + V_SYNC;

    //==========================================================================
    // Internal combinational timing qualifiers
    //--------------------------------------------------------------------------
    // These wires describe the logical timing state associated with the current
    // registered counters. Final output polarity is applied afterward.
    //==========================================================================
    wire h_last;
    wire v_last;

    wire hsync_active_i;
    wire vsync_active_i;
    wire active_video_i;

    assign h_last = (hcount == (H_TOTAL - 1));
    assign v_last = (vcount == (V_TOTAL - 1));

    assign hsync_active_i =
        (hcount >= H_SYNC_START) && (hcount < H_SYNC_END);

    assign vsync_active_i =
        (vcount >= V_SYNC_START) && (vcount < V_SYNC_END);

    assign active_video_i =
        (hcount < H_ACTIVE) && (vcount < V_ACTIVE);

    //==========================================================================
    // Counter progression
    //--------------------------------------------------------------------------
    // Raster advances left-to-right across a line, then top-to-bottom across
    // lines. At the last pixel of the last line, both counters roll to zero.
    //
    // frame_tick assertion rule:
    //   Assert for one cycle exactly when the counters roll into (0,0).
    //==========================================================================
    always @(posedge clk_pix) begin
        if (rst_pix) begin
            hcount     <= 10'd0;
            vcount     <= 10'd0;
            frame_tick <= 1'b0;
        end else begin
            frame_tick <= 1'b0;

            if (h_last) begin
                hcount <= 10'd0;

                if (v_last) begin
                    vcount     <= 10'd0;
                    frame_tick <= 1'b1;
                end else begin
                    vcount <= vcount + 10'd1;
                end
            end else begin
                hcount <= hcount + 10'd1;
            end
        end
    end

    //==========================================================================
    // Output generation
    //--------------------------------------------------------------------------
    // Outputs are derived from the current registered counters.
    //
    // Polarity convention:
    //   POL = 1  -> active-high sync pulse
    //   POL = 0  -> active-low  sync pulse
    //==========================================================================
    always @* begin
        active_video = active_video_i;

        if (HSYNC_POL != 0)
            vga_hsync = hsync_active_i;
        else
            vga_hsync = ~hsync_active_i;

        if (VSYNC_POL != 0)
            vga_vsync = vsync_active_i;
        else
            vga_vsync = ~vsync_active_i;
    end

endmodule

`default_nettype wire