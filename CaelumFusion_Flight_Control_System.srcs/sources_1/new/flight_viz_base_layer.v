`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// flight_viz_base_layer
//------------------------------------------------------------------------------
// ROLE
//   Deterministic base/background renderer for the flight visualization suite.
//
// PURPOSE
//   Provide a visually stable backdrop behind telemetry text and diagnostic
//   overlays without introducing additional stateful behavior.
//
// DESIGN PHILOSOPHY
//   This layer is intentionally simple:
//
//     1) It must never become the source of semantic ambiguity.
//     2) It must make pane extents and raster health easy to inspect.
//     3) It should consume relatively little logic.
//     4) It should remain fully deterministic for every pixel.
//
// OUTPUT CONTRACT
//   - base_on  : asserted when this layer is defining the pixel color
//   - base_rgb : RGB444 pixel color for the base layer
//
// COLOR MODEL
//   RGB444 is assumed:
//     [11:8] red
//     [7:4]  green
//     [3:0]  blue
//
// LAYERING INTENT
//   The parent top-level is expected to compose pixels in priority order such
//   as:
//
//       overlay > base > black
//
//   Therefore this layer does not need transparency modeling beyond base_on.
//
// GEOMETRY
//   The current geometry is chosen to resemble the observed visualization
//   layout:
//
//     - top banner strip
//     - left dark instrument area
//     - large center/right light panel
//     - bottom status region with guide lines
//
//   Exact dimensions can be tuned later without changing the architectural role
//   of this module.
//==============================================================================
module flight_viz_base_layer (
    input  wire        clk_pix,
    input  wire        rst_pix,
    input  wire [9:0]  hcount,
    input  wire [9:0]  vcount,
    input  wire        active_video,
    output reg         base_on,
    output reg  [11:0] base_rgb
);

    //==========================================================================
    // Region constants
    //--------------------------------------------------------------------------
    // These constants define a conservative static layout.
    //
    // Top strip:
    //   narrow banner area for headline telemetry
    //
    // Left pane:
    //   darker instrument region
    //
    // Center/right pane:
    //   brighter panel region for dense text
    //
    // Bottom strip:
    //   reserved for debug/status lines
    //==========================================================================
    localparam integer TOP_Y0        = 0;
    localparam integer TOP_Y1        = 31;

    localparam integer LEFT_X0       = 18;
    localparam integer LEFT_X1       = 223;
    localparam integer LEFT_Y0       = 48;
    localparam integer LEFT_Y1       = 393;

    localparam integer RIGHT_X0      = 256;
    localparam integer RIGHT_X1      = 607;
    localparam integer RIGHT_Y0      = 48;
    localparam integer RIGHT_Y1      = 393;

    localparam integer BOT_LINE0_Y   = 404;
    localparam integer BOT_LINE1_Y   = 443;

    //--------------------------------------------------------------------------
    // Interior guide lines inside left pane
    //--------------------------------------------------------------------------
    localparam integer LEFT_GUIDE0_X0 = 60;
    localparam integer LEFT_GUIDE0_X1 = 170;
    localparam integer LEFT_GUIDE0_Y  = 206;

    localparam integer LEFT_GUIDE1_X0 = 44;
    localparam integer LEFT_GUIDE1_X1 = 112;
    localparam integer LEFT_GUIDE1_Y  = 330;

    //==========================================================================
    // Region predicates
    //==========================================================================
    wire in_top_strip;
    wire in_left_pane;
    wire in_right_pane;
    wire on_bottom_line0;
    wire on_bottom_line1;
    wire on_left_guide0;
    wire on_left_guide1;
    wire on_grid;
    wire on_vertical_divider;

    assign in_top_strip =
        (vcount >= TOP_Y0) && (vcount <= TOP_Y1);

    assign in_left_pane =
        (hcount >= LEFT_X0) && (hcount <= LEFT_X1) &&
        (vcount >= LEFT_Y0) && (vcount <= LEFT_Y1);

    assign in_right_pane =
        (hcount >= RIGHT_X0) && (hcount <= RIGHT_X1) &&
        (vcount >= RIGHT_Y0) && (vcount <= RIGHT_Y1);

    assign on_bottom_line0 =
        (vcount == BOT_LINE0_Y);

    assign on_bottom_line1 =
        (vcount == BOT_LINE1_Y);

    assign on_left_guide0 =
        (vcount == LEFT_GUIDE0_Y) &&
        (hcount >= LEFT_GUIDE0_X0) &&
        (hcount <= LEFT_GUIDE0_X1);

    assign on_left_guide1 =
        (vcount == LEFT_GUIDE1_Y) &&
        (hcount >= LEFT_GUIDE1_X0) &&
        (hcount <= LEFT_GUIDE1_X1);

    assign on_vertical_divider =
        (hcount == 239) && (vcount >= 48) && (vcount <= 393);

    //--------------------------------------------------------------------------
    // Faint grid
    //
    // PURPOSE
    //   Useful for visually detecting raster instability, scaling artifacts, and
    //   panel alignment issues.
    //
    // IMPLEMENTATION
    //   Very sparse lines using low-cost modulus by powers of two where
    //   possible.
    //--------------------------------------------------------------------------
    assign on_grid =
        (((hcount[5:0] == 6'd0) || (vcount[5:0] == 6'd0)) &&
         (vcount >= 32));

    //==========================================================================
    // Base layer combinational rendering
    //--------------------------------------------------------------------------
    // Priority order:
    //   1) inactive video -> base off / black
    //   2) top strip
    //   3) right pane
    //   4) left pane
    //   5) guide lines / dividers / bottom rules
    //   6) faint grid
    //   7) default dark field
    //
    // COLORS
    //   Chosen to resemble a subdued engineering HUD:
    //     - background         : very dark blue-gray
    //     - top strip          : light gray-blue
    //     - right pane         : medium-light gray-blue
    //     - left pane          : dark charcoal-blue
    //     - guides / rules     : brighter desaturated cyan-white
    //     - grid               : very faint blue-gray
    //==========================================================================
    always @* begin
        base_on  = 1'b0;
        base_rgb = 12'h000;

        if (active_video) begin
            base_on  = 1'b1;
            base_rgb = 12'h112;

            if (in_top_strip) begin
                base_rgb = 12'hCDF;
            end

            if (in_left_pane) begin
                base_rgb = 12'h112;
            end

            if (in_right_pane) begin
                base_rgb = 12'h9BD;
            end

            if (on_grid) begin
                base_rgb = 12'h223;
            end

            if (on_vertical_divider) begin
                base_rgb = 12'h345;
            end

            if (on_bottom_line0 || on_bottom_line1) begin
                base_rgb = 12'h7AC;
            end

            if (on_left_guide0 || on_left_guide1) begin
                base_rgb = 12'h8CE;
            end
        end
    end

endmodule

`default_nettype wire