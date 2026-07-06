`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// flight_vga_page_mux_pix
//------------------------------------------------------------------------------
// ROLE
//   PIX-domain final page selector for the flight VGA renderer.
//
// CONTRACT
//   - Inputs are already render-stage aligned.
//   - Active-video gating is owned here so every page shares the same blacking
//     behavior outside the visible region.
//   - Telemetry text overlays only the primary HUD page. Diagnostic pages render
//     their own engineering evidence without text compositor occlusion.
//==============================================================================
module flight_vga_page_mux_pix #(
    parameter [1:0] PAGE_HUD         = 2'd0,
    parameter [1:0] PAGE_SENSOR_DIAG = 2'd1
)(
    input  wire        active_pix,
    input  wire [1:0]  page_select_pix,

    input  wire [11:0] hud_rgb,
    input  wire [11:0] sensor_diag_rgb,

    input  wire        telemetry_overlay_on,
    input  wire [11:0] telemetry_overlay_rgb,

    output wire [11:0] vga_rgb
);

    wire sensor_diag_page = (page_select_pix == PAGE_SENSOR_DIAG);
    wire [11:0] page_rgb = sensor_diag_page ? sensor_diag_rgb : hud_rgb;

    assign vga_rgb =
        (!active_pix) ? 12'h000 :
        (telemetry_overlay_on && !sensor_diag_page) ? telemetry_overlay_rgb :
                                                      page_rgb;

endmodule

`default_nettype wire
