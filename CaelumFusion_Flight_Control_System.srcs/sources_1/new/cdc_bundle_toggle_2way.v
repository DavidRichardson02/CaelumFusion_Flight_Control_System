`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// cdc_bundle_toggle_2way
//------------------------------------------------------------------------------
// Toggle-based CDC for a coherent bundle transfer.
//
// CURRENT USE MODEL
//   - Source provides src_bundle and src_key.
//   - Bundle transfer occurs whenever src_key changes.
//   - Destination receives the coherent dst_bundle and a one-cycle dst_update.
//
// SOURCE SIDE
//   - src_bundle is copied into a stable holding register each time src_key
//     changes state.
//   - src_key acts as the publication event marker.
//
// DESTINATION SIDE
//   - src_key is synchronized into dst_clk.
//   - On each detected toggle edge, the held source bundle is captured into the
//     destination register and dst_update is pulsed for one dst_clk cycle.
//
// CONTRACT
//   - src_bundle must be fully updated before src_key toggles.
//   - Source-held bundle remains stable until the next src_key event.
//   - Event rate must be slow enough that destination sees every toggle.
//
// NOTES
//   - The module name is preserved for compatibility with existing code.
//   - No reverse acknowledgement path is currently implemented because the
//     present visualization integration does not consume one.
//==============================================================================
module cdc_bundle_toggle_2way #(
    parameter integer W = 240
)(
    //--------------------------------------------------------------------------
    // Source domain
    //--------------------------------------------------------------------------
    input  wire             src_clk,
    input  wire             src_rst,
    input  wire [W-1:0]     src_bundle,
    input  wire [31:0]      src_key,

    //--------------------------------------------------------------------------
    // Destination domain
    //--------------------------------------------------------------------------
    input  wire             dst_clk,
    input  wire             dst_rst,
    output reg  [W-1:0]     dst_bundle,
    output reg              dst_update
);

    //--------------------------------------------------------------------------
    // Source-side held snapshot
    //--------------------------------------------------------------------------
    reg [W-1:0]  src_bundle_hold;
    reg [31:0]   src_key_d;
    reg          src_evt_toggle;

    always @(posedge src_clk) begin
        if (src_rst) begin
            src_bundle_hold <= {W{1'b0}};
            src_key_d       <= 32'd0;
            src_evt_toggle  <= 1'b0;
        end else begin
            src_key_d <= src_key;

            if (src_key != src_key_d) begin
                src_bundle_hold <= src_bundle;
                src_evt_toggle  <= ~src_evt_toggle;
            end
        end
    end

    //--------------------------------------------------------------------------
    // Destination-side event synchronizer
    //--------------------------------------------------------------------------
    reg dst_sync_ff1;
    reg dst_sync_ff2;
    reg dst_sync_ff3;

    always @(posedge dst_clk) begin
        if (dst_rst) begin
            dst_sync_ff1 <= 1'b0;
            dst_sync_ff2 <= 1'b0;
            dst_sync_ff3 <= 1'b0;
        end else begin
            dst_sync_ff1 <= src_evt_toggle;
            dst_sync_ff2 <= dst_sync_ff1;
            dst_sync_ff3 <= dst_sync_ff2;
        end
    end

    wire dst_evt_edge = dst_sync_ff2 ^ dst_sync_ff3;

    //--------------------------------------------------------------------------
    // Destination capture
    //--------------------------------------------------------------------------
    always @(posedge dst_clk) begin
        if (dst_rst) begin
            dst_bundle <= {W{1'b0}};
            dst_update <= 1'b0;
        end else begin
            dst_update <= 1'b0;

            if (dst_evt_edge) begin
                dst_bundle <= src_bundle_hold;
                dst_update <= 1'b1;
            end
        end
    end

endmodule

`default_nettype wire