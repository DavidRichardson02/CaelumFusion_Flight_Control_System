`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// cdc_word_toggle
//------------------------------------------------------------------------------
// Toggle-based CDC for a coherent multi-bit word.
//
// SOURCE SIDE
//   - src_word is copied into a stable holding register each time src_toggle
//     changes state.
//   - src_toggle is treated as the publication event marker.
//
// DESTINATION SIDE
//   - src_toggle is synchronized into dst_clk.
//   - On each detected toggle edge, the held source word is captured into the
//     destination register and dst_pulse is asserted for one dst_clk cycle.
//
// CONTRACT
//   - src_word must be stable before/when src_toggle changes.
//   - src_word_hold remains stable until the next src_toggle event.
//   - Destination observes one coherent snapshot per toggle event.
//
// NOTES
//   - This is the classic “held word + event toggle” snapshot-transfer pattern.
//   - No backpressure / acknowledgement is implemented.
//   - Source event rate must be low enough that destination can observe each
//     toggle transition.
//==============================================================================
module cdc_word_toggle #(
    parameter integer W = 10
)(
    //--------------------------------------------------------------------------
    // Source domain
    //--------------------------------------------------------------------------
    input  wire             src_clk,
    input  wire             src_rst,
    input  wire [W-1:0]     src_word,
    input  wire             src_toggle,

    //--------------------------------------------------------------------------
    // Destination domain
    //--------------------------------------------------------------------------
    input  wire             dst_clk,
    input  wire             dst_rst,
    output reg  [W-1:0]     dst_word,
    output reg              dst_pulse
);

    //--------------------------------------------------------------------------
    // Source-side held snapshot
    //--------------------------------------------------------------------------
    reg [W-1:0] src_word_hold;
    reg         src_toggle_d;

    always @(posedge src_clk) begin
        if (src_rst) begin
            src_word_hold <= {W{1'b0}};
            src_toggle_d  <= 1'b0;
        end else begin
            src_toggle_d <= src_toggle;

            if (src_toggle ^ src_toggle_d)
                src_word_hold <= src_word;
        end
    end

    //--------------------------------------------------------------------------
    // Destination-side toggle synchronizer
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
            dst_sync_ff1 <= src_toggle;
            dst_sync_ff2 <= dst_sync_ff1;
            dst_sync_ff3 <= dst_sync_ff2;
        end
    end

    wire dst_toggle_edge = dst_sync_ff2 ^ dst_sync_ff3;

    //--------------------------------------------------------------------------
    // Destination capture
    //--------------------------------------------------------------------------
    always @(posedge dst_clk) begin
        if (dst_rst) begin
            dst_word  <= {W{1'b0}};
            dst_pulse <= 1'b0;
        end else begin
            dst_pulse <= 1'b0;

            if (dst_toggle_edge) begin
                dst_word  <= src_word_hold;
                dst_pulse <= 1'b1;
            end
        end
    end

endmodule

`default_nettype wire