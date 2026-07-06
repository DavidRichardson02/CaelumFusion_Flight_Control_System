`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// flight_viz_bundle_cdc
//------------------------------------------------------------------------------
// ROLE
//   Transfer a packed visualization bundle from SYS clock domain into PIX clock
//   domain using a stable source hold register plus toggle-event synchronization.
//
// HIGH-LEVEL CONTRACT
//   Source domain:
//     - src_bundle is the semantic image that should be published
//     - src_publish_pulse marks the instant at which that image becomes the next
//       official snapshot for transfer
//
//   Destination domain:
//     - dst_bundle_shadow is the latest CDC-delivered shadow image
//     - dst_update_pulse is a one-cycle pulse indicating that a new shadow image
//       has just been captured in dst_clk domain
//
// WHY THIS MODULE EXISTS
//   The visualization path requires a clear separation between:
//
//     1) semantic publication in SYS domain
//     2) cross-domain arrival in PIX domain
//     3) frame-boundary visible commit in PIX domain
//
//   This module performs only step (2).
//   It does NOT decide when the visible HUD changes. That is the responsibility
//   of the PIX-domain frame-commit logic in flight_viz_suite_top.
//
// CDC STRATEGY
//   The chosen strategy is:
//
//     - When src_publish_pulse occurs, latch src_bundle into src_bundle_hold.
//     - Simultaneously toggle src_toggle.
//     - Synchronize src_toggle into the destination domain.
//     - Detect a toggle edge in the destination domain.
//     - On that edge, sample src_bundle_hold into dst_bundle_shadow.
//
// IMPORTANT ASSUMPTION
//   This design assumes that:
//     - src_bundle_hold remains stable until the next publish event
//     - src_publish_pulse is not issued so rapidly that the destination domain
//       misses distinct toggle events
//
//   In the present flight visualization use case, that assumption is appropriate
//   because semantic bundle publications are slow relative to the pixel clock and
//   frame timing.
//
// ROBUSTNESS NOTES
//   - The toggle event itself is synchronized with a small FF chain.
//   - The wide bus is not individually synchronized bit-by-bit.
//   - Instead, the wide bus is treated as a snapshot that is held stable long
//     enough to be sampled safely after the synchronized event arrives.
//
//   This is the standard and appropriate pattern for low-rate snapshot transfer.
//
// RESET POLICY
//   - Both domains reset their local state independently.
//   - After reset, dst_bundle_shadow is zero and dst_update_pulse is low.
//==============================================================================
module flight_viz_bundle_cdc #(
    parameter integer BUNDLE_W = 640
)(
    //==========================================================================
    // Source domain
    //==========================================================================
    input  wire                 src_clk,
    input  wire                 src_rst,
    input  wire [BUNDLE_W-1:0]  src_bundle,
    input  wire                 src_publish_pulse,

    //==========================================================================
    // Destination domain
    //==========================================================================
    input  wire                 dst_clk,
    input  wire                 dst_rst,
    output reg  [BUNDLE_W-1:0]  dst_bundle_shadow,
    output reg                  dst_update_pulse
);

    //==========================================================================
    // SECTION 1: Source-domain snapshot hold and event toggle
    //--------------------------------------------------------------------------
    // src_bundle_hold
    //   Stable copy of the most recently published semantic image.
    //
    // src_toggle
    //   Event marker toggled once per publication.
    //
    // PUBLICATION RULE
    //   Only src_publish_pulse is allowed to update src_bundle_hold.
    //   This guarantees that the destination side samples a well-defined
    //   snapshot image rather than an arbitrary live combinational bus.
    //==========================================================================
    reg [BUNDLE_W-1:0] src_bundle_hold;
    reg                src_toggle;

    always @(posedge src_clk) begin
        if (src_rst) begin
            src_bundle_hold <= {BUNDLE_W{1'b0}};
            src_toggle      <= 1'b0;
        end else if (src_publish_pulse) begin
            src_bundle_hold <= src_bundle;
            src_toggle      <= ~src_toggle;
        end
    end

    //==========================================================================
    // SECTION 2: Destination-domain synchronization of event toggle
    //--------------------------------------------------------------------------
    // A 3-stage synchronizer is used for the toggle event.
    //
    // WHY 3 STAGES
    //   Two stages are generally sufficient for metastability reduction.
    //   A third stage is convenient here because it gives a clean "previous vs
    //   current" comparison for toggle-change detection without reusing the same
    //   FF chain ambiguously.
    //
    // dst_toggle_sync_ff[0]
    //   first sampled version of src_toggle
    // dst_toggle_sync_ff[1]
    //   second sampled version
    // dst_toggle_sync_ff[2]
    //   previous settled version for edge detection
    //==========================================================================
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg [2:0] dst_toggle_sync_ff;

    always @(posedge dst_clk) begin
        if (dst_rst) begin
            dst_toggle_sync_ff <= 3'b000;
        end else begin
            dst_toggle_sync_ff[0] <= src_toggle;
            dst_toggle_sync_ff[1] <= dst_toggle_sync_ff[0];
            dst_toggle_sync_ff[2] <= dst_toggle_sync_ff[1];
        end
    end

    wire dst_toggle_changed;
    assign dst_toggle_changed =
        (dst_toggle_sync_ff[2] ^ dst_toggle_sync_ff[1]);

    //==========================================================================
    // SECTION 3: Destination-domain shadow capture
    //--------------------------------------------------------------------------
    // On a detected synchronized toggle change:
    //   - sample the stable source hold bus into dst_bundle_shadow
    //   - assert dst_update_pulse for exactly one dst_clk cycle
    //
    // WHY CAPTURE ON THE CHANGE EVENT
    //   The source hold bus has already been made stable in the source domain at
    //   the same instant the toggle flipped. By the time the synchronized toggle
    //   transition reaches the destination domain, the held bus has had ample
    //   time to settle.
    //
    // IMPORTANT
    //   This block does not attempt frame alignment. It merely produces a clean
    //   destination-domain shadow image and event pulse.
    //==========================================================================
    always @(posedge dst_clk) begin
        if (dst_rst) begin
            dst_bundle_shadow <= {BUNDLE_W{1'b0}};
            dst_update_pulse  <= 1'b0;
        end else begin
            dst_update_pulse <= 1'b0;

            if (dst_toggle_changed) begin
                dst_bundle_shadow <= src_bundle_hold;
                dst_update_pulse  <= 1'b1;
            end
        end
    end

endmodule

`default_nettype wire
