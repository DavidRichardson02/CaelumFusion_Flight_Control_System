`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// snapshot_cdc_bundle
//------------------------------------------------------------------------------
// Two-way toggle CDC for a stable multi-bit bundle.
// SYS side:
//  - Detects sys_seq change.
//  - If not busy, latches sys_bundle_in into sys_hold and toggles req.
//  - Holds sys_hold stable until ack observed.
//
// PIX side:
//  - Syncs req toggle.
//  - On new req, captures sys_hold into pix_bundle_out.
//  - Toggles ack and emits pix_update_pulse for one pix_clk.
//
// Safety model
//  - sys_hold is held stable for many pix_clk cycles until ack returns.
//  - pix capture is a single registered sample on pix_clk after req edge.
//  - For additional robustness, capture uses a 2-sample settle window.
//==============================================================================

module snapshot_cdc_bundle #(
    parameter integer W = 105
)(
    input  wire         sys_clk,
    input  wire         sys_rst,
    input  wire [15:0]  sys_seq,
    input  wire [W-1:0] sys_bundle_in,

    input  wire         pix_clk,
    input  wire         pix_rst,
    output reg  [W-1:0] pix_bundle_out,
    output reg          pix_update_pulse
);
    // SYS holding register (stable during handshake)
    reg [W-1:0] sys_hold;

    // SYS sequence tracking
    reg [15:0] sys_seq_prev;

    // Toggle handshake
    reg sys_req_tog;
    reg pix_ack_tog;

    // Sync ack into SYS
    reg ack_ff1, ack_ff2;
    wire ack_sync = ack_ff2;

    // SYS busy flag
    reg sys_busy;

    always @(posedge sys_clk) begin
        if (sys_rst) begin
            sys_hold     <= {W{1'b0}};
            sys_seq_prev <= 16'd0;
            sys_req_tog  <= 1'b0;
            sys_busy     <= 1'b0;
            ack_ff1      <= 1'b0;
            ack_ff2      <= 1'b0;
        end else begin
            // sync ack
            ack_ff1 <= pix_ack_tog;
            ack_ff2 <= ack_ff1;

            // clear busy when ack toggles
            if (sys_busy && (ack_sync != sys_req_tog)) begin
                // ack has not yet matched req; keep busy
            end
            if (sys_busy && (ack_sync == sys_req_tog)) begin
                sys_busy <= 1'b0;
            end

            // detect new snapshot (seq changed)
            if (sys_seq != sys_seq_prev) begin
                sys_seq_prev <= sys_seq;

                // if not busy, launch request
                if (!sys_busy) begin
                    sys_hold    <= sys_bundle_in;
                    sys_req_tog <= ~sys_req_tog;
                    sys_busy    <= 1'b1;
                end
            end
        end
    end

    // PIX: sync req
    reg req_ff1, req_ff2;
    wire req_sync = req_ff2;

    // PIX track last req
    reg req_seen;

    // 2-sample settle capture
    reg [W-1:0] cap_a, cap_b;
    reg [1:0]   settle_ctr;
    reg         capturing;

    always @(posedge pix_clk) begin
        if (pix_rst) begin
            req_ff1 <= 1'b0;
            req_ff2 <= 1'b0;
            req_seen <= 1'b0;

            pix_bundle_out <= {W{1'b0}};
            pix_update_pulse <= 1'b0;

            pix_ack_tog <= 1'b0;

            cap_a <= {W{1'b0}};
            cap_b <= {W{1'b0}};
            settle_ctr <= 2'd0;
            capturing <= 1'b0;
        end else begin
            pix_update_pulse <= 1'b0;

            req_ff1 <= sys_req_tog;
            req_ff2 <= req_ff1;

            // New request detected when req_sync != req_seen
            if (!capturing && (req_sync != req_seen)) begin
                capturing   <= 1'b1;
                settle_ctr  <= 2'd2;   // two-sample window
            end

            if (capturing) begin
                // sample sys_hold (asynchronous bus, held stable in SYS)
                cap_a <= sys_hold;
                cap_b <= cap_a;

                if (settle_ctr != 2'd0) begin
                    settle_ctr <= settle_ctr - 2'd1;
                end else begin
                    // accept capture
                    pix_bundle_out <= cap_b;
                    pix_update_pulse <= 1'b1;

                    // acknowledge
                    pix_ack_tog <= req_sync;
                    req_seen    <= req_sync;

                    capturing   <= 1'b0;
                end
            end
        end
    end

endmodule


`default_nettype wire