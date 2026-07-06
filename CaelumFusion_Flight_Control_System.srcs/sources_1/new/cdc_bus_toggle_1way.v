//==============================================================================
// cdc_bus_toggle_1way
//------------------------------------------------------------------------------
// One-way CDC for a small bus using toggle+hold.
// SYS side:
//  - On sys_pulse, capture bus into hold and toggle req.
// PIX side:
//  - Sync req, capture hold into pix_bus on edge, emit pix_pulse.
//
// No feedback required for this signal (write pointer is monotonic and periodic).
//==============================================================================
module cdc_bus_toggle_1way #(
    parameter integer W = 10
)(
    input  wire         sys_clk,
    input  wire         sys_rst,
    input  wire         sys_pulse,
    input  wire [W-1:0] sys_bus,

    input  wire         pix_clk,
    input  wire         pix_rst,
    output reg  [W-1:0] pix_bus,
    output reg          pix_pulse
);
    reg [W-1:0] hold;
    reg req_tog;

    always @(posedge sys_clk) begin
        if (sys_rst) begin
            hold    <= {W{1'b0}};
            req_tog <= 1'b0;
        end else begin
            if (sys_pulse) begin
                hold    <= sys_bus;
                req_tog <= ~req_tog;
            end
        end
    end

    reg ff1, ff2;
    reg seen;

    always @(posedge pix_clk) begin
        if (pix_rst) begin
            ff1 <= 1'b0;
            ff2 <= 1'b0;
            seen <= 1'b0;
            pix_bus <= {W{1'b0}};
            pix_pulse <= 1'b0;
        end else begin
            pix_pulse <= 1'b0;
            ff1 <= req_tog;
            ff2 <= ff1;

            if (ff2 != seen) begin
                seen <= ff2;
                pix_bus <= hold;
                pix_pulse <= 1'b1;
            end
        end
    end
endmodule
