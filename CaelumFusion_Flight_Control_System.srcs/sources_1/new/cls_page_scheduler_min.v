`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// cls_page_scheduler_min
//------------------------------------------------------------------------------
// Minimal page scheduler.
//
// Page IDs:
//   0 = boot ladder
//   1 = summary
//   2 = counters
//   3 = build/version
//   7 = latched fault page
//
// Priority:
//   fault override > boot incomplete > manual selected steady-state page
//==============================================================================
module cls_page_scheduler_min (
    input  wire       clk,
    input  wire       rst,

    input  wire       boot_complete,
    input  wire       fault_latched,

    input  wire       page_step_pulse,

    output reg  [2:0] page_id
);

    reg [1:0] steady_page;

    always @(posedge clk) begin
        if (rst) begin
            steady_page <= 2'd0;
            page_id     <= 3'd0;
        end else begin
            if (page_step_pulse) begin
                steady_page <= steady_page + 2'd1;
            end

            if (fault_latched) begin
                page_id <= 3'd7;
            end else if (!boot_complete) begin
                page_id <= 3'd0;
            end else begin
                case (steady_page)
                    2'd0: page_id <= 3'd1;
                    2'd1: page_id <= 3'd2;
                    2'd2: page_id <= 3'd3;
                    default: page_id <= 3'd1;
                endcase
            end
        end
    end

endmodule

`default_nettype wire