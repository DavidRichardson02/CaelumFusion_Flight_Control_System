`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// cls_refresh_fsm
//------------------------------------------------------------------------------
// Minimal refresh FSM for a 16x2 UART-connected character display.
//
// Policy:
//   - periodic full refresh
//   - send CR/LF, then line0[16], then CR/LF, then line1[16]
//   - refresh immediately on force_refresh
//
// Notes:
//   - This is a conservative first draft.
//   - Exact command set can be replaced later by a CLS-specific command encoder.
//==============================================================================
module cls_refresh_fsm #(
    parameter integer REFRESH_HZ = 2,
    parameter integer CLK_HZ     = 100_000_000
)(
    input  wire         clk,
    input  wire         rst,

    input  wire [127:0] line0,
    input  wire [127:0] line1,

    input  wire         force_refresh,

    output reg  [7:0]   tx_data,
    output reg          tx_start,
    input  wire         tx_busy,
    input  wire         tx_done
);

    localparam integer REFRESH_DIV = CLK_HZ / REFRESH_HZ;

    localparam [4:0]
        S_IDLE      = 5'd0,
        S_SEND_CR0  = 5'd1,
        S_WAIT_CR0  = 5'd2,
        S_SEND_LF0  = 5'd3,
        S_WAIT_LF0  = 5'd4,
        S_SEND_L0   = 5'd5,
        S_WAIT_L0   = 5'd6,
        S_SEND_CR1  = 5'd7,
        S_WAIT_CR1  = 5'd8,
        S_SEND_LF1  = 5'd9,
        S_WAIT_LF1  = 5'd10,
        S_SEND_L1   = 5'd11,
        S_WAIT_L1   = 5'd12;

    reg [4:0]  st;
    reg [31:0] refresh_ctr;
    reg [4:0]  char_idx;

    reg [127:0] line0_q;
    reg [127:0] line1_q;

    wire refresh_due = (refresh_ctr == 32'd0);

    function [7:0] char_at16;
        input [127:0] line;
        input [4:0] idx;
        begin
            case (idx)
                5'd0:  char_at16 = line[127:120];
                5'd1:  char_at16 = line[119:112];
                5'd2:  char_at16 = line[111:104];
                5'd3:  char_at16 = line[103:96];
                5'd4:  char_at16 = line[95:88];
                5'd5:  char_at16 = line[87:80];
                5'd6:  char_at16 = line[79:72];
                5'd7:  char_at16 = line[71:64];
                5'd8:  char_at16 = line[63:56];
                5'd9:  char_at16 = line[55:48];
                5'd10: char_at16 = line[47:40];
                5'd11: char_at16 = line[39:32];
                5'd12: char_at16 = line[31:24];
                5'd13: char_at16 = line[23:16];
                5'd14: char_at16 = line[15:8];
                default: char_at16 = line[7:0];
            endcase
        end
    endfunction

    always @(posedge clk) begin
        if (rst) begin
            st          <= S_IDLE;
            refresh_ctr <= REFRESH_DIV - 1;
            char_idx    <= 5'd0;
            line0_q     <= {16{" "}};
            line1_q     <= {16{" "}};
            tx_data     <= 8'd0;
            tx_start    <= 1'b0;
        end else begin
            tx_start <= 1'b0;

            if (refresh_ctr != 0)
                refresh_ctr <= refresh_ctr - 32'd1;

            case (st)
                S_IDLE: begin
                    if (force_refresh || refresh_due) begin
                        refresh_ctr <= REFRESH_DIV - 1;
                        line0_q     <= line0;
                        line1_q     <= line1;
                        st          <= S_SEND_CR0;
                    end
                end

                S_SEND_CR0: begin
                    if (!tx_busy) begin
                        tx_data  <= 8'h0D;
                        tx_start <= 1'b1;
                        st       <= S_WAIT_CR0;
                    end
                end
                S_WAIT_CR0: if (tx_done) st <= S_SEND_LF0;

                S_SEND_LF0: begin
                    if (!tx_busy) begin
                        tx_data  <= 8'h0A;
                        tx_start <= 1'b1;
                        char_idx <= 5'd0;
                        st       <= S_WAIT_LF0;
                    end
                end
                S_WAIT_LF0: if (tx_done) st <= S_SEND_L0;

                S_SEND_L0: begin
                    if (!tx_busy) begin
                        tx_data  <= char_at16(line0_q, char_idx);
                        tx_start <= 1'b1;
                        st       <= S_WAIT_L0;
                    end
                end
                S_WAIT_L0: begin
                    if (tx_done) begin
                        if (char_idx == 5'd15)
                            st <= S_SEND_CR1;
                        else begin
                            char_idx <= char_idx + 5'd1;
                            st <= S_SEND_L0;
                        end
                    end
                end

                S_SEND_CR1: begin
                    if (!tx_busy) begin
                        tx_data  <= 8'h0D;
                        tx_start <= 1'b1;
                        st       <= S_WAIT_CR1;
                    end
                end
                S_WAIT_CR1: if (tx_done) st <= S_SEND_LF1;

                S_SEND_LF1: begin
                    if (!tx_busy) begin
                        tx_data  <= 8'h0A;
                        tx_start <= 1'b1;
                        char_idx <= 5'd0;
                        st       <= S_WAIT_LF1;
                    end
                end
                S_WAIT_LF1: if (tx_done) st <= S_SEND_L1;

                S_SEND_L1: begin
                    if (!tx_busy) begin
                        tx_data  <= char_at16(line1_q, char_idx);
                        tx_start <= 1'b1;
                        st       <= S_WAIT_L1;
                    end
                end
                S_WAIT_L1: begin
                    if (tx_done) begin
                        if (char_idx == 5'd15)
                            st <= S_IDLE;
                        else begin
                            char_idx <= char_idx + 5'd1;
                            st <= S_SEND_L1;
                        end
                    end
                end

                default: st <= S_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire