`timescale 1ns / 1ps
`default_nettype none

//==============================================================================
// uart_rx_8n1
//------------------------------------------------------------------------------
// Small SYS-clock UART receiver for 8N1 streams.
//
// The asynchronous RX pin is synchronized before edge detection. byte_valid is a
// one-cycle pulse after a good stop bit. framing_error pulses when the stop bit
// is low; the bad byte is not reported as valid.
//==============================================================================
module uart_rx_8n1 #(
    parameter integer CLK_HZ = 100_000_000,
    parameter integer BAUD   = 115_200
) (
    input  wire       clk,
    input  wire       rst,
    input  wire       rx,
    output reg  [7:0] byte_data,
    output reg        byte_valid,
    output reg        framing_error
);
    localparam integer CLKS_PER_BIT_I =
        (BAUD > 0) ? (CLK_HZ / BAUD) : 1;
    localparam integer CLKS_PER_BIT =
        (CLKS_PER_BIT_I > 0) ? CLKS_PER_BIT_I : 1;
    localparam integer HALF_BIT =
        (CLKS_PER_BIT > 1) ? (CLKS_PER_BIT / 2) : 1;

    localparam [1:0]
        ST_IDLE  = 2'd0,
        ST_START = 2'd1,
        ST_DATA  = 2'd2,
        ST_STOP  = 2'd3;

    (* ASYNC_REG = "TRUE" *) reg rx_meta_r;
    (* ASYNC_REG = "TRUE" *) reg rx_sync_r;
    reg [1:0] state_r;
    reg [31:0] baud_ctr_r;
    reg [2:0] bit_idx_r;
    reg [7:0] shift_r;

    always @(posedge clk) begin
        if (rst) begin
            rx_meta_r      <= 1'b1;
            rx_sync_r      <= 1'b1;
            state_r        <= ST_IDLE;
            baud_ctr_r     <= 32'd0;
            bit_idx_r      <= 3'd0;
            shift_r        <= 8'd0;
            byte_data      <= 8'd0;
            byte_valid     <= 1'b0;
            framing_error  <= 1'b0;
        end else begin
            rx_meta_r     <= rx;
            rx_sync_r     <= rx_meta_r;
            byte_valid    <= 1'b0;
            framing_error <= 1'b0;

            case (state_r)
                ST_IDLE: begin
                    baud_ctr_r <= 32'd0;
                    bit_idx_r  <= 3'd0;
                    if (!rx_sync_r) begin
                        state_r    <= ST_START;
                        baud_ctr_r <= HALF_BIT[31:0];
                    end
                end

                ST_START: begin
                    if (baud_ctr_r != 32'd0) begin
                        baud_ctr_r <= baud_ctr_r - 32'd1;
                    end else if (!rx_sync_r) begin
                        state_r    <= ST_DATA;
                        baud_ctr_r <= (CLKS_PER_BIT - 1);
                        bit_idx_r  <= 3'd0;
                    end else begin
                        state_r <= ST_IDLE;
                    end
                end

                ST_DATA: begin
                    if (baud_ctr_r != 32'd0) begin
                        baud_ctr_r <= baud_ctr_r - 32'd1;
                    end else begin
                        shift_r[bit_idx_r] <= rx_sync_r;
                        baud_ctr_r <= (CLKS_PER_BIT - 1);
                        if (bit_idx_r == 3'd7) begin
                            state_r <= ST_STOP;
                        end else begin
                            bit_idx_r <= bit_idx_r + 3'd1;
                        end
                    end
                end

                ST_STOP: begin
                    if (baud_ctr_r != 32'd0) begin
                        baud_ctr_r <= baud_ctr_r - 32'd1;
                    end else begin
                        state_r <= ST_IDLE;
                        if (rx_sync_r) begin
                            byte_data  <= shift_r;
                            byte_valid <= 1'b1;
                        end else begin
                            framing_error <= 1'b1;
                        end
                    end
                end

                default: begin
                    state_r <= ST_IDLE;
                end
            endcase
        end
    end
endmodule

`default_nettype wire
