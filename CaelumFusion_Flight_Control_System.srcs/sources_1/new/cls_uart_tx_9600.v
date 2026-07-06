`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// cls_uart_tx_9600
//------------------------------------------------------------------------------
// Simple 8N1 UART transmitter.
// Default target:
//   CLK_HZ   = 100 MHz
//   BAUD_HZ  = 9600
//
// Interface:
//   - start is sampled only when busy=0
//   - done pulses for one clk cycle after stop bit completes
//==============================================================================
module cls_uart_tx_9600 #(
    parameter integer CLK_HZ  = 100_000_000,
    parameter integer BAUD_HZ = 9600
)(
    input  wire       clk,
    input  wire       rst,

    input  wire [7:0] data_in,
    input  wire       start,

    output reg        tx,
    output reg        busy,
    output reg        done
);

    localparam integer BAUD_DIV = CLK_HZ / BAUD_HZ;
    localparam [1:0]
        S_IDLE  = 2'd0,
        S_START = 2'd1,
        S_DATA  = 2'd2,
        S_STOP  = 2'd3;

    reg [1:0]  st;
    reg [15:0] baud_ctr;
    reg [2:0]  bit_idx;
    reg [7:0]  shreg;

    always @(posedge clk) begin
        if (rst) begin
            st       <= S_IDLE;
            baud_ctr <= 16'd0;
            bit_idx  <= 3'd0;
            shreg    <= 8'd0;
            tx       <= 1'b1;
            busy     <= 1'b0;
            done     <= 1'b0;
        end else begin
            done <= 1'b0;

            case (st)
                S_IDLE: begin
                    tx   <= 1'b1;
                    busy <= 1'b0;
                    if (start) begin
                        shreg    <= data_in;
                        baud_ctr <= BAUD_DIV - 1;
                        bit_idx  <= 3'd0;
                        tx       <= 1'b0; // start bit
                        busy     <= 1'b1;
                        st       <= S_START;
                    end
                end

                S_START: begin
                    if (baud_ctr == 0) begin
                        baud_ctr <= BAUD_DIV - 1;
                        tx       <= shreg[0];
                        st       <= S_DATA;
                    end else begin
                        baud_ctr <= baud_ctr - 16'd1;
                    end
                end

                S_DATA: begin
                    if (baud_ctr == 0) begin
                        baud_ctr <= BAUD_DIV - 1;
                        if (bit_idx == 3'd7) begin
                            tx <= 1'b1; // stop bit
                            st <= S_STOP;
                        end else begin
                            bit_idx <= bit_idx + 3'd1;
                            tx      <= shreg[bit_idx + 3'd1];
                        end
                    end else begin
                        baud_ctr <= baud_ctr - 16'd1;
                    end
                end

                S_STOP: begin
                    if (baud_ctr == 0) begin
                        tx   <= 1'b1;
                        busy <= 1'b0;
                        done <= 1'b1;
                        st   <= S_IDLE;
                    end else begin
                        baud_ctr <= baud_ctr - 16'd1;
                    end
                end

                default: begin
                    st   <= S_IDLE;
                    tx   <= 1'b1;
                    busy <= 1'b0;
                end
            endcase
        end
    end

endmodule

`default_nettype wire