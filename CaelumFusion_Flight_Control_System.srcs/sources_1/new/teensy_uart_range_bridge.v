`timescale 1ns / 1ps
`default_nettype none

`include "telemetry_defs_vh.vh"

//==============================================================================
// teensy_uart_range_bridge
//------------------------------------------------------------------------------
// UART transport wrapper for the fixed Teensy bridge packet ingress.
//
// Physical contract:
//   - UART RX is FPGA input from Teensy TX, 3.3 V logic, idle high.
//   - UART TX is currently idle high and reserved for future ACK/debug traffic.
//
// Frame format, byte order:
//   A5 5A, type, status, seq[15:8], seq[7:0],
//   timestamp[31:24], timestamp[23:16], timestamp[15:8], timestamp[7:0],
//   payload[47:40] ... payload[7:0],
//   aux[15:8], aux[7:0],
//   source_flags[15:8], source_flags[7:0],
//   checksum[15:8], checksum[7:0]
//==============================================================================
module teensy_uart_range_bridge #(
    parameter integer CLK_HZ                    = 100_000_000,
    parameter integer BAUD                      = 115_200,
    parameter integer PAYLOAD_W                 = 48,
    parameter [15:0]  RANGE_FRESH_MAX_MS        = 16'd500,
    parameter [15:0]  HEARTBEAT_FRESH_MAX_MS    = 16'd500,
    parameter integer REQUIRE_HEARTBEAT_FOR_DATA = 1,
    parameter [15:0]  MAX_RANGE_CM              = 16'd10000,
    parameter [15:0]  MIN_RANGE_CONFIDENCE      = 16'd1
) (
    input  wire                 clk,
    input  wire                 rst,
    input  wire                 enable,
    input  wire [31:0]          now_us,
    input  wire                 tick_1us,
    input  wire                 uart_rx,
    output wire                 uart_tx,

    output wire [31:0]          rng_t_us,
    output wire [15:0]          rng_seq,
    output wire                 rng_valid,
    output wire [7:0]           rng_status,
    output wire [PAYLOAD_W-1:0] rng_payload,
    output wire [15:0]          rng_age_ms,

    output wire [7:0]           bridge_last_type,
    output wire [15:0]          bridge_last_seq,
    output wire                 bridge_heartbeat_seen,
    output wire [15:0]          bridge_heartbeat_seq,
    output wire [15:0]          bridge_heartbeat_age_ms,
    output wire [15:0]          bridge_checksum_fault_count,
    output wire [15:0]          bridge_unsupported_count,
    output reg  [15:0]          uart_framing_error_count
);
    localparam [1:0]
        RX_WAIT_A5 = 2'd0,
        RX_WAIT_5A = 2'd1,
        RX_COLLECT = 2'd2,
        RX_EMIT    = 2'd3;

    wire [7:0] uart_byte_w;
    wire       uart_byte_valid_w;
    wire       uart_framing_error_w;

    uart_rx_8n1 #(
        .CLK_HZ(CLK_HZ),
        .BAUD  (BAUD)
    ) u_uart_rx_8n1 (
        .clk           (clk),
        .rst           (rst),
        .rx            (uart_rx),
        .byte_data     (uart_byte_w),
        .byte_valid    (uart_byte_valid_w),
        .framing_error (uart_framing_error_w)
    );

    reg [1:0]  rx_state_r;
    reg [4:0]  frame_idx_r;
    reg        pkt_valid_r;
    wire       pkt_ready_w;
    reg [7:0]  pkt_type_r;
    reg [7:0]  pkt_status_r;
    reg [15:0] pkt_seq_r;
    reg [31:0] pkt_timestamp_us_r;
    reg [PAYLOAD_W-1:0] pkt_payload_r;
    reg [15:0] pkt_aux_r;
    reg [15:0] pkt_source_flags_r;
    reg [15:0] pkt_checksum_r;

    assign uart_tx = 1'b1;

    always @(posedge clk) begin
        if (rst || !enable) begin
            rx_state_r     <= RX_WAIT_A5;
            frame_idx_r    <= 5'd0;
            pkt_valid_r    <= 1'b0;
            pkt_type_r     <= 8'd0;
            pkt_status_r   <= `ST_NOT_INITIALIZED;
            pkt_seq_r      <= 16'd0;
            pkt_timestamp_us_r <= 32'd0;
            pkt_payload_r  <= {PAYLOAD_W{1'b0}};
            pkt_aux_r      <= 16'd0;
            pkt_source_flags_r <= 16'd0;
            pkt_checksum_r <= 16'd0;
            uart_framing_error_count <= 16'd0;
        end else begin
            pkt_valid_r <= 1'b0;

            if (uart_framing_error_w &&
                (uart_framing_error_count != 16'hFFFF))
                uart_framing_error_count <= uart_framing_error_count + 16'd1;

            case (rx_state_r)
                RX_WAIT_A5: begin
                    frame_idx_r <= 5'd0;
                    if (uart_byte_valid_w && (uart_byte_w == 8'hA5))
                        rx_state_r <= RX_WAIT_5A;
                end

                RX_WAIT_5A: begin
                    if (uart_byte_valid_w) begin
                        if (uart_byte_w == 8'h5A) begin
                            frame_idx_r <= 5'd0;
                            rx_state_r  <= RX_COLLECT;
                        end else if (uart_byte_w == 8'hA5) begin
                            rx_state_r <= RX_WAIT_5A;
                        end else begin
                            rx_state_r <= RX_WAIT_A5;
                        end
                    end
                end

                RX_COLLECT: begin
                    if (uart_byte_valid_w) begin
                        case (frame_idx_r)
                            5'd0:  pkt_type_r <= uart_byte_w;
                            5'd1:  pkt_status_r <= uart_byte_w;
                            5'd2:  pkt_seq_r[15:8] <= uart_byte_w;
                            5'd3:  pkt_seq_r[7:0] <= uart_byte_w;
                            5'd4:  pkt_timestamp_us_r[31:24] <= uart_byte_w;
                            5'd5:  pkt_timestamp_us_r[23:16] <= uart_byte_w;
                            5'd6:  pkt_timestamp_us_r[15:8] <= uart_byte_w;
                            5'd7:  pkt_timestamp_us_r[7:0] <= uart_byte_w;
                            5'd8:  pkt_payload_r[47:40] <= uart_byte_w;
                            5'd9:  pkt_payload_r[39:32] <= uart_byte_w;
                            5'd10: pkt_payload_r[31:24] <= uart_byte_w;
                            5'd11: pkt_payload_r[23:16] <= uart_byte_w;
                            5'd12: pkt_payload_r[15:8] <= uart_byte_w;
                            5'd13: pkt_payload_r[7:0] <= uart_byte_w;
                            5'd14: pkt_aux_r[15:8] <= uart_byte_w;
                            5'd15: pkt_aux_r[7:0] <= uart_byte_w;
                            5'd16: pkt_source_flags_r[15:8] <= uart_byte_w;
                            5'd17: pkt_source_flags_r[7:0] <= uart_byte_w;
                            5'd18: pkt_checksum_r[15:8] <= uart_byte_w;
                            5'd19: pkt_checksum_r[7:0] <= uart_byte_w;
                            default: begin end
                        endcase

                        if (frame_idx_r == 5'd19) begin
                            rx_state_r <= RX_EMIT;
                        end else begin
                            frame_idx_r <= frame_idx_r + 5'd1;
                        end
                    end
                end

                RX_EMIT: begin
                    if (pkt_ready_w) begin
                        pkt_valid_r <= 1'b1;
                        rx_state_r  <= RX_WAIT_A5;
                    end
                end

                default: begin
                    rx_state_r <= RX_WAIT_A5;
                end
            endcase
        end
    end

    teensy_bridge_packet_ingress #(
        .PAYLOAD_W                 (PAYLOAD_W),
        .RANGE_FRESH_MAX_MS        (RANGE_FRESH_MAX_MS),
        .HEARTBEAT_FRESH_MAX_MS    (HEARTBEAT_FRESH_MAX_MS),
        .REQUIRE_HEARTBEAT_FOR_DATA(REQUIRE_HEARTBEAT_FOR_DATA),
        .MAX_RANGE_CM              (MAX_RANGE_CM),
        .MIN_RANGE_CONFIDENCE      (MIN_RANGE_CONFIDENCE)
    ) u_teensy_bridge_packet_ingress (
        .clk                         (clk),
        .rst                         (rst),
        .enable                      (enable),
        .now_us                      (now_us),
        .tick_1us                    (tick_1us),
        .pkt_valid                   (pkt_valid_r),
        .pkt_ready                   (pkt_ready_w),
        .pkt_type                    (pkt_type_r),
        .pkt_status                  (pkt_status_r),
        .pkt_seq                     (pkt_seq_r),
        .pkt_timestamp_us            (pkt_timestamp_us_r),
        .pkt_payload                 (pkt_payload_r),
        .pkt_aux                     (pkt_aux_r),
        .pkt_source_flags            (pkt_source_flags_r),
        .pkt_checksum                (pkt_checksum_r),
        .rng_t_us                    (rng_t_us),
        .rng_seq                     (rng_seq),
        .rng_valid                   (rng_valid),
        .rng_status                  (rng_status),
        .rng_payload                 (rng_payload),
        .rng_age_ms                  (rng_age_ms),
        .bridge_last_type            (bridge_last_type),
        .bridge_last_seq             (bridge_last_seq),
        .bridge_heartbeat_seen       (bridge_heartbeat_seen),
        .bridge_heartbeat_seq        (bridge_heartbeat_seq),
        .bridge_heartbeat_age_ms     (bridge_heartbeat_age_ms),
        .bridge_checksum_fault_count (bridge_checksum_fault_count),
        .bridge_unsupported_count    (bridge_unsupported_count)
    );
endmodule

`default_nettype wire
