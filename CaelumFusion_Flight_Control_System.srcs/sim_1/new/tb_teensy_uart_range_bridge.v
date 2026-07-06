`timescale 1ns / 1ps
`default_nettype none

`include "telemetry_defs_vh.vh"

module tb_teensy_uart_range_bridge;
    localparam integer CLK_HZ = 1_000_000;
    localparam integer BAUD   = 100_000;
    localparam integer BIT_CYCLES = CLK_HZ / BAUD;

    reg clk = 1'b0;
    always #500 clk = ~clk;

    reg rst;
    reg enable;
    reg [31:0] now_us;
    reg tick_1us;
    reg uart_rx;
    wire uart_tx;

    wire [31:0] rng_t_us;
    wire [15:0] rng_seq;
    wire        rng_valid;
    wire [7:0]  rng_status;
    wire [47:0] rng_payload;
    wire [15:0] rng_age_ms;
    wire [7:0]  bridge_last_type;
    wire [15:0] bridge_last_seq;
    wire        bridge_heartbeat_seen;
    wire [15:0] bridge_heartbeat_seq;
    wire [15:0] bridge_heartbeat_age_ms;
    wire [15:0] bridge_checksum_fault_count;
    wire [15:0] bridge_unsupported_count;
    wire [15:0] uart_framing_error_count;

    teensy_uart_range_bridge #(
        .CLK_HZ(CLK_HZ),
        .BAUD(BAUD),
        .RANGE_FRESH_MAX_MS(16'd5),
        .HEARTBEAT_FRESH_MAX_MS(16'd5),
        .REQUIRE_HEARTBEAT_FOR_DATA(1),
        .MAX_RANGE_CM(16'd2000),
        .MIN_RANGE_CONFIDENCE(16'd10)
    ) dut (
        .clk                         (clk),
        .rst                         (rst),
        .enable                      (enable),
        .now_us                      (now_us),
        .tick_1us                    (tick_1us),
        .uart_rx                     (uart_rx),
        .uart_tx                     (uart_tx),
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
        .bridge_unsupported_count    (bridge_unsupported_count),
        .uart_framing_error_count    (uart_framing_error_count)
    );

    function [15:0] checksum16;
        input [7:0]  typ;
        input [7:0]  stat;
        input [15:0] seq;
        input [31:0] timestamp_us;
        input [47:0] payload;
        input [15:0] aux;
        input [15:0] source_flags;
        begin
            checksum16 =
                `TELEM_PKT_SYNC ^
                {stat, typ} ^
                seq ^
                timestamp_us[31:16] ^ timestamp_us[15:0] ^
                payload[47:32] ^ payload[31:16] ^ payload[15:0] ^
                aux ^
                source_flags;
        end
    endfunction

    task expect;
        input condition;
        input [140*8-1:0] message;
        begin
            if (!condition) begin
                $display("FAIL: %0s", message);
                $finish;
            end
        end
    endtask

    task wait_cycles;
        input integer cycles;
        integer i;
        begin
            for (i = 0; i < cycles; i = i + 1)
                @(posedge clk);
        end
    endtask

    task send_uart_byte;
        input [7:0] b;
        integer i;
        begin
            uart_rx = 1'b0;
            wait_cycles(BIT_CYCLES);
            for (i = 0; i < 8; i = i + 1) begin
                uart_rx = b[i];
                wait_cycles(BIT_CYCLES);
            end
            uart_rx = 1'b1;
            wait_cycles(BIT_CYCLES);
            wait_cycles(BIT_CYCLES);
        end
    endtask

    task send_frame;
        input [7:0] typ;
        input [7:0] stat;
        input [15:0] seq;
        input [31:0] timestamp_us;
        input [47:0] payload;
        input [15:0] aux;
        input [15:0] source_flags;
        input corrupt;
        reg [15:0] sum;
        begin
            sum = checksum16(typ, stat, seq, timestamp_us, payload, aux, source_flags);
            if (corrupt)
                sum = sum ^ 16'h0001;

            send_uart_byte(8'hA5);
            send_uart_byte(8'h5A);
            send_uart_byte(typ);
            send_uart_byte(stat);
            send_uart_byte(seq[15:8]);
            send_uart_byte(seq[7:0]);
            send_uart_byte(timestamp_us[31:24]);
            send_uart_byte(timestamp_us[23:16]);
            send_uart_byte(timestamp_us[15:8]);
            send_uart_byte(timestamp_us[7:0]);
            send_uart_byte(payload[47:40]);
            send_uart_byte(payload[39:32]);
            send_uart_byte(payload[31:24]);
            send_uart_byte(payload[23:16]);
            send_uart_byte(payload[15:8]);
            send_uart_byte(payload[7:0]);
            send_uart_byte(aux[15:8]);
            send_uart_byte(aux[7:0]);
            send_uart_byte(source_flags[15:8]);
            send_uart_byte(source_flags[7:0]);
            send_uart_byte(sum[15:8]);
            send_uart_byte(sum[7:0]);
            wait_cycles(40);
        end
    endtask

    task tick_us;
        input integer count;
        integer i;
        begin
            for (i = 0; i < count; i = i + 1) begin
                @(negedge clk);
                tick_1us = 1'b1;
                now_us = now_us + 32'd1;
                @(negedge clk);
                tick_1us = 1'b0;
            end
            @(posedge clk);
        end
    endtask

    initial begin
        rst = 1'b1;
        enable = 1'b0;
        now_us = 32'd0;
        tick_1us = 1'b0;
        uart_rx = 1'b1;

        wait_cycles(8);
        rst = 1'b0;
        wait_cycles(4);

        expect(uart_tx == 1'b1, "UART TX idles high");
        expect(!rng_valid, "reset clears range valid");
        expect(rng_status == `ST_NOT_INITIALIZED, "reset range status is not initialized");

        enable = 1'b1;
        wait_cycles(8);

        send_frame(`PKT_TEENSY_RANGE_AGL, `ST_OK, 16'h0100,
                   32'h0001_0100, {16'd150, 16'd90, 16'd0},
                   16'h1111,
                   (16'd1 << `EXT_SRC_REAL_BIT) |
                   (16'd1 << `EXT_SRC_TEENSY_BRIDGE_BIT),
                   1'b0);
        expect(!rng_valid, "range before heartbeat is rejected");
        expect(rng_status == `ST_STALE_REJECT, "missing heartbeat reports stale reject");

        send_frame(`PKT_TEENSY_HEARTBEAT, `ST_OK, 16'h0200,
                   32'h0002_0000, 48'd0, 16'h2222,
                   (16'd1 << `EXT_SRC_TEENSY_BRIDGE_BIT),
                   1'b0);
        expect(bridge_heartbeat_seen, "heartbeat frame sets seen");
        expect(bridge_heartbeat_seq == 16'h0200, "heartbeat sequence publishes");

        send_frame(`PKT_TEENSY_RANGE_AGL, `ST_OK, 16'h0201,
                   32'h0002_0201, {16'd185, 16'd95, 16'd0},
                   16'h3333,
                   (16'd1 << `EXT_SRC_REAL_BIT) |
                   (16'd1 << `EXT_SRC_TEENSY_BRIDGE_BIT),
                   1'b0);
        expect(rng_valid, "range after heartbeat is valid");
        expect(rng_status == `ST_OK, "range status OK");
        expect(rng_seq == 16'h0201, "range sequence publishes");
        expect(rng_t_us == 32'h0002_0201, "range timestamp publishes");
        expect(rng_payload[47:32] == 16'd185, "range height publishes");
        expect(rng_payload[31:16] == 16'd95, "range confidence publishes");

        send_frame(`PKT_TEENSY_RANGE_AGL, `ST_OK, 16'h0202,
                   32'h0002_0202, {16'd190, 16'd88, 16'd0},
                   16'h4444,
                   (16'd1 << `EXT_SRC_TEENSY_BRIDGE_BIT),
                   1'b1);
        expect(!rng_valid, "corrupt range frame suppresses valid");
        expect(rng_status == `ST_CONFIG_ERROR, "corrupt range reports config error");
        expect(bridge_checksum_fault_count == 16'd1, "checksum fault counted");
        expect(uart_framing_error_count == 16'd0, "valid 8N1 frames have no framing errors");

        tick_us(7000);
        send_frame(`PKT_TEENSY_RANGE_AGL, `ST_OK, 16'h0203,
                   32'h0002_0203, {16'd191, 16'd88, 16'd0},
                   16'h5555,
                   (16'd1 << `EXT_SRC_TEENSY_BRIDGE_BIT),
                   1'b0);
        expect(!rng_valid, "stale heartbeat rejects later range");
        expect(rng_status == `ST_STALE_REJECT, "stale heartbeat status is stale reject");

        send_frame(8'h7E, `ST_OK, 16'h0300,
                   32'h0003_0000, 48'h0001_0002_0003,
                   16'h6666, 16'd0, 1'b0);
        expect(bridge_unsupported_count == 16'd1, "unsupported frame counted");
        expect(bridge_last_type == 8'h7E, "last unsupported type recorded");
        expect(bridge_last_seq == 16'h0300, "last unsupported sequence recorded");

        $display("PASS: tb_teensy_uart_range_bridge");
        $finish;
    end
endmodule

`default_nettype wire
