`timescale 1ns / 1ps
`default_nettype none

`include "telemetry_defs_vh.vh"

module tb_teensy_bridge_packet_ingress;
    reg clk = 1'b0;
    always #5 clk = ~clk;

    reg rst;
    reg enable;
    reg [31:0] now_us;
    reg tick_1us;

    reg        pkt_valid;
    wire       pkt_ready;
    reg [7:0]  pkt_type;
    reg [7:0]  pkt_status;
    reg [15:0] pkt_seq;
    reg [31:0] pkt_timestamp_us;
    reg [47:0] pkt_payload;
    reg [15:0] pkt_aux;
    reg [15:0] pkt_source_flags;
    reg [15:0] pkt_checksum;

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

    teensy_bridge_packet_ingress #(
        .RANGE_FRESH_MAX_MS        (16'd3),
        .HEARTBEAT_FRESH_MAX_MS    (16'd2),
        .REQUIRE_HEARTBEAT_FOR_DATA(1),
        .MAX_RANGE_CM              (16'd2000),
        .MIN_RANGE_CONFIDENCE      (16'd10)
    ) dut (
        .clk                         (clk),
        .rst                         (rst),
        .enable                      (enable),
        .now_us                      (now_us),
        .tick_1us                    (tick_1us),
        .pkt_valid                   (pkt_valid),
        .pkt_ready                   (pkt_ready),
        .pkt_type                    (pkt_type),
        .pkt_status                  (pkt_status),
        .pkt_seq                     (pkt_seq),
        .pkt_timestamp_us            (pkt_timestamp_us),
        .pkt_payload                 (pkt_payload),
        .pkt_aux                     (pkt_aux),
        .pkt_source_flags            (pkt_source_flags),
        .pkt_checksum                (pkt_checksum),
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
        input [160*8-1:0] message;
        begin
            if (!condition) begin
                $display("FAIL: %0s", message);
                $finish;
            end
        end
    endtask

    task clear_packet;
        begin
            pkt_valid        = 1'b0;
            pkt_type         = 8'd0;
            pkt_status       = `ST_NOT_INITIALIZED;
            pkt_seq          = 16'd0;
            pkt_timestamp_us = 32'd0;
            pkt_payload      = 48'd0;
            pkt_aux          = 16'd0;
            pkt_source_flags = 16'd0;
            pkt_checksum     = 16'd0;
        end
    endtask

    task set_checksum;
        begin
            pkt_checksum = checksum16(pkt_type, pkt_status, pkt_seq,
                                      pkt_timestamp_us, pkt_payload, pkt_aux,
                                      pkt_source_flags);
        end
    endtask

    task send_packet;
        begin
            @(negedge clk);
            expect(pkt_ready, "packet ingress ready before send");
            pkt_valid = 1'b1;
            @(negedge clk);
            pkt_valid = 1'b0;
            @(posedge clk);
            #1;
        end
    endtask

    task send_heartbeat;
        input [15:0] seq;
        begin
            pkt_type         = `PKT_TEENSY_HEARTBEAT;
            pkt_status       = `ST_OK;
            pkt_seq          = seq;
            pkt_timestamp_us = 32'h0102_0304;
            pkt_payload      = 48'h0000_0000_0000;
            pkt_aux          = 16'hCAFE;
            pkt_source_flags = (16'd1 << `EXT_SRC_TEENSY_BRIDGE_BIT);
            set_checksum();
            send_packet();
        end
    endtask

    task send_range;
        input [15:0] seq;
        input [15:0] height_cm;
        input [15:0] confidence;
        begin
            pkt_type         = `PKT_TEENSY_RANGE_AGL;
            pkt_status       = `ST_OK;
            pkt_seq          = seq;
            pkt_timestamp_us = {16'h1111, seq};
            pkt_payload      = {height_cm, confidence, 16'h0055};
            pkt_aux          = 16'h1234;
            pkt_source_flags = (16'd1 << `EXT_SRC_REAL_BIT) |
                               (16'd1 << `EXT_SRC_TEENSY_BRIDGE_BIT);
            set_checksum();
            send_packet();
        end
    endtask

    task wait_us_ticks;
        input integer ticks;
        integer i;
        begin
            for (i = 0; i < ticks; i = i + 1) begin
                @(negedge clk);
                tick_1us = 1'b1;
                now_us = now_us + 32'd1;
                @(negedge clk);
                tick_1us = 1'b0;
            end
            @(posedge clk);
            #1;
        end
    endtask

    initial begin
        rst = 1'b1;
        enable = 1'b0;
        now_us = 32'd0;
        tick_1us = 1'b0;
        clear_packet();

        repeat (4) @(posedge clk);
        rst = 1'b0;
        @(posedge clk);
        #1;

        expect(!pkt_ready, "disabled bridge is not ready");
        expect(!rng_valid, "reset clears range valid");
        expect(rng_status == `ST_NOT_INITIALIZED, "reset range status is not initialized");
        expect(rng_age_ms == 16'hFFFF, "reset range age is stale sentinel");

        enable = 1'b1;
        @(posedge clk);
        #1;
        expect(pkt_ready, "enabled bridge is ready");

        send_range(16'h0010, 16'd150, 16'd80);
        expect(!rng_valid, "range before heartbeat is not valid");
        expect(rng_status == `ST_STALE_REJECT, "missing heartbeat marks range stale");
        expect(rng_payload[47:32] == 16'd150, "missing-heartbeat packet still preserves height evidence");

        send_heartbeat(16'h0100);
        expect(bridge_heartbeat_seen, "heartbeat packet sets seen flag");
        expect(bridge_heartbeat_seq == 16'h0100, "heartbeat sequence publishes");
        expect(bridge_heartbeat_age_ms == 16'd0, "heartbeat age resets");

        send_range(16'h0101, 16'd185, 16'd95);
        expect(rng_valid, "fresh range packet publishes valid");
        expect(rng_status == `ST_OK, "fresh range packet status is OK");
        expect(rng_seq == 16'h0101, "range sequence publishes");
        expect(rng_t_us == 32'h1111_0101, "range timestamp publishes");
        expect(rng_payload[47:32] == 16'd185, "range height publishes");
        expect(rng_payload[31:16] == 16'd95, "range confidence publishes");
        expect(rng_payload[`EXT_SRC_REAL_BIT] == 1'b1, "range payload provenance keeps real bit");
        expect(rng_payload[`EXT_SRC_TEENSY_BRIDGE_BIT] == 1'b1, "range payload provenance keeps bridge bit");

        pkt_type         = `PKT_TEENSY_RANGE_AGL;
        pkt_status       = `ST_OK;
        pkt_seq          = 16'h0102;
        pkt_timestamp_us = 32'h1111_0102;
        pkt_payload      = {16'd190, 16'd90, 16'h0000};
        pkt_aux          = 16'h1234;
        pkt_source_flags = 16'd0;
        set_checksum();
        pkt_checksum = pkt_checksum ^ 16'h0001;
        send_packet();
        expect(!rng_valid, "corrupt range packet suppresses valid");
        expect(rng_status == `ST_CONFIG_ERROR, "corrupt range packet reports config error");
        expect(rng_payload == 48'd0, "corrupt range packet clears payload");
        expect(bridge_checksum_fault_count == 16'd1, "checksum fault count increments");

        send_heartbeat(16'h0103);
        send_range(16'h0104, 16'd210, 16'd88);
        wait_us_ticks(4000);
        expect(!rng_valid, "stale range packet clears valid");
        expect(rng_status == `ST_STALE_REJECT, "stale range packet reports stale reject");
        expect(rng_age_ms > 16'd3, "range age exceeds freshness threshold");

        wait_us_ticks(3000);
        send_range(16'h0105, 16'd220, 16'd88);
        expect(!rng_valid, "range after missing heartbeat remains invalid");
        expect(rng_status == `ST_STALE_REJECT, "stale heartbeat rejects range packet");

        pkt_type         = 8'h7E;
        pkt_status       = `ST_OK;
        pkt_seq          = 16'h0200;
        pkt_timestamp_us = 32'h2222_0000;
        pkt_payload      = 48'h0001_0002_0003;
        pkt_aux          = 16'h7777;
        pkt_source_flags = 16'd0;
        set_checksum();
        send_packet();
        expect(bridge_unsupported_count == 16'd1, "unsupported packet count increments");
        expect(bridge_last_type == 8'h7E, "last packet type records unsupported packet");
        expect(bridge_last_seq == 16'h0200, "last packet sequence records unsupported packet");

        send_heartbeat(16'h0201);
        send_range(16'h0202, 16'd230, 16'd99);
        send_range(16'h0203, 16'd231, 16'd99);
        expect(rng_valid, "deterministic replay sequence ends valid");
        expect(rng_seq == 16'h0203, "deterministic replay sequence preserves final seq");
        expect(rng_payload[47:32] == 16'd231, "deterministic replay sequence preserves final height");

        $display("PASS: tb_teensy_bridge_packet_ingress");
        $finish;
    end
endmodule

`default_nettype wire
