`timescale 1ns / 1ps
`default_nettype none

`include "telemetry_defs_vh.vh"

//==============================================================================
// teensy_bridge_packet_ingress
//------------------------------------------------------------------------------
// Fixed-packet SYS-domain ingress for decoded Teensy/host bridge evidence.
//
// This block is not a UART/SPI parser and does not own SD-card, telemetry-radio,
// GNSS, lidar, or filesystem behavior. A transport block upstream must present
// one coherent packet per ready/valid handshake. This module verifies a simple
// 16-bit XOR checksum, dispatches supported packet types, and publishes the
// first bridge-owned raw bank: rangefinder / lidar AGL evidence.
//
// Range packet payload contract for PKT_TEENSY_RANGE_AGL:
//   pkt_payload[47:32] = range height above ground, centimeters
//   pkt_payload[31:16] = confidence / quality, centi-percent style scalar
//   pkt_payload[15:0]  = producer-defined raw detail, replaced by source flags
//                        in rng_payload so provenance survives downstream
//
// Checksum contract:
//   checksum = XOR of TELEM_PKT_SYNC, {pkt_status,pkt_type}, pkt_seq,
//              pkt_timestamp_us[31:16], pkt_timestamp_us[15:0],
//              payload words, pkt_aux, and pkt_source_flags.
//==============================================================================
module teensy_bridge_packet_ingress #(
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

    input  wire                 pkt_valid,
    output wire                 pkt_ready,
    input  wire [7:0]           pkt_type,
    input  wire [7:0]           pkt_status,
    input  wire [15:0]          pkt_seq,
    input  wire [31:0]          pkt_timestamp_us,
    input  wire [PAYLOAD_W-1:0] pkt_payload,
    input  wire [15:0]          pkt_aux,
    input  wire [15:0]          pkt_source_flags,
    input  wire [15:0]          pkt_checksum,

    output reg  [31:0]          rng_t_us,
    output reg  [15:0]          rng_seq,
    output reg                  rng_valid,
    output reg  [7:0]           rng_status,
    output reg  [PAYLOAD_W-1:0] rng_payload,
    output reg  [15:0]          rng_age_ms,

    output reg  [7:0]           bridge_last_type,
    output reg  [15:0]          bridge_last_seq,
    output reg                  bridge_heartbeat_seen,
    output reg  [15:0]          bridge_heartbeat_seq,
    output reg  [15:0]          bridge_heartbeat_age_ms,
    output reg  [15:0]          bridge_checksum_fault_count,
    output reg  [15:0]          bridge_unsupported_count
);
    localparam [15:0] SRC_DEFAULT_BRIDGE =
        (16'd1 << `EXT_SRC_TEENSY_BRIDGE_BIT);

    assign pkt_ready = enable && !rst;

    function [15:0] inc_sat16;
        input [15:0] v;
        begin
            inc_sat16 = (v == 16'hFFFF) ? 16'hFFFF : (v + 16'd1);
        end
    endfunction

    function [15:0] checksum16;
        input [7:0]           typ;
        input [7:0]           stat;
        input [15:0]          seq;
        input [31:0]          timestamp_us;
        input [PAYLOAD_W-1:0] payload;
        input [15:0]          aux;
        input [15:0]          source_flags;
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

    wire packet_accept_w = pkt_valid && pkt_ready;
    wire checksum_ok_w =
        pkt_checksum == checksum16(pkt_type, pkt_status, pkt_seq,
                                   pkt_timestamp_us, pkt_payload, pkt_aux,
                                   pkt_source_flags);
    wire packet_is_heartbeat_w = (pkt_type == `PKT_TEENSY_HEARTBEAT);
    wire packet_is_range_w     = (pkt_type == `PKT_TEENSY_RANGE_AGL);
    wire packet_supported_w    = packet_is_heartbeat_w || packet_is_range_w;

    wire [15:0] source_flags_w =
        (pkt_source_flags == 16'd0) ? SRC_DEFAULT_BRIDGE : pkt_source_flags;
    wire [31:0] sample_time_us_w =
        (pkt_timestamp_us == 32'd0) ? now_us : pkt_timestamp_us;
    wire [15:0] range_height_cm_w = pkt_payload[47:32];
    wire [15:0] range_conf_w      = pkt_payload[31:16];

    wire heartbeat_required_w = (REQUIRE_HEARTBEAT_FOR_DATA != 0) ? 1'b1 : 1'b0;
    wire heartbeat_fresh_w =
        !heartbeat_required_w ||
        (bridge_heartbeat_seen &&
         (bridge_heartbeat_age_ms <= HEARTBEAT_FRESH_MAX_MS));
    wire range_height_ok_w = (range_height_cm_w <= MAX_RANGE_CM);
    wire range_quality_ok_w = (range_conf_w >= MIN_RANGE_CONFIDENCE);
    wire range_ok_w =
        checksum_ok_w &&
        (pkt_status == `ST_OK) &&
        heartbeat_fresh_w &&
        range_height_ok_w &&
        range_quality_ok_w;

    wire [7:0] range_status_w =
        !checksum_ok_w         ? `ST_CONFIG_ERROR :
        (pkt_status != `ST_OK) ? pkt_status :
        !heartbeat_fresh_w     ? `ST_STALE_REJECT :
        !range_height_ok_w     ? `ST_RANGE_REJECT :
        !range_quality_ok_w    ? `ST_DATA_NOT_READY :
                                 `ST_OK;

    reg [9:0] age_us_div_r;
    wire age_ms_tick_w = tick_1us && (age_us_div_r == 10'd999);
    wire [15:0] rng_age_inc_w = inc_sat16(rng_age_ms);
    wire [15:0] heartbeat_age_inc_w = inc_sat16(bridge_heartbeat_age_ms);

    always @(posedge clk) begin
        if (rst) begin
            age_us_div_r                <= 10'd0;
            rng_t_us                    <= 32'd0;
            rng_seq                     <= 16'd0;
            rng_valid                   <= 1'b0;
            rng_status                  <= `ST_NOT_INITIALIZED;
            rng_payload                 <= {PAYLOAD_W{1'b0}};
            rng_age_ms                  <= 16'hFFFF;
            bridge_last_type            <= 8'd0;
            bridge_last_seq             <= 16'd0;
            bridge_heartbeat_seen       <= 1'b0;
            bridge_heartbeat_seq        <= 16'd0;
            bridge_heartbeat_age_ms     <= 16'hFFFF;
            bridge_checksum_fault_count <= 16'd0;
            bridge_unsupported_count    <= 16'd0;
        end else if (!enable) begin
            age_us_div_r            <= 10'd0;
            rng_t_us                <= 32'd0;
            rng_seq                 <= 16'd0;
            rng_valid               <= 1'b0;
            rng_status              <= `ST_NOT_INITIALIZED;
            rng_payload             <= {PAYLOAD_W{1'b0}};
            rng_age_ms              <= 16'hFFFF;
            bridge_last_type        <= 8'd0;
            bridge_last_seq         <= 16'd0;
            bridge_heartbeat_seen   <= 1'b0;
            bridge_heartbeat_seq    <= 16'd0;
            bridge_heartbeat_age_ms <= 16'hFFFF;
        end else begin
            if (tick_1us) begin
                if (age_us_div_r == 10'd999)
                    age_us_div_r <= 10'd0;
                else
                    age_us_div_r <= age_us_div_r + 10'd1;
            end

            if (packet_accept_w) begin
                bridge_last_type <= pkt_type;
                bridge_last_seq  <= pkt_seq;

                if (!checksum_ok_w) begin
                    if (bridge_checksum_fault_count != 16'hFFFF)
                        bridge_checksum_fault_count <= bridge_checksum_fault_count + 16'd1;
                end else if (!packet_supported_w) begin
                    if (bridge_unsupported_count != 16'hFFFF)
                        bridge_unsupported_count <= bridge_unsupported_count + 16'd1;
                end

                if (packet_is_heartbeat_w && checksum_ok_w) begin
                    bridge_heartbeat_seen   <= 1'b1;
                    bridge_heartbeat_seq    <= pkt_seq;
                    bridge_heartbeat_age_ms <= 16'd0;
                end

                if (packet_is_range_w) begin
                    rng_t_us    <= sample_time_us_w;
                    rng_seq     <= pkt_seq;
                    rng_valid   <= range_ok_w;
                    rng_status  <= range_status_w;
                    rng_age_ms  <= 16'd0;
                    rng_payload <= checksum_ok_w ?
                        {range_height_cm_w, range_conf_w, source_flags_w} :
                        {PAYLOAD_W{1'b0}};
                end
            end else if (age_ms_tick_w) begin
                if (bridge_heartbeat_age_ms != 16'hFFFF)
                    bridge_heartbeat_age_ms <= heartbeat_age_inc_w;

                if (rng_age_ms != 16'hFFFF) begin
                    rng_age_ms <= rng_age_inc_w;
                    if (rng_valid && (rng_age_inc_w > RANGE_FRESH_MAX_MS)) begin
                        rng_valid  <= 1'b0;
                        rng_status <= `ST_STALE_REJECT;
                    end
                end
            end
        end
    end

endmodule

`default_nettype wire
