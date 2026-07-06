`timescale 1ns / 1ps
`default_nettype none

`include "telemetry_defs_vh.vh"

//==============================================================================
// gnss_bridge_snapshot_source
//------------------------------------------------------------------------------
// SYS-domain decoded-packet bridge for GPS/GNSS evidence supplied by a Teensy or
// host-side parser.
//
// This block is deliberately not a UART parser and not a navigation estimator.
// A transport/parser upstream owns bytes, framing, escaping, and protocol
// details. This module accepts one coherent decoded GNSS packet per ready/valid
// handshake, verifies a deterministic 16-bit XOR checksum over the packet
// fields, tracks stale age and PPS age, and publishes a single-writer gnss_*
// snapshot bank for later EKF/nav binding.
//
// Packet checksum contract:
//   checksum = XOR of all 16-bit words formed from the decoded packet fields:
//              seq, {status,fix_type}, {num_sats,8'h00}, hdop, lat hi/lo,
//              lon hi/lo, alt hi/lo, vel N/E/D, ground speed, course hi/lo,
//              source_flags.
//
// Invalid checksums never publish position/velocity as valid evidence. They
// still commit a status snapshot with ST_CONFIG_ERROR, the observed checksum,
// and a saturating checksum-fault counter so the bridge fault is visible.
//==============================================================================
module gnss_bridge_snapshot_source #(
    parameter [15:0] GNSS_FRESH_MAX_MS      = 16'd1000,
    parameter [15:0] PPS_FRESH_MAX_MS       = 16'd1200,
    parameter [7:0]  MIN_FIX_TYPE           = 8'd3,
    parameter [7:0]  MIN_NUM_SATS           = 8'd4,
    parameter [15:0] MAX_HDOP_CENTI         = 16'd500,
    parameter integer REQUIRE_PPS_FOR_VALID = 0
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        enable,
    input  wire [31:0] now_us,
    input  wire        tick_1us,

    input  wire        pkt_valid,
    output wire        pkt_ready,
    input  wire [15:0] pkt_seq,
    input  wire [7:0]  pkt_status,
    input  wire [7:0]  pkt_fix_type,
    input  wire [7:0]  pkt_num_sats,
    input  wire [15:0] pkt_hdop_centi,
    input  wire [31:0] pkt_lat_e7,
    input  wire [31:0] pkt_lon_e7,
    input  wire [31:0] pkt_alt_cm_msl,
    input  wire [15:0] pkt_vel_n_cms,
    input  wire [15:0] pkt_vel_e_cms,
    input  wire [15:0] pkt_vel_d_cms,
    input  wire [15:0] pkt_ground_speed_cms,
    input  wire [31:0] pkt_course_mdeg,
    input  wire [15:0] pkt_source_flags,
    input  wire [15:0] pkt_checksum,

    input  wire        pps_pulse,

    output reg  [31:0] gnss_t_us,
    output reg  [15:0] gnss_seq,
    output reg         gnss_valid,
    output reg  [7:0]  gnss_status,
    output reg  [15:0] gnss_age_ms,
    output reg  [7:0]  gnss_fix_type,
    output reg  [7:0]  gnss_num_sats,
    output reg  [15:0] gnss_hdop_centi,
    output reg  [31:0] gnss_lat_e7,
    output reg  [31:0] gnss_lon_e7,
    output reg  [31:0] gnss_alt_cm_msl,
    output reg  [15:0] gnss_vel_n_cms,
    output reg  [15:0] gnss_vel_e_cms,
    output reg  [15:0] gnss_vel_d_cms,
    output reg  [15:0] gnss_ground_speed_cms,
    output reg  [31:0] gnss_course_mdeg,
    output reg         gnss_pps_seen,
    output reg  [15:0] gnss_pps_seq,
    output reg  [15:0] gnss_pps_age_ms,
    output reg  [15:0] gnss_source_flags,
    output reg  [15:0] gnss_checksum,
    output reg  [15:0] gnss_checksum_fault_count
);

    localparam [15:0] SRC_DEFAULT_BRIDGE =
        (16'd1 << `EXT_SRC_TEENSY_BRIDGE_BIT);

    assign pkt_ready = enable && !rst;

    function [15:0] checksum16;
        input [15:0] seq;
        input [7:0]  status;
        input [7:0]  fix_type;
        input [7:0]  num_sats;
        input [15:0] hdop_centi;
        input [31:0] lat_e7;
        input [31:0] lon_e7;
        input [31:0] alt_cm_msl;
        input [15:0] vel_n_cms;
        input [15:0] vel_e_cms;
        input [15:0] vel_d_cms;
        input [15:0] ground_speed_cms;
        input [31:0] course_mdeg;
        input [15:0] source_flags;
        begin
            checksum16 =
                seq ^
                {status, fix_type} ^
                {num_sats, 8'h00} ^
                hdop_centi ^
                lat_e7[31:16] ^ lat_e7[15:0] ^
                lon_e7[31:16] ^ lon_e7[15:0] ^
                alt_cm_msl[31:16] ^ alt_cm_msl[15:0] ^
                vel_n_cms ^ vel_e_cms ^ vel_d_cms ^
                ground_speed_cms ^
                course_mdeg[31:16] ^ course_mdeg[15:0] ^
                source_flags;
        end
    endfunction

    function [15:0] inc_sat16;
        input [15:0] v;
        begin
            inc_sat16 = (v == 16'hFFFF) ? 16'hFFFF : (v + 16'd1);
        end
    endfunction

    wire packet_accept_w = pkt_valid && pkt_ready;
    wire [15:0] expected_checksum_w =
        checksum16(pkt_seq, pkt_status, pkt_fix_type, pkt_num_sats,
                   pkt_hdop_centi, pkt_lat_e7, pkt_lon_e7, pkt_alt_cm_msl,
                   pkt_vel_n_cms, pkt_vel_e_cms, pkt_vel_d_cms,
                   pkt_ground_speed_cms, pkt_course_mdeg, pkt_source_flags);
    wire checksum_ok_w = (pkt_checksum == expected_checksum_w);
    wire fix_quality_ok_w =
        (pkt_fix_type >= MIN_FIX_TYPE) &&
        (pkt_num_sats >= MIN_NUM_SATS) &&
        (pkt_hdop_centi <= MAX_HDOP_CENTI);
    wire pps_required_w = (REQUIRE_PPS_FOR_VALID != 0) ? 1'b1 : 1'b0;
    wire pps_fresh_w =
        !pps_required_w ||
        (gnss_pps_seen && (gnss_pps_age_ms <= PPS_FRESH_MAX_MS));
    wire source_flags_zero_w = (pkt_source_flags == 16'd0);
    wire [15:0] source_flags_w =
        source_flags_zero_w ? SRC_DEFAULT_BRIDGE : pkt_source_flags;

    wire packet_ok_w =
        checksum_ok_w &&
        (pkt_status == `ST_OK) &&
        fix_quality_ok_w &&
        pps_fresh_w;

    wire [7:0] packet_status_w =
        !checksum_ok_w              ? `ST_CONFIG_ERROR :
        (pkt_status != `ST_OK)      ? pkt_status :
        !fix_quality_ok_w           ? `ST_DATA_NOT_READY :
        !pps_fresh_w                ? `ST_STALE_REJECT :
                                      `ST_OK;

    reg [9:0] age_us_div_r;
    wire age_ms_tick_w = tick_1us && (age_us_div_r == 10'd999);
    wire [15:0] gnss_age_inc_w = inc_sat16(gnss_age_ms);
    wire [15:0] pps_age_inc_w  = inc_sat16(gnss_pps_age_ms);

    always @(posedge clk) begin
        if (rst) begin
            age_us_div_r              <= 10'd0;
            gnss_t_us                 <= 32'd0;
            gnss_seq                  <= 16'd0;
            gnss_valid                <= 1'b0;
            gnss_status               <= `ST_NOT_INITIALIZED;
            gnss_age_ms               <= 16'hFFFF;
            gnss_fix_type             <= 8'd0;
            gnss_num_sats             <= 8'd0;
            gnss_hdop_centi           <= 16'd0;
            gnss_lat_e7               <= 32'd0;
            gnss_lon_e7               <= 32'd0;
            gnss_alt_cm_msl           <= 32'd0;
            gnss_vel_n_cms            <= 16'd0;
            gnss_vel_e_cms            <= 16'd0;
            gnss_vel_d_cms            <= 16'd0;
            gnss_ground_speed_cms     <= 16'd0;
            gnss_course_mdeg          <= 32'd0;
            gnss_pps_seen             <= 1'b0;
            gnss_pps_seq              <= 16'd0;
            gnss_pps_age_ms           <= 16'hFFFF;
            gnss_source_flags         <= 16'd0;
            gnss_checksum             <= 16'd0;
            gnss_checksum_fault_count <= 16'd0;
        end else if (!enable) begin
            age_us_div_r          <= 10'd0;
            gnss_t_us             <= 32'd0;
            gnss_seq              <= 16'd0;
            gnss_valid            <= 1'b0;
            gnss_status           <= `ST_NOT_INITIALIZED;
            gnss_age_ms           <= 16'hFFFF;
            gnss_fix_type         <= 8'd0;
            gnss_num_sats         <= 8'd0;
            gnss_hdop_centi       <= 16'd0;
            gnss_lat_e7           <= 32'd0;
            gnss_lon_e7           <= 32'd0;
            gnss_alt_cm_msl       <= 32'd0;
            gnss_vel_n_cms        <= 16'd0;
            gnss_vel_e_cms        <= 16'd0;
            gnss_vel_d_cms        <= 16'd0;
            gnss_ground_speed_cms <= 16'd0;
            gnss_course_mdeg      <= 32'd0;
            gnss_pps_seen         <= 1'b0;
            gnss_pps_seq          <= 16'd0;
            gnss_pps_age_ms       <= 16'hFFFF;
            gnss_source_flags     <= 16'd0;
            gnss_checksum         <= 16'd0;
        end else begin
            if (tick_1us) begin
                if (age_us_div_r == 10'd999)
                    age_us_div_r <= 10'd0;
                else
                    age_us_div_r <= age_us_div_r + 10'd1;
            end

            if (pps_pulse) begin
                gnss_pps_seen   <= 1'b1;
                gnss_pps_seq    <= gnss_pps_seq + 16'd1;
                gnss_pps_age_ms <= 16'd0;
            end else if (age_ms_tick_w && (gnss_pps_age_ms != 16'hFFFF)) begin
                gnss_pps_age_ms <= pps_age_inc_w;
            end

            if (packet_accept_w) begin
                gnss_t_us         <= now_us;
                gnss_seq          <= pkt_seq;
                gnss_valid        <= packet_ok_w;
                gnss_status       <= packet_status_w;
                gnss_age_ms       <= 16'd0;
                gnss_fix_type     <= pkt_fix_type;
                gnss_num_sats     <= pkt_num_sats;
                gnss_hdop_centi   <= pkt_hdop_centi;
                gnss_source_flags <= source_flags_w;
                gnss_checksum     <= pkt_checksum;

                if (packet_ok_w) begin
                    gnss_lat_e7           <= pkt_lat_e7;
                    gnss_lon_e7           <= pkt_lon_e7;
                    gnss_alt_cm_msl       <= pkt_alt_cm_msl;
                    gnss_vel_n_cms        <= pkt_vel_n_cms;
                    gnss_vel_e_cms        <= pkt_vel_e_cms;
                    gnss_vel_d_cms        <= pkt_vel_d_cms;
                    gnss_ground_speed_cms <= pkt_ground_speed_cms;
                    gnss_course_mdeg      <= pkt_course_mdeg;
                end else begin
                    gnss_lat_e7           <= 32'd0;
                    gnss_lon_e7           <= 32'd0;
                    gnss_alt_cm_msl       <= 32'd0;
                    gnss_vel_n_cms        <= 16'd0;
                    gnss_vel_e_cms        <= 16'd0;
                    gnss_vel_d_cms        <= 16'd0;
                    gnss_ground_speed_cms <= 16'd0;
                    gnss_course_mdeg      <= 32'd0;
                end

                if (!checksum_ok_w && (gnss_checksum_fault_count != 16'hFFFF))
                    gnss_checksum_fault_count <= gnss_checksum_fault_count + 16'd1;
            end else if (age_ms_tick_w && (gnss_age_ms != 16'hFFFF)) begin
                gnss_age_ms <= gnss_age_inc_w;
                if (gnss_valid && (gnss_age_inc_w > GNSS_FRESH_MAX_MS)) begin
                    gnss_valid  <= 1'b0;
                    gnss_status <= `ST_STALE_REJECT;
                end
            end
        end
    end

endmodule

`default_nettype wire
