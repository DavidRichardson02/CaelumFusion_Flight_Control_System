`timescale 1ns/1ps
`default_nettype none

`include "telemetry_defs_vh.vh"

//==============================================================================
// mag1_bench_snapshot_source
//------------------------------------------------------------------------------
// Default-disabled MAG1 snapshot publisher for bench/synthetic redundant-
// magnetometer evidence. This block deliberately does not fuse, tilt-correct,
// or replace the existing MAG0 heading source. It mirrors MAG0 into a MAG1-
// shaped raw snapshot with optional signed offsets so the extension hub, VGA
// evidence page, and black-box stream can validate their contracts before a
// physical second magnetometer or Teensy bridge is connected.
//
// A build must explicitly drive enable=1 to emit synthetic evidence. When
// disabled, MAG1 remains not-initialized with age=FFFF and no valid snapshot.
//==============================================================================
module mag1_bench_snapshot_source #(
    parameter [15:0] MAG1_BENCH_OFFSET_X = 16'sd0,
    parameter [15:0] MAG1_BENCH_OFFSET_Y = 16'sd0,
    parameter [15:0] MAG1_BENCH_OFFSET_Z = 16'sd0,
    parameter [15:0] MAG_FRESH_MAX_MS    = `MAG_FRESH_MAX_MS,
    parameter [15:0] MAG1_SEQUENCE_OFFSET = 16'd0
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        enable,
    input  wire        cfg_offset_x_en,
    input  wire        cfg_offset_y_en,
    input  wire        cfg_offset_z_en,

    input  wire [31:0] mag0_t_us,
    input  wire [15:0] mag0_seq,
    input  wire        mag0_valid,
    input  wire [7:0]  mag0_status,
    input  wire [47:0] mag0_payload,
    input  wire [15:0] mag0_age_ms,

    output reg  [31:0] mag1_t_us,
    output reg  [15:0] mag1_seq,
    output reg         mag1_valid,
    output reg  [7:0]  mag1_status,
    output reg  [47:0] mag1_payload,
    output reg  [15:0] mag1_age_ms,
    output reg  [7:0]  mag1_cal_state,
    output reg  [7:0]  mag1_source_flags,
    output reg  [15:0] mag1_bridge_checksum
);
    localparam [7:0] CAL_STATE_SYNTH_UNCAL = 8'h80;
    localparam [7:0] SRC_SYNTHETIC         = (8'd1 << `EXT_SRC_SYNTHETIC_BIT);

    wire signed [15:0] mag0_x_w = mag0_payload[15:0];
    wire signed [15:0] mag0_y_w = mag0_payload[31:16];
    wire signed [15:0] mag0_z_w = mag0_payload[47:32];
    wire signed [15:0] off_x_w  = cfg_offset_x_en ? MAG1_BENCH_OFFSET_X : 16'sd0;
    wire signed [15:0] off_y_w  = cfg_offset_y_en ? MAG1_BENCH_OFFSET_Y : 16'sd0;
    wire signed [15:0] off_z_w  = cfg_offset_z_en ? MAG1_BENCH_OFFSET_Z : 16'sd0;

    function signed [15:0] sat_add_s16;
        input signed [15:0] a;
        input signed [15:0] b;
        reg signed [16:0] s;
        begin
            s = {a[15], a} + {b[15], b};
            if (s > 17'sd32767)
                sat_add_s16 = 16'sh7FFF;
            else if (s < -17'sd32768)
                sat_add_s16 = 16'sh8000;
            else
                sat_add_s16 = s[15:0];
        end
    endfunction

    wire signed [15:0] mag1_x_w = sat_add_s16(mag0_x_w, off_x_w);
    wire signed [15:0] mag1_y_w = sat_add_s16(mag0_y_w, off_y_w);
    wire signed [15:0] mag1_z_w = sat_add_s16(mag0_z_w, off_z_w);
    wire [47:0] mag1_payload_w = {mag1_z_w[15:0], mag1_y_w[15:0], mag1_x_w[15:0]};
    wire [15:0] mag1_seq_w = mag0_seq + MAG1_SEQUENCE_OFFSET;
    wire [7:0] mag1_status_w =
        (!mag0_valid) ? `ST_MISSING_INPUT :
        (mag0_status != `ST_OK) ? mag0_status :
        (mag0_age_ms > MAG_FRESH_MAX_MS) ? `ST_STALE_REJECT :
        `ST_OK;
    wire mag1_valid_w = mag0_valid;
    wire [15:0] mag1_checksum_w =
        mag1_payload_w[15:0] ^
        mag1_payload_w[31:16] ^
        mag1_payload_w[47:32] ^
        mag1_seq_w ^
        {mag1_status_w, SRC_SYNTHETIC};

    always @(posedge clk) begin
        if (rst) begin
            mag1_t_us            <= 32'd0;
            mag1_seq             <= 16'd0;
            mag1_valid           <= 1'b0;
            mag1_status          <= `ST_NOT_INITIALIZED;
            mag1_payload         <= 48'd0;
            mag1_age_ms          <= 16'hFFFF;
            mag1_cal_state       <= 8'd0;
            mag1_source_flags    <= 8'd0;
            mag1_bridge_checksum <= 16'd0;
        end else if (!enable) begin
            mag1_t_us            <= 32'd0;
            mag1_seq             <= 16'd0;
            mag1_valid           <= 1'b0;
            mag1_status          <= `ST_NOT_INITIALIZED;
            mag1_payload         <= 48'd0;
            mag1_age_ms          <= 16'hFFFF;
            mag1_cal_state       <= 8'd0;
            mag1_source_flags    <= 8'd0;
            mag1_bridge_checksum <= 16'd0;
        end else begin
            mag1_t_us            <= mag0_t_us;
            mag1_seq             <= mag1_seq_w;
            mag1_valid           <= mag1_valid_w;
            mag1_status          <= mag1_status_w;
            mag1_payload         <= mag0_valid ? mag1_payload_w : 48'd0;
            mag1_age_ms          <= mag0_valid ? mag0_age_ms : 16'hFFFF;
            mag1_cal_state       <= CAL_STATE_SYNTH_UNCAL;
            mag1_source_flags    <= SRC_SYNTHETIC;
            mag1_bridge_checksum <= mag0_valid ? mag1_checksum_w : 16'd0;
        end
    end
endmodule

`default_nettype wire
