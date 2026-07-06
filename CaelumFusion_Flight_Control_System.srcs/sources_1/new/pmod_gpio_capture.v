`timescale 1ns/1ps
`default_nettype none

`include "telemetry_defs_vh.vh"

//==============================================================================
// pmod_gpio_capture
//------------------------------------------------------------------------------
// Synchronous capture for slow single-bit Pmod evidence sources:
//   - Pmod LS1 infrared detector channels S1..S4
//   - Pmod PIR motion detector output
//
// Each raw board input is independently synchronized before any edge detection.
// The published snapshot uses the existing sun/light extension lane. The LS1
// channels form a coarse luma estimate, while the lower payload words preserve
// event flags and a saturating event counter for black-box review.
//
// Snapshot payload:
//   [47:32] coarse LS1 luma estimate
//   [31:16] flags: {7'd0, pir_level, ls1_level[3:0], pir_rise, pir_fall,
//                   any_ls1_rise, any_ls1_fall}
//   [15:0]  saturating GPIO event count
//==============================================================================
module pmod_gpio_capture #(
    parameter [15:0] SAMPLE_MAX_AGE_MS = 16'd500
)(
    input  wire        clk,
    input  wire        rst,
    input  wire [31:0] time_us,
    input  wire        tick_1us,
    input  wire        sample_event,
    input  wire [3:0]  ls1_s_raw,
    input  wire        pir_motion_raw,

    output wire [3:0]  ls1_s_level,
    output wire        pir_motion_level,

    output reg  [31:0] sun_t_us,
    output reg  [15:0] sun_seq,
    output reg         sun_valid,
    output reg  [7:0]  sun_status,
    output reg  [47:0] sun_payload,
    output reg  [15:0] sun_age_ms
);
    wire [3:0] ls1_rise;
    wire [3:0] ls1_fall;
    wire [3:0] ls1_toggle;
    wire       pir_rise;
    wire       pir_fall;
    wire       pir_toggle;

    sync_bit_3ff u_ls1_s0_sync (
        .clk(clk), .rst(rst), .async_in(ls1_s_raw[0]),
        .sync_level(ls1_s_level[0]),
        .rise_pulse(ls1_rise[0]),
        .fall_pulse(ls1_fall[0]),
        .toggle_pulse(ls1_toggle[0])
    );

    sync_bit_3ff u_ls1_s1_sync (
        .clk(clk), .rst(rst), .async_in(ls1_s_raw[1]),
        .sync_level(ls1_s_level[1]),
        .rise_pulse(ls1_rise[1]),
        .fall_pulse(ls1_fall[1]),
        .toggle_pulse(ls1_toggle[1])
    );

    sync_bit_3ff u_ls1_s2_sync (
        .clk(clk), .rst(rst), .async_in(ls1_s_raw[2]),
        .sync_level(ls1_s_level[2]),
        .rise_pulse(ls1_rise[2]),
        .fall_pulse(ls1_fall[2]),
        .toggle_pulse(ls1_toggle[2])
    );

    sync_bit_3ff u_ls1_s3_sync (
        .clk(clk), .rst(rst), .async_in(ls1_s_raw[3]),
        .sync_level(ls1_s_level[3]),
        .rise_pulse(ls1_rise[3]),
        .fall_pulse(ls1_fall[3]),
        .toggle_pulse(ls1_toggle[3])
    );

    sync_bit_3ff u_pir_motion_sync (
        .clk(clk), .rst(rst), .async_in(pir_motion_raw),
        .sync_level(pir_motion_level),
        .rise_pulse(pir_rise),
        .fall_pulse(pir_fall),
        .toggle_pulse(pir_toggle)
    );

    function [15:0] luma_from_ls1;
        input [3:0] level;
        reg [2:0] count;
        begin
            count = {2'd0, level[0]} + {2'd0, level[1]} +
                    {2'd0, level[2]} + {2'd0, level[3]};
            case (count)
                3'd0: luma_from_ls1 = 16'd0;
                3'd1: luma_from_ls1 = 16'd16384;
                3'd2: luma_from_ls1 = 16'd32768;
                3'd3: luma_from_ls1 = 16'd49152;
                default: luma_from_ls1 = 16'hFFFF;
            endcase
        end
    endfunction

    wire any_gpio_event_w = (|ls1_toggle) | pir_toggle;
    wire any_ls1_rise_w   = |ls1_rise;
    wire any_ls1_fall_w   = |ls1_fall;
    wire [15:0] flags_w = {
        7'd0,
        pir_motion_level,
        ls1_s_level,
        pir_rise,
        pir_fall,
        any_ls1_rise_w,
        any_ls1_fall_w
    };

    reg [9:0] age_us_div_r;
    reg [15:0] event_count_r;
    wire age_ms_tick_w = tick_1us && (age_us_div_r == 10'd999);
    wire commit_w = sample_event | any_gpio_event_w;

    always @(posedge clk) begin
        if (rst) begin
            age_us_div_r <= 10'd0;
            event_count_r <= 16'd0;
            sun_t_us      <= 32'd0;
            sun_seq       <= 16'd0;
            sun_valid     <= 1'b0;
            sun_status    <= `ST_NOT_INITIALIZED;
            sun_payload   <= 48'd0;
            sun_age_ms    <= 16'hFFFF;
        end else begin
            if (tick_1us) begin
                if (age_us_div_r == 10'd999)
                    age_us_div_r <= 10'd0;
                else
                    age_us_div_r <= age_us_div_r + 10'd1;
            end

            if (any_gpio_event_w && (event_count_r != 16'hFFFF))
                event_count_r <= event_count_r + 16'd1;

            if (commit_w) begin
                sun_t_us    <= time_us;
                sun_seq     <= sun_seq + 16'd1;
                sun_valid   <= 1'b1;
                sun_status  <= `ST_OK;
                sun_payload <= {luma_from_ls1(ls1_s_level), flags_w, event_count_r};
                sun_age_ms  <= 16'd0;
            end else if (age_ms_tick_w && (sun_age_ms != 16'hFFFF)) begin
                sun_age_ms <= sun_age_ms + 16'd1;
                if (sun_age_ms > SAMPLE_MAX_AGE_MS)
                    sun_status <= `ST_STALE_REJECT;
            end
        end
    end
endmodule

`default_nettype wire
