`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// flight_attitude_math_sys
//------------------------------------------------------------------------------
// SYS-domain attitude math for the visualization / derived-state path.
//
// Computed quantities:
//   roll    = atan2(acc_y, acc_z)
//   heading = atan2(mag_y, mag_x), wrapped to [0, 2*pi)
//
// Payload contracts used by the current project:
//   ADXL362 / ACL2 accelerometer: {AX[15:0], AY[15:0], AZ[15:0]}
//   MMC3416 / CMPS2 magnetometer: {MZ[15:0], MY[15:0], MX[15:0]}
//
// Sequencing contract:
//   - A valid, ST_OK, nonzero-vector sample is queued when its source sequence
//     changes.
//   - One shared CORDIC services roll first, then heading. At 100 Hz / 10 Hz
//     source rates this has ample margin and avoids duplicating the CORDIC.
//   - roll_update/head_update are asserted one cycle after the corresponding
//     result registers have been updated, so downstream snapshot banks can use
//     the pulse as a coherent commit event.
//==============================================================================
module flight_attitude_math_sys #(
    parameter integer PI_Q12              = 12868,
    parameter integer TWO_PI_Q12          = 25736,

    // Set to 1 for the current MMC3416/LIS2MDL payload order {Z,Y,X};
    // set to 0 for a conventional {X,Y,Z} magnetometer payload.
    parameter integer MAG_PAYLOAD_ZYX     = 1,

    // Compile-time mounting/sign corrections. Keep defaults at the raw sensor
    // convention until physical board orientation is intentionally calibrated.
    parameter integer ACC_ROLL_Y_SIGN     = 1,
    parameter integer ACC_ROLL_X_SIGN     = 1,
    parameter integer MAG_HEAD_Y_SIGN     = 1,
    parameter integer MAG_HEAD_X_SIGN     = 1
)(
    input  wire         sys_clk,
    input  wire         sys_rst,

    input  wire [15:0]  acc_seq,
    input  wire         acc_valid,
    input  wire [7:0]   acc_status,
    input  wire [47:0]  acc_payload,

    input  wire [15:0]  mag_seq,
    input  wire         mag_valid,
    input  wire [7:0]   mag_status,
    input  wire [47:0]  mag_payload,

    output reg  signed [15:0] roll_q12,
    output reg         [15:0] head_q12_u,

    output reg  signed [15:0] roll_sin_q15,
    output reg  signed [15:0] roll_cos_q15,

    output reg  signed [15:0] head_sin_q15,
    output reg  signed [15:0] head_cos_q15,

    output reg                 roll_valid,
    output reg                 head_valid,
    output reg                 roll_update,
    output reg                 head_update,
    output reg          [15:0] roll_seq_ref,
    output reg          [15:0] head_seq_ref,
    output reg          [31:0] roll_mdeg,
    output reg          [31:0] heading_mdeg,
    output wire                busy
);

    localparam [1:0]
        JOB_NONE = 2'd0,
        JOB_ROLL = 2'd1,
        JOB_HEAD = 2'd2;

    localparam [31:0] FULL_TURN_MDEG = 32'd360000;

    function signed [15:0] apply_sign16;
        input signed [15:0] value;
        input integer       sign_sel;
        begin
            if (sign_sel < 0)
                apply_sign16 = -value;
            else
                apply_sign16 = value;
        end
    endfunction

    function signed [31:0] q12_signed_to_mdeg;
        input signed [15:0] q12_val;
        reg signed [31:0] ext;
        begin
            // Q4.12 radians -> millidegrees. 14 mdeg/count is within about
            // 0.16 degree at pi and avoids a DSP in the always-live SYS path.
            ext = {{16{q12_val[15]}}, q12_val};
            q12_signed_to_mdeg = (ext <<< 4) - (ext <<< 1);
        end
    endfunction

    function [31:0] q12_unsigned_to_mdeg;
        input [15:0] q12_val;
        reg [31:0] ext;
        reg [31:0] approx;
        begin
            ext    = {16'd0, q12_val};
            approx = (ext << 4) - (ext << 1);

            if (approx >= FULL_TURN_MDEG)
                q12_unsigned_to_mdeg = approx - FULL_TURN_MDEG;
            else
                q12_unsigned_to_mdeg = approx;
        end
    endfunction

    //--------------------------------------------------------------------------
    // Payload decode. The raw X/Z wires are kept explicit to make the payload
    // order visible at this boundary.
    //--------------------------------------------------------------------------
    wire signed [15:0] acc_ax_raw = acc_payload[47:32];
    wire signed [15:0] acc_ay_raw = acc_payload[31:16];
    wire signed [15:0] acc_az_raw = acc_payload[15:0];

    wire signed [15:0] mag_x_raw =
        (MAG_PAYLOAD_ZYX != 0) ? mag_payload[15:0]  : mag_payload[47:32];
    wire signed [15:0] mag_y_raw = mag_payload[31:16];
    wire signed [15:0] mag_z_raw =
        (MAG_PAYLOAD_ZYX != 0) ? mag_payload[47:32] : mag_payload[15:0];

    wire signed [15:0] roll_y_w = apply_sign16(acc_ay_raw, ACC_ROLL_Y_SIGN);
    wire signed [15:0] roll_x_w = apply_sign16(acc_az_raw, ACC_ROLL_X_SIGN);
    wire signed [15:0] head_y_w = apply_sign16(mag_y_raw, MAG_HEAD_Y_SIGN);
    wire signed [15:0] head_x_w = apply_sign16(mag_x_raw, MAG_HEAD_X_SIGN);

    wire acc_good_w = acc_valid && (acc_status == 8'h00);
    wire mag_good_w = mag_valid && (mag_status == 8'h00);

    wire acc_vec_nonzero_w = (roll_y_w != 16'sd0) || (roll_x_w != 16'sd0);
    wire mag_vec_nonzero_w = (head_y_w != 16'sd0) || (head_x_w != 16'sd0);

    //--------------------------------------------------------------------------
    // Pending sample queues. Each queue stores the newest sequence-qualified
    // sample not yet launched into the shared CORDIC.
    //--------------------------------------------------------------------------
    reg        acc_seq_seen_valid;
    reg [15:0] acc_seq_seen;
    reg        mag_seq_seen_valid;
    reg [15:0] mag_seq_seen;

    reg        roll_pending;
    reg [15:0] roll_pending_seq;
    reg signed [15:0] roll_pending_y;
    reg signed [15:0] roll_pending_x;

    reg        head_pending;
    reg [15:0] head_pending_seq;
    reg signed [15:0] head_pending_y;
    reg signed [15:0] head_pending_x;

    wire acc_new_good_w = acc_good_w &&
                          (!acc_seq_seen_valid || (acc_seq != acc_seq_seen));
    wire mag_new_good_w = mag_good_w &&
                          (!mag_seq_seen_valid || (mag_seq != mag_seq_seen));

    //--------------------------------------------------------------------------
    // Shared CORDIC engine.
    //--------------------------------------------------------------------------
    reg                cordic_start;
    reg signed [15:0] cordic_y_in;
    reg signed [15:0] cordic_x_in;
    wire               cordic_busy;
    wire               cordic_done;
    wire signed [15:0] cordic_angle_q12;
    wire signed [15:0] cordic_sin_q15;
    wire signed [15:0] cordic_cos_q15;

    reg [1:0]  active_job;
    reg [15:0] active_seq;

    reg roll_update_next;
    reg head_update_next;

    cordic_atan2_q12 #(
        .PI_Q12(PI_Q12)
    ) u_cordic_atan2_q12 (
        .clk       (sys_clk),
        .rst       (sys_rst),
        .start     (cordic_start),
        .y_in      (cordic_y_in),
        .x_in      (cordic_x_in),
        .busy      (cordic_busy),
        .done      (cordic_done),
        .angle_q12 (cordic_angle_q12),
        .sin_q15   (cordic_sin_q15),
        .cos_q15   (cordic_cos_q15)
    );

    wire [15:0] head_wrapped_q12;
    angle_wrap_0_2pi #(
        .PI_Q12    (PI_Q12),
        .TWO_PI_Q12(TWO_PI_Q12)
    ) u_head_wrap_0_2pi (
        .ang_in_q12 (cordic_angle_q12),
        .ang_out_q12(head_wrapped_q12)
    );

    assign busy = cordic_busy |
                  cordic_start |
                  roll_pending |
                  head_pending |
                  (active_job != JOB_NONE);

    always @(posedge sys_clk) begin
        if (sys_rst) begin
            acc_seq_seen_valid <= 1'b0;
            acc_seq_seen       <= 16'd0;
            mag_seq_seen_valid <= 1'b0;
            mag_seq_seen       <= 16'd0;

            roll_pending       <= 1'b0;
            roll_pending_seq   <= 16'd0;
            roll_pending_y     <= 16'sd0;
            roll_pending_x     <= 16'sd0;

            head_pending       <= 1'b0;
            head_pending_seq   <= 16'd0;
            head_pending_y     <= 16'sd0;
            head_pending_x     <= 16'sd0;

            cordic_start       <= 1'b0;
            cordic_y_in        <= 16'sd0;
            cordic_x_in        <= 16'sd0;
            active_job         <= JOB_NONE;
            active_seq         <= 16'd0;

            roll_update_next   <= 1'b0;
            head_update_next   <= 1'b0;

            roll_q12           <= 16'sd0;
            head_q12_u         <= 16'd0;
            roll_sin_q15       <= 16'sd0;
            roll_cos_q15       <= 16'sd0;
            head_sin_q15       <= 16'sd0;
            head_cos_q15       <= 16'sd0;

            roll_valid         <= 1'b0;
            head_valid         <= 1'b0;
            roll_update        <= 1'b0;
            head_update        <= 1'b0;
            roll_seq_ref       <= 16'd0;
            head_seq_ref       <= 16'd0;
            roll_mdeg          <= 32'd0;
            heading_mdeg       <= 32'd0;
        end else begin
            cordic_start     <= 1'b0;
            roll_update      <= roll_update_next;
            head_update      <= head_update_next;
            roll_update_next <= 1'b0;
            head_update_next <= 1'b0;

            if (cordic_done) begin
                if (active_job == JOB_ROLL) begin
                    roll_q12         <= cordic_angle_q12;
                    roll_sin_q15     <= cordic_sin_q15;
                    roll_cos_q15     <= cordic_cos_q15;
                    roll_seq_ref     <= active_seq;
                    roll_mdeg        <= q12_signed_to_mdeg(cordic_angle_q12);
                    roll_valid       <= 1'b1;
                    roll_update_next <= 1'b1;
                end else if (active_job == JOB_HEAD) begin
                    head_q12_u       <= head_wrapped_q12;
                    head_sin_q15     <= cordic_sin_q15;
                    head_cos_q15     <= cordic_cos_q15;
                    head_seq_ref     <= active_seq;
                    heading_mdeg     <= q12_unsigned_to_mdeg(head_wrapped_q12);
                    head_valid       <= 1'b1;
                    head_update_next <= 1'b1;
                end

                active_job <= JOB_NONE;
                active_seq <= 16'd0;
            end

            if ((active_job == JOB_NONE) && !cordic_busy && !cordic_start) begin
                if (roll_pending) begin
                    cordic_y_in      <= roll_pending_y;
                    cordic_x_in      <= roll_pending_x;
                    cordic_start     <= 1'b1;
                    active_job       <= JOB_ROLL;
                    active_seq       <= roll_pending_seq;
                    roll_pending     <= 1'b0;
                end else if (head_pending) begin
                    cordic_y_in      <= head_pending_y;
                    cordic_x_in      <= head_pending_x;
                    cordic_start     <= 1'b1;
                    active_job       <= JOB_HEAD;
                    active_seq       <= head_pending_seq;
                    head_pending     <= 1'b0;
                end
            end

            if (acc_new_good_w) begin
                acc_seq_seen_valid <= 1'b1;
                acc_seq_seen       <= acc_seq;

                if (acc_vec_nonzero_w) begin
                    roll_pending     <= 1'b1;
                    roll_pending_seq <= acc_seq;
                    roll_pending_y   <= roll_y_w;
                    roll_pending_x   <= roll_x_w;
                end
            end

            if (mag_new_good_w) begin
                mag_seq_seen_valid <= 1'b1;
                mag_seq_seen       <= mag_seq;

                if (mag_vec_nonzero_w) begin
                    head_pending     <= 1'b1;
                    head_pending_seq <= mag_seq;
                    head_pending_y   <= head_y_w;
                    head_pending_x   <= head_x_w;
                end
            end
        end
    end

    // Keep raw axes visible to synthesis/lint when only roll/head axes are used.
    wire _unused_axis_ok = &{1'b0, acc_ax_raw[0], mag_z_raw[0]};
endmodule

`default_nettype wire
