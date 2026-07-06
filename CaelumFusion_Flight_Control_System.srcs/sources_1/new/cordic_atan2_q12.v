`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// cordic_atan2_q12
//------------------------------------------------------------------------------
// Fixed-iteration CORDIC:
//   - vectoring mode computes atan2(y, x) in Q4.12
//   - rotation mode computes sin/cos in Q1.15
//
// Notes:
//   - iterative state updates are written directly into working registers
//   - avoids stale-step nonblocking update bug
//==============================================================================
module cordic_atan2_q12 #(
    parameter integer ITER = 16,
    parameter integer PI_Q12    = 12868,
    parameter integer K_INV_Q15 = 19899
)(
    input  wire               clk,
    input  wire               rst,

    input  wire               start,
    input  wire signed [15:0] y_in,
    input  wire signed [15:0] x_in,

    output reg                busy,
    output reg                done,

    output reg signed [15:0]  angle_q12,
    output reg signed [15:0]  sin_q15,
    output reg signed [15:0]  cos_q15
);

    function [15:0] atan_q12;
        input [4:0] i;
        begin
            case (i)
                5'd0:  atan_q12 = 16'd3217;
                5'd1:  atan_q12 = 16'd1899;
                5'd2:  atan_q12 = 16'd1003;
                5'd3:  atan_q12 = 16'd509;
                5'd4:  atan_q12 = 16'd256;
                5'd5:  atan_q12 = 16'd128;
                5'd6:  atan_q12 = 16'd64;
                5'd7:  atan_q12 = 16'd32;
                5'd8:  atan_q12 = 16'd16;
                5'd9:  atan_q12 = 16'd8;
                5'd10: atan_q12 = 16'd4;
                5'd11: atan_q12 = 16'd2;
                5'd12: atan_q12 = 16'd1;
                5'd13: atan_q12 = 16'd1;
                5'd14: atan_q12 = 16'd0;
                default: atan_q12 = 16'd0;
            endcase
        end
    endfunction

    function signed [15:0] sat_s16_from_s18;
        input signed [17:0] v;
        begin
            if (v > 18'sd32767)
                sat_s16_from_s18 = 16'sh7FFF;
            else if (v < -18'sd32768)
                sat_s16_from_s18 = -16'sd32768;
            else
                sat_s16_from_s18 = v[15:0];
        end
    endfunction

    localparam [2:0]
        S_IDLE    = 3'd0,
        S_VEC     = 3'd1,
        S_VEC_FIN = 3'd2,
        S_ROT     = 3'd3,
        S_DONE    = 3'd4;

    localparam signed [16:0] PI_Q12_W      = PI_Q12;
    localparam signed [16:0] HALF_PI_Q12_W = PI_Q12 / 2;

    reg [2:0] st;
    reg [4:0] iter;

    reg signed [19:0] xw, yw;
    reg signed [17:0] xs, ys;
    reg signed [15:0] zq;
    reg signed [15:0] zr;
    reg signed [15:0] zoff_q12;
    reg                rot_sin_neg;
    reg                rot_cos_neg;

    wire signed [16:0] angle_sum_q12_w =
        $signed({zq[15], zq}) + $signed({zoff_q12[15], zoff_q12});

    always @(posedge clk) begin
        if (rst) begin
            st        <= S_IDLE;
            busy      <= 1'b0;
            done      <= 1'b0;
            iter      <= 5'd0;
            xw        <= 20'sd0;
            yw        <= 20'sd0;
            xs        <= 18'sd0;
            ys        <= 18'sd0;
            zq        <= 16'sd0;
            zr        <= 16'sd0;
            zoff_q12  <= 16'sd0;
            rot_sin_neg <= 1'b0;
            rot_cos_neg <= 1'b0;
            angle_q12 <= 16'sd0;
            sin_q15   <= 16'sd0;
            cos_q15   <= 16'sd0;
        end else begin
            done <= 1'b0;

            case (st)
                S_IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        if ((x_in == 16'sd0) && (y_in == 16'sd0)) begin
                            angle_q12   <= 16'sd0;
                            sin_q15     <= 16'sd0;
                            cos_q15     <= 16'sh7FFF;
                            rot_sin_neg <= 1'b0;
                            rot_cos_neg <= 1'b0;
                            done        <= 1'b1;
                        end else if (x_in[15]) begin
                            xw       <= -{{4{x_in[15]}}, x_in};
                            yw       <= -{{4{y_in[15]}}, y_in};
                            zoff_q12 <= y_in[15] ? -$signed(PI_Q12_W[15:0]) : $signed(PI_Q12_W[15:0]);
                        end else begin
                            xw       <= {{4{x_in[15]}}, x_in};
                            yw       <= {{4{y_in[15]}}, y_in};
                            zoff_q12 <= 16'sd0;
                        end

                        if ((x_in != 16'sd0) || (y_in != 16'sd0)) begin
                            zq          <= 16'sd0;
                            iter        <= 5'd0;
                            busy        <= 1'b1;
                            rot_sin_neg <= 1'b0;
                            rot_cos_neg <= 1'b0;
                            st          <= S_VEC;
                        end
                    end
                end

                S_VEC: begin
                    if (yw[19] == 1'b0) begin
                        xw <= xw + (yw >>> iter);
                        yw <= yw - (xw >>> iter);
                        zq <= zq + $signed(atan_q12(iter));
                    end else begin
                        xw <= xw - (yw >>> iter);
                        yw <= yw + (xw >>> iter);
                        zq <= zq - $signed(atan_q12(iter));
                    end

                    if (iter == (ITER-1))
                        st <= S_VEC_FIN;
                    else
                        iter <= iter + 5'd1;
                end

                S_VEC_FIN: begin
                    angle_q12 <= angle_sum_q12_w[15:0];
                    xs        <= $signed(K_INV_Q15[15:0]);
                    ys        <= 18'sd0;
                    iter      <= 5'd0;

                    // Rotation-mode CORDIC converges around +/-pi/2. Fold
                    // quadrants II and III into that range and restore signs
                    // at S_DONE so sin/cos are valid for full atan2 output.
                    if (angle_sum_q12_w > HALF_PI_Q12_W) begin
                        zr          <= (PI_Q12_W - angle_sum_q12_w);
                        rot_sin_neg <= 1'b0;
                        rot_cos_neg <= 1'b1;
                    end else if (angle_sum_q12_w < -HALF_PI_Q12_W) begin
                        zr          <= (-PI_Q12_W - angle_sum_q12_w);
                        rot_sin_neg <= 1'b0;
                        rot_cos_neg <= 1'b1;
                    end else begin
                        zr          <= angle_sum_q12_w[15:0];
                        rot_sin_neg <= 1'b0;
                        rot_cos_neg <= 1'b0;
                    end

                    st <= S_ROT;
                end

                S_ROT: begin
                    if (zr[15] == 1'b0) begin
                        xs <= xs - (ys >>> iter);
                        ys <= ys + (xs >>> iter);
                        zr <= zr - $signed(atan_q12(iter));
                    end else begin
                        xs <= xs + (ys >>> iter);
                        ys <= ys - (xs >>> iter);
                        zr <= zr + $signed(atan_q12(iter));
                    end

                    if (iter == (ITER-1))
                        st <= S_DONE;
                    else
                        iter <= iter + 5'd1;
                end

                S_DONE: begin
                    cos_q15 <= rot_cos_neg ? -sat_s16_from_s18(xs) : sat_s16_from_s18(xs);
                    sin_q15 <= rot_sin_neg ? -sat_s16_from_s18(ys) : sat_s16_from_s18(ys);
                    busy    <= 1'b0;
                    done    <= 1'b1;
                    st      <= S_IDLE;
                end

                default: begin
                    busy <= 1'b0;
                    st   <= S_IDLE;
                end
            endcase
        end
    end

endmodule

`default_nettype wire
