`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// l3g4200d_i2c_job
//------------------------------------------------------------------------------
// Digilent Pmod GYRO / ST L3G4200D low-rate I2C acquisition job.
//
// Snapshot payload:
//   {Z_H, Z_L, Y_H, Y_L, X_H, X_L}
//==============================================================================
module l3g4200d_i2c_job #(
    parameter [6:0]  GYRO_ADDR7     = 7'h69,
    parameter [6:0]  GYRO_ADDR7_ALT = 7'h68,
    parameter [31:0] CMD_TIMEOUT_US = 32'd3000
)(
    input  wire        clk,
    input  wire        rst,
    input  wire [31:0] time_us,
    input  wire        epoch_10hz,
    input  wire        grant,

    output reg         cmd_valid,
    input  wire        cmd_ready,
    output reg  [6:0]  cmd_addr7,
    output reg  [7:0]  cmd_wlen,
    output reg  [7:0]  cmd_rlen,
    output reg         cmd_repstart,
    output reg  [31:0] cmd_timeout_us,

    output reg         w_valid,
    input  wire        w_ready,
    output reg  [7:0]  w_data,
    output reg         w_last,

    input  wire        r_valid,
    output reg         r_ready,
    input  wire [7:0]  r_data,
    input  wire        r_last,

    input  wire        done,
    input  wire [3:0]  done_code,
    output reg         busy,

    output reg         snap_commit,
    output reg         snap_valid_in,
    output reg  [7:0]  snap_status_in,
    output reg  [47:0] snap_payload_in,
    output reg         init_done
);
    localparam [1:0]
        OP_WHO   = 2'd0,
        OP_CTRL1 = 2'd1,
        OP_CTRL4 = 2'd2,
        OP_READ  = 2'd3;

    localparam [2:0]
        S_IDLE      = 3'd0,
        S_CMD       = 3'd1,
        S_W_STREAM  = 3'd2,
        S_WAIT_DONE = 3'd3;

    localparam [7:0]
        REG_WHO_AM_I  = 8'h0F,
        REG_CTRL1     = 8'h20,
        REG_CTRL4     = 8'h23,
        REG_OUT_X_L   = 8'h28,
        WHO_EXPECT    = 8'hD3,
        CTRL1_NORMAL  = 8'h0F,
        CTRL4_250DPS  = 8'h00,
        STAT_OK       = 8'h00,
        STAT_NOT_INIT = 8'h01;

    reg [1:0] op_r;
    reg [2:0] st_r;
    reg       pending_sample_r;
    reg [7:0] widx_r;
    reg [7:0] ridx_r;
    reg [6:0] active_addr7_r;

    reg [7:0] who_r;
    reg [7:0] xl_r;
    reg [7:0] xh_r;
    reg [7:0] yl_r;
    reg [7:0] yh_r;
    reg [7:0] zl_r;
    reg [7:0] zh_r;

    wire _unused_time_ok = time_us[0];

    function [7:0] op_reg_addr;
        input [1:0] op;
        begin
            case (op)
                OP_WHO:   op_reg_addr = REG_WHO_AM_I;
                OP_CTRL1: op_reg_addr = REG_CTRL1;
                OP_CTRL4: op_reg_addr = REG_CTRL4;
                default:  op_reg_addr = REG_OUT_X_L | 8'h80;
            endcase
        end
    endfunction

    function [7:0] op_reg_data;
        input [1:0] op;
        begin
            case (op)
                OP_CTRL1: op_reg_data = CTRL1_NORMAL;
                OP_CTRL4: op_reg_data = CTRL4_250DPS;
                default:  op_reg_data = 8'd0;
            endcase
        end
    endfunction

    always @(posedge clk) begin
        if (rst) begin
            op_r             <= OP_WHO;
            st_r             <= S_IDLE;
            pending_sample_r <= 1'b0;
            widx_r           <= 8'd0;
            ridx_r           <= 8'd0;
            active_addr7_r   <= GYRO_ADDR7;

            who_r <= 8'd0;
            xl_r  <= 8'd0;
            xh_r  <= 8'd0;
            yl_r  <= 8'd0;
            yh_r  <= 8'd0;
            zl_r  <= 8'd0;
            zh_r  <= 8'd0;

            cmd_valid      <= 1'b0;
            cmd_addr7      <= GYRO_ADDR7;
            cmd_wlen       <= 8'd0;
            cmd_rlen       <= 8'd0;
            cmd_repstart   <= 1'b0;
            cmd_timeout_us <= CMD_TIMEOUT_US;
            w_valid        <= 1'b0;
            w_data         <= 8'd0;
            w_last         <= 1'b0;
            r_ready        <= 1'b0;
            busy           <= 1'b0;

            snap_commit     <= 1'b0;
            snap_valid_in   <= 1'b0;
            snap_status_in  <= STAT_NOT_INIT;
            snap_payload_in <= 48'd0;
            init_done       <= 1'b0;
        end else begin
            snap_commit <= 1'b0;

            if (epoch_10hz)
                pending_sample_r <= 1'b1;

            case (st_r)
                S_IDLE: begin
                    busy      <= 1'b0;
                    cmd_valid <= 1'b0;
                    w_valid   <= 1'b0;
                    r_ready   <= 1'b0;

                    if (grant && (!init_done || pending_sample_r)) begin
                        busy           <= 1'b1;
                        cmd_addr7      <= active_addr7_r;
                        cmd_timeout_us <= CMD_TIMEOUT_US;
                        widx_r         <= 8'd0;
                        ridx_r         <= 8'd0;
                        op_r           <= init_done ? OP_READ : OP_WHO;
                        st_r           <= S_CMD;
                    end
                end

                S_CMD: begin
                    cmd_valid <= 1'b1;
                    if (op_r == OP_WHO) begin
                        cmd_wlen     <= 8'd1;
                        cmd_rlen     <= 8'd1;
                        cmd_repstart <= 1'b1;
                    end else if (op_r == OP_READ) begin
                        cmd_wlen     <= 8'd1;
                        cmd_rlen     <= 8'd6;
                        cmd_repstart <= 1'b1;
                    end else begin
                        cmd_wlen     <= 8'd2;
                        cmd_rlen     <= 8'd0;
                        cmd_repstart <= 1'b0;
                    end

                    if (cmd_ready) begin
                        cmd_valid <= 1'b0;
                        w_valid   <= 1'b1;
                        w_data    <= op_reg_addr(op_r);
                        w_last    <= ((op_r == OP_WHO) || (op_r == OP_READ));
                        widx_r    <= 8'd1;
                        st_r      <= S_W_STREAM;
                    end
                end

                S_W_STREAM: begin
                    if (w_valid && w_ready) begin
                        if ((op_r == OP_CTRL1) || (op_r == OP_CTRL4)) begin
                            if (widx_r == 8'd1) begin
                                w_data <= op_reg_data(op_r);
                                w_last <= 1'b1;
                                widx_r <= 8'd2;
                            end else begin
                                w_valid <= 1'b0;
                            end
                        end else begin
                            w_valid <= 1'b0;
                        end
                    end

                    if (!w_valid) begin
                        r_ready <= 1'b1;
                        st_r    <= S_WAIT_DONE;
                    end
                end

                S_WAIT_DONE: begin
                    if (r_valid) begin
                        case (ridx_r)
                            8'd0: begin
                                if (op_r == OP_WHO)
                                    who_r <= r_data;
                                else
                                    xl_r <= r_data;
                            end
                            8'd1: xh_r <= r_data;
                            8'd2: yl_r <= r_data;
                            8'd3: yh_r <= r_data;
                            8'd4: zl_r <= r_data;
                            8'd5: zh_r <= r_data;
                            default: begin end
                        endcase
                        ridx_r <= ridx_r + 8'd1;
                        if (r_last)
                            r_ready <= 1'b0;
                    end

                    if (done) begin
                        busy    <= 1'b0;
                        r_ready <= 1'b0;
                        if (done_code != 4'd0) begin
                            snap_valid_in  <= 1'b0;
                            snap_status_in <= 8'hE0 | {4'd0, done_code};
                            if (!init_done)
                                active_addr7_r <= (active_addr7_r == GYRO_ADDR7) ? GYRO_ADDR7_ALT : GYRO_ADDR7;
                            st_r <= S_IDLE;
                        end else begin
                            case (op_r)
                                OP_WHO: begin
                                    if (who_r == WHO_EXPECT) begin
                                        op_r <= OP_CTRL1;
                                        st_r <= S_CMD;
                                    end else begin
                                        snap_valid_in  <= 1'b0;
                                        snap_status_in <= 8'hE5;
                                        active_addr7_r <= (active_addr7_r == GYRO_ADDR7) ? GYRO_ADDR7_ALT : GYRO_ADDR7;
                                        st_r           <= S_IDLE;
                                    end
                                end
                                OP_CTRL1: begin
                                    op_r <= OP_CTRL4;
                                    st_r <= S_CMD;
                                end
                                OP_CTRL4: begin
                                    init_done <= 1'b1;
                                    st_r      <= S_IDLE;
                                end
                                default: begin
                                    pending_sample_r <= 1'b0;
                                    snap_commit      <= 1'b1;
                                    snap_valid_in    <= 1'b1;
                                    snap_status_in   <= STAT_OK;
                                    snap_payload_in  <= {zh_r, zl_r, yh_r, yl_r, xh_r, xl_r};
                                    st_r             <= S_IDLE;
                                end
                            endcase
                        end
                    end
                end

                default: begin
                    st_r <= S_IDLE;
                end
            endcase
        end
    end
endmodule

`default_nettype wire
