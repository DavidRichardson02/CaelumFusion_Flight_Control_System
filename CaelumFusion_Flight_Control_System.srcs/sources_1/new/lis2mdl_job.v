`timescale 1ns/1ps
`default_nettype none
`ifndef CAELUM_LIS2MDL_JOB_V
`define CAELUM_LIS2MDL_JOB_V

`include "telemetry_defs_vh.vh"

//==============================================================================
// lis2mdl_job
//------------------------------------------------------------------------------
// ROLE
//   Deterministic bring-up / periodic acquisition FSM for LIS2MDL.
//
// DATASHEET-ALIGNED CHOICES
//   - WHO_AM_I register: 0x4F
//   - WHO_AM_I expected value: 0x40
//   - CFG_REG_A: 0x60
//   - CFG_REG_C: 0x62
//   - Output data starts at OUTX_L_REG = 0x68
//
// INIT SEQUENCE
//   1) Read WHO_AM_I, expect 0x40
//   2) Write CFG_REG_A = 0x80
//        COMP_TEMP_EN = 1
//        LP = 0
//        ODR = 10 Hz
//        MD = continuous mode
//   3) Write CFG_REG_C = 0x01
//        DRDY-on-INT enable (per uploaded startup guidance)
//
// PERIODIC READ
//   Repeated-start burst from 0x68 across:
//     OUTX_L, OUTX_H, OUTY_L, OUTY_H, OUTZ_L, OUTZ_H
//
// SNAPSHOT PAYLOAD
//   {Z_H, Z_L, Y_H, Y_L, X_H, X_L}
//==============================================================================
module lis2mdl_job #(
    parameter [6:0]  LIS2MDL_ADDR7  = 7'h1E,
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
        OP_CFGA  = 2'd1,
        OP_CFGC  = 2'd2,
        OP_READ  = 2'd3;

    localparam [2:0]
        ST_IDLE      = 3'd0,
        ST_CMD       = 3'd1,
        ST_W_STREAM  = 3'd2,
        ST_WAIT_DONE = 3'd3;

    reg [1:0] op_r;
    reg [2:0] st_r;

    reg       pending_sample_r;
    reg       cmd_armed_r;
    reg [7:0] widx_r;
    reg [7:0] ridx_r;

    reg [7:0] who_r;
    reg [7:0] xl_r;
    reg [7:0] xh_r;
    reg [7:0] yl_r;
    reg [7:0] yh_r;
    reg [7:0] zl_r;
    reg [7:0] zh_r;

    function [7:0] status_from_done_code;
        input [3:0] code;
        begin
            case (code)
                4'd1,
                4'd2,
                4'd5: status_from_done_code = `ST_I2C_NACK;

                4'd3,
                4'd6: status_from_done_code = `ST_I2C_TIMEOUT;

                4'd4: status_from_done_code = `ST_INTERNAL_OVERFLOW;

                default: status_from_done_code = `ST_CONFIG_ERROR;
            endcase
        end
    endfunction

    always @(posedge clk) begin
        if (rst) begin
            op_r <= OP_WHO;
            st_r <= ST_IDLE;

            pending_sample_r <= 1'b0;
            cmd_armed_r      <= 1'b0;

            cmd_valid      <= 1'b0;
            cmd_addr7      <= LIS2MDL_ADDR7;
            cmd_wlen       <= 8'd0;
            cmd_rlen       <= 8'd0;
            cmd_repstart   <= 1'b0;
            cmd_timeout_us <= CMD_TIMEOUT_US;

            w_valid <= 1'b0;
            w_data  <= 8'd0;
            w_last  <= 1'b0;

            r_ready <= 1'b0;
            busy    <= 1'b0;

            snap_commit     <= 1'b0;
            snap_valid_in   <= 1'b0;
            snap_status_in  <= `ST_NOT_INITIALIZED;
            snap_payload_in <= 48'd0;

            init_done <= 1'b0;

            widx_r <= 8'd0;
            ridx_r <= 8'd0;

            who_r <= 8'd0;
            xl_r  <= 8'd0;
            xh_r  <= 8'd0;
            yl_r  <= 8'd0;
            yh_r  <= 8'd0;
            zl_r  <= 8'd0;
            zh_r  <= 8'd0;
        end else begin
            snap_commit <= 1'b0;

            if (epoch_10hz)
                pending_sample_r <= 1'b1;

            case (st_r)
                ST_IDLE: begin
                    busy      <= 1'b0;
                    cmd_valid <= 1'b0;
                    w_valid   <= 1'b0;
                    r_ready   <= 1'b0;

                    if (grant && (!init_done || pending_sample_r)) begin
                        busy           <= 1'b1;
                        cmd_addr7      <= LIS2MDL_ADDR7;
                        cmd_timeout_us <= CMD_TIMEOUT_US;
                        cmd_armed_r    <= 1'b0;
                        widx_r         <= 8'd0;
                        ridx_r         <= 8'd0;

                        if (!init_done)
                            op_r <= OP_WHO;
                        else
                            op_r <= OP_READ;

                        st_r <= ST_CMD;
                    end
                end

                ST_CMD: begin
                    busy      <= 1'b1;
                    cmd_valid <= 1'b1;

                    case (op_r)
                        OP_WHO: begin
                            cmd_wlen     <= 8'd1;
                            cmd_rlen     <= 8'd1;
                            cmd_repstart <= 1'b1;
                        end

                        OP_CFGA,
                        OP_CFGC: begin
                            cmd_wlen     <= 8'd2;
                            cmd_rlen     <= 8'd0;
                            cmd_repstart <= 1'b0;
                        end

                        default: begin
                            cmd_wlen     <= 8'd1;
                            cmd_rlen     <= 8'd6;
                            cmd_repstart <= 1'b1;
                        end
                    endcase

                    if (!cmd_armed_r) begin
                        cmd_armed_r <= 1'b1;
                    end else if (cmd_ready) begin
                        cmd_valid   <= 1'b0;
                        cmd_armed_r <= 1'b0;
                        w_valid     <= 1'b1;

                        case (op_r)
                            OP_WHO: begin
                                w_data <= 8'h4F;
                                w_last <= 1'b1;
                                widx_r <= 8'd0;
                            end

                            OP_CFGA: begin
                                w_data <= 8'h60;
                                w_last <= 1'b0;
                                widx_r <= 8'd1;
                            end

                            OP_CFGC: begin
                                w_data <= 8'h62;
                                w_last <= 1'b0;
                                widx_r <= 8'd1;
                            end

                            default: begin
                                w_data <= 8'h68;
                                w_last <= 1'b1;
                                widx_r <= 8'd0;
                            end
                        endcase

                        st_r      <= ST_W_STREAM;
                    end
                end

                ST_W_STREAM: begin
                    busy <= 1'b1;

                    if (w_valid && w_ready) begin
                        case (op_r)
                            OP_WHO: begin
                                w_valid <= 1'b0;
                            end

                            OP_CFGA: begin
                                if (widx_r == 8'd1) begin
                                    w_data  <= 8'h80;
                                    w_last  <= 1'b1;
                                    widx_r  <= 8'd2;
                                end else begin
                                    w_valid <= 1'b0;
                                end
                            end

                            OP_CFGC: begin
                                if (widx_r == 8'd1) begin
                                    w_data  <= 8'h01;
                                    w_last  <= 1'b1;
                                    widx_r  <= 8'd2;
                                end else begin
                                    w_valid <= 1'b0;
                                end
                            end

                            default: begin
                                w_valid <= 1'b0;
                            end
                        endcase
                    end

                    if (!w_valid) begin
                        r_ready <= 1'b1;
                        st_r    <= ST_WAIT_DONE;
                    end
                end

                ST_WAIT_DONE: begin
                    busy <= 1'b1;

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
                        r_ready <= 1'b0;

                        if (done_code != 4'h0) begin
                            busy          <= 1'b0;
                            snap_valid_in  <= 1'b0;
                            snap_status_in <= status_from_done_code(done_code);
                            st_r           <= ST_IDLE;
                        end else begin
                            case (op_r)
                                OP_WHO: begin
                                    if (who_r == 8'h40) begin
                                        op_r        <= OP_CFGA;
                                        cmd_armed_r <= 1'b0;
                                        st_r        <= ST_CMD;
                                    end else begin
                                        busy          <= 1'b0;
                                        snap_valid_in  <= 1'b0;
                                        snap_status_in <= `ST_SENSOR_ID_MISMATCH;
                                        st_r           <= ST_IDLE;
                                    end
                                end

                                OP_CFGA: begin
                                    op_r        <= OP_CFGC;
                                    cmd_armed_r <= 1'b0;
                                    st_r        <= ST_CMD;
                                end

                                OP_CFGC: begin
                                    busy      <= 1'b0;
                                    init_done <= 1'b1;
                                    st_r      <= ST_IDLE;
                                end

                                default: begin
                                    busy             <= 1'b0;
                                    pending_sample_r <= 1'b0;
                                    snap_commit      <= 1'b1;
                                    snap_valid_in    <= 1'b1;
                                    snap_status_in   <= `ST_OK;
                                    snap_payload_in  <= {zh_r, zl_r, yh_r, yl_r, xh_r, xl_r};
                                    st_r             <= ST_IDLE;
                                end
                            endcase
                        end
                    end
                end

                default: begin
                    st_r <= ST_IDLE;
                end
            endcase
        end
    end
endmodule

`endif


`default_nettype wire
