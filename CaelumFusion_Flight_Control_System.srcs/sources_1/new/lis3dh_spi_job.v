`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// lis3dh_spi_job
//------------------------------------------------------------------------------
// SPI-mode LIS3DH job FSM.
//
// WIRE PROTOCOL
//   - Uses SPI mode 3 through the shared engine.
//   - Single-byte writes use header = register address.
//   - Reads use header = 0x80 | addr.
//   - Multi-byte reads use header = 0xC0 | addr.
//
// INIT SEQUENCE
//   1) WHO_AM_I  (0x0F) == 0x33
//   2) CTRL_REG1 (0x20) = 0x57
//   3) CTRL_REG4 (0x23) = 0xA8
//
// PERIODIC READ
//   Burst from OUT_X_L (0x28) across 6 bytes.
//
// SNAPSHOT PAYLOAD
//   {AX, AY, AZ} with each axis preserved as low-byte then high-byte.
//==============================================================================
module lis3dh_spi_job #(
    parameter [31:0] CMD_TIMEOUT_US = 32'd2000
)(
    input  wire        clk,
    input  wire        rst,

    input  wire        epoch_100hz,
    input  wire        grant,

    output reg         cmd_valid,
    input  wire        cmd_ready,
    output reg  [7:0]  cmd_wlen,
    output reg  [7:0]  cmd_rlen,
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
        S_REQ    = 3'd0,
        S_WB0    = 3'd1,
        S_WB1    = 3'd2,
        S_STREAM = 3'd3,
        S_WAIT   = 3'd4,
        S_COMMIT = 3'd5,
        S_IDLE   = 3'd6;

    localparam [7:0]
        REG_WHO_AM_I = 8'h0F,
        REG_CTRL1    = 8'h20,
        REG_CTRL4    = 8'h23,
        REG_OUT_X_L  = 8'h28,
        WHO_EXPECT   = 8'h33,
        STAT_OK      = 8'h00,
        STAT_NOT_INIT= 8'h01,
        STAT_BAD_WHO = 8'hE5,
        STAT_RD_LEN  = 8'hE1;

    reg [1:0] op_r;
    reg [2:0] state_r;

    reg       pending_sample_r;
    reg [2:0] ridx_r;
    reg       rd_len_err_r;

    reg [7:0] who_r;
    reg [15:0] ax_r;
    reg [15:0] ay_r;
    reg [15:0] az_r;

    reg       commit_valid_r;
    reg [7:0] commit_status_r;
    reg [47:0] commit_payload_r;
    reg [2:0] commit_next_state_r;

    function [7:0] op_wlen;
        input [1:0] op;
        begin
            case (op)
                OP_CTRL1,
                OP_CTRL4: op_wlen = 8'd2;
                default:  op_wlen = 8'd1;
            endcase
        end
    endfunction

    function [7:0] op_rlen;
        input [1:0] op;
        begin
            case (op)
                OP_WHO:  op_rlen = 8'd1;
                OP_READ: op_rlen = 8'd6;
                default: op_rlen = 8'd0;
            endcase
        end
    endfunction

    function [7:0] op_header;
        input [1:0] op;
        begin
            case (op)
                OP_WHO:   op_header = 8'h80 | REG_WHO_AM_I;
                OP_CTRL1: op_header = REG_CTRL1;
                OP_CTRL4: op_header = REG_CTRL4;
                default:  op_header = 8'hC0 | REG_OUT_X_L;
            endcase
        end
    endfunction

    function [7:0] op_data;
        input [1:0] op;
        begin
            case (op)
                OP_CTRL1: op_data = 8'h57;
                OP_CTRL4: op_data = 8'hA8;
                default:  op_data = 8'h00;
            endcase
        end
    endfunction

    always @(posedge clk) begin
        if (rst) begin
            op_r                <= OP_WHO;
            state_r             <= S_REQ;
            pending_sample_r    <= 1'b0;
            ridx_r              <= 3'd0;
            rd_len_err_r        <= 1'b0;
            who_r               <= 8'd0;
            ax_r                <= 16'd0;
            ay_r                <= 16'd0;
            az_r                <= 16'd0;
            commit_valid_r      <= 1'b0;
            commit_status_r     <= STAT_NOT_INIT;
            commit_payload_r    <= 48'd0;
            commit_next_state_r <= S_REQ;
            init_done           <= 1'b0;
        end else begin
            if (epoch_100hz)
                pending_sample_r <= 1'b1;

            case (state_r)
                S_REQ: begin
                    if (grant && cmd_ready) begin
                        ridx_r       <= 3'd0;
                        rd_len_err_r <= 1'b0;
                        state_r      <= S_WB0;
                    end
                end

                S_WB0: begin
                    if (w_ready) begin
                        if (op_wlen(op_r) == 8'd1) begin
                            if (op_rlen(op_r) != 8'd0)
                                state_r <= S_STREAM;
                            else
                                state_r <= S_WAIT;
                        end else begin
                            state_r <= S_WB1;
                        end
                    end
                end

                S_WB1: begin
                    if (w_ready) begin
                        if (op_rlen(op_r) != 8'd0)
                            state_r <= S_STREAM;
                        else
                            state_r <= S_WAIT;
                    end
                end

                S_STREAM: begin
                    if (r_valid) begin
                        case (op_r)
                            OP_WHO: begin
                                who_r <= r_data;
                                if (ridx_r != 3'd0)
                                    rd_len_err_r <= 1'b1;
                            end

                            OP_READ: begin
                                case (ridx_r)
                                    3'd0: ax_r[7:0]   <= r_data;
                                    3'd1: ax_r[15:8]  <= r_data;
                                    3'd2: ay_r[7:0]   <= r_data;
                                    3'd3: ay_r[15:8]  <= r_data;
                                    3'd4: az_r[7:0]   <= r_data;
                                    3'd5: az_r[15:8]  <= r_data;
                                    default: rd_len_err_r <= 1'b1;
                                endcase
                            end

                            default: begin
                                rd_len_err_r <= 1'b1;
                            end
                        endcase

                        if (ridx_r < op_rlen(op_r))
                            ridx_r <= ridx_r + 3'd1;
                        else if (op_rlen(op_r) != 8'd0)
                            rd_len_err_r <= 1'b1;
                    end

                    if (done) begin
                        if ((done_code != 4'd0) ||
                            rd_len_err_r ||
                            (((r_valid ? (ridx_r + 3'd1) : ridx_r) != op_rlen(op_r)))) begin
                            commit_valid_r      <= 1'b0;
                            commit_status_r     <= (done_code != 4'd0) ? (8'hE0 | {4'd0, done_code})
                                                                        : STAT_RD_LEN;
                            commit_payload_r    <= 48'd0;
                            commit_next_state_r <= S_REQ;
                            if (!init_done)
                                op_r <= OP_WHO;
                            else
                                op_r <= OP_READ;
                            state_r <= S_COMMIT;
                        end else begin
                            case (op_r)
                                OP_WHO: begin
                                    if ((r_valid ? r_data : who_r) == WHO_EXPECT) begin
                                        op_r    <= OP_CTRL1;
                                        state_r <= S_REQ;
                                    end else begin
                                        init_done           <= 1'b0;
                                        commit_valid_r      <= 1'b0;
                                        commit_status_r     <= STAT_BAD_WHO;
                                        commit_payload_r    <= 48'd0;
                                        commit_next_state_r <= S_REQ;
                                        op_r                <= OP_WHO;
                                        state_r             <= S_COMMIT;
                                    end
                                end

                                OP_READ: begin
                                    pending_sample_r      <= 1'b0;
                                    commit_valid_r        <= 1'b1;
                                    commit_status_r       <= STAT_OK;
                                    commit_payload_r      <= {ax_r, ay_r, az_r};
                                    commit_next_state_r   <= S_IDLE;
                                    op_r                  <= OP_READ;
                                    state_r               <= S_COMMIT;
                                end

                                default: begin
                                    state_r <= S_IDLE;
                                end
                            endcase
                        end
                    end
                end

                S_WAIT: begin
                    if (done) begin
                        if (done_code != 4'd0) begin
                            init_done           <= 1'b0;
                            commit_valid_r      <= 1'b0;
                            commit_status_r     <= 8'hE0 | {4'd0, done_code};
                            commit_payload_r    <= 48'd0;
                            commit_next_state_r <= S_REQ;
                            op_r                <= OP_WHO;
                            state_r             <= S_COMMIT;
                        end else begin
                            case (op_r)
                                OP_CTRL1: begin
                                    op_r    <= OP_CTRL4;
                                    state_r <= S_REQ;
                                end

                                OP_CTRL4: begin
                                    init_done <= 1'b1;
                                    state_r   <= S_IDLE;
                                end

                                default: begin
                                    state_r <= S_IDLE;
                                end
                            endcase
                        end
                    end
                end

                S_COMMIT: begin
                    state_r <= commit_next_state_r;
                end

                S_IDLE: begin
                    if (init_done && pending_sample_r) begin
                        op_r       <= OP_READ;
                        state_r    <= S_REQ;
                    end
                end

                default: begin
                    op_r     <= OP_WHO;
                    state_r  <= S_REQ;
                    init_done<= 1'b0;
                end
            endcase
        end
    end

    always @(*) begin
        cmd_valid      = (state_r == S_REQ);
        cmd_wlen       = op_wlen(op_r);
        cmd_rlen       = op_rlen(op_r);
        cmd_timeout_us = CMD_TIMEOUT_US;

        w_valid        = 1'b0;
        w_data         = 8'd0;
        w_last         = 1'b0;

        if (state_r == S_WB0) begin
            w_valid = 1'b1;
            w_data  = op_header(op_r);
            w_last  = (op_wlen(op_r) == 8'd1);
        end else if (state_r == S_WB1) begin
            w_valid = 1'b1;
            w_data  = op_data(op_r);
            w_last  = 1'b1;
        end

        r_ready        = (state_r == S_STREAM);

        snap_commit     = (state_r == S_COMMIT);
        snap_valid_in   = commit_valid_r;
        snap_status_in  = commit_status_r;
        snap_payload_in = commit_payload_r;
    end

endmodule

`default_nettype wire
