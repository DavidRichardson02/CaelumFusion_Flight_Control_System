`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// adxl362_spi_job
//------------------------------------------------------------------------------
// ADXL362 / Digilent Pmod ACL2 acquisition job for the dedicated SPI mode-0
// engine.
//
// SPI TRANSACTIONS
//   - Read PARTID  (0x02) and require 0xF2.
//   - Write INTMAP1 (0x2A) = 0x01 to map DATA_READY to active-high INT1.
//   - Write INTMAP2 (0x2B) = 0x00 to leave INT2 unmapped.
//   - Write POWER_CTL (0x2D) = 0x02 to enter measurement mode.
//   - Wait for the first default-ODR measurement to become valid.
//   - Read XDATA_L..ZDATA_H (0x0E..0x13) at the 100 Hz epoch cadence.
//
// SNAPSHOT PAYLOAD
//   {AX, AY, AZ}, where each axis is the ADXL362 little-endian 16-bit sample
//   reconstructed as a conventional high-byte:low-byte word.
//==============================================================================
module adxl362_spi_job #(
    parameter [31:0] CMD_TIMEOUT_US  = 32'd2000,
    parameter [31:0] POWERUP_WAIT_US = 32'd5000,
    parameter [31:0] MEASURE_WAIT_US = 32'd40000
)(
    input  wire        clk,
    input  wire        rst,

    input  wire [31:0] time_us,
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

    output reg         busy,

    output reg         snap_commit,
    output reg         snap_valid_in,
    output reg  [7:0]  snap_status_in,
    output reg  [47:0] snap_payload_in,

    output reg         init_done
);

    localparam [2:0]
        OP_PARTID    = 3'd0,
        OP_INTMAP1   = 3'd1,
        OP_INTMAP2   = 3'd2,
        OP_POWER_CTL = 3'd3,
        OP_SAMPLE    = 3'd4;

    localparam [3:0]
        S_POWERUP      = 4'd0,
        S_REQ          = 4'd1,
        S_WB0          = 4'd2,
        S_WB1          = 4'd3,
        S_WB2          = 4'd4,
        S_STREAM       = 4'd5,
        S_WAIT_DONE    = 4'd6,
        S_MEASURE_WAIT = 4'd7,
        S_COMMIT       = 4'd8,
        S_IDLE         = 4'd9;

    localparam [7:0]
        CMD_WRITE_REG      = 8'h0A,
        CMD_READ_REG       = 8'h0B,
        REG_PARTID         = 8'h02,
        REG_XDATA_L        = 8'h0E,
        REG_INTMAP1        = 8'h2A,
        REG_INTMAP2        = 8'h2B,
        REG_POWER_CTL      = 8'h2D,
        PARTID_EXPECT      = 8'hF2,
        INTMAP1_DATA_READY = 8'h01,
        INTMAP2_DISABLED   = 8'h00,
        POWER_CTL_MEASURE  = 8'h02,
        STAT_OK            = 8'h00,
        STAT_NOT_INIT      = 8'h01,
        STAT_READ_LENGTH   = 8'hE1,
        STAT_BAD_PARTID    = 8'hE5;

    reg [2:0]  op_r;
    reg [3:0]  state_r;

    reg        pending_sample_r;
    reg [2:0]  ridx_r;
    reg        rd_len_err_r;

    reg [7:0]  partid_r;
    reg [15:0] ax_r;
    reg [15:0] ay_r;
    reg [15:0] az_r;

    reg [31:0] measure_wait_start_us_r;

    reg        commit_valid_r;
    reg [7:0]  commit_status_r;
    reg [47:0] commit_payload_r;
    reg [3:0]  commit_next_state_r;

    function [7:0] op_wlen;
        input [2:0] op;
        begin
            case (op)
                OP_INTMAP1:   op_wlen = 8'd3;
                OP_INTMAP2:   op_wlen = 8'd3;
                OP_POWER_CTL: op_wlen = 8'd3;
                default:      op_wlen = 8'd2;
            endcase
        end
    endfunction

    function [7:0] op_rlen;
        input [2:0] op;
        begin
            case (op)
                OP_PARTID: op_rlen = 8'd1;
                OP_SAMPLE: op_rlen = 8'd6;
                default:   op_rlen = 8'd0;
            endcase
        end
    endfunction

    function [2:0] op_rlen3;
        input [2:0] op;
        begin
            case (op)
                OP_PARTID: op_rlen3 = 3'd1;
                OP_SAMPLE: op_rlen3 = 3'd6;
                default:   op_rlen3 = 3'd0;
            endcase
        end
    endfunction

    function [7:0] op_wbyte;
        input [2:0] op;
        input [1:0] idx;
        begin
            case (op)
                OP_INTMAP1: begin
                    case (idx)
                        2'd0:    op_wbyte = CMD_WRITE_REG;
                        2'd1:    op_wbyte = REG_INTMAP1;
                        default: op_wbyte = INTMAP1_DATA_READY;
                    endcase
                end

                OP_INTMAP2: begin
                    case (idx)
                        2'd0:    op_wbyte = CMD_WRITE_REG;
                        2'd1:    op_wbyte = REG_INTMAP2;
                        default: op_wbyte = INTMAP2_DISABLED;
                    endcase
                end

                OP_POWER_CTL: begin
                    case (idx)
                        2'd0:    op_wbyte = CMD_WRITE_REG;
                        2'd1:    op_wbyte = REG_POWER_CTL;
                        default: op_wbyte = POWER_CTL_MEASURE;
                    endcase
                end

                OP_PARTID: begin
                    case (idx)
                        2'd0:    op_wbyte = CMD_READ_REG;
                        default: op_wbyte = REG_PARTID;
                    endcase
                end

                default: begin
                    case (idx)
                        2'd0:    op_wbyte = CMD_READ_REG;
                        default: op_wbyte = REG_XDATA_L;
                    endcase
                end
            endcase
        end
    endfunction

    always @(posedge clk) begin
        if (rst) begin
            op_r                    <= OP_PARTID;
            state_r                 <= S_POWERUP;
            pending_sample_r        <= 1'b0;
            ridx_r                  <= 3'd0;
            rd_len_err_r            <= 1'b0;
            partid_r                <= 8'd0;
            ax_r                    <= 16'd0;
            ay_r                    <= 16'd0;
            az_r                    <= 16'd0;
            measure_wait_start_us_r <= 32'd0;
            commit_valid_r          <= 1'b0;
            commit_status_r         <= STAT_NOT_INIT;
            commit_payload_r        <= 48'd0;
            commit_next_state_r     <= S_POWERUP;
            init_done               <= 1'b0;
        end else begin
            if (epoch_100hz)
                pending_sample_r <= 1'b1;

            case (state_r)
                S_POWERUP: begin
                    init_done <= 1'b0;
                    op_r      <= OP_PARTID;

                    if (time_us >= POWERUP_WAIT_US)
                        state_r <= S_REQ;
                end

                S_REQ: begin
                    if (grant && cmd_ready) begin
                        ridx_r       <= 3'd0;
                        rd_len_err_r <= 1'b0;
                        state_r      <= S_WB0;
                    end
                end

                S_WB0: begin
                    if (w_ready)
                        state_r <= S_WB1;
                end

                S_WB1: begin
                    if (w_ready) begin
                        if (op_wlen(op_r) == 8'd2) begin
                            if (op_rlen(op_r) == 8'd0)
                                state_r <= S_WAIT_DONE;
                            else
                                state_r <= S_STREAM;
                        end else begin
                            state_r <= S_WB2;
                        end
                    end
                end

                S_WB2: begin
                    if (w_ready) begin
                        if (op_rlen(op_r) == 8'd0)
                            state_r <= S_WAIT_DONE;
                        else
                            state_r <= S_STREAM;
                    end
                end

                S_STREAM: begin
                    if (r_valid) begin
                        case (op_r)
                            OP_PARTID: begin
                                if (ridx_r == 3'd0)
                                    partid_r <= r_data;
                                else
                                    rd_len_err_r <= 1'b1;
                            end

                            OP_SAMPLE: begin
                                case (ridx_r)
                                    3'd0: ax_r[7:0]  <= r_data;
                                    3'd1: ax_r[15:8] <= r_data;
                                    3'd2: ay_r[7:0]  <= r_data;
                                    3'd3: ay_r[15:8] <= r_data;
                                    3'd4: az_r[7:0]  <= r_data;
                                    3'd5: az_r[15:8] <= r_data;
                                    default: rd_len_err_r <= 1'b1;
                                endcase
                            end

                            default: begin
                                rd_len_err_r <= 1'b1;
                            end
                        endcase

                        if (ridx_r >= op_rlen3(op_r)) begin
                            rd_len_err_r <= 1'b1;
                        end else begin
                            if (r_last != (ridx_r == (op_rlen3(op_r) - 3'd1)))
                                rd_len_err_r <= 1'b1;

                            if (ridx_r != 3'd7)
                                ridx_r <= ridx_r + 3'd1;
                        end
                    end

                    if (done) begin
                        if ((done_code != 4'd0) ||
                            rd_len_err_r ||
                            (ridx_r != op_rlen3(op_r))) begin
                            commit_valid_r      <= 1'b0;
                            commit_status_r     <= (done_code != 4'd0) ? (8'hE0 | {4'd0, done_code})
                                                                        : STAT_READ_LENGTH;
                            commit_payload_r    <= 48'd0;
                            commit_next_state_r <= init_done ? S_IDLE : S_POWERUP;
                            pending_sample_r    <= init_done ? 1'b0 : pending_sample_r;
                            op_r                <= init_done ? OP_SAMPLE : OP_PARTID;
                            state_r             <= S_COMMIT;
                        end else begin
                            case (op_r)
                                OP_PARTID: begin
                                    if (partid_r == PARTID_EXPECT) begin
                                        op_r    <= OP_INTMAP1;
                                        state_r <= S_REQ;
                                    end else begin
                                        commit_valid_r      <= 1'b0;
                                        commit_status_r     <= STAT_BAD_PARTID;
                                        commit_payload_r    <= {40'd0, partid_r};
                                        commit_next_state_r <= S_POWERUP;
                                        op_r                <= OP_PARTID;
                                        state_r             <= S_COMMIT;
                                    end
                                end

                                OP_SAMPLE: begin
                                    pending_sample_r      <= 1'b0;
                                    commit_valid_r        <= 1'b1;
                                    commit_status_r       <= STAT_OK;
                                    commit_payload_r      <= {ax_r, ay_r, az_r};
                                    commit_next_state_r   <= S_IDLE;
                                    op_r                  <= OP_SAMPLE;
                                    state_r               <= S_COMMIT;
                                end

                                default: begin
                                    state_r <= S_IDLE;
                                end
                            endcase
                        end
                    end
                end

                S_WAIT_DONE: begin
                    if (done) begin
                        if (done_code != 4'd0) begin
                            init_done           <= 1'b0;
                            commit_valid_r      <= 1'b0;
                            commit_status_r     <= 8'hE0 | {4'd0, done_code};
                            commit_payload_r    <= 48'd0;
                            commit_next_state_r <= S_POWERUP;
                            op_r                <= OP_PARTID;
                            state_r             <= S_COMMIT;
                        end else begin
                            case (op_r)
                                OP_INTMAP1: begin
                                    op_r    <= OP_INTMAP2;
                                    state_r <= S_REQ;
                                end

                                OP_INTMAP2: begin
                                    op_r    <= OP_POWER_CTL;
                                    state_r <= S_REQ;
                                end

                                OP_POWER_CTL: begin
                                    measure_wait_start_us_r <= time_us;
                                    state_r                 <= S_MEASURE_WAIT;
                                end

                                default: begin
                                    state_r <= S_IDLE;
                                end
                            endcase
                        end
                    end
                end

                S_MEASURE_WAIT: begin
                    if ((time_us - measure_wait_start_us_r) >= MEASURE_WAIT_US) begin
                        init_done <= 1'b1;
                        op_r      <= OP_SAMPLE;
                        state_r   <= S_IDLE;
                    end
                end

                S_COMMIT: begin
                    state_r <= commit_next_state_r;
                end

                S_IDLE: begin
                    if (!init_done) begin
                        op_r    <= OP_PARTID;
                        state_r <= S_POWERUP;
                    end else if (pending_sample_r) begin
                        op_r    <= OP_SAMPLE;
                        state_r <= S_REQ;
                    end
                end

                default: begin
                    op_r      <= OP_PARTID;
                    state_r   <= S_POWERUP;
                    init_done <= 1'b0;
                end
            endcase
        end
    end

    always @(*) begin
        cmd_valid      = (state_r == S_REQ) && grant;
        cmd_wlen       = op_wlen(op_r);
        cmd_rlen       = op_rlen(op_r);
        cmd_timeout_us = CMD_TIMEOUT_US;

        w_valid        = 1'b0;
        w_data         = 8'd0;
        w_last         = 1'b0;

        if (state_r == S_WB0) begin
            w_valid = 1'b1;
            w_data  = op_wbyte(op_r, 2'd0);
            w_last  = (op_wlen(op_r) == 8'd1);
        end else if (state_r == S_WB1) begin
            w_valid = 1'b1;
            w_data  = op_wbyte(op_r, 2'd1);
            w_last  = (op_wlen(op_r) == 8'd2);
        end else if (state_r == S_WB2) begin
            w_valid = 1'b1;
            w_data  = op_wbyte(op_r, 2'd2);
            w_last  = 1'b1;
        end

        r_ready        = (state_r == S_STREAM);
        busy           = !((state_r == S_IDLE) ||
                           (state_r == S_POWERUP) ||
                           (state_r == S_MEASURE_WAIT));

        snap_commit     = (state_r == S_COMMIT);
        snap_valid_in   = commit_valid_r;
        snap_status_in  = commit_status_r;
        snap_payload_in = commit_payload_r;
    end

endmodule

`default_nettype wire
