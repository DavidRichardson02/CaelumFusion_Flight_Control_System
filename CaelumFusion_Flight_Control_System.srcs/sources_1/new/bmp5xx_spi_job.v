`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// bmp5xx_spi_job
//------------------------------------------------------------------------------
// SPI-mode BMP5xx job FSM.
//
// WIRE PROTOCOL
//   - Uses SPI mode 3 through the shared engine.
//   - Read header = register address | 0x80.
//   - Write header = register address.
//
// ROBUSTNESS CHOICES
//   - Waits through a power-up guard time before first transaction.
//   - Performs a one-byte prime read before the real WHO_AM_I validation, which
//     mirrors Bosch's software driver recommendation for BMP5 SPI bring-up.
//   - Accepts both 0x50 and 0x51 chip IDs across the BMP5xx family.
//
// INIT SEQUENCE
//   PRIME -> WHO_AM_I -> FIFO_SEL -> INT_CONFIG -> INT_SOURCE -> OSR_CONFIG ->
//   ODR_CONFIG
//
// PERIODIC READ
//   Burst from TEMP_XLSB (0x1D) across 6 bytes.
//
// SNAPSHOT PAYLOAD
//   {PRESS_MSB, PRESS_LSB, PRESS_XLSB, TEMP_MSB, TEMP_LSB, TEMP_XLSB}
//==============================================================================
module bmp5xx_spi_job #(
    parameter [31:0] CMD_TIMEOUT_US  = 32'd8000,
    parameter [31:0] POWERUP_WAIT_US = 32'd3000
)(
    input  wire        clk,
    input  wire        rst,
    input  wire [31:0] time_us,

    input  wire        epoch_50hz,
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

    localparam [2:0]
        OP_PRIME    = 3'd0,
        OP_WHO      = 3'd1,
        OP_FIFO_SEL = 3'd2,
        OP_INT_CFG  = 3'd3,
        OP_INT_SRC  = 3'd4,
        OP_OSR_CFG  = 3'd5,
        OP_ODR_CFG  = 3'd6,
        OP_READ     = 3'd7;

    localparam [2:0]
        S_REQ    = 3'd0,
        S_WB0    = 3'd1,
        S_WB1    = 3'd2,
        S_STREAM = 3'd3,
        S_WAIT   = 3'd4,
        S_COMMIT = 3'd5,
        S_IDLE   = 3'd6;

    localparam [7:0]
        REG_CHIP_ID    = 8'h01,
        REG_INT_CONFIG = 8'h14,
        REG_INT_SOURCE = 8'h15,
        REG_FIFO_SEL   = 8'h18,
        REG_TEMP_XLSB  = 8'h1D,
        REG_OSR_CONFIG = 8'h36,
        REG_ODR_CONFIG = 8'h37,

        CHIP_ID_A      = 8'h50,
        CHIP_ID_B      = 8'h51,
        STAT_OK        = 8'h00,
        STAT_NOT_INIT  = 8'h01,
        STAT_BAD_WHO   = 8'hE3,
        STAT_RD_LEN    = 8'hE1;

    localparam [7:0]
        FIFO_SEL_VALUE   = 8'h00,
        INT_CONFIG_VALUE = 8'h0A,
        INT_SOURCE_VALUE = 8'h01,
        OSR_CONFIG_VALUE = 8'h59,
        ODR_CONFIG_VALUE = 8'hBD;

    reg [2:0] op_r;
    reg [2:0] state_r;

    reg       pending_sample_r;
    reg [2:0] ridx_r;
    reg       rd_len_err_r;

    reg [7:0] chip_id_r;
    reg [7:0] temp_xlsb_r;
    reg [7:0] temp_lsb_r;
    reg [7:0] temp_msb_r;
    reg [7:0] press_xlsb_r;
    reg [7:0] press_lsb_r;
    reg [7:0] press_msb_r;

    reg       commit_valid_r;
    reg [7:0] commit_status_r;
    reg [47:0] commit_payload_r;
    reg [2:0] commit_next_state_r;

    wire req_ready_w;
    assign req_ready_w = (time_us >= POWERUP_WAIT_US);

    function [7:0] op_wlen;
        input [2:0] op;
        begin
            case (op)
                OP_FIFO_SEL,
                OP_INT_CFG,
                OP_INT_SRC,
                OP_OSR_CFG,
                OP_ODR_CFG: op_wlen = 8'd2;
                default:    op_wlen = 8'd1;
            endcase
        end
    endfunction

    function [7:0] op_rlen;
        input [2:0] op;
        begin
            case (op)
                OP_PRIME,
                OP_WHO:  op_rlen = 8'd1;
                OP_READ: op_rlen = 8'd6;
                default: op_rlen = 8'd0;
            endcase
        end
    endfunction

    function [7:0] op_header;
        input [2:0] op;
        begin
            case (op)
                OP_PRIME,
                OP_WHO:      op_header = 8'h80 | REG_CHIP_ID;
                OP_FIFO_SEL: op_header = REG_FIFO_SEL;
                OP_INT_CFG:  op_header = REG_INT_CONFIG;
                OP_INT_SRC:  op_header = REG_INT_SOURCE;
                OP_OSR_CFG:  op_header = REG_OSR_CONFIG;
                OP_ODR_CFG:  op_header = REG_ODR_CONFIG;
                default:     op_header = 8'h80 | REG_TEMP_XLSB;
            endcase
        end
    endfunction

    function [7:0] op_data;
        input [2:0] op;
        begin
            case (op)
                OP_FIFO_SEL: op_data = FIFO_SEL_VALUE;
                OP_INT_CFG:  op_data = INT_CONFIG_VALUE;
                OP_INT_SRC:  op_data = INT_SOURCE_VALUE;
                OP_OSR_CFG:  op_data = OSR_CONFIG_VALUE;
                OP_ODR_CFG:  op_data = ODR_CONFIG_VALUE;
                default:     op_data = 8'h00;
            endcase
        end
    endfunction

    always @(posedge clk) begin
        if (rst) begin
            op_r                <= OP_PRIME;
            state_r             <= S_REQ;
            pending_sample_r    <= 1'b0;
            ridx_r              <= 3'd0;
            rd_len_err_r        <= 1'b0;
            chip_id_r           <= 8'd0;
            temp_xlsb_r         <= 8'd0;
            temp_lsb_r          <= 8'd0;
            temp_msb_r          <= 8'd0;
            press_xlsb_r        <= 8'd0;
            press_lsb_r         <= 8'd0;
            press_msb_r         <= 8'd0;
            commit_valid_r      <= 1'b0;
            commit_status_r     <= STAT_NOT_INIT;
            commit_payload_r    <= 48'd0;
            commit_next_state_r <= S_REQ;
            init_done           <= 1'b0;
        end else begin
            if (epoch_50hz)
                pending_sample_r <= 1'b1;

            case (state_r)
                S_REQ: begin
                    if (req_ready_w && grant && cmd_ready) begin
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
                            OP_PRIME,
                            OP_WHO: begin
                                chip_id_r <= r_data;
                                if (ridx_r != 3'd0)
                                    rd_len_err_r <= 1'b1;
                            end

                            OP_READ: begin
                                case (ridx_r)
                                    3'd0: temp_xlsb_r  <= r_data;
                                    3'd1: temp_lsb_r   <= r_data;
                                    3'd2: temp_msb_r   <= r_data;
                                    3'd3: press_xlsb_r <= r_data;
                                    3'd4: press_lsb_r  <= r_data;
                                    3'd5: press_msb_r  <= r_data;
                                    default: rd_len_err_r <= 1'b1;
                                endcase
                            end

                            default: begin
                                rd_len_err_r <= 1'b1;
                            end
                        endcase

                        if (ridx_r < (op_rlen(op_r) - 8'd1))
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
                                op_r <= OP_PRIME;
                            else
                                op_r <= OP_READ;
                            state_r <= S_COMMIT;
                        end else begin
                            case (op_r)
                                OP_PRIME: begin
                                    op_r    <= OP_WHO;
                                    state_r <= S_REQ;
                                end

                                OP_WHO: begin
                                    if (((r_valid ? r_data : chip_id_r) == CHIP_ID_A) ||
                                        ((r_valid ? r_data : chip_id_r) == CHIP_ID_B)) begin
                                        op_r    <= OP_FIFO_SEL;
                                        state_r <= S_REQ;
                                    end else begin
                                        init_done           <= 1'b0;
                                        commit_valid_r      <= 1'b0;
                                        commit_status_r     <= STAT_BAD_WHO;
                                        commit_payload_r    <= 48'd0;
                                        commit_next_state_r <= S_REQ;
                                        op_r                <= OP_PRIME;
                                        state_r             <= S_COMMIT;
                                    end
                                end

                                OP_READ: begin
                                    pending_sample_r      <= 1'b0;
                                    commit_valid_r        <= 1'b1;
                                    commit_status_r       <= STAT_OK;
                                    commit_payload_r      <= {
                                        press_msb_r,
                                        press_lsb_r,
                                        press_xlsb_r,
                                        temp_msb_r,
                                        temp_lsb_r,
                                        temp_xlsb_r
                                    };
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
                            op_r                <= OP_PRIME;
                            state_r             <= S_COMMIT;
                        end else begin
                            case (op_r)
                                OP_FIFO_SEL: begin
                                    op_r    <= OP_INT_CFG;
                                    state_r <= S_REQ;
                                end

                                OP_INT_CFG: begin
                                    op_r    <= OP_INT_SRC;
                                    state_r <= S_REQ;
                                end

                                OP_INT_SRC: begin
                                    op_r    <= OP_OSR_CFG;
                                    state_r <= S_REQ;
                                end

                                OP_OSR_CFG: begin
                                    op_r    <= OP_ODR_CFG;
                                    state_r <= S_REQ;
                                end

                                OP_ODR_CFG: begin
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
                        op_r    <= OP_READ;
                        state_r <= S_REQ;
                    end
                end

                default: begin
                    op_r      <= OP_PRIME;
                    state_r   <= S_REQ;
                    init_done <= 1'b0;
                end
            endcase
        end
    end

    always @(*) begin
        cmd_valid      = (state_r == S_REQ) && req_ready_w;
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
