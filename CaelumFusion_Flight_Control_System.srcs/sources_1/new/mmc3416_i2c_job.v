`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// mmc3416_i2c_job
//------------------------------------------------------------------------------
// CMPS2/MMC34160PJ I2C magnetometer acquisition job.
//
// Contract:
//   - Does not drive SCL/SDA directly; it only talks to i2c_master_engine through
//     the shared job-mux command/write/read/done interface.
//   - Default address is the Digilent Pmod CMPS2 7-bit address 7'h30.
//   - Publishes one committed 48-bit magnetic snapshot per successful read.
//   - Payload order is {MZ[15:0], MY[15:0], MX[15:0]}, i.e. Z/Y/X, because the
//     project derived-state path is instantiated with MAG_PAYLOAD_ZYX=1.
//
// Notes:
//   This module name and port surface are intentionally unchanged. Replace the
//   existing mmc3416_i2c_job definition with this implementation; do not add a
//   second module with the same name to the same Vivado fileset.
//==============================================================================
module mmc3416_i2c_job #(
    parameter [6:0]  MMC3416_ADDR7      = 7'h30,
    parameter [6:0]  MMC3416_ADDR7_MIN  = 7'h30,
    parameter [6:0]  MMC3416_ADDR7_MAX  = 7'h30,
    parameter integer ADDR_PROBE_EN     = 0,
    parameter [31:0] CMD_TIMEOUT_US     = 32'd3000,
    parameter [31:0] POWERUP_WAIT_US    = 32'd10000,
    parameter [31:0] REFILL_WAIT_US     = 32'd50000,
    parameter [31:0] SETRESET_WAIT_US   = 32'd1000,
    parameter [31:0] POLL_GAP_US        = 32'd1000,
    parameter [31:0] MEAS_TIMEOUT_US    = 32'd15000,
    parameter [7:0]  CONTROL1_VALUE     = 8'h00
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
    localparam [2:0]
        OP_ID     = 3'd0,
        OP_CTRL1  = 3'd1,
        OP_REFILL = 3'd2,
        OP_SET    = 3'd3,
        OP_TM     = 3'd4,
        OP_STATUS = 3'd5,
        OP_READ   = 3'd6;

    localparam [3:0]
        S_RESET_WAIT  = 4'd0,
        S_IDLE        = 4'd1,
        S_CMD         = 4'd2,
        S_WRITE       = 4'd3,
        S_READ        = 4'd4,
        S_WAIT_DONE   = 4'd5,
        S_HANDLE_DONE = 4'd6,
        S_DELAY       = 4'd7,
        S_COMMIT      = 4'd8;

    localparam [1:0]
        DELAY_TO_OP       = 2'd0,
        DELAY_INIT_DONE   = 2'd1,
        DELAY_TO_STATUS   = 2'd2,
        DELAY_RETRY_ID    = 2'd3;

    localparam [7:0]
        REG_XOUT_L        = 8'h00,
        REG_STATUS        = 8'h06,
        REG_CONTROL0      = 8'h07,
        REG_CONTROL1      = 8'h08,
        REG_PRODUCT_ID    = 8'h20,
        PRODUCT_ID_EXPECT = 8'h06,
        CTRL0_TM          = 8'h01,
        CTRL0_SET         = 8'h20,
        CTRL0_REFILL      = 8'h80,
        STATUS_MEAS_DONE  = 8'h01,
        STAT_OK           = 8'h00,
        STAT_NOT_INIT     = 8'h01,
        STAT_I2C_ERROR    = 8'hE0,
        STAT_READ_LENGTH  = 8'hE1,
        STAT_BAD_ID       = 8'hE5,
        STAT_MEAS_TIMEOUT = 8'hE6;

    reg [3:0]  st_r;
    reg [2:0]  op_r;
    reg [1:0]  delay_kind_r;
    reg [2:0]  delay_next_op_r;
    reg [31:0] delay_start_us_r;
    reg [31:0] delay_us_r;
    reg [31:0] meas_start_us_r;
    reg        pending_sample_r;
    reg [6:0]  active_addr7_r;
    reg [7:0]  widx_r;
    reg [7:0]  ridx_r;
    reg        rd_len_err_r;
    reg [7:0]  id_r;
    reg [7:0]  status_r;
    reg [7:0]  xl_r;
    reg [7:0]  xh_r;
    reg [7:0]  yl_r;
    reg [7:0]  yh_r;
    reg [7:0]  zl_r;
    reg [7:0]  zh_r;
    reg        commit_valid_r;
    reg [7:0]  commit_status_r;
    reg [47:0] commit_payload_r;

    function [7:0] op_wlen;
        input [2:0] op;
        begin
            case (op)
                OP_CTRL1,
                OP_REFILL,
                OP_SET,
                OP_TM:   op_wlen = 8'd2;
                default: op_wlen = 8'd1;
            endcase
        end
    endfunction

    function [7:0] op_rlen;
        input [2:0] op;
        begin
            case (op)
                OP_ID,
                OP_STATUS: op_rlen = 8'd1;
                OP_READ:   op_rlen = 8'd6;
                default:   op_rlen = 8'd0;
            endcase
        end
    endfunction

    function [7:0] op_reg_addr;
        input [2:0] op;
        begin
            case (op)
                OP_ID:     op_reg_addr = REG_PRODUCT_ID;
                OP_CTRL1:  op_reg_addr = REG_CONTROL1;
                OP_REFILL,
                OP_SET,
                OP_TM:     op_reg_addr = REG_CONTROL0;
                OP_STATUS: op_reg_addr = REG_STATUS;
                default:   op_reg_addr = REG_XOUT_L;
            endcase
        end
    endfunction

    function [7:0] op_reg_data;
        input [2:0] op;
        begin
            case (op)
                OP_CTRL1:  op_reg_data = CONTROL1_VALUE;
                OP_REFILL: op_reg_data = CTRL0_REFILL;
                OP_SET:    op_reg_data = CTRL0_SET;
                OP_TM:     op_reg_data = CTRL0_TM;
                default:   op_reg_data = 8'h00;
            endcase
        end
    endfunction

    wire [7:0] op_wlen_w;
    wire [7:0] op_rlen_w;
    assign op_wlen_w = op_wlen(op_r);
    assign op_rlen_w = op_rlen(op_r);

    wire [8:0] rx_count_next_w;
    wire       rd_len_err_now_w;
    assign rx_count_next_w = {1'b0, ridx_r} + (r_valid ? 9'd1 : 9'd0);
    assign rd_len_err_now_w =
        rd_len_err_r |
        (r_valid && ({1'b0, ridx_r} >= {1'b0, op_rlen_w})) |
        (r_valid && r_last && (ridx_r != (op_rlen_w - 8'd1)));

    wire delay_done_w;
    assign delay_done_w = ((time_us - delay_start_us_r) >= delay_us_r);

    wire meas_timeout_w;
    assign meas_timeout_w = ((time_us - meas_start_us_r) >= MEAS_TIMEOUT_US);

    wire probe_can_advance_w;
    assign probe_can_advance_w =
        (ADDR_PROBE_EN != 0) && (active_addr7_r < MMC3416_ADDR7_MAX);

    always @(posedge clk) begin
        if (rst) begin
            st_r             <= S_RESET_WAIT;
            op_r             <= OP_ID;
            delay_kind_r     <= DELAY_TO_OP;
            delay_next_op_r  <= OP_ID;
            delay_start_us_r <= 32'd0;
            delay_us_r       <= POWERUP_WAIT_US;
            meas_start_us_r  <= 32'd0;
            pending_sample_r <= 1'b0;
            active_addr7_r   <= (ADDR_PROBE_EN != 0) ? MMC3416_ADDR7_MIN : MMC3416_ADDR7;
            widx_r           <= 8'd0;
            ridx_r           <= 8'd0;
            rd_len_err_r     <= 1'b0;
            id_r             <= 8'd0;
            status_r         <= 8'd0;
            xl_r             <= 8'd0;
            xh_r             <= 8'd0;
            yl_r             <= 8'd0;
            yh_r             <= 8'd0;
            zl_r             <= 8'd0;
            zh_r             <= 8'd0;
            commit_valid_r   <= 1'b0;
            commit_status_r  <= STAT_NOT_INIT;
            commit_payload_r <= 48'd0;
            cmd_valid        <= 1'b0;
            cmd_addr7        <= 7'd0;
            cmd_wlen         <= 8'd0;
            cmd_rlen         <= 8'd0;
            cmd_repstart     <= 1'b0;
            cmd_timeout_us   <= CMD_TIMEOUT_US;
            w_valid          <= 1'b0;
            w_data           <= 8'd0;
            w_last           <= 1'b0;
            r_ready          <= 1'b0;
            busy             <= 1'b0;
            snap_commit      <= 1'b0;
            snap_valid_in    <= 1'b0;
            snap_status_in   <= STAT_NOT_INIT;
            snap_payload_in  <= 48'd0;
            init_done        <= 1'b0;
        end else begin
            snap_commit <= 1'b0;

            if (epoch_10hz)
                pending_sample_r <= 1'b1;

            case (st_r)
                S_RESET_WAIT: begin
                    busy <= 1'b1;
                    cmd_valid <= 1'b0;
                    w_valid <= 1'b0;
                    r_ready <= 1'b0;
                    if (delay_done_w) begin
                        busy <= 1'b0;
                        st_r <= S_IDLE;
                    end
                end

                S_IDLE: begin
                    busy <= 1'b0;
                    cmd_valid <= 1'b0;
                    w_valid <= 1'b0;
                    r_ready <= 1'b0;
                    if (grant) begin
                        busy <= 1'b1;
                        widx_r <= 8'd0;
                        ridx_r <= 8'd0;
                        rd_len_err_r <= 1'b0;
                        if (!init_done) begin
                            op_r <= OP_ID;
                            st_r <= S_CMD;
                        end else if (pending_sample_r) begin
                            pending_sample_r <= 1'b0;
                            op_r <= OP_TM;
                            meas_start_us_r <= time_us;
                            st_r <= S_CMD;
                        end
                    end
                end

                S_CMD: begin
                    busy <= 1'b1;
                    cmd_valid      <= 1'b1;
                    cmd_addr7      <= active_addr7_r;
                    cmd_wlen       <= op_wlen_w;
                    cmd_rlen       <= op_rlen_w;
                    cmd_repstart   <= (op_rlen_w != 8'd0);
                    cmd_timeout_us <= CMD_TIMEOUT_US;
                    w_valid        <= 1'b0;
                    r_ready        <= 1'b0;
                    if (cmd_valid && cmd_ready) begin
                        cmd_valid <= 1'b0;
                        widx_r    <= 8'd0;
                        ridx_r    <= 8'd0;
                        st_r      <= S_WRITE;
                    end
                end

                S_WRITE: begin
                    busy <= 1'b1;
                    w_valid <= 1'b1;
                    if (widx_r == 8'd0)
                        w_data <= op_reg_addr(op_r);
                    else
                        w_data <= op_reg_data(op_r);
                    w_last <= (widx_r == (op_wlen_w - 8'd1));
                    if (w_valid && w_ready) begin
                        if (widx_r == (op_wlen_w - 8'd1)) begin
                            w_valid <= 1'b0;
                            w_last  <= 1'b0;
                            if (op_rlen_w != 8'd0) begin
                                r_ready <= 1'b1;
                                st_r    <= S_READ;
                            end else begin
                                st_r <= S_WAIT_DONE;
                            end
                        end else begin
                            // Prepare the next byte in the same cycle that the
                            // current byte is accepted. The engine samples the
                            // registered stream outputs on the following clock.
                            widx_r <= widx_r + 8'd1;
                            w_data <= op_reg_data(op_r);
                            w_last <= ((widx_r + 8'd1) == (op_wlen_w - 8'd1));
                        end
                    end
                end

                S_READ: begin
                    busy <= 1'b1;
                    r_ready <= 1'b1;
                    if (r_valid) begin
                        case (op_r)
                            OP_ID: begin
                                if (ridx_r == 8'd0) id_r <= r_data;
                                else rd_len_err_r <= 1'b1;
                            end
                            OP_STATUS: begin
                                if (ridx_r == 8'd0) status_r <= r_data;
                                else rd_len_err_r <= 1'b1;
                            end
                            OP_READ: begin
                                case (ridx_r)
                                    8'd0: xl_r <= r_data;
                                    8'd1: xh_r <= r_data;
                                    8'd2: yl_r <= r_data;
                                    8'd3: yh_r <= r_data;
                                    8'd4: zl_r <= r_data;
                                    8'd5: zh_r <= r_data;
                                    default: rd_len_err_r <= 1'b1;
                                endcase
                            end
                            default: rd_len_err_r <= 1'b1;
                        endcase

                        if (ridx_r >= op_rlen_w)
                            rd_len_err_r <= 1'b1;
                        if (r_last && (ridx_r != (op_rlen_w - 8'd1)))
                            rd_len_err_r <= 1'b1;
                        ridx_r <= ridx_r + 8'd1;
                    end

                    if (done) begin
                        r_ready <= 1'b0;
                        if (done_code != 4'd0)
                            commit_status_r <= STAT_I2C_ERROR;
                        else if (rd_len_err_now_w || (rx_count_next_w != {1'b0, op_rlen_w}))
                            commit_status_r <= STAT_READ_LENGTH;
                        else
                            commit_status_r <= STAT_OK;
                        st_r <= S_HANDLE_DONE;
                    end
                end

                S_WAIT_DONE: begin
                    busy <= 1'b1;
                    w_valid <= 1'b0;
                    r_ready <= 1'b0;
                    if (done) begin
                        if (done_code != 4'd0)
                            commit_status_r <= STAT_I2C_ERROR;
                        else
                            commit_status_r <= STAT_OK;
                        st_r <= S_HANDLE_DONE;
                    end
                end

                S_HANDLE_DONE: begin
                    busy <= 1'b1;
                    case (op_r)
                        OP_ID: begin
                            if (commit_status_r == STAT_OK) begin
                                if (id_r == PRODUCT_ID_EXPECT) begin
                                    op_r <= OP_CTRL1;
                                    st_r <= S_CMD;
                                end else if (probe_can_advance_w) begin
                                    active_addr7_r <= active_addr7_r + 7'd1;
                                    op_r <= OP_ID;
                                    st_r <= S_CMD;
                                end else begin
                                    commit_valid_r   <= 1'b0;
                                    commit_status_r  <= STAT_BAD_ID;
                                    commit_payload_r <= {40'd0, id_r};
                                    st_r <= S_COMMIT;
                                end
                            end else begin
                                if (probe_can_advance_w) begin
                                    active_addr7_r <= active_addr7_r + 7'd1;
                                    op_r <= OP_ID;
                                    st_r <= S_CMD;
                                end else begin
                                    commit_valid_r   <= 1'b0;
                                    commit_payload_r <= 48'd0;
                                    st_r <= S_COMMIT;
                                end
                            end
                        end

                        OP_CTRL1: begin
                            if (commit_status_r == STAT_OK) begin
                                op_r <= OP_REFILL;
                                st_r <= S_CMD;
                            end else begin
                                commit_valid_r <= 1'b0;
                                commit_payload_r <= 48'd0;
                                st_r <= S_COMMIT;
                            end
                        end

                        OP_REFILL: begin
                            if (commit_status_r == STAT_OK) begin
                                delay_kind_r    <= DELAY_TO_OP;
                                delay_next_op_r <= OP_SET;
                                delay_start_us_r<= time_us;
                                delay_us_r      <= REFILL_WAIT_US;
                                st_r            <= S_DELAY;
                            end else begin
                                commit_valid_r <= 1'b0;
                                commit_payload_r <= 48'd0;
                                st_r <= S_COMMIT;
                            end
                        end

                        OP_SET: begin
                            if (commit_status_r == STAT_OK) begin
                                delay_kind_r     <= DELAY_INIT_DONE;
                                delay_start_us_r <= time_us;
                                delay_us_r       <= SETRESET_WAIT_US;
                                st_r             <= S_DELAY;
                            end else begin
                                commit_valid_r <= 1'b0;
                                commit_payload_r <= 48'd0;
                                st_r <= S_COMMIT;
                            end
                        end

                        OP_TM: begin
                            if (commit_status_r == STAT_OK) begin
                                delay_kind_r     <= DELAY_TO_STATUS;
                                delay_start_us_r <= time_us;
                                delay_us_r       <= POLL_GAP_US;
                                st_r             <= S_DELAY;
                            end else begin
                                commit_valid_r <= 1'b0;
                                commit_payload_r <= 48'd0;
                                st_r <= S_COMMIT;
                            end
                        end

                        OP_STATUS: begin
                            if (commit_status_r != STAT_OK) begin
                                commit_valid_r <= 1'b0;
                                commit_payload_r <= 48'd0;
                                st_r <= S_COMMIT;
                            end else if ((status_r & STATUS_MEAS_DONE) != 8'd0) begin
                                op_r <= OP_READ;
                                st_r <= S_CMD;
                            end else if (meas_timeout_w) begin
                                commit_valid_r   <= 1'b0;
                                commit_status_r  <= STAT_MEAS_TIMEOUT;
                                commit_payload_r <= {40'd0, status_r};
                                st_r <= S_COMMIT;
                            end else begin
                                delay_kind_r     <= DELAY_TO_STATUS;
                                delay_start_us_r <= time_us;
                                delay_us_r       <= POLL_GAP_US;
                                st_r             <= S_DELAY;
                            end
                        end

                        OP_READ: begin
                            if (commit_status_r == STAT_OK) begin
                                commit_valid_r   <= 1'b1;
                                commit_status_r  <= STAT_OK;
                                commit_payload_r <= {zh_r, zl_r, yh_r, yl_r, xh_r, xl_r};
                            end else begin
                                commit_valid_r   <= 1'b0;
                                commit_payload_r <= {zh_r, zl_r, yh_r, yl_r, xh_r, xl_r};
                            end
                            st_r <= S_COMMIT;
                        end

                        default: begin
                            commit_valid_r <= 1'b0;
                            commit_status_r <= STAT_I2C_ERROR;
                            commit_payload_r <= 48'd0;
                            st_r <= S_COMMIT;
                        end
                    endcase
                end

                S_DELAY: begin
                    busy <= 1'b1;
                    cmd_valid <= 1'b0;
                    w_valid <= 1'b0;
                    r_ready <= 1'b0;
                    if (delay_done_w) begin
                        case (delay_kind_r)
                            DELAY_TO_OP: begin
                                op_r <= delay_next_op_r;
                                st_r <= S_CMD;
                            end
                            DELAY_INIT_DONE: begin
                                init_done <= 1'b1;
                                st_r <= S_IDLE;
                            end
                            DELAY_TO_STATUS: begin
                                op_r <= OP_STATUS;
                                st_r <= S_CMD;
                            end
                            default: begin
                                op_r <= OP_ID;
                                st_r <= S_CMD;
                            end
                        endcase
                    end
                end

                S_COMMIT: begin
                    busy            <= 1'b1;
                    snap_commit     <= 1'b1;
                    snap_valid_in   <= commit_valid_r;
                    snap_status_in  <= commit_status_r;
                    snap_payload_in <= commit_payload_r;
                    if (!init_done) begin
                        // Retry initialization after reporting the failure once.
                        active_addr7_r   <= (ADDR_PROBE_EN != 0) ? MMC3416_ADDR7_MIN : MMC3416_ADDR7;
                        delay_kind_r     <= DELAY_RETRY_ID;
                        delay_start_us_r <= time_us;
                        delay_us_r       <= POWERUP_WAIT_US;
                        st_r             <= S_DELAY;
                    end else begin
                        st_r <= S_IDLE;
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
