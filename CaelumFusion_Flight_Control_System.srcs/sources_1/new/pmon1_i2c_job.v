`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// pmon1_i2c_job
//------------------------------------------------------------------------------
// Digilent Pmod PMON1 / ADM1191 power monitor acquisition job.
//
// Contract:
//   - Does not drive SCL/SDA directly; it only talks to i2c_master_engine through
//     the shared job-mux command/write/read/done interface.
//   - Default address is 7'h38, not the ADM1191 all-jumpers-00 7'h30 address,
//     because this project already uses 7'h30 for the CMPS2/MMC34160PJ
//     magnetometer. PMON1 jumpers must be moved to match PMON1_ADDR7 before
//     enabling this path.
//   - Publishes one committed 48-bit power snapshot after each completed sample.
//
// Snapshot payload:
//   [47:40] ADM1191 status byte from STATUS_RD readback.
//   [39:28] 12-bit voltage ADC code, raw ADM1191 format.
//   [27:16] 12-bit current ADC code, raw ADM1191 format.
//   [15:0]  reserved for later fixed-point scaling / alert provenance.
//==============================================================================
module pmon1_i2c_job #(
    parameter [6:0]  PMON1_ADDR7      = 7'h38,
    parameter [31:0] CMD_TIMEOUT_US   = 32'd3000,
    parameter [31:0] POWERUP_WAIT_US  = 32'd3000,
    parameter [7:0]  DATA_CMD         = 8'h05, // V_CONT | I_CONT, high range
    parameter [7:0]  STATUS_CMD       = 8'h45  // STATUS_RD | V_CONT | I_CONT
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
    localparam [7:0]
        STAT_OK          = 8'h00,
        STAT_NOT_INIT    = 8'h01,
        STAT_I2C_ERROR   = 8'hE0,
        STAT_READ_LENGTH = 8'hE1;

    localparam [0:0]
        OP_DATA   = 1'b0,
        OP_STATUS = 1'b1;

    localparam [2:0]
        S_IDLE   = 3'd0,
        S_CMD    = 3'd1,
        S_WRITE  = 3'd2,
        S_READ   = 3'd3,
        S_COMMIT = 3'd4;

    reg [2:0]  st_r;
    reg        op_r;
    reg        pending_sample_r;
    reg        cmd_armed_r;
    reg [7:0]  ridx_r;
    reg        rd_len_err_r;
    reg [3:0]  done_code_r;

    reg [7:0]  data_b0_r;
    reg [7:0]  data_b1_r;
    reg [7:0]  data_b2_r;
    reg [7:0]  adm_status_r;
    reg [11:0] voltage_code_r;
    reg [11:0] current_code_r;

    function [7:0] op_rlen;
        input op;
        begin
            if (op == OP_STATUS)
                op_rlen = 8'd1;
            else
                op_rlen = 8'd3;
        end
    endfunction

    function [7:0] op_cmd;
        input op;
        begin
            if (op == OP_STATUS)
                op_cmd = STATUS_CMD;
            else
                op_cmd = DATA_CMD;
        end
    endfunction

    wire [7:0] expected_rlen_w;
    wire [8:0] rx_count_next_w;
    wire       rd_len_err_now_w;

    assign expected_rlen_w = op_rlen(op_r);
    assign rx_count_next_w = {1'b0, ridx_r} + (r_valid ? 9'd1 : 9'd0);
    assign rd_len_err_now_w =
        rd_len_err_r |
        (r_valid && ({1'b0, ridx_r} >= {1'b0, expected_rlen_w})) |
        (r_valid && r_last  && (rx_count_next_w != {1'b0, expected_rlen_w})) |
        (r_valid && !r_last && (rx_count_next_w == {1'b0, expected_rlen_w})) |
        (done && (rx_count_next_w != {1'b0, expected_rlen_w}));

    always @(posedge clk) begin
        if (rst) begin
            st_r              <= S_IDLE;
            op_r              <= OP_DATA;
            pending_sample_r  <= 1'b0;
            cmd_armed_r       <= 1'b0;
            ridx_r            <= 8'd0;
            rd_len_err_r      <= 1'b0;
            done_code_r       <= 4'd0;

            data_b0_r         <= 8'd0;
            data_b1_r         <= 8'd0;
            data_b2_r         <= 8'd0;
            adm_status_r      <= 8'd0;
            voltage_code_r    <= 12'd0;
            current_code_r    <= 12'd0;

            cmd_valid         <= 1'b0;
            cmd_addr7         <= PMON1_ADDR7;
            cmd_wlen          <= 8'd0;
            cmd_rlen          <= 8'd0;
            cmd_repstart      <= 1'b0;
            cmd_timeout_us    <= CMD_TIMEOUT_US;

            w_valid           <= 1'b0;
            w_data            <= 8'd0;
            w_last            <= 1'b0;

            r_ready           <= 1'b0;
            busy              <= 1'b0;

            snap_commit       <= 1'b0;
            snap_valid_in     <= 1'b0;
            snap_status_in    <= STAT_NOT_INIT;
            snap_payload_in   <= 48'd0;
            init_done         <= 1'b0;
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

                    if ((time_us >= POWERUP_WAIT_US) &&
                        grant &&
                        (!init_done || pending_sample_r)) begin
                        busy             <= 1'b1;
                        pending_sample_r <= 1'b0;
                        op_r             <= OP_DATA;
                        ridx_r           <= 8'd0;
                        rd_len_err_r     <= 1'b0;
                        done_code_r      <= 4'd0;
                        cmd_addr7        <= PMON1_ADDR7;
                        cmd_wlen         <= 8'd1;
                        cmd_rlen         <= op_rlen(OP_DATA);
                        cmd_repstart     <= 1'b1;
                        cmd_timeout_us   <= CMD_TIMEOUT_US;
                        cmd_armed_r      <= 1'b0;
                        st_r             <= S_CMD;
                    end
                end

                S_CMD: begin
                    busy           <= 1'b1;
                    cmd_valid      <= 1'b1;
                    cmd_addr7      <= PMON1_ADDR7;
                    cmd_wlen       <= 8'd1;
                    cmd_rlen       <= expected_rlen_w;
                    cmd_repstart   <= 1'b1;
                    cmd_timeout_us <= CMD_TIMEOUT_US;

                    if (!cmd_armed_r) begin
                        cmd_armed_r <= 1'b1;
                    end else if (cmd_ready) begin
                        cmd_valid    <= 1'b0;
                        cmd_armed_r  <= 1'b0;
                        w_valid      <= 1'b1;
                        w_data       <= op_cmd(op_r);
                        w_last       <= 1'b1;
                        ridx_r       <= 8'd0;
                        rd_len_err_r <= 1'b0;
                        done_code_r  <= 4'd0;
                        st_r         <= S_WRITE;
                    end
                end

                S_WRITE: begin
                    busy <= 1'b1;

                    if (w_valid && w_ready) begin
                        w_valid <= 1'b0;
                        r_ready <= 1'b1;
                        st_r    <= S_READ;
                    end
                end

                S_READ: begin
                    busy    <= 1'b1;
                    r_ready <= 1'b1;

                    if (r_valid) begin
                        if (op_r == OP_DATA) begin
                            case (ridx_r)
                                8'd0: data_b0_r <= r_data;
                                8'd1: data_b1_r <= r_data;
                                8'd2: begin
                                    data_b2_r      <= r_data;
                                    voltage_code_r <= {data_b0_r, r_data[7:4]};
                                    current_code_r <= {data_b1_r, r_data[3:0]};
                                end
                                default: begin
                                end
                            endcase
                        end else begin
                            if (ridx_r == 8'd0)
                                adm_status_r <= r_data;
                        end

                        ridx_r       <= ridx_r + 8'd1;
                        rd_len_err_r <= rd_len_err_now_w;
                    end

                    if (done) begin
                        r_ready      <= 1'b0;
                        done_code_r  <= done_code;
                        rd_len_err_r <= (done_code == 4'd0) ? rd_len_err_now_w
                                                            : rd_len_err_r;

                        if ((op_r == OP_DATA) &&
                            (done_code == 4'd0) &&
                            !rd_len_err_now_w) begin
                            op_r           <= OP_STATUS;
                            cmd_addr7      <= PMON1_ADDR7;
                            cmd_wlen       <= 8'd1;
                            cmd_rlen       <= op_rlen(OP_STATUS);
                            cmd_repstart   <= 1'b1;
                            cmd_timeout_us <= CMD_TIMEOUT_US;
                            cmd_armed_r    <= 1'b0;
                            st_r           <= S_CMD;
                        end else begin
                            st_r <= S_COMMIT;
                        end
                    end
                end

                S_COMMIT: begin
                    busy            <= 1'b0;
                    snap_commit     <= 1'b1;
                    snap_payload_in <= {adm_status_r, voltage_code_r,
                                        current_code_r, 16'h0000};

                    if (rd_len_err_r) begin
                        snap_valid_in  <= 1'b0;
                        snap_status_in <= STAT_READ_LENGTH;
                    end else if (done_code_r != 4'd0) begin
                        snap_valid_in  <= 1'b0;
                        snap_status_in <= STAT_I2C_ERROR;
                    end else begin
                        snap_valid_in  <= 1'b1;
                        snap_status_in <= STAT_OK;
                        init_done      <= 1'b1;
                    end

                    st_r <= S_IDLE;
                end

                default: begin
                    st_r <= S_IDLE;
                end
            endcase
        end
    end
endmodule

`default_nettype wire
