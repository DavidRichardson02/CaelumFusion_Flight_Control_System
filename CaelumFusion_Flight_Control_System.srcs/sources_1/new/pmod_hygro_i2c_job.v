`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// pmod_hygro_i2c_job
//------------------------------------------------------------------------------
// Digilent Pmod HYGRO / TI HDC1080-class humidity and temperature acquisition.
//
// Electrical/protocol contract:
//   - I2C address 7'h40.
//   - The shared i2c_master_engine owns SCL/SDA.
//   - The job triggers temperature+humidity conversion by writing pointer 0x00,
//     waits for conversion to complete, then performs a read-only 4-byte read.
//   - Sampling is intentionally slow; the Pmod HYGRO manual recommends at least
//     one second between samples to avoid self-heating.
//
// Snapshot payload:
//   [47:32] signed temperature in centi-degrees C
//   [31:16] relative humidity in centi-percent RH
//   [15:0]  reserved
//==============================================================================
module pmod_hygro_i2c_job #(
    parameter [6:0]  HYGRO_ADDR7        = 7'h40,
    parameter [31:0] CMD_TIMEOUT_US     = 32'd5000,
    parameter [31:0] POWERUP_WAIT_US    = 32'd15000,
    parameter [31:0] CONVERSION_WAIT_US = 32'd15000
)(
    input  wire        clk,
    input  wire        rst,
    input  wire [31:0] time_us,
    input  wire        epoch_1hz,
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
    localparam [0:0]
        OP_TRIGGER = 1'b0,
        OP_READ    = 1'b1;

    localparam [2:0]
        S_IDLE      = 3'd0,
        S_CMD       = 3'd1,
        S_WRITE     = 3'd2,
        S_READ      = 3'd3,
        S_CONV_WAIT = 3'd4,
        S_COMMIT    = 3'd5;

    localparam [7:0]
        REG_TEMP_RH      = 8'h00,
        STAT_OK          = 8'h00,
        STAT_NOT_INIT    = 8'h01,
        STAT_I2C_ERROR   = 8'hE0,
        STAT_READ_LENGTH = 8'hE1;

    reg       op_r;
    reg [2:0] st_r;
    reg       pending_sample_r;
    reg [7:0] ridx_r;
    reg       rd_len_err_r;
    reg [3:0] done_code_r;
    reg [31:0] conv_start_us_r;

    reg [7:0] temp_msb_r;
    reg [7:0] temp_lsb_r;
    reg [7:0] rh_msb_r;
    reg [7:0] rh_lsb_r;

    wire [15:0] temp_raw_w = {temp_msb_r, temp_lsb_r};
    wire [15:0] rh_raw_w   = {rh_msb_r, rh_lsb_r};

    function [15:0] temp_cdeg_from_raw;
        input [15:0] raw;
        reg [31:0] scaled;
        reg signed [31:0] signed_scaled;
        begin
            scaled = ({16'd0, raw} * 32'd16500) >> 16;
            signed_scaled = $signed({1'b0, scaled[30:0]}) - 32'sd4000;
            temp_cdeg_from_raw = signed_scaled[15:0];
        end
    endfunction

    function [15:0] rh_centi_from_raw;
        input [15:0] raw;
        reg [31:0] scaled;
        begin
            scaled = ({16'd0, raw} * 32'd10000) >> 16;
            rh_centi_from_raw = scaled[15:0];
        end
    endfunction

    wire [15:0] temp_cdeg_w = temp_cdeg_from_raw(temp_raw_w);
    wire [15:0] rh_centi_w  = rh_centi_from_raw(rh_raw_w);
    wire [8:0]  rx_count_next_w = {1'b0, ridx_r} + (r_valid ? 9'd1 : 9'd0);
    wire        rd_len_err_now_w =
        rd_len_err_r |
        (r_valid && ({1'b0, ridx_r} >= 9'd4)) |
        (r_valid && r_last  && (rx_count_next_w != 9'd4)) |
        (r_valid && !r_last && (rx_count_next_w == 9'd4)) |
        (done && (rx_count_next_w != 9'd4));

    always @(posedge clk) begin
        if (rst) begin
            op_r             <= OP_TRIGGER;
            st_r             <= S_IDLE;
            pending_sample_r <= 1'b0;
            ridx_r           <= 8'd0;
            rd_len_err_r     <= 1'b0;
            done_code_r      <= 4'd0;
            conv_start_us_r  <= 32'd0;

            temp_msb_r       <= 8'd0;
            temp_lsb_r       <= 8'd0;
            rh_msb_r         <= 8'd0;
            rh_lsb_r         <= 8'd0;

            cmd_valid        <= 1'b0;
            cmd_addr7        <= HYGRO_ADDR7;
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

            if (epoch_1hz)
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
                        busy           <= 1'b1;
                        op_r           <= OP_TRIGGER;
                        cmd_addr7      <= HYGRO_ADDR7;
                        cmd_wlen       <= 8'd1;
                        cmd_rlen       <= 8'd0;
                        cmd_repstart   <= 1'b0;
                        cmd_timeout_us <= CMD_TIMEOUT_US;
                        st_r           <= S_CMD;
                    end
                end

                S_CMD: begin
                    busy      <= 1'b1;
                    cmd_valid <= 1'b1;

                    if (op_r == OP_READ) begin
                        cmd_wlen     <= 8'd0;
                        cmd_rlen     <= 8'd4;
                        cmd_repstart <= 1'b0;
                    end

                    if (cmd_ready) begin
                        cmd_valid <= 1'b0;
                        if (op_r == OP_TRIGGER) begin
                            w_valid <= 1'b1;
                            w_data  <= REG_TEMP_RH;
                            w_last  <= 1'b1;
                            st_r    <= S_WRITE;
                        end else begin
                            ridx_r       <= 8'd0;
                            rd_len_err_r <= 1'b0;
                            done_code_r  <= 4'd0;
                            r_ready      <= 1'b1;
                            st_r         <= S_READ;
                        end
                    end
                end

                S_WRITE: begin
                    busy <= 1'b1;
                    if (w_valid && w_ready) begin
                        w_valid <= 1'b0;
                    end

                    if (!w_valid) begin
                        st_r <= S_READ;
                    end
                end

                S_READ: begin
                    busy <= 1'b1;
                    if (op_r == OP_READ)
                        r_ready <= 1'b1;

                    if (r_valid) begin
                        case (ridx_r)
                            8'd0: temp_msb_r <= r_data;
                            8'd1: temp_lsb_r <= r_data;
                            8'd2: rh_msb_r   <= r_data;
                            8'd3: rh_lsb_r   <= r_data;
                            default: begin end
                        endcase

                        ridx_r       <= ridx_r + 8'd1;
                        rd_len_err_r <= rd_len_err_now_w;

                        if (r_last)
                            r_ready <= 1'b0;
                    end

                    if (done) begin
                        r_ready     <= 1'b0;
                        done_code_r <= done_code;
                        if (op_r == OP_TRIGGER) begin
                            if (done_code != 4'd0) begin
                                st_r <= S_COMMIT;
                            end else begin
                                conv_start_us_r <= time_us;
                                st_r            <= S_CONV_WAIT;
                            end
                        end else begin
                            rd_len_err_r <= (done_code == 4'd0) ? rd_len_err_now_w
                                                               : rd_len_err_r;
                            st_r <= S_COMMIT;
                        end
                    end
                end

                S_CONV_WAIT: begin
                    busy <= 1'b1;
                    if ((time_us - conv_start_us_r) >= CONVERSION_WAIT_US) begin
                        if (grant) begin
                            op_r           <= OP_READ;
                            cmd_addr7      <= HYGRO_ADDR7;
                            cmd_wlen       <= 8'd0;
                            cmd_rlen       <= 8'd4;
                            cmd_repstart   <= 1'b0;
                            cmd_timeout_us <= CMD_TIMEOUT_US;
                            st_r           <= S_CMD;
                        end
                    end
                end

                S_COMMIT: begin
                    busy          <= 1'b0;
                    snap_commit   <= 1'b1;
                    snap_payload_in <= {temp_cdeg_w, rh_centi_w, 16'd0};

                    if (rd_len_err_r) begin
                        snap_valid_in  <= 1'b0;
                        snap_status_in <= STAT_READ_LENGTH;
                    end else if (done_code_r != 4'd0) begin
                        snap_valid_in  <= 1'b0;
                        snap_status_in <= STAT_I2C_ERROR | {4'd0, done_code_r};
                    end else begin
                        snap_valid_in    <= 1'b1;
                        snap_status_in   <= STAT_OK;
                        pending_sample_r <= 1'b0;
                        init_done        <= 1'b1;
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
