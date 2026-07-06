`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// lis3dh_job
//------------------------------------------------------------------------------
// Repaired LIS3DH job FSM.
//
// TRANSACTION MODEL
//   - grant authorizes launch.
//   - S_INIT_REQ / S_RD_REQ assert cmd_valid.
//   - Actual ownership begins on cmd_valid && cmd_ready.
//   - During read transactions, bytes are consumed before done.
//   - Snapshot commit occurs only after transaction completion.
//
// DEFAULT INIT SCRIPT
//   CTRL_REG1(0x20) = 0x57
//   CTRL_REG4(0x23) = 0xA8
//
// ADDRESS PROBE
//   - Primary address  : 0x18 (SA0 = 0)
//   - Alternate address: 0x19 (SA0 = 1)
//   - On any init-transaction failure, the FSM toggles to the alternate
//     address and restarts initialization from the first init pair.
//
// PERIODIC READ
//   pointer write: REG_OUT_X_L | 0x80
//   repeated-start read: 6 bytes => X_L,X_H,Y_L,Y_H,Z_L,Z_H
//==============================================================================
module lis3dh_job #(
    parameter [6:0]  DEV_ADDR7          = 7'h18,
    parameter [6:0]  DEV_ADDR7_ALT      = 7'h19,
    parameter [31:0] I2C_TIMEOUT_US     = 32'd2_000,

    parameter integer INIT_LEN           = 2,
    parameter [8*INIT_LEN-1:0] INIT_REGS = {8'h23, 8'h20},
    parameter [8*INIT_LEN-1:0] INIT_VALS = {8'hA8, 8'h57},

    parameter [7:0]  REG_OUT_X_L         = 8'h28,
    parameter [7:0]  READ_LEN            = 8'd6
)(
    input  wire        clk,
    input  wire        rst,
    input  wire [31:0] time_us,

    input  wire        epoch_100hz,
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
    input  wire        busy,

    output reg         snap_commit,
    output reg         snap_valid_in,
    output reg  [7:0]  snap_status_in,
    output reg  [47:0] snap_payload_in,

    output reg         init_done
);

    localparam [3:0]
        S_RESET        = 4'd0,
        S_INIT_REQ     = 4'd1,
        S_INIT_WB0     = 4'd2,
        S_INIT_WB1     = 4'd3,
        S_INIT_WAIT    = 4'd4,
        S_IDLE         = 4'd5,
        S_RD_REQ       = 4'd6,
        S_RD_WB0       = 4'd7,
        S_RD_STREAM    = 4'd8,
        S_RD_FINISH    = 4'd9,
        S_RD_COMMIT    = 4'd10
    ;

    localparam [7:0]
        STAT_OK                = 8'h00,
        STAT_ERR_SHORT_READ    = 8'hE1,
        STAT_ERR_LONG_READ     = 8'hE2;

    reg [3:0] st;

    reg [7:0] init_idx;
    reg [7:0] rd_idx;

    reg [15:0] ax;
    reg [15:0] ay;
    reg [15:0] az;

    reg [6:0] active_addr7_r;

    reg        rd_done_seen;
    reg [3:0]  rd_done_code_q;
    reg        rd_len_err;

    function [7:0] init_reg_at;
        input [7:0] idx;
        begin
            init_reg_at = INIT_REGS[8*idx +: 8];
        end
    endfunction

    function [7:0] init_val_at;
        input [7:0] idx;
        begin
            init_val_at = INIT_VALS[8*idx +: 8];
        end
    endfunction

    always @(posedge clk) begin
        if (rst) begin
            st             <= S_RESET;
            init_idx       <= 8'd0;
            init_done      <= 1'b0;

            rd_idx         <= 8'd0;
            ax             <= 16'd0;
            ay             <= 16'd0;
            az             <= 16'd0;

            active_addr7_r <= DEV_ADDR7;

            rd_done_seen   <= 1'b0;
            rd_done_code_q <= 4'h0;
            rd_len_err     <= 1'b0;
        end else begin
            case (st)
                //------------------------------------------------------------------
                // Reset / init
                //------------------------------------------------------------------
                S_RESET: begin
                    init_idx       <= 8'd0;
                    init_done      <= 1'b0;
                    rd_idx         <= 8'd0;
                    rd_done_seen   <= 1'b0;
                    rd_done_code_q <= 4'h0;
                    rd_len_err     <= 1'b0;
                    active_addr7_r <= DEV_ADDR7;
                    st             <= S_INIT_REQ;
                end

                S_INIT_REQ: begin
                    if (!busy && grant && cmd_ready) begin
                        st <= S_INIT_WB0;
                    end
                end

                S_INIT_WB0: begin
                    if (w_ready) begin
                        st <= S_INIT_WB1;
                    end
                end

                S_INIT_WB1: begin
                    if (w_ready) begin
                        st <= S_INIT_WAIT;
                    end
                end

                S_INIT_WAIT: begin
                    if (done) begin
                        if (done_code == 4'h0) begin
                            if (init_idx == (INIT_LEN-1)) begin
                                init_done <= 1'b1;
                                st        <= S_IDLE;
                            end else begin
                                init_idx  <= init_idx + 8'd1;
                                st        <= S_INIT_REQ;
                            end
                        end else begin
                            init_idx       <= 8'd0;
                            init_done      <= 1'b0;
                            active_addr7_r <=
                                (active_addr7_r == DEV_ADDR7) ? DEV_ADDR7_ALT : DEV_ADDR7;
                            st <= S_INIT_REQ;
                        end
                    end
                end

                //------------------------------------------------------------------
                // Idle / periodic read
                //------------------------------------------------------------------
                S_IDLE: begin
                    if (init_done && epoch_100hz) begin
                        rd_idx         <= 8'd0;
                        rd_done_seen   <= 1'b0;
                        rd_done_code_q <= 4'h0;
                        rd_len_err     <= 1'b0;
                        st             <= S_RD_REQ;
                    end
                end

                S_RD_REQ: begin
                    if (!busy && grant && cmd_ready) begin
                        rd_idx         <= 8'd0;
                        rd_done_seen   <= 1'b0;
                        rd_done_code_q <= 4'h0;
                        rd_len_err     <= 1'b0;
                        st             <= S_RD_WB0;
                    end
                end

                S_RD_WB0: begin
                    if (w_ready) begin
                        st <= S_RD_STREAM;
                    end
                end

                //------------------------------------------------------------------
                // Active read transaction:
                //   - consume r_valid bytes as they arrive
                //   - observe done independently
                //------------------------------------------------------------------
                S_RD_STREAM: begin
                    if (r_valid) begin
                        case (rd_idx)
                            3'd0: ax[7:0]   <= r_data;
                            3'd1: ax[15:8]  <= r_data;
                            3'd2: ay[7:0]   <= r_data;
                            3'd3: ay[15:8]  <= r_data;
                            3'd4: az[7:0]   <= r_data;
                            3'd5: az[15:8]  <= r_data;
                            default: rd_len_err <= 1'b1;
                        endcase

                        if (rd_idx < (READ_LEN-1)) begin
                            rd_idx <= rd_idx + 8'd1;
                        end else if (rd_idx == (READ_LEN-1)) begin
                            rd_idx <= READ_LEN;
                        end else begin
                            // Extra byte beyond expected payload.
                            rd_len_err <= 1'b1;
                        end
                    end

                    if (done) begin
                        rd_done_seen   <= 1'b1;
                        rd_done_code_q <= done_code;

                        if (rd_idx != READ_LEN) begin
                            if (READ_LEN != 0) begin
                                if (rd_idx != (READ_LEN-1) || !r_valid) begin
                                    rd_len_err <= 1'b1;
                                end
                            end
                        end

                        st <= S_RD_FINISH;
                    end
                end

                //------------------------------------------------------------------
                // One-cycle separation after final engine done pulse
                //------------------------------------------------------------------
                S_RD_FINISH: begin
                    st <= S_RD_COMMIT;
                end

                S_RD_COMMIT: begin
                    st <= S_IDLE;
                end

                default: begin
                    st <= S_RESET;
                end
            endcase
        end
    end

    always @(*) begin
        cmd_valid      = 1'b0;
        cmd_addr7      = active_addr7_r;
        cmd_wlen       = 8'd0;
        cmd_rlen       = 8'd0;
        cmd_repstart   = 1'b0;
        cmd_timeout_us = I2C_TIMEOUT_US;

        w_valid        = 1'b0;
        w_data         = 8'd0;
        w_last         = 1'b0;

        r_ready        = 1'b0;

        snap_commit     = 1'b0;
        snap_valid_in   = 1'b0;
        snap_status_in  = 8'd0;
        snap_payload_in = 48'd0;

        //----------------------------------------------------------------------
        // Init transaction:
        //   command is asserted in S_INIT_REQ
        //   payload bytes are driven in S_INIT_WB0 / S_INIT_WB1
        //----------------------------------------------------------------------
        if (st == S_INIT_REQ) begin
            cmd_valid    = 1'b1;
            cmd_wlen     = 8'd2;
            cmd_rlen     = 8'd0;
            cmd_repstart = 1'b0;
        end

        if (st == S_INIT_WB0) begin
            w_valid = 1'b1;
            w_data  = init_reg_at(init_idx);
            w_last  = 1'b0;
        end

        if (st == S_INIT_WB1) begin
            w_valid = 1'b1;
            w_data  = init_val_at(init_idx);
            w_last  = 1'b1;
        end

        //----------------------------------------------------------------------
        // Read transaction:
        //   command is asserted in S_RD_REQ
        //   pointer byte is driven in S_RD_WB0
        //   read bytes are consumed in S_RD_STREAM
        //----------------------------------------------------------------------
        if (st == S_RD_REQ) begin
            cmd_valid    = 1'b1;
            cmd_wlen     = 8'd1;
            cmd_rlen     = READ_LEN;
            cmd_repstart = 1'b1;
        end

        if (st == S_RD_WB0) begin
            w_valid = 1'b1;
            w_data  = REG_OUT_X_L | 8'h80;
            w_last  = 1'b1;
        end

        if (st == S_RD_STREAM) begin
            r_ready = 1'b1;
        end

        //----------------------------------------------------------------------
        // Snapshot commit
        //----------------------------------------------------------------------
        if (st == S_RD_COMMIT) begin
            snap_commit     = 1'b1;
            snap_payload_in = {ax, ay, az};

            if (rd_len_err) begin
                snap_valid_in  = 1'b0;
                snap_status_in = STAT_ERR_SHORT_READ;
            end else if (rd_done_code_q != 4'h0) begin
                snap_valid_in  = 1'b0;
                snap_status_in = {4'h0, rd_done_code_q};
            end else begin
                snap_valid_in  = 1'b1;
                snap_status_in = STAT_OK;
            end
        end
    end

endmodule

`default_nettype wire
