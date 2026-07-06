`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// bmp585_job
//------------------------------------------------------------------------------
// ROLE
//   Bosch-authoritative BMP585 bring-up + acquisition FSM.
//
// CONTRACT
//   - Matches the job interface already used by rocket_i2c_suite_top.
//   - Publishes one 48-bit raw snapshot payload per committed sample.
//   - Uses repeated-start register pointer + burst read transactions.
//   - Does not issue CMD soft reset over I2C because BMP585 does not ACK
//     the 0xB6 soft-reset command on I2C.
//   - Probes both legal I2C addresses during bring-up:
//       primary   0x47
//       alternate 0x46
//
// AUTHORITATIVE REGISTER CHOICES
//   CHIP_ID      = 0x01, expect 0x51
//   CHIP_STATUS  = 0x11
//   INT_CONFIG   = 0x14
//   INT_SOURCE   = 0x15
//   FIFO_SEL     = 0x18
//   TEMP_XLSB    = 0x1D
//   PRESS_XLSB   = 0x20
//   OSR_CONFIG   = 0x36
//   ODR_CONFIG   = 0x37
//
// CONFIG POLICY
//   - FIFO disabled
//   - Data-ready interrupt source enabled
//   - INT configured pulsed / active-high / push-pull / enabled
//   - OSR_CONFIG:
//       osr_t   = 2x
//       osr_p   = 8x
//       press_en= 1
//     => 0x59
//   - ODR_CONFIG:
//       deep_dis = 1
//       odr      = 50 Hz (0x0F)
//       pwr_mode = NORMAL (0x1)
//     => 0xBD
//
// PERIODIC READ
//   Burst from 0x1D for 6 bytes:
//     TEMP_XLSB, TEMP_LSB, TEMP_MSB, PRESS_XLSB, PRESS_LSB, PRESS_MSB
//
// SNAPSHOT PAYLOAD
//   {press_msb, press_lsb, press_xlsb, temp_msb, temp_lsb, temp_xlsb}
//==============================================================================
module bmp585_job #(
    parameter [6:0]  BMP585_ADDR7     = 7'h47,
    parameter [6:0]  BMP585_ADDR7_ALT = 7'h46,
    parameter [31:0] CMD_TIMEOUT_US   = 32'd4000,
    parameter [31:0] POWERUP_WAIT_US  = 32'd3000
)(
    input  wire        clk,
    input  wire        rst,
    input  wire [31:0] time_us,
    input  wire        epoch_50hz,
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
        OP_WHO      = 3'd0,
        OP_FIFO_SEL = 3'd1,
        OP_INT_CFG  = 3'd2,
        OP_INT_SRC  = 3'd3,
        OP_OSR_CFG  = 3'd4,
        OP_ODR_CFG  = 3'd5,
        OP_READ     = 3'd6;

    localparam [2:0]
        ST_IDLE      = 3'd0,
        ST_CMD       = 3'd1,
        ST_W_STREAM  = 3'd2,
        ST_WAIT_DONE = 3'd3;

    localparam [7:0] REG_CHIP_ID      = 8'h01;
    localparam [7:0] REG_CHIP_STATUS  = 8'h11;
    localparam [7:0] REG_INT_CONFIG   = 8'h14;
    localparam [7:0] REG_INT_SOURCE   = 8'h15;
    localparam [7:0] REG_FIFO_SEL     = 8'h18;
    localparam [7:0] REG_TEMP_XLSB    = 8'h1D;
    localparam [7:0] REG_OSR_CONFIG   = 8'h36;
    localparam [7:0] REG_ODR_CONFIG   = 8'h37;

    localparam [7:0] CHIP_ID_EXPECT   = 8'h51;
    localparam [7:0] STAT_OK          = 8'h00;
    localparam [7:0] STAT_NOT_INIT    = 8'h01;

    // INT_CONFIG: pulsed=0, active-high=1<<1, push-pull=0<<2, int_en=1<<3
    localparam [7:0] INT_CONFIG_VALUE = 8'h0A;

    // INT_SOURCE: drdy_data_reg_en = bit0
    localparam [7:0] INT_SOURCE_VALUE = 8'h01;

    // FIFO_SEL: FIFO disabled
    localparam [7:0] FIFO_SEL_VALUE   = 8'h00;

    // OSR_CONFIG:
    //   osr_t     = 2x  -> 0x1 at bits [2:0]
    //   osr_p     = 8x  -> 0x3 at bits [5:3]
    //   press_en  = 1   -> bit 6
    // => 0x59
    localparam [7:0] OSR_CONFIG_VALUE = 8'h59;

    // ODR_CONFIG:
    //   pwr_mode  = normal -> 0x1 at bits [1:0]
    //   odr       = 50Hz   -> 0x0F at bits [6:2]
    //   deep_dis  = 1      -> bit 7
    // => 0x80 + (0x0F<<2) + 0x01 = 0xBD
    localparam [7:0] ODR_CONFIG_VALUE = 8'hBD;

    reg [2:0] op_r;
    reg [2:0] st_r;

    reg       pending_sample_r;
    reg       cmd_armed_r;
    reg [7:0] widx_r;
    reg [7:0] ridx_r;

    reg [7:0] chip_id_r;
    reg [7:0] chip_status_r;

    reg [7:0] temp_xlsb_r;
    reg [7:0] temp_lsb_r;
    reg [7:0] temp_msb_r;
    reg [7:0] press_xlsb_r;
    reg [7:0] press_lsb_r;
    reg [7:0] press_msb_r;

    reg [6:0] active_addr7_r;

    function [7:0] op_reg_addr;
        input [2:0] op;
        begin
            case (op)
                OP_WHO:      op_reg_addr = REG_CHIP_ID;
                OP_FIFO_SEL: op_reg_addr = REG_FIFO_SEL;
                OP_INT_CFG:  op_reg_addr = REG_INT_CONFIG;
                OP_INT_SRC:  op_reg_addr = REG_INT_SOURCE;
                OP_OSR_CFG:  op_reg_addr = REG_OSR_CONFIG;
                OP_ODR_CFG:  op_reg_addr = REG_ODR_CONFIG;
                default:     op_reg_addr = REG_TEMP_XLSB;
            endcase
        end
    endfunction

    function [7:0] op_reg_data;
        input [2:0] op;
        begin
            case (op)
                OP_FIFO_SEL: op_reg_data = FIFO_SEL_VALUE;
                OP_INT_CFG:  op_reg_data = INT_CONFIG_VALUE;
                OP_INT_SRC:  op_reg_data = INT_SOURCE_VALUE;
                OP_OSR_CFG:  op_reg_data = OSR_CONFIG_VALUE;
                OP_ODR_CFG:  op_reg_data = ODR_CONFIG_VALUE;
                default:     op_reg_data = 8'h00;
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
            cmd_addr7      <= BMP585_ADDR7;
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
            snap_status_in  <= STAT_NOT_INIT;
            snap_payload_in <= 48'd0;

            init_done <= 1'b0;

            widx_r <= 8'd0;
            ridx_r <= 8'd0;

            chip_id_r     <= 8'd0;
            chip_status_r <= 8'd0;

            temp_xlsb_r  <= 8'd0;
            temp_lsb_r   <= 8'd0;
            temp_msb_r   <= 8'd0;
            press_xlsb_r <= 8'd0;
            press_lsb_r  <= 8'd0;
            press_msb_r  <= 8'd0;
            active_addr7_r <= BMP585_ADDR7;
        end else begin
            snap_commit <= 1'b0;

            if (epoch_50hz)
                pending_sample_r <= 1'b1;

            case (st_r)
                ST_IDLE: begin
                    busy      <= 1'b0;
                    cmd_valid <= 1'b0;
                    w_valid   <= 1'b0;
                    r_ready   <= 1'b0;

                    if ((time_us >= POWERUP_WAIT_US) &&
                        grant &&
                        (!init_done || pending_sample_r)) begin
                        busy           <= 1'b1;
                        cmd_addr7      <= active_addr7_r;
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

                    if (!cmd_armed_r) begin
                        cmd_armed_r <= 1'b1;
                    end else if (cmd_ready) begin
                        cmd_valid   <= 1'b0;
                        cmd_armed_r <= 1'b0;
                        w_valid     <= 1'b1;

                        if ((op_r == OP_WHO) || (op_r == OP_READ)) begin
                            w_data <= op_reg_addr(op_r);
                            w_last <= 1'b1;
                            widx_r <= 8'd0;
                        end else begin
                            w_data <= op_reg_addr(op_r);
                            w_last <= 1'b0;
                            widx_r <= 8'd1;
                        end

                        st_r      <= ST_W_STREAM;
                    end
                end

                ST_W_STREAM: begin
                    busy <= 1'b1;

                    if (w_valid && w_ready) begin
                        if ((op_r == OP_WHO) || (op_r == OP_READ)) begin
                            w_valid <= 1'b0;
                        end else begin
                            if (widx_r == 8'd1) begin
                                w_data <= op_reg_data(op_r);
                                w_last <= 1'b1;
                                widx_r <= 8'd2;
                            end else begin
                                w_valid <= 1'b0;
                            end
                        end
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
                                    chip_id_r <= r_data;
                                else
                                    temp_xlsb_r <= r_data;
                            end
                            8'd1: temp_lsb_r   <= r_data;
                            8'd2: temp_msb_r   <= r_data;
                            8'd3: press_xlsb_r <= r_data;
                            8'd4: press_lsb_r  <= r_data;
                            8'd5: press_msb_r  <= r_data;
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
                            snap_status_in <= 8'hE0 | {4'd0, done_code};
                            if (!init_done) begin
                                if (op_r == OP_WHO) begin
                                    active_addr7_r <=
                                        (active_addr7_r == BMP585_ADDR7) ? BMP585_ADDR7_ALT : BMP585_ADDR7;
                                end
                                op_r <= OP_WHO;
                            end
                            st_r           <= ST_IDLE;
                        end else begin
                            case (op_r)
                                OP_WHO: begin
                                    if (chip_id_r == CHIP_ID_EXPECT) begin
                                        op_r        <= OP_FIFO_SEL;
                                        cmd_armed_r <= 1'b0;
                                        st_r        <= ST_CMD;
                                    end else begin
                                        busy          <= 1'b0;
                                        snap_valid_in  <= 1'b0;
                                        snap_status_in <= 8'hE3;
                                        active_addr7_r <=
                                            (active_addr7_r == BMP585_ADDR7) ? BMP585_ADDR7_ALT : BMP585_ADDR7;
                                        op_r           <= OP_WHO;
                                        st_r           <= ST_IDLE;
                                    end
                                end

                                OP_FIFO_SEL: begin
                                    op_r        <= OP_INT_CFG;
                                    cmd_armed_r <= 1'b0;
                                    st_r        <= ST_CMD;
                                end

                                OP_INT_CFG: begin
                                    op_r        <= OP_INT_SRC;
                                    cmd_armed_r <= 1'b0;
                                    st_r        <= ST_CMD;
                                end

                                OP_INT_SRC: begin
                                    op_r        <= OP_OSR_CFG;
                                    cmd_armed_r <= 1'b0;
                                    st_r        <= ST_CMD;
                                end

                                OP_OSR_CFG: begin
                                    op_r        <= OP_ODR_CFG;
                                    cmd_armed_r <= 1'b0;
                                    st_r        <= ST_CMD;
                                end

                                OP_ODR_CFG: begin
                                    busy      <= 1'b0;
                                    init_done <= 1'b1;
                                    st_r      <= ST_IDLE;
                                end

                                default: begin
                                    busy             <= 1'b0;
                                    pending_sample_r <= 1'b0;
                                    snap_commit      <= 1'b1;
                                    snap_valid_in    <= 1'b1;
                                    snap_status_in   <= STAT_OK;
                                    snap_payload_in  <= {
                                        press_msb_r,
                                        press_lsb_r,
                                        press_xlsb_r,
                                        temp_msb_r,
                                        temp_lsb_r,
                                        temp_xlsb_r
                                    };
                                    st_r <= ST_IDLE;
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

`default_nettype wire
