`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// i2c_master_engine
//------------------------------------------------------------------------------
// ROLE
//   Shared open-drain I2C master engine.
//
// PURPOSE
//   Execute one transaction at a time on behalf of the upstream job mux.
//
// SUPPORTED FORMS
//   1) Write-only:
//        ST  SAD+W  W0 W1 ... WN  SP
//
//   2) Write-then-read with repeated-start:
//        ST  SAD+W  W0 W1 ... WK  SR  SAD+R  R0 R1 ... RM  SP
//
// LINE DISCIPLINE
//   - Open-drain drive only.
//   - drive-low => line = 0
//   - release   => external pull-up makes line = 1
//
// TIMING MODEL
//   - One bit is executed across four quarter-phases:
//       Q0: SCL low, SDA may change
//       Q1: SCL low, hold/setup
//       Q2: SCL released high, sample zone
//       Q3: SCL high, hold
//
// LIMITATIONS
//   - No clock stretching support in this revision.
//   - No arbitration-loss handling in this revision.
//==============================================================================
module i2c_master_engine #(
    parameter integer CLK_HZ     = 100_000_000,
    parameter integer I2C_HZ     = 100_000,
    parameter integer MAX_WBYTES = 8,
    parameter integer MAX_RBYTES = 16
)(
    input  wire        clk,
    input  wire        rst,

    //--------------------------------------------------------------------------
    // Command channel
    //--------------------------------------------------------------------------
    input  wire        cmd_valid,
    output reg         cmd_ready,
    input  wire [6:0]  cmd_addr7,
    input  wire [7:0]  cmd_wlen,
    input  wire [7:0]  cmd_rlen,
    input  wire        cmd_repstart,
    input  wire [31:0] cmd_timeout_us,

    //--------------------------------------------------------------------------
    // Write preload channel
    //--------------------------------------------------------------------------
    input  wire        w_valid,
    output reg         w_ready,
    input  wire [7:0]  w_data,
    input  wire        w_last,

    //--------------------------------------------------------------------------
    // Read return channel
    //--------------------------------------------------------------------------
    output reg         r_valid,
    input  wire        r_ready,
    output reg  [7:0]  r_data,
    output reg         r_last,

    //--------------------------------------------------------------------------
    // Completion
    //--------------------------------------------------------------------------
    output reg         done,
    output reg  [3:0]  done_code,
    output reg         busy,

    //--------------------------------------------------------------------------
    // 1-us tick from suite timebase
    //--------------------------------------------------------------------------
    input  wire        tick_1us,

    //--------------------------------------------------------------------------
    // Physical bus
    //--------------------------------------------------------------------------
    output wire       scl,
    inout  wire        sda
);

    //--------------------------------------------------------------------------
    // Done codes
    //--------------------------------------------------------------------------
    localparam [3:0] DONE_OK              = 4'd0;
    localparam [3:0] DONE_NACK_ADDR_W     = 4'd1;
    localparam [3:0] DONE_NACK_DATA_W     = 4'd2;
    localparam [3:0] DONE_TIMEOUT_PRELOAD = 4'd3;
    localparam [3:0] DONE_LENGTH_ERR      = 4'd4;
    localparam [3:0] DONE_NACK_ADDR_R     = 4'd5;
    localparam [3:0] DONE_TIMEOUT_BUS     = 4'd6;

    //--------------------------------------------------------------------------
    // Quarter-period divider
    //--------------------------------------------------------------------------
    localparam integer QTR_DIV = (CLK_HZ / (I2C_HZ * 4));

    //--------------------------------------------------------------------------
    // Top-level states
    //--------------------------------------------------------------------------
    localparam [3:0]
        S_IDLE        = 4'd0,
        S_PRELOAD     = 4'd1,
        S_EXEC        = 4'd2,
        S_STREAM_READ = 4'd3,
        S_DONE        = 4'd4,
        S_FAIL        = 4'd5;

    reg [3:0] state;

    //--------------------------------------------------------------------------
    // Transaction script states
    //--------------------------------------------------------------------------
    localparam [3:0]
        T_START       = 4'd0,
        T_ADDR_W      = 4'd1,
        T_WRITE_BYTES = 4'd2,
        T_RESTART     = 4'd3,
        T_ADDR_R      = 4'd4,
        T_READ_BYTES  = 4'd5,
        T_STOP        = 4'd6,
        T_FINISH      = 4'd7;

    reg [3:0] tstate;

    //--------------------------------------------------------------------------
    // Micro-op kinds
    //--------------------------------------------------------------------------
    localparam [2:0]
        U_NONE        = 3'd0,
        U_START       = 3'd1,
        U_RESTART     = 3'd2,
        U_STOP        = 3'd3,
        U_TX_BYTE     = 3'd4,
        U_TX_ACK      = 3'd5,
        U_RX_BYTE     = 3'd6,
        U_RX_MASTER   = 3'd7;

    reg [2:0] uop;

    //--------------------------------------------------------------------------
    // Quarter-phase stepping
    //--------------------------------------------------------------------------
    reg [15:0] div_ctr;
    reg [1:0]  qphase;
    reg        step_q;

    //--------------------------------------------------------------------------
    // Open-drain drivers
    //--------------------------------------------------------------------------
    reg scl_drv_low;
    reg sda_drv_low;

    wire scl_in;
    wire sda_in;

    assign scl    = scl_drv_low ? 1'b0 : 1'bz;
    assign sda    = sda_drv_low ? 1'b0 : 1'bz;
    assign scl_in = scl;
    assign sda_in = sda;

    //--------------------------------------------------------------------------
    // Latched command
    //--------------------------------------------------------------------------
    reg [6:0]  addr7_r;
    reg [7:0]  wlen_r;
    reg [7:0]  rlen_r;
    reg        repstart_r;
    reg [31:0] timeout_us_r;

    //--------------------------------------------------------------------------
    // Write/read buffers
    //--------------------------------------------------------------------------
    reg [7:0] wbuf [0:MAX_WBYTES-1];
    reg [7:0] rbuf [0:MAX_RBYTES-1];

    reg [7:0] preload_count;
    reg [7:0] tx_index;
    reg [7:0] rx_index;
    reg [7:0] rstream_index;

    //--------------------------------------------------------------------------
    // Byte machinery
    //--------------------------------------------------------------------------
    reg [7:0] tx_byte;
    reg [7:0] rx_shift;
    reg [2:0] bit_index;
    reg       ack_ok;

    //--------------------------------------------------------------------------
    // Timeout counters
    //--------------------------------------------------------------------------
    reg [31:0] preload_timeout_ctr;
    reg [31:0] bus_timeout_ctr;

    integer i;

    //--------------------------------------------------------------------------
    // Quarter-phase generator
    //--------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            div_ctr <= 16'd0;
            qphase  <= 2'd0;
            step_q  <= 1'b0;
        end else begin
            step_q <= 1'b0;

            if (state == S_EXEC) begin
                // Hold the micro-phase machine at phase 0 until a micro-op
                // has been launched by the main FSM. This preserves the
                // original intent of forcing each new micro-op to start from
                // qphase = 0, but without giving qphase a second writer.
                if (uop == U_NONE) begin
                    div_ctr <= 16'd0;
                    qphase  <= 2'd0;
                end else if (div_ctr == (QTR_DIV - 1)) begin
                    div_ctr <= 16'd0;
                    qphase  <= qphase + 2'd1;
                    step_q  <= 1'b1;
                end else begin
                    div_ctr <= div_ctr + 16'd1;
                end
            end else begin
                div_ctr <= 16'd0;
                qphase  <= 2'd0;
            end
        end
    end


    //--------------------------------------------------------------------------
    // Main FSM
    //--------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            state               <= S_IDLE;
            tstate              <= T_START;
            uop                 <= U_NONE;

            cmd_ready           <= 1'b1;
            w_ready             <= 1'b0;
            r_valid             <= 1'b0;
            r_data              <= 8'h00;
            r_last              <= 1'b0;
            done                <= 1'b0;
            done_code           <= DONE_OK;
            busy                <= 1'b0;

            addr7_r             <= 7'd0;
            wlen_r              <= 8'd0;
            rlen_r              <= 8'd0;
            repstart_r          <= 1'b0;
            timeout_us_r        <= 32'd0;

            preload_count       <= 8'd0;
            tx_index            <= 8'd0;
            rx_index            <= 8'd0;
            rstream_index       <= 8'd0;

            tx_byte             <= 8'h00;
            rx_shift            <= 8'h00;
            bit_index           <= 3'd7;
            ack_ok              <= 1'b0;

            preload_timeout_ctr <= 32'd0;
            bus_timeout_ctr     <= 32'd0;

            scl_drv_low         <= 1'b0;
            sda_drv_low         <= 1'b0;

            for (i = 0; i < MAX_WBYTES; i = i + 1)
                wbuf[i] <= 8'h00;

            for (i = 0; i < MAX_RBYTES; i = i + 1)
                rbuf[i] <= 8'h00;

        end else begin
            cmd_ready <= 1'b0;
            w_ready   <= 1'b0;
            r_valid   <= 1'b0;
            r_last    <= 1'b0;
            done      <= 1'b0;

            if ((state == S_PRELOAD) && tick_1us)
                preload_timeout_ctr <= preload_timeout_ctr + 32'd1;

            if ((state == S_EXEC) && tick_1us)
                bus_timeout_ctr <= bus_timeout_ctr + 32'd1;

            case (state)
                //------------------------------------------------------------------
                // Idle / command accept
                //------------------------------------------------------------------
                S_IDLE: begin
                    busy                <= 1'b0;
                    cmd_ready           <= 1'b1;
                    scl_drv_low         <= 1'b0;
                    sda_drv_low         <= 1'b0;
                    preload_timeout_ctr <= 32'd0;
                    bus_timeout_ctr     <= 32'd0;

                    if (cmd_valid) begin
                        if ((cmd_wlen > MAX_WBYTES[7:0]) || (cmd_rlen > MAX_RBYTES[7:0])) begin
                            done_code <= DONE_LENGTH_ERR;
                            state     <= S_FAIL;
                        end else begin
                            busy         <= 1'b1;
                            addr7_r      <= cmd_addr7;
                            wlen_r       <= cmd_wlen;
                            rlen_r       <= cmd_rlen;
                            repstart_r   <= cmd_repstart;
                            timeout_us_r <= cmd_timeout_us;

                            preload_count       <= 8'd0;
                            tx_index            <= 8'd0;
                            rx_index            <= 8'd0;
                            rstream_index       <= 8'd0;
                            bit_index           <= 3'd7;
                            ack_ok              <= 1'b0;
                            tstate              <= T_START;
                            uop                 <= U_NONE;
                            preload_timeout_ctr <= 32'd0;
                            bus_timeout_ctr     <= 32'd0;

                            if (cmd_wlen != 8'd0)
                                state <= S_PRELOAD;
                            else
                                state <= S_EXEC;
                        end
                    end
                end

                //------------------------------------------------------------------
                // Collect write bytes before bus launch
                //------------------------------------------------------------------
                S_PRELOAD: begin
                    busy    <= 1'b1;
                    w_ready <= 1'b1;

                    if ((timeout_us_r != 32'd0) && (preload_timeout_ctr >= timeout_us_r)) begin
                        done_code <= DONE_TIMEOUT_PRELOAD;
                        state     <= S_FAIL;
                    end else if (w_valid) begin
                        wbuf[preload_count] <= w_data;
                        preload_count       <= preload_count + 8'd1;

                        if (preload_count + 8'd1 >= wlen_r)
                            state <= S_EXEC;
                    end
                end

                //------------------------------------------------------------------
                // Bus execution
                //------------------------------------------------------------------
                S_EXEC: begin
                    busy <= 1'b1;

                    if ((timeout_us_r != 32'd0) && (bus_timeout_ctr >= timeout_us_r)) begin
                        done_code   <= DONE_TIMEOUT_BUS;
                        scl_drv_low <= 1'b0;
                        sda_drv_low <= 1'b0;
                        state       <= S_FAIL;
                    end else if (uop == U_NONE) begin
                        case (tstate)
                            T_START: begin
                            uop    <= U_START;
                        end

                        T_ADDR_W: begin
                            uop         <= U_TX_BYTE;
                            bit_index   <= 3'd7;
                            tx_byte     <= {addr7_r,1'b0};
                            scl_drv_low <= 1'b1;
                            sda_drv_low <= ~addr7_r[6];
                        end

                        T_WRITE_BYTES: begin
                            uop         <= U_TX_BYTE;
                            bit_index   <= 3'd7;
                            tx_byte     <= wbuf[tx_index];
                            scl_drv_low <= 1'b1;
                            sda_drv_low <= ~wbuf[tx_index][7];
                        end

                        T_RESTART: begin
                            uop    <= U_RESTART;
                        end

                        T_ADDR_R: begin
                            uop         <= U_TX_BYTE;
                            bit_index   <= 3'd7;
                            tx_byte     <= {addr7_r,1'b1};
                            scl_drv_low <= 1'b1;
                            sda_drv_low <= ~addr7_r[6];
                        end

                        T_READ_BYTES: begin
                            uop         <= U_RX_BYTE;
                            bit_index   <= 3'd7;
                            rx_shift    <= 8'h00;
                            scl_drv_low <= 1'b1;
                            sda_drv_low <= 1'b0;
                        end

                        T_STOP: begin
                            uop    <= U_STOP;
                        end

                        T_FINISH: begin
                            if (rlen_r != 8'd0) begin
                                rstream_index <= 8'd0;
                                state         <= S_STREAM_READ;
                            end else begin
                                state         <= S_DONE;
                            end
                        end

                        default: begin
                            tstate <= T_START;
                        end
                        endcase
                    end else if (step_q) begin
                        case (uop)
                            //------------------------------------------------------
                            // START
                            //
                            // Existing jobs use write-only or write-then-read
                            // scripts. HYGRO/HDC1080-class devices need a
                            // clean read-only transaction after a conversion
                            // delay, so a zero-write/nonzero-read command
                            // proceeds directly to SAD+R instead of issuing an
                            // empty SAD+W phase first.
                            //------------------------------------------------------
                            U_START: begin
                                case (qphase)
                                    2'd0: begin
                                        scl_drv_low <= 1'b0;
                                        sda_drv_low <= 1'b0;
                                    end
                                    2'd1: begin
                                        scl_drv_low <= 1'b0;
                                        sda_drv_low <= 1'b1;
                                    end
                                    2'd2: begin
                                        scl_drv_low <= 1'b1;
                                        sda_drv_low <= 1'b1;
                                    end
                                    2'd3: begin
                                        uop    <= U_NONE;
                                        if ((wlen_r == 8'd0) && (rlen_r != 8'd0))
                                            tstate <= T_ADDR_R;
                                        else
                                            tstate <= T_ADDR_W;
                                    end
                                endcase
                            end

                            //------------------------------------------------------
                            // REPEATED START
                            //------------------------------------------------------
                            U_RESTART: begin
                                case (qphase)
                                    2'd0: begin
                                        scl_drv_low <= 1'b1;
                                        sda_drv_low <= 1'b0;
                                    end
                                    2'd1: begin
                                        scl_drv_low <= 1'b0;
                                        sda_drv_low <= 1'b0;
                                    end
                                    2'd2: begin
                                        scl_drv_low <= 1'b0;
                                        sda_drv_low <= 1'b1;
                                    end
                                    2'd3: begin
                                        scl_drv_low <= 1'b1;
                                        sda_drv_low <= 1'b1;
                                        uop         <= U_NONE;
                                        tstate      <= T_ADDR_R;
                                    end
                                endcase
                            end

                            //------------------------------------------------------
                            // STOP
                            //------------------------------------------------------
                            U_STOP: begin
                                case (qphase)
                                    2'd0: begin
                                        scl_drv_low <= 1'b1;
                                        sda_drv_low <= 1'b1;
                                    end
                                    2'd1: begin
                                        scl_drv_low <= 1'b0;
                                        sda_drv_low <= 1'b1;
                                    end
                                    2'd2: begin
                                        scl_drv_low <= 1'b0;
                                        sda_drv_low <= 1'b0;
                                    end
                                    2'd3: begin
                                        uop    <= U_NONE;
                                        tstate <= T_FINISH;
                                    end
                                endcase
                            end

                            //------------------------------------------------------
                            // TX byte bits
                            //------------------------------------------------------
                            U_TX_BYTE: begin
                                case (qphase)
                                    2'd0: begin
                                        scl_drv_low <= 1'b1;
                                        sda_drv_low <= ~tx_byte[bit_index];
                                    end
                                    2'd1: begin
                                        scl_drv_low <= 1'b1;
                                    end
                                    2'd2: begin
                                        scl_drv_low <= 1'b0;
                                    end
                                    2'd3: begin
                                        if (bit_index != 3'd0) begin
                                            bit_index <= bit_index - 3'd1;
                                        end else begin
                                            uop      <= U_TX_ACK;
                                            bit_index<= 3'd7;
                                        end
                                    end
                                endcase
                            end

                            //------------------------------------------------------
                            // Sample slave ACK after transmitted byte
                            //------------------------------------------------------
                            U_TX_ACK: begin
                                case (qphase)
                                    2'd0: begin
                                        scl_drv_low <= 1'b1;
                                        sda_drv_low <= 1'b0; // release SDA
                                    end
                                    2'd1: begin
                                        scl_drv_low <= 1'b1;
                                    end
                                    2'd2: begin
                                        scl_drv_low <= 1'b0;
                                        ack_ok      <= (sda_in == 1'b0);
                                    end
                                    2'd3: begin
                                        scl_drv_low <= 1'b1;
                                        uop         <= U_NONE;

                                        case (tstate)
                                            T_ADDR_W: begin
                                                if (!ack_ok) begin
                                                    done_code <= DONE_NACK_ADDR_W;
                                                    state     <= S_FAIL;
                                                end else if (wlen_r != 8'd0) begin
                                                    tx_index <= 8'd0;
                                                    tstate   <= T_WRITE_BYTES;
                                                end else if (rlen_r != 8'd0) begin
                                                    if (repstart_r)
                                                        tstate <= T_RESTART;
                                                    else
                                                        tstate <= T_STOP;
                                                end else begin
                                                    tstate <= T_STOP;
                                                end
                                            end

                                            T_WRITE_BYTES: begin
                                                if (!ack_ok) begin
                                                    done_code <= DONE_NACK_DATA_W;
                                                    state     <= S_FAIL;
                                                end else if (tx_index + 8'd1 < wlen_r) begin
                                                    tx_index <= tx_index + 8'd1;
                                                    tstate   <= T_WRITE_BYTES;
                                                end else if (rlen_r != 8'd0) begin
                                                    if (repstart_r)
                                                        tstate <= T_RESTART;
                                                    else
                                                        tstate <= T_STOP;
                                                end else begin
                                                    tstate <= T_STOP;
                                                end
                                            end

                                            T_ADDR_R: begin
                                                if (!ack_ok) begin
                                                    done_code <= DONE_NACK_ADDR_R;
                                                    state     <= S_FAIL;
                                                end else begin
                                                    rx_index <= 8'd0;
                                                    tstate   <= T_READ_BYTES;
                                                end
                                            end

                                            default: begin
                                                tstate <= T_STOP;
                                            end
                                        endcase
                                    end
                                endcase
                            end

                            //------------------------------------------------------
                            // RX byte bits
                            //------------------------------------------------------
                            U_RX_BYTE: begin
                                case (qphase)
                                    2'd0: begin
                                        scl_drv_low <= 1'b1;
                                        sda_drv_low <= 1'b0; // release for slave data
                                    end
                                    2'd1: begin
                                        scl_drv_low <= 1'b1;
                                    end
                                    2'd2: begin
                                        scl_drv_low        <= 1'b0;
                                        rx_shift[bit_index]<= sda_in;
                                    end
                                    2'd3: begin
                                        if (bit_index != 3'd0) begin
                                            bit_index <= bit_index - 3'd1;
                                        end else begin
                                            rbuf[rx_index] <= rx_shift;
                                            uop            <= U_RX_MASTER;
                                        end
                                    end
                                endcase
                            end

                            //------------------------------------------------------
                            // Master ACK/NACK after received byte
                            //------------------------------------------------------
                            U_RX_MASTER: begin
                                case (qphase)
                                    2'd0: begin
                                        scl_drv_low <= 1'b1;
                                        // ACK all but last byte; NACK final byte
                                        if (rx_index + 8'd1 < rlen_r)
                                            sda_drv_low <= 1'b1;
                                        else
                                            sda_drv_low <= 1'b0;
                                    end
                                    2'd1: begin
                                        scl_drv_low <= 1'b1;
                                    end
                                    2'd2: begin
                                        scl_drv_low <= 1'b0;
                                    end
                                    2'd3: begin
                                        scl_drv_low <= 1'b1;
                                        sda_drv_low <= 1'b0;
                                        uop         <= U_NONE;

                                        if (rx_index + 8'd1 < rlen_r) begin
                                            rx_index <= rx_index + 8'd1;
                                            tstate   <= T_READ_BYTES;
                                        end else begin
                                            tstate   <= T_STOP;
                                        end
                                    end
                                endcase
                            end

                            default: begin
                                uop <= U_NONE;
                            end
                        endcase
                    end
                end

                //------------------------------------------------------------------
                // Return buffered read bytes to caller
                //------------------------------------------------------------------
                S_STREAM_READ: begin
                    busy   <= 1'b1;
                    r_valid<= 1'b1;
                    r_data <= rbuf[rstream_index];
                    r_last <= (rstream_index + 8'd1 >= rlen_r);

                    if (r_ready) begin
                        if (rstream_index + 8'd1 >= rlen_r)
                            state <= S_DONE;
                        else
                            rstream_index <= rstream_index + 8'd1;
                    end
                end

                //------------------------------------------------------------------
                // Success
                //------------------------------------------------------------------
                S_DONE: begin
                    busy      <= 1'b0;
                    done      <= 1'b1;
                    done_code <= DONE_OK;
                    state     <= S_IDLE;
                end

                //------------------------------------------------------------------
                // Failure
                //------------------------------------------------------------------
                S_FAIL: begin
                    busy        <= 1'b0;
                    done        <= 1'b1;
                    scl_drv_low <= 1'b0;
                    sda_drv_low <= 1'b0;
                    state       <= S_IDLE;
                end

                default: begin
                    state <= S_IDLE;
                end
            endcase
        end
    end

endmodule

`default_nettype wire
