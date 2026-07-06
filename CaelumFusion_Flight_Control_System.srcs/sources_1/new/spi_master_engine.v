`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// spi_master_engine
//------------------------------------------------------------------------------
// Shared SPI master engine for the CaelumFusion sensor stack.
//
// CURRENT WIRE PROTOCOL
//   - SPI mode 3 only:
//       * SCLK idles high
//       * outputs change on falling edges
//       * inputs sample on rising edges
//   - One transaction at a time, selected by cmd_cs_sel.
//   - Optional 3-wire data phase for LIS2MDL via the dedicated SDIO pin.
//
// TRANSACTION MODEL
//   1) Upstream presents command with write and read lengths.
//   2) Engine preloads write bytes from the job before asserting CS.
//   3) Engine clocks out write bytes, then clocks in read bytes if requested.
//   4) Read bytes are buffered locally and streamed back after CS deasserts.
//
// DONE CODES
//   0 : OK
//   1 : preload timeout
//   2 : transfer timeout
//   3 : length error
//   4 : protocol error (bad w_last discipline)
//==============================================================================
module spi_master_engine #(
    parameter integer CLK_HZ     = 100_000_000,
    parameter integer SPI_HZ     = 1_000_000,
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
    input  wire [1:0]  cmd_cs_sel,
    input  wire        cmd_3wire,
    input  wire [7:0]  cmd_wlen,
    input  wire [7:0]  cmd_rlen,
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
    // Physical SPI pins
    //--------------------------------------------------------------------------
    output reg         spi_sclk,
    output reg         spi_mosi,
    input  wire        spi_miso,
    inout  wire        lis2mdl_sdio,
    output reg         lis3dh_cs_n,
    output reg         bmp5xx_cs_n,
    output reg         lis2mdl_cs_n
);

    localparam integer HALF_DIV = (CLK_HZ / (SPI_HZ * 2));

    localparam [3:0]
        DONE_OK              = 4'd0,
        DONE_TIMEOUT_PRELOAD = 4'd1,
        DONE_TIMEOUT_XFER    = 4'd2,
        DONE_LENGTH_ERR      = 4'd3,
        DONE_PROTOCOL_ERR    = 4'd4;

    localparam [2:0]
        S_IDLE    = 3'd0,
        S_PRELOAD = 3'd1,
        S_EXEC    = 3'd2,
        S_STREAM  = 3'd3,
        S_DONE    = 3'd4,
        S_FAIL    = 3'd5;

    localparam [1:0]
        CS_LIS3DH  = 2'd0,
        CS_BMP5XX  = 2'd1,
        CS_LIS2MDL = 2'd2;

    reg [2:0] state;

    reg [1:0]  cs_sel_r;
    reg        threewire_r;
    reg [7:0]  wlen_r;
    reg [7:0]  rlen_r;
    reg [31:0] timeout_us_r;

    reg [7:0] wbuf [0:MAX_WBYTES-1];
    reg [7:0] rbuf [0:MAX_RBYTES-1];

    reg [7:0] preload_count;
    reg [7:0] tx_index;
    reg [7:0] rx_index;
    reg [7:0] stream_index;

    reg [7:0] tx_shift;
    reg [7:0] rx_shift;
    reg [2:0] bit_index;
    reg       bit_phase;

    reg [31:0] timeout_ctr;

    reg [15:0] div_ctr;
    reg        step_half;

    reg sdio_oe;
    reg sdio_out;
    wire sdio_in;
    reg [7:0] rx_byte_final;

    integer i;

    assign lis2mdl_sdio = sdio_oe ? sdio_out : 1'bz;
    assign sdio_in      = lis2mdl_sdio;

    always @(posedge clk) begin
        if (rst) begin
            div_ctr    <= 16'd0;
            step_half  <= 1'b0;
        end else begin
            step_half <= 1'b0;

            if (state == S_EXEC) begin
                if (div_ctr == (HALF_DIV - 1)) begin
                    div_ctr   <= 16'd0;
                    step_half <= 1'b1;
                end else begin
                    div_ctr <= div_ctr + 16'd1;
                end
            end else begin
                div_ctr <= 16'd0;
            end
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            state         <= S_IDLE;

            cmd_ready     <= 1'b1;
            w_ready       <= 1'b0;
            r_valid       <= 1'b0;
            r_data        <= 8'd0;
            r_last        <= 1'b0;
            done          <= 1'b0;
            done_code     <= DONE_OK;
            busy          <= 1'b0;

            spi_sclk      <= 1'b1;
            spi_mosi      <= 1'b0;
            lis3dh_cs_n   <= 1'b1;
            bmp5xx_cs_n   <= 1'b1;
            lis2mdl_cs_n  <= 1'b1;
            sdio_oe       <= 1'b0;
            sdio_out      <= 1'b0;

            cs_sel_r      <= CS_LIS3DH;
            threewire_r   <= 1'b0;
            wlen_r        <= 8'd0;
            rlen_r        <= 8'd0;
            timeout_us_r  <= 32'd0;
            preload_count <= 8'd0;
            tx_index      <= 8'd0;
            rx_index      <= 8'd0;
            stream_index  <= 8'd0;
            tx_shift      <= 8'd0;
            rx_shift      <= 8'd0;
            bit_index     <= 3'd7;
            bit_phase     <= 1'b0;
            timeout_ctr   <= 32'd0;
            rx_byte_final <= 8'd0;

            for (i = 0; i < MAX_WBYTES; i = i + 1)
                wbuf[i] <= 8'd0;
            for (i = 0; i < MAX_RBYTES; i = i + 1)
                rbuf[i] <= 8'd0;
        end else begin
            done <= 1'b0;

            if (r_valid && r_ready) begin
                if (stream_index == (rlen_r - 8'd1)) begin
                    r_valid      <= 1'b0;
                    r_last       <= 1'b0;
                    stream_index <= 8'd0;
                end else begin
                    stream_index <= stream_index + 8'd1;
                    r_data       <= rbuf[stream_index + 8'd1];
                    r_last       <= ((stream_index + 8'd1) == (rlen_r - 8'd1));
                end
            end

            if (tick_1us && busy) begin
                if (timeout_ctr >= timeout_us_r) begin
                    spi_sclk     <= 1'b1;
                    spi_mosi     <= 1'b0;
                    lis3dh_cs_n  <= 1'b1;
                    bmp5xx_cs_n  <= 1'b1;
                    lis2mdl_cs_n <= 1'b1;
                    sdio_oe      <= 1'b0;
                    sdio_out     <= 1'b0;
                    w_ready      <= 1'b0;
                    r_valid      <= 1'b0;
                    done_code    <= (state == S_PRELOAD) ? DONE_TIMEOUT_PRELOAD
                                                         : DONE_TIMEOUT_XFER;
                    state        <= S_FAIL;
                end else begin
                    timeout_ctr <= timeout_ctr + 32'd1;
                end
            end

            if (!(tick_1us && busy && (timeout_ctr >= timeout_us_r))) begin
                case (state)
                    S_IDLE: begin
                        cmd_ready    <= 1'b1;
                        w_ready      <= 1'b0;
                        busy         <= 1'b0;
                        spi_sclk     <= 1'b1;
                        spi_mosi     <= 1'b0;
                        lis3dh_cs_n  <= 1'b1;
                        bmp5xx_cs_n  <= 1'b1;
                        lis2mdl_cs_n <= 1'b1;
                        sdio_oe      <= 1'b0;
                        sdio_out     <= 1'b0;
                        timeout_ctr  <= 32'd0;
                        stream_index <= 8'd0;

                        if (cmd_valid && cmd_ready) begin
                            cmd_ready    <= 1'b0;
                            busy         <= 1'b1;
                            cs_sel_r     <= cmd_cs_sel;
                            threewire_r  <= cmd_3wire;
                            wlen_r       <= cmd_wlen;
                            rlen_r       <= cmd_rlen;
                            timeout_us_r <= cmd_timeout_us;
                            preload_count<= 8'd0;
                            tx_index     <= 8'd0;
                            rx_index     <= 8'd0;
                            bit_index    <= 3'd7;
                            bit_phase    <= 1'b0;
                            rx_shift     <= 8'd0;
                            done_code    <= DONE_OK;

                            if ((cmd_wlen == 8'd0) ||
                                (cmd_wlen > MAX_WBYTES[7:0]) ||
                                (cmd_rlen > MAX_RBYTES[7:0])) begin
                                done_code <= DONE_LENGTH_ERR;
                                state     <= S_FAIL;
                            end else begin
                                w_ready <= 1'b1;
                                state   <= S_PRELOAD;
                            end
                        end
                    end

                    S_PRELOAD: begin
                        if (w_valid && w_ready) begin
                            wbuf[preload_count] <= w_data;

                            if ((preload_count < (wlen_r - 8'd1)) && w_last) begin
                                w_ready   <= 1'b0;
                                done_code <= DONE_PROTOCOL_ERR;
                                state     <= S_FAIL;
                            end else if ((preload_count == (wlen_r - 8'd1)) && !w_last) begin
                                w_ready   <= 1'b0;
                                done_code <= DONE_PROTOCOL_ERR;
                                state     <= S_FAIL;
                            end else if (preload_count == (wlen_r - 8'd1)) begin
                                w_ready     <= 1'b0;
                                tx_index    <= 8'd0;
                                rx_index    <= 8'd0;
                                bit_index   <= 3'd7;
                                bit_phase   <= 1'b0;
                                tx_shift    <= (wlen_r == 8'd1) ? w_data : wbuf[0];
                                rx_shift    <= 8'd0;
                                spi_sclk    <= 1'b1;
                                spi_mosi    <= (wlen_r == 8'd1) ? w_data[7] : wbuf[0][7];
                                lis3dh_cs_n <= 1'b1;
                                bmp5xx_cs_n <= 1'b1;
                                lis2mdl_cs_n<= 1'b1;

                                case (cs_sel_r)
                                    CS_LIS3DH:  lis3dh_cs_n  <= 1'b0;
                                    CS_BMP5XX:  bmp5xx_cs_n  <= 1'b0;
                                    CS_LIS2MDL: lis2mdl_cs_n <= 1'b0;
                                    default: begin
                                    end
                                endcase

                                if (threewire_r) begin
                                    sdio_oe  <= 1'b1;
                                    sdio_out <= (wlen_r == 8'd1) ? w_data[7] : wbuf[0][7];
                                end else begin
                                    sdio_oe  <= 1'b0;
                                    sdio_out <= 1'b0;
                                end

                                state <= S_EXEC;
                            end else begin
                                preload_count <= preload_count + 8'd1;
                            end
                        end
                    end

                    S_EXEC: begin
                        if (step_half) begin
                            if (!bit_phase) begin
                                spi_sclk  <= 1'b0;
                                bit_phase <= 1'b1;
                            end else begin
                                spi_sclk  <= 1'b1;
                                bit_phase <= 1'b0;

                                if (tx_index >= wlen_r) begin
                                    if (threewire_r)
                                        rx_shift[bit_index] <= sdio_in;
                                    else
                                        rx_shift[bit_index] <= spi_miso;
                                end

                                if (bit_index != 3'd0) begin
                                    bit_index <= bit_index - 3'd1;

                                    if (tx_index < wlen_r) begin
                                        spi_mosi <= tx_shift[bit_index - 3'd1];
                                        if (threewire_r) begin
                                            sdio_oe  <= 1'b1;
                                            sdio_out <= tx_shift[bit_index - 3'd1];
                                        end
                                    end else begin
                                        spi_mosi <= 1'b0;
                                        if (threewire_r) begin
                                            sdio_oe  <= 1'b0;
                                            sdio_out <= 1'b0;
                                        end
                                    end
                                end else begin
                                    if (tx_index < wlen_r) begin
                                        if ((tx_index + 8'd1) < wlen_r) begin
                                            tx_index  <= tx_index + 8'd1;
                                            tx_shift  <= wbuf[tx_index + 8'd1];
                                            bit_index <= 3'd7;
                                            spi_mosi  <= wbuf[tx_index + 8'd1][7];
                                            if (threewire_r) begin
                                                sdio_oe  <= 1'b1;
                                                sdio_out <= wbuf[tx_index + 8'd1][7];
                                            end
                                        end else if (rlen_r != 8'd0) begin
                                            tx_index  <= tx_index + 8'd1;
                                            tx_shift  <= 8'd0;
                                            bit_index <= 3'd7;
                                            spi_mosi  <= 1'b0;
                                            if (threewire_r) begin
                                                sdio_oe  <= 1'b0;
                                                sdio_out <= 1'b0;
                                            end
                                        end else begin
                                            spi_mosi     <= 1'b0;
                                            lis3dh_cs_n  <= 1'b1;
                                            bmp5xx_cs_n  <= 1'b1;
                                            lis2mdl_cs_n <= 1'b1;
                                            sdio_oe      <= 1'b0;
                                            sdio_out     <= 1'b0;
                                            done_code    <= DONE_OK;
                                            state        <= S_DONE;
                                        end
                                    end else begin
                                        if (threewire_r)
                                            rx_byte_final <= {rx_shift[7:1], sdio_in};
                                        else
                                            rx_byte_final <= {rx_shift[7:1], spi_miso};

                                        rbuf[rx_index] <= threewire_r ? {rx_shift[7:1], sdio_in}
                                                                      : {rx_shift[7:1], spi_miso};

                                        if ((rx_index + 8'd1) < rlen_r) begin
                                            rx_index   <= rx_index + 8'd1;
                                            rx_shift   <= 8'd0;
                                            bit_index  <= 3'd7;
                                            spi_mosi   <= 1'b0;
                                            if (threewire_r) begin
                                                sdio_oe  <= 1'b0;
                                                sdio_out <= 1'b0;
                                            end
                                        end else begin
                                            spi_mosi     <= 1'b0;
                                            lis3dh_cs_n  <= 1'b1;
                                            bmp5xx_cs_n  <= 1'b1;
                                            lis2mdl_cs_n <= 1'b1;
                                            sdio_oe      <= 1'b0;
                                            sdio_out     <= 1'b0;
                                            stream_index <= 8'd0;
                                            r_valid      <= 1'b1;
                                            r_last       <= (rlen_r == 8'd1);
                                            if (rlen_r == 8'd1)
                                                r_data <= threewire_r ? {rx_shift[7:1], sdio_in}
                                                                      : {rx_shift[7:1], spi_miso};
                                            else
                                                r_data <= rbuf[0];
                                            done_code <= DONE_OK;
                                            state     <= S_STREAM;
                                        end
                                    end
                                end
                            end
                        end
                    end

                    S_STREAM: begin
                        if (!r_valid) begin
                            state <= S_DONE;
                        end
                    end

                    S_DONE: begin
                        done      <= 1'b1;
                        busy      <= 1'b0;
                        cmd_ready <= 1'b1;
                        w_ready   <= 1'b0;
                        state     <= S_IDLE;
                    end

                    S_FAIL: begin
                        done        <= 1'b1;
                        busy        <= 1'b0;
                        cmd_ready   <= 1'b1;
                        w_ready     <= 1'b0;
                        r_valid     <= 1'b0;
                        r_last      <= 1'b0;
                        spi_sclk    <= 1'b1;
                        spi_mosi    <= 1'b0;
                        lis3dh_cs_n <= 1'b1;
                        bmp5xx_cs_n <= 1'b1;
                        lis2mdl_cs_n<= 1'b1;
                        sdio_oe     <= 1'b0;
                        sdio_out    <= 1'b0;
                        state       <= S_IDLE;
                    end

                    default: begin
                        state <= S_IDLE;
                    end
                endcase
            end
        end
    end

endmodule

`default_nettype wire
