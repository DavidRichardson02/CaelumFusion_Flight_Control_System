`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// pmod_dpot_spi_job
//------------------------------------------------------------------------------
// Command-gated, write-only SPI mode-0 primitive for the Digilent Pmod DPOT.
//
// This module is intentionally an actuator primitive, not a telemetry source.
// A higher-level authority block must decide when dpot_update_valid may assert.
//==============================================================================
module pmod_dpot_spi_job #(
    parameter [31:0] CMD_TIMEOUT_US = 32'd2000
)(
    input  wire       clk,
    input  wire       rst,
    input  wire       dpot_update_valid,
    output reg        dpot_update_ready,
    input  wire [7:0] dpot_update_value,

    output reg        cmd_valid,
    input  wire       cmd_ready,
    output reg  [7:0] cmd_wlen,
    output reg  [7:0] cmd_rlen,
    output reg [31:0] cmd_timeout_us,

    output reg        w_valid,
    input  wire       w_ready,
    output reg  [7:0] w_data,
    output reg        w_last,

    input  wire       done,
    input  wire [3:0] done_code,
    output reg        busy,
    output reg  [7:0] last_value,
    output reg  [7:0] status
);
    localparam [1:0]
        S_IDLE  = 2'd0,
        S_CMD   = 2'd1,
        S_WRITE = 2'd2,
        S_DONE  = 2'd3;

    localparam [7:0]
        STAT_OK       = 8'h00,
        STAT_NOT_INIT = 8'h01,
        STAT_SPI_ERR  = 8'hE0;

    reg [1:0] st_r;
    reg [7:0] pending_value_r;

    always @(posedge clk) begin
        if (rst) begin
            st_r              <= S_IDLE;
            pending_value_r   <= 8'd0;
            dpot_update_ready <= 1'b1;
            cmd_valid         <= 1'b0;
            cmd_wlen          <= 8'd1;
            cmd_rlen          <= 8'd0;
            cmd_timeout_us    <= CMD_TIMEOUT_US;
            w_valid           <= 1'b0;
            w_data            <= 8'd0;
            w_last            <= 1'b1;
            busy              <= 1'b0;
            last_value        <= 8'd0;
            status            <= STAT_NOT_INIT;
        end else begin
            case (st_r)
                S_IDLE: begin
                    busy              <= 1'b0;
                    cmd_valid         <= 1'b0;
                    w_valid           <= 1'b0;
                    dpot_update_ready <= 1'b1;

                    if (dpot_update_valid) begin
                        pending_value_r   <= dpot_update_value;
                        dpot_update_ready <= 1'b0;
                        busy              <= 1'b1;
                        st_r              <= S_CMD;
                    end
                end

                S_CMD: begin
                    busy           <= 1'b1;
                    cmd_valid      <= 1'b1;
                    cmd_wlen       <= 8'd1;
                    cmd_rlen       <= 8'd0;
                    cmd_timeout_us <= CMD_TIMEOUT_US;

                    if (cmd_ready) begin
                        cmd_valid <= 1'b0;
                        w_valid   <= 1'b1;
                        w_data    <= pending_value_r;
                        w_last    <= 1'b1;
                        st_r      <= S_WRITE;
                    end
                end

                S_WRITE: begin
                    busy <= 1'b1;
                    if (w_valid && w_ready)
                        w_valid <= 1'b0;
                    if (done)
                        st_r <= S_DONE;
                end

                S_DONE: begin
                    busy <= 1'b0;
                    if (done_code == 4'd0) begin
                        last_value <= pending_value_r;
                        status     <= STAT_OK;
                    end else begin
                        status <= STAT_SPI_ERR | {4'd0, done_code};
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
