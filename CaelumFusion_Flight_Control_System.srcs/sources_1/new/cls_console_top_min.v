`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// cls_console_top_min
//------------------------------------------------------------------------------
// Minimal CLS console subsystem.
//
// Includes:
//   - page scheduler
//   - page formatter
//   - refresh FSM
//   - UART transmitter
//
// First milestone:
//   - boot ladder
//   - summary page
//   - counter page
//   - build/version page
//   - fault override
//   - manual page step
//==============================================================================
module cls_console_top_min #(
    parameter integer CLK_HZ   = 100_000_000,
    parameter integer BAUD_HZ  = 9600,
    parameter integer BUILD_ID = 16'd3
)(
    input  wire         clk,
    input  wire         rst,

    // page step input should already be debounced and one-pulse wide
    input  wire         page_step_pulse,

    // boot milestones
    input  wire         boot_clk_ok,
    input  wire         boot_timebase_ok,
    input  wire         boot_i2c_ok,
    input  wire         boot_acc_init_done,
    input  wire         boot_bmp_init_done,
    input  wire         boot_mag_init_done,
    input  wire         boot_viz_alive,
    input  wire         boot_complete,

    // derived snapshot summary
    input  wire [15:0]  alt16,
    input  wire signed [15:0] vspd16,
    input  wire signed [15:0] roll_q12,
    input  wire [15:0]  head_q12_u,
    input  wire [15:0]  der_seq,
    input  wire         der_valid,
    input  wire [7:0]   der_status,

    // counters
    input  wire [15:0]  tx_count,
    input  wire [7:0]   timeout_count,
    input  wire [7:0]   nack_addr_count,
    input  wire [7:0]   nack_data_count,
    input  wire [15:0]  cdc_count,
    input  wire [15:0]  frame_count,

    // fault latch input
    input  wire         fault_latched,
    input  wire [7:0]   fault_code,
    input  wire [15:0]  fault_arg,

    // UART TX to CLS RXD
    output wire         cls_tx
);

    wire [2:0] page_id;
    wire [127:0] line0;
    wire [127:0] line1;

    wire [7:0] tx_data;
    wire       tx_start;
    wire       tx_busy;
    wire       tx_done;

    cls_page_scheduler_min u_sched (
        .clk            (clk),
        .rst            (rst),
        .boot_complete  (boot_complete),
        .fault_latched  (fault_latched),
        .page_step_pulse(page_step_pulse),
        .page_id        (page_id)
    );

    cls_page_formatter_min u_fmt (
        .page_id           (page_id),

        .boot_clk_ok       (boot_clk_ok),
        .boot_timebase_ok  (boot_timebase_ok),
        .boot_i2c_ok       (boot_i2c_ok),
        .boot_acc_init_done(boot_acc_init_done),
        .boot_bmp_init_done(boot_bmp_init_done),
        .boot_mag_init_done(boot_mag_init_done),
        .boot_viz_alive    (boot_viz_alive),
        .boot_complete     (boot_complete),

        .alt16             (alt16),
        .vspd16            (vspd16),
        .roll_q12          (roll_q12),
        .head_q12_u        (head_q12_u),
        .der_seq           (der_seq),
        .der_valid         (der_valid),
        .der_status        (der_status),

        .tx_count          (tx_count),
        .timeout_count     (timeout_count),
        .nack_addr_count   (nack_addr_count),
        .nack_data_count   (nack_data_count),
        .cdc_count         (cdc_count),
        .frame_count       (frame_count),

        .build_id          (BUILD_ID),

        .fault_code        (fault_code),
        .fault_arg         (fault_arg),

        .line0             (line0),
        .line1             (line1)
    );

    cls_refresh_fsm #(
        .REFRESH_HZ(2),
        .CLK_HZ    (CLK_HZ)
    ) u_refresh (
        .clk          (clk),
        .rst          (rst),
        .line0        (line0),
        .line1        (line1),
        .force_refresh(page_step_pulse),
        .tx_data      (tx_data),
        .tx_start     (tx_start),
        .tx_busy      (tx_busy),
        .tx_done      (tx_done)
    );

    cls_uart_tx_9600 #(
        .CLK_HZ (CLK_HZ),
        .BAUD_HZ(BAUD_HZ)
    ) u_uart (
        .clk     (clk),
        .rst     (rst),
        .data_in (tx_data),
        .start   (tx_start),
        .tx      (cls_tx),
        .busy    (tx_busy),
        .done    (tx_done)
    );

endmodule

`default_nettype wire