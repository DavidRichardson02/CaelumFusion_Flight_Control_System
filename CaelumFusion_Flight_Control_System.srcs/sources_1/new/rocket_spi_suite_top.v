`timescale 1ns/1ps
`default_nettype none

`include "telemetry_defs_vh.vh"

//==============================================================================
// rocket_spi_suite_top
//------------------------------------------------------------------------------
// Canonical SYS-domain sensor acquisition suite for the parallel SPI path.
//
// EXTERNAL CONTRACT
//   - Matches the published snapshot / derived-state surface of
//     rocket_i2c_suite_top so the visualization stack can be reused.
//   - The legacy observability names der_i2c_* are preserved for compatibility;
//     in this SPI suite they carry protocol-error and timeout counts.
//
// BUS MAP
//   - Shared SCLK / MOSI
//   - Shared MISO for LIS3DH and BMP5xx
//   - Dedicated LIS2MDL SDIO for its default 3-wire SPI mode
//   - One CS per sensor
//==============================================================================
module rocket_spi_suite_top #(
    parameter integer CLK_HZ                  = 100_000_000,
    parameter integer SNAP_PAYLOAD_W          = 48,
    parameter integer RATE_100HZ_US           = 10_000,
    parameter integer RATE_50HZ_US            = 20_000,
    parameter integer RATE_10HZ_US            = 100_000,
    parameter integer BMP_FRESH_MAX_MS        = 150,
    parameter integer ACC_FRESH_MAX_MS        = 80,
    parameter integer MAG_FRESH_MAX_MS        = 250,
    parameter [31:0] BUILD_ID                 = 32'h0000_0002,
    parameter [15:0] SCHEMA_WORD              = 16'hCF10
)(
    input  wire                      clk,
    input  wire                      rst,

    output wire                      spi_sclk,
    output wire                      spi_mosi,
    input  wire                      spi_miso,
    inout  wire                      lis2mdl_sdio,
    output wire                      lis3dh_cs_n,
    output wire                      bmp5xx_cs_n,
    output wire                      lis2mdl_cs_n,

    output wire [31:0]               bmp_t_us,
    output wire [15:0]               bmp_seq,
    output wire                      bmp_valid,
    output wire [7:0]                bmp_status,
    output wire [SNAP_PAYLOAD_W-1:0] bmp_payload,

    output wire [31:0]               acc_t_us,
    output wire [15:0]               acc_seq,
    output wire                      acc_valid,
    output wire [7:0]                acc_status,
    output wire [SNAP_PAYLOAD_W-1:0] acc_payload,

    output wire [31:0]               mag_t_us,
    output wire [15:0]               mag_seq,
    output wire                      mag_valid,
    output wire [7:0]                mag_status,
    output wire [SNAP_PAYLOAD_W-1:0] mag_payload,

    output wire [31:0]               der_t_us,
    output wire [15:0]               der_seq,
    output wire [7:0]                der_source_id,
    output wire [7:0]                der_status,
    output wire                      der_valid,

    output wire                      der_alt_fresh,
    output wire                      der_vspd_fresh,
    output wire                      der_roll_fresh,
    output wire                      der_head_fresh,

    output wire [15:0]               der_bmp_seq_ref,
    output wire [15:0]               der_acc_seq_ref,
    output wire [15:0]               der_mag_seq_ref,

    output wire [15:0]               der_bmp_age_ms,
    output wire [15:0]               der_acc_age_ms,
    output wire [15:0]               der_mag_age_ms,

    output wire                      der_bmp_valid_ref,
    output wire                      der_acc_valid_ref,
    output wire                      der_mag_valid_ref,

    output wire [31:0]               der_altitude_cm,
    output wire [31:0]               der_vertical_speed_cms,
    output wire [31:0]               der_roll_mdeg,
    output wire [31:0]               der_heading_mdeg,

    output wire [15:0]               der_i2c_nack_count,
    output wire [15:0]               der_i2c_timeout_count,
    output wire [15:0]               der_txn_rate_hz,
    output wire [31:0]               der_cdc_update_count,
    output wire [31:0]               der_frame_count,
    output wire [31:0]               der_build_id,
    output wire [15:0]               der_schema_word
);

    //--------------------------------------------------------------------------
    // Timebase and scheduler
    //--------------------------------------------------------------------------
    wire [31:0] time_us;
    wire        tick_1us;
    wire        epoch_100hz;
    wire        epoch_50hz;
    wire        epoch_10hz;

    timebase_us #(
        .CLK_HZ(CLK_HZ)
    ) u_timebase_us (
        .clk      (clk),
        .rst      (rst),
        .time_us  (time_us),
        .tick_1us (tick_1us)
    );

    epoch_scheduler #(
        .RATE_100HZ_US(RATE_100HZ_US),
        .RATE_50HZ_US (RATE_50HZ_US),
        .RATE_10HZ_US (RATE_10HZ_US)
    ) u_epoch_scheduler (
        .clk         (clk),
        .rst         (rst),
        .tick_1us    (tick_1us),
        .epoch_100hz (epoch_100hz),
        .epoch_50hz  (epoch_50hz),
        .epoch_10hz  (epoch_10hz)
    );

    //--------------------------------------------------------------------------
    // Cadence intent and shared engine contract
    //--------------------------------------------------------------------------
    wire want_lis3dh  = epoch_100hz;
    wire want_bmp5xx  = epoch_50hz;
    wire want_lis2mdl = epoch_10hz;

    wire        e_cmd_valid;
    wire        e_cmd_ready;
    wire [1:0]  e_cmd_cs_sel;
    wire        e_cmd_3wire;
    wire [7:0]  e_cmd_wlen;
    wire [7:0]  e_cmd_rlen;
    wire [31:0] e_cmd_timeout_us;

    wire        e_w_valid;
    wire        e_w_ready;
    wire [7:0]  e_w_data;
    wire        e_w_last;

    wire        e_r_valid;
    wire        e_r_ready;
    wire [7:0]  e_r_data;
    wire        e_r_last;

    wire        e_done;
    wire [3:0]  e_done_code;
    wire        e_busy;

    //--------------------------------------------------------------------------
    // Fixed-priority arbiter
    //--------------------------------------------------------------------------
    wire grant_lis3dh;
    wire grant_bmp5xx;
    wire grant_lis2mdl;

    spi_job_arbiter u_spi_job_arbiter (
        .clk           (clk),
        .rst           (rst),
        .want_lis3dh   (want_lis3dh),
        .want_bmp5xx   (want_bmp5xx),
        .want_lis2mdl  (want_lis2mdl),
        .engine_busy   (e_busy),
        .grant_lis3dh  (grant_lis3dh),
        .grant_bmp5xx  (grant_bmp5xx),
        .grant_lis2mdl (grant_lis2mdl)
    );

    //--------------------------------------------------------------------------
    // Job 0: LIS3DH
    //--------------------------------------------------------------------------
    wire        j0_cmd_valid;
    wire        j0_cmd_ready;
    wire [7:0]  j0_cmd_wlen;
    wire [7:0]  j0_cmd_rlen;
    wire [31:0] j0_cmd_timeout_us;
    wire        j0_w_valid;
    wire        j0_w_ready;
    wire [7:0]  j0_w_data;
    wire        j0_w_last;
    wire        j0_r_valid;
    wire        j0_r_ready;
    wire [7:0]  j0_r_data;
    wire        j0_r_last;
    wire        j0_done;
    wire [3:0]  j0_done_code;
    wire        j0_snap_commit;
    wire        j0_snap_valid_in;
    wire [7:0]  j0_snap_status_in;
    wire [SNAP_PAYLOAD_W-1:0] j0_snap_payload_in;
    wire        j0_init_done;

    lis3dh_spi_job u_lis3dh_spi_job (
        .clk             (clk),
        .rst             (rst),
        .epoch_100hz     (epoch_100hz),
        .grant           (grant_lis3dh),
        .cmd_valid       (j0_cmd_valid),
        .cmd_ready       (j0_cmd_ready),
        .cmd_wlen        (j0_cmd_wlen),
        .cmd_rlen        (j0_cmd_rlen),
        .cmd_timeout_us  (j0_cmd_timeout_us),
        .w_valid         (j0_w_valid),
        .w_ready         (j0_w_ready),
        .w_data          (j0_w_data),
        .w_last          (j0_w_last),
        .r_valid         (j0_r_valid),
        .r_ready         (j0_r_ready),
        .r_data          (j0_r_data),
        .r_last          (j0_r_last),
        .done            (j0_done),
        .done_code       (j0_done_code),
        .snap_commit     (j0_snap_commit),
        .snap_valid_in   (j0_snap_valid_in),
        .snap_status_in  (j0_snap_status_in),
        .snap_payload_in (j0_snap_payload_in),
        .init_done       (j0_init_done)
    );

    snapshot_regs #(
        .PAYLOAD_W(SNAP_PAYLOAD_W)
    ) u_acc_snapshot_regs (
        .clk          (clk),
        .rst          (rst),
        .time_us      (time_us),
        .commit       (j0_snap_commit),
        .valid_in     (j0_snap_valid_in),
        .status_in    (j0_snap_status_in),
        .payload_in   (j0_snap_payload_in),
        .snap_t_us    (acc_t_us),
        .snap_seq     (acc_seq),
        .snap_valid   (acc_valid),
        .snap_status  (acc_status),
        .snap_payload (acc_payload)
    );

    //--------------------------------------------------------------------------
    // Job 1: BMP5xx
    //--------------------------------------------------------------------------
    wire        j1_cmd_valid;
    wire        j1_cmd_ready;
    wire [7:0]  j1_cmd_wlen;
    wire [7:0]  j1_cmd_rlen;
    wire [31:0] j1_cmd_timeout_us;
    wire        j1_w_valid;
    wire        j1_w_ready;
    wire [7:0]  j1_w_data;
    wire        j1_w_last;
    wire        j1_r_valid;
    wire        j1_r_ready;
    wire [7:0]  j1_r_data;
    wire        j1_r_last;
    wire        j1_done;
    wire [3:0]  j1_done_code;
    wire        j1_snap_commit;
    wire        j1_snap_valid_in;
    wire [7:0]  j1_snap_status_in;
    wire [SNAP_PAYLOAD_W-1:0] j1_snap_payload_in;
    wire        j1_init_done;

    bmp5xx_spi_job u_bmp5xx_spi_job (
        .clk             (clk),
        .rst             (rst),
        .time_us         (time_us),
        .epoch_50hz      (epoch_50hz),
        .grant           (grant_bmp5xx),
        .cmd_valid       (j1_cmd_valid),
        .cmd_ready       (j1_cmd_ready),
        .cmd_wlen        (j1_cmd_wlen),
        .cmd_rlen        (j1_cmd_rlen),
        .cmd_timeout_us  (j1_cmd_timeout_us),
        .w_valid         (j1_w_valid),
        .w_ready         (j1_w_ready),
        .w_data          (j1_w_data),
        .w_last          (j1_w_last),
        .r_valid         (j1_r_valid),
        .r_ready         (j1_r_ready),
        .r_data          (j1_r_data),
        .r_last          (j1_r_last),
        .done            (j1_done),
        .done_code       (j1_done_code),
        .snap_commit     (j1_snap_commit),
        .snap_valid_in   (j1_snap_valid_in),
        .snap_status_in  (j1_snap_status_in),
        .snap_payload_in (j1_snap_payload_in),
        .init_done       (j1_init_done)
    );

    snapshot_regs #(
        .PAYLOAD_W(SNAP_PAYLOAD_W)
    ) u_bmp_snapshot_regs (
        .clk          (clk),
        .rst          (rst),
        .time_us      (time_us),
        .commit       (j1_snap_commit),
        .valid_in     (j1_snap_valid_in),
        .status_in    (j1_snap_status_in),
        .payload_in   (j1_snap_payload_in),
        .snap_t_us    (bmp_t_us),
        .snap_seq     (bmp_seq),
        .snap_valid   (bmp_valid),
        .snap_status  (bmp_status),
        .snap_payload (bmp_payload)
    );

    //--------------------------------------------------------------------------
    // Job 2: LIS2MDL
    //--------------------------------------------------------------------------
    wire        j2_cmd_valid;
    wire        j2_cmd_ready;
    wire [7:0]  j2_cmd_wlen;
    wire [7:0]  j2_cmd_rlen;
    wire [31:0] j2_cmd_timeout_us;
    wire        j2_w_valid;
    wire        j2_w_ready;
    wire [7:0]  j2_w_data;
    wire        j2_w_last;
    wire        j2_r_valid;
    wire        j2_r_ready;
    wire [7:0]  j2_r_data;
    wire        j2_r_last;
    wire        j2_done;
    wire [3:0]  j2_done_code;
    wire        j2_snap_commit;
    wire        j2_snap_valid_in;
    wire [7:0]  j2_snap_status_in;
    wire [SNAP_PAYLOAD_W-1:0] j2_snap_payload_in;
    wire        j2_init_done;

    lis2mdl_spi_job u_lis2mdl_spi_job (
        .clk             (clk),
        .rst             (rst),
        .epoch_10hz      (epoch_10hz),
        .grant           (grant_lis2mdl),
        .cmd_valid       (j2_cmd_valid),
        .cmd_ready       (j2_cmd_ready),
        .cmd_wlen        (j2_cmd_wlen),
        .cmd_rlen        (j2_cmd_rlen),
        .cmd_timeout_us  (j2_cmd_timeout_us),
        .w_valid         (j2_w_valid),
        .w_ready         (j2_w_ready),
        .w_data          (j2_w_data),
        .w_last          (j2_w_last),
        .r_valid         (j2_r_valid),
        .r_ready         (j2_r_ready),
        .r_data          (j2_r_data),
        .r_last          (j2_r_last),
        .done            (j2_done),
        .done_code       (j2_done_code),
        .snap_commit     (j2_snap_commit),
        .snap_valid_in   (j2_snap_valid_in),
        .snap_status_in  (j2_snap_status_in),
        .snap_payload_in (j2_snap_payload_in),
        .init_done       (j2_init_done)
    );

    snapshot_regs #(
        .PAYLOAD_W(SNAP_PAYLOAD_W)
    ) u_mag_snapshot_regs (
        .clk          (clk),
        .rst          (rst),
        .time_us      (time_us),
        .commit       (j2_snap_commit),
        .valid_in     (j2_snap_valid_in),
        .status_in    (j2_snap_status_in),
        .payload_in   (j2_snap_payload_in),
        .snap_t_us    (mag_t_us),
        .snap_seq     (mag_seq),
        .snap_valid   (mag_valid),
        .snap_status  (mag_status),
        .snap_payload (mag_payload)
    );

    //--------------------------------------------------------------------------
    // Owner-latching mux and shared engine
    //--------------------------------------------------------------------------
    spi_job_mux u_spi_job_mux (
        .clk              (clk),
        .rst              (rst),
        .grant_lis3dh     (grant_lis3dh),
        .grant_bmp5xx     (grant_bmp5xx),
        .grant_lis2mdl    (grant_lis2mdl),

        .j0_cmd_valid     (j0_cmd_valid),
        .j0_cmd_ready     (j0_cmd_ready),
        .j0_cmd_wlen      (j0_cmd_wlen),
        .j0_cmd_rlen      (j0_cmd_rlen),
        .j0_cmd_timeout_us(j0_cmd_timeout_us),
        .j0_w_valid       (j0_w_valid),
        .j0_w_ready       (j0_w_ready),
        .j0_w_data        (j0_w_data),
        .j0_w_last        (j0_w_last),
        .j0_r_valid       (j0_r_valid),
        .j0_r_ready       (j0_r_ready),
        .j0_r_data        (j0_r_data),
        .j0_r_last        (j0_r_last),
        .j0_done          (j0_done),
        .j0_done_code     (j0_done_code),

        .j1_cmd_valid     (j1_cmd_valid),
        .j1_cmd_ready     (j1_cmd_ready),
        .j1_cmd_wlen      (j1_cmd_wlen),
        .j1_cmd_rlen      (j1_cmd_rlen),
        .j1_cmd_timeout_us(j1_cmd_timeout_us),
        .j1_w_valid       (j1_w_valid),
        .j1_w_ready       (j1_w_ready),
        .j1_w_data        (j1_w_data),
        .j1_w_last        (j1_w_last),
        .j1_r_valid       (j1_r_valid),
        .j1_r_ready       (j1_r_ready),
        .j1_r_data        (j1_r_data),
        .j1_r_last        (j1_r_last),
        .j1_done          (j1_done),
        .j1_done_code     (j1_done_code),

        .j2_cmd_valid     (j2_cmd_valid),
        .j2_cmd_ready     (j2_cmd_ready),
        .j2_cmd_wlen      (j2_cmd_wlen),
        .j2_cmd_rlen      (j2_cmd_rlen),
        .j2_cmd_timeout_us(j2_cmd_timeout_us),
        .j2_w_valid       (j2_w_valid),
        .j2_w_ready       (j2_w_ready),
        .j2_w_data        (j2_w_data),
        .j2_w_last        (j2_w_last),
        .j2_r_valid       (j2_r_valid),
        .j2_r_ready       (j2_r_ready),
        .j2_r_data        (j2_r_data),
        .j2_r_last        (j2_r_last),
        .j2_done          (j2_done),
        .j2_done_code     (j2_done_code),

        .e_cmd_valid      (e_cmd_valid),
        .e_cmd_ready      (e_cmd_ready),
        .e_cmd_cs_sel     (e_cmd_cs_sel),
        .e_cmd_3wire      (e_cmd_3wire),
        .e_cmd_wlen       (e_cmd_wlen),
        .e_cmd_rlen       (e_cmd_rlen),
        .e_cmd_timeout_us (e_cmd_timeout_us),
        .e_w_valid        (e_w_valid),
        .e_w_ready        (e_w_ready),
        .e_w_data         (e_w_data),
        .e_w_last         (e_w_last),
        .e_r_valid        (e_r_valid),
        .e_r_ready        (e_r_ready),
        .e_r_data         (e_r_data),
        .e_r_last         (e_r_last),
        .e_done           (e_done),
        .e_done_code      (e_done_code)
    );

    spi_master_engine #(
        .CLK_HZ    (CLK_HZ),
        .SPI_HZ    (1_000_000),
        .MAX_WBYTES(8),
        .MAX_RBYTES(16)
    ) u_spi_master_engine (
        .clk            (clk),
        .rst            (rst),
        .cmd_valid      (e_cmd_valid),
        .cmd_ready      (e_cmd_ready),
        .cmd_cs_sel     (e_cmd_cs_sel),
        .cmd_3wire      (e_cmd_3wire),
        .cmd_wlen       (e_cmd_wlen),
        .cmd_rlen       (e_cmd_rlen),
        .cmd_timeout_us (e_cmd_timeout_us),
        .w_valid        (e_w_valid),
        .w_ready        (e_w_ready),
        .w_data         (e_w_data),
        .w_last         (e_w_last),
        .r_valid        (e_r_valid),
        .r_ready        (e_r_ready),
        .r_data         (e_r_data),
        .r_last         (e_r_last),
        .done           (e_done),
        .done_code      (e_done_code),
        .busy           (e_busy),
        .tick_1us       (tick_1us),
        .spi_sclk       (spi_sclk),
        .spi_mosi       (spi_mosi),
        .spi_miso       (spi_miso),
        .lis2mdl_sdio   (lis2mdl_sdio),
        .lis3dh_cs_n    (lis3dh_cs_n),
        .bmp5xx_cs_n    (bmp5xx_cs_n),
        .lis2mdl_cs_n   (lis2mdl_cs_n)
    );

    //--------------------------------------------------------------------------
    // Age computation
    //--------------------------------------------------------------------------
    function [15:0] age_ms_from_snap;
        input [31:0] now_us;
        input [31:0] snap_us;
        reg   [31:0] delta_us;
        reg   [31:0] delta_ms;
        begin
            if (now_us >= snap_us)
                delta_us = now_us - snap_us;
            else
                delta_us = 32'hFFFF_FFFF;

            delta_ms = delta_us / 32'd1000;

            if (delta_ms > 32'd65535)
                age_ms_from_snap = 16'hFFFF;
            else
                age_ms_from_snap = delta_ms[15:0];
        end
    endfunction

    wire [15:0] bmp_age_ms = age_ms_from_snap(time_us, bmp_t_us);
    wire [15:0] acc_age_ms = age_ms_from_snap(time_us, acc_t_us);
    wire [15:0] mag_age_ms = age_ms_from_snap(time_us, mag_t_us);

    //--------------------------------------------------------------------------
    // Global observability ownership
    //--------------------------------------------------------------------------
    reg [15:0] txn_count_window_r;
    reg [15:0] txn_rate_hz_r;
    reg [15:0] bus_proto_err_count_r;
    reg [15:0] bus_timeout_count_r;
    reg [31:0] cdc_update_count_r;
    reg [31:0] frame_count_r;
    reg [19:0] us_to_1s_ctr;

    always @(posedge clk) begin
        if (rst) begin
            txn_count_window_r  <= 16'd0;
            txn_rate_hz_r       <= 16'd0;
            bus_proto_err_count_r <= 16'd0;
            bus_timeout_count_r <= 16'd0;
            cdc_update_count_r  <= 32'd0;
            frame_count_r       <= 32'd0;
            us_to_1s_ctr        <= 20'd0;
        end else begin
            if (e_done)
                txn_count_window_r <= txn_count_window_r + 16'd1;

            if (e_done && ((e_done_code == 4'h3) || (e_done_code == 4'h4)))
                bus_proto_err_count_r <= bus_proto_err_count_r + 16'd1;

            if (e_done && ((e_done_code == 4'h1) || (e_done_code == 4'h2)))
                bus_timeout_count_r <= bus_timeout_count_r + 16'd1;

            if (j0_snap_commit || j1_snap_commit || j2_snap_commit)
                cdc_update_count_r <= cdc_update_count_r + 32'd1;

            if (tick_1us) begin
                if (us_to_1s_ctr == 20'd999_999) begin
                    us_to_1s_ctr      <= 20'd0;
                    txn_rate_hz_r     <= txn_count_window_r;
                    txn_count_window_r<= 16'd0;
                    frame_count_r     <= frame_count_r + 32'd1;
                end else begin
                    us_to_1s_ctr <= us_to_1s_ctr + 20'd1;
                end
            end
        end
    end

    assign der_i2c_nack_count    = bus_proto_err_count_r;
    assign der_i2c_timeout_count = bus_timeout_count_r;
    assign der_txn_rate_hz       = txn_rate_hz_r;
    assign der_cdc_update_count  = cdc_update_count_r;
    assign der_frame_count       = frame_count_r;
    assign der_build_id          = BUILD_ID;
    assign der_schema_word       = SCHEMA_WORD;

    //--------------------------------------------------------------------------
    // Derived-state producer
    //--------------------------------------------------------------------------
    derived_state_producer #(
        .BUILD_ID_CONST    (BUILD_ID),
        .SCHEMA_WORD_CONST (SCHEMA_WORD),
        .MAG_PAYLOAD_ZYX   (1)
    ) u_derived_state_producer (
        .clk                   (clk),
        .rst                   (rst),
        .now_us                (time_us),

        .bmp_t_us              (bmp_t_us),
        .bmp_seq               (bmp_seq),
        .bmp_valid             (bmp_valid),
        .bmp_status            (bmp_status),
        .bmp_payload           (bmp_payload),
        .bmp_age_ms            (bmp_age_ms),

        .acc_t_us              (acc_t_us),
        .acc_seq               (acc_seq),
        .acc_valid             (acc_valid),
        .acc_status            (acc_status),
        .acc_payload           (acc_payload),
        .acc_age_ms            (acc_age_ms),

        .mag_t_us              (mag_t_us),
        .mag_seq               (mag_seq),
        .mag_valid             (mag_valid),
        .mag_status            (mag_status),
        .mag_payload           (mag_payload),
        .mag_age_ms            (mag_age_ms),

        .i2c_nack_count        (bus_proto_err_count_r),
        .i2c_timeout_count     (bus_timeout_count_r),
        .txn_rate_hz           (txn_rate_hz_r),
        .cdc_update_count      (cdc_update_count_r),
        .frame_count           (frame_count_r),

        .der_t_us              (der_t_us),
        .der_seq               (der_seq),
        .der_source_id         (der_source_id),
        .der_status            (der_status),
        .der_valid             (der_valid),

        .der_alt_fresh         (der_alt_fresh),
        .der_vspd_fresh        (der_vspd_fresh),
        .der_roll_fresh        (der_roll_fresh),
        .der_head_fresh        (der_head_fresh),

        .der_bmp_seq_ref       (der_bmp_seq_ref),
        .der_acc_seq_ref       (der_acc_seq_ref),
        .der_mag_seq_ref       (der_mag_seq_ref),

        .der_bmp_age_ms        (der_bmp_age_ms),
        .der_acc_age_ms        (der_acc_age_ms),
        .der_mag_age_ms        (der_mag_age_ms),

        .der_bmp_valid_ref     (der_bmp_valid_ref),
        .der_acc_valid_ref     (der_acc_valid_ref),
        .der_mag_valid_ref     (der_mag_valid_ref),

        .der_altitude_cm       (der_altitude_cm),
        .der_vertical_speed_cms(der_vertical_speed_cms),
        .der_roll_mdeg         (der_roll_mdeg),
        .der_heading_mdeg      (der_heading_mdeg),

        .der_i2c_nack_count    (),
        .der_i2c_timeout_count (),
        .der_txn_rate_hz       (),
        .der_cdc_update_count  (),
        .der_frame_count       (),
        .der_build_id          (),
        .der_schema_word       ()
    );

endmodule

`default_nettype wire
