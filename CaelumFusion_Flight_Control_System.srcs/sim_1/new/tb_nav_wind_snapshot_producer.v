`timescale 1ns / 1ps
`default_nettype none

`include "telemetry_defs_vh.vh"

module tb_nav_wind_snapshot_producer;

    reg clk;
    reg rst;
    reg enable;
    reg sample_event;
    reg [31:0] now_us;

    reg        ekf_valid;
    reg [7:0]  ekf_status;
    reg [15:0] ekf_seq;
    reg [15:0] ekf_age_ms;
    reg [15:0] ekf_downrange_m;
    reg [15:0] ekf_crossrange_m;
    reg [15:0] ekf_source_flags;

    reg        gnss_valid;
    reg [7:0]  gnss_status;
    reg [15:0] gnss_seq;
    reg [15:0] gnss_age_ms;
    reg [7:0]  gnss_fix_type;
    reg [15:0] gnss_source_flags;

    reg        wind_est_valid;
    reg [7:0]  wind_est_status;
    reg [15:0] wind_est_seq;
    reg [15:0] wind_est_age_ms;
    reg [15:0] wind_est_x_cms;
    reg [15:0] wind_est_y_cms;
    reg [15:0] wind_est_z_cms;
    reg [15:0] wind_est_source_flags;

    wire [31:0] nav_t_us;
    wire [15:0] nav_seq;
    wire        nav_valid;
    wire [7:0]  nav_status;
    wire [7:0]  nav_flags;
    wire [15:0] nav_downrange_m;
    wire [15:0] nav_crossrange_m;
    wire [15:0] nav_age_ms;

    wire [31:0] wind_t_us;
    wire [15:0] wind_seq;
    wire        wind_valid;
    wire [7:0]  wind_status;
    wire [7:0]  wind_flags;
    wire [15:0] wind_x_cms;
    wire [15:0] wind_y_cms;
    wire [15:0] wind_z_cms;
    wire [15:0] wind_age_ms;

    nav_wind_snapshot_producer #(
        .NAV_FRESH_MAX_MS      (16'd100),
        .WIND_FRESH_MAX_MS     (16'd100),
        .REQUIRE_GNSS_FOR_NAV  (1)
    ) dut (
        .clk                  (clk),
        .rst                  (rst),
        .enable               (enable),
        .sample_event         (sample_event),
        .now_us               (now_us),
        .ekf_valid            (ekf_valid),
        .ekf_status           (ekf_status),
        .ekf_seq              (ekf_seq),
        .ekf_age_ms           (ekf_age_ms),
        .ekf_downrange_m      (ekf_downrange_m),
        .ekf_crossrange_m     (ekf_crossrange_m),
        .ekf_source_flags     (ekf_source_flags),
        .gnss_valid           (gnss_valid),
        .gnss_status          (gnss_status),
        .gnss_seq             (gnss_seq),
        .gnss_age_ms          (gnss_age_ms),
        .gnss_fix_type        (gnss_fix_type),
        .gnss_source_flags    (gnss_source_flags),
        .wind_est_valid       (wind_est_valid),
        .wind_est_status      (wind_est_status),
        .wind_est_seq         (wind_est_seq),
        .wind_est_age_ms      (wind_est_age_ms),
        .wind_est_x_cms       (wind_est_x_cms),
        .wind_est_y_cms       (wind_est_y_cms),
        .wind_est_z_cms       (wind_est_z_cms),
        .wind_est_source_flags(wind_est_source_flags),
        .nav_t_us             (nav_t_us),
        .nav_seq              (nav_seq),
        .nav_valid            (nav_valid),
        .nav_status           (nav_status),
        .nav_flags            (nav_flags),
        .nav_downrange_m      (nav_downrange_m),
        .nav_crossrange_m     (nav_crossrange_m),
        .nav_age_ms           (nav_age_ms),
        .wind_t_us            (wind_t_us),
        .wind_seq             (wind_seq),
        .wind_valid           (wind_valid),
        .wind_status          (wind_status),
        .wind_flags           (wind_flags),
        .wind_x_cms           (wind_x_cms),
        .wind_y_cms           (wind_y_cms),
        .wind_z_cms           (wind_z_cms),
        .wind_age_ms          (wind_age_ms)
    );

    always #5 clk = ~clk;

    task expect;
        input condition;
        input [180*8-1:0] message;
        begin
            if (!condition) begin
                $display("FAIL: %0s", message);
                $finish;
            end
        end
    endtask

    task pulse_sample;
        begin
            @(negedge clk);
            sample_event = 1'b1;
            @(negedge clk);
            sample_event = 1'b0;
            @(posedge clk);
            #1;
        end
    endtask

    task clear_sources;
        begin
            ekf_valid             = 1'b0;
            ekf_status            = `ST_NOT_INITIALIZED;
            ekf_seq               = 16'd0;
            ekf_age_ms            = 16'hFFFF;
            ekf_downrange_m       = 16'd0;
            ekf_crossrange_m      = 16'd0;
            ekf_source_flags      = 16'd0;
            gnss_valid            = 1'b0;
            gnss_status           = `ST_NOT_INITIALIZED;
            gnss_seq              = 16'd0;
            gnss_age_ms           = 16'hFFFF;
            gnss_fix_type         = 8'd0;
            gnss_source_flags     = 16'd0;
            wind_est_valid        = 1'b0;
            wind_est_status       = `ST_NOT_INITIALIZED;
            wind_est_seq          = 16'd0;
            wind_est_age_ms       = 16'hFFFF;
            wind_est_x_cms        = 16'd0;
            wind_est_y_cms        = 16'd0;
            wind_est_z_cms        = 16'd0;
            wind_est_source_flags = 16'd0;
        end
    endtask

    initial begin
        clk = 1'b0;
        rst = 1'b1;
        enable = 1'b0;
        sample_event = 1'b0;
        now_us = 32'd0;
        clear_sources();

        repeat (3) @(posedge clk);
        rst = 1'b0;
        @(posedge clk);
        #1;

        expect(nav_valid == 1'b0, "nav invalid after reset");
        expect(nav_status == `ST_NOT_INITIALIZED, "nav reset status is not initialized");
        expect(wind_valid == 1'b0, "wind invalid after reset");
        expect(wind_status == `ST_NOT_INITIALIZED, "wind reset status is not initialized");

        now_us = 32'd1000;
        pulse_sample();
        expect(nav_seq == 16'd0, "disabled producer does not advance nav sequence");
        expect(wind_seq == 16'd0, "disabled producer does not advance wind sequence");

        enable = 1'b1;
        now_us = 32'd2000;
        pulse_sample();
        expect(nav_seq == 16'd1, "enabled missing-source nav snapshot advances sequence");
        expect(nav_valid == 1'b0, "nav invalid when EKF/GNSS are missing");
        expect(nav_status == `ST_MISSING_INPUT, "nav reports missing required source");
        expect(wind_seq == 16'd1, "enabled missing-source wind snapshot advances sequence");
        expect(wind_valid == 1'b0, "wind invalid when wind estimator is missing");
        expect(wind_status == `ST_MISSING_INPUT, "wind reports missing source");

        ekf_valid        = 1'b1;
        ekf_status       = `ST_OK;
        ekf_seq          = 16'h0101;
        ekf_age_ms       = 16'd20;
        ekf_downrange_m  = 16'd42;
        ekf_crossrange_m = 16'hFFF0;

        gnss_valid       = 1'b1;
        gnss_status      = `ST_OK;
        gnss_seq         = 16'h0202;
        gnss_age_ms      = 16'd30;
        gnss_fix_type    = 8'd3;

        wind_est_valid   = 1'b1;
        wind_est_status  = `ST_OK;
        wind_est_seq     = 16'h0303;
        wind_est_age_ms  = 16'd40;
        wind_est_x_cms   = 16'd120;
        wind_est_y_cms   = 16'hFFE2;
        wind_est_z_cms   = 16'd5;

        now_us = 32'd3000;
        pulse_sample();
        expect(nav_valid == 1'b1, "fresh EKF and GNSS publish valid nav");
        expect(nav_status == `ST_OK, "fresh EKF and GNSS publish OK nav status");
        expect(nav_flags[0] == 1'b1, "nav flag records fresh EKF");
        expect(nav_flags[1] == 1'b1, "nav flag records GNSS requirement satisfied");
        expect(nav_flags[3] == 1'b1, "nav flag records real producer bound");
        expect(nav_flags[5] == 1'b1, "nav flag records render-ready nav");
        expect(nav_downrange_m == 16'd42, "nav downrange comes from EKF input");
        expect(nav_crossrange_m == 16'hFFF0, "nav crossrange comes from EKF input");
        expect(nav_age_ms == 16'd30, "nav age is max EKF/GNSS source age");

        expect(wind_valid == 1'b1, "fresh wind estimator publishes valid wind");
        expect(wind_status == `ST_OK, "fresh wind estimator publishes OK status");
        expect(wind_flags[0] == 1'b1, "wind flag records fresh wind estimator");
        expect(wind_flags[3] == 1'b1, "wind flag records real producer bound");
        expect(wind_x_cms == 16'd120, "wind x comes from wind estimator");
        expect(wind_y_cms == 16'hFFE2, "wind y comes from wind estimator");
        expect(wind_z_cms == 16'd5, "wind z comes from wind estimator");
        expect(wind_age_ms == 16'd40, "wind age comes from wind estimator");

        gnss_age_ms = 16'd101;
        now_us = 32'd4000;
        pulse_sample();
        expect(nav_valid == 1'b0, "stale GNSS suppresses nav valid");
        expect(nav_status == `ST_STALE_REJECT, "stale GNSS reports stale reject");
        expect(nav_flags[7] == 1'b1, "nav flag records stale/faulted source");
        expect(wind_valid == 1'b1, "GNSS stale does not invalidate independent wind estimate");

        gnss_age_ms = 16'd30;
        ekf_status = `ST_NUMERIC_FAULT;
        now_us = 32'd5000;
        pulse_sample();
        expect(nav_valid == 1'b0, "EKF fault suppresses nav valid");
        expect(nav_status == `ST_NUMERIC_FAULT, "EKF fault status propagates");

        ekf_status = `ST_OK;
        wind_est_age_ms = 16'd101;
        now_us = 32'd6000;
        pulse_sample();
        expect(wind_valid == 1'b0, "stale wind estimator suppresses wind valid");
        expect(wind_status == `ST_STALE_REJECT, "stale wind estimator reports stale reject");
        expect(wind_flags[7] == 1'b1, "wind flag records stale/faulted source");

        enable = 1'b0;
        @(posedge clk);
        #1;
        expect(nav_valid == 1'b0, "disable clears nav valid");
        expect(nav_status == `ST_NOT_INITIALIZED, "disable returns nav to not initialized");
        expect(wind_valid == 1'b0, "disable clears wind valid");
        expect(wind_status == `ST_NOT_INITIALIZED, "disable returns wind to not initialized");

        $display("PASS tb_nav_wind_snapshot_producer");
        $finish;
    end

endmodule

`default_nettype wire
