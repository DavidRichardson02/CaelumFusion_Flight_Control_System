`timescale 1ns / 1ps
`default_nettype none

`include "telemetry_defs_vh.vh"

module tb_landing_nav_wind_observer;

    reg         der_valid;
    reg  [7:0]  der_status;
    reg         der_alt_fresh;
    reg         der_vspd_fresh;
    reg  [15:0] der_bmp_age_ms;
    reg  [31:0] der_altitude_cm;

    reg         ext_valid;
    reg  [7:0]  ext_status;
    reg  [15:0] ext_present_flags;
    reg  [15:0] ext_fault_flags;
    reg  [15:0] ext_air_speed_cms;
    reg  [15:0] ext_flow_dx;
    reg  [15:0] ext_flow_dy;
    reg  [15:0] ext_max_age_ms;

    wire        nav_valid;
    wire [7:0]  nav_status;
    wire [7:0]  nav_flags;
    wire [15:0] nav_downrange_m;
    wire [15:0] nav_crossrange_m;
    wire [15:0] nav_age_ms;

    wire        wind_valid;
    wire [7:0]  wind_status;
    wire [15:0] wind_x_cms;
    wire [15:0] wind_y_cms;
    wire [15:0] wind_z_cms;
    wire [15:0] wind_age_ms;

    landing_nav_wind_observer dut (
        .der_valid         (der_valid),
        .der_status        (der_status),
        .der_alt_fresh     (der_alt_fresh),
        .der_vspd_fresh    (der_vspd_fresh),
        .der_bmp_age_ms    (der_bmp_age_ms),
        .der_altitude_cm   (der_altitude_cm),
        .ext_valid         (ext_valid),
        .ext_status        (ext_status),
        .ext_present_flags (ext_present_flags),
        .ext_fault_flags   (ext_fault_flags),
        .ext_air_speed_cms (ext_air_speed_cms),
        .ext_flow_dx       (ext_flow_dx),
        .ext_flow_dy       (ext_flow_dy),
        .ext_max_age_ms    (ext_max_age_ms),
        .nav_valid         (nav_valid),
        .nav_status        (nav_status),
        .nav_flags         (nav_flags),
        .nav_downrange_m   (nav_downrange_m),
        .nav_crossrange_m  (nav_crossrange_m),
        .nav_age_ms        (nav_age_ms),
        .wind_valid        (wind_valid),
        .wind_status       (wind_status),
        .wind_x_cms        (wind_x_cms),
        .wind_y_cms        (wind_y_cms),
        .wind_z_cms        (wind_z_cms),
        .wind_age_ms       (wind_age_ms)
    );

    task expect;
        input condition;
        input [160*8-1:0] message;
        begin
            if (!condition) begin
                $display("FAIL: %0s", message);
                $finish;
            end
        end
    endtask

    task clear_inputs;
        begin
            der_valid         = 1'b0;
            der_status        = `ST_MISSING_INPUT;
            der_alt_fresh     = 1'b0;
            der_vspd_fresh    = 1'b0;
            der_bmp_age_ms    = 16'hFFFF;
            der_altitude_cm   = 32'd0;
            ext_valid         = 1'b0;
            ext_status        = `ST_MISSING_INPUT;
            ext_present_flags = 16'd0;
            ext_fault_flags   = 16'd0;
            ext_air_speed_cms = 16'd0;
            ext_flow_dx       = 16'd0;
            ext_flow_dy       = 16'd0;
            ext_max_age_ms    = 16'hFFFF;
        end
    endtask

    initial begin
        clear_inputs();
        #1;
        expect(nav_valid == 1'b0, "compat shim keeps nav invalid when real producer is absent");
        expect(nav_status == `ST_MISSING_INPUT, "compat shim reports missing real nav producer");
        expect(wind_valid == 1'b0, "compat shim keeps wind invalid when real producer is absent");
        expect(wind_status == `ST_MISSING_INPUT, "compat shim reports missing real wind producer");
        expect(nav_age_ms == 16'hFFFF, "missing nav age is saturated");
        expect(wind_age_ms == 16'hFFFF, "missing wind age is saturated");

        clear_inputs();
        der_valid       = 1'b1;
        der_status      = `ST_OK;
        der_alt_fresh   = 1'b1;
        der_vspd_fresh  = 1'b1;
        der_bmp_age_ms  = 16'd24;
        der_altitude_cm = 32'd305000;
        #1;
        expect(nav_valid == 1'b0, "baro-derived altitude cannot publish nav valid");
        expect(nav_status == `ST_MISSING_INPUT, "baro-derived altitude leaves nav missing");
        expect(nav_flags == 8'h10, "compat shim exposes degraded/unbound nav contract");
        expect(nav_downrange_m == 16'd0, "compat shim does not derive downrange from altitude");
        expect(nav_age_ms == 16'hFFFF, "nav age stays saturated without real producer");

        clear_inputs();
        der_valid         = 1'b1;
        der_status        = `ST_OK;
        der_alt_fresh     = 1'b1;
        der_vspd_fresh    = 1'b1;
        der_bmp_age_ms    = 16'd35;
        der_altitude_cm   = 32'd305000;
        ext_valid         = 1'b1;
        ext_status        = `ST_OK;
        ext_present_flags = (16'd1 << `EXT_PRESENT_AIR_BIT) |
                            (16'd1 << `EXT_PRESENT_FLOW_BIT);
        ext_air_speed_cms = 16'd420;
        ext_flow_dx       = 16'h0100;
        ext_flow_dy       = 16'hFFE0;
        ext_max_age_ms    = 16'd60;
        #1;
        expect(nav_valid == 1'b0, "flow and air evidence cannot publish nav valid");
        expect(nav_status == `ST_MISSING_INPUT, "flow and air evidence leave nav missing");
        expect(nav_crossrange_m == 16'd0, "compat shim does not derive crossrange from flow");
        expect(wind_valid == 1'b0, "raw air/flow evidence cannot publish wind valid");
        expect(wind_status == `ST_MISSING_INPUT, "raw air/flow evidence leaves wind missing");
        expect(wind_x_cms == 16'd0, "compat shim does not derive wind x from airspeed");
        expect(wind_y_cms == 16'd0, "compat shim does not derive wind y from flow");
        expect(wind_z_cms == 16'd0, "wind z remains zero while producer is missing");
        expect(wind_age_ms == 16'hFFFF, "wind age stays saturated without real producer");

        ext_max_age_ms = 16'd1200;
        #1;
        expect(nav_status == `ST_MISSING_INPUT, "stale proxy evidence still cannot publish nav");
        expect(wind_status == `ST_MISSING_INPUT, "stale proxy evidence still cannot publish wind");

        ext_max_age_ms  = 16'd60;
        ext_fault_flags = 16'h0001;
        #1;
        expect(nav_status == `ST_MISSING_INPUT, "faulted proxy evidence still cannot publish nav");
        expect(wind_status == `ST_MISSING_INPUT, "faulted proxy evidence still cannot publish wind");
        expect(nav_flags[7] == 1'b0, "compat shim does not promote extension faults into nav producer evidence");

        $display("PASS tb_landing_nav_wind_observer");
        $finish;
    end

endmodule

`default_nettype wire
