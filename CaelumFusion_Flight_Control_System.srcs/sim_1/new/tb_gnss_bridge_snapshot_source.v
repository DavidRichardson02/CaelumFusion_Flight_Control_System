`timescale 1ns / 1ps
`default_nettype none

`include "telemetry_defs_vh.vh"

module tb_gnss_bridge_snapshot_source;

    reg clk;
    reg rst;
    reg enable;
    reg [31:0] now_us;
    reg tick_1us;

    reg        pkt_valid;
    wire       pkt_ready;
    reg [15:0] pkt_seq;
    reg [7:0]  pkt_status;
    reg [7:0]  pkt_fix_type;
    reg [7:0]  pkt_num_sats;
    reg [15:0] pkt_hdop_centi;
    reg [31:0] pkt_lat_e7;
    reg [31:0] pkt_lon_e7;
    reg [31:0] pkt_alt_cm_msl;
    reg [15:0] pkt_vel_n_cms;
    reg [15:0] pkt_vel_e_cms;
    reg [15:0] pkt_vel_d_cms;
    reg [15:0] pkt_ground_speed_cms;
    reg [31:0] pkt_course_mdeg;
    reg [15:0] pkt_source_flags;
    reg [15:0] pkt_checksum;
    reg        pps_pulse;

    wire [31:0] gnss_t_us;
    wire [15:0] gnss_seq;
    wire        gnss_valid;
    wire [7:0]  gnss_status;
    wire [15:0] gnss_age_ms;
    wire [7:0]  gnss_fix_type;
    wire [7:0]  gnss_num_sats;
    wire [15:0] gnss_hdop_centi;
    wire [31:0] gnss_lat_e7;
    wire [31:0] gnss_lon_e7;
    wire [31:0] gnss_alt_cm_msl;
    wire [15:0] gnss_vel_n_cms;
    wire [15:0] gnss_vel_e_cms;
    wire [15:0] gnss_vel_d_cms;
    wire [15:0] gnss_ground_speed_cms;
    wire [31:0] gnss_course_mdeg;
    wire        gnss_pps_seen;
    wire [15:0] gnss_pps_seq;
    wire [15:0] gnss_pps_age_ms;
    wire [15:0] gnss_source_flags;
    wire [15:0] gnss_checksum;
    wire [15:0] gnss_checksum_fault_count;

    gnss_bridge_snapshot_source #(
        .GNSS_FRESH_MAX_MS     (16'd3),
        .PPS_FRESH_MAX_MS      (16'd2),
        .MIN_FIX_TYPE          (8'd3),
        .MIN_NUM_SATS          (8'd4),
        .MAX_HDOP_CENTI        (16'd500),
        .REQUIRE_PPS_FOR_VALID (0)
    ) dut (
        .clk                       (clk),
        .rst                       (rst),
        .enable                    (enable),
        .now_us                    (now_us),
        .tick_1us                  (tick_1us),
        .pkt_valid                 (pkt_valid),
        .pkt_ready                 (pkt_ready),
        .pkt_seq                   (pkt_seq),
        .pkt_status                (pkt_status),
        .pkt_fix_type              (pkt_fix_type),
        .pkt_num_sats              (pkt_num_sats),
        .pkt_hdop_centi            (pkt_hdop_centi),
        .pkt_lat_e7                (pkt_lat_e7),
        .pkt_lon_e7                (pkt_lon_e7),
        .pkt_alt_cm_msl            (pkt_alt_cm_msl),
        .pkt_vel_n_cms             (pkt_vel_n_cms),
        .pkt_vel_e_cms             (pkt_vel_e_cms),
        .pkt_vel_d_cms             (pkt_vel_d_cms),
        .pkt_ground_speed_cms      (pkt_ground_speed_cms),
        .pkt_course_mdeg           (pkt_course_mdeg),
        .pkt_source_flags          (pkt_source_flags),
        .pkt_checksum              (pkt_checksum),
        .pps_pulse                 (pps_pulse),
        .gnss_t_us                 (gnss_t_us),
        .gnss_seq                  (gnss_seq),
        .gnss_valid                (gnss_valid),
        .gnss_status               (gnss_status),
        .gnss_age_ms               (gnss_age_ms),
        .gnss_fix_type             (gnss_fix_type),
        .gnss_num_sats             (gnss_num_sats),
        .gnss_hdop_centi           (gnss_hdop_centi),
        .gnss_lat_e7               (gnss_lat_e7),
        .gnss_lon_e7               (gnss_lon_e7),
        .gnss_alt_cm_msl           (gnss_alt_cm_msl),
        .gnss_vel_n_cms            (gnss_vel_n_cms),
        .gnss_vel_e_cms            (gnss_vel_e_cms),
        .gnss_vel_d_cms            (gnss_vel_d_cms),
        .gnss_ground_speed_cms     (gnss_ground_speed_cms),
        .gnss_course_mdeg          (gnss_course_mdeg),
        .gnss_pps_seen             (gnss_pps_seen),
        .gnss_pps_seq              (gnss_pps_seq),
        .gnss_pps_age_ms           (gnss_pps_age_ms),
        .gnss_source_flags         (gnss_source_flags),
        .gnss_checksum             (gnss_checksum),
        .gnss_checksum_fault_count (gnss_checksum_fault_count)
    );

    always #5 clk = ~clk;

    function [15:0] checksum16;
        input [15:0] seq;
        input [7:0]  status;
        input [7:0]  fix_type;
        input [7:0]  num_sats;
        input [15:0] hdop_centi;
        input [31:0] lat_e7;
        input [31:0] lon_e7;
        input [31:0] alt_cm_msl;
        input [15:0] vel_n_cms;
        input [15:0] vel_e_cms;
        input [15:0] vel_d_cms;
        input [15:0] ground_speed_cms;
        input [31:0] course_mdeg;
        input [15:0] source_flags;
        begin
            checksum16 =
                seq ^
                {status, fix_type} ^
                {num_sats, 8'h00} ^
                hdop_centi ^
                lat_e7[31:16] ^ lat_e7[15:0] ^
                lon_e7[31:16] ^ lon_e7[15:0] ^
                alt_cm_msl[31:16] ^ alt_cm_msl[15:0] ^
                vel_n_cms ^ vel_e_cms ^ vel_d_cms ^
                ground_speed_cms ^
                course_mdeg[31:16] ^ course_mdeg[15:0] ^
                source_flags;
        end
    endfunction

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

    task clear_packet;
        begin
            pkt_valid            = 1'b0;
            pkt_seq              = 16'd0;
            pkt_status           = `ST_NOT_INITIALIZED;
            pkt_fix_type         = 8'd0;
            pkt_num_sats         = 8'd0;
            pkt_hdop_centi       = 16'd0;
            pkt_lat_e7           = 32'd0;
            pkt_lon_e7           = 32'd0;
            pkt_alt_cm_msl       = 32'd0;
            pkt_vel_n_cms        = 16'd0;
            pkt_vel_e_cms        = 16'd0;
            pkt_vel_d_cms        = 16'd0;
            pkt_ground_speed_cms = 16'd0;
            pkt_course_mdeg      = 32'd0;
            pkt_source_flags     = 16'd0;
            pkt_checksum         = 16'd0;
        end
    endtask

    task drive_valid_fix_fields;
        input [15:0] seq;
        begin
            pkt_seq              = seq;
            pkt_status           = `ST_OK;
            pkt_fix_type         = 8'd3;
            pkt_num_sats         = 8'd9;
            pkt_hdop_centi       = 16'd85;
            pkt_lat_e7           = 32'h16F2_3A44;
            pkt_lon_e7           = 32'hC12B_9910;
            pkt_alt_cm_msl       = 32'd154320;
            pkt_vel_n_cms        = 16'd120;
            pkt_vel_e_cms        = 16'hFFE2;
            pkt_vel_d_cms        = 16'd5;
            pkt_ground_speed_cms = 16'd124;
            pkt_course_mdeg      = 32'd92345;
            pkt_source_flags     = (16'd1 << `EXT_SRC_REAL_BIT) |
                                   (16'd1 << `EXT_SRC_TEENSY_BRIDGE_BIT);
            pkt_checksum         = checksum16(
                pkt_seq, pkt_status, pkt_fix_type, pkt_num_sats,
                pkt_hdop_centi, pkt_lat_e7, pkt_lon_e7, pkt_alt_cm_msl,
                pkt_vel_n_cms, pkt_vel_e_cms, pkt_vel_d_cms,
                pkt_ground_speed_cms, pkt_course_mdeg, pkt_source_flags
            );
        end
    endtask

    task send_packet;
        begin
            @(negedge clk);
            expect(pkt_ready == 1'b1, "packet interface ready before send");
            pkt_valid = 1'b1;
            @(negedge clk);
            pkt_valid = 1'b0;
            @(posedge clk);
            #1;
        end
    endtask

    task wait_us_ticks;
        input integer ticks;
        integer i;
        begin
            for (i = 0; i < ticks; i = i + 1) begin
                @(negedge clk);
                tick_1us = 1'b1;
                now_us = now_us + 32'd1;
                @(negedge clk);
                tick_1us = 1'b0;
            end
            @(posedge clk);
            #1;
        end
    endtask

    task pulse_pps;
        begin
            @(negedge clk);
            pps_pulse = 1'b1;
            @(negedge clk);
            pps_pulse = 1'b0;
            @(posedge clk);
            #1;
        end
    endtask

    initial begin
        clk = 1'b0;
        rst = 1'b1;
        enable = 1'b0;
        now_us = 32'd0;
        tick_1us = 1'b0;
        pps_pulse = 1'b0;
        clear_packet();

        repeat (3) @(posedge clk);
        rst = 1'b0;
        @(posedge clk);
        #1;

        expect(pkt_ready == 1'b0, "bridge not ready while disabled");
        expect(gnss_valid == 1'b0, "reset clears GNSS valid");
        expect(gnss_status == `ST_NOT_INITIALIZED, "reset status is not initialized");
        expect(gnss_age_ms == 16'hFFFF, "reset GNSS age is saturated");
        expect(gnss_pps_seen == 1'b0, "reset clears PPS seen");
        expect(gnss_pps_age_ms == 16'hFFFF, "reset PPS age is saturated");
        expect(gnss_checksum_fault_count == 16'd0, "reset clears checksum fault count");

        enable = 1'b1;
        @(posedge clk);
        #1;
        expect(pkt_ready == 1'b1, "bridge ready when enabled");

        drive_valid_fix_fields(16'h0101);
        send_packet();
        expect(gnss_valid == 1'b1, "valid fix publishes GNSS valid");
        expect(gnss_status == `ST_OK, "valid fix publishes OK status");
        expect(gnss_seq == 16'h0101, "packet sequence publishes");
        expect(gnss_age_ms == 16'd0, "valid fix resets GNSS age");
        expect(gnss_fix_type == 8'd3, "fix type publishes");
        expect(gnss_num_sats == 8'd9, "satellite count publishes");
        expect(gnss_hdop_centi == 16'd85, "HDOP publishes");
        expect(gnss_lat_e7 == 32'h16F2_3A44, "latitude publishes");
        expect(gnss_lon_e7 == 32'hC12B_9910, "longitude publishes");
        expect(gnss_alt_cm_msl == 32'd154320, "altitude publishes");
        expect(gnss_vel_e_cms == 16'hFFE2, "signed east velocity raw field publishes");
        expect(gnss_source_flags[0] == 1'b1, "source flags preserve real bit");
        expect(gnss_source_flags[1] == 1'b1, "source flags preserve bridge bit");
        expect(gnss_checksum == pkt_checksum, "checksum publishes");

        pulse_pps();
        expect(gnss_pps_seen == 1'b1, "PPS pulse sets seen flag");
        expect(gnss_pps_seq == 16'd1, "PPS pulse increments PPS sequence");
        expect(gnss_pps_age_ms == 16'd0, "PPS pulse resets PPS age");

        wait_us_ticks(1000);
        expect(gnss_age_ms == 16'd1, "GNSS age increments by one ms");
        expect(gnss_pps_age_ms == 16'd1, "PPS age increments by one ms");

        wait_us_ticks(3000);
        expect(gnss_valid == 1'b0, "stale fix clears GNSS valid");
        expect(gnss_status == `ST_STALE_REJECT, "stale fix reports stale reject");
        expect(gnss_age_ms > 16'd3, "stale age exceeds freshness threshold");
        expect(gnss_pps_age_ms > 16'd2, "PPS age can exceed PPS freshness threshold");

        drive_valid_fix_fields(16'h0102);
        pkt_checksum = pkt_checksum ^ 16'h0001;
        send_packet();
        expect(gnss_valid == 1'b0, "checksum fault suppresses GNSS valid");
        expect(gnss_status == `ST_CONFIG_ERROR, "checksum fault reports config error");
        expect(gnss_checksum_fault_count == 16'd1, "checksum fault count increments");
        expect(gnss_lat_e7 == 32'd0, "checksum fault clears position fields");
        expect(gnss_age_ms == 16'd0, "checksum fault commits diagnostic snapshot age");

        drive_valid_fix_fields(16'h0103);
        pkt_fix_type = 8'd1;
        pkt_checksum = checksum16(
            pkt_seq, pkt_status, pkt_fix_type, pkt_num_sats,
            pkt_hdop_centi, pkt_lat_e7, pkt_lon_e7, pkt_alt_cm_msl,
            pkt_vel_n_cms, pkt_vel_e_cms, pkt_vel_d_cms,
            pkt_ground_speed_cms, pkt_course_mdeg, pkt_source_flags
        );
        send_packet();
        expect(gnss_valid == 1'b0, "low fix type suppresses GNSS valid");
        expect(gnss_status == `ST_DATA_NOT_READY, "low fix type reports data not ready");

        $display("PASS tb_gnss_bridge_snapshot_source");
        $finish;
    end

endmodule

`default_nettype wire
