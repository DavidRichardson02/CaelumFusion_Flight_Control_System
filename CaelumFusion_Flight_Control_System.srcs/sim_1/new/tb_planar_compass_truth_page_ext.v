`timescale 1ns/1ps
`default_nettype none

`include "telemetry_defs_vh.vh"

module tb_planar_compass_truth_page_ext;
    reg sys_clk = 1'b0;
    reg pix_clk = 1'b0;
    always #5  sys_clk = ~sys_clk;
    always #20 pix_clk = ~pix_clk;

    reg sys_rst;
    reg pix_rst;
    reg page_enable_sys;

    reg [15:0] mag_seq;
    reg        mag_valid;
    reg [7:0]  mag_status;
    reg [47:0] mag_payload;
    reg [15:0] mag_age_ms;
    reg [15:0] mag1_seq;
    reg        mag1_valid;
    reg [7:0]  mag1_status;
    reg [47:0] mag1_payload;
    reg [15:0] mag1_age_ms;

    reg        der_valid;
    reg [7:0]  der_status;
    reg        der_head_fresh;
    reg [15:0] der_mag_seq_ref;
    reg [31:0] der_heading_mdeg;

    reg        ext_valid;
    reg [7:0]  ext_status;
    reg [15:0] ext_present_flags;
    reg [15:0] ext_fault_flags;
    reg [15:0] ext_mag_delta_l1;
    reg [15:0] ext_mag_norm_primary;
    reg [15:0] ext_mag_norm_secondary;
    reg        ext_mag_sequence_aligned;
    reg        ext_mag_disagreement;
    reg [3:0]  ext_mag_sector_delta;
    reg [15:0] ext_mag_norm_delta_l1;
    reg [15:0] ext_mag_iron_residual;
    reg [7:0]  ext_mag_cal_state;
    reg [7:0]  ext_mag_source_flags;
    reg [15:0] ext_mag_bridge_checksum;
    reg [15:0] ext_max_age_ms;

    reg [15:0] i2c_nack_count;
    reg [15:0] i2c_timeout_count;
    reg [15:0] txn_rate_hz;

    wire        vga_hsync_out;
    wire        vga_vsync_out;
    wire [11:0] vga_rgb_out;

    integer fail_count;

    planar_compass_truth_page_vga #(
        .SYS_CLK_HZ(100_000_000),
        .UI_UPDATE_HZ(100_000),
        .COMPASS_TRUTH_PAGE_DEFAULT(1)
    ) dut (
        .sys_clk(sys_clk),
        .sys_rst(sys_rst),
        .page_enable_sys(page_enable_sys),
        .mag_seq(mag_seq),
        .mag_valid(mag_valid),
        .mag_status(mag_status),
        .mag_payload(mag_payload),
        .mag_age_ms(mag_age_ms),
        .mag1_seq(mag1_seq),
        .mag1_valid(mag1_valid),
        .mag1_status(mag1_status),
        .mag1_payload(mag1_payload),
        .mag1_age_ms(mag1_age_ms),
        .der_valid(der_valid),
        .der_status(der_status),
        .der_head_fresh(der_head_fresh),
        .der_mag_seq_ref(der_mag_seq_ref),
        .der_heading_mdeg(der_heading_mdeg),
        .ext_valid(ext_valid),
        .ext_status(ext_status),
        .ext_present_flags(ext_present_flags),
        .ext_fault_flags(ext_fault_flags),
        .ext_mag_delta_l1(ext_mag_delta_l1),
        .ext_mag_norm_primary(ext_mag_norm_primary),
        .ext_mag_norm_secondary(ext_mag_norm_secondary),
        .ext_mag_sequence_aligned(ext_mag_sequence_aligned),
        .ext_mag_disagreement(ext_mag_disagreement),
        .ext_mag_sector_delta(ext_mag_sector_delta),
        .ext_mag_norm_delta_l1(ext_mag_norm_delta_l1),
        .ext_mag_iron_residual(ext_mag_iron_residual),
        .ext_mag_cal_state(ext_mag_cal_state),
        .ext_mag_source_flags(ext_mag_source_flags),
        .ext_mag_bridge_checksum(ext_mag_bridge_checksum),
        .ext_max_age_ms(ext_max_age_ms),
        .i2c_nack_count(i2c_nack_count),
        .i2c_timeout_count(i2c_timeout_count),
        .txn_rate_hz(txn_rate_hz),
        .pix_clk(pix_clk),
        .pix_rst(pix_rst),
        .vga_hsync_in(1'b0),
        .vga_vsync_in(1'b0),
        .vga_rgb_in(12'h000),
        .vga_hsync_out(vga_hsync_out),
        .vga_vsync_out(vga_vsync_out),
        .vga_rgb_out(vga_rgb_out)
    );

    task wait_cdc;
        begin
            repeat (2500) @(posedge sys_clk);
            repeat (24) @(posedge pix_clk);
        end
    endtask

    task expect_true;
        input condition;
        input [8*96-1:0] message;
        begin
            if (!condition) begin
                $display("FAIL: %0s", message);
                fail_count = fail_count + 1;
            end
        end
    endtask

    initial begin
        fail_count = 0;
        sys_rst = 1'b1;
        pix_rst = 1'b1;
        page_enable_sys = 1'b1;

        mag_seq = 16'h0101;
        mag_valid = 1'b1;
        mag_status = `ST_OK;
        mag_payload = {16'h0010, 16'h0020, 16'h0030};
        mag_age_ms = 16'd12;
        mag1_seq = 16'h0101;
        mag1_valid = 1'b1;
        mag1_status = `ST_OK;
        mag1_payload = {16'h0011, 16'h0021, 16'h0031};
        mag1_age_ms = 16'd13;

        der_valid = 1'b1;
        der_status = `ST_OK;
        der_head_fresh = 1'b1;
        der_mag_seq_ref = 16'h0101;
        der_heading_mdeg = 32'd45000;

        ext_valid = 1'b1;
        ext_status = `ST_OK;
        ext_present_flags = (16'd1 << `EXT_PRESENT_MAG0_BIT) |
                            (16'd1 << `EXT_PRESENT_MAG1_BIT);
        ext_fault_flags = 16'd0;
        ext_mag_delta_l1 = 16'd9;
        ext_mag_norm_primary = 16'd600;
        ext_mag_norm_secondary = 16'd605;
        ext_mag_sequence_aligned = 1'b1;
        ext_mag_disagreement = 1'b0;
        ext_mag_sector_delta = 4'd0;
        ext_mag_norm_delta_l1 = 16'd5;
        ext_mag_iron_residual = 16'd5;
        ext_mag_cal_state = 8'h80;
        ext_mag_source_flags = (8'd1 << `EXT_SRC_SYNTHETIC_BIT);
        ext_mag_bridge_checksum = 16'h1A2B;
        ext_max_age_ms = 16'd21;

        i2c_nack_count = 16'd0;
        i2c_timeout_count = 16'd0;
        txn_rate_hz = 16'd50;

        repeat (12) @(posedge sys_clk);
        sys_rst = 1'b0;
        pix_rst = 1'b0;

        wait_cdc();
        expect_true(dut.ext_valid_pix == 1'b1, "extension valid reached pixel bundle");
        expect_true(dut.ext_status_pix == `ST_OK, "extension status reached pixel bundle");
        expect_true(dut.ext_mag_good_w == 1'b1, "healthy redundant magnetometer pair is marked good");
        expect_true(dut.ext_delta_bar_w == 9'd9, "small L1 delta maps directly into the evidence bar");
        expect_true(dut.ext_mag_norm_primary_pix == 16'd600, "primary norm reached pixel bundle");
        expect_true(dut.ext_mag_norm_secondary_pix == 16'd605, "secondary norm reached pixel bundle");
        expect_true(dut.mag1_seq_pix == 16'h0101, "MAG1 sequence reached pixel bundle");
        expect_true(dut.mag1_payload_pix == {16'h0011, 16'h0021, 16'h0031}, "MAG1 payload reached pixel bundle");
        expect_true(dut.ext_mag_sequence_aligned_pix == 1'b1, "MAG sequence alignment reached pixel bundle");
        expect_true(dut.ext_mag_source_flags_pix[`EXT_SRC_SYNTHETIC_BIT] == 1'b1, "MAG1 synthetic source flag reached pixel bundle");
        expect_true(dut.ext_mag_bridge_checksum_pix == 16'h1A2B, "MAG1 checksum reached pixel bundle");

        ext_status = `ST_PLAUSIBILITY_REJECT;
        ext_fault_flags = (16'd1 << `EXT_FLG_MAG_DISAGREE_BIT);
        ext_mag_disagreement = 1'b1;
        ext_mag_sequence_aligned = 1'b0;
        ext_mag_sector_delta = 4'd2;
        ext_mag_delta_l1 = 16'd300;
        ext_mag_norm_delta_l1 = 16'd80;
        ext_mag_iron_residual = 16'd80;
        wait_cdc();
        expect_true(dut.ext_mag_disagree_w == 1'b1, "magnetic disagreement flag reached pixel logic");
        expect_true(dut.ext_mag_disagreement_pix == 1'b1, "explicit magnetic disagreement detail reached pixel bundle");
        expect_true(dut.ext_mag_hard_fault_w == 1'b1, "magnetic disagreement is a hard evidence fault");
        expect_true(dut.ext_mag_good_w == 1'b0, "faulted redundant pair is not marked good");
        expect_true(dut.ext_delta_bar_w == 9'd240, "large L1 delta saturates the evidence bar");
        expect_true(dut.ext_mag_sector_delta_pix == 4'd2, "sector delta reached pixel bundle");

        ext_status = `ST_MISSING_INPUT;
        ext_present_flags = (16'd1 << `EXT_PRESENT_MAG0_BIT);
        ext_fault_flags = (16'd1 << `EXT_FLG_MAG_PAIR_MISSING_BIT);
        ext_mag_disagreement = 1'b0;
        ext_mag_delta_l1 = 16'd0;
        ext_mag_norm_secondary = 16'd0;
        mag1_valid = 1'b0;
        mag1_status = `ST_NOT_INITIALIZED;
        ext_mag_source_flags = 8'd0;
        wait_cdc();
        expect_true(dut.ext_mag_pair_missing_w == 1'b1, "pair-missing flag reached pixel logic");
        expect_true(dut.ext_mag1_present_w == 1'b0, "secondary present bit is cleared");
        expect_true(dut.ext_mag_good_w == 1'b0, "single magnetometer is not marked as a redundant good pair");

        if (fail_count == 0) begin
            $display("PASS: tb_planar_compass_truth_page_ext");
        end else begin
            $display("FAIL: tb_planar_compass_truth_page_ext failures=%0d", fail_count);
        end
        $finish;
    end
endmodule

`default_nettype wire
