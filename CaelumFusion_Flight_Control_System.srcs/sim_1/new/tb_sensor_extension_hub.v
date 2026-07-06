`timescale 1ns/1ps
`default_nettype none

`include "telemetry_defs_vh.vh"

module tb_sensor_extension_hub;
    localparam integer PAYLOAD_W = 48;

    reg clk = 1'b0;
    always #5 clk = ~clk;

    reg rst;

    reg [31:0] bmp_t_us;
    reg [15:0] bmp_seq;
    reg        bmp_valid;
    reg [7:0]  bmp_status;
    reg [PAYLOAD_W-1:0] bmp_payload;
    reg [15:0] bmp_age_ms;

    reg [31:0] acc_t_us;
    reg [15:0] acc_seq;
    reg        acc_valid;
    reg [7:0]  acc_status;
    reg [PAYLOAD_W-1:0] acc_payload;
    reg [15:0] acc_age_ms;

    reg [31:0] mag_t_us;
    reg [15:0] mag_seq;
    reg        mag_valid;
    reg [7:0]  mag_status;
    reg [PAYLOAD_W-1:0] mag_payload;
    reg [15:0] mag_age_ms;

    reg [31:0] pwr_t_us;
    reg [15:0] pwr_seq;
    reg        pwr_valid;
    reg [7:0]  pwr_status;
    reg [PAYLOAD_W-1:0] pwr_payload;
    reg [15:0] pwr_age_ms;

    reg [31:0] mag1_t_us;
    reg [15:0] mag1_seq;
    reg        mag1_valid;
    reg [7:0]  mag1_status;
    reg [PAYLOAD_W-1:0] mag1_payload;
    reg [15:0] mag1_age_ms;
    reg [7:0]  mag1_cal_state;
    reg [7:0]  mag1_source_flags;
    reg [15:0] mag1_bridge_checksum;

    reg [31:0] rng_t_us;
    reg [15:0] rng_seq;
    reg        rng_valid;
    reg [7:0]  rng_status;
    reg [PAYLOAD_W-1:0] rng_payload;
    reg [15:0] rng_age_ms;

    reg [31:0] air_t_us;
    reg [15:0] air_seq;
    reg        air_valid;
    reg [7:0]  air_status;
    reg [PAYLOAD_W-1:0] air_payload;
    reg [15:0] air_age_ms;

    reg [31:0] env_t_us;
    reg [15:0] env_seq;
    reg        env_valid;
    reg [7:0]  env_status;
    reg [PAYLOAD_W-1:0] env_payload;
    reg [15:0] env_age_ms;

    reg [31:0] sun_t_us;
    reg [15:0] sun_seq;
    reg        sun_valid;
    reg [7:0]  sun_status;
    reg [PAYLOAD_W-1:0] sun_payload;
    reg [15:0] sun_age_ms;

    reg [31:0] flow_t_us;
    reg [15:0] flow_seq;
    reg        flow_valid;
    reg [7:0]  flow_status;
    reg [PAYLOAD_W-1:0] flow_payload;
    reg [15:0] flow_age_ms;

    reg        diag_selftest_enable;
    reg        diag_fault_inject_enable;
    reg [3:0]  diag_fault_mode;
    reg        log_runtime_enable;
    reg        log_emit_req;
    reg        log_stream_ready;

    wire       log_stream_valid;
    wire [31:0] log_stream_word;
    wire       log_stream_last;

    wire       ext_valid;
    wire [7:0] ext_status;
    wire [15:0] ext_present_flags;
    wire [15:0] ext_fault_flags;
    wire [15:0] ext_mag_delta_l1;
    wire [15:0] ext_mag_norm_primary;
    wire [15:0] ext_mag_norm_secondary;
    wire        ext_mag_sequence_aligned;
    wire        ext_mag_disagreement;
    wire [3:0]  ext_mag_sector_delta;
    wire [15:0] ext_mag_norm_delta_l1;
    wire [15:0] ext_mag_iron_residual;
    wire [7:0]  ext_mag_cal_state;
    wire [7:0]  ext_mag_source_flags;
    wire [15:0] ext_mag_bridge_checksum;
    wire [15:0] ext_rng_height_cm;
    wire [15:0] ext_air_dp_pa;
    wire [15:0] ext_air_speed_cms;
    wire [15:0] ext_env_temp_cdeg;
    wire [15:0] ext_env_rh_centi;
    wire [15:0] ext_sun_luma;
    wire [15:0] ext_flow_dx;
    wire [15:0] ext_flow_dy;
    wire [15:0] ext_log_seq;
    wire [15:0] ext_log_drop_count;
    wire [15:0] ext_max_age_ms;

    sensor_extension_hub #(
        .PAYLOAD_W(PAYLOAD_W),
        .ENABLE_BLACKBOX_LOG(1)
    ) dut (
        .clk(clk),
        .rst(rst),
        .bmp_t_us(bmp_t_us),
        .bmp_seq(bmp_seq),
        .bmp_valid(bmp_valid),
        .bmp_status(bmp_status),
        .bmp_payload(bmp_payload),
        .bmp_age_ms(bmp_age_ms),
        .acc_t_us(acc_t_us),
        .acc_seq(acc_seq),
        .acc_valid(acc_valid),
        .acc_status(acc_status),
        .acc_payload(acc_payload),
        .acc_age_ms(acc_age_ms),
        .mag_t_us(mag_t_us),
        .mag_seq(mag_seq),
        .mag_valid(mag_valid),
        .mag_status(mag_status),
        .mag_payload(mag_payload),
        .mag_age_ms(mag_age_ms),
        .pwr_t_us(pwr_t_us),
        .pwr_seq(pwr_seq),
        .pwr_valid(pwr_valid),
        .pwr_status(pwr_status),
        .pwr_payload(pwr_payload),
        .pwr_age_ms(pwr_age_ms),
        .mag1_t_us(mag1_t_us),
        .mag1_seq(mag1_seq),
        .mag1_valid(mag1_valid),
        .mag1_status(mag1_status),
        .mag1_payload(mag1_payload),
        .mag1_age_ms(mag1_age_ms),
        .mag1_cal_state(mag1_cal_state),
        .mag1_source_flags(mag1_source_flags),
        .mag1_bridge_checksum(mag1_bridge_checksum),
        .rng_t_us(rng_t_us),
        .rng_seq(rng_seq),
        .rng_valid(rng_valid),
        .rng_status(rng_status),
        .rng_payload(rng_payload),
        .rng_age_ms(rng_age_ms),
        .air_t_us(air_t_us),
        .air_seq(air_seq),
        .air_valid(air_valid),
        .air_status(air_status),
        .air_payload(air_payload),
        .air_age_ms(air_age_ms),
        .env_t_us(env_t_us),
        .env_seq(env_seq),
        .env_valid(env_valid),
        .env_status(env_status),
        .env_payload(env_payload),
        .env_age_ms(env_age_ms),
        .sun_t_us(sun_t_us),
        .sun_seq(sun_seq),
        .sun_valid(sun_valid),
        .sun_status(sun_status),
        .sun_payload(sun_payload),
        .sun_age_ms(sun_age_ms),
        .flow_t_us(flow_t_us),
        .flow_seq(flow_seq),
        .flow_valid(flow_valid),
        .flow_status(flow_status),
        .flow_payload(flow_payload),
        .flow_age_ms(flow_age_ms),
        .diag_selftest_enable(diag_selftest_enable),
        .diag_fault_inject_enable(diag_fault_inject_enable),
        .diag_fault_mode(diag_fault_mode),
        .log_runtime_enable(log_runtime_enable),
        .log_emit_req(log_emit_req),
        .log_stream_ready(log_stream_ready),
        .log_stream_valid(log_stream_valid),
        .log_stream_word(log_stream_word),
        .log_stream_last(log_stream_last),
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
        .ext_rng_height_cm(ext_rng_height_cm),
        .ext_air_dp_pa(ext_air_dp_pa),
        .ext_air_speed_cms(ext_air_speed_cms),
        .ext_env_temp_cdeg(ext_env_temp_cdeg),
        .ext_env_rh_centi(ext_env_rh_centi),
        .ext_sun_luma(ext_sun_luma),
        .ext_flow_dx(ext_flow_dx),
        .ext_flow_dy(ext_flow_dy),
        .ext_log_seq(ext_log_seq),
        .ext_log_drop_count(ext_log_drop_count),
        .ext_max_age_ms(ext_max_age_ms)
    );

    task fail;
        input [8*160-1:0] msg;
        begin
            $display("FAIL: %0s", msg);
            $finish;
        end
    endtask

    task expect;
        input condition;
        input [8*160-1:0] msg;
        begin
            if (!condition)
                fail(msg);
        end
    endtask

    task tick;
        begin
            @(posedge clk);
            #1;
        end
    endtask

    task expect_frame_word;
        input integer idx;
        input [31:0] expected_word;
        input [8*160-1:0] msg;
        begin
            expect(log_stream_valid == 1'b1, "blackbox stream is valid while frame is emitted");
            expect(log_stream_word == expected_word, msg);
            expect(log_stream_last == (idx == 28), "blackbox frame last flag matches word index");
        end
    endtask

    task expect_live_mag_blackbox_frame;
        integer idx;
        reg [31:0] expected_mag_meta;
        begin
            expected_mag_meta = 32'd0;
            expected_mag_meta[`EXT_MAG_META_SRC_FLAGS_MSB:`EXT_MAG_META_SRC_FLAGS_LSB] =
                (8'd1 << `EXT_SRC_REAL_BIT);
            expected_mag_meta[`EXT_MAG_META_CAL_STATE_MSB:`EXT_MAG_META_CAL_STATE_LSB] =
                8'h21;
            expected_mag_meta[`EXT_MAG_META_SECTOR_DELTA_MSB:`EXT_MAG_META_SECTOR_DELTA_LSB] =
                4'd1;
            expected_mag_meta[`EXT_MAG_META_DISAGREE_BIT] = 1'b0;
            expected_mag_meta[`EXT_MAG_META_SEQ_ALIGNED_BIT] = 1'b1;

            log_runtime_enable = 1'b1;
            log_emit_req = 1'b1;
            tick();
            log_emit_req = 1'b0;

            for (idx = 0; idx < 29; idx = idx + 1) begin
                case (idx)
                    0:  expect_frame_word(idx, {`TELEM_PKT_SYNC, `PKT_BLACKBOX_WORD, 8'h02},
                                          "blackbox frame header advertises v2 MAG metadata contract");
                    1:  expect_frame_word(idx, {16'd1, 8'd29, 8'd0},
                                          "blackbox frame length is 29 words");
                    26: expect_frame_word(idx, expected_mag_meta,
                                          "blackbox frame carries compact MAG1 metadata map");
                    27: expect_frame_word(idx, {16'd4, 16'd4},
                                          "blackbox frame carries MAG norm-delta and residual evidence");
                    28: expect_frame_word(idx, {16'h1234, 16'd0},
                                          "blackbox frame carries MAG1 bridge checksum");
                    default: begin
                        expect(log_stream_valid == 1'b1,
                               "blackbox stream is valid while frame is emitted");
                        expect(log_stream_last == 1'b0,
                               "blackbox frame last flag is low before final word");
                    end
                endcase

                tick();
            end

            expect(log_stream_valid == 1'b0, "blackbox stream deasserts after final word");
            expect(ext_log_seq == 16'd1, "blackbox log sequence increments after emitted frame");
            expect(ext_log_drop_count == 16'd0, "blackbox log emits without drops");
            log_runtime_enable = 1'b0;
        end
    endtask

    task clear_inputs;
        begin
            bmp_t_us = 32'd0;
            bmp_seq = 16'd0;
            bmp_valid = 1'b0;
            bmp_status = `ST_NOT_INITIALIZED;
            bmp_payload = 48'd0;
            bmp_age_ms = 16'hFFFF;

            acc_t_us = 32'd0;
            acc_seq = 16'd0;
            acc_valid = 1'b0;
            acc_status = `ST_NOT_INITIALIZED;
            acc_payload = 48'd0;
            acc_age_ms = 16'hFFFF;

            mag_t_us = 32'd0;
            mag_seq = 16'd0;
            mag_valid = 1'b0;
            mag_status = `ST_NOT_INITIALIZED;
            mag_payload = 48'd0;
            mag_age_ms = 16'hFFFF;

            pwr_t_us = 32'd0;
            pwr_seq = 16'd0;
            pwr_valid = 1'b0;
            pwr_status = `ST_NOT_INITIALIZED;
            pwr_payload = 48'd0;
            pwr_age_ms = 16'hFFFF;

            mag1_t_us = 32'd0;
            mag1_seq = 16'd0;
            mag1_valid = 1'b0;
            mag1_status = `ST_NOT_INITIALIZED;
            mag1_payload = 48'd0;
            mag1_age_ms = 16'hFFFF;
            mag1_cal_state = 8'd0;
            mag1_source_flags = 8'd0;
            mag1_bridge_checksum = 16'd0;

            rng_t_us = 32'd0;
            rng_seq = 16'd0;
            rng_valid = 1'b0;
            rng_status = `ST_NOT_INITIALIZED;
            rng_payload = 48'd0;
            rng_age_ms = 16'hFFFF;

            air_t_us = 32'd0;
            air_seq = 16'd0;
            air_valid = 1'b0;
            air_status = `ST_NOT_INITIALIZED;
            air_payload = 48'd0;
            air_age_ms = 16'hFFFF;

            env_t_us = 32'd0;
            env_seq = 16'd0;
            env_valid = 1'b0;
            env_status = `ST_NOT_INITIALIZED;
            env_payload = 48'd0;
            env_age_ms = 16'hFFFF;

            sun_t_us = 32'd0;
            sun_seq = 16'd0;
            sun_valid = 1'b0;
            sun_status = `ST_NOT_INITIALIZED;
            sun_payload = 48'd0;
            sun_age_ms = 16'hFFFF;

            flow_t_us = 32'd0;
            flow_seq = 16'd0;
            flow_valid = 1'b0;
            flow_status = `ST_NOT_INITIALIZED;
            flow_payload = 48'd0;
            flow_age_ms = 16'hFFFF;

            diag_selftest_enable = 1'b0;
            diag_fault_inject_enable = 1'b0;
            diag_fault_mode = 4'd0;
            log_runtime_enable = 1'b0;
            log_emit_req = 1'b0;
            log_stream_ready = 1'b1;
        end
    endtask

    initial begin
        rst = 1'b1;
        clear_inputs();
        repeat (4) tick();

        rst = 1'b0;
        repeat (2) tick();
        expect(ext_valid == 1'b0, "idle extension hub remains invalid");
        expect(ext_status == `ST_NOT_INITIALIZED, "idle extension hub reports not initialized");
        expect(ext_present_flags == 16'd0, "idle extension hub has no present flags");
        expect(ext_max_age_ms == 16'hFFFF, "idle extension hub age is stale sentinel");

        diag_selftest_enable = 1'b1;
        tick();
        expect(ext_valid == 1'b1, "diagnostic self-test publishes extension evidence");
        expect(ext_status == `ST_OK, "diagnostic self-test reports OK without injected faults");
        expect(ext_present_flags[`EXT_PRESENT_DIAG_BIT] == 1'b1, "diagnostic self-test sets diagnostic present bit");
        expect(ext_present_flags[`EXT_PRESENT_RANGE_BIT] == 1'b1, "diagnostic self-test exposes range placeholder");
        expect(ext_present_flags[`EXT_PRESENT_AIR_BIT] == 1'b1, "diagnostic self-test exposes air placeholder");
        expect(ext_present_flags[`EXT_PRESENT_ENV_BIT] == 1'b1, "diagnostic self-test exposes environment placeholder");
        expect(ext_present_flags[`EXT_PRESENT_SUN_BIT] == 1'b1, "diagnostic self-test exposes sun placeholder");
        expect(ext_present_flags[`EXT_PRESENT_FLOW_BIT] == 1'b1, "diagnostic self-test exposes flow placeholder");
        expect(ext_fault_flags == 16'd0, "diagnostic self-test is not a fault by itself");
        expect(ext_mag_source_flags[`EXT_SRC_SYNTHETIC_BIT] == 1'b1, "diagnostic self-test is marked synthetic");
        expect(ext_mag_cal_state == 8'hC1, "diagnostic self-test publishes deterministic calibration state");
        expect(ext_mag_bridge_checksum == 16'hCA1B, "diagnostic self-test publishes deterministic checksum");
        expect(ext_rng_height_cm == 16'd185, "diagnostic self-test range value is deterministic");
        expect(ext_air_speed_cms == 16'd1250, "diagnostic self-test airspeed value is deterministic");
        expect(ext_env_temp_cdeg == 16'd2345, "diagnostic self-test temperature value is deterministic");
        expect(ext_flow_dy == 16'hFFF8, "diagnostic self-test flow dy value is deterministic");
        expect(ext_max_age_ms == 16'd44, "diagnostic self-test age is deterministic");

        diag_selftest_enable = 1'b0;
        diag_fault_inject_enable = 1'b1;
        diag_fault_mode = 4'd0;
        tick();
        expect(ext_valid == 1'b1, "fault injection publishes diagnostic evidence");
        expect(ext_present_flags[`EXT_PRESENT_DIAG_BIT] == 1'b1, "fault injection sets diagnostic present bit");
        expect(ext_fault_flags[`EXT_FLG_DIAG_FAULT_INJECT_BIT] == 1'b1, "fault injection sets explicit injection flag");
        expect(ext_fault_flags[`EXT_FLG_RAW_STATUS_ERR_BIT] == 1'b1, "default fault injection sets raw-status fault flag");
        expect(ext_status == `ST_CONFIG_ERROR, "default fault injection reports config error");

        diag_selftest_enable = 1'b1;
        diag_fault_inject_enable = 1'b1;
        diag_fault_mode = 4'b0010;
        tick();
        expect(ext_status == `ST_STALE_REJECT, "stale fault-injection mode has stale status priority");
        expect(ext_fault_flags[`EXT_FLG_RANGE_STALE_BIT] == 1'b1, "stale fault-injection mode sets range stale");
        expect(ext_fault_flags[`EXT_FLG_AIR_STALE_BIT] == 1'b1, "stale fault-injection mode sets air stale");

        diag_fault_mode = 4'b0100;
        tick();
        expect(ext_status == `ST_PLAUSIBILITY_REJECT, "plausibility fault-injection mode has plausibility status priority");
        expect(ext_mag_disagreement == 1'b1, "plausibility fault-injection mode sets MAG disagreement evidence");

        clear_inputs();
        mag_t_us = 32'd1000;
        mag_seq = 16'h0033;
        mag_valid = 1'b1;
        mag_status = `ST_OK;
        mag_payload = {16'd300, 16'd200, 16'd100};
        mag_age_ms = 16'd6;
        mag1_t_us = 32'd1004;
        mag1_seq = 16'h0033;
        mag1_valid = 1'b1;
        mag1_status = `ST_OK;
        mag1_payload = {16'd302, 16'd201, 16'd101};
        mag1_age_ms = 16'd7;
        mag1_cal_state = 8'h21;
        mag1_source_flags = (8'd1 << `EXT_SRC_REAL_BIT);
        mag1_bridge_checksum = 16'h1234;
        pwr_valid = 1'b1;
        pwr_status = `ST_OK;
        pwr_age_ms = 16'd20;
        tick();
        expect(ext_valid == 1'b1, "fresh MAG pair publishes extension evidence");
        expect(ext_status == `ST_OK, "fresh MAG pair reports OK");
        expect(ext_present_flags[`EXT_PRESENT_MAG0_BIT] == 1'b1, "fresh MAG pair marks MAG0 present");
        expect(ext_present_flags[`EXT_PRESENT_MAG1_BIT] == 1'b1, "fresh MAG pair marks MAG1 present");
        expect(ext_present_flags[`EXT_PRESENT_DIAG_BIT] == 1'b0, "normal live evidence does not mark diagnostic present");
        expect(ext_mag_delta_l1 == 16'd4, "fresh MAG pair delta is computed in raw L1 units");
        expect(ext_mag_norm_primary == 16'd600, "fresh MAG0 norm is computed");
        expect(ext_mag_norm_secondary == 16'd604, "fresh MAG1 norm is computed");
        expect(ext_mag_sequence_aligned == 1'b1, "fresh MAG pair sequence alignment is reported");
        expect(ext_mag_sector_delta == 4'd1, "fresh MAG pair sector delta is reported");
        expect(ext_mag_source_flags == (8'd1 << `EXT_SRC_REAL_BIT), "fresh MAG pair preserves MAG1 source flags");
        expect(ext_mag_bridge_checksum == 16'h1234, "fresh MAG pair preserves MAG1 checksum");
        expect(ext_max_age_ms == 16'd20, "PMON age contributes to max extension age without claiming an extension-present bit");
        expect_live_mag_blackbox_frame();

        $display("PASS: tb_sensor_extension_hub");
        $finish;
    end
endmodule

`default_nettype wire
