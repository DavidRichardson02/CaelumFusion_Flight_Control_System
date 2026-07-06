`timescale 1ns/1ps
`default_nettype none

`include "telemetry_defs_vh.vh"

//==============================================================================
// tb_rocket_i2c_suite_mag1_physical
//------------------------------------------------------------------------------
// Focused physical-MAG1 bench for rocket_i2c_suite_top.
//
// Proves the deliberate LIS2MDL/MAG1 path, gated by cfg_ext_i2c_en, can:
//   - run the real shared-I2C engine against a register-aware LIS2MDL model
//   - read WHO_AM_I = 0x40 from register 0x4F
//   - write CFG_REG_A = 0x80 and CFG_REG_C = 0x01 during initialization
//   - publish a real-source MAG1 snapshot from OUTX/Y/Z registers
//   - keep derived heading behavior independent of MAG1 evidence
//==============================================================================
module tb_rocket_i2c_suite_mag1_physical;
    localparam integer CLK_HZ        = 2_000_000;
    localparam integer CLK_PERIOD_NS = 500;
    localparam integer WAIT_LIMIT    = 120_000;

    reg clk;
    reg rst;
    reg cfg_ext_i2c_en;

    tri1 scl;
    tri1 sda;

    pullup u_pullup_scl (scl);
    pullup u_pullup_sda (sda);

    wire adxl362_cs_n;
    wire adxl362_mosi;
    wire adxl362_sclk;

    wire [31:0] bmp_t_us;
    wire [15:0] bmp_seq;
    wire        bmp_valid;
    wire [7:0]  bmp_status;
    wire [47:0] bmp_payload;

    wire [31:0] acc_t_us;
    wire [15:0] acc_seq;
    wire        acc_valid;
    wire [7:0]  acc_status;
    wire [47:0] acc_payload;

    wire [31:0] mag_t_us;
    wire [15:0] mag_seq;
    wire        mag_valid;
    wire [7:0]  mag_status;
    wire [47:0] mag_payload;

    wire [31:0] pwr_t_us;
    wire [15:0] pwr_seq;
    wire        pwr_valid;
    wire [7:0]  pwr_status;
    wire [47:0] pwr_payload;
    wire [15:0] pwr_age_ms;

    wire [31:0] env_t_us;
    wire [15:0] env_seq;
    wire        env_valid;
    wire [7:0]  env_status;
    wire [47:0] env_payload;
    wire [15:0] env_age_ms;

    wire [31:0] mag1_t_us;
    wire [15:0] mag1_seq;
    wire        mag1_valid;
    wire [7:0]  mag1_status;
    wire [47:0] mag1_payload;
    wire [15:0] mag1_age_ms;
    wire [7:0]  mag1_cal_state;
    wire [7:0]  mag1_source_flags;
    wire [15:0] mag1_bridge_checksum;

    wire [31:0] gyro_t_us;
    wire [15:0] gyro_seq;
    wire        gyro_valid;
    wire [7:0]  gyro_status;
    wire [47:0] gyro_payload;
    wire [15:0] gyro_age_ms;

    wire [31:0] der_t_us;
    wire [15:0] der_seq;
    wire [7:0]  der_source_id;
    wire [7:0]  der_status;
    wire        der_valid;
    wire        der_alt_fresh;
    wire        der_vspd_fresh;
    wire        der_roll_fresh;
    wire        der_head_fresh;
    wire [15:0] der_bmp_seq_ref;
    wire [15:0] der_acc_seq_ref;
    wire [15:0] der_mag_seq_ref;
    wire [15:0] der_bmp_age_ms;
    wire [15:0] der_acc_age_ms;
    wire [15:0] der_mag_age_ms;
    wire        der_bmp_valid_ref;
    wire        der_acc_valid_ref;
    wire        der_mag_valid_ref;
    wire [31:0] der_altitude_cm;
    wire [31:0] der_vertical_speed_cms;
    wire [31:0] der_roll_mdeg;
    wire [31:0] der_heading_mdeg;

    wire [15:0] der_i2c_nack_count;
    wire [15:0] der_i2c_timeout_count;
    wire [15:0] der_txn_rate_hz;
    wire [31:0] der_cdc_update_count;
    wire [31:0] der_frame_count;
    wire [31:0] der_build_id;
    wire [15:0] der_schema_word;

    integer errors;
    reg [7:0] prev_j6_w_data;
    reg       prev_j6_w_seen;
    reg       saw_who_reg_select;
    reg       saw_cfg_a_write;
    reg       saw_cfg_c_write;
    reg       saw_data_reg_select;

    rocket_i2c_suite_top #(
        .CLK_HZ                 (CLK_HZ),
        .RATE_100HZ_US          (1000),
        .RATE_50HZ_US           (20000),
        .RATE_10HZ_US           (1000),
        .BUILD_ID               (32'hCAFE_1E01),
        .SCHEMA_WORD            (16'hCF1E),
        .USE_LIS3DH_I2C_ACC     (0),
        .USE_ADXL362_SPI_ACC    (0),
        .USE_CMPS2_MMC3416_MAG  (0),
        .USE_PMON1_PWR          (0),
        .USE_HYGRO_ENV          (0),
        .USE_GYRO_I2C           (0),
        .USE_LIS2MDL_MAG1       (1)
    ) dut (
        .clk                    (clk),
        .rst                    (rst),
        .cfg_lis3dh_i2c_acc_en  (1'b0),
        .cfg_adxl362_spi_acc_en (1'b0),
        .cfg_cmps2_mmc3416_mag_en(1'b0),
        .cfg_pmon1_pwr_en       (1'b0),
        .cfg_ext_i2c_en         (cfg_ext_i2c_en),
        .scl                    (scl),
        .sda                    (sda),
        .adxl362_cs_n           (adxl362_cs_n),
        .adxl362_mosi           (adxl362_mosi),
        .adxl362_miso           (1'b0),
        .adxl362_sclk           (adxl362_sclk),
        .adxl362_int1           (1'b0),
        .adxl362_int2           (1'b0),
        .bmp_t_us               (bmp_t_us),
        .bmp_seq                (bmp_seq),
        .bmp_valid              (bmp_valid),
        .bmp_status             (bmp_status),
        .bmp_payload            (bmp_payload),
        .acc_t_us               (acc_t_us),
        .acc_seq                (acc_seq),
        .acc_valid              (acc_valid),
        .acc_status             (acc_status),
        .acc_payload            (acc_payload),
        .mag_t_us               (mag_t_us),
        .mag_seq                (mag_seq),
        .mag_valid              (mag_valid),
        .mag_status             (mag_status),
        .mag_payload            (mag_payload),
        .pwr_t_us               (pwr_t_us),
        .pwr_seq                (pwr_seq),
        .pwr_valid              (pwr_valid),
        .pwr_status             (pwr_status),
        .pwr_payload            (pwr_payload),
        .pwr_age_ms             (pwr_age_ms),
        .env_t_us               (env_t_us),
        .env_seq                (env_seq),
        .env_valid              (env_valid),
        .env_status             (env_status),
        .env_payload            (env_payload),
        .env_age_ms             (env_age_ms),
        .mag1_t_us              (mag1_t_us),
        .mag1_seq               (mag1_seq),
        .mag1_valid             (mag1_valid),
        .mag1_status            (mag1_status),
        .mag1_payload           (mag1_payload),
        .mag1_age_ms            (mag1_age_ms),
        .mag1_cal_state         (mag1_cal_state),
        .mag1_source_flags      (mag1_source_flags),
        .mag1_bridge_checksum   (mag1_bridge_checksum),
        .gyro_t_us              (gyro_t_us),
        .gyro_seq               (gyro_seq),
        .gyro_valid             (gyro_valid),
        .gyro_status            (gyro_status),
        .gyro_payload           (gyro_payload),
        .gyro_age_ms            (gyro_age_ms),
        .der_t_us               (der_t_us),
        .der_seq                (der_seq),
        .der_source_id          (der_source_id),
        .der_status             (der_status),
        .der_valid              (der_valid),
        .der_alt_fresh          (der_alt_fresh),
        .der_vspd_fresh         (der_vspd_fresh),
        .der_roll_fresh         (der_roll_fresh),
        .der_head_fresh         (der_head_fresh),
        .der_bmp_seq_ref        (der_bmp_seq_ref),
        .der_acc_seq_ref        (der_acc_seq_ref),
        .der_mag_seq_ref        (der_mag_seq_ref),
        .der_bmp_age_ms         (der_bmp_age_ms),
        .der_acc_age_ms         (der_acc_age_ms),
        .der_mag_age_ms         (der_mag_age_ms),
        .der_bmp_valid_ref      (der_bmp_valid_ref),
        .der_acc_valid_ref      (der_acc_valid_ref),
        .der_mag_valid_ref      (der_mag_valid_ref),
        .der_altitude_cm        (der_altitude_cm),
        .der_vertical_speed_cms (der_vertical_speed_cms),
        .der_roll_mdeg          (der_roll_mdeg),
        .der_heading_mdeg       (der_heading_mdeg),
        .der_i2c_nack_count     (der_i2c_nack_count),
        .der_i2c_timeout_count  (der_i2c_timeout_count),
        .der_txn_rate_hz        (der_txn_rate_hz),
        .der_cdc_update_count   (der_cdc_update_count),
        .der_frame_count        (der_frame_count),
        .der_build_id           (der_build_id),
        .der_schema_word        (der_schema_word)
    );

    lis2mdl_physical_i2c_slave_model u_mag1_model (
        .rst (rst),
        .scl (scl),
        .sda (sda)
    );

    initial begin
        clk = 1'b0;
        forever #(CLK_PERIOD_NS/2) clk = ~clk;
    end

    always @(posedge clk) begin
        if (rst) begin
            prev_j6_w_data     <= 8'd0;
            prev_j6_w_seen     <= 1'b0;
            saw_who_reg_select <= 1'b0;
            saw_cfg_a_write    <= 1'b0;
            saw_cfg_c_write    <= 1'b0;
            saw_data_reg_select<= 1'b0;
        end else if (dut.j6_w_valid && dut.j6_w_ready) begin
            if (dut.j6_w_data == 8'h4F)
                saw_who_reg_select <= 1'b1;
            if (prev_j6_w_seen && (prev_j6_w_data == 8'h60) &&
                (dut.j6_w_data == 8'h80))
                saw_cfg_a_write <= 1'b1;
            if (prev_j6_w_seen && (prev_j6_w_data == 8'h62) &&
                (dut.j6_w_data == 8'h01))
                saw_cfg_c_write <= 1'b1;
            if (dut.j6_w_data == 8'h68)
                saw_data_reg_select <= 1'b1;

            prev_j6_w_data <= dut.j6_w_data;
            prev_j6_w_seen <= 1'b1;
        end
    end

    task expect_true;
        input condition;
        input [8*160-1:0] message;
        begin
            if (!condition) begin
                $display("FAIL: %0s", message);
                errors = errors + 1;
            end
        end
    endtask

    task wait_for_mag1_valid;
        integer guard;
        begin
            guard = 0;
            while (!mag1_valid && (guard < WAIT_LIMIT)) begin
                @(posedge clk);
                guard = guard + 1;
            end
            expect_true(guard < WAIT_LIMIT, "physical LIS2MDL/MAG1 produced a valid snapshot");
        end
    endtask

    initial begin
        errors = 0;
        rst = 1'b1;
        cfg_ext_i2c_en = 1'b0;
        repeat (20) @(posedge clk);

        rst = 1'b0;
        repeat (40) @(posedge clk);
        expect_true(!mag1_valid, "SW15 extension gate low leaves MAG1 invalid");
        expect_true(mag1_status == `ST_NOT_INITIALIZED,
                    "SW15 extension gate low leaves MAG1 not initialized");
        expect_true(mag1_age_ms == 16'hFFFF,
                    "SW15 extension gate low leaves MAG1 age at stale sentinel");

        cfg_ext_i2c_en = 1'b1;
        wait_for_mag1_valid();

        expect_true(mag1_status == `ST_OK, "physical LIS2MDL/MAG1 status is OK");
        expect_true(mag1_payload == {16'h6655, 16'h4433, 16'h2211},
                    "physical LIS2MDL/MAG1 payload preserves Z/Y/X register order");
        expect_true(mag1_seq == 16'd1, "physical LIS2MDL/MAG1 first committed sequence is 1");
        expect_true(mag1_age_ms == 16'd0, "physical LIS2MDL/MAG1 age resets on commit");
        expect_true(mag1_cal_state == 8'h01, "physical LIS2MDL/MAG1 calibration state marks live path");
        expect_true(mag1_source_flags == (8'd1 << `EXT_SRC_REAL_BIT),
                    "physical LIS2MDL/MAG1 is marked real, not synthetic");
        expect_true(!mag1_source_flags[`EXT_SRC_SYNTHETIC_BIT],
                    "physical LIS2MDL/MAG1 does not set synthetic provenance");
        expect_true(mag1_bridge_checksum == (16'h2211 ^ 16'h4433 ^ 16'h6655 ^
                                             mag1_seq ^ {mag1_status, mag1_source_flags}),
                    "physical LIS2MDL/MAG1 checksum is derived from live snapshot");
        expect_true(saw_who_reg_select, "physical LIS2MDL/MAG1 selected WHO_AM_I register");
        expect_true(saw_cfg_a_write, "physical LIS2MDL/MAG1 initialization wrote CFG_REG_A");
        expect_true(saw_cfg_c_write, "physical LIS2MDL/MAG1 initialization wrote CFG_REG_C");
        expect_true(saw_data_reg_select, "physical LIS2MDL/MAG1 selected OUTX_L data register");
        expect_true(der_status == `ST_MISSING_INPUT,
                    "MAG1 evidence does not change flight-facing derived heading status");
        expect_true(der_mag_valid_ref == 1'b0,
                    "MAG1 evidence is not fused into the MAG0 heading reference");

        if (errors == 0) begin
            $display("PASS: tb_rocket_i2c_suite_mag1_physical");
            $finish;
        end else begin
            $display("FAIL: tb_rocket_i2c_suite_mag1_physical errors=%0d", errors);
            $finish;
        end
    end
endmodule

//==============================================================================
// lis2mdl_physical_i2c_slave_model
//------------------------------------------------------------------------------
// Minimal register-aware open-drain LIS2MDL model for the physical MAG1 bench.
//==============================================================================
module lis2mdl_physical_i2c_slave_model (
    input  wire rst,
    inout  wire scl,
    inout  wire sda
);
    localparam [6:0] DEV_ADDR7 = 7'h1E;

    reg        sda_oe_r;
    assign sda = sda_oe_r ? 1'b0 : 1'bz;

    reg [7:0] mem [0:255];
    reg [7:0] shreg_r;
    reg [2:0] bit_ctr_r;
    reg [7:0] reg_ptr_r;
    reg       selected_r;
    reg       rw_r;
    reg       first_write_byte_r;
    reg [7:0] tx_byte_r;
    reg [3:0] state_r;
    reg       ignore_ack_release_stop;
    reg       rd_release_pending_r;

    reg       cfg_a_written;
    reg       cfg_c_written;
    reg [7:0] cfg_a_value;
    reg [7:0] cfg_c_value;

    localparam [3:0]
        S_IDLE          = 4'd0,
        S_ADDR          = 4'd1,
        S_ADDR_GOT      = 4'd2,
        S_ADDR_ACK_HOLD = 4'd3,
        S_WR            = 4'd4,
        S_WR_GOT        = 4'd5,
        S_WR_ACK_HOLD   = 4'd6,
        S_RD            = 4'd7,
        S_RD_MASTER_ACK = 4'd8;

    integer i;

    initial begin
        for (i = 0; i < 256; i = i + 1)
            mem[i] = 8'h00;

        mem[8'h4F] = 8'h40; // WHO_AM_I
        mem[8'h68] = 8'h11; // OUTX_L
        mem[8'h69] = 8'h22; // OUTX_H
        mem[8'h6A] = 8'h33; // OUTY_L
        mem[8'h6B] = 8'h44; // OUTY_H
        mem[8'h6C] = 8'h55; // OUTZ_L
        mem[8'h6D] = 8'h66; // OUTZ_H

        state_r            = S_IDLE;
        sda_oe_r           = 1'b0;
        shreg_r            = 8'd0;
        bit_ctr_r          = 3'd7;
        reg_ptr_r          = 8'd0;
        selected_r         = 1'b0;
        rw_r               = 1'b0;
        first_write_byte_r = 1'b1;
        tx_byte_r          = 8'd0;
        ignore_ack_release_stop = 1'b0;
        rd_release_pending_r = 1'b0;
        cfg_a_written      = 1'b0;
        cfg_c_written      = 1'b0;
        cfg_a_value        = 8'd0;
        cfg_c_value        = 8'd0;
    end

    always @(posedge rst) begin
        state_r            <= S_IDLE;
        sda_oe_r           <= 1'b0;
        shreg_r            <= 8'd0;
        bit_ctr_r          <= 3'd7;
        reg_ptr_r          <= 8'd0;
        selected_r         <= 1'b0;
        rw_r               <= 1'b0;
        first_write_byte_r <= 1'b1;
        tx_byte_r          <= 8'd0;
        ignore_ack_release_stop <= 1'b0;
        rd_release_pending_r <= 1'b0;
        cfg_a_written      <= 1'b0;
        cfg_c_written      <= 1'b0;
        cfg_a_value        <= 8'd0;
        cfg_c_value        <= 8'd0;
    end

    always @(negedge sda) begin
        if (!rst && (scl === 1'b1)) begin
            state_r            <= S_ADDR;
            sda_oe_r           <= 1'b0;
            bit_ctr_r          <= 3'd7;
            selected_r         <= 1'b0;
            first_write_byte_r <= 1'b1;
            rd_release_pending_r <= 1'b0;
        end
    end

    always @(posedge sda) begin
        if (!rst && (scl === 1'b1)) begin
            if (ignore_ack_release_stop) begin
                ignore_ack_release_stop = 1'b0;
            end else begin
                state_r    <= S_IDLE;
                sda_oe_r   <= 1'b0;
                selected_r <= 1'b0;
            end
        end
    end

    always @(posedge scl) begin
        if (!rst) begin
            case (state_r)
                S_ADDR: begin
                    shreg_r[bit_ctr_r] <= sda;
                    if (bit_ctr_r == 3'd0)
                        state_r <= S_ADDR_GOT;
                    else
                        bit_ctr_r <= bit_ctr_r - 3'd1;
                end

                S_WR: begin
                    shreg_r[bit_ctr_r] <= sda;
                    if (bit_ctr_r == 3'd0)
                        state_r <= S_WR_GOT;
                    else
                        bit_ctr_r <= bit_ctr_r - 3'd1;
                end

                S_RD_MASTER_ACK: begin
                    if (sda == 1'b0) begin
                        tx_byte_r <= mem[reg_ptr_r];
                        bit_ctr_r <= 3'd7;
                        state_r   <= S_RD;
                        rd_release_pending_r <= 1'b0;
                    end else begin
                        state_r    <= S_IDLE;
                        selected_r <= 1'b0;
                        rd_release_pending_r <= 1'b0;
                    end
                end

                default: begin end
            endcase
        end
    end

    always @(negedge scl) begin
        if (!rst) begin
            case (state_r)
                S_ADDR_GOT: begin
                    if (shreg_r[7:1] == DEV_ADDR7) begin
                        selected_r <= 1'b1;
                        rw_r       <= shreg_r[0];
                        sda_oe_r   <= 1'b1;
                        ignore_ack_release_stop = 1'b1;
                        state_r    <= S_ADDR_ACK_HOLD;
                    end else begin
                        selected_r <= 1'b0;
                        sda_oe_r   <= 1'b0;
                        state_r    <= S_IDLE;
                    end
                end

                S_ADDR_ACK_HOLD: begin
                    if (selected_r) begin
                        if (rw_r) begin
                            tx_byte_r <= mem[reg_ptr_r];
                            bit_ctr_r <= 3'd6;
                            sda_oe_r  <= ~mem[reg_ptr_r][7];
                            state_r   <= S_RD;
                            rd_release_pending_r <= 1'b0;
                        end else begin
                            sda_oe_r  <= 1'b0;
                            bit_ctr_r <= 3'd7;
                            state_r   <= S_WR;
                        end
                    end else begin
                        sda_oe_r <= 1'b0;
                        state_r  <= S_IDLE;
                    end
                end

                S_WR_GOT: begin
                    if (first_write_byte_r) begin
                        reg_ptr_r          <= shreg_r;
                        first_write_byte_r <= 1'b0;
                    end else begin
                        mem[reg_ptr_r] <= shreg_r;
                        if (reg_ptr_r == 8'h60) begin
                            cfg_a_written <= 1'b1;
                            cfg_a_value   <= shreg_r;
                        end
                        if (reg_ptr_r == 8'h62) begin
                            cfg_c_written <= 1'b1;
                            cfg_c_value   <= shreg_r;
                        end
                        reg_ptr_r <= reg_ptr_r + 8'd1;
                    end
                    sda_oe_r <= 1'b1;
                    ignore_ack_release_stop = 1'b1;
                    state_r  <= S_WR_ACK_HOLD;
                end

                S_WR_ACK_HOLD: begin
                    sda_oe_r  <= 1'b0;
                    bit_ctr_r <= 3'd7;
                    state_r   <= S_WR;
                end

                S_RD: begin
                    if (rd_release_pending_r) begin
                        sda_oe_r  <= 1'b0;
                        reg_ptr_r <= reg_ptr_r + 8'd1;
                        state_r   <= S_RD_MASTER_ACK;
                        rd_release_pending_r <= 1'b0;
                    end else begin
                        sda_oe_r <= ~tx_byte_r[bit_ctr_r];
                        if (bit_ctr_r == 3'd0)
                            rd_release_pending_r <= 1'b1;
                        else
                            bit_ctr_r <= bit_ctr_r - 3'd1;
                    end
                end

                default: begin end
            endcase
        end
    end
endmodule

`default_nettype wire
