`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// tb_rocket_i2c_suite_pmon1
//------------------------------------------------------------------------------
// Real-engine PMON1 integration check for rocket_i2c_suite_top.
//
// Two suite instances run in parallel:
//   - ACK bus: BMP585 model + PMON1 model at 7'h38
//   - NACK bus: BMP585 model only; PMON1 address is intentionally missing
//
// Proves:
//   - cfg_pmon1_pwr_en low leaves pwr_* not initialized/stale and does not
//     address PMON1
//   - cfg_pmon1_pwr_en high publishes voltage/current/status payload from 7'h38
//   - missing PMON1 commits invalid, non-OK evidence and increments NACK health
//==============================================================================
module tb_rocket_i2c_suite_pmon1;
    localparam integer CLK_HZ        = 2_000_000;
    localparam integer CLK_PERIOD_NS = 500;
    localparam integer WAIT_LIMIT    = 50_000;

    reg clk;
    reg rst;

    reg cfg_pmon_ack;
    reg cfg_pmon_nack;

    tri1 scl_ack;
    tri1 sda_ack;
    tri1 scl_nack;
    tri1 sda_nack;

    pullup u_pullup_scl_ack  (scl_ack);
    pullup u_pullup_sda_ack  (sda_ack);
    pullup u_pullup_scl_nack (scl_nack);
    pullup u_pullup_sda_nack (sda_nack);

    wire adxl_cs_ack;
    wire adxl_mosi_ack;
    wire adxl_sclk_ack;
    wire adxl_cs_nack;
    wire adxl_mosi_nack;
    wire adxl_sclk_nack;

    wire [31:0] bmp_t_us_ack;
    wire [15:0] bmp_seq_ack;
    wire        bmp_valid_ack;
    wire [7:0]  bmp_status_ack;
    wire [47:0] bmp_payload_ack;
    wire [31:0] acc_t_us_ack;
    wire [15:0] acc_seq_ack;
    wire        acc_valid_ack;
    wire [7:0]  acc_status_ack;
    wire [47:0] acc_payload_ack;
    wire [31:0] mag_t_us_ack;
    wire [15:0] mag_seq_ack;
    wire        mag_valid_ack;
    wire [7:0]  mag_status_ack;
    wire [47:0] mag_payload_ack;
    wire [31:0] pwr_t_us_ack;
    wire [15:0] pwr_seq_ack;
    wire        pwr_valid_ack;
    wire [7:0]  pwr_status_ack;
    wire [47:0] pwr_payload_ack;
    wire [15:0] pwr_age_ms_ack;
    wire [31:0] env_t_us_ack;
    wire [15:0] env_seq_ack;
    wire        env_valid_ack;
    wire [7:0]  env_status_ack;
    wire [47:0] env_payload_ack;
    wire [15:0] env_age_ms_ack;
    wire [31:0] mag1_t_us_ack;
    wire [15:0] mag1_seq_ack;
    wire        mag1_valid_ack;
    wire [7:0]  mag1_status_ack;
    wire [47:0] mag1_payload_ack;
    wire [15:0] mag1_age_ms_ack;
    wire [7:0]  mag1_cal_state_ack;
    wire [7:0]  mag1_source_flags_ack;
    wire [15:0] mag1_bridge_checksum_ack;
    wire [31:0] gyro_t_us_ack;
    wire [15:0] gyro_seq_ack;
    wire        gyro_valid_ack;
    wire [7:0]  gyro_status_ack;
    wire [47:0] gyro_payload_ack;
    wire [15:0] gyro_age_ms_ack;
    wire [31:0] der_t_us_ack;
    wire [15:0] der_seq_ack;
    wire [7:0]  der_source_id_ack;
    wire [7:0]  der_status_ack;
    wire        der_valid_ack;
    wire        der_alt_fresh_ack;
    wire        der_vspd_fresh_ack;
    wire        der_roll_fresh_ack;
    wire        der_head_fresh_ack;
    wire [15:0] der_bmp_seq_ref_ack;
    wire [15:0] der_acc_seq_ref_ack;
    wire [15:0] der_mag_seq_ref_ack;
    wire [15:0] der_bmp_age_ms_ack;
    wire [15:0] der_acc_age_ms_ack;
    wire [15:0] der_mag_age_ms_ack;
    wire        der_bmp_valid_ref_ack;
    wire        der_acc_valid_ref_ack;
    wire        der_mag_valid_ref_ack;
    wire [31:0] der_altitude_cm_ack;
    wire [31:0] der_vertical_speed_cms_ack;
    wire [31:0] der_roll_mdeg_ack;
    wire [31:0] der_heading_mdeg_ack;
    wire [15:0] der_i2c_nack_count_ack;
    wire [15:0] der_i2c_timeout_count_ack;
    wire [15:0] der_txn_rate_hz_ack;
    wire [31:0] der_cdc_update_count_ack;
    wire [31:0] der_frame_count_ack;
    wire [31:0] der_build_id_ack;
    wire [15:0] der_schema_word_ack;

    wire [31:0] bmp_t_us_nack;
    wire [15:0] bmp_seq_nack;
    wire        bmp_valid_nack;
    wire [7:0]  bmp_status_nack;
    wire [47:0] bmp_payload_nack;
    wire [31:0] acc_t_us_nack;
    wire [15:0] acc_seq_nack;
    wire        acc_valid_nack;
    wire [7:0]  acc_status_nack;
    wire [47:0] acc_payload_nack;
    wire [31:0] mag_t_us_nack;
    wire [15:0] mag_seq_nack;
    wire        mag_valid_nack;
    wire [7:0]  mag_status_nack;
    wire [47:0] mag_payload_nack;
    wire [31:0] pwr_t_us_nack;
    wire [15:0] pwr_seq_nack;
    wire        pwr_valid_nack;
    wire [7:0]  pwr_status_nack;
    wire [47:0] pwr_payload_nack;
    wire [15:0] pwr_age_ms_nack;
    wire [31:0] env_t_us_nack;
    wire [15:0] env_seq_nack;
    wire        env_valid_nack;
    wire [7:0]  env_status_nack;
    wire [47:0] env_payload_nack;
    wire [15:0] env_age_ms_nack;
    wire [31:0] mag1_t_us_nack;
    wire [15:0] mag1_seq_nack;
    wire        mag1_valid_nack;
    wire [7:0]  mag1_status_nack;
    wire [47:0] mag1_payload_nack;
    wire [15:0] mag1_age_ms_nack;
    wire [7:0]  mag1_cal_state_nack;
    wire [7:0]  mag1_source_flags_nack;
    wire [15:0] mag1_bridge_checksum_nack;
    wire [31:0] gyro_t_us_nack;
    wire [15:0] gyro_seq_nack;
    wire        gyro_valid_nack;
    wire [7:0]  gyro_status_nack;
    wire [47:0] gyro_payload_nack;
    wire [15:0] gyro_age_ms_nack;
    wire [31:0] der_t_us_nack;
    wire [15:0] der_seq_nack;
    wire [7:0]  der_source_id_nack;
    wire [7:0]  der_status_nack;
    wire        der_valid_nack;
    wire        der_alt_fresh_nack;
    wire        der_vspd_fresh_nack;
    wire        der_roll_fresh_nack;
    wire        der_head_fresh_nack;
    wire [15:0] der_bmp_seq_ref_nack;
    wire [15:0] der_acc_seq_ref_nack;
    wire [15:0] der_mag_seq_ref_nack;
    wire [15:0] der_bmp_age_ms_nack;
    wire [15:0] der_acc_age_ms_nack;
    wire [15:0] der_mag_age_ms_nack;
    wire        der_bmp_valid_ref_nack;
    wire        der_acc_valid_ref_nack;
    wire        der_mag_valid_ref_nack;
    wire [31:0] der_altitude_cm_nack;
    wire [31:0] der_vertical_speed_cms_nack;
    wire [31:0] der_roll_mdeg_nack;
    wire [31:0] der_heading_mdeg_nack;
    wire [15:0] der_i2c_nack_count_nack;
    wire [15:0] der_i2c_timeout_count_nack;
    wire [15:0] der_txn_rate_hz_nack;
    wire [31:0] der_cdc_update_count_nack;
    wire [31:0] der_frame_count_nack;
    wire [31:0] der_build_id_nack;
    wire [15:0] der_schema_word_nack;

    integer errors;

    rocket_i2c_suite_top #(
        .CLK_HZ                 (CLK_HZ),
        .RATE_100HZ_US          (1000),
        .RATE_50HZ_US           (20000),
        .RATE_10HZ_US           (1000),
        .BUILD_ID               (32'hCAFE_3801),
        .SCHEMA_WORD            (16'hCF38),
        .USE_LIS3DH_I2C_ACC     (0),
        .USE_ADXL362_SPI_ACC    (0),
        .USE_CMPS2_MMC3416_MAG  (0),
        .USE_PMON1_PWR          (1),
        .PMON1_ADDR7            (7'h38),
        .USE_HYGRO_ENV          (0),
        .USE_GYRO_I2C           (0),
        .USE_LIS2MDL_MAG1       (0)
    ) dut_ack (
        .clk                    (clk),
        .rst                    (rst),
        .cfg_lis3dh_i2c_acc_en  (1'b0),
        .cfg_adxl362_spi_acc_en (1'b0),
        .cfg_cmps2_mmc3416_mag_en(1'b0),
        .cfg_pmon1_pwr_en       (cfg_pmon_ack),
        .cfg_ext_i2c_en         (1'b0),
        .scl                    (scl_ack),
        .sda                    (sda_ack),
        .adxl362_cs_n           (adxl_cs_ack),
        .adxl362_mosi           (adxl_mosi_ack),
        .adxl362_miso           (1'b0),
        .adxl362_sclk           (adxl_sclk_ack),
        .adxl362_int1           (1'b0),
        .adxl362_int2           (1'b0),
        .bmp_t_us               (bmp_t_us_ack),
        .bmp_seq                (bmp_seq_ack),
        .bmp_valid              (bmp_valid_ack),
        .bmp_status             (bmp_status_ack),
        .bmp_payload            (bmp_payload_ack),
        .acc_t_us               (acc_t_us_ack),
        .acc_seq                (acc_seq_ack),
        .acc_valid              (acc_valid_ack),
        .acc_status             (acc_status_ack),
        .acc_payload            (acc_payload_ack),
        .mag_t_us               (mag_t_us_ack),
        .mag_seq                (mag_seq_ack),
        .mag_valid              (mag_valid_ack),
        .mag_status             (mag_status_ack),
        .mag_payload            (mag_payload_ack),
        .pwr_t_us               (pwr_t_us_ack),
        .pwr_seq                (pwr_seq_ack),
        .pwr_valid              (pwr_valid_ack),
        .pwr_status             (pwr_status_ack),
        .pwr_payload            (pwr_payload_ack),
        .pwr_age_ms             (pwr_age_ms_ack),
        .env_t_us               (env_t_us_ack),
        .env_seq                (env_seq_ack),
        .env_valid              (env_valid_ack),
        .env_status             (env_status_ack),
        .env_payload            (env_payload_ack),
        .env_age_ms             (env_age_ms_ack),
        .mag1_t_us              (mag1_t_us_ack),
        .mag1_seq               (mag1_seq_ack),
        .mag1_valid             (mag1_valid_ack),
        .mag1_status            (mag1_status_ack),
        .mag1_payload           (mag1_payload_ack),
        .mag1_age_ms            (mag1_age_ms_ack),
        .mag1_cal_state         (mag1_cal_state_ack),
        .mag1_source_flags      (mag1_source_flags_ack),
        .mag1_bridge_checksum   (mag1_bridge_checksum_ack),
        .gyro_t_us              (gyro_t_us_ack),
        .gyro_seq               (gyro_seq_ack),
        .gyro_valid             (gyro_valid_ack),
        .gyro_status            (gyro_status_ack),
        .gyro_payload           (gyro_payload_ack),
        .gyro_age_ms            (gyro_age_ms_ack),
        .der_t_us               (der_t_us_ack),
        .der_seq                (der_seq_ack),
        .der_source_id          (der_source_id_ack),
        .der_status             (der_status_ack),
        .der_valid              (der_valid_ack),
        .der_alt_fresh          (der_alt_fresh_ack),
        .der_vspd_fresh         (der_vspd_fresh_ack),
        .der_roll_fresh         (der_roll_fresh_ack),
        .der_head_fresh         (der_head_fresh_ack),
        .der_bmp_seq_ref        (der_bmp_seq_ref_ack),
        .der_acc_seq_ref        (der_acc_seq_ref_ack),
        .der_mag_seq_ref        (der_mag_seq_ref_ack),
        .der_bmp_age_ms         (der_bmp_age_ms_ack),
        .der_acc_age_ms         (der_acc_age_ms_ack),
        .der_mag_age_ms         (der_mag_age_ms_ack),
        .der_bmp_valid_ref      (der_bmp_valid_ref_ack),
        .der_acc_valid_ref      (der_acc_valid_ref_ack),
        .der_mag_valid_ref      (der_mag_valid_ref_ack),
        .der_altitude_cm        (der_altitude_cm_ack),
        .der_vertical_speed_cms (der_vertical_speed_cms_ack),
        .der_roll_mdeg          (der_roll_mdeg_ack),
        .der_heading_mdeg       (der_heading_mdeg_ack),
        .der_i2c_nack_count     (der_i2c_nack_count_ack),
        .der_i2c_timeout_count  (der_i2c_timeout_count_ack),
        .der_txn_rate_hz        (der_txn_rate_hz_ack),
        .der_cdc_update_count   (der_cdc_update_count_ack),
        .der_frame_count        (der_frame_count_ack),
        .der_build_id           (der_build_id_ack),
        .der_schema_word        (der_schema_word_ack)
    );

    rocket_i2c_suite_top #(
        .CLK_HZ                 (CLK_HZ),
        .RATE_100HZ_US          (1000),
        .RATE_50HZ_US           (20000),
        .RATE_10HZ_US           (1000),
        .BUILD_ID               (32'hCAFE_3802),
        .SCHEMA_WORD            (16'hCF38),
        .USE_LIS3DH_I2C_ACC     (0),
        .USE_ADXL362_SPI_ACC    (0),
        .USE_CMPS2_MMC3416_MAG  (0),
        .USE_PMON1_PWR          (1),
        .PMON1_ADDR7            (7'h38),
        .USE_HYGRO_ENV          (0),
        .USE_GYRO_I2C           (0),
        .USE_LIS2MDL_MAG1       (0)
    ) dut_nack (
        .clk                    (clk),
        .rst                    (rst),
        .cfg_lis3dh_i2c_acc_en  (1'b0),
        .cfg_adxl362_spi_acc_en (1'b0),
        .cfg_cmps2_mmc3416_mag_en(1'b0),
        .cfg_pmon1_pwr_en       (cfg_pmon_nack),
        .cfg_ext_i2c_en         (1'b0),
        .scl                    (scl_nack),
        .sda                    (sda_nack),
        .adxl362_cs_n           (adxl_cs_nack),
        .adxl362_mosi           (adxl_mosi_nack),
        .adxl362_miso           (1'b0),
        .adxl362_sclk           (adxl_sclk_nack),
        .adxl362_int1           (1'b0),
        .adxl362_int2           (1'b0),
        .bmp_t_us               (bmp_t_us_nack),
        .bmp_seq                (bmp_seq_nack),
        .bmp_valid              (bmp_valid_nack),
        .bmp_status             (bmp_status_nack),
        .bmp_payload            (bmp_payload_nack),
        .acc_t_us               (acc_t_us_nack),
        .acc_seq                (acc_seq_nack),
        .acc_valid              (acc_valid_nack),
        .acc_status             (acc_status_nack),
        .acc_payload            (acc_payload_nack),
        .mag_t_us               (mag_t_us_nack),
        .mag_seq                (mag_seq_nack),
        .mag_valid              (mag_valid_nack),
        .mag_status             (mag_status_nack),
        .mag_payload            (mag_payload_nack),
        .pwr_t_us               (pwr_t_us_nack),
        .pwr_seq                (pwr_seq_nack),
        .pwr_valid              (pwr_valid_nack),
        .pwr_status             (pwr_status_nack),
        .pwr_payload            (pwr_payload_nack),
        .pwr_age_ms             (pwr_age_ms_nack),
        .env_t_us               (env_t_us_nack),
        .env_seq                (env_seq_nack),
        .env_valid              (env_valid_nack),
        .env_status             (env_status_nack),
        .env_payload            (env_payload_nack),
        .env_age_ms             (env_age_ms_nack),
        .mag1_t_us              (mag1_t_us_nack),
        .mag1_seq               (mag1_seq_nack),
        .mag1_valid             (mag1_valid_nack),
        .mag1_status            (mag1_status_nack),
        .mag1_payload           (mag1_payload_nack),
        .mag1_age_ms            (mag1_age_ms_nack),
        .mag1_cal_state         (mag1_cal_state_nack),
        .mag1_source_flags      (mag1_source_flags_nack),
        .mag1_bridge_checksum   (mag1_bridge_checksum_nack),
        .gyro_t_us              (gyro_t_us_nack),
        .gyro_seq               (gyro_seq_nack),
        .gyro_valid             (gyro_valid_nack),
        .gyro_status            (gyro_status_nack),
        .gyro_payload           (gyro_payload_nack),
        .gyro_age_ms            (gyro_age_ms_nack),
        .der_t_us               (der_t_us_nack),
        .der_seq                (der_seq_nack),
        .der_source_id          (der_source_id_nack),
        .der_status             (der_status_nack),
        .der_valid              (der_valid_nack),
        .der_alt_fresh          (der_alt_fresh_nack),
        .der_vspd_fresh         (der_vspd_fresh_nack),
        .der_roll_fresh         (der_roll_fresh_nack),
        .der_head_fresh         (der_head_fresh_nack),
        .der_bmp_seq_ref        (der_bmp_seq_ref_nack),
        .der_acc_seq_ref        (der_acc_seq_ref_nack),
        .der_mag_seq_ref        (der_mag_seq_ref_nack),
        .der_bmp_age_ms         (der_bmp_age_ms_nack),
        .der_acc_age_ms         (der_acc_age_ms_nack),
        .der_mag_age_ms         (der_mag_age_ms_nack),
        .der_bmp_valid_ref      (der_bmp_valid_ref_nack),
        .der_acc_valid_ref      (der_acc_valid_ref_nack),
        .der_mag_valid_ref      (der_mag_valid_ref_nack),
        .der_altitude_cm        (der_altitude_cm_nack),
        .der_vertical_speed_cms (der_vertical_speed_cms_nack),
        .der_roll_mdeg          (der_roll_mdeg_nack),
        .der_heading_mdeg       (der_heading_mdeg_nack),
        .der_i2c_nack_count     (der_i2c_nack_count_nack),
        .der_i2c_timeout_count  (der_i2c_timeout_count_nack),
        .der_txn_rate_hz        (der_txn_rate_hz_nack),
        .der_cdc_update_count   (der_cdc_update_count_nack),
        .der_frame_count        (der_frame_count_nack),
        .der_build_id           (der_build_id_nack),
        .der_schema_word        (der_schema_word_nack)
    );

    i2c_simple_sensor_model #(
        .I2C_ADDR   (7'h47),
        .READ_BYTE0 (8'h51),
        .READ_BYTE1 (8'h00),
        .READ_BYTE2 (8'h00),
        .READ_BYTE3 (8'h11),
        .READ_BYTE4 (8'h22),
        .READ_BYTE5 (8'h33)
    ) u_bmp_ack (
        .scl (scl_ack),
        .sda (sda_ack)
    );

    i2c_simple_sensor_model #(
        .I2C_ADDR   (7'h38),
        .READ_BYTE0 (8'hA5),
        .READ_BYTE1 (8'h3C),
        .READ_BYTE2 (8'h9F),
        .READ_BYTE3 (8'h00),
        .READ_BYTE4 (8'h00),
        .READ_BYTE5 (8'h00),
        .ONE_WRITE_THEN_ADDR (1'b1)
    ) u_pmon_ack (
        .scl (scl_ack),
        .sda (sda_ack)
    );

    i2c_simple_sensor_model #(
        .I2C_ADDR   (7'h47),
        .READ_BYTE0 (8'h51),
        .READ_BYTE1 (8'h00),
        .READ_BYTE2 (8'h00),
        .READ_BYTE3 (8'h44),
        .READ_BYTE4 (8'h55),
        .READ_BYTE5 (8'h66)
    ) u_bmp_nack (
        .scl (scl_nack),
        .sda (sda_nack)
    );

    initial begin
        clk = 1'b0;
        forever #(CLK_PERIOD_NS/2) clk = ~clk;
    end

    task expect;
        input condition;
        input [8*160-1:0] message;
        begin
            if (!condition) begin
                errors = errors + 1;
                $display("TB FAIL: %s at %0t", message, $time);
            end
        end
    endtask

    task wait_cycles;
        input integer cycles;
        integer i;
        begin
            for (i = 0; i < cycles; i = i + 1)
                @(posedge clk);
        end
    endtask

    task wait_for_pmon_ack_update;
        integer i;
        begin
            for (i = 0;
                 (i < WAIT_LIMIT) &&
                 !((pwr_seq_ack != 16'd0) && (pwr_valid_ack === 1'b1));
                 i = i + 1)
                @(posedge clk);
            if (!((pwr_seq_ack != 16'd0) && (pwr_valid_ack === 1'b1))) begin
                $display("TB_INFO: ACK timeout pwr_seq=%0d valid=%b status=0x%02h age=%0d nack=%0d timeout=%0d pmon_hits=%0d pmon_reads=%0d pmon_writes=%0d st=%0d init=%b busy=%b done_code_r=%0d owner=%0d eng_state=%0d eng_done_code=%0d eng_wlen=%0d eng_rlen=%0d e_w_valid=%b e_w_ready=%b j3_w_valid=%b j3_w_ready=%b",
                         pwr_seq_ack, pwr_valid_ack, pwr_status_ack, pwr_age_ms_ack,
                         der_i2c_nack_count_ack, der_i2c_timeout_count_ack,
                         u_pmon_ack.addr_match_count, u_pmon_ack.read_data_count,
                         u_pmon_ack.write_byte_count,
                         dut_ack.u_pmon1_i2c_job.st_r,
                         dut_ack.u_pmon1_i2c_job.init_done,
                         dut_ack.u_pmon1_i2c_job.busy,
                         dut_ack.u_pmon1_i2c_job.done_code_r,
                         dut_ack.u_i2c_job_mux.owner_q,
                         dut_ack.u_i2c_master_engine.state,
                         dut_ack.u_i2c_master_engine.done_code,
                         dut_ack.u_i2c_master_engine.wlen_r,
                         dut_ack.u_i2c_master_engine.rlen_r,
                         dut_ack.e_w_valid,
                         dut_ack.e_w_ready,
                         dut_ack.j3_w_valid,
                         dut_ack.j3_w_ready);
            end
            expect((pwr_seq_ack != 16'd0) && (pwr_valid_ack === 1'b1),
                   "ACK PMON path published a valid snapshot");
        end
    endtask

    task wait_for_pmon_nack_update;
        integer i;
        begin
            for (i = 0;
                 (i < WAIT_LIMIT) &&
                 !((pwr_seq_nack != 16'd0) && (pwr_valid_nack === 1'b0) &&
                   (pwr_status_nack != 8'h01));
                 i = i + 1)
                @(posedge clk);
            if (!((pwr_seq_nack != 16'd0) && (pwr_valid_nack === 1'b0) &&
                  (pwr_status_nack != 8'h01))) begin
                $display("TB_INFO: NACK timeout pwr_seq=%0d valid=%b status=0x%02h age=%0d nack=%0d timeout=%0d st=%0d init=%b busy=%b",
                         pwr_seq_nack, pwr_valid_nack, pwr_status_nack,
                         pwr_age_ms_nack, der_i2c_nack_count_nack,
                         der_i2c_timeout_count_nack,
                         dut_nack.u_pmon1_i2c_job.st_r,
                         dut_nack.u_pmon1_i2c_job.init_done,
                         dut_nack.u_pmon1_i2c_job.busy);
            end
            expect((pwr_seq_nack != 16'd0) && (pwr_valid_nack === 1'b0) &&
                   (pwr_status_nack != 8'h01),
                   "missing PMON path published invalid non-initial status");
        end
    endtask

    initial begin
        errors = 0;
        rst = 1'b1;
        cfg_pmon_ack = 1'b0;
        cfg_pmon_nack = 1'b0;

        wait_cycles(40);
        rst = 1'b0;

        wait_cycles(8_000);

        expect(pwr_seq_ack == 16'd0, "SW10-low/disabled ACK path does not publish PMON sequence");
        expect(pwr_valid_ack == 1'b0, "SW10-low/disabled ACK path reports PMON invalid");
        expect(pwr_status_ack == 8'h01, "SW10-low/disabled ACK path reports not initialized");
        expect(pwr_age_ms_ack == 16'hFFFF, "SW10-low/disabled ACK path reports stale age");
        expect(u_pmon_ack.addr_match_count == 0, "SW10-low/disabled ACK path does not address 7'h38");

        expect(pwr_seq_nack == 16'd0, "SW10-low/disabled NACK path does not publish PMON sequence");
        expect(pwr_valid_nack == 1'b0, "SW10-low/disabled NACK path reports PMON invalid");
        expect(pwr_status_nack == 8'h01, "SW10-low/disabled NACK path reports not initialized");
        expect(pwr_age_ms_nack == 16'hFFFF, "SW10-low/disabled NACK path reports stale age");

        cfg_pmon_ack = 1'b1;
        cfg_pmon_nack = 1'b1;

        wait_for_pmon_ack_update();
        wait_for_pmon_nack_update();

        expect(u_pmon_ack.addr_match_count >= 4, "ACK PMON model saw data and status transactions");
        expect(u_pmon_ack.read_data_count >= 3, "ACK PMON model delivered data bytes");
        expect(pwr_status_ack == 8'h00, "ACK PMON snapshot status is OK");
        expect(pwr_payload_ack == 48'hA5A5_F3CF_0000,
               "ACK PMON payload carries status, voltage, current, reserved fields");
        expect(pwr_age_ms_ack != 16'hFFFF, "ACK PMON age leaves stale sentinel after update");
        expect(der_i2c_nack_count_ack == 16'd0, "ACK PMON path has no I2C NACKs");

        expect(pwr_valid_nack == 1'b0, "missing PMON snapshot is invalid");
        expect(pwr_status_nack == 8'hE0, "missing PMON snapshot reports I2C error status");
        expect(pwr_status_nack != 8'h00, "missing PMON does not report stale-good OK status");
        expect(der_i2c_nack_count_nack != 16'd0, "missing PMON increments suite NACK counter");

        $display("TB_INFO: ack_pwr_seq=%0d ack_payload=0x%012h ack_age=%0d pmon_addr_hits=%0d pmon_reads=%0d",
                 pwr_seq_ack, pwr_payload_ack, pwr_age_ms_ack,
                 u_pmon_ack.addr_match_count, u_pmon_ack.read_data_count);
        $display("TB_INFO: nack_pwr_seq=%0d nack_status=0x%02h nack_count=%0d timeout_count=%0d",
                 pwr_seq_nack, pwr_status_nack,
                 der_i2c_nack_count_nack, der_i2c_timeout_count_nack);

        if (errors == 0) begin
            $display("PASS: tb_rocket_i2c_suite_pmon1");
            $finish;
        end

        $display("FAIL: tb_rocket_i2c_suite_pmon1 errors=%0d", errors);
        $finish;
    end
endmodule

`default_nettype wire
