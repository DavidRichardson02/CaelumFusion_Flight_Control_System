`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// caelumfusion_top_vga_spi
//------------------------------------------------------------------------------
// Board-facing canonical integration top for the SPI sensor suite and VGA
// visualizer. This keeps the existing I2C top intact and exposes a parallel
// SPI bring-up path using the same published telemetry contract.
//
// SPI BUS MAP
//   - Shared SCLK / MOSI
//   - Shared MISO for LIS3DH and BMP5xx
//   - Dedicated LIS2MDL SDIO for default 3-wire SPI mode
//   - One chip-select per sensor
//
// NOTES
//   - rocket_spi_suite_top preserves the published snapshot / derived-state
//     interface already consumed by flight_viz_suite_top.
//   - The visualizer metadata inputs keep their legacy i2c_* names; in the SPI
//     path these carry the suite's protocol-error and timeout counters.
//   - CLS remains disabled here, so cls_tx is held high.
//==============================================================================
module caelumfusion_top_vga_spi #(
    parameter integer SYS_CLK_HZ = 100_000_000,

    parameter integer H_ACTIVE   = 640,
    parameter integer H_FP       = 16,
    parameter integer H_SYNC     = 96,
    parameter integer H_BP       = 48,
    parameter integer V_ACTIVE   = 480,
    parameter integer V_FP       = 10,
    parameter integer V_SYNC     = 2,
    parameter integer V_BP       = 33,
    parameter integer HSYNC_POL  = 0,
    parameter integer VSYNC_POL  = 0,

    parameter [15:0] BUILD_ID    = 16'd4,
    parameter integer USE_BLACKBOX_LOG = 0,
    parameter integer USE_MAG1_BENCH_SOURCE = 0,
    parameter [15:0]  MAG1_BENCH_OFFSET_X   = 16'sd0,
    parameter [15:0]  MAG1_BENCH_OFFSET_Y   = 16'sd0,
    parameter [15:0]  MAG1_BENCH_OFFSET_Z   = 16'sd0
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        sw_arm_raw,
    input  wire        sw_policy_enable_raw,

    output wire        spi_sclk,
    output wire        spi_mosi,
    input  wire        spi_miso,
    inout  wire        lis2mdl_sdio,
    output wire        lis3dh_cs_n,
    output wire        bmp5xx_cs_n,
    output wire        lis2mdl_cs_n,

    // Shared manual debug input: held high enables visualizer self-test.
    input  wire        btn_page_raw,

    // CLS UART TX remains idle-high while the console block is disabled.
    output wire        cls_tx,

    output wire        vga_hsync,
    output wire        vga_vsync,
    output wire [11:0] vga_rgb
);
    localparam [15:0] SCHEMA_WORD   = 16'hCF14;
    localparam [31:0] BUILD_ID_WORD = {16'h0000, BUILD_ID};

    //==========================================================================
    // Raw published snapshots
    //==========================================================================
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

    wire [31:0] mag1_t_us;
    wire [15:0] mag1_seq;
    wire        mag1_valid;
    wire [7:0]  mag1_status;
    wire [47:0] mag1_payload;
    wire [15:0] mag1_age_ms;
    wire [7:0]  mag1_cal_state;
    wire [7:0]  mag1_source_flags;
    wire [15:0] mag1_bridge_checksum;

    wire        ext_valid;
    wire [7:0]  ext_status;
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

    //==========================================================================
    // Derived-state publication bank
    //==========================================================================
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

    //==========================================================================
    // Observability / metadata published by the SYS suite
    //==========================================================================
    wire [15:0] proto_err_count;
    wire [15:0] timeout_count;
    wire [15:0] txn_rate_hz;
    wire [31:0] cdc_update_count;
    wire [31:0] frame_count_sys;
    wire [31:0] build_id;
    wire [15:0] schema_word;

    //==========================================================================
    // SYS-domain authority phase / safety gates
    //==========================================================================
    wire [3:0] auth_phase_code_sys;
    wire       auth_phase_valid_sys;
    wire       safety_runtime_ok_sys;
    wire       safety_allows_actuation_sys;
    wire       policy_runtime_enable_sys;
    wire       software_armed_sys;

    //==========================================================================
    // PIX clock / reset
    //==========================================================================
    wire pix_clk;
    wire clk_locked;
    wire pix_rst;

    reg [1:0] pix_rst_ff;
    always @(posedge pix_clk or posedge rst) begin
        if (rst)
            pix_rst_ff <= 2'b11;
        else if (!clk_locked)
            pix_rst_ff <= 2'b11;
        else
            pix_rst_ff <= {pix_rst_ff[0], 1'b0};
    end

    assign pix_rst = pix_rst_ff[1];

    // Preserve currently unused suite metadata in this top.
    wire _unused_suite_meta_ok;
    assign _unused_suite_meta_ok = der_t_us[0] ^ der_seq[0] ^ der_source_id[0] ^ frame_count_sys[0];

    //==========================================================================
    // Xilinx 7-series clock generator
    //==========================================================================
    clock_gen_xilinx_7series u_clock_gen (
        .clk_100m_in (clk),
        .mmcm_reset  (rst),
        .clk_25m_out (pix_clk),
        .mmcm_locked (clk_locked),
        .clk_valid   ()
    );

    //==========================================================================
    // SYS sensors + publication layer
    //==========================================================================
    rocket_spi_suite_top #(
        .CLK_HZ      (SYS_CLK_HZ),
        .BUILD_ID    (BUILD_ID_WORD),
        .SCHEMA_WORD (SCHEMA_WORD)
    ) u_sys_sensors (
        .clk                    (clk),
        .rst                    (rst),

        .spi_sclk               (spi_sclk),
        .spi_mosi               (spi_mosi),
        .spi_miso               (spi_miso),
        .lis2mdl_sdio           (lis2mdl_sdio),
        .lis3dh_cs_n            (lis3dh_cs_n),
        .bmp5xx_cs_n            (bmp5xx_cs_n),
        .lis2mdl_cs_n           (lis2mdl_cs_n),

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

        .der_i2c_nack_count     (proto_err_count),
        .der_i2c_timeout_count  (timeout_count),
        .der_txn_rate_hz        (txn_rate_hz),
        .der_cdc_update_count   (cdc_update_count),
        .der_frame_count        (frame_count_sys),
        .der_build_id           (build_id),
        .der_schema_word        (schema_word)
    );

    //==========================================================================
    // Future sensor-extension evidence and raw black-box logging contract
    //==========================================================================
    wire        ext_log_stream_valid;
    wire [31:0] ext_log_stream_word;
    wire        ext_log_stream_last;

    mag1_bench_snapshot_source #(
        .MAG1_BENCH_OFFSET_X (MAG1_BENCH_OFFSET_X),
        .MAG1_BENCH_OFFSET_Y (MAG1_BENCH_OFFSET_Y),
        .MAG1_BENCH_OFFSET_Z (MAG1_BENCH_OFFSET_Z),
        .MAG_FRESH_MAX_MS    (16'd200),
        .MAG1_SEQUENCE_OFFSET(16'd0)
    ) u_mag1_bench_snapshot_source (
        .clk                 (clk),
        .rst                 (rst),
        .enable              (USE_MAG1_BENCH_SOURCE != 0),
        .cfg_offset_x_en     (1'b1),
        .cfg_offset_y_en     (1'b1),
        .cfg_offset_z_en     (1'b1),

        .mag0_t_us           (mag_t_us),
        .mag0_seq            (mag_seq),
        .mag0_valid          (mag_valid),
        .mag0_status         (mag_status),
        .mag0_payload        (mag_payload),
        .mag0_age_ms         (der_mag_age_ms),

        .mag1_t_us           (mag1_t_us),
        .mag1_seq            (mag1_seq),
        .mag1_valid          (mag1_valid),
        .mag1_status         (mag1_status),
        .mag1_payload        (mag1_payload),
        .mag1_age_ms         (mag1_age_ms),
        .mag1_cal_state      (mag1_cal_state),
        .mag1_source_flags   (mag1_source_flags),
        .mag1_bridge_checksum(mag1_bridge_checksum)
    );

    sensor_extension_hub #(
        .PAYLOAD_W           (48),
        .ENABLE_BLACKBOX_LOG (USE_BLACKBOX_LOG)
    ) u_sensor_extension_hub (
        .clk                 (clk),
        .rst                 (rst),

        .bmp_t_us            (bmp_t_us),
        .bmp_seq             (bmp_seq),
        .bmp_valid           (bmp_valid),
        .bmp_status          (bmp_status),
        .bmp_payload         (bmp_payload),
        .bmp_age_ms          (der_bmp_age_ms),

        .acc_t_us            (acc_t_us),
        .acc_seq             (acc_seq),
        .acc_valid           (acc_valid),
        .acc_status          (acc_status),
        .acc_payload         (acc_payload),
        .acc_age_ms          (der_acc_age_ms),

        .mag_t_us            (mag_t_us),
        .mag_seq             (mag_seq),
        .mag_valid           (mag_valid),
        .mag_status          (mag_status),
        .mag_payload         (mag_payload),
        .mag_age_ms          (der_mag_age_ms),

        .pwr_t_us            (32'd0),
        .pwr_seq             (16'd0),
        .pwr_valid           (1'b0),
        .pwr_status          (8'h01),
        .pwr_payload         (48'd0),
        .pwr_age_ms          (16'hFFFF),

        .mag1_t_us           (mag1_t_us),
        .mag1_seq            (mag1_seq),
        .mag1_valid          (mag1_valid),
        .mag1_status         (mag1_status),
        .mag1_payload        (mag1_payload),
        .mag1_age_ms         (mag1_age_ms),
        .mag1_cal_state      (mag1_cal_state),
        .mag1_source_flags   (mag1_source_flags),
        .mag1_bridge_checksum(mag1_bridge_checksum),

        .rng_t_us            (32'd0),
        .rng_seq             (16'd0),
        .rng_valid           (1'b0),
        .rng_status          (8'h01),
        .rng_payload         (48'd0),
        .rng_age_ms          (16'hFFFF),

        .air_t_us            (32'd0),
        .air_seq             (16'd0),
        .air_valid           (1'b0),
        .air_status          (8'h01),
        .air_payload         (48'd0),
        .air_age_ms          (16'hFFFF),

        .env_t_us            (32'd0),
        .env_seq             (16'd0),
        .env_valid           (1'b0),
        .env_status          (8'h01),
        .env_payload         (48'd0),
        .env_age_ms          (16'hFFFF),

        .sun_t_us            (32'd0),
        .sun_seq             (16'd0),
        .sun_valid           (1'b0),
        .sun_status          (8'h01),
        .sun_payload         (48'd0),
        .sun_age_ms          (16'hFFFF),

        .flow_t_us           (32'd0),
        .flow_seq            (16'd0),
        .flow_valid          (1'b0),
        .flow_status         (8'h01),
        .flow_payload        (48'd0),
        .flow_age_ms         (16'hFFFF),

        .diag_selftest_enable(1'b0),
        .diag_fault_inject_enable(1'b0),
        .diag_fault_mode     (4'd0),

        .log_runtime_enable  (1'b1),
        .log_emit_req        (1'b0),
        .log_stream_ready    (1'b1),
        .log_stream_valid    (ext_log_stream_valid),
        .log_stream_word     (ext_log_stream_word),
        .log_stream_last     (ext_log_stream_last),

        .ext_valid           (ext_valid),
        .ext_status          (ext_status),
        .ext_present_flags   (ext_present_flags),
        .ext_fault_flags     (ext_fault_flags),
        .ext_mag_delta_l1    (ext_mag_delta_l1),
        .ext_mag_norm_primary(ext_mag_norm_primary),
        .ext_mag_norm_secondary(ext_mag_norm_secondary),
        .ext_mag_sequence_aligned(ext_mag_sequence_aligned),
        .ext_mag_disagreement(ext_mag_disagreement),
        .ext_mag_sector_delta(ext_mag_sector_delta),
        .ext_mag_norm_delta_l1(ext_mag_norm_delta_l1),
        .ext_mag_iron_residual(ext_mag_iron_residual),
        .ext_mag_cal_state   (ext_mag_cal_state),
        .ext_mag_source_flags(ext_mag_source_flags),
        .ext_mag_bridge_checksum(ext_mag_bridge_checksum),
        .ext_rng_height_cm   (ext_rng_height_cm),
        .ext_air_dp_pa       (ext_air_dp_pa),
        .ext_air_speed_cms   (ext_air_speed_cms),
        .ext_env_temp_cdeg   (ext_env_temp_cdeg),
        .ext_env_rh_centi    (ext_env_rh_centi),
        .ext_sun_luma        (ext_sun_luma),
        .ext_flow_dx         (ext_flow_dx),
        .ext_flow_dy         (ext_flow_dy),
        .ext_log_seq         (ext_log_seq),
        .ext_log_drop_count  (ext_log_drop_count),
        .ext_max_age_ms      (ext_max_age_ms)
    );

    wire _unused_ext_log_stream_ok;
    assign _unused_ext_log_stream_ok =
        ext_log_stream_valid ^ ext_log_stream_word[0] ^ ext_log_stream_last;

    //==========================================================================
    // Authority phase / gate producer
    //==========================================================================
    authority_gate_phase_sys u_authority_gate_phase (
        .clk                    (clk),
        .rst                    (rst),
        .sw_arm_raw             (sw_arm_raw),
        .sw_policy_enable_raw   (sw_policy_enable_raw),

        .bmp_valid              (bmp_valid),
        .bmp_status             (bmp_status),
        .bmp_age_ms             (der_bmp_age_ms),
        .acc_valid              (acc_valid),
        .acc_status             (acc_status),
        .acc_age_ms             (der_acc_age_ms),
        .mag_valid              (mag_valid),
        .mag_status             (mag_status),
        .mag_age_ms             (der_mag_age_ms),

        .der_valid              (der_valid),
        .der_status             (der_status),
        .der_alt_fresh          (der_alt_fresh),
        .der_vspd_fresh         (der_vspd_fresh),
        .der_roll_fresh         (der_roll_fresh),
        .der_head_fresh         (der_head_fresh),
        .der_bmp_age_ms         (der_bmp_age_ms),
        .der_acc_age_ms         (der_acc_age_ms),
        .der_mag_age_ms         (der_mag_age_ms),
        .der_bmp_valid_ref      (der_bmp_valid_ref),
        .der_acc_valid_ref      (der_acc_valid_ref),
        .der_mag_valid_ref      (der_mag_valid_ref),
        .der_vertical_speed_cms ($signed(der_vertical_speed_cms)),
        .der_altitude_cm        (der_altitude_cm),

        .auth_phase_code        (auth_phase_code_sys),
        .auth_phase_valid       (auth_phase_valid_sys),
        .safety_runtime_ok      (safety_runtime_ok_sys),
        .safety_allows_actuation(safety_allows_actuation_sys),
        .policy_runtime_enable  (policy_runtime_enable_sys),
    .software_armed         (software_armed_sys)
);

    wire        landing_nav_valid;
    wire [7:0]  landing_nav_status;
    wire [7:0]  landing_nav_flags;
    wire [15:0] landing_nav_downrange_m;
    wire [15:0] landing_nav_crossrange_m;
    wire [15:0] landing_nav_age_ms;
    wire        landing_wind_valid;
    wire [7:0]  landing_wind_status;
    wire [15:0] landing_wind_x_cms;
    wire [15:0] landing_wind_y_cms;
    wire [15:0] landing_wind_z_cms;
    wire [15:0] landing_wind_age_ms;

    landing_nav_wind_observer u_landing_nav_wind_observer (
        .der_valid            (der_valid),
        .der_status           (der_status),
        .der_alt_fresh        (der_alt_fresh),
        .der_vspd_fresh       (der_vspd_fresh),
        .der_bmp_age_ms       (der_bmp_age_ms),
        .der_altitude_cm      (der_altitude_cm),
        .ext_valid            (ext_valid),
        .ext_status           (ext_status),
        .ext_present_flags    (ext_present_flags),
        .ext_fault_flags      (ext_fault_flags),
        .ext_air_speed_cms    (ext_air_speed_cms),
        .ext_flow_dx          (ext_flow_dx),
        .ext_flow_dy          (ext_flow_dy),
        .ext_max_age_ms       (ext_max_age_ms),
        .nav_valid            (landing_nav_valid),
        .nav_status           (landing_nav_status),
        .nav_flags            (landing_nav_flags),
        .nav_downrange_m      (landing_nav_downrange_m),
        .nav_crossrange_m     (landing_nav_crossrange_m),
        .nav_age_ms           (landing_nav_age_ms),
        .wind_valid           (landing_wind_valid),
        .wind_status          (landing_wind_status),
        .wind_x_cms           (landing_wind_x_cms),
        .wind_y_cms           (landing_wind_y_cms),
        .wind_z_cms           (landing_wind_z_cms),
        .wind_age_ms          (landing_wind_age_ms)
    );

    //==========================================================================
    // Visualization suite
    //==========================================================================
    flight_viz_suite_top #(
        .PAYLOAD_W (48),

        .H_ACTIVE  (H_ACTIVE),
        .H_FP      (H_FP),
        .H_SYNC    (H_SYNC),
        .H_BP      (H_BP),
        .V_ACTIVE  (V_ACTIVE),
        .V_FP      (V_FP),
        .V_SYNC    (V_SYNC),
        .V_BP      (V_BP),
        .HSYNC_POL (HSYNC_POL),
        .VSYNC_POL (VSYNC_POL)
    ) u_viz (
        .sys_clk               (clk),
        .sys_rst               (rst),

        .bmp_t_us              (bmp_t_us),
        .bmp_seq               (bmp_seq),
        .bmp_valid             (bmp_valid),
        .bmp_status            (bmp_status),
        .bmp_payload           (bmp_payload),
        .bmp_age_ms            (der_bmp_age_ms),

        .acc_t_us              (acc_t_us),
        .acc_seq               (acc_seq),
        .acc_valid             (acc_valid),
        .acc_status            (acc_status),
        .acc_payload           (acc_payload),
        .acc_age_ms            (der_acc_age_ms),

        .mag_t_us              (mag_t_us),
        .mag_seq               (mag_seq),
        .mag_valid             (mag_valid),
        .mag_status            (mag_status),
        .mag_payload           (mag_payload),
        .mag_age_ms            (der_mag_age_ms),

        .pwr_t_us              (32'd0),
        .pwr_seq               (16'd0),
        .pwr_valid             (1'b0),
        .pwr_status            (8'h01),
        .pwr_payload           (48'd0),
        .pwr_age_ms            (16'hFFFF),

        .ext_valid             (ext_valid),
        .ext_status            (ext_status),
        .ext_present_flags     (ext_present_flags),
        .ext_fault_flags       (ext_fault_flags),
        .ext_mag_delta_l1      (ext_mag_delta_l1),
        .ext_mag_norm_primary  (ext_mag_norm_primary),
        .ext_mag_norm_secondary(ext_mag_norm_secondary),
        .ext_mag_sequence_aligned(ext_mag_sequence_aligned),
        .ext_mag_disagreement  (ext_mag_disagreement),
        .ext_mag_sector_delta  (ext_mag_sector_delta),
        .ext_mag_source_flags  (ext_mag_source_flags),
        .ext_rng_height_cm     (ext_rng_height_cm),
        .ext_air_dp_pa         (ext_air_dp_pa),
        .ext_air_speed_cms     (ext_air_speed_cms),
        .ext_env_temp_cdeg     (ext_env_temp_cdeg),
        .ext_env_rh_centi      (ext_env_rh_centi),
        .ext_sun_luma          (ext_sun_luma),
        .ext_flow_dx           (ext_flow_dx),
        .ext_flow_dy           (ext_flow_dy),
        .ext_log_seq           (ext_log_seq),
        .ext_log_drop_count    (ext_log_drop_count),
        .ext_max_age_ms        (ext_max_age_ms),

        .der_valid             (der_valid),
        .der_status            (der_status),
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

    .nav_valid             (landing_nav_valid),
    .nav_status            (landing_nav_status),
    .nav_flags             (landing_nav_flags),
    .nav_downrange_m       (landing_nav_downrange_m),
    .nav_crossrange_m      (landing_nav_crossrange_m),
    .nav_age_ms            (landing_nav_age_ms),

    .wind_valid            (landing_wind_valid),
    .wind_status           (landing_wind_status),
    .wind_x_cms            (landing_wind_x_cms),
    .wind_y_cms            (landing_wind_y_cms),
    .wind_z_cms            (landing_wind_z_cms),
    .wind_age_ms           (landing_wind_age_ms),

        .auth_phase_code_sys   (auth_phase_code_sys),
        .auth_phase_valid_sys  (auth_phase_valid_sys),
        .safety_runtime_ok_sys (safety_runtime_ok_sys),
        .safety_allows_actuation_sys(safety_allows_actuation_sys),
        .policy_runtime_enable_sys(policy_runtime_enable_sys),
        .software_armed_sys    (software_armed_sys),

        .i2c_nack_count        (proto_err_count),
        .i2c_timeout_count     (timeout_count),
        .txn_rate_hz           (txn_rate_hz),
        .cdc_update_count_sys  (cdc_update_count),
        .build_id              (build_id),
        .schema_word           (schema_word),

        .viz_selftest_en_sys   (btn_page_raw),
        .vga_page_select_sys   (btn_page_raw ? 2'd1 : 2'd0),
        .history_freeze_sys    (1'b0),

        .pix_clk               (pix_clk),
        .pix_rst               (pix_rst),

        .vga_hsync             (vga_hsync),
        .vga_vsync             (vga_vsync),
        .vga_rgb               (vga_rgb)
    );

    assign cls_tx = 1'b1;

endmodule

`default_nettype wire
