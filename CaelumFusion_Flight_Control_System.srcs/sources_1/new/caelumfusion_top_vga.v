`timescale 1ns/1ps
`default_nettype none

`include "telemetry_defs_vh.vh"

//==============================================================================
// caelumfusion_top_vga
//------------------------------------------------------------------------------
// Canonical board-facing integration top for the CaelumFusion sensor + VGA
// stack. This top owns:
//
//   1) pixel clock / reset generation
//   2) metadata and schema parameterization
//   3) raw/derived telemetry routing into the visualizer
//   4) conservative disabled-CLS behavior
//
// BUS SELECTION
//   Default build: LIS3DH auxiliary accelerometer on shared I2C,
//                  CMPS2/MMC34160PJ heading path at I2C 7'h30,
//                  and BMP I2C path. The ACL2/ADXL362 SPI path is compiled
//                  in for the live-control build and can be runtime-gated.
//   SPI build: add Verilog define CAELUM_SENSOR_SPI in Vivado
//
// NOTES
//   - The legacy shared-SPI build keeps its original sensor pins. The ACL2 pins
//     remain board-facing in both builds so the active XDC has stable port names.
//   - The visualizer keeps its legacy i2c_* observability names; in SPI mode
//     those ports carry the SPI suite's protocol-error and timeout counters.
//==============================================================================
module caelumfusion_top_vga #(
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
    parameter integer ADXL_IRQ_POLICY = 1,

    // Basys-3 resource-safe default: keep the shared-I2C LIS3DH path compiled.
    // The ADXL362/Pmod ACL2 SPI path remains available by parameter override,
    // but compiling both accelerometer stacks plus the diagnostic VGA pages does
    // not fit comfortably in xc7a35t.
    parameter integer USE_LIS3DH_I2C_ACC    = 1,
    parameter integer USE_ADXL362_SPI_ACC   = 1,

    // CMPS2/MMC34160PJ heading publication default. This does not add pins; the
    // sensor shares the existing scl/sda bus and is addressed as 7'h30.
    parameter integer USE_CMPS2_MMC3416_MAG = 1,

    // PMON1/ADM1191 power publication logic is present in the default bring-up
    // bitstream; SW10 still gates live requests/publication on the shared bus.
    parameter integer USE_PMON1_PWR         = 1,
    parameter [6:0]   PMON1_ADDR7           = 7'h38,
    parameter integer USE_HYGRO_ENV         = 0,
    parameter integer USE_GYRO_I2C          = 0,
    // Physical LIS2MDL/MAG1 is a deliberate hardware-validation path. SW15
    // still gates live requests/publication so the board can boot with optional
    // extension devices disconnected.
    parameter integer USE_LIS2MDL_MAG1      = 1,
    parameter integer USE_BLACKBOX_LOG      = 1,
    // Fixed-packet external-MCU UART range bridge. The parameter and ports keep
    // their historical Teensy names, but the active bench producer is now the
    // EK-TM4C123GXL LaunchPad. Default-off keeps disconnected JXADC UART noise
    // from becoming range evidence. SW15 is the runtime extension gate.
    parameter integer USE_TEENSY_UART_RANGE_BRIDGE = 0,
    parameter integer TEENSY_UART_BAUD             = 115_200,
    // Synthetic MAG1 is useful for bench evidence. It remains explicitly SW3
    // gated and tagged so it cannot be mistaken for physical MAG1 validation.
    parameter integer USE_MAG1_BENCH_SOURCE = 1,
    parameter [15:0]  MAG1_BENCH_OFFSET_X   = 16'sd0,
    parameter [15:0]  MAG1_BENCH_OFFSET_Y   = 16'sd0,
    parameter [15:0]  MAG1_BENCH_OFFSET_Z   = 16'sd0,

    // Full-screen Planar Compass Truth page overlay. This diagnostic renderer is
    // intentionally off in the default Basys-3 build because it costs several
    // thousand LUTs. Set this to 1 for a larger device or a diagnostic-only run.
    parameter integer USE_COMPASS_TRUTH_PAGE = 0,

    // Secondary in-HUD sensor diagnostic page. The primary HUD remains enabled;
    // this optional page is off by default to recover LUT margin on Basys-3.
    parameter integer USE_SENSOR_DIAG_PAGE = 1,

    // Compact full-screen science/evidence pages. These are pure VGA overlays
    // after the legacy HUD/diagnostic/compass path and use direct view IDs 4-6.
    parameter integer USE_SCIENCE_PAGES = 1,

    // Optional debug-build direct selector.  When zero, BTNR preserves the
    // source-default direct compass request.  When nonzero, BTNR latches the
    // synchronized SW13:SW11 value into view_direct_id_sys only while SW3 MAG1
    // bench and SW2+SW6 diagnostic fault injection are inactive; during those
    // ownership modes the request is rejected through the reserved invalid ID.
    parameter integer USE_SWITCH_ENCODED_VIEW_SELECT = 1,

    // Frozen text labels are useful in rich diagnostic builds, but they are
    // expensive pixel-domain combinational logic. The default Basys-3 image
    // keeps the graphical HUD and disables the text overlay for LUT margin.
    parameter integer USE_TELEMETRY_TEXT_OVERLAY = 1,

    // Set to 1 only when USE_COMPASS_TRUTH_PAGE is also enabled and the build
    // should boot directly into the compass truth page.
    parameter integer COMPASS_TRUTH_PAGE_DEFAULT = 0
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        sw_arm_raw,
    input  wire        sw_policy_enable_raw,

`ifdef CAELUM_SENSOR_SPI
    output wire        spi_sclk,
    output wire        spi_mosi,
    input  wire        spi_miso,
    inout  wire        lis2mdl_sdio,
    output wire        lis3dh_cs_n,
    output wire        bmp5xx_cs_n,
    output wire        lis2mdl_cs_n,
`else
    output wire        scl,
    inout  wire        sda,
`endif

    output wire        adxl362_cs_n,
    output wire        adxl362_mosi,
    input  wire        adxl362_miso,
    output wire        adxl362_sclk,
    input  wire        adxl362_int1,
    input  wire        adxl362_int2,

    input  wire        btn_page_raw,
    input  wire        btn_prev_raw,
    input  wire        btn_direct_compass_raw,
    input  wire        sw_selftest_raw,
    input  wire        sw_mag1_bench_raw,
    input  wire        sw_compass_page_raw,
    input  wire        sw_history_freeze_raw,
    input  wire        sw_log_diag_raw,
    input  wire        sw_lis3dh_i2c_acc_raw,
    input  wire        sw_adxl362_spi_acc_raw,
    input  wire        sw_cmps2_mmc3416_mag_raw,
    input  wire        sw_pmon1_pwr_raw,
    input  wire        sw_mag1_offset_x_raw,
    input  wire        sw_mag1_offset_y_raw,
    input  wire        sw_mag1_offset_z_raw,
    input  wire        sw_compass_default_raw,
    input  wire        sw_ext_i2c_raw,
    input  wire [3:0]  ls1_s_raw,
    input  wire        pir_motion_raw,
    output wire        dpot_cs_n,
    output wire        dpot_mosi,
    output wire        dpot_sclk,
    output wire        cls_tx,
    input  wire        teensy_uart_rx_raw,
    output wire        teensy_uart_tx,

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

    wire [31:0] mag1_phys_t_us;
    wire [15:0] mag1_phys_seq;
    wire        mag1_phys_valid;
    wire [7:0]  mag1_phys_status;
    wire [47:0] mag1_phys_payload;
    wire [15:0] mag1_phys_age_ms;
    wire [7:0]  mag1_phys_cal_state;
    wire [7:0]  mag1_phys_source_flags;
    wire [15:0] mag1_phys_bridge_checksum;

    wire [31:0] mag1_bench_t_us;
    wire [15:0] mag1_bench_seq;
    wire        mag1_bench_valid;
    wire [7:0]  mag1_bench_status;
    wire [47:0] mag1_bench_payload;
    wire [15:0] mag1_bench_age_ms;
    wire [7:0]  mag1_bench_cal_state;
    wire [7:0]  mag1_bench_source_flags;
    wire [15:0] mag1_bench_bridge_checksum;

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

    wire [31:0] gyro_t_us;
    wire [15:0] gyro_seq;
    wire        gyro_valid;
    wire [7:0]  gyro_status;
    wire [47:0] gyro_payload;
    wire [15:0] gyro_age_ms;

    wire [31:0] sun_t_us;
    wire [15:0] sun_seq;
    wire        sun_valid;
    wire [7:0]  sun_status;
    wire [47:0] sun_payload;
    wire [15:0] sun_age_ms;
    wire [3:0]  ls1_s_level_sys;
    wire        pir_motion_level_sys;

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
    wire [31:0] rng_bridge_t_us;
    wire [15:0] rng_bridge_seq;
    wire        rng_bridge_valid;
    wire [7:0]  rng_bridge_status;
    wire [47:0] rng_bridge_payload;
    wire [15:0] rng_bridge_age_ms;
    wire [7:0]  teensy_bridge_last_type;
    wire [15:0] teensy_bridge_last_seq;
    wire        teensy_bridge_heartbeat_seen;
    wire [15:0] teensy_bridge_heartbeat_seq;
    wire [15:0] teensy_bridge_heartbeat_age_ms;
    wire [15:0] teensy_bridge_checksum_fault_count;
    wire [15:0] teensy_bridge_unsupported_count;
    wire [15:0] teensy_uart_framing_error_count;
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
    wire [15:0] bus_error_count;
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
    // Live board controls
    //---------------------------------------------------------------------------
    // SW0/SW1 are consumed by the authority gate below. The remaining switches
    // are synchronous runtime masks for hardware already present in the
    // bitstream. They do not override USE_* parameters that excluded a block at
    // elaboration time.
    //==========================================================================
    wire btn_next_level_sys;
    wire btn_next_rise_sys;
    wire btn_next_fall_sys;
    wire btn_next_toggle_sys;
    wire btn_prev_level_sys;
    wire btn_prev_rise_sys;
    wire btn_prev_fall_sys;
    wire btn_prev_toggle_sys;
    wire btn_direct_compass_level_sys;
    wire btn_direct_compass_rise_sys;
    wire btn_direct_compass_fall_sys;
    wire btn_direct_compass_toggle_sys;

    caelumfusion_button_debounce_pulse_sys #(
        .STABLE_CYCLES (250000),
        .COUNT_W       (19)
    ) u_btn_next_page_ctrl (
        .clk          (clk),
        .rst          (rst),
        .async_in     (btn_page_raw),
        .sync_level   (btn_next_level_sys),
        .rise_pulse   (btn_next_rise_sys),
        .fall_pulse   (btn_next_fall_sys),
        .toggle_pulse (btn_next_toggle_sys)
    );

    caelumfusion_button_debounce_pulse_sys #(
        .STABLE_CYCLES (250000),
        .COUNT_W       (19)
    ) u_btn_prev_page_ctrl (
        .clk          (clk),
        .rst          (rst),
        .async_in     (btn_prev_raw),
        .sync_level   (btn_prev_level_sys),
        .rise_pulse   (btn_prev_rise_sys),
        .fall_pulse   (btn_prev_fall_sys),
        .toggle_pulse (btn_prev_toggle_sys)
    );

    caelumfusion_button_debounce_pulse_sys #(
        .STABLE_CYCLES (250000),
        .COUNT_W       (19)
    ) u_btn_direct_compass_ctrl (
        .clk          (clk),
        .rst          (rst),
        .async_in     (btn_direct_compass_raw),
        .sync_level   (btn_direct_compass_level_sys),
        .rise_pulse   (btn_direct_compass_rise_sys),
        .fall_pulse   (btn_direct_compass_fall_sys),
        .toggle_pulse (btn_direct_compass_toggle_sys)
    );

    wire sw_selftest_level_sys;
    wire sw_mag1_bench_level_sys;
    wire sw_compass_page_level_sys;
    wire sw_history_freeze_level_sys;
    wire sw_log_diag_level_sys;
    wire sw_lis3dh_i2c_acc_level_sys;
    wire sw_adxl362_spi_acc_level_sys;
    wire sw_cmps2_mmc3416_mag_level_sys;
    wire sw_pmon1_pwr_level_sys;
    wire sw_mag1_offset_x_level_sys;
    wire sw_mag1_offset_y_level_sys;
    wire sw_mag1_offset_z_level_sys;
    wire sw_compass_default_level_sys;
    wire sw_ext_i2c_level_sys;

    sync_bit_3ff u_sw_selftest_sync (
        .clk(clk), .rst(rst), .async_in(sw_selftest_raw),
        .sync_level(sw_selftest_level_sys), .rise_pulse(), .fall_pulse(), .toggle_pulse()
    );
    sync_bit_3ff u_sw_mag1_bench_sync (
        .clk(clk), .rst(rst), .async_in(sw_mag1_bench_raw),
        .sync_level(sw_mag1_bench_level_sys), .rise_pulse(), .fall_pulse(), .toggle_pulse()
    );
    sync_bit_3ff u_sw_compass_page_sync (
        .clk(clk), .rst(rst), .async_in(sw_compass_page_raw),
        .sync_level(sw_compass_page_level_sys), .rise_pulse(), .fall_pulse(), .toggle_pulse()
    );
    sync_bit_3ff u_sw_history_freeze_sync (
        .clk(clk), .rst(rst), .async_in(sw_history_freeze_raw),
        .sync_level(sw_history_freeze_level_sys), .rise_pulse(), .fall_pulse(), .toggle_pulse()
    );
    sync_bit_3ff u_sw_log_diag_sync (
        .clk(clk), .rst(rst), .async_in(sw_log_diag_raw),
        .sync_level(sw_log_diag_level_sys), .rise_pulse(), .fall_pulse(), .toggle_pulse()
    );
    sync_bit_3ff u_sw_lis3dh_i2c_acc_sync (
        .clk(clk), .rst(rst), .async_in(sw_lis3dh_i2c_acc_raw),
        .sync_level(sw_lis3dh_i2c_acc_level_sys), .rise_pulse(), .fall_pulse(), .toggle_pulse()
    );
    sync_bit_3ff u_sw_adxl362_spi_acc_sync (
        .clk(clk), .rst(rst), .async_in(sw_adxl362_spi_acc_raw),
        .sync_level(sw_adxl362_spi_acc_level_sys), .rise_pulse(), .fall_pulse(), .toggle_pulse()
    );
    sync_bit_3ff u_sw_cmps2_mmc3416_mag_sync (
        .clk(clk), .rst(rst), .async_in(sw_cmps2_mmc3416_mag_raw),
        .sync_level(sw_cmps2_mmc3416_mag_level_sys), .rise_pulse(), .fall_pulse(), .toggle_pulse()
    );
    sync_bit_3ff u_sw_pmon1_pwr_sync (
        .clk(clk), .rst(rst), .async_in(sw_pmon1_pwr_raw),
        .sync_level(sw_pmon1_pwr_level_sys), .rise_pulse(), .fall_pulse(), .toggle_pulse()
    );
    sync_bit_3ff u_sw_mag1_offset_x_sync (
        .clk(clk), .rst(rst), .async_in(sw_mag1_offset_x_raw),
        .sync_level(sw_mag1_offset_x_level_sys), .rise_pulse(), .fall_pulse(), .toggle_pulse()
    );
    sync_bit_3ff u_sw_mag1_offset_y_sync (
        .clk(clk), .rst(rst), .async_in(sw_mag1_offset_y_raw),
        .sync_level(sw_mag1_offset_y_level_sys), .rise_pulse(), .fall_pulse(), .toggle_pulse()
    );
    sync_bit_3ff u_sw_mag1_offset_z_sync (
        .clk(clk), .rst(rst), .async_in(sw_mag1_offset_z_raw),
        .sync_level(sw_mag1_offset_z_level_sys), .rise_pulse(), .fall_pulse(), .toggle_pulse()
    );
    sync_bit_3ff u_sw_compass_default_sync (
        .clk(clk), .rst(rst), .async_in(sw_compass_default_raw),
        .sync_level(sw_compass_default_level_sys), .rise_pulse(), .fall_pulse(), .toggle_pulse()
    );
    sync_bit_3ff u_sw_ext_i2c_sync (
        .clk(clk), .rst(rst), .async_in(sw_ext_i2c_raw),
        .sync_level(sw_ext_i2c_level_sys), .rise_pulse(), .fall_pulse(), .toggle_pulse()
    );

    wire _unused_button_levels_ok;
    assign _unused_button_levels_ok =
        btn_next_level_sys ^ btn_next_fall_sys ^ btn_next_toggle_sys ^
        btn_prev_level_sys ^ btn_prev_fall_sys ^ btn_prev_toggle_sys ^
        btn_direct_compass_level_sys ^ btn_direct_compass_fall_sys ^
        btn_direct_compass_toggle_sys;

    // DPOT is physically allocated but held inactive until a command-authority
    // path is added. CS_N high prevents accidental wiper updates.
    assign dpot_cs_n = 1'b1;
    assign dpot_mosi = 1'b0;
    assign dpot_sclk = 1'b0;

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

    // Preserve suite metadata not consumed by the current visualizer top.
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
`ifdef CAELUM_SENSOR_SPI
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

        .der_i2c_nack_count     (bus_error_count),
        .der_i2c_timeout_count  (timeout_count),
        .der_txn_rate_hz        (txn_rate_hz),
        .der_cdc_update_count   (cdc_update_count),
        .der_frame_count        (frame_count_sys),
        .der_build_id           (build_id),
        .der_schema_word        (schema_word)
    );

    assign pwr_t_us    = 32'd0;
    assign pwr_seq     = 16'd0;
    assign pwr_valid   = 1'b0;
    assign pwr_status  = 8'h01;
    assign pwr_payload = 48'd0;
    assign pwr_age_ms  = 16'hFFFF;

    assign env_t_us     = 32'd0;
    assign env_seq      = 16'd0;
    assign env_valid    = 1'b0;
    assign env_status   = 8'h01;
    assign env_payload  = 48'd0;
    assign env_age_ms   = 16'hFFFF;

    assign mag1_phys_t_us            = 32'd0;
    assign mag1_phys_seq             = 16'd0;
    assign mag1_phys_valid           = 1'b0;
    assign mag1_phys_status          = `ST_NOT_INITIALIZED;
    assign mag1_phys_payload         = 48'd0;
    assign mag1_phys_age_ms          = 16'hFFFF;
    assign mag1_phys_cal_state       = 8'd0;
    assign mag1_phys_source_flags    = 8'd0;
    assign mag1_phys_bridge_checksum = 16'd0;

    assign gyro_t_us    = 32'd0;
    assign gyro_seq     = 16'd0;
    assign gyro_valid   = 1'b0;
    assign gyro_status  = 8'h01;
    assign gyro_payload = 48'd0;
    assign gyro_age_ms  = 16'hFFFF;

    assign adxl362_cs_n = 1'b1;
    assign adxl362_mosi = 1'b0;
    assign adxl362_sclk = 1'b0;
`else
    rocket_i2c_suite_top #(
        .CLK_HZ              (SYS_CLK_HZ),
        .BUILD_ID            (BUILD_ID_WORD),
        .SCHEMA_WORD         (SCHEMA_WORD),
        .ADXL_IRQ_POLICY     (ADXL_IRQ_POLICY),
        .USE_LIS3DH_I2C_ACC    (USE_LIS3DH_I2C_ACC),
        .USE_ADXL362_SPI_ACC   (USE_ADXL362_SPI_ACC),
        .USE_CMPS2_MMC3416_MAG (USE_CMPS2_MMC3416_MAG),
        .USE_PMON1_PWR         (USE_PMON1_PWR),
        .PMON1_ADDR7           (PMON1_ADDR7),
        .USE_HYGRO_ENV         (USE_HYGRO_ENV),
        .USE_GYRO_I2C          (USE_GYRO_I2C),
        .USE_LIS2MDL_MAG1      (USE_LIS2MDL_MAG1)
    ) u_sys_sensors (
        .clk                    (clk),
        .rst                    (rst),
        .cfg_lis3dh_i2c_acc_en  (sw_lis3dh_i2c_acc_level_sys),
        .cfg_adxl362_spi_acc_en (sw_adxl362_spi_acc_level_sys),
        .cfg_cmps2_mmc3416_mag_en(sw_cmps2_mmc3416_mag_level_sys),
        .cfg_pmon1_pwr_en       (sw_pmon1_pwr_level_sys),
        .cfg_ext_i2c_en         (sw_ext_i2c_level_sys),

        .scl                    (scl),
        .sda                    (sda),
        .adxl362_cs_n           (adxl362_cs_n),
        .adxl362_mosi           (adxl362_mosi),
        .adxl362_miso           (adxl362_miso),
        .adxl362_sclk           (adxl362_sclk),
        .adxl362_int1           (adxl362_int1),
        .adxl362_int2           (adxl362_int2),

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

        .mag1_t_us              (mag1_phys_t_us),
        .mag1_seq               (mag1_phys_seq),
        .mag1_valid             (mag1_phys_valid),
        .mag1_status            (mag1_phys_status),
        .mag1_payload           (mag1_phys_payload),
        .mag1_age_ms            (mag1_phys_age_ms),
        .mag1_cal_state         (mag1_phys_cal_state),
        .mag1_source_flags      (mag1_phys_source_flags),
        .mag1_bridge_checksum   (mag1_phys_bridge_checksum),

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

        .der_i2c_nack_count     (bus_error_count),
        .der_i2c_timeout_count  (timeout_count),
        .der_txn_rate_hz        (txn_rate_hz),
        .der_cdc_update_count   (cdc_update_count),
        .der_frame_count        (frame_count_sys),
        .der_build_id           (build_id),
        .der_schema_word        (schema_word)
    );
`endif

    //==========================================================================
    // Future sensor-extension evidence and raw black-box logging contract
    //--------------------------------------------------------------------------
    // Physical drivers publish into this stable snapshot contract. HYGRO drives
    // env_*, LS1/PIR drive sun_*, and LIS2MDL can drive MAG1 only when the
    // optional extension I2C group is compiled and SW15 enables it. The external
    // MCU UART range bridge can feed rng_* when compiled and SW15-enabled;
    // air/flow banks remain explicitly unavailable.
    //==========================================================================
    wire        ext_log_stream_valid;
    wire [31:0] ext_log_stream_word;
    wire        ext_log_stream_last;
    localparam integer LOG_EMIT_HZ = 10;
    localparam integer LOG_EMIT_DIVISOR =
        (SYS_CLK_HZ > LOG_EMIT_HZ) ? (SYS_CLK_HZ / LOG_EMIT_HZ) : 1;
    reg [31:0] log_emit_div_r;
    reg        log_emit_req_sys;

    always @(posedge clk) begin
        if (rst || !sw_log_diag_level_sys) begin
            log_emit_div_r   <= 32'd0;
            log_emit_req_sys <= 1'b0;
        end else begin
            log_emit_req_sys <= 1'b0;
            if (log_emit_div_r >= (LOG_EMIT_DIVISOR - 1)) begin
                log_emit_div_r   <= 32'd0;
                log_emit_req_sys <= 1'b1;
            end else begin
                log_emit_div_r <= log_emit_div_r + 32'd1;
            end
        end
    end

    wire diag_fault_inject_sys = sw_selftest_level_sys & sw_log_diag_level_sys;
    wire [2:0] diag_fault_select_sys =
        {sw_mag1_offset_z_level_sys, sw_mag1_offset_y_level_sys,
         sw_mag1_offset_x_level_sys};
    wire [2:0] diag_fault_class_sys =
        !diag_fault_inject_sys ? `DIAG_FAULT_NONE :
        (diag_fault_select_sys == 3'd0) ? `DIAG_FAULT_STATUS :
        (diag_fault_select_sys <= `DIAG_FAULT_OUT_OF_RANGE) ?
            diag_fault_select_sys :
            `DIAG_FAULT_STATUS;
    wire [1:0] diag_fault_bank_sys =
        sw_pmon1_pwr_level_sys ? `DIAG_BANK_PWR :
        sw_cmps2_mmc3416_mag_level_sys ? `DIAG_BANK_MAG :
        (sw_lis3dh_i2c_acc_level_sys || sw_adxl362_spi_acc_level_sys) ?
            `DIAG_BANK_ACC :
            `DIAG_BANK_BMP;
    wire [3:0] diag_fault_mode_sys = diag_fault_inject_sys ?
        {1'b0,
         (diag_fault_class_sys == `DIAG_FAULT_OUT_OF_RANGE),
         (diag_fault_class_sys == `DIAG_FAULT_STALE),
         ((diag_fault_class_sys == `DIAG_FAULT_STATUS) ||
          (diag_fault_class_sys == `DIAG_FAULT_INVALID_PAYLOAD))} :
        4'd0;

    wire       render_direct_valid_sys;
    wire [2:0] render_direct_view_id_sys;
    wire       render_direct_selector_collision_sys;

    caelumfusion_vga_direct_view_arbiter_sys #(
        .USE_SWITCH_ENCODED_VIEW_SELECT(USE_SWITCH_ENCODED_VIEW_SELECT)
    ) u_render_direct_view_arbiter (
        .direct_button_pulse_sys (btn_direct_compass_rise_sys),
        .sw_mag1_bench_level_sys (sw_mag1_bench_level_sys),
        .diag_fault_inject_sys   (diag_fault_inject_sys),
        .sw_view_id0_level_sys   (sw_mag1_offset_x_level_sys),
        .sw_view_id1_level_sys   (sw_mag1_offset_y_level_sys),
        .sw_view_id2_level_sys   (sw_mag1_offset_z_level_sys),
        .view_direct_valid_sys   (render_direct_valid_sys),
        .view_direct_id_sys      (render_direct_view_id_sys),
        .selector_collision_sys  (render_direct_selector_collision_sys)
    );

    wire [31:0] ext_time_us;
    wire        ext_tick_1us;
    timebase_us #(
        .CLK_HZ(SYS_CLK_HZ)
    ) u_extension_timebase_us (
        .clk      (clk),
        .rst      (rst),
        .time_us  (ext_time_us),
        .tick_1us (ext_tick_1us)
    );

    localparam integer GPIO_SAMPLE_HZ = 10;
    localparam integer GPIO_SAMPLE_DIVISOR =
        (SYS_CLK_HZ > GPIO_SAMPLE_HZ) ? (SYS_CLK_HZ / GPIO_SAMPLE_HZ) : 1;
    reg [31:0] gpio_sample_div_r;
    reg        gpio_sample_req_sys;

    always @(posedge clk) begin
        if (rst) begin
            gpio_sample_div_r  <= 32'd0;
            gpio_sample_req_sys <= 1'b0;
        end else begin
            gpio_sample_req_sys <= 1'b0;
            if (gpio_sample_div_r >= (GPIO_SAMPLE_DIVISOR - 1)) begin
                gpio_sample_div_r  <= 32'd0;
                gpio_sample_req_sys <= 1'b1;
            end else begin
                gpio_sample_div_r <= gpio_sample_div_r + 32'd1;
            end
        end
    end

    pmod_gpio_capture u_pmod_gpio_capture (
        .clk              (clk),
        .rst              (rst),
        .time_us          (ext_time_us),
        .tick_1us         (ext_tick_1us),
        .sample_event     (gpio_sample_req_sys),
        .ls1_s_raw        (ls1_s_raw),
        .pir_motion_raw   (pir_motion_raw),
        .ls1_s_level      (ls1_s_level_sys),
        .pir_motion_level (pir_motion_level_sys),
        .sun_t_us         (sun_t_us),
        .sun_seq          (sun_seq),
        .sun_valid        (sun_valid),
        .sun_status       (sun_status),
        .sun_payload      (sun_payload),
        .sun_age_ms       (sun_age_ms)
    );

    teensy_uart_range_bridge #(
        .CLK_HZ                     (SYS_CLK_HZ),
        .BAUD                       (TEENSY_UART_BAUD),
        .PAYLOAD_W                  (48),
        .RANGE_FRESH_MAX_MS         (16'd500),
        .HEARTBEAT_FRESH_MAX_MS     (16'd500),
        .REQUIRE_HEARTBEAT_FOR_DATA (1),
        .MAX_RANGE_CM               (16'd10000),
        .MIN_RANGE_CONFIDENCE       (16'd1)
    ) u_teensy_uart_range_bridge (
        .clk                         (clk),
        .rst                         (rst),
        .enable                      ((USE_TEENSY_UART_RANGE_BRIDGE != 0) &&
                                      sw_ext_i2c_level_sys),
        .now_us                      (ext_time_us),
        .tick_1us                    (ext_tick_1us),
        .uart_rx                     (teensy_uart_rx_raw),
        .uart_tx                     (teensy_uart_tx),
        .rng_t_us                    (rng_bridge_t_us),
        .rng_seq                     (rng_bridge_seq),
        .rng_valid                   (rng_bridge_valid),
        .rng_status                  (rng_bridge_status),
        .rng_payload                 (rng_bridge_payload),
        .rng_age_ms                  (rng_bridge_age_ms),
        .bridge_last_type            (teensy_bridge_last_type),
        .bridge_last_seq             (teensy_bridge_last_seq),
        .bridge_heartbeat_seen       (teensy_bridge_heartbeat_seen),
        .bridge_heartbeat_seq        (teensy_bridge_heartbeat_seq),
        .bridge_heartbeat_age_ms     (teensy_bridge_heartbeat_age_ms),
        .bridge_checksum_fault_count (teensy_bridge_checksum_fault_count),
        .bridge_unsupported_count    (teensy_bridge_unsupported_count),
        .uart_framing_error_count    (teensy_uart_framing_error_count)
    );

    wire _unused_teensy_bridge_diag_ok;
    assign _unused_teensy_bridge_diag_ok =
        teensy_bridge_last_type[0] ^
        teensy_bridge_last_seq[0] ^
        teensy_bridge_heartbeat_seen ^
        teensy_bridge_heartbeat_seq[0] ^
        teensy_bridge_heartbeat_age_ms[0] ^
        teensy_bridge_checksum_fault_count[0] ^
        teensy_bridge_unsupported_count[0] ^
        teensy_uart_framing_error_count[0];

    // SW2+SW6 enables a diagnostic snapshot view. The physical snapshot banks
    // remain owned by rocket_i2c_suite_top; only evidence/visualization readers
    // see the deterministic injected view.
    wire [31:0] bmp_diag_t_us;
    wire [15:0] bmp_diag_seq;
    wire        bmp_diag_valid;
    wire [7:0]  bmp_diag_status;
    wire [47:0] bmp_diag_payload;
    wire [15:0] bmp_diag_age_ms;
    wire        bmp_diag_injected;

    wire [31:0] acc_diag_t_us;
    wire [15:0] acc_diag_seq;
    wire        acc_diag_valid;
    wire [7:0]  acc_diag_status;
    wire [47:0] acc_diag_payload;
    wire [15:0] acc_diag_age_ms;
    wire        acc_diag_injected;

    wire [31:0] mag_diag_t_us;
    wire [15:0] mag_diag_seq;
    wire        mag_diag_valid;
    wire [7:0]  mag_diag_status;
    wire [47:0] mag_diag_payload;
    wire [15:0] mag_diag_age_ms;
    wire        mag_diag_injected;

    wire [31:0] pwr_diag_t_us;
    wire [15:0] pwr_diag_seq;
    wire        pwr_diag_valid;
    wire [7:0]  pwr_diag_status;
    wire [47:0] pwr_diag_payload;
    wire [15:0] pwr_diag_age_ms;
    wire        pwr_diag_injected;

    snapshot_fault_injector #(
        .INVALID_PAYLOAD      (48'hBAD0_BA00_0000),
        .OUT_OF_RANGE_PAYLOAD (48'hFFFF_FFFF_FFFF)
    ) u_bmp_fault_injector (
        .clk         (clk),
        .rst         (rst),
        .enable      (diag_fault_inject_sys && (diag_fault_bank_sys == `DIAG_BANK_BMP)),
        .fault_class (diag_fault_class_sys),
        .in_t_us     (bmp_t_us),
        .in_seq      (bmp_seq),
        .in_valid    (bmp_valid),
        .in_status   (bmp_status),
        .in_payload  (bmp_payload),
        .in_age_ms   (der_bmp_age_ms),
        .out_t_us    (bmp_diag_t_us),
        .out_seq     (bmp_diag_seq),
        .out_valid   (bmp_diag_valid),
        .out_status  (bmp_diag_status),
        .out_payload (bmp_diag_payload),
        .out_age_ms  (bmp_diag_age_ms),
        .injected    (bmp_diag_injected)
    );

    snapshot_fault_injector #(
        .INVALID_PAYLOAD      (48'hBAD0_AC00_0000),
        .OUT_OF_RANGE_PAYLOAD ({16'h7FFF, 16'h8000, 16'h7FFF})
    ) u_acc_fault_injector (
        .clk         (clk),
        .rst         (rst),
        .enable      (diag_fault_inject_sys && (diag_fault_bank_sys == `DIAG_BANK_ACC)),
        .fault_class (diag_fault_class_sys),
        .in_t_us     (acc_t_us),
        .in_seq      (acc_seq),
        .in_valid    (acc_valid),
        .in_status   (acc_status),
        .in_payload  (acc_payload),
        .in_age_ms   (der_acc_age_ms),
        .out_t_us    (acc_diag_t_us),
        .out_seq     (acc_diag_seq),
        .out_valid   (acc_diag_valid),
        .out_status  (acc_diag_status),
        .out_payload (acc_diag_payload),
        .out_age_ms  (acc_diag_age_ms),
        .injected    (acc_diag_injected)
    );

    snapshot_fault_injector #(
        .INVALID_PAYLOAD      (48'hBAD0_AA00_0000),
        .OUT_OF_RANGE_PAYLOAD ({16'h7FFF, 16'h8000, 16'h7FFF})
    ) u_mag_fault_injector (
        .clk         (clk),
        .rst         (rst),
        .enable      (diag_fault_inject_sys && (diag_fault_bank_sys == `DIAG_BANK_MAG)),
        .fault_class (diag_fault_class_sys),
        .in_t_us     (mag_t_us),
        .in_seq      (mag_seq),
        .in_valid    (mag_valid),
        .in_status   (mag_status),
        .in_payload  (mag_payload),
        .in_age_ms   (der_mag_age_ms),
        .out_t_us    (mag_diag_t_us),
        .out_seq     (mag_diag_seq),
        .out_valid   (mag_diag_valid),
        .out_status  (mag_diag_status),
        .out_payload (mag_diag_payload),
        .out_age_ms  (mag_diag_age_ms),
        .injected    (mag_diag_injected)
    );

    snapshot_fault_injector #(
        .INVALID_PAYLOAD      (48'hBAD0_0A00_0000),
        .OUT_OF_RANGE_PAYLOAD ({8'hFF, 12'hFFF, 12'hFFF, 16'hFFFF})
    ) u_pwr_fault_injector (
        .clk         (clk),
        .rst         (rst),
        .enable      (diag_fault_inject_sys && (diag_fault_bank_sys == `DIAG_BANK_PWR)),
        .fault_class (diag_fault_class_sys),
        .in_t_us     (pwr_t_us),
        .in_seq      (pwr_seq),
        .in_valid    (pwr_valid),
        .in_status   (pwr_status),
        .in_payload  (pwr_payload),
        .in_age_ms   (pwr_age_ms),
        .out_t_us    (pwr_diag_t_us),
        .out_seq     (pwr_diag_seq),
        .out_valid   (pwr_diag_valid),
        .out_status  (pwr_diag_status),
        .out_payload (pwr_diag_payload),
        .out_age_ms  (pwr_diag_age_ms),
        .injected    (pwr_diag_injected)
    );

    mag1_bench_snapshot_source #(
        .MAG1_BENCH_OFFSET_X (MAG1_BENCH_OFFSET_X),
        .MAG1_BENCH_OFFSET_Y (MAG1_BENCH_OFFSET_Y),
        .MAG1_BENCH_OFFSET_Z (MAG1_BENCH_OFFSET_Z),
        .MAG_FRESH_MAX_MS    (16'd200),
        .MAG1_SEQUENCE_OFFSET(16'd0)
    ) u_mag1_bench_snapshot_source (
        .clk                 (clk),
        .rst                 (rst),
        .enable              ((USE_MAG1_BENCH_SOURCE != 0) && sw_mag1_bench_level_sys),
        .cfg_offset_x_en     (sw_mag1_offset_x_level_sys),
        .cfg_offset_y_en     (sw_mag1_offset_y_level_sys),
        .cfg_offset_z_en     (sw_mag1_offset_z_level_sys),

        .mag0_t_us           (mag_diag_t_us),
        .mag0_seq            (mag_diag_seq),
        .mag0_valid          (mag_diag_valid),
        .mag0_status         (mag_diag_status),
        .mag0_payload        (mag_diag_payload),
        .mag0_age_ms         (mag_diag_age_ms),

        .mag1_t_us           (mag1_bench_t_us),
        .mag1_seq            (mag1_bench_seq),
        .mag1_valid          (mag1_bench_valid),
        .mag1_status         (mag1_bench_status),
        .mag1_payload        (mag1_bench_payload),
        .mag1_age_ms         (mag1_bench_age_ms),
        .mag1_cal_state      (mag1_bench_cal_state),
        .mag1_source_flags   (mag1_bench_source_flags),
        .mag1_bridge_checksum(mag1_bench_bridge_checksum)
    );

    wire mag1_phys_present_w =
        mag1_phys_valid || (mag1_phys_status != `ST_NOT_INITIALIZED);

    assign mag1_t_us = mag1_phys_present_w ? mag1_phys_t_us : mag1_bench_t_us;
    assign mag1_seq = mag1_phys_present_w ? mag1_phys_seq : mag1_bench_seq;
    assign mag1_valid = mag1_phys_present_w ? mag1_phys_valid : mag1_bench_valid;
    assign mag1_status = mag1_phys_present_w ? mag1_phys_status : mag1_bench_status;
    assign mag1_payload = mag1_phys_present_w ? mag1_phys_payload : mag1_bench_payload;
    assign mag1_age_ms = mag1_phys_present_w ? mag1_phys_age_ms : mag1_bench_age_ms;
    assign mag1_cal_state = mag1_phys_present_w ? mag1_phys_cal_state : mag1_bench_cal_state;
    assign mag1_source_flags = mag1_phys_present_w ? mag1_phys_source_flags : mag1_bench_source_flags;
    assign mag1_bridge_checksum =
        mag1_phys_present_w ? mag1_phys_bridge_checksum : mag1_bench_bridge_checksum;

    sensor_extension_hub #(
        .PAYLOAD_W           (48),
        .ENABLE_BLACKBOX_LOG (USE_BLACKBOX_LOG)
    ) u_sensor_extension_hub (
        .clk                 (clk),
        .rst                 (rst),

        .bmp_t_us            (bmp_diag_t_us),
        .bmp_seq             (bmp_diag_seq),
        .bmp_valid           (bmp_diag_valid),
        .bmp_status          (bmp_diag_status),
        .bmp_payload         (bmp_diag_payload),
        .bmp_age_ms          (bmp_diag_age_ms),

        .acc_t_us            (acc_diag_t_us),
        .acc_seq             (acc_diag_seq),
        .acc_valid           (acc_diag_valid),
        .acc_status          (acc_diag_status),
        .acc_payload         (acc_diag_payload),
        .acc_age_ms          (acc_diag_age_ms),

        .mag_t_us            (mag_diag_t_us),
        .mag_seq             (mag_diag_seq),
        .mag_valid           (mag_diag_valid),
        .mag_status          (mag_diag_status),
        .mag_payload         (mag_diag_payload),
        .mag_age_ms          (mag_diag_age_ms),

        .pwr_t_us            (pwr_diag_t_us),
        .pwr_seq             (pwr_diag_seq),
        .pwr_valid           (pwr_diag_valid),
        .pwr_status          (pwr_diag_status),
        .pwr_payload         (pwr_diag_payload),
        .pwr_age_ms          (pwr_diag_age_ms),

        .mag1_t_us           (mag1_t_us),
        .mag1_seq            (mag1_seq),
        .mag1_valid          (mag1_valid),
        .mag1_status         (mag1_status),
        .mag1_payload        (mag1_payload),
        .mag1_age_ms         (mag1_age_ms),
        .mag1_cal_state      (mag1_cal_state),
        .mag1_source_flags   (mag1_source_flags),
        .mag1_bridge_checksum(mag1_bridge_checksum),

        .rng_t_us            (rng_bridge_t_us),
        .rng_seq             (rng_bridge_seq),
        .rng_valid           (rng_bridge_valid),
        .rng_status          (rng_bridge_status),
        .rng_payload         (rng_bridge_payload),
        .rng_age_ms          (rng_bridge_age_ms),

        .air_t_us            (32'd0),
        .air_seq             (16'd0),
        .air_valid           (1'b0),
        .air_status          (8'h01),
        .air_payload         (48'd0),
        .air_age_ms          (16'hFFFF),

        .env_t_us            (env_t_us),
        .env_seq             (env_seq),
        .env_valid           (env_valid),
        .env_status          (env_status),
        .env_payload         (env_payload),
        .env_age_ms          (env_age_ms),

        .sun_t_us            (sun_t_us),
        .sun_seq             (sun_seq),
        .sun_valid           (sun_valid),
        .sun_status          (sun_status),
        .sun_payload         (sun_payload),
        .sun_age_ms          (sun_age_ms),

        .flow_t_us           (32'd0),
        .flow_seq            (16'd0),
        .flow_valid          (1'b0),
        .flow_status         (8'h01),
        .flow_payload        (48'd0),
        .flow_age_ms         (16'hFFFF),

        .diag_selftest_enable(sw_selftest_level_sys),
        .diag_fault_inject_enable(diag_fault_inject_sys),
        .diag_fault_mode     (diag_fault_mode_sys),

        .log_runtime_enable  (sw_log_diag_level_sys),
        .log_emit_req        (log_emit_req_sys),
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
        ext_log_stream_valid ^ ext_log_stream_word[0] ^ ext_log_stream_last ^
        bmp_diag_injected ^ acc_diag_injected ^ mag_diag_injected ^
        pwr_diag_injected ^
        gyro_t_us[0] ^ gyro_seq[0] ^ gyro_valid ^ gyro_status[0] ^
        gyro_payload[0] ^ gyro_age_ms[0] ^
        ls1_s_level_sys[0] ^ pir_motion_level_sys;

    //==========================================================================
    // Authority phase / gate producer
    //==========================================================================
    authority_gate_phase_sys u_authority_gate_phase (
        .clk                    (clk),
        .rst                    (rst),
        .sw_arm_raw             (sw_arm_raw),
        .sw_policy_enable_raw   (sw_policy_enable_raw),

        .bmp_valid              (bmp_diag_valid),
        .bmp_status             (bmp_diag_status),
        .bmp_age_ms             (bmp_diag_age_ms),
        .acc_valid              (acc_diag_valid),
        .acc_status             (acc_diag_status),
        .acc_age_ms             (acc_diag_age_ms),
        .mag_valid              (mag_diag_valid),
        .mag_status             (mag_diag_status),
        .mag_age_ms             (mag_diag_age_ms),

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

    //==========================================================================
    // VGA render-control wiring
    //==========================================================================
    wire legacy_compass_page_hold_sys;
    wire legacy_selftest_hold_sys;
    assign legacy_compass_page_hold_sys =
        sw_compass_page_level_sys | sw_compass_default_level_sys;
    assign legacy_selftest_hold_sys = sw_selftest_level_sys & !diag_fault_inject_sys;

    wire [2:0] render_view_sel_sys;
    wire [2:0] render_view_effective_sys;
    wire       render_view_changed_pulse_sys;
    wire       render_cfg_invalid_view_sys;
    wire       render_flight_selftest_en_sys;
    wire       render_compass_page_enable_sys;

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

    // Until real EKF/GNSS/wind-estimator signals exist in SYS domain, keep the
    // nav/wind viewport explicitly unbound. The compatibility shim reports
    // ST_MISSING_INPUT rather than deriving display proxies from altitude,
    // airspeed, or optical-flow evidence. Bind nav_wind_snapshot_producer here
    // only after those real source contracts are present.
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
    // Unified VGA renderer / compositor
    //==========================================================================
    caelumfusion_vga_render_control #(
        .SYS_CLK_HZ                 (SYS_CLK_HZ),
        .H_ACTIVE                   (H_ACTIVE),
        .H_FP                       (H_FP),
        .H_SYNC                     (H_SYNC),
        .H_BP                       (H_BP),
        .V_ACTIVE                   (V_ACTIVE),
        .V_FP                       (V_FP),
        .V_SYNC                     (V_SYNC),
        .V_BP                       (V_BP),
        .HSYNC_POL                  (HSYNC_POL),
        .VSYNC_POL                  (VSYNC_POL),
        .MAG_PLOT_SHIFT             (8),
        .ENABLE_SENSOR_DIAG_PAGE    (USE_SENSOR_DIAG_PAGE),
        .ENABLE_COMPASS_TRUTH_PAGE  (USE_COMPASS_TRUTH_PAGE),
        .ENABLE_SCIENCE_PAGES       (USE_SCIENCE_PAGES),
        .ENABLE_TELEMETRY_TEXT_OVERLAY(USE_TELEMETRY_TEXT_OVERLAY),
        .ENABLE_RENDER_STATUS_STRIP (1),
        .COMPASS_TRUTH_PAGE_DEFAULT (COMPASS_TRUTH_PAGE_DEFAULT)
    ) u_vga_render_control (
        .sys_clk                    (clk),
        .sys_rst                    (rst),

        .view_next_pulse_sys        (btn_next_rise_sys),
        .view_prev_pulse_sys        (btn_prev_rise_sys),
        .view_direct_valid_sys      (render_direct_valid_sys),
        .view_direct_id_sys         (render_direct_view_id_sys),
        .cfg_invalid_view_clear_sys (1'b0),
        .legacy_compass_page_hold_sys(legacy_compass_page_hold_sys),
        .legacy_selftest_hold_sys   (legacy_selftest_hold_sys),
        .history_freeze_sys         (sw_history_freeze_level_sys),
        .direct_selector_collision_sys(render_direct_selector_collision_sys),

        .view_sel_sys               (render_view_sel_sys),
        .view_effective_sys         (render_view_effective_sys),
        .view_changed_pulse_sys     (render_view_changed_pulse_sys),
        .cfg_invalid_view_sys       (render_cfg_invalid_view_sys),
        .flight_selftest_en_sys     (render_flight_selftest_en_sys),
        .compass_page_enable_sys    (render_compass_page_enable_sys),

        .bmp_t_us                   (bmp_diag_t_us),
        .bmp_seq                    (bmp_diag_seq),
        .bmp_valid                  (bmp_diag_valid),
        .bmp_status                 (bmp_diag_status),
        .bmp_payload                (bmp_diag_payload),
        .bmp_age_ms                 (bmp_diag_age_ms),

        .acc_t_us                   (acc_diag_t_us),
        .acc_seq                    (acc_diag_seq),
        .acc_valid                  (acc_diag_valid),
        .acc_status                 (acc_diag_status),
        .acc_payload                (acc_diag_payload),
        .acc_age_ms                 (acc_diag_age_ms),

        .mag_t_us                   (mag_diag_t_us),
        .mag_seq                    (mag_diag_seq),
        .mag_valid                  (mag_diag_valid),
        .mag_status                 (mag_diag_status),
        .mag_payload                (mag_diag_payload),
        .mag_age_ms                 (mag_diag_age_ms),
        .mag1_seq                   (mag1_seq),
        .mag1_valid                 (mag1_valid),
        .mag1_status                (mag1_status),
        .mag1_payload               (mag1_payload),
        .mag1_age_ms                (mag1_age_ms),

        .pwr_t_us                   (pwr_diag_t_us),
        .pwr_seq                    (pwr_diag_seq),
        .pwr_valid                  (pwr_diag_valid),
        .pwr_status                 (pwr_diag_status),
        .pwr_payload                (pwr_diag_payload),
        .pwr_age_ms                 (pwr_diag_age_ms),

        .ext_valid                  (ext_valid),
        .ext_status                 (ext_status),
        .ext_present_flags          (ext_present_flags),
        .ext_fault_flags            (ext_fault_flags),
        .ext_mag_delta_l1           (ext_mag_delta_l1),
        .ext_mag_norm_primary       (ext_mag_norm_primary),
        .ext_mag_norm_secondary     (ext_mag_norm_secondary),
        .ext_mag_sequence_aligned   (ext_mag_sequence_aligned),
        .ext_mag_disagreement       (ext_mag_disagreement),
        .ext_mag_sector_delta       (ext_mag_sector_delta),
        .ext_mag_norm_delta_l1      (ext_mag_norm_delta_l1),
        .ext_mag_iron_residual      (ext_mag_iron_residual),
        .ext_mag_cal_state          (ext_mag_cal_state),
        .ext_mag_source_flags       (ext_mag_source_flags),
        .ext_mag_bridge_checksum    (ext_mag_bridge_checksum),
        .ext_rng_height_cm          (ext_rng_height_cm),
        .ext_air_dp_pa              (ext_air_dp_pa),
        .ext_air_speed_cms          (ext_air_speed_cms),
        .ext_env_temp_cdeg          (ext_env_temp_cdeg),
        .ext_env_rh_centi           (ext_env_rh_centi),
        .ext_sun_luma               (ext_sun_luma),
        .ext_flow_dx                (ext_flow_dx),
        .ext_flow_dy                (ext_flow_dy),
        .ext_log_seq                (ext_log_seq),
        .ext_log_drop_count         (ext_log_drop_count),
        .ext_max_age_ms             (ext_max_age_ms),

        .der_valid                  (der_valid),
        .der_status                 (der_status),
        .der_alt_fresh              (der_alt_fresh),
        .der_vspd_fresh             (der_vspd_fresh),
        .der_roll_fresh             (der_roll_fresh),
        .der_head_fresh             (der_head_fresh),
        .der_bmp_seq_ref            (der_bmp_seq_ref),
        .der_acc_seq_ref            (der_acc_seq_ref),
        .der_mag_seq_ref            (der_mag_seq_ref),
        .der_bmp_age_ms             (der_bmp_age_ms),
        .der_acc_age_ms             (der_acc_age_ms),
        .der_mag_age_ms             (der_mag_age_ms),
        .der_bmp_valid_ref          (der_bmp_valid_ref),
        .der_acc_valid_ref          (der_acc_valid_ref),
        .der_mag_valid_ref          (der_mag_valid_ref),
        .der_altitude_cm            (der_altitude_cm),
        .der_vertical_speed_cms     (der_vertical_speed_cms),
        .der_roll_mdeg              (der_roll_mdeg),
        .der_heading_mdeg           (der_heading_mdeg),

        .nav_valid                  (landing_nav_valid),
        .nav_status                 (landing_nav_status),
        .nav_flags                  (landing_nav_flags),
        .nav_downrange_m            (landing_nav_downrange_m),
        .nav_crossrange_m           (landing_nav_crossrange_m),
        .nav_age_ms                 (landing_nav_age_ms),

        .wind_valid                 (landing_wind_valid),
        .wind_status                (landing_wind_status),
        .wind_x_cms                 (landing_wind_x_cms),
        .wind_y_cms                 (landing_wind_y_cms),
        .wind_z_cms                 (landing_wind_z_cms),
        .wind_age_ms                (landing_wind_age_ms),

        .auth_phase_code_sys        (auth_phase_code_sys),
        .auth_phase_valid_sys       (auth_phase_valid_sys),
        .safety_runtime_ok_sys      (safety_runtime_ok_sys),
        .safety_allows_actuation_sys(safety_allows_actuation_sys),
        .policy_runtime_enable_sys  (policy_runtime_enable_sys),
        .software_armed_sys         (software_armed_sys),
        .i2c_nack_count             (bus_error_count),
        .i2c_timeout_count          (timeout_count),
        .txn_rate_hz                (txn_rate_hz),
        .cdc_update_count_sys       (cdc_update_count),
        .build_id                   (build_id),
        .schema_word                (schema_word),

        .pix_clk                    (pix_clk),
        .pix_rst                    (pix_rst),
        .vga_hsync                  (vga_hsync),
        .vga_vsync                  (vga_vsync),
        .vga_rgb                    (vga_rgb)
    );

    wire _unused_render_control_status_ok;
    assign _unused_render_control_status_ok =
        render_view_sel_sys[0] ^ render_view_effective_sys[0] ^
        render_view_changed_pulse_sys ^ render_cfg_invalid_view_sys ^
        render_flight_selftest_en_sys ^ render_compass_page_enable_sys ^
        render_direct_selector_collision_sys;

    assign cls_tx = 1'b1;

endmodule

//==============================================================================
// caelumfusion_button_debounce_pulse_sys
//------------------------------------------------------------------------------
// Synchronizes and lightly debounces a human-operated pushbutton into clk.
// This is intentionally local to the board-facing top so source-set membership
// stays stable and no multi-bit CDC contract is implied.
//==============================================================================
module caelumfusion_button_debounce_pulse_sys #(
    parameter integer STABLE_CYCLES = 250000,
    parameter integer COUNT_W       = 19
)(
    input  wire clk,
    input  wire rst,
    input  wire async_in,

    output reg  sync_level,
    output wire rise_pulse,
    output wire fall_pulse,
    output wire toggle_pulse
);
    localparam [COUNT_W-1:0] STABLE_MAX = STABLE_CYCLES;

    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg [2:0] sync_ff;
    reg candidate_level;
    reg [COUNT_W-1:0] stable_count;
    reg sync_level_q;

    always @(posedge clk) begin
        if (rst) begin
            sync_ff         <= 3'b000;
            candidate_level <= 1'b0;
            stable_count    <= {COUNT_W{1'b0}};
            sync_level      <= 1'b0;
            sync_level_q    <= 1'b0;
        end else begin
            sync_ff <= {sync_ff[1:0], async_in};
            sync_level_q <= sync_level;

            if (sync_ff[2] != candidate_level) begin
                candidate_level <= sync_ff[2];
                stable_count    <= {COUNT_W{1'b0}};
            end else if (stable_count != STABLE_MAX) begin
                stable_count <= stable_count + {{(COUNT_W-1){1'b0}}, 1'b1};
            end else begin
                sync_level <= candidate_level;
            end
        end
    end

    assign rise_pulse   =  sync_level & ~sync_level_q;
    assign fall_pulse   = ~sync_level &  sync_level_q;
    assign toggle_pulse =  sync_level ^  sync_level_q;
endmodule

`default_nettype wire
