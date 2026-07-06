`timescale 1ns/1ps
`default_nettype none

`include "telemetry_defs_vh.vh"

//==============================================================================
// rocket_i2c_suite_top
//------------------------------------------------------------------------------
// ROLE
//   Canonical SYS-domain sensor acquisition suite for the CaelumFusion flight
//   control / visualization stack.ssure sensor on I2C
//     4) MMC3416xPJ / Pmod CMPS2 magnetometer on I2C
//
//
// PURPOSE
//   Own the deterministic shared-I2C acquisition path for:
//
//     1) LIS3DH auxiliary accelerometer on the shared I2C engine
//     2) ADXL362 / Pmod ACL2 accelerometer on a dedicated mode-0 SPI engine
//     3) BMP585 / BMP58x-class pre
//   and publish:
//
//     A) raw committed snapshot banks
//     B) a derived-state publication bank
//     C) transaction / health / metadata observability
//
// ARCHITECTURE
//   The suite is organized into five layers:
//
//     Layer 1: timebase + epoch scheduler
//     Layer 2: cadence intent / fixed-priority arbitration
//     Layer 3: per-sensor job FSMs
//     Layer 4: shared engine + shared bus ownership
//     Layer 5: snapshot and derived publication
//
// KEY DESIGN INVARIANTS
//   - SYS-domain only. No pixel-domain logic appears here.
//   - One shared I2C engine owns scl/sda drive behavior.
//   - Jobs never drive the bus directly.
//   - Only committed snapshots are published.
//   - Snapshot banks are single-writer structures.
//   - Derived publication is computed from committed snapshots only.
//   - All counters and observability fields have a single owning always block.
//   - No dynamic scheduling, no queues, no unbounded loops.
//
// CADENCE POLICY
//   - LIS3DH  : 100 Hz when USE_LIS3DH_I2C_ACC != 0
//   - ADXL362 : 100 Hz polling or synchronized INT1/INT2 edge by parameter
//                when USE_ADXL362_SPI_ACC != 0
//   - BMP585  :  50 Hz
//   - MMC3416 :  10 Hz when USE_CMPS2_MMC3416_MAG != 0
//   - PMON1   :  10 Hz when USE_PMON1_PWR != 0
//   - HYGRO   :   1 Hz when USE_HYGRO_ENV != 0
//   - GYRO    :  10 Hz when USE_GYRO_I2C != 0
//   - LIS2MDL :  10 Hz when USE_LIS2MDL_MAG1 != 0
//   - ADXL362 init maps DATA_READY to active-high INT1 before measurement mode.
//
// ARBITRATION POLICY
//   Fixed priority:
//     I2C auxiliary accelerometer slot > BMP585 > primary magnetometer slot >
//     PMON1 > HYGRO > GYRO > redundant LIS2MDL/MAG1 slot
//
// ACCELEROMETER PUBLICATION POLICY
//   The external acc_* snapshot bank is selected by parameters:
//     USE_LIS3DH_I2C_ACC=1, USE_ADXL362_SPI_ACC=0 -> LIS3DH only
//     USE_LIS3DH_I2C_ACC=0, USE_ADXL362_SPI_ACC=1 -> ADXL362 only
//     USE_LIS3DH_I2C_ACC=1, USE_ADXL362_SPI_ACC=1 -> ADXL362 priority,
//                                                       LIS3DH fallback
//
// MAGNETOMETER / HEADING POLICY
//   Pmod CMPS2 / MMC34160PJ is the active magnetic-field source in the I2C
//   build. It is locked to the Digilent module's 7-bit I2C address 7'h30 and
//   publishes mag_* snapshots that feed der_heading_mdeg through the
//   derived-state attitude math. The legacy LIS2MDL-named arbiter/mux signal
//   names are intentionally preserved for interface stability; they now select
//   the CMPS2/MMC34160PJ job in this build.
//
//   LIS3DH is not a magnetometer and is used only as auxiliary acceleration,
//   redundancy, vibration, and shock evidence.
//
// NOTES
//   - This top is intentionally integration-complete even if leaf job modules
//     continue to evolve.
//   - The raw payload buses remain 48 bits to preserve compatibility with the
//     project's existing snapshot contract.
//   - The derived-state publisher below is intentionally conservative and uses
//     placeholder-but-structured decoding rules that can later be replaced by
//     authoritative physical decode modules without changing the suite-level
//     interface.
//==============================================================================
module rocket_i2c_suite_top #(
    parameter integer CLK_HZ                  = 100_000_000,

    //--------------------------------------------------------------------------
    // Snapshot payload width
    //--------------------------------------------------------------------------
    parameter integer SNAP_PAYLOAD_W          = 48,

    //--------------------------------------------------------------------------
    // Scheduler cadences
    //--------------------------------------------------------------------------
    parameter integer RATE_100HZ_US           = 10_000,
    parameter integer RATE_50HZ_US            = 20_000,
    parameter integer RATE_10HZ_US            = 100_000,

    //--------------------------------------------------------------------------
    // Freshness thresholds used by the derived publisher
    //--------------------------------------------------------------------------
    parameter integer BMP_FRESH_MAX_MS        = 150,
    parameter integer ACC_FRESH_MAX_MS        = 80,
    parameter integer MAG_FRESH_MAX_MS        = 250,

    //--------------------------------------------------------------------------
    // Build / schema ownership
    //--------------------------------------------------------------------------
    parameter [31:0] BUILD_ID                 = 32'h0000_0001,
    parameter [15:0] SCHEMA_WORD              = 16'hCF10,

    // ADXL362 interrupt trigger selection:
    //   0: keep deterministic 100 Hz polling
    //   1: sample on synchronized INT1 rising edge
    //   2: sample on synchronized INT1 falling edge
    //   3: sample on synchronized INT2 rising edge
    //   4: sample on synchronized INT2 falling edge
    //
    // Board integration selects 1 for Digilent ACL2 INT1 data-ready. The suite
    // default remains polling for benches or integrations that do not wire INTx.
    parameter integer ADXL_IRQ_POLICY         = 1,

    // Accelerometer-source integration controls. Bring-up defaults prove the
    // shared-I2C LIS3DH path alone. Set both to 1 after hardware bring-up for
    // ADXL362-priority redundancy/fallback mode.
    parameter integer USE_LIS3DH_I2C_ACC    = 1,
    parameter integer USE_ADXL362_SPI_ACC   = 1,

    // Pmod CMPS2 / MMC34160PJ magnetometer source for mag_* and heading.
    // Keep enabled for the Basys-3 shared-I2C build. The module name and all
    // existing downstream signal names are preserved.
    parameter integer USE_CMPS2_MMC3416_MAG = 1,

    // Pmod PMON1 / ADM1191 power monitor source. Disabled by default so boards
    // without PMON1 do not add bus traffic or error counts. Do not use 7'h30
    // while CMPS2/MMC34160PJ is present on the same bus.
    parameter integer USE_PMON1_PWR         = 1,
    parameter [6:0]   PMON1_ADDR7           = 7'h38,

    // Optional shared-I2C extension devices. Runtime cfg_ext_i2c_en gates these
    // paths as a group so a board can boot with SW15 low before all optional
    // devices are physically attached and address-verified.
    parameter integer USE_HYGRO_ENV         = 0,
    parameter integer USE_GYRO_I2C          = 0,
    // Redundant physical MAG1 is a deliberate hardware-validation path. Keep
    // SW15 low until the LIS2MDL wiring, pullups, and address are known-good.
    parameter integer USE_LIS2MDL_MAG1      = 1
)(
    input  wire                      clk,
    input  wire                      rst,

    // Runtime path enables. These switches do not create hardware that was
    // excluded by the USE_* parameters; they only gate already-elaborated paths.
    input  wire                      cfg_lis3dh_i2c_acc_en,
    input  wire                      cfg_adxl362_spi_acc_en,
    input  wire                      cfg_cmps2_mmc3416_mag_en,
    input  wire                      cfg_pmon1_pwr_en,
    input  wire                      cfg_ext_i2c_en,

    //--------------------------------------------------------------------------
    // Shared physical I2C bus
    //--------------------------------------------------------------------------
    output wire                      scl,
    inout  wire                      sda,

    //--------------------------------------------------------------------------
    // Dedicated ADXL362 / Pmod ACL2 SPI pins
    //--------------------------------------------------------------------------
    output wire                      adxl362_cs_n,
    output wire                      adxl362_mosi,
    input  wire                      adxl362_miso,
    output wire                      adxl362_sclk,
    input  wire                      adxl362_int1,
    input  wire                      adxl362_int2,

    //--------------------------------------------------------------------------
    // Raw published snapshots
    //--------------------------------------------------------------------------
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

    output wire [31:0]               pwr_t_us,
    output wire [15:0]               pwr_seq,
    output wire                      pwr_valid,
    output wire [7:0]                pwr_status,
    output wire [SNAP_PAYLOAD_W-1:0] pwr_payload,
    output wire [15:0]               pwr_age_ms,

    output wire [31:0]               env_t_us,
    output wire [15:0]               env_seq,
    output wire                      env_valid,
    output wire [7:0]                env_status,
    output wire [SNAP_PAYLOAD_W-1:0] env_payload,
    output wire [15:0]               env_age_ms,

    output wire [31:0]               mag1_t_us,
    output wire [15:0]               mag1_seq,
    output wire                      mag1_valid,
    output wire [7:0]                mag1_status,
    output wire [SNAP_PAYLOAD_W-1:0] mag1_payload,
    output wire [15:0]               mag1_age_ms,
    output wire [7:0]                mag1_cal_state,
    output wire [7:0]                mag1_source_flags,
    output wire [15:0]               mag1_bridge_checksum,

    output wire [31:0]               gyro_t_us,
    output wire [15:0]               gyro_seq,
    output wire                      gyro_valid,
    output wire [7:0]                gyro_status,
    output wire [SNAP_PAYLOAD_W-1:0] gyro_payload,
    output wire [15:0]               gyro_age_ms,

    //--------------------------------------------------------------------------
    // Derived-state publication bank
    //--------------------------------------------------------------------------
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

    //--------------------------------------------------------------------------
    // Observability / metadata
    //--------------------------------------------------------------------------
    output wire [15:0]               der_i2c_nack_count,
    output wire [15:0]               der_i2c_timeout_count,
    output wire [15:0]               der_txn_rate_hz,
    output wire [31:0]               der_cdc_update_count,
    output wire [31:0]               der_frame_count,
    output wire [31:0]               der_build_id,
    output wire [15:0]               der_schema_word
);

    //==========================================================================
    // 1) Timebase
    //==========================================================================
    wire [31:0] time_us;
    wire        tick_1us;

    timebase_us #(
        .CLK_HZ(CLK_HZ)
    ) u_timebase_us (
        .clk      (clk),
        .rst      (rst),
        .time_us  (time_us),
        .tick_1us (tick_1us)
    );

    //==========================================================================
    // 2) Epoch scheduler
    //==========================================================================
    wire epoch_100hz;
    wire epoch_50hz;
    wire epoch_10hz;

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

    reg [3:0] ext_10hz_div_r;
    reg       epoch_1hz;

    always @(posedge clk) begin
        if (rst) begin
            ext_10hz_div_r <= 4'd0;
            epoch_1hz      <= 1'b0;
        end else begin
            epoch_1hz <= 1'b0;
            if (epoch_10hz) begin
                if (ext_10hz_div_r == 4'd9) begin
                    ext_10hz_div_r <= 4'd0;
                    epoch_1hz      <= 1'b1;
                end else begin
                    ext_10hz_div_r <= ext_10hz_div_r + 4'd1;
                end
            end
        end
    end

    //==========================================================================
    // 3) Build-time + runtime sensor path enables
    //--------------------------------------------------------------------------
    // Keep disabled optional paths held in reset so their command streams
    // remain quiescent. CMPS2/MMC34160PJ stays enabled by default because it is
    // the board's magnetometer and derived-heading source. Runtime enables
    // mask only paths that are already present in the bitstream.
    //==========================================================================
    wire lis3dh_path_en;
    wire adxl_path_en;
    wire cmps2_mag_path_en;
    wire pmon1_path_en;
    wire hygro_path_en;
    wire gyro_path_en;
    wire lis2mdl_mag1_path_en;
    wire lis3dh_path_rst;
    wire adxl_path_rst;
    wire cmps2_mag_path_rst;
    wire pmon1_path_rst;
    wire hygro_path_rst;
    wire gyro_path_rst;
    wire lis2mdl_mag1_path_rst;

    assign lis3dh_path_en     = ((USE_LIS3DH_I2C_ACC    != 0) && cfg_lis3dh_i2c_acc_en)    ? 1'b1 : 1'b0;
    assign adxl_path_en       = ((USE_ADXL362_SPI_ACC   != 0) && cfg_adxl362_spi_acc_en)   ? 1'b1 : 1'b0;
    assign cmps2_mag_path_en  = ((USE_CMPS2_MMC3416_MAG != 0) && cfg_cmps2_mmc3416_mag_en) ? 1'b1 : 1'b0;
    assign pmon1_path_en      = ((USE_PMON1_PWR         != 0) && cfg_pmon1_pwr_en)         ? 1'b1 : 1'b0;
    assign hygro_path_en      = ((USE_HYGRO_ENV         != 0) && cfg_ext_i2c_en)            ? 1'b1 : 1'b0;
    assign gyro_path_en       = ((USE_GYRO_I2C          != 0) && cfg_ext_i2c_en)            ? 1'b1 : 1'b0;
    assign lis2mdl_mag1_path_en =
        ((USE_LIS2MDL_MAG1    != 0) && cfg_ext_i2c_en)                                     ? 1'b1 : 1'b0;
    assign lis3dh_path_rst    = rst | ~lis3dh_path_en;
    assign adxl_path_rst      = rst | ~adxl_path_en;
    assign cmps2_mag_path_rst = rst | ~cmps2_mag_path_en;
    assign pmon1_path_rst     = rst | ~pmon1_path_en;
    assign hygro_path_rst     = rst | ~hygro_path_en;
    assign gyro_path_rst      = rst | ~gyro_path_en;
    assign lis2mdl_mag1_path_rst = rst | ~lis2mdl_mag1_path_en;

    //==========================================================================
    // 4) Cadence intent
    //--------------------------------------------------------------------------
    // Epoch pulses are one-cycle events. Because 50 Hz and 10 Hz epochs can
    // coincide with the 100 Hz LIS3DH epoch, requests are latched until the
    // fixed-priority arbiter grants the corresponding job. Multi-transaction
    // jobs also keep their intent asserted while their internal FSM is active so
    // each follow-on I2C transaction can reacquire the shared engine.
    //==========================================================================
    wire want_lis3dh;
    wire want_bmp585;
    wire want_lis2mdl;
    wire want_pmon1;
    wire want_hygro;
    wire want_gyro;
    wire want_lis2mdl_mag1;

    wire grant_lis3dh;
    wire grant_bmp585;
    wire grant_lis2mdl;
    wire grant_pmon1;
    wire grant_hygro;
    wire grant_gyro;
    wire grant_lis2mdl_mag1;

    wire j0_mux_busy;
    wire j1_job_busy;
    wire j2_job_busy;
    wire j3_job_busy;
    wire j4_job_busy;
    wire j5_job_busy;
    wire j6_job_busy;
    wire j0_init_done;
    wire j3_init_done;
    wire j4_init_done;
    wire j5_init_done;
    wire j6_init_done;

    reg req_lis3dh_r;
    reg req_bmp585_r;
    reg req_lis2mdl_r;
    reg req_pmon1_r;
    reg req_hygro_r;
    reg req_gyro_r;
    reg req_lis2mdl_mag1_r;

    always @(posedge clk) begin
        if (rst) begin
            req_lis3dh_r <= 1'b0;
            req_bmp585_r <= 1'b0;
            req_lis2mdl_r <= 1'b0;
            req_pmon1_r <= 1'b0;
            req_hygro_r <= 1'b0;
            req_gyro_r <= 1'b0;
            req_lis2mdl_mag1_r <= 1'b0;
        end else begin
            if (lis3dh_path_en && epoch_100hz)
                req_lis3dh_r <= 1'b1;

            if (epoch_50hz)
                req_bmp585_r <= 1'b1;

            if (cmps2_mag_path_en && epoch_10hz)
                req_lis2mdl_r <= 1'b1;

            if (pmon1_path_en && epoch_10hz)
                req_pmon1_r <= 1'b1;

            if (hygro_path_en && epoch_1hz)
                req_hygro_r <= 1'b1;

            if (gyro_path_en && epoch_10hz)
                req_gyro_r <= 1'b1;

            if (lis2mdl_mag1_path_en && epoch_10hz)
                req_lis2mdl_mag1_r <= 1'b1;

            if (!lis3dh_path_en)
                req_lis3dh_r <= 1'b0;

            if (!cmps2_mag_path_en)
                req_lis2mdl_r <= 1'b0;

            if (!pmon1_path_en)
                req_pmon1_r <= 1'b0;

            if (!hygro_path_en)
                req_hygro_r <= 1'b0;

            if (!gyro_path_en)
                req_gyro_r <= 1'b0;

            if (!lis2mdl_mag1_path_en)
                req_lis2mdl_mag1_r <= 1'b0;

            if (grant_lis3dh)
                req_lis3dh_r <= 1'b0;

            if (grant_bmp585)
                req_bmp585_r <= 1'b0;

            if (grant_lis2mdl)
                req_lis2mdl_r <= 1'b0;

            if (grant_pmon1)
                req_pmon1_r <= 1'b0;

            if (grant_hygro)
                req_hygro_r <= 1'b0;

            if (grant_gyro)
                req_gyro_r <= 1'b0;

            if (grant_lis2mdl_mag1)
                req_lis2mdl_mag1_r <= 1'b0;
        end
    end

    // LIS3DH is an auxiliary I2C accelerometer. CMPS2/MMC34160PJ is the
    // magnetometer/heading source. Requests are latched so they are not lost
    // when 10 Hz/50 Hz epochs collide with the 100 Hz accelerometer epoch.
    assign want_lis3dh  = lis3dh_path_en    ? (req_lis3dh_r | j0_mux_busy | !j0_init_done) : 1'b0;
    assign want_bmp585  = req_bmp585_r      | j1_job_busy;
    assign want_lis2mdl = cmps2_mag_path_en ? (req_lis2mdl_r | j2_job_busy) : 1'b0;
    assign want_pmon1   = pmon1_path_en     ? (req_pmon1_r | j3_job_busy | !j3_init_done) : 1'b0;
    assign want_hygro   = hygro_path_en     ? (req_hygro_r | j4_job_busy | !j4_init_done) : 1'b0;
    assign want_gyro    = gyro_path_en      ? (req_gyro_r | j5_job_busy | !j5_init_done) : 1'b0;
    assign want_lis2mdl_mag1 =
        lis2mdl_mag1_path_en ? (req_lis2mdl_mag1_r | j6_job_busy | !j6_init_done) : 1'b0;



    //==========================================================================
    // 5) Shared engine command / stream contract
    //--------------------------------------------------------------------------
    // The engine contract is intentionally generic:
    //
    //   cmd_* : transfer descriptor
    //   w_*   : outbound write-data stream
    //   r_*   : inbound read-data stream
    //   done  : completion pulse with code
    //
    // One and only one job is connected to the engine at a time through the
    // job mux.
    //==========================================================================
    wire        e_cmd_valid;
    wire        e_cmd_ready;
    wire [6:0]  e_cmd_addr7;
    wire [7:0]  e_cmd_wlen;
    wire [7:0]  e_cmd_rlen;
    wire        e_cmd_repstart;
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

    //==========================================================================
    // 6) Fixed-priority arbiter
    //==========================================================================
    i2c_job_arbiter7 u_i2c_job_arbiter (
        .clk                 (clk),
        .rst                 (rst),
        .want_lis3dh         (want_lis3dh),
        .want_bmp585         (want_bmp585),
        .want_cmps2          (want_lis2mdl),
        .want_pmon1          (want_pmon1),
        .want_hygro          (want_hygro),
        .want_gyro           (want_gyro),
        .want_lis2mdl_mag1   (want_lis2mdl_mag1),
        .engine_busy         (e_busy),
        .grant_lis3dh        (grant_lis3dh),
        .grant_bmp585        (grant_bmp585),
        .grant_cmps2         (grant_lis2mdl),
        .grant_pmon1         (grant_pmon1),
        .grant_hygro         (grant_hygro),
        .grant_gyro          (grant_gyro),
        .grant_lis2mdl_mag1  (grant_lis2mdl_mag1)
    );

    //==========================================================================
    // 7) Per-job ports
    //--------------------------------------------------------------------------
    // Job 0 = LIS3DH auxiliary I2C accelerometer slot
    // Job 1 = BMP585
    // Job 2 = MMC34160PJ / Pmod CMPS2 magnetometer slot at I2C 7'h30
    //         Legacy internal lis2mdl grant/mux names are preserved.
    // Job 3 = PMON1 / ADM1191 power-monitor slot. Default address 7'h38 avoids
    //         the CMPS2/MMC34160PJ 7'h30 address.
    // Job 4 = Pmod HYGRO / HDC1080 environmental slot at 7'h40.
    // Job 5 = Pmod GYRO / L3G4200D angular-rate slot at 7'h69/7'h68.
    // Job 6 = LIS2MDL redundant magnetometer slot at 7'h1E for MAG1 evidence.
    //==========================================================================

    //--------------------------------------------------------------------------
    // Job 0: LIS3DH
    //--------------------------------------------------------------------------
    wire        j0_cmd_valid;
    wire        j0_cmd_ready;
    wire [6:0]  j0_cmd_addr7;
    wire [7:0]  j0_cmd_wlen;
    wire [7:0]  j0_cmd_rlen;
    wire        j0_cmd_repstart;
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

    lis3dh_job u_lis3dh_job (
        .clk             (clk),
        .rst             (lis3dh_path_rst),
        .time_us         (time_us),
        .epoch_100hz     (epoch_100hz),
        .grant           (grant_lis3dh),

        .cmd_valid       (j0_cmd_valid),
        .cmd_ready       (j0_cmd_ready),
        .cmd_addr7       (j0_cmd_addr7),
        .cmd_wlen        (j0_cmd_wlen),
        .cmd_rlen        (j0_cmd_rlen),
        .cmd_repstart    (j0_cmd_repstart),
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
        .busy            (j0_mux_busy),

        .snap_commit     (j0_snap_commit),
        .snap_valid_in   (j0_snap_valid_in),
        .snap_status_in  (j0_snap_status_in),
        .snap_payload_in (j0_snap_payload_in),

        .init_done       (j0_init_done)
    );



    wire [31:0] lis3dh_t_us;
    wire [15:0] lis3dh_seq;
    wire        lis3dh_valid;
    wire [7:0]  lis3dh_status;
    wire [SNAP_PAYLOAD_W-1:0] lis3dh_payload;

    snapshot_regs #(
        .PAYLOAD_W(SNAP_PAYLOAD_W)
    ) u_lis3dh_snapshot_regs (
        .clk          (clk),
        .rst          (lis3dh_path_rst),
        .time_us      (time_us),
        .commit       (j0_snap_commit),
        .valid_in     (j0_snap_valid_in),
        .status_in    (j0_snap_status_in),
        .payload_in   (j0_snap_payload_in),
        .snap_t_us    (lis3dh_t_us),
        .snap_seq     (lis3dh_seq),
        .snap_valid   (lis3dh_valid),
        .snap_status  (lis3dh_status),
        .snap_payload (lis3dh_payload)
    );



    //--------------------------------------------------------------------------
    // Dedicated ADXL362 / Pmod ACL2 accelerometer path
    //--------------------------------------------------------------------------
    wire        adxl_cmd_valid;
    wire        adxl_cmd_ready;
    wire [7:0]  adxl_cmd_wlen;
    wire [7:0]  adxl_cmd_rlen;
    wire [31:0] adxl_cmd_timeout_us;

    wire        adxl_w_valid;
    wire        adxl_w_ready;
    wire [7:0]  adxl_w_data;
    wire        adxl_w_last;

    wire        adxl_r_valid;
    wire        adxl_r_ready;
    wire [7:0]  adxl_r_data;
    wire        adxl_r_last;

    wire        adxl_done;
    wire [3:0]  adxl_done_code;
    wire        adxl_engine_busy;
    wire        adxl_job_busy;
    wire        adxl_init_done;

    wire        adxl_snap_commit;
    wire        adxl_snap_valid_in;
    wire [7:0]  adxl_snap_status_in;
    wire [SNAP_PAYLOAD_W-1:0] adxl_snap_payload_in;

    localparam integer ADXL_IRQ_POLICY_POLL_100HZ = 0;
    localparam integer ADXL_IRQ_POLICY_INT1_RISE  = 1;
    localparam integer ADXL_IRQ_POLICY_INT1_FALL  = 2;
    localparam integer ADXL_IRQ_POLICY_INT2_RISE  = 3;
    localparam integer ADXL_IRQ_POLICY_INT2_FALL  = 4;

    wire adxl_int1_level;
    wire adxl_int1_rise;
    wire adxl_int1_fall;
    wire adxl_int1_toggle;
    wire adxl_int2_level;
    wire adxl_int2_rise;
    wire adxl_int2_fall;
    wire adxl_int2_toggle;

    // ADXL362 INT1/INT2 are asynchronous to clk. Synchronize before using them
    // for any policy decision. The ADXL362 job maps DATA_READY to active-high
    // INT1 while the part is still in standby.
    sync_bit_3ff u_adxl362_int1_sync (
        .clk          (clk),
        .rst          (adxl_path_rst),
        .async_in     (adxl362_int1),
        .sync_level   (adxl_int1_level),
        .rise_pulse   (adxl_int1_rise),
        .fall_pulse   (adxl_int1_fall),
        .toggle_pulse (adxl_int1_toggle)
    );

    sync_bit_3ff u_adxl362_int2_sync (
        .clk          (clk),
        .rst          (adxl_path_rst),
        .async_in     (adxl362_int2),
        .sync_level   (adxl_int2_level),
        .rise_pulse   (adxl_int2_rise),
        .fall_pulse   (adxl_int2_fall),
        .toggle_pulse (adxl_int2_toggle)
    );

    wire adxl_sample_event_raw;
    wire adxl_sample_event;

    adxl_irq_sample_event #(
        .POLICY (ADXL_IRQ_POLICY)
    ) u_adxl_irq_sample_event (
        .poll_event   (epoch_100hz),
        .int1_rise    (adxl_int1_rise),
        .int1_fall    (adxl_int1_fall),
        .int2_rise    (adxl_int2_rise),
        .int2_fall    (adxl_int2_fall),
        .sample_event (adxl_sample_event_raw)
    );

    // Keep the ADXL362 synchronizers and false-path assumptions valid, but
    // prevent the SPI job from launching when the build selects LIS3DH-only
    // accelerometer publication for bring-up.
    assign adxl_sample_event = adxl_path_en ? adxl_sample_event_raw : 1'b0;

    wire        adxl_grant;
    assign adxl_grant = adxl_path_en && !adxl_engine_busy;

    adxl362_spi_job #(
        .CMD_TIMEOUT_US  (32'd2000),
        .POWERUP_WAIT_US (32'd5000),
        .MEASURE_WAIT_US (32'd40000)
    ) u_adxl362_spi_job (
        .clk             (clk),
        .rst             (adxl_path_rst),
        .time_us         (time_us),
        .epoch_100hz     (adxl_sample_event),
        .grant           (adxl_grant),

        .cmd_valid       (adxl_cmd_valid),
        .cmd_ready       (adxl_cmd_ready),
        .cmd_wlen        (adxl_cmd_wlen),
        .cmd_rlen        (adxl_cmd_rlen),
        .cmd_timeout_us  (adxl_cmd_timeout_us),

        .w_valid         (adxl_w_valid),
        .w_ready         (adxl_w_ready),
        .w_data          (adxl_w_data),
        .w_last          (adxl_w_last),

        .r_valid         (adxl_r_valid),
        .r_ready         (adxl_r_ready),
        .r_data          (adxl_r_data),
        .r_last          (adxl_r_last),

        .done            (adxl_done),
        .done_code       (adxl_done_code),
        .busy            (adxl_job_busy),

        .snap_commit     (adxl_snap_commit),
        .snap_valid_in   (adxl_snap_valid_in),
        .snap_status_in  (adxl_snap_status_in),
        .snap_payload_in (adxl_snap_payload_in),

        .init_done       (adxl_init_done)
    );

    spi_master_engine_mode0 #(
        .CLK_HZ     (CLK_HZ),
        .SPI_HZ     (1_000_000),
        .MAX_WBYTES (4),
        .MAX_RBYTES (8)
    ) u_adxl362_spi_master_engine_mode0 (
        .clk             (clk),
        .rst             (adxl_path_rst),

        .cmd_valid       (adxl_cmd_valid),
        .cmd_ready       (adxl_cmd_ready),
        .cmd_wlen        (adxl_cmd_wlen),
        .cmd_rlen        (adxl_cmd_rlen),
        .cmd_timeout_us  (adxl_cmd_timeout_us),

        .w_valid         (adxl_w_valid),
        .w_ready         (adxl_w_ready),
        .w_data          (adxl_w_data),
        .w_last          (adxl_w_last),

        .r_valid         (adxl_r_valid),
        .r_ready         (adxl_r_ready),
        .r_data          (adxl_r_data),
        .r_last          (adxl_r_last),

        .done            (adxl_done),
        .done_code       (adxl_done_code),
        .busy            (adxl_engine_busy),

        .tick_1us        (tick_1us),

        .spi_sclk        (adxl362_sclk),
        .spi_mosi        (adxl362_mosi),
        .spi_miso        (adxl362_miso),
        .adxl362_cs_n    (adxl362_cs_n)
    );

    wire _unused_adxl_status_ok;
    assign _unused_adxl_status_ok =
        adxl_job_busy   ^ adxl_init_done  ^
        adxl_int1_rise  ^ adxl_int1_fall  ^
        adxl_int1_level ^ adxl_int1_toggle ^
        adxl_int2_rise  ^ adxl_int2_fall  ^
        adxl_int2_level ^ adxl_int2_toggle;

    //--------------------------------------------------------------------------
    // Public accelerometer publication selector
    //--------------------------------------------------------------------------
    // acc_* is the stable compatibility surface consumed by derived-state and
    // visualization logic. LIS3DH remains auxiliary and never drives mag_* or
    // heading directly.
    //
    // Dual-source tie-breaker:
    //   - Valid ADXL362 reports win.
    //   - LIS3DH reports are accepted whenever ADXL362 is not committing a
    //     valid report in the same cycle.
    //   - Invalid ADXL362 reports do not overwrite an existing or simultaneous
    //     LIS3DH fallback publication; they publish only when no LIS3DH
    //     fallback exists yet.
    //--------------------------------------------------------------------------
    wire acc_use_lis3dh_only;
    wire acc_use_adxl_only;
    wire acc_use_dual;
    wire dual_select_adxl;
    wire dual_accept_adxl_valid;
    wire dual_accept_lis3dh;
    wire dual_accept_adxl_invalid;

    wire        acc_sel_commit;
    wire        acc_sel_valid_in;
    wire [7:0]  acc_sel_status_in;
    wire [SNAP_PAYLOAD_W-1:0] acc_sel_payload_in;

    assign acc_use_lis3dh_only =  lis3dh_path_en && !adxl_path_en;
    assign acc_use_adxl_only   = !lis3dh_path_en &&  adxl_path_en;
    assign acc_use_dual        =  lis3dh_path_en &&  adxl_path_en;

    assign dual_accept_adxl_valid   = adxl_snap_commit &&  adxl_snap_valid_in;
    assign dual_accept_lis3dh       = j0_snap_commit   && !dual_accept_adxl_valid;
    assign dual_accept_adxl_invalid = adxl_snap_commit && !adxl_snap_valid_in &&
                                      !lis3dh_valid    && !j0_snap_commit;
    assign dual_select_adxl = dual_accept_adxl_valid | dual_accept_adxl_invalid;

    assign acc_sel_commit =
        acc_use_lis3dh_only ? j0_snap_commit :
        acc_use_adxl_only   ? adxl_snap_commit :
        acc_use_dual        ? (dual_accept_adxl_valid |
                               dual_accept_lis3dh |
                               dual_accept_adxl_invalid) :
                              1'b0;

    assign acc_sel_valid_in =
        acc_use_lis3dh_only ? j0_snap_valid_in :
        acc_use_adxl_only   ? adxl_snap_valid_in :
        acc_use_dual        ? (dual_select_adxl ? adxl_snap_valid_in
                                                : j0_snap_valid_in) :
                              1'b0;

    assign acc_sel_status_in =
        acc_use_lis3dh_only ? j0_snap_status_in :
        acc_use_adxl_only   ? adxl_snap_status_in :
        acc_use_dual        ? (dual_select_adxl ? adxl_snap_status_in
                                                : j0_snap_status_in) :
                              8'h01;

    assign acc_sel_payload_in =
        acc_use_lis3dh_only ? j0_snap_payload_in :
        acc_use_adxl_only   ? adxl_snap_payload_in :
        acc_use_dual        ? (dual_select_adxl ? adxl_snap_payload_in
                                                : j0_snap_payload_in) :
                              {SNAP_PAYLOAD_W{1'b0}};

    snapshot_regs #(
        .PAYLOAD_W(SNAP_PAYLOAD_W)
    ) u_acc_snapshot_regs (
        .clk          (clk),
        .rst          (rst | ~(lis3dh_path_en | adxl_path_en)),
        .time_us      (time_us),
        .commit       (acc_sel_commit),
        .valid_in     (acc_sel_valid_in),
        .status_in    (acc_sel_status_in),
        .payload_in   (acc_sel_payload_in),
        .snap_t_us    (acc_t_us),
        .snap_seq     (acc_seq),
        .snap_valid   (acc_valid),
        .snap_status  (acc_status),
        .snap_payload (acc_payload)
    );

    //--------------------------------------------------------------------------
    // Job 1: BMP585
    //--------------------------------------------------------------------------
    wire        j1_cmd_valid;
    wire        j1_cmd_ready;
    wire [6:0]  j1_cmd_addr7;
    wire [7:0]  j1_cmd_wlen;
    wire [7:0]  j1_cmd_rlen;
    wire        j1_cmd_repstart;
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
    wire        j1_mux_busy;

    wire        j1_snap_commit;
    wire        j1_snap_valid_in;
    wire [7:0]  j1_snap_status_in;
    wire [SNAP_PAYLOAD_W-1:0] j1_snap_payload_in;

    wire        j1_init_done;

    bmp585_job #(
        .BMP585_ADDR7    (7'h47),
        .CMD_TIMEOUT_US  (32'd8000),
        .POWERUP_WAIT_US (32'd3000)
    ) u_bmp585_job (
        .clk             (clk),
        .rst             (rst),
        .time_us         (time_us),
        .epoch_50hz      (epoch_50hz),
        .grant           (grant_bmp585),

        .cmd_valid       (j1_cmd_valid),
        .cmd_ready       (j1_cmd_ready),
        .cmd_addr7       (j1_cmd_addr7),
        .cmd_wlen        (j1_cmd_wlen),
        .cmd_rlen        (j1_cmd_rlen),
        .cmd_repstart    (j1_cmd_repstart),
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
        .busy            (j1_job_busy),

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
    // Job 2: MMC34160PJ / Pmod CMPS2 magnetometer at fixed I2C address 7'h30
    //--------------------------------------------------------------------------
    wire        j2_cmd_valid;
    wire        j2_cmd_ready;
    wire [6:0]  j2_cmd_addr7;
    wire [7:0]  j2_cmd_wlen;
    wire [7:0]  j2_cmd_rlen;
    wire        j2_cmd_repstart;
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
    wire        j2_mux_busy;

    wire        j2_snap_commit;
    wire        j2_snap_valid_in;
    wire [7:0]  j2_snap_status_in;
    wire [SNAP_PAYLOAD_W-1:0] j2_snap_payload_in;

    wire        j2_init_done;

    mmc3416_i2c_job #(
        .MMC3416_ADDR7     (7'h30),
        .MMC3416_ADDR7_MIN (7'h30),
        .MMC3416_ADDR7_MAX (7'h30),
        .ADDR_PROBE_EN     (0),
        .CMD_TIMEOUT_US    (32'd3000),
        .POWERUP_WAIT_US   (32'd10000),
        .REFILL_WAIT_US    (32'd50000),
        .SETRESET_WAIT_US  (32'd1000),
        .POLL_GAP_US       (32'd1000),
        .MEAS_TIMEOUT_US   (32'd15000),
        .CONTROL1_VALUE    (8'h00)
    ) u_mmc3416_i2c_job (
        .clk             (clk),
        .rst             (cmps2_mag_path_rst),
        .time_us         (time_us),
        .epoch_10hz      (epoch_10hz),
        .grant           (grant_lis2mdl),

        .cmd_valid       (j2_cmd_valid),
        .cmd_ready       (j2_cmd_ready),
        .cmd_addr7       (j2_cmd_addr7),
        .cmd_wlen        (j2_cmd_wlen),
        .cmd_rlen        (j2_cmd_rlen),
        .cmd_repstart    (j2_cmd_repstart),
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
        .busy            (j2_job_busy),

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
        .rst          (cmps2_mag_path_rst),
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
    // Job 3: PMON1 / ADM1191 power monitor
    //--------------------------------------------------------------------------
    wire        j3_cmd_valid;
    wire        j3_cmd_ready;
    wire [6:0]  j3_cmd_addr7;
    wire [7:0]  j3_cmd_wlen;
    wire [7:0]  j3_cmd_rlen;
    wire        j3_cmd_repstart;
    wire [31:0] j3_cmd_timeout_us;

    wire        j3_w_valid;
    wire        j3_w_ready;
    wire [7:0]  j3_w_data;
    wire        j3_w_last;

    wire        j3_r_valid;
    wire        j3_r_ready;
    wire [7:0]  j3_r_data;
    wire        j3_r_last;

    wire        j3_done;
    wire [3:0]  j3_done_code;
    wire        j3_mux_busy;

    wire        j3_snap_commit;
    wire        j3_snap_valid_in;
    wire [7:0]  j3_snap_status_in;
    wire [SNAP_PAYLOAD_W-1:0] j3_snap_payload_in;

    pmon1_i2c_job #(
        .PMON1_ADDR7     (PMON1_ADDR7),
        .CMD_TIMEOUT_US  (32'd3000),
        .POWERUP_WAIT_US (32'd3000)
    ) u_pmon1_i2c_job (
        .clk             (clk),
        .rst             (pmon1_path_rst),
        .time_us         (time_us),
        .epoch_10hz      (epoch_10hz),
        .grant           (grant_pmon1),

        .cmd_valid       (j3_cmd_valid),
        .cmd_ready       (j3_cmd_ready),
        .cmd_addr7       (j3_cmd_addr7),
        .cmd_wlen        (j3_cmd_wlen),
        .cmd_rlen        (j3_cmd_rlen),
        .cmd_repstart    (j3_cmd_repstart),
        .cmd_timeout_us  (j3_cmd_timeout_us),

        .w_valid         (j3_w_valid),
        .w_ready         (j3_w_ready),
        .w_data          (j3_w_data),
        .w_last          (j3_w_last),

        .r_valid         (j3_r_valid),
        .r_ready         (j3_r_ready),
        .r_data          (j3_r_data),
        .r_last          (j3_r_last),

        .done            (j3_done),
        .done_code       (j3_done_code),
        .busy            (j3_job_busy),

        .snap_commit     (j3_snap_commit),
        .snap_valid_in   (j3_snap_valid_in),
        .snap_status_in  (j3_snap_status_in),
        .snap_payload_in (j3_snap_payload_in),

        .init_done       (j3_init_done)
    );

    snapshot_regs #(
        .PAYLOAD_W(SNAP_PAYLOAD_W)
    ) u_pwr_snapshot_regs (
        .clk          (clk),
        .rst          (pmon1_path_rst),
        .time_us      (time_us),
        .commit       (j3_snap_commit),
        .valid_in     (j3_snap_valid_in),
        .status_in    (j3_snap_status_in),
        .payload_in   (j3_snap_payload_in),
        .snap_t_us    (pwr_t_us),
        .snap_seq     (pwr_seq),
        .snap_valid   (pwr_valid),
        .snap_status  (pwr_status),
        .snap_payload (pwr_payload)
    );

    //--------------------------------------------------------------------------
    // Job 4: Pmod HYGRO / HDC1080 environmental sensor
    //--------------------------------------------------------------------------
    wire        j4_cmd_valid;
    wire        j4_cmd_ready;
    wire [6:0]  j4_cmd_addr7;
    wire [7:0]  j4_cmd_wlen;
    wire [7:0]  j4_cmd_rlen;
    wire        j4_cmd_repstart;
    wire [31:0] j4_cmd_timeout_us;

    wire        j4_w_valid;
    wire        j4_w_ready;
    wire [7:0]  j4_w_data;
    wire        j4_w_last;

    wire        j4_r_valid;
    wire        j4_r_ready;
    wire [7:0]  j4_r_data;
    wire        j4_r_last;

    wire        j4_done;
    wire [3:0]  j4_done_code;
    wire        j4_mux_busy;

    wire        j4_snap_commit;
    wire        j4_snap_valid_in;
    wire [7:0]  j4_snap_status_in;
    wire [SNAP_PAYLOAD_W-1:0] j4_snap_payload_in;

    pmod_hygro_i2c_job #(
        .HYGRO_ADDR7        (7'h40),
        .CMD_TIMEOUT_US     (32'd5000),
        .POWERUP_WAIT_US    (32'd15000),
        .CONVERSION_WAIT_US (32'd15000)
    ) u_pmod_hygro_i2c_job (
        .clk             (clk),
        .rst             (hygro_path_rst),
        .time_us         (time_us),
        .epoch_1hz       (epoch_1hz),
        .grant           (grant_hygro),

        .cmd_valid       (j4_cmd_valid),
        .cmd_ready       (j4_cmd_ready),
        .cmd_addr7       (j4_cmd_addr7),
        .cmd_wlen        (j4_cmd_wlen),
        .cmd_rlen        (j4_cmd_rlen),
        .cmd_repstart    (j4_cmd_repstart),
        .cmd_timeout_us  (j4_cmd_timeout_us),

        .w_valid         (j4_w_valid),
        .w_ready         (j4_w_ready),
        .w_data          (j4_w_data),
        .w_last          (j4_w_last),

        .r_valid         (j4_r_valid),
        .r_ready         (j4_r_ready),
        .r_data          (j4_r_data),
        .r_last          (j4_r_last),

        .done            (j4_done),
        .done_code       (j4_done_code),
        .busy            (j4_job_busy),

        .snap_commit     (j4_snap_commit),
        .snap_valid_in   (j4_snap_valid_in),
        .snap_status_in  (j4_snap_status_in),
        .snap_payload_in (j4_snap_payload_in),
        .init_done       (j4_init_done)
    );

    snapshot_regs #(
        .PAYLOAD_W(SNAP_PAYLOAD_W)
    ) u_env_snapshot_regs (
        .clk          (clk),
        .rst          (hygro_path_rst),
        .time_us      (time_us),
        .commit       (j4_snap_commit),
        .valid_in     (j4_snap_valid_in),
        .status_in    (j4_snap_status_in),
        .payload_in   (j4_snap_payload_in),
        .snap_t_us    (env_t_us),
        .snap_seq     (env_seq),
        .snap_valid   (env_valid),
        .snap_status  (env_status),
        .snap_payload (env_payload)
    );

    //--------------------------------------------------------------------------
    // Job 5: Pmod GYRO / L3G4200D
    //--------------------------------------------------------------------------
    wire        j5_cmd_valid;
    wire        j5_cmd_ready;
    wire [6:0]  j5_cmd_addr7;
    wire [7:0]  j5_cmd_wlen;
    wire [7:0]  j5_cmd_rlen;
    wire        j5_cmd_repstart;
    wire [31:0] j5_cmd_timeout_us;

    wire        j5_w_valid;
    wire        j5_w_ready;
    wire [7:0]  j5_w_data;
    wire        j5_w_last;

    wire        j5_r_valid;
    wire        j5_r_ready;
    wire [7:0]  j5_r_data;
    wire        j5_r_last;

    wire        j5_done;
    wire [3:0]  j5_done_code;
    wire        j5_mux_busy;

    wire        j5_snap_commit;
    wire        j5_snap_valid_in;
    wire [7:0]  j5_snap_status_in;
    wire [SNAP_PAYLOAD_W-1:0] j5_snap_payload_in;

    l3g4200d_i2c_job #(
        .GYRO_ADDR7     (7'h69),
        .GYRO_ADDR7_ALT (7'h68),
        .CMD_TIMEOUT_US (32'd3000)
    ) u_l3g4200d_i2c_job (
        .clk             (clk),
        .rst             (gyro_path_rst),
        .time_us         (time_us),
        .epoch_10hz      (epoch_10hz),
        .grant           (grant_gyro),

        .cmd_valid       (j5_cmd_valid),
        .cmd_ready       (j5_cmd_ready),
        .cmd_addr7       (j5_cmd_addr7),
        .cmd_wlen        (j5_cmd_wlen),
        .cmd_rlen        (j5_cmd_rlen),
        .cmd_repstart    (j5_cmd_repstart),
        .cmd_timeout_us  (j5_cmd_timeout_us),

        .w_valid         (j5_w_valid),
        .w_ready         (j5_w_ready),
        .w_data          (j5_w_data),
        .w_last          (j5_w_last),

        .r_valid         (j5_r_valid),
        .r_ready         (j5_r_ready),
        .r_data          (j5_r_data),
        .r_last          (j5_r_last),

        .done            (j5_done),
        .done_code       (j5_done_code),
        .busy            (j5_job_busy),

        .snap_commit     (j5_snap_commit),
        .snap_valid_in   (j5_snap_valid_in),
        .snap_status_in  (j5_snap_status_in),
        .snap_payload_in (j5_snap_payload_in),
        .init_done       (j5_init_done)
    );

    snapshot_regs #(
        .PAYLOAD_W(SNAP_PAYLOAD_W)
    ) u_gyro_snapshot_regs (
        .clk          (clk),
        .rst          (gyro_path_rst),
        .time_us      (time_us),
        .commit       (j5_snap_commit),
        .valid_in     (j5_snap_valid_in),
        .status_in    (j5_snap_status_in),
        .payload_in   (j5_snap_payload_in),
        .snap_t_us    (gyro_t_us),
        .snap_seq     (gyro_seq),
        .snap_valid   (gyro_valid),
        .snap_status  (gyro_status),
        .snap_payload (gyro_payload)
    );

    //--------------------------------------------------------------------------
    // Job 6: LIS2MDL physical redundant magnetometer for MAG1 evidence
    //--------------------------------------------------------------------------
    wire        j6_cmd_valid;
    wire        j6_cmd_ready;
    wire [6:0]  j6_cmd_addr7;
    wire [7:0]  j6_cmd_wlen;
    wire [7:0]  j6_cmd_rlen;
    wire        j6_cmd_repstart;
    wire [31:0] j6_cmd_timeout_us;

    wire        j6_w_valid;
    wire        j6_w_ready;
    wire [7:0]  j6_w_data;
    wire        j6_w_last;

    wire        j6_r_valid;
    wire        j6_r_ready;
    wire [7:0]  j6_r_data;
    wire        j6_r_last;

    wire        j6_done;
    wire [3:0]  j6_done_code;
    wire        j6_mux_busy;

    wire        j6_snap_commit;
    wire        j6_snap_valid_in;
    wire [7:0]  j6_snap_status_in;
    wire [SNAP_PAYLOAD_W-1:0] j6_snap_payload_in;





    lis2mdl_job #(
        .LIS2MDL_ADDR7  (7'h1E),
        .CMD_TIMEOUT_US (32'd3000)
    ) u_lis2mdl_job (
        .clk             (clk),
        .rst             (lis2mdl_mag1_path_rst),
        .time_us         (time_us),
        .epoch_10hz      (epoch_10hz),
        .grant           (grant_lis2mdl_mag1),

        .cmd_valid       (j6_cmd_valid),
        .cmd_ready       (j6_cmd_ready),
        .cmd_addr7       (j6_cmd_addr7),
        .cmd_wlen        (j6_cmd_wlen),
        .cmd_rlen        (j6_cmd_rlen),
        .cmd_repstart    (j6_cmd_repstart),
        .cmd_timeout_us  (j6_cmd_timeout_us),

        .w_valid         (j6_w_valid),
        .w_ready         (j6_w_ready),
        .w_data          (j6_w_data),
        .w_last          (j6_w_last),

        .r_valid         (j6_r_valid),
        .r_ready         (j6_r_ready),
        .r_data          (j6_r_data),
        .r_last          (j6_r_last),

        .done            (j6_done),
        .done_code       (j6_done_code),
        .busy            (j6_job_busy),

        .snap_commit     (j6_snap_commit),
        .snap_valid_in   (j6_snap_valid_in),
        .snap_status_in  (j6_snap_status_in),
        .snap_payload_in (j6_snap_payload_in),
        .init_done       (j6_init_done)
    );


    snapshot_regs #(
        .PAYLOAD_W(SNAP_PAYLOAD_W)
    ) u_lis2mdl_mag1_snapshot_regs (
        .clk          (clk),
        .rst          (lis2mdl_mag1_path_rst),
        .time_us      (time_us),
        .commit       (j6_snap_commit),
        .valid_in     (j6_snap_valid_in),
        .status_in    (j6_snap_status_in),
        .payload_in   (j6_snap_payload_in),
        .snap_t_us    (mag1_t_us),
        .snap_seq     (mag1_seq),
        .snap_valid   (mag1_valid),
        .snap_status  (mag1_status),
        .snap_payload (mag1_payload)
    );

    assign mag1_cal_state = mag1_valid ? 8'h01 : 8'd0;
    assign mag1_source_flags = mag1_valid ? (8'd1 << `EXT_SRC_REAL_BIT) : 8'd0;
    assign mag1_bridge_checksum =
        mag1_valid ? (mag1_payload[15:0] ^
                      mag1_payload[31:16] ^
                      mag1_payload[47:32] ^
                      mag1_seq ^
                      {mag1_status, mag1_source_flags}) :
                     16'd0;

    //==========================================================================
    // 8) Job mux
    //--------------------------------------------------------------------------
    // The mux is the only path between per-job streams and the shared engine.
    // It prevents accidental multi-job ownership of the engine contract.
    //==========================================================================
    i2c_job_mux7 u_i2c_job_mux (
        .clk              (clk),
        .rst              (rst),

        .grant_j0         (grant_lis3dh),
        .grant_j1         (grant_bmp585),
        .grant_j2         (grant_lis2mdl),
        .grant_j3         (grant_pmon1),
        .grant_j4         (grant_hygro),
        .grant_j5         (grant_gyro),
        .grant_j6         (grant_lis2mdl_mag1),

        .j0_cmd_valid     (j0_cmd_valid),
        .j0_cmd_ready     (j0_cmd_ready),
        .j0_cmd_addr7     (j0_cmd_addr7),
        .j0_cmd_wlen      (j0_cmd_wlen),
        .j0_cmd_rlen      (j0_cmd_rlen),
        .j0_cmd_repstart  (j0_cmd_repstart),
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
        .j0_busy          (j0_mux_busy),

        .j1_cmd_valid     (j1_cmd_valid),
        .j1_cmd_ready     (j1_cmd_ready),
        .j1_cmd_addr7     (j1_cmd_addr7),
        .j1_cmd_wlen      (j1_cmd_wlen),
        .j1_cmd_rlen      (j1_cmd_rlen),
        .j1_cmd_repstart  (j1_cmd_repstart),
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
        .j1_busy          (j1_mux_busy),

        .j2_cmd_valid     (j2_cmd_valid),
        .j2_cmd_ready     (j2_cmd_ready),
        .j2_cmd_addr7     (j2_cmd_addr7),
        .j2_cmd_wlen      (j2_cmd_wlen),
        .j2_cmd_rlen      (j2_cmd_rlen),
        .j2_cmd_repstart  (j2_cmd_repstart),
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
        .j2_busy          (j2_mux_busy),

        .j3_cmd_valid     (j3_cmd_valid),
        .j3_cmd_ready     (j3_cmd_ready),
        .j3_cmd_addr7     (j3_cmd_addr7),
        .j3_cmd_wlen      (j3_cmd_wlen),
        .j3_cmd_rlen      (j3_cmd_rlen),
        .j3_cmd_repstart  (j3_cmd_repstart),
        .j3_cmd_timeout_us(j3_cmd_timeout_us),
        .j3_w_valid       (j3_w_valid),
        .j3_w_ready       (j3_w_ready),
        .j3_w_data        (j3_w_data),
        .j3_w_last        (j3_w_last),
        .j3_r_valid       (j3_r_valid),
        .j3_r_ready       (j3_r_ready),
        .j3_r_data        (j3_r_data),
        .j3_r_last        (j3_r_last),
        .j3_done          (j3_done),
        .j3_done_code     (j3_done_code),
        .j3_busy          (j3_mux_busy),

        .j4_cmd_valid     (j4_cmd_valid),
        .j4_cmd_ready     (j4_cmd_ready),
        .j4_cmd_addr7     (j4_cmd_addr7),
        .j4_cmd_wlen      (j4_cmd_wlen),
        .j4_cmd_rlen      (j4_cmd_rlen),
        .j4_cmd_repstart  (j4_cmd_repstart),
        .j4_cmd_timeout_us(j4_cmd_timeout_us),
        .j4_w_valid       (j4_w_valid),
        .j4_w_ready       (j4_w_ready),
        .j4_w_data        (j4_w_data),
        .j4_w_last        (j4_w_last),
        .j4_r_valid       (j4_r_valid),
        .j4_r_ready       (j4_r_ready),
        .j4_r_data        (j4_r_data),
        .j4_r_last        (j4_r_last),
        .j4_done          (j4_done),
        .j4_done_code     (j4_done_code),
        .j4_busy          (j4_mux_busy),

        .j5_cmd_valid     (j5_cmd_valid),
        .j5_cmd_ready     (j5_cmd_ready),
        .j5_cmd_addr7     (j5_cmd_addr7),
        .j5_cmd_wlen      (j5_cmd_wlen),
        .j5_cmd_rlen      (j5_cmd_rlen),
        .j5_cmd_repstart  (j5_cmd_repstart),
        .j5_cmd_timeout_us(j5_cmd_timeout_us),
        .j5_w_valid       (j5_w_valid),
        .j5_w_ready       (j5_w_ready),
        .j5_w_data        (j5_w_data),
        .j5_w_last        (j5_w_last),
        .j5_r_valid       (j5_r_valid),
        .j5_r_ready       (j5_r_ready),
        .j5_r_data        (j5_r_data),
        .j5_r_last        (j5_r_last),
        .j5_done          (j5_done),
        .j5_done_code     (j5_done_code),
        .j5_busy          (j5_mux_busy),

        .j6_cmd_valid     (j6_cmd_valid),
        .j6_cmd_ready     (j6_cmd_ready),
        .j6_cmd_addr7     (j6_cmd_addr7),
        .j6_cmd_wlen      (j6_cmd_wlen),
        .j6_cmd_rlen      (j6_cmd_rlen),
        .j6_cmd_repstart  (j6_cmd_repstart),
        .j6_cmd_timeout_us(j6_cmd_timeout_us),
        .j6_w_valid       (j6_w_valid),
        .j6_w_ready       (j6_w_ready),
        .j6_w_data        (j6_w_data),
        .j6_w_last        (j6_w_last),
        .j6_r_valid       (j6_r_valid),
        .j6_r_ready       (j6_r_ready),
        .j6_r_data        (j6_r_data),
        .j6_r_last        (j6_r_last),
        .j6_done          (j6_done),
        .j6_done_code     (j6_done_code),
        .j6_busy          (j6_mux_busy),

        .e_cmd_valid      (e_cmd_valid),
        .e_cmd_ready      (e_cmd_ready),
        .e_cmd_addr7      (e_cmd_addr7),
        .e_cmd_wlen       (e_cmd_wlen),
        .e_cmd_rlen       (e_cmd_rlen),
        .e_cmd_repstart   (e_cmd_repstart),
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

    //==========================================================================
    // 9) Shared I2C engine
    //--------------------------------------------------------------------------
    // This module is assumed to be the single electrical owner of scl/sda.
    // Suite-level logic interacts only through the abstract engine contract.
    //==========================================================================
    i2c_master_engine #(
        .CLK_HZ    (CLK_HZ),
        .I2C_HZ    (100_000),
        .MAX_WBYTES(8),
        .MAX_RBYTES(16)
    ) u_i2c_master_engine (
        .clk             (clk),
        .rst             (rst),

        .cmd_valid       (e_cmd_valid),
        .cmd_ready       (e_cmd_ready),
        .cmd_addr7       (e_cmd_addr7),
        .cmd_wlen        (e_cmd_wlen),
        .cmd_rlen        (e_cmd_rlen),
        .cmd_repstart    (e_cmd_repstart),
        .cmd_timeout_us  (e_cmd_timeout_us),

        .w_valid         (e_w_valid),
        .w_ready         (e_w_ready),
        .w_data          (e_w_data),
        .w_last          (e_w_last),

        .r_valid         (e_r_valid),
        .r_ready         (e_r_ready),
        .r_data          (e_r_data),
        .r_last          (e_r_last),

        .done            (e_done),
        .done_code       (e_done_code),
        .busy            (e_busy),

        .tick_1us        (tick_1us),

        .scl             (scl),
        .sda             (sda)
    );



    //==========================================================================
    // 10) Age computation for committed raw snapshots
    //--------------------------------------------------------------------------
    // Snapshot ages are presented in milliseconds, saturating at 16 bits.
    //
    // Timing policy:
    //   Do not divide the free-running microsecond timebase by 1000 on every
    //   SYS clock. Ages are display/derived-freshness observability fields, so
    //   they are maintained as explicit millisecond counters:
    //
    //     - each published raw snapshot commit resets that sensor's age to zero
    //     - one shared 1 ms tick increments non-saturated age counters
    //     - counters saturate at 16'hFFFF
    //
    // This preserves the published age/freshness contract while keeping the
    // timebase out of the visualization and derived-status combinational cone.
    //==========================================================================
    reg [9:0]  age_us_div_r;
    reg [15:0] bmp_age_ms_r;
    reg [15:0] acc_age_ms_r;
    reg [15:0] mag_age_ms_r;
    reg [15:0] pwr_age_ms_r;
    reg [15:0] env_age_ms_r;
    reg [15:0] gyro_age_ms_r;
    reg [15:0] mag1_age_ms_r;

    wire age_ms_tick_w;
    assign age_ms_tick_w = tick_1us && (age_us_div_r == 10'd999);

    always @(posedge clk) begin
        if (rst) begin
            age_us_div_r <= 10'd0;
            bmp_age_ms_r <= 16'd0;
            acc_age_ms_r <= 16'd0;
            mag_age_ms_r <= 16'd0;
            pwr_age_ms_r <= 16'd0;
            env_age_ms_r <= 16'd0;
            gyro_age_ms_r <= 16'd0;
            mag1_age_ms_r <= 16'd0;
        end else begin
            if (tick_1us) begin
                if (age_us_div_r == 10'd999)
                    age_us_div_r <= 10'd0;
                else
                    age_us_div_r <= age_us_div_r + 10'd1;
            end

            if (j1_snap_commit)
                bmp_age_ms_r <= 16'd0;
            else if (age_ms_tick_w && (bmp_age_ms_r != 16'hFFFF))
                bmp_age_ms_r <= bmp_age_ms_r + 16'd1;

            if (!(lis3dh_path_en | adxl_path_en))
                acc_age_ms_r <= 16'hFFFF;
            else if (acc_sel_commit)
                acc_age_ms_r <= 16'd0;
            else if (age_ms_tick_w && (acc_age_ms_r != 16'hFFFF))
                acc_age_ms_r <= acc_age_ms_r + 16'd1;

            if (!cmps2_mag_path_en)
                mag_age_ms_r <= 16'hFFFF;
            else if (j2_snap_commit)
                mag_age_ms_r <= 16'd0;
            else if (age_ms_tick_w && (mag_age_ms_r != 16'hFFFF))
                mag_age_ms_r <= mag_age_ms_r + 16'd1;

            if (!pmon1_path_en)
                pwr_age_ms_r <= 16'hFFFF;
            else if (j3_snap_commit)
                pwr_age_ms_r <= 16'd0;
            else if (age_ms_tick_w && (pwr_age_ms_r != 16'hFFFF))
                pwr_age_ms_r <= pwr_age_ms_r + 16'd1;

            if (!hygro_path_en)
                env_age_ms_r <= 16'hFFFF;
            else if (j4_snap_commit)
                env_age_ms_r <= 16'd0;
            else if (age_ms_tick_w && (env_age_ms_r != 16'hFFFF))
                env_age_ms_r <= env_age_ms_r + 16'd1;

            if (!gyro_path_en)
                gyro_age_ms_r <= 16'hFFFF;
            else if (j5_snap_commit)
                gyro_age_ms_r <= 16'd0;
            else if (age_ms_tick_w && (gyro_age_ms_r != 16'hFFFF))
                gyro_age_ms_r <= gyro_age_ms_r + 16'd1;

            if (!lis2mdl_mag1_path_en)
                mag1_age_ms_r <= 16'hFFFF;
            else if (j6_snap_commit)
                mag1_age_ms_r <= 16'd0;
            else if (age_ms_tick_w && (mag1_age_ms_r != 16'hFFFF))
                mag1_age_ms_r <= mag1_age_ms_r + 16'd1;
        end
    end

    wire [15:0] bmp_age_ms;
    wire [15:0] acc_age_ms;
    wire [15:0] mag_age_ms;

    assign bmp_age_ms = bmp_age_ms_r;
    assign acc_age_ms = acc_age_ms_r;
    assign mag_age_ms = mag_age_ms_r;
    assign pwr_age_ms = pwr_age_ms_r;
    assign env_age_ms = env_age_ms_r;
    assign gyro_age_ms = gyro_age_ms_r;
    assign mag1_age_ms = mag1_age_ms_r;

    //==========================================================================
    // 11) Global transaction / observability ownership
    //--------------------------------------------------------------------------
    // All suite-level counters are owned here, in one place.
    //==========================================================================
    reg [15:0] txn_count_window_r;
    reg [15:0] txn_rate_hz_r;
    reg [15:0] i2c_nack_count_r;
    reg [15:0] i2c_timeout_count_r;
    reg [31:0] cdc_update_count_r;
    reg [31:0] frame_count_r;

    reg [19:0] us_to_1s_ctr;

    always @(posedge clk) begin
        if (rst) begin
            txn_count_window_r <= 16'd0;
            txn_rate_hz_r      <= 16'd0;
            i2c_nack_count_r   <= 16'd0;
            i2c_timeout_count_r<= 16'd0;
            cdc_update_count_r <= 32'd0;
            frame_count_r      <= 32'd0;
            us_to_1s_ctr       <= 20'd0;
        end else begin
            // Count completed engine transactions.
            if (e_done)
                txn_count_window_r <= txn_count_window_r + 16'd1;

            // Completion code ownership follows i2c_master_engine:
            //   1,2,5 -> NACK on address/write-data/read-address phase
            //   3,6   -> preload or bus timeout
            //   4     -> command length error, not an electrical NACK/timeout
            if (e_done && ((e_done_code == 4'd1) ||
                           (e_done_code == 4'd2) ||
                           (e_done_code == 4'd5)))
                i2c_nack_count_r <= i2c_nack_count_r + 16'd1;

            if (e_done && ((e_done_code == 4'd3) ||
                           (e_done_code == 4'd6)))
                i2c_timeout_count_r <= i2c_timeout_count_r + 16'd1;

            // Placeholder observability surfaces retained for compatibility.
            // Count committed public snapshot updates, including extension banks.
            if (acc_sel_commit || j1_snap_commit || j2_snap_commit ||
                j3_snap_commit || j4_snap_commit || j5_snap_commit ||
                j6_snap_commit)
                cdc_update_count_r <= cdc_update_count_r + 32'd1;

            if (tick_1us) begin
                if (us_to_1s_ctr == 20'd999_999) begin
                    us_to_1s_ctr   <= 20'd0;
                    txn_rate_hz_r  <= txn_count_window_r;
                    txn_count_window_r <= 16'd0;
                    frame_count_r  <= frame_count_r + 32'd1;
                end else begin
                    us_to_1s_ctr <= us_to_1s_ctr + 20'd1;
                end
            end
        end
    end

    assign der_i2c_nack_count   = i2c_nack_count_r;
    assign der_i2c_timeout_count= i2c_timeout_count_r;
    assign der_txn_rate_hz      = txn_rate_hz_r;
    assign der_cdc_update_count = cdc_update_count_r;
    assign der_frame_count      = frame_count_r;
    assign der_build_id         = BUILD_ID;
    assign der_schema_word      = SCHEMA_WORD;

    //==========================================================================
    // 12) Derived-state producer
    //--------------------------------------------------------------------------
    // Raw committed snapshots remain owned by this suite. All derived-state
    // math, freshness, status, sequence provenance, and publication sequencing
    // are delegated to the shared producer.
    //==========================================================================
    derived_state_producer #(
        .BUILD_ID_CONST    (BUILD_ID),
        .SCHEMA_WORD_CONST (SCHEMA_WORD),
        // MMC34160PJ job publishes {MZ, MY, MX}; heading uses atan2(MY, MX).
        .MAG_PAYLOAD_ZYX   (1),
        .ALT_FRESH_MAX_MS  (BMP_FRESH_MAX_MS),
        .VSPD_FRESH_MAX_MS (BMP_FRESH_MAX_MS),
        .ROLL_FRESH_MAX_MS (ACC_FRESH_MAX_MS),
        .HEAD_FRESH_MAX_MS (MAG_FRESH_MAX_MS)
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

        .i2c_nack_count        (i2c_nack_count_r),
        .i2c_timeout_count     (i2c_timeout_count_r),
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
