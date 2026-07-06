`timescale 1ns/1ps
`default_nettype none

`include "flight_viz_bundle_defs.vh"
`include "telemetry_defs_vh.vh"

//==============================================================================
// tb_flight_visualizer_pix
//------------------------------------------------------------------------------
// PURPOSE
//   Self-checking simulation testbench for the full flight_visualizer_pix path.
//
// PROVEN PROPERTIES
//   1) Compact, clipped telemetry text overlay has priority only in its owned
//      layout bands.
//   2) Outside active video, vga_rgb is black.
//   3) Telemetry overlay 24-bit RGB is packed to 12-bit VGA as:
//        {R[23:20], G[15:12], B[7:4]}
//   4) The newest history sample is marked on the right edge of both charts.
//   5) The attitude/vertical-vector instrument renders from committed roll.
//   6) The apogee-authority physics overlay renders target, uncertainty,
//      prediction corridor, gate, and command evidence.
//   7) Panel titles are visible and text does not collide with instrument
//      centers.
//
// TEST STRATEGY
//   - Load a nominal committed visualization bundle.
//   - Wait for known active pixels inside compact status/title/debug glyphs.
//   - Check overlay_on, packed color, clipping, and override priority.
//   - Wait for a known inactive pixel and check blacking.
//==============================================================================
module tb_flight_visualizer_pix;

    //--------------------------------------------------------------------------
    // Clock / reset
    //--------------------------------------------------------------------------
    reg pix_clk;
    reg pix_rst;

    //--------------------------------------------------------------------------
    // DUT stimulus
    //--------------------------------------------------------------------------
    reg  [`VIZ_BUNDLE_W-1:0] viz_bundle_pix;
    reg                      viz_update_pix;
    reg  [1:0]               vga_page_select_pix;

    reg  [9:0]               wr_ptr_pix;
    reg                      wr_ptr_update_pix;

    wire [9:0]               alt_rd_addr;
    reg  [15:0]              alt_rd_data;

    wire [9:0]               vspd_rd_addr;
    reg  [15:0]              vspd_rd_data;

    wire                     vga_hsync;
    wire                     vga_vsync;
    wire [11:0]              vga_rgb;

    //--------------------------------------------------------------------------
    // Simple BRAM models
    //--------------------------------------------------------------------------
    reg [15:0] alt_mem  [0:1023];
    reg [15:0] vspd_mem [0:1023];

    integer i;

    //--------------------------------------------------------------------------
    // DUT
    //--------------------------------------------------------------------------
    flight_visualizer_pix u_dut (
        .pix_clk           (pix_clk),
        .pix_rst           (pix_rst),

        .viz_bundle_pix    (viz_bundle_pix),
        .viz_update_pix    (viz_update_pix),
        .vga_page_select_pix(vga_page_select_pix),

        .wr_ptr_pix        (wr_ptr_pix),
        .wr_ptr_update_pix (wr_ptr_update_pix),

        .alt_rd_addr       (alt_rd_addr),
        .alt_rd_data       (alt_rd_data),

        .vspd_rd_addr      (vspd_rd_addr),
        .vspd_rd_data      (vspd_rd_data),

        .vga_hsync         (vga_hsync),
        .vga_vsync         (vga_vsync),
        .vga_rgb           (vga_rgb)
    );

    //--------------------------------------------------------------------------
    // Clock
    //--------------------------------------------------------------------------
    initial begin
        pix_clk = 1'b0;
        forever #5 pix_clk = ~pix_clk;
    end

    //--------------------------------------------------------------------------
    // Simple synchronous BRAM read model
    //--------------------------------------------------------------------------
    always @(posedge pix_clk) begin
        alt_rd_data  <= alt_mem[alt_rd_addr];
        vspd_rd_data <= vspd_mem[vspd_rd_addr];
    end

    //--------------------------------------------------------------------------
    // Bundle construction helpers
    //--------------------------------------------------------------------------
    task clear_bundle;
        begin
            viz_bundle_pix = {`VIZ_BUNDLE_W{1'b0}};
        end
    endtask

    task load_nominal_bundle;
        begin
            clear_bundle();

            // Raw BMP summary
            viz_bundle_pix[`VIZ_BMP_SEQ_MSB:`VIZ_BMP_SEQ_LSB]       = 16'h007A;
            viz_bundle_pix[`VIZ_BMP_VALID_BIT]                      = 1'b1;
            viz_bundle_pix[`VIZ_BMP_STATUS_MSB:`VIZ_BMP_STATUS_LSB] = 8'h00;
            viz_bundle_pix[`VIZ_BMP_AGE_MS_MSB:`VIZ_BMP_AGE_MS_LSB] = 16'd14;

            // Raw ACC summary
            viz_bundle_pix[`VIZ_ACC_SEQ_MSB:`VIZ_ACC_SEQ_LSB]       = 16'h012C;
            viz_bundle_pix[`VIZ_ACC_VALID_BIT]                      = 1'b1;
            viz_bundle_pix[`VIZ_ACC_STATUS_MSB:`VIZ_ACC_STATUS_LSB] = 8'h00;
            viz_bundle_pix[`VIZ_ACC_AGE_MS_MSB:`VIZ_ACC_AGE_MS_LSB] = 16'd4;

            // Raw MAG summary
            viz_bundle_pix[`VIZ_MAG_SEQ_MSB:`VIZ_MAG_SEQ_LSB]       = 16'h0051;
            viz_bundle_pix[`VIZ_MAG_VALID_BIT]                      = 1'b1;
            viz_bundle_pix[`VIZ_MAG_STATUS_MSB:`VIZ_MAG_STATUS_LSB] = 8'h00;
            viz_bundle_pix[`VIZ_MAG_AGE_MS_MSB:`VIZ_MAG_AGE_MS_LSB] = 16'd98;

            // Derived summary
            viz_bundle_pix[`VIZ_DER_VALID_BIT]                      = 1'b1;
            viz_bundle_pix[`VIZ_DER_STATUS_MSB:`VIZ_DER_STATUS_LSB] = 8'h00;

            viz_bundle_pix[`VIZ_DER_ALT_FRESH_BIT]  = 1'b1;
            viz_bundle_pix[`VIZ_DER_VSPD_FRESH_BIT] = 1'b1;
            viz_bundle_pix[`VIZ_DER_ROLL_FRESH_BIT] = 1'b1;
            viz_bundle_pix[`VIZ_DER_HEAD_FRESH_BIT] = 1'b1;

            viz_bundle_pix[`VIZ_DER_BMP_SEQ_REF_MSB:`VIZ_DER_BMP_SEQ_REF_LSB] = 16'h007A;
            viz_bundle_pix[`VIZ_DER_ACC_SEQ_REF_MSB:`VIZ_DER_ACC_SEQ_REF_LSB] = 16'h012C;
            viz_bundle_pix[`VIZ_DER_MAG_SEQ_REF_MSB:`VIZ_DER_MAG_SEQ_REF_LSB] = 16'h0051;

            viz_bundle_pix[`VIZ_DER_BMP_AGE_MS_MSB:`VIZ_DER_BMP_AGE_MS_LSB] = 16'd14;
            viz_bundle_pix[`VIZ_DER_ACC_AGE_MS_MSB:`VIZ_DER_ACC_AGE_MS_LSB] = 16'd4;
            viz_bundle_pix[`VIZ_DER_MAG_AGE_MS_MSB:`VIZ_DER_MAG_AGE_MS_LSB] = 16'd98;

            viz_bundle_pix[`VIZ_DER_BMP_VALID_REF_BIT] = 1'b1;
            viz_bundle_pix[`VIZ_DER_ACC_VALID_REF_BIT] = 1'b1;
            viz_bundle_pix[`VIZ_DER_MAG_VALID_REF_BIT] = 1'b1;

            // 1200 m and 200 m/s gives a visible authority case:
            //   no-brake apogee ~= 3214 m, above the 3048 m target
            //   full-brake drag estimate ~= 2207 m, below the target
            viz_bundle_pix[`VIZ_DER_ALT_CM_MSB:`VIZ_DER_ALT_CM_LSB]       = 32'd120000;
            viz_bundle_pix[`VIZ_DER_VSPD_CMS_MSB:`VIZ_DER_VSPD_CMS_LSB]   = 32'd20000;
            viz_bundle_pix[`VIZ_DER_ROLL_MDEG_MSB:`VIZ_DER_ROLL_MDEG_LSB] = 32'd12000;
            viz_bundle_pix[`VIZ_DER_HEAD_MDEG_MSB:`VIZ_DER_HEAD_MDEG_LSB] = 32'd123000;

            // Published SYS-domain apogee authority / drag-servo policy record.
            viz_bundle_pix[`VIZ_AUTH_VALID_BIT] = 1'b1;
            viz_bundle_pix[`VIZ_AUTH_STATUS_MSB:`VIZ_AUTH_STATUS_LSB] = 8'h00;
            viz_bundle_pix[`VIZ_AUTH_FLAGS_MSB:`VIZ_AUTH_FLAGS_LSB] = 8'h9F;
            viz_bundle_pix[`VIZ_AUTH_TARGET_CM_MSB:`VIZ_AUTH_TARGET_CM_LSB] = 32'd304800;
            viz_bundle_pix[`VIZ_AUTH_PRED_NO_CM_MSB:`VIZ_AUTH_PRED_NO_CM_LSB] = 32'd321416;
            viz_bundle_pix[`VIZ_AUTH_PRED_FULL_CM_MSB:`VIZ_AUTH_PRED_FULL_CM_LSB] = 32'd220708;
            viz_bundle_pix[`VIZ_AUTH_UNC_CM_MSB:`VIZ_AUTH_UNC_CM_LSB] = 16'd1780;
            viz_bundle_pix[`VIZ_AUTH_CMD_U8_MSB:`VIZ_AUTH_CMD_U8_LSB] = 8'd64;
            viz_bundle_pix[`VIZ_AUTH_SERVO_US_MSB:`VIZ_AUTH_SERVO_US_LSB] = 12'd1250;
            viz_bundle_pix[`VIZ_AUTH_PHASE_MSB:`VIZ_AUTH_PHASE_LSB] = `VIZ_AUTH_PHASE_BRAKE;
            viz_bundle_pix[`VIZ_AUTH_GATE_MSB:`VIZ_AUTH_GATE_LSB] = 7'h3F;

            // PMON1 / ADM1191 power monitor evidence for bottom debug strip.
            viz_bundle_pix[`VIZ_PWR_VALID_BIT] = 1'b1;
            viz_bundle_pix[`VIZ_PWR_STATUS_MSB:`VIZ_PWR_STATUS_LSB] = 8'h00;
            viz_bundle_pix[`VIZ_PWR_SEQ_MSB:`VIZ_PWR_SEQ_LSB] = 16'h0F5A;
            viz_bundle_pix[`VIZ_PWR_AGE_MS_MSB:`VIZ_PWR_AGE_MS_LSB] = 16'd44;
            viz_bundle_pix[`VIZ_PWR_VOLT_CODE_MSB:`VIZ_PWR_VOLT_CODE_LSB] = 12'h5A0;
            viz_bundle_pix[`VIZ_PWR_CURR_CODE_MSB:`VIZ_PWR_CURR_CODE_LSB] = 12'h120;
            viz_bundle_pix[`VIZ_PWR_ALERT_MSB:`VIZ_PWR_ALERT_LSB] = 8'h00;

            // Extension, navigation, and wind evidence for the diagnostics page.
            viz_bundle_pix[`VIZ_EXT_VALID_BIT] = 1'b1;
            viz_bundle_pix[`VIZ_EXT_STATUS_MSB:`VIZ_EXT_STATUS_LSB] = 8'h00;
            viz_bundle_pix[`VIZ_EXT_PRESENT_MSB:`VIZ_EXT_PRESENT_LSB] =
                (16'd1 << `EXT_PRESENT_RANGE_BIT) |
                (16'd1 << `EXT_PRESENT_AIR_BIT) |
                (16'd1 << `EXT_PRESENT_ENV_BIT) |
                (16'd1 << `EXT_PRESENT_SUN_BIT) |
                (16'd1 << `EXT_PRESENT_FLOW_BIT);
            viz_bundle_pix[`VIZ_EXT_FAULT_MSB:`VIZ_EXT_FAULT_LSB] = 16'd0;
            viz_bundle_pix[`VIZ_EXT_MAG_DELTA_L1_MSB:`VIZ_EXT_MAG_DELTA_L1_LSB] = 16'd18;
            viz_bundle_pix[`VIZ_EXT_MAG_NORM0_MSB:`VIZ_EXT_MAG_NORM0_LSB] = 16'd960;
            viz_bundle_pix[`VIZ_EXT_MAG_NORM1_MSB:`VIZ_EXT_MAG_NORM1_LSB] = 16'd972;
            viz_bundle_pix[`VIZ_EXT_RNG_HEIGHT_CM_MSB:`VIZ_EXT_RNG_HEIGHT_CM_LSB] = 16'd185;
            viz_bundle_pix[`VIZ_EXT_AIR_DP_PA_MSB:`VIZ_EXT_AIR_DP_PA_LSB] = 16'd42;
            viz_bundle_pix[`VIZ_EXT_AIR_SPEED_CMS_MSB:`VIZ_EXT_AIR_SPEED_CMS_LSB] = 16'd1250;
            viz_bundle_pix[`VIZ_EXT_ENV_TEMP_CDEG_MSB:`VIZ_EXT_ENV_TEMP_CDEG_LSB] = 16'd2345;
            viz_bundle_pix[`VIZ_EXT_ENV_RH_CENTI_MSB:`VIZ_EXT_ENV_RH_CENTI_LSB] = 16'd4520;
            viz_bundle_pix[`VIZ_EXT_SUN_LUMA_MSB:`VIZ_EXT_SUN_LUMA_LSB] = 16'd8192;
            viz_bundle_pix[`VIZ_EXT_FLOW_DX_MSB:`VIZ_EXT_FLOW_DX_LSB] = 16'd12;
            viz_bundle_pix[`VIZ_EXT_FLOW_DY_MSB:`VIZ_EXT_FLOW_DY_LSB] = 16'hFFF8;
            viz_bundle_pix[`VIZ_EXT_LOG_SEQ_MSB:`VIZ_EXT_LOG_SEQ_LSB] = 16'h4001;
            viz_bundle_pix[`VIZ_EXT_LOG_DROP_MSB:`VIZ_EXT_LOG_DROP_LSB] = 16'd0;
            viz_bundle_pix[`VIZ_EXT_MAX_AGE_MS_MSB:`VIZ_EXT_MAX_AGE_MS_LSB] = 16'd44;

            viz_bundle_pix[`VIZ_NAV_VALID_BIT] = 1'b1;
            viz_bundle_pix[`VIZ_NAV_STATUS_MSB:`VIZ_NAV_STATUS_LSB] = 8'h00;
            viz_bundle_pix[`VIZ_NAV_FLAGS_MSB:`VIZ_NAV_FLAGS_LSB] = 8'h03;
            viz_bundle_pix[`VIZ_NAV_DOWNRANGE_M_MSB:`VIZ_NAV_DOWNRANGE_M_LSB] = 16'd40;
            viz_bundle_pix[`VIZ_NAV_CROSSRANGE_M_MSB:`VIZ_NAV_CROSSRANGE_M_LSB] = 16'd8;
            viz_bundle_pix[`VIZ_NAV_AGE_MS_MSB:`VIZ_NAV_AGE_MS_LSB] = 16'd20;

            viz_bundle_pix[`VIZ_WIND_VALID_BIT] = 1'b1;
            viz_bundle_pix[`VIZ_WIND_STATUS_MSB:`VIZ_WIND_STATUS_LSB] = 8'h00;
            viz_bundle_pix[`VIZ_WIND_X_CMS_MSB:`VIZ_WIND_X_CMS_LSB] = 16'd160;
            viz_bundle_pix[`VIZ_WIND_Y_CMS_MSB:`VIZ_WIND_Y_CMS_LSB] = 16'hFFC0;
            viz_bundle_pix[`VIZ_WIND_Z_CMS_MSB:`VIZ_WIND_Z_CMS_LSB] = 16'd64;
            viz_bundle_pix[`VIZ_WIND_AGE_MS_MSB:`VIZ_WIND_AGE_MS_LSB] = 16'd18;

            // Platform health
            viz_bundle_pix[`VIZ_I2C_NACK_MSB:`VIZ_I2C_NACK_LSB] = 16'd0;
            viz_bundle_pix[`VIZ_I2C_TMO_MSB:`VIZ_I2C_TMO_LSB]   = 16'd0;
            viz_bundle_pix[`VIZ_TXN_RATE_MSB:`VIZ_TXN_RATE_LSB] = 16'd50;
            viz_bundle_pix[`VIZ_CDC_UPD_MSB:`VIZ_CDC_UPD_LSB]   = 32'd12345;
            viz_bundle_pix[`VIZ_BUILD_ID_MSB:`VIZ_BUILD_ID_LSB] = 32'h00003F7C;
            viz_bundle_pix[`VIZ_SCHEMA_MSB:`VIZ_SCHEMA_LSB]     = 16'h1A2B;
        end
    endtask

    task load_denied_authority_bundle;
        begin
            load_nominal_bundle();
            viz_bundle_pix[`VIZ_BMP_SEQ_MSB:`VIZ_BMP_SEQ_LSB] = 16'h007B;
            viz_bundle_pix[`VIZ_DER_BMP_SEQ_REF_MSB:`VIZ_DER_BMP_SEQ_REF_LSB] = 16'h007B;
            viz_bundle_pix[`VIZ_AUTH_SERVO_US_MSB:`VIZ_AUTH_SERVO_US_LSB] = 12'd1000;
            viz_bundle_pix[`VIZ_AUTH_GATE_MSB:`VIZ_AUTH_GATE_LSB] = 7'h2D;
        end
    endtask

    task commit_bundle;
        begin
            @(posedge pix_clk);
            viz_update_pix <= 1'b1;
            @(posedge pix_clk);
            viz_update_pix <= 1'b0;
        end
    endtask

    task commit_wr_ptr;
        input [9:0] ptr;
        begin
            @(posedge pix_clk);
            wr_ptr_pix        <= ptr;
            wr_ptr_update_pix <= 1'b1;
            @(posedge pix_clk);
            wr_ptr_update_pix <= 1'b0;
        end
    endtask

    task wait_committed_bundle_seq;
        input [15:0] seq;
        integer watchdog;
        begin
            watchdog = 0;
            while (u_dut.viz_q[`VIZ_BMP_SEQ_MSB:`VIZ_BMP_SEQ_LSB] !== seq) begin
                @(posedge pix_clk);
                watchdog = watchdog + 1;
                if (watchdog > 1000000) begin
                    $display("TB FAIL: committed visualization bundle timeout at t=%0t", $time);
                    $finish;
                end
            end

            repeat (4) @(posedge pix_clk);
        end
    endtask

    task wait_committed_nominal_bundle;
        begin
            wait_committed_bundle_seq(16'h007A);
        end
    endtask

    //--------------------------------------------------------------------------
    // Wait helpers
    //--------------------------------------------------------------------------
    task wait_q1_pixel;
        input [10:0] x_tgt;
        input [10:0] y_tgt;
        input        active_tgt;
        integer      watchdog;
        begin
            watchdog = 0;
            while (!((u_dut.x_q1 == x_tgt) &&
                     (u_dut.y_q1 == y_tgt) &&
                     (u_dut.active_q1 == active_tgt))) begin
                @(posedge pix_clk);
                watchdog = watchdog + 1;
                if (watchdog > 1000000) begin
                    $display("TB FAIL: wait_q1_pixel timeout at t=%0t", $time);
                    $finish;
                end
            end
            @(posedge pix_clk);
        end
    endtask

    task wait_for_any_inactive_q1;
        integer watchdog;
        begin
            watchdog = 0;
            while (u_dut.active_q1 !== 1'b0) begin
                @(posedge pix_clk);
                watchdog = watchdog + 1;
                if (watchdog > 1000000) begin
                    $display("TB FAIL: wait_for_any_inactive_q1 timeout at t=%0t", $time);
                    $finish;
                end
            end
            @(posedge pix_clk);
        end
    endtask

    //--------------------------------------------------------------------------
    // Checks
    //--------------------------------------------------------------------------
    task check_equal_12;
        input [11:0] got;
        input [11:0] exp;
        input [255:0] msg;
        begin
            if (got !== exp) begin
                $display("TB FAIL: %0s  got=%03h exp=%03h  t=%0t", msg, got, exp, $time);
                $finish;
            end else begin
                $display("TB PASS: %0s  value=%03h  t=%0t", msg, got, $time);
            end
        end
    endtask

    task check_equal_1;
        input got;
        input exp;
        input [255:0] msg;
        begin
            if (got !== exp) begin
                $display("TB FAIL: %0s  got=%0b exp=%0b  t=%0t", msg, got, exp, $time);
                $finish;
            end else begin
                $display("TB PASS: %0s  value=%0b  t=%0t", msg, got, $time);
            end
        end
    endtask

    task check_overlay_priority_and_packing;
        begin
            // Target pixel:
            //   compact top BMP object origin = (20,6)
            //   first glyph 'B'
            //   local row 0 / col 0 is lit
            //
            // Expected:
            //   telemetry_overlay_on    = 1
            //   telemetry_overlay_rgb24 = 24'h70D070
            //   telemetry_overlay_rgb12 = 12'h7D7
            //   base scene beneath text = compact BMP health pill
            //   final vga_rgb           = 12'h7D7
            wait_q1_pixel(11'd20, 11'd6, 1'b1);

            check_equal_1(u_dut.telemetry_overlay_on, 1'b1,
                          "overlay asserted on known BMP glyph pixel");

            if (u_dut.telemetry_overlay_rgb_24 !== 24'h70D070) begin
                $display("TB FAIL: overlay rgb24 mismatch  got=%06h exp=%06h  t=%0t",
                         u_dut.telemetry_overlay_rgb_24, 24'h70D070, $time);
                $finish;
            end else begin
                $display("TB PASS: overlay rgb24 packed source matches  value=%06h  t=%0t",
                         u_dut.telemetry_overlay_rgb_24, $time);
            end

            check_equal_12(u_dut.telemetry_overlay_rgb_12, 12'h7D7,
                           "24-bit to 12-bit telemetry packing");

            check_equal_12(u_dut.rgb_base_q, 12'h2B5,
                           "base scene beneath text is compact BMP health pill");

            check_equal_12(vga_rgb, 12'h7D7,
                           "final mux gives overlay priority over base scene");
        end
    endtask

    task check_top_band_background_without_overlay;
        begin
            // Chosen pixel lies in top band and safely before the BMP text
            // object origin. wait_q1_pixel advances one clock after matching,
            // so keep this target more than one pixel away from x=20.
            // Background is flat 12'h111 there.
            wait_q1_pixel(11'd2, 11'd8, 1'b1);

            check_equal_1(u_dut.telemetry_overlay_on, 1'b0,
                          "overlay inactive on nearby top-band background pixel");

            check_equal_12(vga_rgb, 12'h111,
                           "top-band background passes through when overlay is inactive");
        end
    endtask

    task check_panel_title_and_clip;
        begin
            // Left panel title object origin = (10,30), first glyph 'A'.
            wait_q1_pixel(11'd10, 11'd30, 1'b1);
            check_equal_1(u_dut.telemetry_overlay_on, 1'b1,
                          "left panel title overlay is visible");
            check_equal_12(vga_rgb, 12'h8DF,
                           "left panel title renders in dedicated title color");

            // The text compositor clips all panel text at y<=105. This pixel is
            // below the telemetry band and before the tape panel begins.
            wait_q1_pixel(11'd10, 11'd106, 1'b1);
            check_equal_1(u_dut.telemetry_overlay_on, 1'b0,
                          "panel text is clipped before instrument region");
        end
    endtask

    task check_inactive_video_blacking;
        begin
            wait_for_any_inactive_q1();
            check_equal_12(vga_rgb, 12'h000,
                           "inactive video is forced to black");
        end
    endtask

    task check_chart_newest_sample_markers;
        begin
            wait_q1_pixel(11'd639, 11'd370, 1'b1);
            check_equal_12(vga_rgb, 12'h666,
                           "altitude chart newest-sample cursor is visible");

            wait_q1_pixel(11'd639, 11'd389, 1'b1);
            check_equal_12(vga_rgb, 12'hFFF,
                           "altitude chart newest-sample endpoint halo is visible");

            wait_q1_pixel(11'd639, 11'd436, 1'b1);
            check_equal_12(vga_rgb, 12'hFFF,
                           "vertical-speed chart newest-sample endpoint halo is visible");
        end
    endtask

    task check_vertical_vector_instrument;
        begin
            // Nominal bundle roll=12 deg maps into octant 0 in the renderer,
            // so the explicit roll-plane vertical vector points right from the
            // panel center at (319,332). Text overlay is not active in this
            // lower instrument area, so final VGA RGB should equal base scene.
            wait_q1_pixel(11'd331, 11'd332, 1'b1);
            check_equal_1(u_dut.telemetry_overlay_on, 1'b0,
                          "overlay inactive over vertical-vector shaft");
            check_equal_12(vga_rgb, 12'h3FF,
                           "vertical-vector shaft renders cyan when roll evidence is valid");

            wait_q1_pixel(11'd319, 11'd332, 1'b1);
            check_equal_12(vga_rgb, 12'hFFF,
                           "vertical-vector center reference dot is visible");
        end
    endtask

    task check_instrument_centers_unobscured;
        begin
            wait_q1_pixel(11'd319, 11'd205, 1'b1);
            check_equal_1(u_dut.telemetry_overlay_on, 1'b0,
                          "overlay inactive at artificial-horizon center");
            check_equal_12(vga_rgb, 12'hFFF,
                           "artificial-horizon center reference remains visible");

            wait_q1_pixel(11'd532, 11'd205, 1'b1);
            check_equal_1(u_dut.telemetry_overlay_on, 1'b0,
                          "overlay inactive at compass center");
            check_equal_12(vga_rgb, 12'hFFF,
                           "compass center reference remains visible");
        end
    endtask

    task check_apogee_authority_envelope;
        begin
            // Fixed-point expectations for the nominal authority fixture:
            // target 304800 cm -> y=199 with the shift-only ladder scale.
            // no-brake apogee 321416 cm -> y=191.
            // command level is 1/4 because the SYS policy published u8=64.
            wait_q1_pixel(11'd172, 11'd199, 1'b1);
            check_equal_1(u_dut.telemetry_overlay_on, 1'b0,
                          "overlay inactive over apogee target marker");
            check_equal_12(vga_rgb, 12'hF0F,
                           "apogee authority target marker renders magenta");

            wait_q1_pixel(11'd192, 11'd191, 1'b1);
            check_equal_12(vga_rgb, 12'hFD0,
                           "no-brake apogee prediction marker renders yellow");

            wait_q1_pixel(11'd178, 11'd198, 1'b1);
            check_equal_12(vga_rgb, 12'h86F,
                           "authority uncertainty boundary renders blue edge");

            wait_q1_pixel(11'd193, 11'd224, 1'b1);
            check_equal_12(vga_rgb, 12'h143,
                           "authority prediction corridor renders reachable physics band");

            wait_q1_pixel(11'd174, 11'd342, 1'b1);
            check_equal_12(vga_rgb, 12'h2F6,
                           "authority per-gate safety cell renders enabled state");

            wait_q1_pixel(11'd199, 11'd341, 1'b1);
            check_equal_12(vga_rgb, 12'h2F6,
                           "authority command bar renders reachable command fill");
        end
    endtask

    task check_apogee_gate_denied_indicator;
        begin
            wait_q1_pixel(11'd174, 11'd342, 1'b1);
            check_equal_12(vga_rgb, 12'hF20,
                           "authority per-gate safety cell renders denied actuator demand");
        end
    endtask

    task check_sensor_diagnostics_page;
        begin
            vga_page_select_pix = 2'd1;

            wait_q1_pixel(11'd20, 11'd6, 1'b1);
            check_equal_12(vga_rgb, 12'h112,
                           "diagnostics page suppresses HUD text overlay");

            wait_q1_pixel(11'd20, 11'd41, 1'b1);
            check_equal_12(vga_rgb, 12'h2F6,
                           "sensor evidence matrix renders BMP OK cell");

            wait_q1_pixel(11'd160, 11'd41, 1'b1);
            check_equal_12(vga_rgb, 12'h2AF,
                           "sensor evidence matrix renders BMP age bar");

            wait_q1_pixel(11'd440, 11'd57, 1'b1);
            check_equal_12(vga_rgb, 12'h2F6,
                           "extension scale renders rangefinder height bar");

            wait_q1_pixel(11'd487, 11'd329, 1'b1);
            check_equal_12(vga_rgb, 12'h2F6,
                           "diagnostics page renders current nav point");

            wait_q1_pixel(11'd489, 11'd353, 1'b1);
            check_equal_12(vga_rgb, 12'hFFF,
                           "diagnostics page renders current wind tip");

            vga_page_select_pix = 2'd0;
            repeat (2) @(posedge pix_clk);
        end
    endtask

    task check_bottom_debug_strip;
        begin
            // Bottom engineering line origin = (10,470). The third character
            // is '0' and has adjacent lit pixels at 1x scale, which keeps both
            // the render-stage pixel and current combinational overlay probe
            // asserted after wait_q1_pixel advances one clock.
            wait_q1_pixel(11'd18, 11'd470, 1'b1);
            check_equal_1(u_dut.telemetry_overlay_on, 1'b1,
                          "bottom debug strip renders with compact 1x text");
            check_equal_12(vga_rgb, 12'hBCD,
                           "bottom debug strip uses muted engineering color");

            wait_q1_pixel(11'd10, 11'd448, 1'b1);
            check_equal_1(u_dut.telemetry_overlay_on, 1'b0,
                          "bottom debug text does not occupy the chart body");
        end
    endtask

    //--------------------------------------------------------------------------
    // Main sequence
    //--------------------------------------------------------------------------
    initial begin
        // Memory init
        for (i = 0; i < 1024; i = i + 1) begin
            alt_mem[i]  = 16'd0;
            vspd_mem[i] = 16'd0;
        end
        alt_mem[1023]  = 16'd1024;
        vspd_mem[1023] = 16'd32;

        pix_rst           = 1'b1;
        viz_bundle_pix    = {`VIZ_BUNDLE_W{1'b0}};
        viz_update_pix    = 1'b0;
        vga_page_select_pix = 2'd0;
        wr_ptr_pix        = 10'd0;
        wr_ptr_update_pix = 1'b0;
        alt_rd_data       = 16'd0;
        vspd_rd_data      = 16'd0;

        repeat (8) @(posedge pix_clk);
        pix_rst = 1'b0;

        load_nominal_bundle();
        commit_bundle();
        commit_wr_ptr(10'd0);
        wait_committed_nominal_bundle();

        // 1) Background pass-through in active video, no overlay.
        check_top_band_background_without_overlay();

        // 2) Overlay priority and exact color packing on a known glyph pixel.
        check_overlay_priority_and_packing();

        // 3) Fixed panel titles and clipping boundaries.
        check_panel_title_and_clip();

        // 4) Newest-sample markers are visible on both history charts.
        check_chart_newest_sample_markers();

        // 5) Attitude/vertical-vector instrument is present in active video.
        check_vertical_vector_instrument();
        check_instrument_centers_unobscured();

        // 6) Apogee authority ladder is present in active video.
        check_apogee_authority_envelope();

        load_denied_authority_bundle();
        commit_bundle();
        wait_committed_bundle_seq(16'h007B);
        check_apogee_gate_denied_indicator();

        // 7) Page-selectable sensor diagnostics render from committed evidence.
        check_sensor_diagnostics_page();

        // 8) Engineering counters are demoted to the bottom debug strip.
        check_bottom_debug_strip();

        // 9) Outside active video, black output is required.
        check_inactive_video_blacking();

        $display("TB PASS: all flight_visualizer_pix overlay-path checks completed.");
        $finish;
    end

endmodule

`default_nettype wire
