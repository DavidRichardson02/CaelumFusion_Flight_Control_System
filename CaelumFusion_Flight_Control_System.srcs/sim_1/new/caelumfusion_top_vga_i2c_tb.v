`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// caelumfusion_top_vga_i2c_tb
//------------------------------------------------------------------------------
// Behavioral top-level testbench for:
//   caelumfusion_top_vga_i2c
//
// Testbench scope:
//   - 100 MHz clock generation
//   - deterministic reset sequencing
//   - open-drain I2C bus with passive pull-ups
//   - simple but protocol-correct sensor stubs for BMP58x/LIS2MDL/LIS3DH
//   - VGA activity observation
//   - SYS-domain snapshot observation through hierarchical probes
//   - coarse health checks against all-X / all-Z collapse
//
// Notes:
//   - This testbench is Verilog-2001 only.
//   - Sensor models are transaction-level responders, not register-accurate.
//   - The simulation source file should use the patched top-level instantiation
//     in which u_sys_sensors receives pix_clk and pix_rst.
//==============================================================================
module caelumfusion_top_vga_i2c_tb;

    //-------------------------------------------------------------------------
    // Global timing
    //-------------------------------------------------------------------------
    parameter integer CLK_PERIOD_NS    = 10;         // 100 MHz
    parameter integer RESET_CYCLES     = 64;
    parameter integer EARLY_SETTLE_CYC = 100000;     // 1 ms
    parameter integer MAIN_RUN_CYC     = 15000000;   // 150 ms

    parameter integer MIN_HSYNC_EDGES  = 1000;
    parameter integer MIN_VSYNC_EDGES  = 10;
    parameter integer MIN_I2C_STARTS   = 3;

    //-------------------------------------------------------------------------
    // DUT I/O
    //-------------------------------------------------------------------------
    reg         clk;
    reg         rst;
    reg         sw_arm_raw;
    reg         sw_policy_enable_raw;
    reg         btn_page_raw;
    tri1        scl;
    tri1        sda;
    wire        cls_tx;
    wire        vga_hsync;
    wire        vga_vsync;
    wire [11:0] vga_rgb;

    pullup u_pullup_scl (scl);
    pullup u_pullup_sda (sda);

    //-------------------------------------------------------------------------
    // DUT
    //-------------------------------------------------------------------------
    caelumfusion_top_vga_i2c dut (
        .clk       (clk),
        .rst       (rst),
        .sw_arm_raw(sw_arm_raw),
        .sw_policy_enable_raw(sw_policy_enable_raw),
        .scl       (scl),
        .sda       (sda),
        .btn_page_raw(btn_page_raw),
        .cls_tx    (cls_tx),
        .vga_hsync (vga_hsync),
        .vga_vsync (vga_vsync),
        .vga_rgb   (vga_rgb)
    );

    //-------------------------------------------------------------------------
    // Sensor stubs
    //-------------------------------------------------------------------------
    i2c_simple_sensor_model #(
        .I2C_ADDR   (7'h47),
        .READ_BYTE0 (8'h12),
        .READ_BYTE1 (8'h34),
        .READ_BYTE2 (8'h56),
        .READ_BYTE3 (8'h78),
        .READ_BYTE4 (8'h9A),
        .READ_BYTE5 (8'hBC)
    ) u_bmp585 (
        .scl (scl),
        .sda (sda)
    );

    i2c_simple_sensor_model #(
        .I2C_ADDR   (7'h30),
        .READ_BYTE0 (8'h21),
        .READ_BYTE1 (8'h43),
        .READ_BYTE2 (8'h65),
        .READ_BYTE3 (8'h87),
        .READ_BYTE4 (8'hA9),
        .READ_BYTE5 (8'hCB)
    ) u_lis2mdl (
        .scl (scl),
        .sda (sda)
    );

    i2c_simple_sensor_model #(
        .I2C_ADDR   (7'h18),
        .READ_BYTE0 (8'h10),
        .READ_BYTE1 (8'h32),
        .READ_BYTE2 (8'h54),
        .READ_BYTE3 (8'h76),
        .READ_BYTE4 (8'h98),
        .READ_BYTE5 (8'hBA)
    ) u_lis3dh (
        .scl (scl),
        .sda (sda)
    );

    //-------------------------------------------------------------------------
    // Clock / reset
    //-------------------------------------------------------------------------
    initial begin
        clk = 1'b0;
        sw_arm_raw = 1'b0;
        sw_policy_enable_raw = 1'b0;
        btn_page_raw = 1'b0;
        forever #(CLK_PERIOD_NS/2) clk = ~clk;
    end

    initial begin
        rst = 1'b1;
        repeat (RESET_CYCLES) @(posedge clk);
        rst = 1'b0;
    end

    //-------------------------------------------------------------------------
    // Global monitors
    //-------------------------------------------------------------------------
    integer hsync_edges;
    integer vsync_edges;
    integer i2c_start_events;
    integer i2c_stop_events;
    integer sim_error_count;
    integer nonzero_rgb_samples;

    initial begin
        hsync_edges         = 0;
        vsync_edges         = 0;
        i2c_start_events    = 0;
        i2c_stop_events     = 0;
        sim_error_count     = 0;
        nonzero_rgb_samples = 0;
    end

    always @(negedge vga_hsync) begin
        if (!rst)
            hsync_edges = hsync_edges + 1;
    end

    always @(negedge vga_vsync) begin
        if (!rst)
            vsync_edges = vsync_edges + 1;
    end

    always @(negedge sda) begin
        if ((!rst) && (scl === 1'b1))
            i2c_start_events = i2c_start_events + 1;
    end

    always @(posedge sda) begin
        if ((!rst) && (scl === 1'b1))
            i2c_stop_events = i2c_stop_events + 1;
    end

    always @(posedge clk) begin
        if ((!rst) && bus12_is_known(vga_rgb) && (vga_rgb != 12'h000))
            nonzero_rgb_samples = nonzero_rgb_samples + 1;
    end

    //-------------------------------------------------------------------------
    // Helper functions
    //-------------------------------------------------------------------------
    function bit_is_known;
        input value;
        begin
            if ((value === 1'b0) || (value === 1'b1))
                bit_is_known = 1'b1;
            else
                bit_is_known = 1'b0;
        end
    endfunction

    function bus8_is_known;
        input [7:0] value;
        integer i;
        begin
            bus8_is_known = 1'b1;
            for (i = 0; i < 8; i = i + 1) begin
                if (!((value[i] === 1'b0) || (value[i] === 1'b1)))
                    bus8_is_known = 1'b0;
            end
        end
    endfunction

    function bus12_is_known;
        input [11:0] value;
        integer i;
        begin
            bus12_is_known = 1'b1;
            for (i = 0; i < 12; i = i + 1) begin
                if (!((value[i] === 1'b0) || (value[i] === 1'b1)))
                    bus12_is_known = 1'b0;
            end
        end
    endfunction

    function bus16_is_known;
        input [15:0] value;
        integer i;
        begin
            bus16_is_known = 1'b1;
            for (i = 0; i < 16; i = i + 1) begin
                if (!((value[i] === 1'b0) || (value[i] === 1'b1)))
                    bus16_is_known = 1'b0;
            end
        end
    endfunction

    function bus32_is_known;
        input [31:0] value;
        integer i;
        begin
            bus32_is_known = 1'b1;
            for (i = 0; i < 32; i = i + 1) begin
                if (!((value[i] === 1'b0) || (value[i] === 1'b1)))
                    bus32_is_known = 1'b0;
            end
        end
    endfunction

    function bus48_is_known;
        input [47:0] value;
        integer i;
        begin
            bus48_is_known = 1'b1;
            for (i = 0; i < 48; i = i + 1) begin
                if (!((value[i] === 1'b0) || (value[i] === 1'b1)))
                    bus48_is_known = 1'b0;
            end
        end
    endfunction

    task fail_msg;
        input [8*120-1:0] msg;
        begin
            sim_error_count = sim_error_count + 1;
            $display("TB_FAIL: %s @ %t", msg, $time);
        end
    endtask

    task info_msg;
        input [8*120-1:0] msg;
        begin
            $display("TB_INFO: %s @ %t", msg, $time);
        end
    endtask

    //-------------------------------------------------------------------------
    // Main sequence
    //-------------------------------------------------------------------------
    initial begin
        info_msg("simulation start");

        @(negedge rst);
        info_msg("reset released");

        repeat (EARLY_SETTLE_CYC) @(posedge clk);

        // Early structural sanity
        if (!bit_is_known(vga_hsync))
            fail_msg("vga_hsync unresolved after early settle window");
        if (!bit_is_known(vga_vsync))
            fail_msg("vga_vsync unresolved after early settle window");
        if (!bus12_is_known(vga_rgb))
            fail_msg("vga_rgb contains X/Z after early settle window");
        if (!bit_is_known(scl))
            fail_msg("scl unresolved after early settle window");
        if (!bit_is_known(sda))
            fail_msg("sda unresolved after early settle window");

        // Internal timebase sanity
        if (!bus32_is_known(dut.u_sys_sensors.u_time.time_us))
            fail_msg("time_us unresolved after early settle window");
        if (dut.u_sys_sensors.u_time.time_us == 32'd0)
            fail_msg("time_us failed to advance from zero after early settle window");

        // Snapshot buses should at least resolve even before first commits.
        if (!bus16_is_known(dut.acc_seq))
            fail_msg("acc_seq unresolved after early settle window");
        if (!bus16_is_known(dut.bmp_seq))
            fail_msg("bmp_seq unresolved after early settle window");
        if (!bus16_is_known(dut.mag_seq))
            fail_msg("mag_seq unresolved after early settle window");
        if (!bus48_is_known(dut.acc_payload))
            fail_msg("acc_payload unresolved after early settle window");
        if (!bus48_is_known(dut.bmp_payload))
            fail_msg("bmp_payload unresolved after early settle window");
        if (!bus48_is_known(dut.mag_payload))
            fail_msg("mag_payload unresolved after early settle window");

        info_msg("entering main runtime window");
        repeat (MAIN_RUN_CYC) @(posedge clk);

        // Final summary
        $display("TB_SUMMARY: time_us=%d hsync_edges=%d vsync_edges=%d i2c_start_events=%d i2c_stop_events=%d nonzero_rgb_samples=%d",
                 dut.u_sys_sensors.u_time.time_us,
                 hsync_edges,
                 vsync_edges,
                 i2c_start_events,
                 i2c_stop_events,
                 nonzero_rgb_samples);

        $display("TB_SENSOR_COUNTS: bmp_addr_hits=%d bmp_reads=%d bmp_writes=%d | mag_addr_hits=%d mag_reads=%d mag_writes=%d | acc_addr_hits=%d acc_reads=%d acc_writes=%d",
                 u_bmp585.addr_match_count, u_bmp585.read_data_count, u_bmp585.write_byte_count,
                 u_lis2mdl.addr_match_count, u_lis2mdl.read_data_count, u_lis2mdl.write_byte_count,
                 u_lis3dh.addr_match_count, u_lis3dh.read_data_count, u_lis3dh.write_byte_count);

        $display("TB_SNAPSHOTS: bmp_seq=%d bmp_valid=%b bmp_status=0x%02h bmp_payload=0x%012h",
                 dut.bmp_seq, dut.bmp_valid, dut.bmp_status, dut.bmp_payload);
        $display("TB_SNAPSHOTS: acc_seq=%d acc_valid=%b acc_status=0x%02h acc_payload=0x%012h",
                 dut.acc_seq, dut.acc_valid, dut.acc_status, dut.acc_payload);
        $display("TB_SNAPSHOTS: mag_seq=%d mag_valid=%b mag_status=0x%02h mag_payload=0x%012h",
                 dut.mag_seq, dut.mag_valid, dut.mag_status, dut.mag_payload);

        // Final pass/fail checks
        if (hsync_edges < MIN_HSYNC_EDGES)
            fail_msg("insufficient hsync activity observed");
        if (vsync_edges < MIN_VSYNC_EDGES)
            fail_msg("insufficient vsync activity observed");
        if (i2c_start_events < MIN_I2C_STARTS)
            fail_msg("insufficient I2C START activity observed");

        if (u_bmp585.addr_match_count < 1)
            fail_msg("BMP58x model was never addressed");
        if (u_lis2mdl.addr_match_count < 1)
            fail_msg("LIS2MDL model was never addressed");
        if (u_lis3dh.addr_match_count < 1)
            fail_msg("LIS3DH model was never addressed");

        if (dut.bmp_seq == 16'd0)
            fail_msg("bmp_seq never incremented");
        if (dut.acc_seq == 16'd0)
            fail_msg("acc_seq never incremented");
        if (dut.mag_seq == 16'd0)
            fail_msg("mag_seq never incremented");

        if (dut.bmp_valid !== 1'b1)
            fail_msg("bmp_valid did not settle high by end of run");
        if (dut.acc_valid !== 1'b1)
            fail_msg("acc_valid did not settle high by end of run");
        if (dut.mag_valid !== 1'b1)
            fail_msg("mag_valid did not settle high by end of run");

        if (!bus8_is_known(dut.bmp_status))
            fail_msg("bmp_status unresolved at end of run");
        if (!bus8_is_known(dut.acc_status))
            fail_msg("acc_status unresolved at end of run");
        if (!bus8_is_known(dut.mag_status))
            fail_msg("mag_status unresolved at end of run");

        if (!bus48_is_known(dut.bmp_payload))
            fail_msg("bmp_payload unresolved at end of run");
        if (!bus48_is_known(dut.acc_payload))
            fail_msg("acc_payload unresolved at end of run");
        if (!bus48_is_known(dut.mag_payload))
            fail_msg("mag_payload unresolved at end of run");

        if (dut.bmp_payload == 48'd0)
            fail_msg("bmp_payload remained zero");
        if (dut.acc_payload == 48'd0)
            fail_msg("acc_payload remained zero");
        if (dut.mag_payload == 48'd0)
            fail_msg("mag_payload remained zero");

        if (nonzero_rgb_samples == 0)
            fail_msg("vga_rgb never produced a nonzero known sample");

        if (sim_error_count == 0)
            $display("TB_RESULT: PASS");
        else
            $display("TB_RESULT: FAIL count=%d", sim_error_count);

        $finish;
    end

endmodule

`default_nettype wire
