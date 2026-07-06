`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// tb_i2c_suite_regression_all3_real_engine
//------------------------------------------------------------------------------
// ROLE
//   Canonical phased regression for the shared-I2C acquisition path using:
//
//     job 0 = lis3dh_job
//     job 1 = bmp585_job
//     job 2 = lis2mdl_job
//
// SHARED COMPONENTS UNDER TEST
//   - i2c_job_mux
//   - i2c_master_engine
//   - one open-drain shared I2C bus
//
// PROVES
//   1) Each job can initialize successfully
//   2) Each job can acquire a sample successfully
//   3) All three jobs share the same mux correctly
//   4) All three jobs share the same real engine correctly
//   5) The common open-drain bus supports all three devices
//
// TEST POLICY
//   The sequence is phased to avoid fixed-priority contention while still
//   validating the exact common transport path:
//
//     Phase A : bmp585_job  init + sample
//     Phase B : lis3dh_job  init + sample
//     Phase C : lis2mdl_job init + sample
//
// IMPORTANT
//   The current mux ownership capture depends on grant being present at command
//   acceptance. Therefore each phase holds the relevant grant for multiple SYS
//   clocks rather than issuing a one-cycle pulse.
//==============================================================================
module tb_i2c_suite_regression_all3_real_engine;

    //--------------------------------------------------------------------------
    // Global timing
    //--------------------------------------------------------------------------
    parameter integer CLK_HZ          = 100_000_000;
    parameter integer CLK_PERIOD_NS   = 10;
    parameter integer US_DIV_CYCLES   = 100;        // 100 MHz -> 1 us
    parameter integer WAIT_LIMIT_CYC  = 2_000_000;

    //--------------------------------------------------------------------------
    // Clock / reset
    //--------------------------------------------------------------------------
    reg clk;
    reg rst;

    //--------------------------------------------------------------------------
    // Timebase model
    //--------------------------------------------------------------------------
    reg        tick_1us;
    reg [31:0] time_us;
    integer    us_div_ctr;

    //--------------------------------------------------------------------------
    // Epochs / grants
    //--------------------------------------------------------------------------
    reg epoch_100hz;
    reg epoch_50hz;
    reg epoch_10hz;

    reg grant_lis3dh;
    reg grant_bmp585;
    reg grant_lis2mdl;

    //--------------------------------------------------------------------------
    // Job 0 : lis3dh_job
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
    wire        j0_busy;

    wire        j0_snap_commit;
    wire        j0_snap_valid_in;
    wire [7:0]  j0_snap_status_in;
    wire [47:0] j0_snap_payload_in;
    wire        j0_init_done;

    //--------------------------------------------------------------------------
    // Job 1 : bmp585_job
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
    wire        j1_busy;

    wire        j1_snap_commit;
    wire        j1_snap_valid_in;
    wire [7:0]  j1_snap_status_in;
    wire [47:0] j1_snap_payload_in;
    wire        j1_init_done;

    //--------------------------------------------------------------------------
    // Job 2 : lis2mdl_job
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
    wire        j2_busy;

    wire        j2_snap_commit;
    wire        j2_snap_valid_in;
    wire [7:0]  j2_snap_status_in;
    wire [47:0] j2_snap_payload_in;
    wire        j2_init_done;

    //--------------------------------------------------------------------------
    // Shared engine-side contract
    //--------------------------------------------------------------------------
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

    //--------------------------------------------------------------------------
    // Shared open-drain bus
    //--------------------------------------------------------------------------
    tri1 scl;
    tri1 sda;

    pullup u_pullup_scl (scl);
    pullup u_pullup_sda (sda);

    //--------------------------------------------------------------------------
    // DUT jobs
    //--------------------------------------------------------------------------

    // job 0 = lis3dh_job
    lis3dh_job u_lis3dh_job (
        .clk             (clk),
        .rst             (rst),
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
        .busy            (j0_busy),

        .snap_commit     (j0_snap_commit),
        .snap_valid_in   (j0_snap_valid_in),
        .snap_status_in  (j0_snap_status_in),
        .snap_payload_in (j0_snap_payload_in),

        .init_done       (j0_init_done)
    );

    // job 1 = bmp585_job

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
            .busy            (j1_busy),

            .snap_commit     (j1_snap_commit),
            .snap_valid_in   (j1_snap_valid_in),
            .snap_status_in  (j1_snap_status_in),
            .snap_payload_in (j1_snap_payload_in),

            .init_done       (j1_init_done)
        );




    // job 2 = lis2mdl_job
    lis2mdl_job u_lis2mdl_job (
        .clk             (clk),
        .rst             (rst),
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
        .busy            (j2_busy),

        .snap_commit     (j2_snap_commit),
        .snap_valid_in   (j2_snap_valid_in),
        .snap_status_in  (j2_snap_status_in),
        .snap_payload_in (j2_snap_payload_in),

        .init_done       (j2_init_done)
    );

    //--------------------------------------------------------------------------
    // Shared mux
    //--------------------------------------------------------------------------
    i2c_job_mux u_mux (
        .clk               (clk),
        .rst               (rst),

        .grant_lis3dh      (grant_lis3dh),
        .grant_bmp585      (grant_bmp585),
        .grant_lis2mdl     (grant_lis2mdl),
        .grant_pmon1       (1'b0),

        // job 0
        .j0_cmd_valid      (j0_cmd_valid),
        .j0_cmd_ready      (j0_cmd_ready),
        .j0_cmd_addr7      (j0_cmd_addr7),
        .j0_cmd_wlen       (j0_cmd_wlen),
        .j0_cmd_rlen       (j0_cmd_rlen),
        .j0_cmd_repstart   (j0_cmd_repstart),
        .j0_cmd_timeout_us (j0_cmd_timeout_us),
        .j0_w_valid        (j0_w_valid),
        .j0_w_ready        (j0_w_ready),
        .j0_w_data         (j0_w_data),
        .j0_w_last         (j0_w_last),
        .j0_r_valid        (j0_r_valid),
        .j0_r_ready        (j0_r_ready),
        .j0_r_data         (j0_r_data),
        .j0_r_last         (j0_r_last),
        .j0_done           (j0_done),
        .j0_done_code      (j0_done_code),
        .j0_busy           (j0_busy),

        // job 1
        .j1_cmd_valid      (j1_cmd_valid),
        .j1_cmd_ready      (j1_cmd_ready),
        .j1_cmd_addr7      (j1_cmd_addr7),
        .j1_cmd_wlen       (j1_cmd_wlen),
        .j1_cmd_rlen       (j1_cmd_rlen),
        .j1_cmd_repstart   (j1_cmd_repstart),
        .j1_cmd_timeout_us (j1_cmd_timeout_us),
        .j1_w_valid        (j1_w_valid),
        .j1_w_ready        (j1_w_ready),
        .j1_w_data         (j1_w_data),
        .j1_w_last         (j1_w_last),
        .j1_r_valid        (j1_r_valid),
        .j1_r_ready        (j1_r_ready),
        .j1_r_data         (j1_r_data),
        .j1_r_last         (j1_r_last),
        .j1_done           (j1_done),
        .j1_done_code      (j1_done_code),
        .j1_busy           (j1_busy),

        // job 2
        .j2_cmd_valid      (j2_cmd_valid),
        .j2_cmd_ready      (j2_cmd_ready),
        .j2_cmd_addr7      (j2_cmd_addr7),
        .j2_cmd_wlen       (j2_cmd_wlen),
        .j2_cmd_rlen       (j2_cmd_rlen),
        .j2_cmd_repstart   (j2_cmd_repstart),
        .j2_cmd_timeout_us (j2_cmd_timeout_us),
        .j2_w_valid        (j2_w_valid),
        .j2_w_ready        (j2_w_ready),
        .j2_w_data         (j2_w_data),
        .j2_w_last         (j2_w_last),
        .j2_r_valid        (j2_r_valid),
        .j2_r_ready        (j2_r_ready),
        .j2_r_data         (j2_r_data),
        .j2_r_last         (j2_r_last),
        .j2_done           (j2_done),
        .j2_done_code      (j2_done_code),
        .j2_busy           (j2_busy),

        // job 3 intentionally inactive in this all-three legacy regression
        .j3_cmd_valid      (1'b0),
        .j3_cmd_ready      (),
        .j3_cmd_addr7      (7'd0),
        .j3_cmd_wlen       (8'd0),
        .j3_cmd_rlen       (8'd0),
        .j3_cmd_repstart   (1'b0),
        .j3_cmd_timeout_us (32'd0),
        .j3_w_valid        (1'b0),
        .j3_w_ready        (),
        .j3_w_data         (8'd0),
        .j3_w_last         (1'b0),
        .j3_r_valid        (),
        .j3_r_ready        (1'b0),
        .j3_r_data         (),
        .j3_r_last         (),
        .j3_done           (),
        .j3_done_code      (),
        .j3_busy           (),

        // engine side
        .e_cmd_valid       (e_cmd_valid),
        .e_cmd_ready       (e_cmd_ready),
        .e_cmd_addr7       (e_cmd_addr7),
        .e_cmd_wlen        (e_cmd_wlen),
        .e_cmd_rlen        (e_cmd_rlen),
        .e_cmd_repstart    (e_cmd_repstart),
        .e_cmd_timeout_us  (e_cmd_timeout_us),

        .e_w_valid         (e_w_valid),
        .e_w_ready         (e_w_ready),
        .e_w_data          (e_w_data),
        .e_w_last          (e_w_last),

        .e_r_valid         (e_r_valid),
        .e_r_ready         (e_r_ready),
        .e_r_data          (e_r_data),
        .e_r_last          (e_r_last),

        .e_done            (e_done),
        .e_done_code       (e_done_code)
    );

    //--------------------------------------------------------------------------
    // Real shared engine
    //--------------------------------------------------------------------------
    i2c_master_engine #(
        .CLK_HZ     (CLK_HZ),
        .I2C_HZ     (100_000),
        .MAX_WBYTES (8),
        .MAX_RBYTES (16)
    ) u_engine (
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

    //--------------------------------------------------------------------------
    // Behavioral slave models on the same shared bus
    //--------------------------------------------------------------------------
    lis3dh_i2c_slave_model u_lis3dh_slave (
        .rst (rst),
        .scl (scl),
        .sda (sda)
    );

    bmp585_i2c_slave_model u_bmp585_slave (
        .rst (rst),
        .scl (scl),
        .sda (sda)
    );

    lis2mdl_i2c_slave_model u_lis2mdl_slave (
        .rst (rst),
        .scl (scl),
        .sda (sda)
    );

    //--------------------------------------------------------------------------
    // Clock generation
    //--------------------------------------------------------------------------
    initial begin
        clk = 1'b0;
        forever #(CLK_PERIOD_NS/2) clk = ~clk;
    end

    //--------------------------------------------------------------------------
    // Microsecond timebase model
    //--------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            us_div_ctr <= 0;
            tick_1us   <= 1'b0;
            time_us    <= 32'd0;
        end else begin
            tick_1us <= 1'b0;

            if (us_div_ctr == (US_DIV_CYCLES - 1)) begin
                us_div_ctr <= 0;
                tick_1us   <= 1'b1;
                time_us    <= time_us + 32'd1;
            end else begin
                us_div_ctr <= us_div_ctr + 1;
            end
        end
    end

    //--------------------------------------------------------------------------
    // Helpers
    //--------------------------------------------------------------------------
    task hold_bmp_grant;
        input integer cycles;
        integer i;
        begin
            grant_bmp585 <= 1'b1;
            for (i = 0; i < cycles; i = i + 1)
                @(posedge clk);
            grant_bmp585 <= 1'b0;
        end
    endtask

    task hold_acc_grant;
        input integer cycles;
        integer i;
        begin
            grant_lis3dh <= 1'b1;
            for (i = 0; i < cycles; i = i + 1)
                @(posedge clk);
            grant_lis3dh <= 1'b0;
        end
    endtask

    task hold_mag_grant;
        input integer cycles;
        integer i;
        begin
            grant_lis2mdl <= 1'b1;
            for (i = 0; i < cycles; i = i + 1)
                @(posedge clk);
            grant_lis2mdl <= 1'b0;
        end
    endtask

    task issue_bmp_epoch_with_grant;
        input integer cycles;
        integer i;
        begin
            epoch_50hz   <= 1'b1;
            grant_bmp585 <= 1'b1;
            @(posedge clk);
            epoch_50hz   <= 1'b0;
            for (i = 1; i < cycles; i = i + 1)
                @(posedge clk);
            grant_bmp585 <= 1'b0;
        end
    endtask

    task issue_acc_epoch_with_grant;
        input integer cycles;
        integer i;
        begin
            epoch_100hz  <= 1'b1;
            grant_lis3dh <= 1'b1;
            @(posedge clk);
            epoch_100hz  <= 1'b0;
            for (i = 1; i < cycles; i = i + 1)
                @(posedge clk);
            grant_lis3dh <= 1'b0;
        end
    endtask

    task issue_mag_epoch_with_grant;
        input integer cycles;
        integer i;
        begin
            epoch_10hz    <= 1'b1;
            grant_lis2mdl <= 1'b1;
            @(posedge clk);
            epoch_10hz    <= 1'b0;
            for (i = 1; i < cycles; i = i + 1)
                @(posedge clk);
            grant_lis2mdl <= 1'b0;
        end
    endtask

    task wait_bmp_init_done;
        integer k;
        begin
            for (k = 0; k < WAIT_LIMIT_CYC; k = k + 1) begin
                @(posedge clk);
                if (j1_init_done)
                    disable wait_bmp_init_done;
            end
            $display("FAIL: BMP585 init timeout");
            $fatal;
        end
    endtask

    task wait_bmp_commit;
        integer k;
        begin
            for (k = 0; k < WAIT_LIMIT_CYC; k = k + 1) begin
                @(posedge clk);
                if (j1_snap_commit)
                    disable wait_bmp_commit;
            end
            $display("FAIL: BMP585 commit timeout");
            $fatal;
        end
    endtask

    task wait_acc_init_done;
        integer k;
        begin
            for (k = 0; k < WAIT_LIMIT_CYC; k = k + 1) begin
                @(posedge clk);
                if (j0_init_done)
                    disable wait_acc_init_done;
            end
            $display("FAIL: LIS3DH init timeout");
            $fatal;
        end
    endtask

    task wait_acc_commit;
        integer k;
        begin
            for (k = 0; k < WAIT_LIMIT_CYC; k = k + 1) begin
                @(posedge clk);
                if (j0_snap_commit)
                    disable wait_acc_commit;
            end
            $display("FAIL: LIS3DH commit timeout");
            $fatal;
        end
    endtask

    task wait_mag_init_done;
        integer k;
        begin
            for (k = 0; k < WAIT_LIMIT_CYC; k = k + 1) begin
                @(posedge clk);
                if (j2_init_done)
                    disable wait_mag_init_done;
            end
            $display("FAIL: LIS2MDL init timeout");
            $fatal;
        end
    endtask

    task wait_mag_commit;
        integer k;
        begin
            for (k = 0; k < WAIT_LIMIT_CYC; k = k + 1) begin
                @(posedge clk);
                if (j2_snap_commit)
                    disable wait_mag_commit;
            end
            $display("FAIL: LIS2MDL commit timeout");
            $fatal;
        end
    endtask

    //--------------------------------------------------------------------------
    // Main regression sequence
    //--------------------------------------------------------------------------
    initial begin
        rst           = 1'b1;
        epoch_100hz   = 1'b0;
        epoch_50hz    = 1'b0;
        epoch_10hz    = 1'b0;
        grant_lis3dh  = 1'b0;
        grant_bmp585  = 1'b0;
        grant_lis2mdl = 1'b0;

        repeat (20) @(posedge clk);
        rst = 1'b0;

        //--------------------------------------------------------------------------
        // Phase A : BMP585
        //--------------------------------------------------------------------------
        // Allow the internal power-up wait to elapse in microseconds.
        repeat (4000 * US_DIV_CYCLES) @(posedge clk);

        grant_bmp585 <= 1'b1;
        wait_bmp_init_done();
        grant_bmp585 <= 1'b0;

        grant_bmp585 <= 1'b1;
        epoch_50hz   <= 1'b1;
        @(posedge clk);
        epoch_50hz   <= 1'b0;
        wait_bmp_commit();
        grant_bmp585 <= 1'b0;

        if (!j1_snap_valid_in) begin
            $display("FAIL: BMP585 snap_valid_in low");
            $fatal;
        end

        if (j1_snap_status_in !== 8'h00) begin
            $display("FAIL: BMP585 status mismatch: %02h", j1_snap_status_in);
            $fatal;
        end

        // bmp585_job snapshot payload:
        // {press_msb,press_lsb,press_xlsb,temp_msb,temp_lsb,temp_xlsb}
        if (j1_snap_payload_in !== 48'h33_22_11_66_55_44) begin
            $display("FAIL: BMP585 payload mismatch: %012h", j1_snap_payload_in);
            $fatal;
        end

        //--------------------------------------------------------------------------
        // Phase B : LIS3DH
        //--------------------------------------------------------------------------
        grant_lis3dh <= 1'b1;
        wait_acc_init_done();
        grant_lis3dh <= 1'b0;

        grant_lis3dh <= 1'b1;
        epoch_100hz  <= 1'b1;
        @(posedge clk);
        epoch_100hz  <= 1'b0;
        wait_acc_commit();
        grant_lis3dh <= 1'b0;

        if (!j0_snap_valid_in) begin
            $display("FAIL: LIS3DH snap_valid_in low");
            $fatal;
        end

        if (j0_snap_status_in !== 8'h00) begin
            $display("FAIL: LIS3DH status mismatch: %02h", j0_snap_status_in);
            $fatal;
        end

        // lis3dh_job snapshot payload:
        // {x_raw,y_raw,z_raw} = {2211,4433,6655}
        if (j0_snap_payload_in !== 48'h2211_4433_6655) begin
            $display("FAIL: LIS3DH payload mismatch: %012h", j0_snap_payload_in);
            $fatal;
        end

        //--------------------------------------------------------------------------
        // Phase C : LIS2MDL
        //--------------------------------------------------------------------------
        grant_lis2mdl <= 1'b1;
        wait_mag_init_done();
        grant_lis2mdl <= 1'b0;

        grant_lis2mdl <= 1'b1;
        epoch_10hz    <= 1'b1;
        @(posedge clk);
        epoch_10hz    <= 1'b0;
        wait_mag_commit();
        grant_lis2mdl <= 1'b0;

        if (!j2_snap_valid_in) begin
            $display("FAIL: LIS2MDL snap_valid_in low");
            $fatal;
        end

        if (j2_snap_status_in !== 8'h00) begin
            $display("FAIL: LIS2MDL status mismatch: %02h", j2_snap_status_in);
            $fatal;
        end

        // Current simple LIS2MDL model read order:
        // XL=11 XH=22 YL=33 YH=44 ZL=55 ZH=66
        // Expected published payload depends on current lis2mdl_job implementation.
        // This canonical regression freezes the same expectation used previously.
        if (j2_snap_payload_in !== 48'h66_55_44_33_22_11) begin
            $display("FAIL: LIS2MDL payload mismatch: %012h", j2_snap_payload_in);
            $fatal;
        end

        $display("PASS: canonical shared-engine phased regression for LIS3DH + BMP585 + LIS2MDL");
        $finish;
    end

endmodule


//==============================================================================
// lis3dh_i2c_slave_model
//------------------------------------------------------------------------------
// Minimal behavioral LIS3DH I2C slave.
// Supports:
//   - address 0x18
//   - register pointer write
//   - repeated-start read
//   - auto-increment read when pointer MSB is set by master
//==============================================================================
module lis3dh_i2c_slave_model (
    input  wire rst,
    inout  wire scl,
    inout  wire sda
);
    localparam [6:0] DEV_ADDR7 = 7'h18;

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

        mem[8'h0F] = 8'h33; // WHO_AM_I
        mem[8'h27] = 8'h80; // STATUS_REG
        mem[8'h28] = 8'h11; // OUT_X_L
        mem[8'h29] = 8'h22; // OUT_X_H
        mem[8'h2A] = 8'h33; // OUT_Y_L
        mem[8'h2B] = 8'h44; // OUT_Y_H
        mem[8'h2C] = 8'h55; // OUT_Z_L
        mem[8'h2D] = 8'h66; // OUT_Z_H
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
                        state_r <= S_IDLE;
                    end
                end

                S_WR_GOT: begin
                    if (first_write_byte_r) begin
                        reg_ptr_r <= shreg_r;
                        if (shreg_r[7])
                            reg_ptr_r <= {1'b0, shreg_r[6:0]};
                        first_write_byte_r <= 1'b0;
                    end else begin
                        mem[reg_ptr_r] <= shreg_r;
                        reg_ptr_r      <= reg_ptr_r + 8'd1;
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


//==============================================================================
// bmp585_i2c_slave_model
//------------------------------------------------------------------------------
// Minimal behavioral BMP585 I2C slave.
// Supports:
//   - address 0x47
//   - CHIP_ID readback
//   - configuration writes
//   - data burst TEMP_XLSB..PRESS_MSB
//==============================================================================
module bmp585_i2c_slave_model (
    input  wire rst,
    inout  wire scl,
    inout  wire sda
);
    localparam [6:0] DEV_ADDR7 = 7'h47;

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

        mem[8'h01] = 8'h51; // CHIP_ID
        mem[8'h27] = 8'h01; // INT_STATUS / DRDY
        mem[8'h1D] = 8'h44; // TEMP_XLSB
        mem[8'h1E] = 8'h55; // TEMP_LSB
        mem[8'h1F] = 8'h66; // TEMP_MSB
        mem[8'h20] = 8'h11; // PRESS_XLSB
        mem[8'h21] = 8'h22; // PRESS_LSB
        mem[8'h22] = 8'h33; // PRESS_MSB
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
                        state_r <= S_IDLE;
                    end
                end

                S_WR_GOT: begin
                    if (first_write_byte_r) begin
                        reg_ptr_r          <= shreg_r;
                        first_write_byte_r <= 1'b0;
                    end else begin
                        mem[reg_ptr_r] <= shreg_r;
                        reg_ptr_r      <= reg_ptr_r + 8'd1;
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


//==============================================================================
// lis2mdl_i2c_slave_model
//------------------------------------------------------------------------------
// Minimal behavioral LIS2MDL I2C slave.
// Supports:
//   - address 0x1E
//   - WHO_AM_I readback
//   - CFG_REG_A / CFG_REG_C writes
//   - data burst OUTX_L..OUTZ_H
//==============================================================================
module lis2mdl_i2c_slave_model (
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
                        state_r <= S_IDLE;
                    end
                end

                S_WR_GOT: begin
                    if (first_write_byte_r) begin
                        reg_ptr_r          <= shreg_r;
                        first_write_byte_r <= 1'b0;
                    end else begin
                        mem[reg_ptr_r] <= shreg_r;
                        reg_ptr_r      <= reg_ptr_r + 8'd1;
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
