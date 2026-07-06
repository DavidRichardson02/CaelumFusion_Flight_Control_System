`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// tb_pmon1_i2c_job
//------------------------------------------------------------------------------
// Focused PMON1/ADM1191 job contract check.
//
// Proves:
//   - the job issues the configured 7'h38 repeated-start read commands
//   - DATA_CMD reads three bytes and STATUS_CMD reads one byte
//   - committed payload preserves status, voltage code, current code, seq-ready
//     raw data ordering
//   - a failed I2C completion commits invalid, non-OK power evidence
//==============================================================================
module tb_pmon1_i2c_job;
    localparam integer CLK_PERIOD_NS = 10;

    reg         clk;
    reg         rst;
    reg  [31:0] time_us;
    reg         epoch_10hz;
    reg         grant;

    wire        cmd_valid;
    reg         cmd_ready;
    wire [6:0]  cmd_addr7;
    wire [7:0]  cmd_wlen;
    wire [7:0]  cmd_rlen;
    wire        cmd_repstart;
    wire [31:0] cmd_timeout_us;

    wire        w_valid;
    reg         w_ready;
    wire [7:0]  w_data;
    wire        w_last;

    reg         r_valid;
    wire        r_ready;
    reg  [7:0]  r_data;
    reg         r_last;

    reg         done;
    reg  [3:0]  done_code;
    wire        busy;

    wire        snap_commit;
    wire        snap_valid_in;
    wire [7:0]  snap_status_in;
    wire [47:0] snap_payload_in;
    wire        init_done;

    integer errors;

    pmon1_i2c_job #(
        .PMON1_ADDR7     (7'h38),
        .CMD_TIMEOUT_US  (32'd77),
        .POWERUP_WAIT_US (32'd10),
        .DATA_CMD        (8'h05),
        .STATUS_CMD      (8'h45)
    ) dut (
        .clk             (clk),
        .rst             (rst),
        .time_us         (time_us),
        .epoch_10hz      (epoch_10hz),
        .grant           (grant),

        .cmd_valid       (cmd_valid),
        .cmd_ready       (cmd_ready),
        .cmd_addr7       (cmd_addr7),
        .cmd_wlen        (cmd_wlen),
        .cmd_rlen        (cmd_rlen),
        .cmd_repstart    (cmd_repstart),
        .cmd_timeout_us  (cmd_timeout_us),

        .w_valid         (w_valid),
        .w_ready         (w_ready),
        .w_data          (w_data),
        .w_last          (w_last),

        .r_valid         (r_valid),
        .r_ready         (r_ready),
        .r_data          (r_data),
        .r_last          (r_last),

        .done            (done),
        .done_code       (done_code),
        .busy            (busy),

        .snap_commit     (snap_commit),
        .snap_valid_in   (snap_valid_in),
        .snap_status_in  (snap_status_in),
        .snap_payload_in (snap_payload_in),
        .init_done       (init_done)
    );

    initial begin
        clk = 1'b0;
        forever #(CLK_PERIOD_NS/2) clk = ~clk;
    end

    task expect;
        input condition;
        input [8*120-1:0] message;
        begin
            if (!condition) begin
                errors = errors + 1;
                $display("TB FAIL: %s at %0t", message, $time);
            end
        end
    endtask

    task reset_dut;
        begin
            rst         = 1'b1;
            time_us     = 32'd0;
            epoch_10hz  = 1'b0;
            grant       = 1'b0;
            cmd_ready   = 1'b0;
            w_ready     = 1'b0;
            r_valid     = 1'b0;
            r_data      = 8'd0;
            r_last      = 1'b0;
            done        = 1'b0;
            done_code   = 4'd0;
            repeat (8) @(posedge clk);
            rst = 1'b0;
            time_us = 32'd20;
            repeat (2) @(posedge clk);
        end
    endtask

    task start_request;
        begin
            grant = 1'b1;
            epoch_10hz = 1'b1;
            @(posedge clk);
            #1;
            epoch_10hz = 1'b0;
        end
    endtask

    task accept_command;
        input [7:0] expected_rlen;
        input [7:0] expected_wdata;
        integer i;
        begin
            cmd_ready = 1'b0;
            for (i = 0; (i < 100) && !cmd_valid; i = i + 1) begin
                @(posedge clk);
                #1;
            end
            expect(cmd_valid, "command became valid");
            expect(cmd_addr7 == 7'h38, "PMON1 command uses address 7'h38");
            expect(cmd_wlen == 8'd1, "PMON1 command writes one command byte");
            expect(cmd_rlen == expected_rlen, "PMON1 command read length matches operation");
            expect(cmd_repstart == 1'b1, "PMON1 command uses repeated-start read");
            expect(cmd_timeout_us == 32'd77, "PMON1 command timeout is propagated");

            cmd_ready = 1'b1;
            @(posedge clk);
            #1;
            cmd_ready = 1'b0;

            for (i = 0; (i < 100) && !w_valid; i = i + 1) begin
                @(posedge clk);
                #1;
            end
            expect(w_valid, "write command byte became valid");
            expect(w_data == expected_wdata, "write command byte matches operation");
            expect(w_last == 1'b1, "PMON1 command byte is marked last");

            w_ready = 1'b1;
            @(posedge clk);
            #1;
            w_ready = 1'b0;
        end
    endtask

    task feed_read3_done_ok;
        begin
            wait (r_ready === 1'b1);
            @(negedge clk);
            r_valid = 1'b1;
            r_data  = 8'hA5;
            r_last  = 1'b0;
            @(negedge clk);
            r_data  = 8'h3C;
            r_last  = 1'b0;
            @(negedge clk);
            r_data  = 8'h9F;
            r_last  = 1'b1;
            @(negedge clk);
            r_valid = 1'b0;
            r_last  = 1'b0;
            @(negedge clk);
            done      = 1'b1;
            done_code = 4'd0;
            @(negedge clk);
            done      = 1'b0;
            done_code = 4'd0;
        end
    endtask

    task feed_read1_done_ok;
        begin
            wait (r_ready === 1'b1);
            @(negedge clk);
            r_valid = 1'b1;
            r_data  = 8'h5A;
            r_last  = 1'b1;
            @(negedge clk);
            r_valid = 1'b0;
            r_last  = 1'b0;
            @(negedge clk);
            done      = 1'b1;
            done_code = 4'd0;
            @(negedge clk);
            done      = 1'b0;
            done_code = 4'd0;
        end
    endtask

    task finish_with_i2c_error;
        begin
            wait (r_ready === 1'b1);
            @(negedge clk);
            done      = 1'b1;
            done_code = 4'd1;
            @(negedge clk);
            done      = 1'b0;
            done_code = 4'd0;
        end
    endtask

    task wait_commit;
        integer i;
        begin
            #1;
            for (i = 0; (i < 100) && !snap_commit; i = i + 1) begin
                @(posedge clk);
                #1;
            end
            expect(snap_commit, "snapshot commit occurred");
        end
    endtask

    initial begin
        errors = 0;

        reset_dut();
        start_request();
        accept_command(8'd3, 8'h05);
        feed_read3_done_ok();
        accept_command(8'd1, 8'h45);
        feed_read1_done_ok();
        wait_commit();

        $display("TB_INFO: success payload=0x%012h valid=%b status=0x%02h init=%b",
                 snap_payload_in, snap_valid_in, snap_status_in, init_done);
        expect(snap_valid_in == 1'b1, "successful PMON sample is valid");
        expect(snap_status_in == 8'h00, "successful PMON sample status is OK");
        expect(snap_payload_in == 48'h5AA5_93CF_0000,
               "PMON payload packs status, voltage, current, reserved");
        expect(init_done == 1'b1, "successful PMON sample marks init done");

        reset_dut();
        start_request();
        accept_command(8'd3, 8'h05);
        finish_with_i2c_error();
        wait_commit();

        expect(snap_valid_in == 1'b0, "failed PMON transaction is invalid");
        expect(snap_status_in == 8'hE0, "failed PMON transaction reports I2C error status");
        expect(init_done == 1'b0, "failed PMON transaction does not mark init done");

        if (errors == 0) begin
            $display("PASS: tb_pmon1_i2c_job");
            $finish;
        end

        $display("FAIL: tb_pmon1_i2c_job errors=%0d", errors);
        $finish;
    end
endmodule

`default_nettype wire
