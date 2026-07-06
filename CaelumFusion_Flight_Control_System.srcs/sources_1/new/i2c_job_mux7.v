`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// i2c_job_mux7
//------------------------------------------------------------------------------
// Expanded one-hot mux/demux between seven job FSMs and one shared I2C engine.
//
// The ownership contract matches i2c_job_mux:
//   - grant_* is a launch authorization pulse, not persistent ownership.
//   - Ownership latches only on e_cmd_valid && e_cmd_ready.
//   - Ownership remains latched until e_done.
//==============================================================================
module i2c_job_mux7 (
    input  wire clk,
    input  wire rst,

    input  wire grant_j0,
    input  wire grant_j1,
    input  wire grant_j2,
    input  wire grant_j3,
    input  wire grant_j4,
    input  wire grant_j5,
    input  wire grant_j6,

    input  wire        j0_cmd_valid,
    output wire        j0_cmd_ready,
    input  wire [6:0]  j0_cmd_addr7,
    input  wire [7:0]  j0_cmd_wlen,
    input  wire [7:0]  j0_cmd_rlen,
    input  wire        j0_cmd_repstart,
    input  wire [31:0] j0_cmd_timeout_us,
    input  wire        j0_w_valid,
    output wire        j0_w_ready,
    input  wire [7:0]  j0_w_data,
    input  wire        j0_w_last,
    output wire        j0_r_valid,
    input  wire        j0_r_ready,
    output wire [7:0]  j0_r_data,
    output wire        j0_r_last,
    output wire        j0_done,
    output wire [3:0]  j0_done_code,
    output wire        j0_busy,

    input  wire        j1_cmd_valid,
    output wire        j1_cmd_ready,
    input  wire [6:0]  j1_cmd_addr7,
    input  wire [7:0]  j1_cmd_wlen,
    input  wire [7:0]  j1_cmd_rlen,
    input  wire        j1_cmd_repstart,
    input  wire [31:0] j1_cmd_timeout_us,
    input  wire        j1_w_valid,
    output wire        j1_w_ready,
    input  wire [7:0]  j1_w_data,
    input  wire        j1_w_last,
    output wire        j1_r_valid,
    input  wire        j1_r_ready,
    output wire [7:0]  j1_r_data,
    output wire        j1_r_last,
    output wire        j1_done,
    output wire [3:0]  j1_done_code,
    output wire        j1_busy,

    input  wire        j2_cmd_valid,
    output wire        j2_cmd_ready,
    input  wire [6:0]  j2_cmd_addr7,
    input  wire [7:0]  j2_cmd_wlen,
    input  wire [7:0]  j2_cmd_rlen,
    input  wire        j2_cmd_repstart,
    input  wire [31:0] j2_cmd_timeout_us,
    input  wire        j2_w_valid,
    output wire        j2_w_ready,
    input  wire [7:0]  j2_w_data,
    input  wire        j2_w_last,
    output wire        j2_r_valid,
    input  wire        j2_r_ready,
    output wire [7:0]  j2_r_data,
    output wire        j2_r_last,
    output wire        j2_done,
    output wire [3:0]  j2_done_code,
    output wire        j2_busy,

    input  wire        j3_cmd_valid,
    output wire        j3_cmd_ready,
    input  wire [6:0]  j3_cmd_addr7,
    input  wire [7:0]  j3_cmd_wlen,
    input  wire [7:0]  j3_cmd_rlen,
    input  wire        j3_cmd_repstart,
    input  wire [31:0] j3_cmd_timeout_us,
    input  wire        j3_w_valid,
    output wire        j3_w_ready,
    input  wire [7:0]  j3_w_data,
    input  wire        j3_w_last,
    output wire        j3_r_valid,
    input  wire        j3_r_ready,
    output wire [7:0]  j3_r_data,
    output wire        j3_r_last,
    output wire        j3_done,
    output wire [3:0]  j3_done_code,
    output wire        j3_busy,

    input  wire        j4_cmd_valid,
    output wire        j4_cmd_ready,
    input  wire [6:0]  j4_cmd_addr7,
    input  wire [7:0]  j4_cmd_wlen,
    input  wire [7:0]  j4_cmd_rlen,
    input  wire        j4_cmd_repstart,
    input  wire [31:0] j4_cmd_timeout_us,
    input  wire        j4_w_valid,
    output wire        j4_w_ready,
    input  wire [7:0]  j4_w_data,
    input  wire        j4_w_last,
    output wire        j4_r_valid,
    input  wire        j4_r_ready,
    output wire [7:0]  j4_r_data,
    output wire        j4_r_last,
    output wire        j4_done,
    output wire [3:0]  j4_done_code,
    output wire        j4_busy,

    input  wire        j5_cmd_valid,
    output wire        j5_cmd_ready,
    input  wire [6:0]  j5_cmd_addr7,
    input  wire [7:0]  j5_cmd_wlen,
    input  wire [7:0]  j5_cmd_rlen,
    input  wire        j5_cmd_repstart,
    input  wire [31:0] j5_cmd_timeout_us,
    input  wire        j5_w_valid,
    output wire        j5_w_ready,
    input  wire [7:0]  j5_w_data,
    input  wire        j5_w_last,
    output wire        j5_r_valid,
    input  wire        j5_r_ready,
    output wire [7:0]  j5_r_data,
    output wire        j5_r_last,
    output wire        j5_done,
    output wire [3:0]  j5_done_code,
    output wire        j5_busy,

    input  wire        j6_cmd_valid,
    output wire        j6_cmd_ready,
    input  wire [6:0]  j6_cmd_addr7,
    input  wire [7:0]  j6_cmd_wlen,
    input  wire [7:0]  j6_cmd_rlen,
    input  wire        j6_cmd_repstart,
    input  wire [31:0] j6_cmd_timeout_us,
    input  wire        j6_w_valid,
    output wire        j6_w_ready,
    input  wire [7:0]  j6_w_data,
    input  wire        j6_w_last,
    output wire        j6_r_valid,
    input  wire        j6_r_ready,
    output wire [7:0]  j6_r_data,
    output wire        j6_r_last,
    output wire        j6_done,
    output wire [3:0]  j6_done_code,
    output wire        j6_busy,

    output reg         e_cmd_valid,
    input  wire        e_cmd_ready,
    output reg  [6:0]  e_cmd_addr7,
    output reg  [7:0]  e_cmd_wlen,
    output reg  [7:0]  e_cmd_rlen,
    output reg         e_cmd_repstart,
    output reg  [31:0] e_cmd_timeout_us,
    output reg         e_w_valid,
    input  wire        e_w_ready,
    output reg  [7:0]  e_w_data,
    output reg         e_w_last,
    input  wire        e_r_valid,
    output reg         e_r_ready,
    input  wire [7:0]  e_r_data,
    input  wire        e_r_last,
    input  wire        e_done,
    input  wire [3:0]  e_done_code
);
    localparam [3:0]
        OWN_NONE = 4'd0,
        OWN_J0   = 4'd1,
        OWN_J1   = 4'd2,
        OWN_J2   = 4'd3,
        OWN_J3   = 4'd4,
        OWN_J4   = 4'd5,
        OWN_J5   = 4'd6,
        OWN_J6   = 4'd7;

    reg [3:0] owner_q;
    reg [3:0] sel_idle;

    always @(*) begin
        if (grant_j0)
            sel_idle = OWN_J0;
        else if (grant_j1)
            sel_idle = OWN_J1;
        else if (grant_j2)
            sel_idle = OWN_J2;
        else if (grant_j3)
            sel_idle = OWN_J3;
        else if (grant_j4)
            sel_idle = OWN_J4;
        else if (grant_j5)
            sel_idle = OWN_J5;
        else if (grant_j6)
            sel_idle = OWN_J6;
        else
            sel_idle = OWN_NONE;
    end

    always @(posedge clk) begin
        if (rst) begin
            owner_q <= OWN_NONE;
        end else if (owner_q == OWN_NONE) begin
            if ((sel_idle != OWN_NONE) && e_cmd_valid && e_cmd_ready)
                owner_q <= sel_idle;
        end else if (e_done) begin
            owner_q <= OWN_NONE;
        end
    end

    always @(*) begin
        e_cmd_valid      = 1'b0;
        e_cmd_addr7      = 7'd0;
        e_cmd_wlen       = 8'd0;
        e_cmd_rlen       = 8'd0;
        e_cmd_repstart   = 1'b0;
        e_cmd_timeout_us = 32'd0;
        e_w_valid        = 1'b0;
        e_w_data         = 8'd0;
        e_w_last         = 1'b0;
        e_r_ready        = 1'b0;

        case ((owner_q == OWN_NONE) ? sel_idle : owner_q)
            OWN_J0: begin
                e_cmd_valid      = (owner_q == OWN_NONE) ? j0_cmd_valid : 1'b0;
                e_cmd_addr7      = j0_cmd_addr7;
                e_cmd_wlen       = j0_cmd_wlen;
                e_cmd_rlen       = j0_cmd_rlen;
                e_cmd_repstart   = j0_cmd_repstart;
                e_cmd_timeout_us = j0_cmd_timeout_us;
                e_w_valid        = j0_w_valid;
                e_w_data         = j0_w_data;
                e_w_last         = j0_w_last;
                e_r_ready        = j0_r_ready;
            end
            OWN_J1: begin
                e_cmd_valid      = (owner_q == OWN_NONE) ? j1_cmd_valid : 1'b0;
                e_cmd_addr7      = j1_cmd_addr7;
                e_cmd_wlen       = j1_cmd_wlen;
                e_cmd_rlen       = j1_cmd_rlen;
                e_cmd_repstart   = j1_cmd_repstart;
                e_cmd_timeout_us = j1_cmd_timeout_us;
                e_w_valid        = j1_w_valid;
                e_w_data         = j1_w_data;
                e_w_last         = j1_w_last;
                e_r_ready        = j1_r_ready;
            end
            OWN_J2: begin
                e_cmd_valid      = (owner_q == OWN_NONE) ? j2_cmd_valid : 1'b0;
                e_cmd_addr7      = j2_cmd_addr7;
                e_cmd_wlen       = j2_cmd_wlen;
                e_cmd_rlen       = j2_cmd_rlen;
                e_cmd_repstart   = j2_cmd_repstart;
                e_cmd_timeout_us = j2_cmd_timeout_us;
                e_w_valid        = j2_w_valid;
                e_w_data         = j2_w_data;
                e_w_last         = j2_w_last;
                e_r_ready        = j2_r_ready;
            end
            OWN_J3: begin
                e_cmd_valid      = (owner_q == OWN_NONE) ? j3_cmd_valid : 1'b0;
                e_cmd_addr7      = j3_cmd_addr7;
                e_cmd_wlen       = j3_cmd_wlen;
                e_cmd_rlen       = j3_cmd_rlen;
                e_cmd_repstart   = j3_cmd_repstart;
                e_cmd_timeout_us = j3_cmd_timeout_us;
                e_w_valid        = j3_w_valid;
                e_w_data         = j3_w_data;
                e_w_last         = j3_w_last;
                e_r_ready        = j3_r_ready;
            end
            OWN_J4: begin
                e_cmd_valid      = (owner_q == OWN_NONE) ? j4_cmd_valid : 1'b0;
                e_cmd_addr7      = j4_cmd_addr7;
                e_cmd_wlen       = j4_cmd_wlen;
                e_cmd_rlen       = j4_cmd_rlen;
                e_cmd_repstart   = j4_cmd_repstart;
                e_cmd_timeout_us = j4_cmd_timeout_us;
                e_w_valid        = j4_w_valid;
                e_w_data         = j4_w_data;
                e_w_last         = j4_w_last;
                e_r_ready        = j4_r_ready;
            end
            OWN_J5: begin
                e_cmd_valid      = (owner_q == OWN_NONE) ? j5_cmd_valid : 1'b0;
                e_cmd_addr7      = j5_cmd_addr7;
                e_cmd_wlen       = j5_cmd_wlen;
                e_cmd_rlen       = j5_cmd_rlen;
                e_cmd_repstart   = j5_cmd_repstart;
                e_cmd_timeout_us = j5_cmd_timeout_us;
                e_w_valid        = j5_w_valid;
                e_w_data         = j5_w_data;
                e_w_last         = j5_w_last;
                e_r_ready        = j5_r_ready;
            end
            OWN_J6: begin
                e_cmd_valid      = (owner_q == OWN_NONE) ? j6_cmd_valid : 1'b0;
                e_cmd_addr7      = j6_cmd_addr7;
                e_cmd_wlen       = j6_cmd_wlen;
                e_cmd_rlen       = j6_cmd_rlen;
                e_cmd_repstart   = j6_cmd_repstart;
                e_cmd_timeout_us = j6_cmd_timeout_us;
                e_w_valid        = j6_w_valid;
                e_w_data         = j6_w_data;
                e_w_last         = j6_w_last;
                e_r_ready        = j6_r_ready;
            end
            default: begin
            end
        endcase
    end

    assign j0_cmd_ready = ((owner_q == OWN_NONE) && (sel_idle == OWN_J0)) ? e_cmd_ready : 1'b0;
    assign j1_cmd_ready = ((owner_q == OWN_NONE) && (sel_idle == OWN_J1)) ? e_cmd_ready : 1'b0;
    assign j2_cmd_ready = ((owner_q == OWN_NONE) && (sel_idle == OWN_J2)) ? e_cmd_ready : 1'b0;
    assign j3_cmd_ready = ((owner_q == OWN_NONE) && (sel_idle == OWN_J3)) ? e_cmd_ready : 1'b0;
    assign j4_cmd_ready = ((owner_q == OWN_NONE) && (sel_idle == OWN_J4)) ? e_cmd_ready : 1'b0;
    assign j5_cmd_ready = ((owner_q == OWN_NONE) && (sel_idle == OWN_J5)) ? e_cmd_ready : 1'b0;
    assign j6_cmd_ready = ((owner_q == OWN_NONE) && (sel_idle == OWN_J6)) ? e_cmd_ready : 1'b0;

    assign j0_w_ready = ((owner_q == OWN_J0) || ((owner_q == OWN_NONE) && (sel_idle == OWN_J0))) ? e_w_ready : 1'b0;
    assign j1_w_ready = ((owner_q == OWN_J1) || ((owner_q == OWN_NONE) && (sel_idle == OWN_J1))) ? e_w_ready : 1'b0;
    assign j2_w_ready = ((owner_q == OWN_J2) || ((owner_q == OWN_NONE) && (sel_idle == OWN_J2))) ? e_w_ready : 1'b0;
    assign j3_w_ready = ((owner_q == OWN_J3) || ((owner_q == OWN_NONE) && (sel_idle == OWN_J3))) ? e_w_ready : 1'b0;
    assign j4_w_ready = ((owner_q == OWN_J4) || ((owner_q == OWN_NONE) && (sel_idle == OWN_J4))) ? e_w_ready : 1'b0;
    assign j5_w_ready = ((owner_q == OWN_J5) || ((owner_q == OWN_NONE) && (sel_idle == OWN_J5))) ? e_w_ready : 1'b0;
    assign j6_w_ready = ((owner_q == OWN_J6) || ((owner_q == OWN_NONE) && (sel_idle == OWN_J6))) ? e_w_ready : 1'b0;

    assign j0_r_valid = (owner_q == OWN_J0) ? e_r_valid : 1'b0;
    assign j1_r_valid = (owner_q == OWN_J1) ? e_r_valid : 1'b0;
    assign j2_r_valid = (owner_q == OWN_J2) ? e_r_valid : 1'b0;
    assign j3_r_valid = (owner_q == OWN_J3) ? e_r_valid : 1'b0;
    assign j4_r_valid = (owner_q == OWN_J4) ? e_r_valid : 1'b0;
    assign j5_r_valid = (owner_q == OWN_J5) ? e_r_valid : 1'b0;
    assign j6_r_valid = (owner_q == OWN_J6) ? e_r_valid : 1'b0;

    assign j0_r_data = e_r_data;
    assign j1_r_data = e_r_data;
    assign j2_r_data = e_r_data;
    assign j3_r_data = e_r_data;
    assign j4_r_data = e_r_data;
    assign j5_r_data = e_r_data;
    assign j6_r_data = e_r_data;

    assign j0_r_last = (owner_q == OWN_J0) ? e_r_last : 1'b0;
    assign j1_r_last = (owner_q == OWN_J1) ? e_r_last : 1'b0;
    assign j2_r_last = (owner_q == OWN_J2) ? e_r_last : 1'b0;
    assign j3_r_last = (owner_q == OWN_J3) ? e_r_last : 1'b0;
    assign j4_r_last = (owner_q == OWN_J4) ? e_r_last : 1'b0;
    assign j5_r_last = (owner_q == OWN_J5) ? e_r_last : 1'b0;
    assign j6_r_last = (owner_q == OWN_J6) ? e_r_last : 1'b0;

    assign j0_done = (owner_q == OWN_J0) ? e_done : 1'b0;
    assign j1_done = (owner_q == OWN_J1) ? e_done : 1'b0;
    assign j2_done = (owner_q == OWN_J2) ? e_done : 1'b0;
    assign j3_done = (owner_q == OWN_J3) ? e_done : 1'b0;
    assign j4_done = (owner_q == OWN_J4) ? e_done : 1'b0;
    assign j5_done = (owner_q == OWN_J5) ? e_done : 1'b0;
    assign j6_done = (owner_q == OWN_J6) ? e_done : 1'b0;

    assign j0_done_code = (owner_q == OWN_J0) ? e_done_code : 4'd0;
    assign j1_done_code = (owner_q == OWN_J1) ? e_done_code : 4'd0;
    assign j2_done_code = (owner_q == OWN_J2) ? e_done_code : 4'd0;
    assign j3_done_code = (owner_q == OWN_J3) ? e_done_code : 4'd0;
    assign j4_done_code = (owner_q == OWN_J4) ? e_done_code : 4'd0;
    assign j5_done_code = (owner_q == OWN_J5) ? e_done_code : 4'd0;
    assign j6_done_code = (owner_q == OWN_J6) ? e_done_code : 4'd0;

    assign j0_busy = (owner_q == OWN_J0);
    assign j1_busy = (owner_q == OWN_J1);
    assign j2_busy = (owner_q == OWN_J2);
    assign j3_busy = (owner_q == OWN_J3);
    assign j4_busy = (owner_q == OWN_J4);
    assign j5_busy = (owner_q == OWN_J5);
    assign j6_busy = (owner_q == OWN_J6);
endmodule

`default_nettype wire
