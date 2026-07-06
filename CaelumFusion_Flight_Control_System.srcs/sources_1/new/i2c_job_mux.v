`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// i2c_job_mux
//------------------------------------------------------------------------------
// ROLE
//   One-hot mux/demux between multiple job FSMs and one shared I2C engine.
//
// CONTRACT
//   - grant_* is a launch authorization pulse, not persistent ownership.
//   - Ownership latches only on a real engine command handshake:
//         e_cmd_valid && e_cmd_ready
//   - Once latched, ownership remains until e_done.
//   - While idle, command-path selection follows grant_*.
//   - While active, all stream/response paths route only to the latched owner.
//==============================================================================
module i2c_job_mux (
    input  wire clk,
    input  wire rst,

    input  wire grant_lis3dh,
    input  wire grant_bmp585,
    input  wire grant_lis2mdl,
    input  wire grant_pmon1,

    // --- job 0 : LIS3DH ------------------------------------------------------
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

    // --- job 1 : BMP585 ------------------------------------------------------
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

    // --- job 2 : LIS2MDL -----------------------------------------------------
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

    // --- job 3 : PMON1 / ADM1191 --------------------------------------------
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

    // --- engine side ---------------------------------------------------------
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

    localparam [2:0]
        OWN_NONE = 3'd0,
        OWN_J0   = 3'd1,
        OWN_J1   = 3'd2,
        OWN_J2   = 3'd3,
        OWN_J3   = 3'd4;

    reg [2:0] owner_q;
    reg [2:0] sel_idle;

    wire any_grant;
    assign any_grant = grant_lis3dh | grant_bmp585 | grant_lis2mdl | grant_pmon1;

    always @(*) begin
        if (grant_lis3dh)
            sel_idle = OWN_J0;
        else if (grant_bmp585)
            sel_idle = OWN_J1;
        else if (grant_lis2mdl)
            sel_idle = OWN_J2;
        else if (grant_pmon1)
            sel_idle = OWN_J3;
        else
            sel_idle = OWN_NONE;
    end

    always @(posedge clk) begin
        if (rst) begin
            owner_q <= OWN_NONE;
        end else begin
            if (owner_q == OWN_NONE) begin
                if (sel_idle != OWN_NONE) begin
                    if (e_cmd_valid && e_cmd_ready)
                        owner_q <= sel_idle;
                end
            end else begin
                if (e_done)
                    owner_q <= OWN_NONE;
            end
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

        if (owner_q == OWN_NONE) begin
            case (sel_idle)
                OWN_J0: begin
                    e_cmd_valid      = j0_cmd_valid;
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
                    e_cmd_valid      = j1_cmd_valid;
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
                    e_cmd_valid      = j2_cmd_valid;
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
                    e_cmd_valid      = j3_cmd_valid;
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
                default: begin
                end
            endcase
        end else begin
            case (owner_q)
                OWN_J0: begin
                    e_w_valid = j0_w_valid;
                    e_w_data  = j0_w_data;
                    e_w_last  = j0_w_last;
                    e_r_ready = j0_r_ready;
                end
                OWN_J1: begin
                    e_w_valid = j1_w_valid;
                    e_w_data  = j1_w_data;
                    e_w_last  = j1_w_last;
                    e_r_ready = j1_r_ready;
                end
                OWN_J2: begin
                    e_w_valid = j2_w_valid;
                    e_w_data  = j2_w_data;
                    e_w_last  = j2_w_last;
                    e_r_ready = j2_r_ready;
                end
                OWN_J3: begin
                    e_w_valid = j3_w_valid;
                    e_w_data  = j3_w_data;
                    e_w_last  = j3_w_last;
                    e_r_ready = j3_r_ready;
                end
                default: begin
                end
            endcase
        end
    end

    assign j0_cmd_ready = ((owner_q == OWN_NONE) && (sel_idle == OWN_J0)) ? e_cmd_ready : 1'b0;
    assign j1_cmd_ready = ((owner_q == OWN_NONE) && (sel_idle == OWN_J1)) ? e_cmd_ready : 1'b0;
    assign j2_cmd_ready = ((owner_q == OWN_NONE) && (sel_idle == OWN_J2)) ? e_cmd_ready : 1'b0;
    assign j3_cmd_ready = ((owner_q == OWN_NONE) && (sel_idle == OWN_J3)) ? e_cmd_ready : 1'b0;

    assign j0_w_ready   = ((owner_q == OWN_J0) || ((owner_q == OWN_NONE) && (sel_idle == OWN_J0))) ? e_w_ready : 1'b0;
    assign j1_w_ready   = ((owner_q == OWN_J1) || ((owner_q == OWN_NONE) && (sel_idle == OWN_J1))) ? e_w_ready : 1'b0;
    assign j2_w_ready   = ((owner_q == OWN_J2) || ((owner_q == OWN_NONE) && (sel_idle == OWN_J2))) ? e_w_ready : 1'b0;
    assign j3_w_ready   = ((owner_q == OWN_J3) || ((owner_q == OWN_NONE) && (sel_idle == OWN_J3))) ? e_w_ready : 1'b0;

    assign j0_r_valid   = (owner_q == OWN_J0) ? e_r_valid : 1'b0;
    assign j1_r_valid   = (owner_q == OWN_J1) ? e_r_valid : 1'b0;
    assign j2_r_valid   = (owner_q == OWN_J2) ? e_r_valid : 1'b0;
    assign j3_r_valid   = (owner_q == OWN_J3) ? e_r_valid : 1'b0;

    assign j0_r_data    = e_r_data;
    assign j1_r_data    = e_r_data;
    assign j2_r_data    = e_r_data;
    assign j3_r_data    = e_r_data;

    assign j0_r_last    = (owner_q == OWN_J0) ? e_r_last : 1'b0;
    assign j1_r_last    = (owner_q == OWN_J1) ? e_r_last : 1'b0;
    assign j2_r_last    = (owner_q == OWN_J2) ? e_r_last : 1'b0;
    assign j3_r_last    = (owner_q == OWN_J3) ? e_r_last : 1'b0;

    assign j0_done      = (owner_q == OWN_J0) ? e_done : 1'b0;
    assign j1_done      = (owner_q == OWN_J1) ? e_done : 1'b0;
    assign j2_done      = (owner_q == OWN_J2) ? e_done : 1'b0;
    assign j3_done      = (owner_q == OWN_J3) ? e_done : 1'b0;

    assign j0_done_code = (owner_q == OWN_J0) ? e_done_code : 4'd0;
    assign j1_done_code = (owner_q == OWN_J1) ? e_done_code : 4'd0;
    assign j2_done_code = (owner_q == OWN_J2) ? e_done_code : 4'd0;
    assign j3_done_code = (owner_q == OWN_J3) ? e_done_code : 4'd0;

    assign j0_busy      = (owner_q == OWN_J0);
    assign j1_busy      = (owner_q == OWN_J1);
    assign j2_busy      = (owner_q == OWN_J2);
    assign j3_busy      = (owner_q == OWN_J3);

endmodule

`default_nettype wire
