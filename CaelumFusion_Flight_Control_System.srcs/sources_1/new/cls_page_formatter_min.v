`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// cls_page_formatter_min
//------------------------------------------------------------------------------
// Produces two fixed 16-character ASCII lines.
//
// Supported pages:
//   0 = boot ladder
//   1 = summary
//   2 = counters
//   3 = build/version
//   7 = fault page
//
// Character packing:
//   line[127:120] = char 0
//   line[119:112] = char 1
//   ...
//   line[7:0]     = char 15
//==============================================================================
module cls_page_formatter_min (
    input  wire [2:0]  page_id,

    // boot ladder milestones
    input  wire        boot_clk_ok,
    input  wire        boot_timebase_ok,
    input  wire        boot_i2c_ok,
    input  wire        boot_acc_init_done,
    input  wire        boot_bmp_init_done,
    input  wire        boot_mag_init_done,
    input  wire        boot_viz_alive,
    input  wire        boot_complete,

    // summary / derived
    input  wire [15:0] alt16,
    input  wire signed [15:0] vspd16,
    input  wire signed [15:0] roll_q12,
    input  wire [15:0] head_q12_u,
    input  wire [15:0] der_seq,
    input  wire        der_valid,
    input  wire [7:0]  der_status,

    // counters
    input  wire [15:0] tx_count,
    input  wire [7:0]  timeout_count,
    input  wire [7:0]  nack_addr_count,
    input  wire [7:0]  nack_data_count,
    input  wire [15:0] cdc_count,
    input  wire [15:0] frame_count,

    // build / mode
    input  wire [15:0] build_id,

    // fault
    input  wire [7:0]  fault_code,
    input  wire [15:0] fault_arg,

    output reg  [127:0] line0,
    output reg  [127:0] line1
);

    function [7:0] asc_hex;
        input [3:0] nib;
        begin
            case (nib)
                4'h0: asc_hex = "0";
                4'h1: asc_hex = "1";
                4'h2: asc_hex = "2";
                4'h3: asc_hex = "3";
                4'h4: asc_hex = "4";
                4'h5: asc_hex = "5";
                4'h6: asc_hex = "6";
                4'h7: asc_hex = "7";
                4'h8: asc_hex = "8";
                4'h9: asc_hex = "9";
                4'hA: asc_hex = "A";
                4'hB: asc_hex = "B";
                4'hC: asc_hex = "C";
                4'hD: asc_hex = "D";
                4'hE: asc_hex = "E";
                default: asc_hex = "F";
            endcase
        end
    endfunction

    function [31:0] asc_dec4_u16;
        input [15:0] v;
        reg [15:0] n;
        reg [7:0] d3, d2, d1, d0;
        begin
            n  = v;
            d3 = "0" + ((n / 1000) % 10);
            d2 = "0" + ((n / 100)  % 10);
            d1 = "0" + ((n / 10)   % 10);
            d0 = "0" + ( n         % 10);
            asc_dec4_u16 = {d3, d2, d1, d0};
        end
    endfunction

    function [39:0] asc_dec5_u16;
        input [15:0] v;
        reg [15:0] n;
        reg [7:0] d4, d3, d2, d1, d0;
        begin
            n  = v;
            d4 = "0" + ((n / 10000) % 10);
            d3 = "0" + ((n / 1000)  % 10);
            d2 = "0" + ((n / 100)   % 10);
            d1 = "0" + ((n / 10)    % 10);
            d0 = "0" + ( n          % 10);
            asc_dec5_u16 = {d4, d3, d2, d1, d0};
        end
    endfunction

    function [31:0] asc_sdec4_s16;
        input signed [15:0] v;
        reg signch;
        reg [15:0] mag;
        reg [7:0] s;
        reg [7:0] d2, d1, d0;
        begin
            signch = v[15];
            mag    = signch ? $unsigned(-v) : $unsigned(v);
            if (signch) s = "-"; else s = "+";
            d2 = "0" + ((mag / 100) % 10);
            d1 = "0" + ((mag / 10)  % 10);
            d0 = "0" + ( mag        % 10);
            asc_sdec4_s16 = {s, d2, d1, d0};
        end
    endfunction

    reg [31:0] alt4;
    reg [31:0] hdg4;
    reg [31:0] der4;
    reg [31:0] tx4;
    reg [31:0] cdc4;
    reg [31:0] frm4;
    reg [31:0] bld4;
    reg [31:0] flt4;
    reg [31:0] arg4;
    reg [31:0] rol4;

    always @(*) begin
        alt4 = asc_dec4_u16(alt16);
        hdg4 = asc_dec4_u16(head_q12_u);
        der4 = asc_dec4_u16(der_seq);
        tx4  = asc_dec4_u16(tx_count);
        cdc4 = asc_dec4_u16(cdc_count);
        frm4 = asc_dec4_u16(frame_count);
        bld4 = asc_dec4_u16(build_id);
        flt4 = asc_dec4_u16({8'd0, fault_code});
        arg4 = asc_dec4_u16(fault_arg);
        rol4 = asc_sdec4_s16(roll_q12);

        case (page_id)
            3'd0: begin
                if (!boot_clk_ok) begin
                    line0 = {"BOOT WAIT CLK   "};
                    line1 = {"RST/CLK PENDING "};
                end else if (!boot_timebase_ok) begin
                    line0 = {"BOOT CLK OK     "};
                    line1 = {"TIMEBASE WAIT   "};
                end else if (!boot_i2c_ok) begin
                    line0 = {"BOOT CLK/TB OK  "};
                    line1 = {"I2C INIT...     "};
                end else if (!boot_acc_init_done) begin
                    line0 = {"I2C OK          "};
                    line1 = {"ACC INIT...     "};
                end else if (!boot_bmp_init_done) begin
                    line0 = {"ACC INIT OK     "};
                    line1 = {"BMP INIT...     "};
                end else if (!boot_mag_init_done) begin
                    line0 = {"BMP INIT OK     "};
                    line1 = {"MAG INIT...     "};
                end else if (!boot_viz_alive) begin
                    line0 = {"SNSR INIT OK    "};
                    line1 = {"VIZ LINK WAIT   "};
                end else if (!boot_complete) begin
                    line0 = {"VIZ LINK OK     "};
                    line1 = {"FINALIZING...   "};
                end else begin
                    line0 = {"INIT COMPLETE   "};
                    line1 = {"MODE ENGINEER   "};
                end
            end

            3'd1: begin
                line0 = {"ALT", alt4, " HD", hdg4, "  "};
                line1 = {"R", rol4, " D", der4, " ",
                         (der_valid ? "O" : "X"),
                         asc_hex(der_status[7:4]),
                         asc_hex(der_status[3:0]),
                         " "};
            end

            3'd2: begin
                line0 = {"TX", tx4, " T",
                         asc_hex(timeout_count[7:4]),
                         asc_hex(timeout_count[3:0]),
                         " N",
                         asc_hex(nack_addr_count[7:4]),
                         asc_hex(nack_addr_count[3:0]),
                         " "};
                line1 = {"ND",
                         asc_hex(nack_data_count[7:4]),
                         asc_hex(nack_data_count[3:0]),
                         " CDC", cdc4, " "};
            end

            3'd3: begin
                line0 = {"BLD", bld4, " PAGE3  "};
                line1 = {"MODE ENG UART   "};
            end

            3'd7: begin
                line0 = {"FAULT ", flt4, "     "};
                line1 = {"ARG   ", arg4, "     "};
            end

            default: begin
                line0 = {"CLS PAGE INVALID"};
                line1 = {"                "};
            end
        endcase
    end

endmodule

`default_nettype wire