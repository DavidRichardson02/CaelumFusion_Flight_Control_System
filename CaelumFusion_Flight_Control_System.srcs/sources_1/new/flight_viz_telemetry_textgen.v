`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// flight_viz_telemetry_textgen
//------------------------------------------------------------------------------
// ROLE
//   Convert PIX-local telemetry fields into fixed ASCII line buses and semantic
//   colors for the telemetry text overlay.
//
// FROZEN TEXT CONTRACT
//
//   Top strip (15 chars each)
//     "BMP V 00 014 7A"
//     "ACC V 00 004 2C"
//     "MAG V 00 098 51"
//
//   Left panel (18 chars each)
//     "ROLL +0012 deg    "
//     "RFRS 1 ACCV 1     "
//     "AAGE 0004ms       "
//     "ASEQ 012C         "
//     "ATT  DERIVED ACC  "
//
//   Middle panel (18 chars each)
//     "ALT  +012345 cm   "
//     "VSPD -000123 cm/s "
//     "AF1 VF1 DV1       "
//     "BREF V1 0014ms    "
//     "BSEQ 007A ST 00   "
//
//   Right panel (18 chars each)
//     "HEAD 00123 deg    "
//     "HFRS 1 MAGV 1     "
//     "MAGE 0098ms       "
//     "MSEQ 0051         "
//     "HDG DERIVED MAG   "
//
//   Bottom strip (48 chars)
//     "PV0000 PI0000 AL00 AG0044 SQ0F5A NK0000 TO0000  "
//
// COLOR RULES
//   Raw top-strip colors
//     gray   : status == 8'h01
//     red    : !valid
//     yellow : valid but stale, or valid with nonzero non-01 status
//     green  : valid, fresh, status == 8'h00
//
//   Panel colors
//     left   : derived validity + roll freshness + derived status
//     middle : derived validity + BOTH alt/vspd freshness + derived status
//     right  : derived validity + heading freshness + derived status
//
//   Bottom strip colors
//     gray   : PMON1 path not initialized
//     white  : PMON1 valid and no PMON1/I2C alert condition
//     yellow : stale PMON1 age or one I2C error counter nonzero
//     red    : invalid PMON1 status, ADM1191 alert bits, or both I2C counters
//==============================================================================
module flight_viz_telemetry_textgen (
    //--------------------------------------------------------------------------
    // PIX-local raw summaries
    //--------------------------------------------------------------------------
    input  wire        bmp_valid_pix,
    input  wire [7:0]  bmp_status_pix,
    input  wire [15:0] bmp_age_ms_pix,
    input  wire [15:0] bmp_seq_pix,

    input  wire        acc_valid_pix,
    input  wire [7:0]  acc_status_pix,
    input  wire [15:0] acc_age_ms_pix,
    input  wire [15:0] acc_seq_pix,

    input  wire        mag_valid_pix,
    input  wire [7:0]  mag_status_pix,
    input  wire [15:0] mag_age_ms_pix,
    input  wire [15:0] mag_seq_pix,

    input  wire        pwr_valid_pix,
    input  wire [7:0]  pwr_status_pix,
    input  wire [15:0] pwr_age_ms_pix,
    input  wire [15:0] pwr_seq_pix,
    input  wire [11:0] pwr_voltage_code_pix,
    input  wire [11:0] pwr_current_code_pix,
    input  wire [7:0]  pwr_alert_status_pix,

    //--------------------------------------------------------------------------
    // PIX-local derived summary
    //--------------------------------------------------------------------------
    input  wire        der_valid_pix,
    input  wire [7:0]  der_status_pix,

    input  wire        der_alt_fresh_pix,
    input  wire        der_vspd_fresh_pix,
    input  wire        der_roll_fresh_pix,
    input  wire        der_head_fresh_pix,

    input  wire [15:0] der_bmp_seq_ref_pix,
    input  wire [15:0] der_acc_seq_ref_pix,
    input  wire [15:0] der_mag_seq_ref_pix,

    input  wire [15:0] der_bmp_age_ms_pix,
    input  wire [15:0] der_acc_age_ms_pix,
    input  wire [15:0] der_mag_age_ms_pix,

    input  wire        der_bmp_valid_ref_pix,
    input  wire        der_acc_valid_ref_pix,
    input  wire        der_mag_valid_ref_pix,

    input  wire [31:0] der_altitude_cm_pix,
    input  wire [31:0] der_vertical_speed_cms_pix,
    input  wire [31:0] der_roll_mdeg_pix,
    input  wire [31:0] der_heading_mdeg_pix,

    //--------------------------------------------------------------------------
    // Platform health
    //--------------------------------------------------------------------------
    input  wire [15:0] i2c_nack_count_pix,
    input  wire [15:0] i2c_timeout_count_pix,
    input  wire [15:0] txn_rate_hz_pix,
    input  wire [31:0] cdc_update_count_pix,
    input  wire [31:0] frame_count_pix,
    input  wire [31:0] build_id_pix,
    input  wire [15:0] schema_word_pix,

    //--------------------------------------------------------------------------
    // Top strip: 15 chars each
    //--------------------------------------------------------------------------
    output reg [119:0] top_bmp_line,
    output reg [119:0] top_acc_line,
    output reg [119:0] top_mag_line,

    output reg [23:0]  top_bmp_rgb,
    output reg [23:0]  top_acc_rgb,
    output reg [23:0]  top_mag_rgb,

    //--------------------------------------------------------------------------
    // Left / middle / right panels: 18 chars each
    //--------------------------------------------------------------------------
    output reg [143:0] left_line0,
    output reg [143:0] left_line1,
    output reg [143:0] left_line2,
    output reg [143:0] left_line3,
    output reg [143:0] left_line4,

    output reg [143:0] mid_line0,
    output reg [143:0] mid_line1,
    output reg [143:0] mid_line2,
    output reg [143:0] mid_line3,
    output reg [143:0] mid_line4,

    output reg [143:0] right_line0,
    output reg [143:0] right_line1,
    output reg [143:0] right_line2,
    output reg [143:0] right_line3,
    output reg [143:0] right_line4,

    output reg [23:0]  left_rgb,
    output reg [23:0]  mid_rgb,
    output reg [23:0]  right_rgb,

    //--------------------------------------------------------------------------
    // Bottom strip: 48 chars
    //--------------------------------------------------------------------------
    output reg [383:0] bottom_line,
    output reg [23:0]  bottom_rgb
);

    //--------------------------------------------------------------------------
    // Freshness thresholds for raw top-strip lanes
    //--------------------------------------------------------------------------
    localparam [15:0] BMP_STALE_MS = 16'd150;
    localparam [15:0] ACC_STALE_MS = 16'd80;
    localparam [15:0] MAG_STALE_MS = 16'd250;

    //--------------------------------------------------------------------------
    // Semantic colors
    //--------------------------------------------------------------------------
    localparam [23:0] RGB_GREEN  = 24'h70D070;
    localparam [23:0] RGB_YELLOW = 24'hFFD060;
    localparam [23:0] RGB_RED    = 24'hFF4040;
    localparam [23:0] RGB_GRAY   = 24'h808890;
    localparam [23:0] RGB_WHITE  = 24'hBFC8D8;

    //--------------------------------------------------------------------------
    // Single-character helpers
    //--------------------------------------------------------------------------
    function [7:0] ch_dec;
        input [3:0] d;
        begin
            ch_dec = 8'h30 + d;
        end
    endfunction

    function [7:0] ch_hex;
        input [3:0] nib;
        begin
            case (nib)
                4'h0: ch_hex = 8'h30;
                4'h1: ch_hex = 8'h31;
                4'h2: ch_hex = 8'h32;
                4'h3: ch_hex = 8'h33;
                4'h4: ch_hex = 8'h34;
                4'h5: ch_hex = 8'h35;
                4'h6: ch_hex = 8'h36;
                4'h7: ch_hex = 8'h37;
                4'h8: ch_hex = 8'h38;
                4'h9: ch_hex = 8'h39;
                4'hA: ch_hex = 8'h41;
                4'hB: ch_hex = 8'h42;
                4'hC: ch_hex = 8'h43;
                4'hD: ch_hex = 8'h44;
                4'hE: ch_hex = 8'h45;
                default: ch_hex = 8'h46;
            endcase
        end
    endfunction

    function [7:0] ch_bit01;
        input bit_in;
        begin
            ch_bit01 = bit_in ? 8'h31 : 8'h30; // '1' : '0'
        end
    endfunction

    function [7:0] ch_valid_top;
        input valid_in;
        begin
            ch_valid_top = valid_in ? 8'h56 : 8'h2D; // 'V' or '-'
        end
    endfunction

    function [7:0] ch_valid_ref;
        input valid_in;
        begin
            ch_valid_ref = valid_in ? 8'h31 : 8'h30; // '1' or '0'
        end
    endfunction

    //--------------------------------------------------------------------------
    // ASCII packed helpers
    //--------------------------------------------------------------------------
    function [15:0] ascii_hex8;
        input [7:0] val;
        begin
            ascii_hex8 = {ch_hex(val[7:4]), ch_hex(val[3:0])};
        end
    endfunction

    function [31:0] ascii_hex16;
        input [15:0] val;
        begin
            ascii_hex16 = {
                ch_hex(val[15:12]),
                ch_hex(val[11:8]),
                ch_hex(val[7:4]),
                ch_hex(val[3:0])
            };
        end
    endfunction

    function [23:0] ascii_dec3;
        input [15:0] val;
        reg [15:0] v;
        reg [3:0] d2, d1, d0;
        begin
            v  = (val > 16'd999) ? 16'd999 : val;
            d2 = (v / 100) % 10;
            d1 = (v / 10)  % 10;
            d0 =  v % 10;
            ascii_dec3 = {ch_dec(d2), ch_dec(d1), ch_dec(d0)};
        end
    endfunction

    function [31:0] ascii_dec4;
        input [15:0] val;
        reg [15:0] v;
        reg [3:0] d3, d2, d1, d0;
        begin
            v  = (val > 16'd9999) ? 16'd9999 : val;
            d3 = (v / 1000) % 10;
            d2 = (v / 100)  % 10;
            d1 = (v / 10)   % 10;
            d0 =  v % 10;
            ascii_dec4 = {ch_dec(d3), ch_dec(d2), ch_dec(d1), ch_dec(d0)};
        end
    endfunction

    function [47:0] ascii_dec6_u32;
        input [31:0] val;
        reg [31:0] v;
        reg [3:0] d5, d4, d3, d2, d1, d0;
        begin
            v  = (val > 32'd999999) ? 32'd999999 : val;
            d5 = (v / 100000) % 10;
            d4 = (v / 10000)  % 10;
            d3 = (v / 1000)   % 10;
            d2 = (v / 100)    % 10;
            d1 = (v / 10)     % 10;
            d0 =  v % 10;
            ascii_dec6_u32 = {
                ch_dec(d5), ch_dec(d4), ch_dec(d3),
                ch_dec(d2), ch_dec(d1), ch_dec(d0)
            };
        end
    endfunction

    // sign + 6 digits
    function [55:0] ascii_sdec7_s32;
        input [31:0] val;
        reg sign_bit;
        reg [31:0] mag;
        reg [31:0] v;
        reg [3:0] d5, d4, d3, d2, d1, d0;
        begin
            sign_bit = val[31];
            if (sign_bit)
                mag = (~val) + 32'd1;
            else
                mag = val;

            v  = (mag > 32'd999999) ? 32'd999999 : mag;
            d5 = (v / 100000) % 10;
            d4 = (v / 10000)  % 10;
            d3 = (v / 1000)   % 10;
            d2 = (v / 100)    % 10;
            d1 = (v / 10)     % 10;
            d0 =  v % 10;

            ascii_sdec7_s32 = {
                sign_bit ? 8'h2D : 8'h2B, // '-' : '+'
                ch_dec(d5), ch_dec(d4), ch_dec(d3),
                ch_dec(d2), ch_dec(d1), ch_dec(d0)
            };
        end
    endfunction

    // sign + 4 digits, derived from signed millidegrees
    function [39:0] ascii_sdeg5_from_mdeg;
        input [31:0] val_mdeg;
        reg sign_bit;
        reg [31:0] mag_mdeg;
        reg [31:0] deg_u;
        reg [3:0] d3, d2, d1, d0;
        begin
            sign_bit = val_mdeg[31];
            if (sign_bit)
                mag_mdeg = (~val_mdeg) + 32'd1;
            else
                mag_mdeg = val_mdeg;

            deg_u = mag_mdeg / 32'd1000;
            if (deg_u > 32'd9999)
                deg_u = 32'd9999;

            d3 = (deg_u / 1000) % 10;
            d2 = (deg_u / 100)  % 10;
            d1 = (deg_u / 10)   % 10;
            d0 =  deg_u % 10;

            ascii_sdeg5_from_mdeg = {
                sign_bit ? 8'h2D : 8'h2B,
                ch_dec(d3), ch_dec(d2), ch_dec(d1), ch_dec(d0)
            };
        end
    endfunction

    // 5 unsigned digits, heading integer degrees
    function [39:0] ascii_udeg5_from_mdeg;
        input [31:0] val_mdeg;
        reg [31:0] deg_u;
        reg [3:0] d4, d3, d2, d1, d0;
        begin
            deg_u = (val_mdeg / 32'd1000) % 32'd360;

            d4 = (deg_u / 10000) % 10;
            d3 = (deg_u / 1000)  % 10;
            d2 = (deg_u / 100)   % 10;
            d1 = (deg_u / 10)    % 10;
            d0 =  deg_u % 10;

            ascii_udeg5_from_mdeg = {
                ch_dec(d4), ch_dec(d3), ch_dec(d2), ch_dec(d1), ch_dec(d0)
            };
        end
    endfunction

    //--------------------------------------------------------------------------
    // Color helpers
    //--------------------------------------------------------------------------
    function [23:0] raw_semantic_rgb;
        input        valid_in;
        input [7:0]  status_in;
        input [15:0] age_ms_in;
        input [15:0] stale_ms_in;
        begin
            if (status_in == 8'h01)
                raw_semantic_rgb = RGB_GRAY;
            else if (!valid_in)
                raw_semantic_rgb = RGB_RED;
            else if ((status_in != 8'h00) || (age_ms_in > stale_ms_in))
                raw_semantic_rgb = RGB_YELLOW;
            else
                raw_semantic_rgb = RGB_GREEN;
        end
    endfunction

    function [23:0] derived_semantic_rgb;
        input       valid_in;
        input       fresh_in;
        input [7:0] status_in;
        begin
            if (status_in == 8'h01)
                derived_semantic_rgb = RGB_GRAY;
            else if (!valid_in)
                derived_semantic_rgb = RGB_RED;
            else if ((status_in != 8'h00) || !fresh_in)
                derived_semantic_rgb = RGB_YELLOW;
            else
                derived_semantic_rgb = RGB_GREEN;
        end
    endfunction

    function [23:0] middle_semantic_rgb;
        input       valid_in;
        input       alt_fresh_in;
        input       vspd_fresh_in;
        input [7:0] status_in;
        begin
            if (status_in == 8'h01)
                middle_semantic_rgb = RGB_GRAY;
            else if (!valid_in)
                middle_semantic_rgb = RGB_RED;
            else if ((status_in != 8'h00) || !alt_fresh_in || !vspd_fresh_in)
                middle_semantic_rgb = RGB_YELLOW;
            else
                middle_semantic_rgb = RGB_GREEN;
        end
    endfunction

    function [23:0] bottom_semantic_rgb;
        input        pwr_valid_in;
        input [7:0]  pwr_status_in;
        input [15:0] pwr_age_ms_in;
        input [7:0]  pwr_alert_status_in;
        input [15:0] nack_in;
        input [15:0] timeout_in;
        begin
            if (!pwr_valid_in && (pwr_status_in == 8'h01))
                bottom_semantic_rgb = RGB_GRAY;
            else if (!pwr_valid_in ||
                     (pwr_status_in != 8'h00) ||
                     (pwr_alert_status_in[5:1] != 5'd0) ||
                     ((nack_in != 16'd0) && (timeout_in != 16'd0)))
                bottom_semantic_rgb = RGB_RED;
            else if ((pwr_age_ms_in > 16'd1000) ||
                     (nack_in != 16'd0) ||
                     (timeout_in != 16'd0))
                bottom_semantic_rgb = RGB_YELLOW;
            else
                bottom_semantic_rgb = RGB_WHITE;
        end
    endfunction

    //--------------------------------------------------------------------------
    // Combinational text and color generation
    //--------------------------------------------------------------------------
    always @(*) begin
        //----------------------------------------------------------------------
        // Top strip: 15 chars each
        //----------------------------------------------------------------------
        top_bmp_line = {
            8'h42, 8'h4D, 8'h50, 8'h20,                         // "BMP "
            ch_valid_top(bmp_valid_pix), 8'h20,                // "V "
            ascii_hex8(bmp_status_pix), 8'h20,                 // "00 "
            ascii_dec3(bmp_age_ms_pix), 8'h20,                 // "014 "
            ascii_hex8(bmp_seq_pix[7:0])                       // "7A"
        };

        top_acc_line = {
            8'h41, 8'h43, 8'h43, 8'h20,                         // "ACC "
            ch_valid_top(acc_valid_pix), 8'h20,
            ascii_hex8(acc_status_pix), 8'h20,
            ascii_dec3(acc_age_ms_pix), 8'h20,
            ascii_hex8(acc_seq_pix[7:0])
        };

        top_mag_line = {
            8'h4D, 8'h41, 8'h47, 8'h20,                         // "MAG "
            ch_valid_top(mag_valid_pix), 8'h20,
            ascii_hex8(mag_status_pix), 8'h20,
            ascii_dec3(mag_age_ms_pix), 8'h20,
            ascii_hex8(mag_seq_pix[7:0])
        };

        top_bmp_rgb = raw_semantic_rgb(bmp_valid_pix, bmp_status_pix, bmp_age_ms_pix, BMP_STALE_MS);
        top_acc_rgb = raw_semantic_rgb(acc_valid_pix, acc_status_pix, acc_age_ms_pix, ACC_STALE_MS);
        top_mag_rgb = raw_semantic_rgb(mag_valid_pix, mag_status_pix, mag_age_ms_pix, MAG_STALE_MS);

        //----------------------------------------------------------------------
        // Left panel: 18 chars each
        //----------------------------------------------------------------------
        left_line0 = {
            8'h52,8'h4F,8'h4C,8'h4C,8'h20,                      // "ROLL "
            ascii_sdeg5_from_mdeg(der_roll_mdeg_pix),          // "+0012"
            8'h20,                                             // " "
            8'h64,8'h65,8'h67,                                 // "deg"
            8'h20,8'h20,8'h20,8'h20                            // 4 spaces
        };

        left_line1 = {
            8'h52,8'h46,8'h52,8'h53,8'h20,                      // "RFRS "
            ch_bit01(der_roll_fresh_pix),
            8'h20,
            8'h41,8'h43,8'h43,8'h56,8'h20,                      // "ACCV "
            ch_bit01(der_acc_valid_ref_pix),
            8'h20,8'h20,8'h20,8'h20,8'h20                       // 5 spaces
        };

        left_line2 = {
            8'h41,8'h41,8'h47,8'h45,8'h20,                      // "AAGE "
            ascii_dec4(der_acc_age_ms_pix),
            8'h6D,8'h73,                                       // "ms"
            8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20          // 7 spaces
        };

        left_line3 = {
            8'h41,8'h53,8'h45,8'h51,8'h20,                      // "ASEQ "
            ascii_hex16(der_acc_seq_ref_pix),
            8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20
        };

        left_line4 = {
            8'h41,8'h54,8'h54,8'h20,8'h20,                      // "ATT  "
            8'h44,8'h45,8'h52,8'h49,8'h56,8'h45,8'h44,          // "DERIVED"
            8'h20,                                             // " "
            8'h41,8'h43,8'h43,                                 // "ACC"
            8'h20,8'h20                                        // 2 spaces
        };

        //----------------------------------------------------------------------
        // Middle panel: 18 chars each
        //----------------------------------------------------------------------
        mid_line0 = {
            8'h41,8'h4C,8'h54,8'h20,8'h20,                      // "ALT  "
            ascii_sdec7_s32(der_altitude_cm_pix),              // "+012345"
            8'h20,
            8'h63,8'h6D,                                       // "cm"
            8'h20,8'h20,8'h20                                  // 3 spaces
        };

        mid_line1 = {
            8'h56,8'h53,8'h50,8'h44,8'h20,                      // "VSPD "
            ascii_sdec7_s32(der_vertical_speed_cms_pix),       // "-000123"
            8'h20,
            8'h63,8'h6D,8'h2F,8'h73,                           // "cm/s"
            8'h20                                              // 1 space
        };

        mid_line2 = {
            8'h41,8'h46,                                       // "AF"
            ch_bit01(der_alt_fresh_pix),
            8'h20,
            8'h56,8'h46,                                       // "VF"
            ch_bit01(der_vspd_fresh_pix),
            8'h20,
            8'h44,8'h56,                                       // "DV"
            ch_bit01(der_valid_pix),
            8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20
        };

        mid_line3 = {
            8'h42,8'h52,8'h45,8'h46,8'h20,                      // "BREF "
            8'h56,                                             // "V"
            ch_valid_ref(der_bmp_valid_ref_pix),
            8'h20,
            ascii_dec4(der_bmp_age_ms_pix),
            8'h6D,8'h73,                                       // "ms"
            8'h20,8'h20,8'h20,8'h20                            // 4 spaces
        };

        mid_line4 = {
            8'h42,8'h53,8'h45,8'h51,8'h20,                      // "BSEQ "
            ascii_hex16(der_bmp_seq_ref_pix),
            8'h20,
            8'h53,8'h54,                                       // "ST"
            8'h20,
            ascii_hex8(der_status_pix),
            8'h20,8'h20,8'h20
        };

        //----------------------------------------------------------------------
        // Right panel: 18 chars each
        //----------------------------------------------------------------------
        right_line0 = {
            8'h48,8'h45,8'h41,8'h44,8'h20,                      // "HEAD "
            ascii_udeg5_from_mdeg(der_heading_mdeg_pix),       // "00123"
            8'h20,
            8'h64,8'h65,8'h67,                                 // "deg"
            8'h20,8'h20,8'h20,8'h20                            // 4 spaces
        };

        right_line1 = {
            8'h48,8'h46,8'h52,8'h53,8'h20,                      // "HFRS "
            ch_bit01(der_head_fresh_pix),
            8'h20,
            8'h4D,8'h41,8'h47,8'h56,8'h20,                      // "MAGV "
            ch_bit01(der_mag_valid_ref_pix),
            8'h20,8'h20,8'h20,8'h20,8'h20
        };

        right_line2 = {
            8'h4D,8'h41,8'h47,8'h45,8'h20,                      // "MAGE "
            ascii_dec4(der_mag_age_ms_pix),
            8'h6D,8'h73,                                       // "ms"
            8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20
        };

        right_line3 = {
            8'h4D,8'h53,8'h45,8'h51,8'h20,                      // "MSEQ "
            ascii_hex16(der_mag_seq_ref_pix),
            8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20
        };

        right_line4 = {
            8'h48,8'h44,8'h47,8'h20,                            // "HDG "
            8'h44,8'h45,8'h52,8'h49,8'h56,8'h45,8'h44,          // "DERIVED"
            8'h20,
            8'h4D,8'h41,8'h47,                                 // "MAG"
            8'h20,8'h20,8'h20                                  // 3 spaces
        };

        //----------------------------------------------------------------------
        // Region colors
        //----------------------------------------------------------------------
        left_rgb  = derived_semantic_rgb(der_valid_pix, der_roll_fresh_pix, der_status_pix);
        mid_rgb   = middle_semantic_rgb(der_valid_pix, der_alt_fresh_pix, der_vspd_fresh_pix, der_status_pix);
        right_rgb = derived_semantic_rgb(der_valid_pix, der_head_fresh_pix, der_status_pix);

        //----------------------------------------------------------------------
        // Bottom strip: 48 chars
        //
        // "PV0000 PI0000 AL00 AG0044 SQ0F5A NK0000 TO0000  "
        //----------------------------------------------------------------------
        bottom_line = {
            8'h50,8'h56,                                       // "PV"
            ascii_hex16({4'd0, pwr_voltage_code_pix}),
            8'h20,

            8'h50,8'h49,                                       // "PI"
            ascii_hex16({4'd0, pwr_current_code_pix}),
            8'h20,

            8'h41,8'h4C,                                       // "AL"
            ascii_hex8(pwr_alert_status_pix),
            8'h20,

            8'h41,8'h47,                                       // "AG"
            ascii_dec4(pwr_age_ms_pix),
            8'h20,

            8'h53,8'h51,                                       // "SQ"
            ascii_hex16(pwr_seq_pix),
            8'h20,

            8'h4E,8'h4B,                                       // "NK"
            ascii_hex16(i2c_nack_count_pix),
            8'h20,

            8'h54,8'h4F,                                       // "TO"
            ascii_hex16(i2c_timeout_count_pix),
            8'h20,8'h20
        };

        bottom_rgb = bottom_semantic_rgb(
            pwr_valid_pix,
            pwr_status_pix,
            pwr_age_ms_pix,
            pwr_alert_status_pix,
            i2c_nack_count_pix,
            i2c_timeout_count_pix
        );

        //----------------------------------------------------------------------
        // frame_count_pix and legacy platform IDs remain available for later
        // debug-page expansion without changing the visible line widths.
        //----------------------------------------------------------------------
    end

endmodule

`default_nettype wire
