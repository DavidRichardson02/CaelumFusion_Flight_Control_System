`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// flight_viz_telemetry_compositor
//------------------------------------------------------------------------------
// ROLE
//   Place frozen telemetry text objects at fixed screen coordinates and render
//   them through one shared 3x5 glyph datapath.
//
// CANONICAL MERGED CONTRACT
//   - Top strip lanes are exactly 15 chars each.
//   - Left/mid/right panel lines are exactly 18 chars each.
//   - Bottom strip is exactly 48 chars.
//   - Character buses are rendered completely; no 16-char truncation is used.
//   - Per-region RGB is consumed exactly as supplied by textgen.
//   - Rendering remains one shared low-area per-pixel text engine.
//   - Each text class is clipped to a deliberate screen band so telemetry never
//     spills into the primary instruments.
//
// MERGED FEATURES PRESERVED
//   From the earlier compositor:
//     - richer punctuation glyph support
//     - single shared renderer architecture
//     - frozen layout intent
//
//   From the newer compositor:
//     - exact 15/18/48 rendering
//     - normalized 48-char line representation
//     - no truncation of 18-char side-panel lines
//     - clean local glyph-space evaluation
//
// RENDERING MODEL
//   - At most one text object is selected for the current pixel.
//   - The current pixel is transformed into local object coordinates.
//   - One character index and one 3x5 glyph row are decoded.
//   - overlay_on / overlay_rgb are purely combinational.
//
// GLYPH GEOMETRY
//   - Nominal glyph bitmap is 3x5.
//   - Normal telemetry uses GLYPH_SCALE / CHAR_ADV_X.
//   - The engineering debug strip uses a fixed 1x glyph path.
//   - Visible ink width remains 3*scale.
//==============================================================================
module flight_viz_telemetry_compositor #(
    parameter integer GLYPH_SCALE = 2,
    parameter integer CHAR_ADV_X  = 7
)(
    input  wire        clk_pix,
    input  wire        rst_pix,

    input  wire [9:0]  hcount,
    input  wire [9:0]  vcount,
    input  wire        active_video,

    //--------------------------------------------------------------------------
    // Top strip: 15 chars each
    //--------------------------------------------------------------------------
    input  wire [119:0] top_bmp_line,
    input  wire [119:0] top_acc_line,
    input  wire [119:0] top_mag_line,

    input  wire [23:0]  top_bmp_rgb,
    input  wire [23:0]  top_acc_rgb,
    input  wire [23:0]  top_mag_rgb,

    //--------------------------------------------------------------------------
    // Left / middle / right panels: 18 chars each
    //--------------------------------------------------------------------------
    input  wire [143:0] left_line0,
    input  wire [143:0] left_line1,
    input  wire [143:0] left_line2,
    input  wire [143:0] left_line3,
    input  wire [143:0] left_line4,

    input  wire [143:0] mid_line0,
    input  wire [143:0] mid_line1,
    input  wire [143:0] mid_line2,
    input  wire [143:0] mid_line3,
    input  wire [143:0] mid_line4,

    input  wire [143:0] right_line0,
    input  wire [143:0] right_line1,
    input  wire [143:0] right_line2,
    input  wire [143:0] right_line3,
    input  wire [143:0] right_line4,

    input  wire [23:0]  left_rgb,
    input  wire [23:0]  mid_rgb,
    input  wire [23:0]  right_rgb,

    //--------------------------------------------------------------------------
    // Bottom strip: 48 chars
    //--------------------------------------------------------------------------
    input  wire [383:0] bottom_line,
    input  wire [23:0]  bottom_rgb,

    //--------------------------------------------------------------------------
    // Rendered overlay output
    //--------------------------------------------------------------------------
    output reg          overlay_on,
    output reg  [23:0]  overlay_rgb
);

    //--------------------------------------------------------------------------
    // Preserve contract pins even though datapath is combinational.
    //--------------------------------------------------------------------------
    wire _unused_ok = clk_pix ^ rst_pix ^ 1'b0;

    //--------------------------------------------------------------------------
    // Professional 640x480 layout geometry.
    //
    // The top strip is intentionally compact. Panel title and telemetry rows
    // live in a shallow title band above the primary instruments, and the
    // bottom engineering counters are demoted to 1x glyphs.
    //--------------------------------------------------------------------------
    localparam integer GLYPH_W = 3;
    localparam integer GLYPH_H = 5;

    localparam integer DEBUG_SCALE = 1;
    localparam integer DEBUG_ADV_X = 4;

    localparam [9:0] X_TOP_BMP = 10'd20;
    localparam [9:0] X_TOP_ACC = 10'd226;
    localparam [9:0] X_TOP_MAG = 10'd432;
    localparam [9:0] Y_TOP     = 10'd6;

    localparam [9:0] X_LEFT    = 10'd10;
    localparam [9:0] X_MID     = 10'd236;
    localparam [9:0] X_RIGHT   = 10'd456;
    localparam [9:0] Y_TITLE   = 10'd30;
    localparam [9:0] Y_ROW0    = 10'd44;
    localparam [9:0] Y_ROW1    = 10'd56;
    localparam [9:0] Y_ROW2    = 10'd68;
    localparam [9:0] Y_ROW3    = 10'd80;
    localparam [9:0] Y_ROW4    = 10'd92;

    localparam [9:0] X_BOTTOM  = 10'd10;
    localparam [9:0] Y_BOTTOM  = 10'd470;

    localparam [9:0] CLIP_TOP_Y0    = 10'd0;
    localparam [9:0] CLIP_TOP_Y1    = 10'd27;
    localparam [9:0] CLIP_PANEL_Y0  = 10'd28;
    localparam [9:0] CLIP_PANEL_Y1  = 10'd105;
    localparam [9:0] CLIP_BOTTOM_Y0 = 10'd468;
    localparam [9:0] CLIP_BOTTOM_Y1 = 10'd479;

    localparam [143:0] TITLE_LEFT  = "ALT VSPD AUTH     ";
    localparam [143:0] TITLE_MID   = "HORIZON ATT       ";
    localparam [143:0] TITLE_RIGHT = "HEADING HEALTH    ";
    localparam [23:0]  RGB_TITLE   = 24'h80D8FF;

    //--------------------------------------------------------------------------
    // Normalized 48-char selected text object
    //
    // Character 0 => [383:376]
    // Character 1 => [375:368]
    // ...
    // Character 47 => [7:0]
    //--------------------------------------------------------------------------
    reg  [383:0] sel_line_norm;
    reg  [5:0]   sel_chars;
    reg  [23:0]  sel_rgb;
    reg  [9:0]   sel_x0;
    reg  [9:0]   sel_y0;
    reg          sel_valid;
    reg          sel_debug_text;

    reg  [9:0]   local_x;
    reg  [9:0]   local_y;

    integer      char_idx_int;
    integer      glyph_row_int;
    integer      glyph_col_int;

    reg  [7:0]   glyph_char;
    reg  [2:0]   glyph_row_bits;
    reg          glyph_pixel_on;

    //--------------------------------------------------------------------------
    // Geometry helpers
    //--------------------------------------------------------------------------
    function integer text_height_px;
        input integer scale_i;
        begin
            text_height_px = GLYPH_H * scale_i;
        end
    endfunction

    function integer text_width_px;
        input integer n_chars_i;
        input integer adv_i;
        begin
            text_width_px = n_chars_i * adv_i;
        end
    endfunction

    function in_y_clip;
        input [9:0] yy;
        input [9:0] y0;
        input [9:0] y1;
        begin
            in_y_clip = (yy >= y0) && (yy <= y1);
        end
    endfunction

    //--------------------------------------------------------------------------
    // Line normalizers
    //--------------------------------------------------------------------------
    function [383:0] norm15;
        input [119:0] line15;
        begin
            norm15 = {
                line15,
                8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,
                8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,
                8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,
                8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,
                8'h20
            };
        end
    endfunction

    function [383:0] norm18;
        input [143:0] line18;
        begin
            norm18 = {
                line18,
                8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,
                8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,
                8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,
                8'h20,8'h20,8'h20,8'h20,8'h20,8'h20
            };
        end
    endfunction

    //--------------------------------------------------------------------------
    // Extract one character from normalized 48-char bus
    //--------------------------------------------------------------------------
    function [7:0] norm_char_at;
        input [383:0] line_norm;
        input [5:0]   idx;
        begin
            case (idx)
                6'd0:  norm_char_at = line_norm[383:376];
                6'd1:  norm_char_at = line_norm[375:368];
                6'd2:  norm_char_at = line_norm[367:360];
                6'd3:  norm_char_at = line_norm[359:352];
                6'd4:  norm_char_at = line_norm[351:344];
                6'd5:  norm_char_at = line_norm[343:336];
                6'd6:  norm_char_at = line_norm[335:328];
                6'd7:  norm_char_at = line_norm[327:320];
                6'd8:  norm_char_at = line_norm[319:312];
                6'd9:  norm_char_at = line_norm[311:304];
                6'd10: norm_char_at = line_norm[303:296];
                6'd11: norm_char_at = line_norm[295:288];
                6'd12: norm_char_at = line_norm[287:280];
                6'd13: norm_char_at = line_norm[279:272];
                6'd14: norm_char_at = line_norm[271:264];
                6'd15: norm_char_at = line_norm[263:256];
                6'd16: norm_char_at = line_norm[255:248];
                6'd17: norm_char_at = line_norm[247:240];
                6'd18: norm_char_at = line_norm[239:232];
                6'd19: norm_char_at = line_norm[231:224];
                6'd20: norm_char_at = line_norm[223:216];
                6'd21: norm_char_at = line_norm[215:208];
                6'd22: norm_char_at = line_norm[207:200];
                6'd23: norm_char_at = line_norm[199:192];
                6'd24: norm_char_at = line_norm[191:184];
                6'd25: norm_char_at = line_norm[183:176];
                6'd26: norm_char_at = line_norm[175:168];
                6'd27: norm_char_at = line_norm[167:160];
                6'd28: norm_char_at = line_norm[159:152];
                6'd29: norm_char_at = line_norm[151:144];
                6'd30: norm_char_at = line_norm[143:136];
                6'd31: norm_char_at = line_norm[135:128];
                6'd32: norm_char_at = line_norm[127:120];
                6'd33: norm_char_at = line_norm[119:112];
                6'd34: norm_char_at = line_norm[111:104];
                6'd35: norm_char_at = line_norm[103:96];
                6'd36: norm_char_at = line_norm[95:88];
                6'd37: norm_char_at = line_norm[87:80];
                6'd38: norm_char_at = line_norm[79:72];
                6'd39: norm_char_at = line_norm[71:64];
                6'd40: norm_char_at = line_norm[63:56];
                6'd41: norm_char_at = line_norm[55:48];
                6'd42: norm_char_at = line_norm[47:40];
                6'd43: norm_char_at = line_norm[39:32];
                6'd44: norm_char_at = line_norm[31:24];
                6'd45: norm_char_at = line_norm[23:16];
                6'd46: norm_char_at = line_norm[15:8];
                6'd47: norm_char_at = line_norm[7:0];
                default: norm_char_at = 8'h20;
            endcase
        end
    endfunction

    //--------------------------------------------------------------------------
    // Fold lowercase ASCII to uppercase so one 3x5 map is sufficient.
    //--------------------------------------------------------------------------
    function [7:0] fold_ascii_upper;
        input [7:0] ch;
        begin
            if ((ch >= 8'h61) && (ch <= 8'h7A))
                fold_ascii_upper = ch - 8'd32;
            else
                fold_ascii_upper = ch;
        end
    endfunction

    //--------------------------------------------------------------------------
    // 3x5 glyph font
    //
    // bit[2] = left pixel
    // bit[1] = mid pixel
    // bit[0] = right pixel
    //
    // This merged table preserves the richer punctuation coverage from the
    // earlier version while remaining compatible with the frozen telemetry HUD.
    //--------------------------------------------------------------------------
    function [2:0] glyph3x5_row;
        input [7:0] ch_in;
        input [2:0] row;
        reg   [7:0] ch;
        begin
            ch = fold_ascii_upper(ch_in);
            glyph3x5_row = 3'b000;

            case (ch)

                // Space and punctuation
                8'h20: glyph3x5_row = 3'b000;

                8'h2D: begin // -
                    case (row)
                        3'd2: glyph3x5_row = 3'b111;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end

                8'h5F: begin // _
                    case (row)
                        3'd4: glyph3x5_row = 3'b111;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end

                8'h2E: begin // .
                    case (row)
                        3'd4: glyph3x5_row = 3'b010;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end

                8'h2C: begin // ,
                    case (row)
                        3'd3: glyph3x5_row = 3'b010;
                        3'd4: glyph3x5_row = 3'b100;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end

                8'h27: begin // '
                    case (row)
                        3'd0: glyph3x5_row = 3'b010;
                        3'd1: glyph3x5_row = 3'b010;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end

                8'h22: begin // "
                    case (row)
                        3'd0: glyph3x5_row = 3'b101;
                        3'd1: glyph3x5_row = 3'b101;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end

                8'h2F: begin // /
                    case (row)
                        3'd0: glyph3x5_row = 3'b001;
                        3'd1: glyph3x5_row = 3'b001;
                        3'd2: glyph3x5_row = 3'b010;
                        3'd3: glyph3x5_row = 3'b100;
                        3'd4: glyph3x5_row = 3'b100;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end

                8'h5C: begin // backslash
                    case (row)
                        3'd0: glyph3x5_row = 3'b100;
                        3'd1: glyph3x5_row = 3'b100;
                        3'd2: glyph3x5_row = 3'b010;
                        3'd3: glyph3x5_row = 3'b001;
                        3'd4: glyph3x5_row = 3'b001;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end

                8'h2B: begin // +
                    case (row)
                        3'd1: glyph3x5_row = 3'b010;
                        3'd2: glyph3x5_row = 3'b111;
                        3'd3: glyph3x5_row = 3'b010;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end

                8'h3D: begin // =
                    case (row)
                        3'd1: glyph3x5_row = 3'b111;
                        3'd3: glyph3x5_row = 3'b111;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end

                8'h28: begin // (
                    case (row)
                        3'd0: glyph3x5_row = 3'b001;
                        3'd1: glyph3x5_row = 3'b010;
                        3'd2: glyph3x5_row = 3'b010;
                        3'd3: glyph3x5_row = 3'b010;
                        3'd4: glyph3x5_row = 3'b001;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end

                8'h29: begin // )
                    case (row)
                        3'd0: glyph3x5_row = 3'b100;
                        3'd1: glyph3x5_row = 3'b010;
                        3'd2: glyph3x5_row = 3'b010;
                        3'd3: glyph3x5_row = 3'b010;
                        3'd4: glyph3x5_row = 3'b100;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end

                8'h5B: begin // [
                    case (row)
                        3'd0: glyph3x5_row = 3'b011;
                        3'd1: glyph3x5_row = 3'b010;
                        3'd2: glyph3x5_row = 3'b010;
                        3'd3: glyph3x5_row = 3'b010;
                        3'd4: glyph3x5_row = 3'b011;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end

                8'h5D: begin // ]
                    case (row)
                        3'd0: glyph3x5_row = 3'b110;
                        3'd1: glyph3x5_row = 3'b010;
                        3'd2: glyph3x5_row = 3'b010;
                        3'd3: glyph3x5_row = 3'b010;
                        3'd4: glyph3x5_row = 3'b110;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end

                8'h3C: begin // <
                    case (row)
                        3'd0: glyph3x5_row = 3'b001;
                        3'd1: glyph3x5_row = 3'b010;
                        3'd2: glyph3x5_row = 3'b100;
                        3'd3: glyph3x5_row = 3'b010;
                        3'd4: glyph3x5_row = 3'b001;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end

                8'h3E: begin // >
                    case (row)
                        3'd0: glyph3x5_row = 3'b100;
                        3'd1: glyph3x5_row = 3'b010;
                        3'd2: glyph3x5_row = 3'b001;
                        3'd3: glyph3x5_row = 3'b010;
                        3'd4: glyph3x5_row = 3'b100;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end

                8'h21: begin // !
                    case (row)
                        3'd0: glyph3x5_row = 3'b010;
                        3'd1: glyph3x5_row = 3'b010;
                        3'd2: glyph3x5_row = 3'b010;
                        3'd4: glyph3x5_row = 3'b010;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end

                8'h3F: begin // ?
                    case (row)
                        3'd0: glyph3x5_row = 3'b110;
                        3'd1: glyph3x5_row = 3'b001;
                        3'd2: glyph3x5_row = 3'b010;
                        3'd4: glyph3x5_row = 3'b010;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end

                8'h3A: begin // :
                    case (row)
                        3'd1: glyph3x5_row = 3'b010;
                        3'd3: glyph3x5_row = 3'b010;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end

                // Digits
                8'h30: begin
                    case (row)
                        3'd0: glyph3x5_row = 3'b111;
                        3'd1: glyph3x5_row = 3'b101;
                        3'd2: glyph3x5_row = 3'b101;
                        3'd3: glyph3x5_row = 3'b101;
                        3'd4: glyph3x5_row = 3'b111;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end
                8'h31: begin
                    case (row)
                        3'd0: glyph3x5_row = 3'b010;
                        3'd1: glyph3x5_row = 3'b110;
                        3'd2: glyph3x5_row = 3'b010;
                        3'd3: glyph3x5_row = 3'b010;
                        3'd4: glyph3x5_row = 3'b111;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end
                8'h32: begin
                    case (row)
                        3'd0: glyph3x5_row = 3'b111;
                        3'd1: glyph3x5_row = 3'b001;
                        3'd2: glyph3x5_row = 3'b111;
                        3'd3: glyph3x5_row = 3'b100;
                        3'd4: glyph3x5_row = 3'b111;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end
                8'h33: begin
                    case (row)
                        3'd0: glyph3x5_row = 3'b111;
                        3'd1: glyph3x5_row = 3'b001;
                        3'd2: glyph3x5_row = 3'b111;
                        3'd3: glyph3x5_row = 3'b001;
                        3'd4: glyph3x5_row = 3'b111;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end
                8'h34: begin
                    case (row)
                        3'd0: glyph3x5_row = 3'b101;
                        3'd1: glyph3x5_row = 3'b101;
                        3'd2: glyph3x5_row = 3'b111;
                        3'd3: glyph3x5_row = 3'b001;
                        3'd4: glyph3x5_row = 3'b001;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end
                8'h35: begin
                    case (row)
                        3'd0: glyph3x5_row = 3'b111;
                        3'd1: glyph3x5_row = 3'b100;
                        3'd2: glyph3x5_row = 3'b111;
                        3'd3: glyph3x5_row = 3'b001;
                        3'd4: glyph3x5_row = 3'b111;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end
                8'h36: begin
                    case (row)
                        3'd0: glyph3x5_row = 3'b111;
                        3'd1: glyph3x5_row = 3'b100;
                        3'd2: glyph3x5_row = 3'b111;
                        3'd3: glyph3x5_row = 3'b101;
                        3'd4: glyph3x5_row = 3'b111;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end
                8'h37: begin
                    case (row)
                        3'd0: glyph3x5_row = 3'b111;
                        3'd1: glyph3x5_row = 3'b001;
                        3'd2: glyph3x5_row = 3'b001;
                        3'd3: glyph3x5_row = 3'b001;
                        3'd4: glyph3x5_row = 3'b001;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end
                8'h38: begin
                    case (row)
                        3'd0: glyph3x5_row = 3'b111;
                        3'd1: glyph3x5_row = 3'b101;
                        3'd2: glyph3x5_row = 3'b111;
                        3'd3: glyph3x5_row = 3'b101;
                        3'd4: glyph3x5_row = 3'b111;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end
                8'h39: begin
                    case (row)
                        3'd0: glyph3x5_row = 3'b111;
                        3'd1: glyph3x5_row = 3'b101;
                        3'd2: glyph3x5_row = 3'b111;
                        3'd3: glyph3x5_row = 3'b001;
                        3'd4: glyph3x5_row = 3'b111;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end

                // Letters A-Z
                8'h41: begin
                    case (row)
                        3'd0: glyph3x5_row = 3'b111;
                        3'd1: glyph3x5_row = 3'b101;
                        3'd2: glyph3x5_row = 3'b111;
                        3'd3: glyph3x5_row = 3'b101;
                        3'd4: glyph3x5_row = 3'b101;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end
                8'h42: begin
                    case (row)
                        3'd0: glyph3x5_row = 3'b110;
                        3'd1: glyph3x5_row = 3'b101;
                        3'd2: glyph3x5_row = 3'b110;
                        3'd3: glyph3x5_row = 3'b101;
                        3'd4: glyph3x5_row = 3'b110;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end
                8'h43: begin
                    case (row)
                        3'd0: glyph3x5_row = 3'b111;
                        3'd1: glyph3x5_row = 3'b100;
                        3'd2: glyph3x5_row = 3'b100;
                        3'd3: glyph3x5_row = 3'b100;
                        3'd4: glyph3x5_row = 3'b111;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end
                8'h44: begin
                    case (row)
                        3'd0: glyph3x5_row = 3'b110;
                        3'd1: glyph3x5_row = 3'b101;
                        3'd2: glyph3x5_row = 3'b101;
                        3'd3: glyph3x5_row = 3'b101;
                        3'd4: glyph3x5_row = 3'b110;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end
                8'h45: begin
                    case (row)
                        3'd0: glyph3x5_row = 3'b111;
                        3'd1: glyph3x5_row = 3'b100;
                        3'd2: glyph3x5_row = 3'b110;
                        3'd3: glyph3x5_row = 3'b100;
                        3'd4: glyph3x5_row = 3'b111;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end
                8'h46: begin
                    case (row)
                        3'd0: glyph3x5_row = 3'b111;
                        3'd1: glyph3x5_row = 3'b100;
                        3'd2: glyph3x5_row = 3'b110;
                        3'd3: glyph3x5_row = 3'b100;
                        3'd4: glyph3x5_row = 3'b100;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end
                8'h47: begin
                    case (row)
                        3'd0: glyph3x5_row = 3'b111;
                        3'd1: glyph3x5_row = 3'b100;
                        3'd2: glyph3x5_row = 3'b101;
                        3'd3: glyph3x5_row = 3'b101;
                        3'd4: glyph3x5_row = 3'b111;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end
                8'h48: begin
                    case (row)
                        3'd0: glyph3x5_row = 3'b101;
                        3'd1: glyph3x5_row = 3'b101;
                        3'd2: glyph3x5_row = 3'b111;
                        3'd3: glyph3x5_row = 3'b101;
                        3'd4: glyph3x5_row = 3'b101;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end
                8'h49: begin
                    case (row)
                        3'd0: glyph3x5_row = 3'b111;
                        3'd1: glyph3x5_row = 3'b010;
                        3'd2: glyph3x5_row = 3'b010;
                        3'd3: glyph3x5_row = 3'b010;
                        3'd4: glyph3x5_row = 3'b111;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end
                8'h4A: begin
                    case (row)
                        3'd0: glyph3x5_row = 3'b001;
                        3'd1: glyph3x5_row = 3'b001;
                        3'd2: glyph3x5_row = 3'b001;
                        3'd3: glyph3x5_row = 3'b101;
                        3'd4: glyph3x5_row = 3'b111;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end
                8'h4B: begin
                    case (row)
                        3'd0: glyph3x5_row = 3'b101;
                        3'd1: glyph3x5_row = 3'b101;
                        3'd2: glyph3x5_row = 3'b110;
                        3'd3: glyph3x5_row = 3'b101;
                        3'd4: glyph3x5_row = 3'b101;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end
                8'h4C: begin
                    case (row)
                        3'd0: glyph3x5_row = 3'b100;
                        3'd1: glyph3x5_row = 3'b100;
                        3'd2: glyph3x5_row = 3'b100;
                        3'd3: glyph3x5_row = 3'b100;
                        3'd4: glyph3x5_row = 3'b111;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end
                8'h4D: begin
                    case (row)
                        3'd0: glyph3x5_row = 3'b101;
                        3'd1: glyph3x5_row = 3'b111;
                        3'd2: glyph3x5_row = 3'b111;
                        3'd3: glyph3x5_row = 3'b101;
                        3'd4: glyph3x5_row = 3'b101;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end
                8'h4E: begin
                    case (row)
                        3'd0: glyph3x5_row = 3'b101;
                        3'd1: glyph3x5_row = 3'b111;
                        3'd2: glyph3x5_row = 3'b111;
                        3'd3: glyph3x5_row = 3'b111;
                        3'd4: glyph3x5_row = 3'b101;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end
                8'h4F: begin
                    case (row)
                        3'd0: glyph3x5_row = 3'b111;
                        3'd1: glyph3x5_row = 3'b101;
                        3'd2: glyph3x5_row = 3'b101;
                        3'd3: glyph3x5_row = 3'b101;
                        3'd4: glyph3x5_row = 3'b111;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end
                8'h50: begin
                    case (row)
                        3'd0: glyph3x5_row = 3'b111;
                        3'd1: glyph3x5_row = 3'b101;
                        3'd2: glyph3x5_row = 3'b111;
                        3'd3: glyph3x5_row = 3'b100;
                        3'd4: glyph3x5_row = 3'b100;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end
                8'h51: begin
                    case (row)
                        3'd0: glyph3x5_row = 3'b111;
                        3'd1: glyph3x5_row = 3'b101;
                        3'd2: glyph3x5_row = 3'b101;
                        3'd3: glyph3x5_row = 3'b111;
                        3'd4: glyph3x5_row = 3'b001;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end
                8'h52: begin
                    case (row)
                        3'd0: glyph3x5_row = 3'b111;
                        3'd1: glyph3x5_row = 3'b101;
                        3'd2: glyph3x5_row = 3'b111;
                        3'd3: glyph3x5_row = 3'b110;
                        3'd4: glyph3x5_row = 3'b101;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end
                8'h53: begin
                    case (row)
                        3'd0: glyph3x5_row = 3'b111;
                        3'd1: glyph3x5_row = 3'b100;
                        3'd2: glyph3x5_row = 3'b111;
                        3'd3: glyph3x5_row = 3'b001;
                        3'd4: glyph3x5_row = 3'b111;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end
                8'h54: begin
                    case (row)
                        3'd0: glyph3x5_row = 3'b111;
                        3'd1: glyph3x5_row = 3'b010;
                        3'd2: glyph3x5_row = 3'b010;
                        3'd3: glyph3x5_row = 3'b010;
                        3'd4: glyph3x5_row = 3'b010;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end
                8'h55: begin
                    case (row)
                        3'd0: glyph3x5_row = 3'b101;
                        3'd1: glyph3x5_row = 3'b101;
                        3'd2: glyph3x5_row = 3'b101;
                        3'd3: glyph3x5_row = 3'b101;
                        3'd4: glyph3x5_row = 3'b111;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end
                8'h56: begin
                    case (row)
                        3'd0: glyph3x5_row = 3'b101;
                        3'd1: glyph3x5_row = 3'b101;
                        3'd2: glyph3x5_row = 3'b101;
                        3'd3: glyph3x5_row = 3'b101;
                        3'd4: glyph3x5_row = 3'b010;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end
                8'h57: begin
                    case (row)
                        3'd0: glyph3x5_row = 3'b101;
                        3'd1: glyph3x5_row = 3'b101;
                        3'd2: glyph3x5_row = 3'b111;
                        3'd3: glyph3x5_row = 3'b111;
                        3'd4: glyph3x5_row = 3'b101;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end
                8'h58: begin
                    case (row)
                        3'd0: glyph3x5_row = 3'b101;
                        3'd1: glyph3x5_row = 3'b101;
                        3'd2: glyph3x5_row = 3'b010;
                        3'd3: glyph3x5_row = 3'b101;
                        3'd4: glyph3x5_row = 3'b101;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end
                8'h59: begin
                    case (row)
                        3'd0: glyph3x5_row = 3'b101;
                        3'd1: glyph3x5_row = 3'b101;
                        3'd2: glyph3x5_row = 3'b010;
                        3'd3: glyph3x5_row = 3'b010;
                        3'd4: glyph3x5_row = 3'b010;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end
                8'h5A: begin
                    case (row)
                        3'd0: glyph3x5_row = 3'b111;
                        3'd1: glyph3x5_row = 3'b001;
                        3'd2: glyph3x5_row = 3'b010;
                        3'd3: glyph3x5_row = 3'b100;
                        3'd4: glyph3x5_row = 3'b111;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end

                default: begin
                    case (row)
                        3'd0: glyph3x5_row = 3'b111;
                        3'd1: glyph3x5_row = 3'b101;
                        3'd2: glyph3x5_row = 3'b101;
                        3'd3: glyph3x5_row = 3'b101;
                        3'd4: glyph3x5_row = 3'b111;
                        default: glyph3x5_row = 3'b000;
                    endcase
                end
            endcase
        end
    endfunction

    //--------------------------------------------------------------------------
    // Object selection
    //
    // Priority order:
    //   top health -> panel titles -> panel telemetry -> bottom debug
    //--------------------------------------------------------------------------
    always @(*) begin
        sel_valid      = 1'b0;
        sel_line_norm  = {384{1'b0}};
        sel_chars      = 6'd0;
        sel_rgb        = 24'h000000;
        sel_x0         = 10'd0;
        sel_y0         = 10'd0;
        sel_debug_text = 1'b0;

        if (active_video) begin
            // Top strip: exact 15-char rendering
            if ((hcount >= X_TOP_BMP) &&
                (hcount <  X_TOP_BMP + text_width_px(15, CHAR_ADV_X)) &&
                (vcount >= Y_TOP) &&
                (vcount <  Y_TOP + text_height_px(GLYPH_SCALE)) &&
                in_y_clip(vcount, CLIP_TOP_Y0, CLIP_TOP_Y1)) begin
                sel_valid     = 1'b1;
                sel_line_norm = norm15(top_bmp_line);
                sel_chars     = 6'd15;
                sel_rgb       = top_bmp_rgb;
                sel_x0        = X_TOP_BMP;
                sel_y0        = Y_TOP;
            end else if ((hcount >= X_TOP_ACC) &&
                         (hcount <  X_TOP_ACC + text_width_px(15, CHAR_ADV_X)) &&
                         (vcount >= Y_TOP) &&
                         (vcount <  Y_TOP + text_height_px(GLYPH_SCALE)) &&
                         in_y_clip(vcount, CLIP_TOP_Y0, CLIP_TOP_Y1)) begin
                sel_valid     = 1'b1;
                sel_line_norm = norm15(top_acc_line);
                sel_chars     = 6'd15;
                sel_rgb       = top_acc_rgb;
                sel_x0        = X_TOP_ACC;
                sel_y0        = Y_TOP;
            end else if ((hcount >= X_TOP_MAG) &&
                         (hcount <  X_TOP_MAG + text_width_px(15, CHAR_ADV_X)) &&
                         (vcount >= Y_TOP) &&
                         (vcount <  Y_TOP + text_height_px(GLYPH_SCALE)) &&
                         in_y_clip(vcount, CLIP_TOP_Y0, CLIP_TOP_Y1)) begin
                sel_valid     = 1'b1;
                sel_line_norm = norm15(top_mag_line);
                sel_chars     = 6'd15;
                sel_rgb       = top_mag_rgb;
                sel_x0        = X_TOP_MAG;
                sel_y0        = Y_TOP;
            end

            // Permanent panel titles.
            else if ((hcount >= X_LEFT) &&
                     (hcount <  X_LEFT + text_width_px(18, CHAR_ADV_X)) &&
                     (vcount >= Y_TITLE) &&
                     (vcount <  Y_TITLE + text_height_px(GLYPH_SCALE)) &&
                     in_y_clip(vcount, CLIP_PANEL_Y0, CLIP_PANEL_Y1)) begin
                sel_valid     = 1'b1;
                sel_line_norm = norm18(TITLE_LEFT);
                sel_chars     = 6'd18;
                sel_rgb       = RGB_TITLE;
                sel_x0        = X_LEFT;
                sel_y0        = Y_TITLE;
            end else if ((hcount >= X_MID) &&
                         (hcount <  X_MID + text_width_px(18, CHAR_ADV_X)) &&
                         (vcount >= Y_TITLE) &&
                         (vcount <  Y_TITLE + text_height_px(GLYPH_SCALE)) &&
                         in_y_clip(vcount, CLIP_PANEL_Y0, CLIP_PANEL_Y1)) begin
                sel_valid     = 1'b1;
                sel_line_norm = norm18(TITLE_MID);
                sel_chars     = 6'd18;
                sel_rgb       = RGB_TITLE;
                sel_x0        = X_MID;
                sel_y0        = Y_TITLE;
            end else if ((hcount >= X_RIGHT) &&
                         (hcount <  X_RIGHT + text_width_px(18, CHAR_ADV_X)) &&
                         (vcount >= Y_TITLE) &&
                         (vcount <  Y_TITLE + text_height_px(GLYPH_SCALE)) &&
                         in_y_clip(vcount, CLIP_PANEL_Y0, CLIP_PANEL_Y1)) begin
                sel_valid     = 1'b1;
                sel_line_norm = norm18(TITLE_RIGHT);
                sel_chars     = 6'd18;
                sel_rgb       = RGB_TITLE;
                sel_x0        = X_RIGHT;
                sel_y0        = Y_TITLE;
            end

            // Left panel: exact 18-char rendering
            else if ((hcount >= X_LEFT) &&
                     (hcount <  X_LEFT + text_width_px(18, CHAR_ADV_X)) &&
                     (vcount >= Y_ROW0) &&
                     (vcount <  Y_ROW0 + text_height_px(GLYPH_SCALE)) &&
                     in_y_clip(vcount, CLIP_PANEL_Y0, CLIP_PANEL_Y1)) begin
                sel_valid     = 1'b1;
                sel_line_norm = norm18(left_line0);
                sel_chars     = 6'd18;
                sel_rgb       = left_rgb;
                sel_x0        = X_LEFT;
                sel_y0        = Y_ROW0;
            end else if ((hcount >= X_LEFT) &&
                         (hcount <  X_LEFT + text_width_px(18, CHAR_ADV_X)) &&
                         (vcount >= Y_ROW1) &&
                         (vcount <  Y_ROW1 + text_height_px(GLYPH_SCALE)) &&
                         in_y_clip(vcount, CLIP_PANEL_Y0, CLIP_PANEL_Y1)) begin
                sel_valid     = 1'b1;
                sel_line_norm = norm18(left_line1);
                sel_chars     = 6'd18;
                sel_rgb       = left_rgb;
                sel_x0        = X_LEFT;
                sel_y0        = Y_ROW1;
            end else if ((hcount >= X_LEFT) &&
                         (hcount <  X_LEFT + text_width_px(18, CHAR_ADV_X)) &&
                         (vcount >= Y_ROW2) &&
                         (vcount <  Y_ROW2 + text_height_px(GLYPH_SCALE)) &&
                         in_y_clip(vcount, CLIP_PANEL_Y0, CLIP_PANEL_Y1)) begin
                sel_valid     = 1'b1;
                sel_line_norm = norm18(left_line2);
                sel_chars     = 6'd18;
                sel_rgb       = left_rgb;
                sel_x0        = X_LEFT;
                sel_y0        = Y_ROW2;
            end else if ((hcount >= X_LEFT) &&
                         (hcount <  X_LEFT + text_width_px(18, CHAR_ADV_X)) &&
                         (vcount >= Y_ROW3) &&
                         (vcount <  Y_ROW3 + text_height_px(GLYPH_SCALE)) &&
                         in_y_clip(vcount, CLIP_PANEL_Y0, CLIP_PANEL_Y1)) begin
                sel_valid     = 1'b1;
                sel_line_norm = norm18(left_line3);
                sel_chars     = 6'd18;
                sel_rgb       = left_rgb;
                sel_x0        = X_LEFT;
                sel_y0        = Y_ROW3;
            end else if ((hcount >= X_LEFT) &&
                         (hcount <  X_LEFT + text_width_px(18, CHAR_ADV_X)) &&
                         (vcount >= Y_ROW4) &&
                         (vcount <  Y_ROW4 + text_height_px(GLYPH_SCALE)) &&
                         in_y_clip(vcount, CLIP_PANEL_Y0, CLIP_PANEL_Y1)) begin
                sel_valid     = 1'b1;
                sel_line_norm = norm18(left_line4);
                sel_chars     = 6'd18;
                sel_rgb       = left_rgb;
                sel_x0        = X_LEFT;
                sel_y0        = Y_ROW4;
            end

            // Middle panel: exact 18-char rendering
            else if ((hcount >= X_MID) &&
                     (hcount <  X_MID + text_width_px(18, CHAR_ADV_X)) &&
                     (vcount >= Y_ROW0) &&
                     (vcount <  Y_ROW0 + text_height_px(GLYPH_SCALE)) &&
                     in_y_clip(vcount, CLIP_PANEL_Y0, CLIP_PANEL_Y1)) begin
                sel_valid     = 1'b1;
                sel_line_norm = norm18(mid_line0);
                sel_chars     = 6'd18;
                sel_rgb       = mid_rgb;
                sel_x0        = X_MID;
                sel_y0        = Y_ROW0;
            end else if ((hcount >= X_MID) &&
                         (hcount <  X_MID + text_width_px(18, CHAR_ADV_X)) &&
                         (vcount >= Y_ROW1) &&
                         (vcount <  Y_ROW1 + text_height_px(GLYPH_SCALE)) &&
                         in_y_clip(vcount, CLIP_PANEL_Y0, CLIP_PANEL_Y1)) begin
                sel_valid     = 1'b1;
                sel_line_norm = norm18(mid_line1);
                sel_chars     = 6'd18;
                sel_rgb       = mid_rgb;
                sel_x0        = X_MID;
                sel_y0        = Y_ROW1;
            end else if ((hcount >= X_MID) &&
                         (hcount <  X_MID + text_width_px(18, CHAR_ADV_X)) &&
                         (vcount >= Y_ROW2) &&
                         (vcount <  Y_ROW2 + text_height_px(GLYPH_SCALE)) &&
                         in_y_clip(vcount, CLIP_PANEL_Y0, CLIP_PANEL_Y1)) begin
                sel_valid     = 1'b1;
                sel_line_norm = norm18(mid_line2);
                sel_chars     = 6'd18;
                sel_rgb       = mid_rgb;
                sel_x0        = X_MID;
                sel_y0        = Y_ROW2;
            end else if ((hcount >= X_MID) &&
                         (hcount <  X_MID + text_width_px(18, CHAR_ADV_X)) &&
                         (vcount >= Y_ROW3) &&
                         (vcount <  Y_ROW3 + text_height_px(GLYPH_SCALE)) &&
                         in_y_clip(vcount, CLIP_PANEL_Y0, CLIP_PANEL_Y1)) begin
                sel_valid     = 1'b1;
                sel_line_norm = norm18(mid_line3);
                sel_chars     = 6'd18;
                sel_rgb       = mid_rgb;
                sel_x0        = X_MID;
                sel_y0        = Y_ROW3;
            end else if ((hcount >= X_MID) &&
                         (hcount <  X_MID + text_width_px(18, CHAR_ADV_X)) &&
                         (vcount >= Y_ROW4) &&
                         (vcount <  Y_ROW4 + text_height_px(GLYPH_SCALE)) &&
                         in_y_clip(vcount, CLIP_PANEL_Y0, CLIP_PANEL_Y1)) begin
                sel_valid     = 1'b1;
                sel_line_norm = norm18(mid_line4);
                sel_chars     = 6'd18;
                sel_rgb       = mid_rgb;
                sel_x0        = X_MID;
                sel_y0        = Y_ROW4;
            end

            // Right panel: exact 18-char rendering
            else if ((hcount >= X_RIGHT) &&
                     (hcount <  X_RIGHT + text_width_px(18, CHAR_ADV_X)) &&
                     (vcount >= Y_ROW0) &&
                     (vcount <  Y_ROW0 + text_height_px(GLYPH_SCALE)) &&
                     in_y_clip(vcount, CLIP_PANEL_Y0, CLIP_PANEL_Y1)) begin
                sel_valid     = 1'b1;
                sel_line_norm = norm18(right_line0);
                sel_chars     = 6'd18;
                sel_rgb       = right_rgb;
                sel_x0        = X_RIGHT;
                sel_y0        = Y_ROW0;
            end else if ((hcount >= X_RIGHT) &&
                         (hcount <  X_RIGHT + text_width_px(18, CHAR_ADV_X)) &&
                         (vcount >= Y_ROW1) &&
                         (vcount <  Y_ROW1 + text_height_px(GLYPH_SCALE)) &&
                         in_y_clip(vcount, CLIP_PANEL_Y0, CLIP_PANEL_Y1)) begin
                sel_valid     = 1'b1;
                sel_line_norm = norm18(right_line1);
                sel_chars     = 6'd18;
                sel_rgb       = right_rgb;
                sel_x0        = X_RIGHT;
                sel_y0        = Y_ROW1;
            end else if ((hcount >= X_RIGHT) &&
                         (hcount <  X_RIGHT + text_width_px(18, CHAR_ADV_X)) &&
                         (vcount >= Y_ROW2) &&
                         (vcount <  Y_ROW2 + text_height_px(GLYPH_SCALE)) &&
                         in_y_clip(vcount, CLIP_PANEL_Y0, CLIP_PANEL_Y1)) begin
                sel_valid     = 1'b1;
                sel_line_norm = norm18(right_line2);
                sel_chars     = 6'd18;
                sel_rgb       = right_rgb;
                sel_x0        = X_RIGHT;
                sel_y0        = Y_ROW2;
            end else if ((hcount >= X_RIGHT) &&
                         (hcount <  X_RIGHT + text_width_px(18, CHAR_ADV_X)) &&
                         (vcount >= Y_ROW3) &&
                         (vcount <  Y_ROW3 + text_height_px(GLYPH_SCALE)) &&
                         in_y_clip(vcount, CLIP_PANEL_Y0, CLIP_PANEL_Y1)) begin
                sel_valid     = 1'b1;
                sel_line_norm = norm18(right_line3);
                sel_chars     = 6'd18;
                sel_rgb       = right_rgb;
                sel_x0        = X_RIGHT;
                sel_y0        = Y_ROW3;
            end else if ((hcount >= X_RIGHT) &&
                         (hcount <  X_RIGHT + text_width_px(18, CHAR_ADV_X)) &&
                         (vcount >= Y_ROW4) &&
                         (vcount <  Y_ROW4 + text_height_px(GLYPH_SCALE)) &&
                         in_y_clip(vcount, CLIP_PANEL_Y0, CLIP_PANEL_Y1)) begin
                sel_valid     = 1'b1;
                sel_line_norm = norm18(right_line4);
                sel_chars     = 6'd18;
                sel_rgb       = right_rgb;
                sel_x0        = X_RIGHT;
                sel_y0        = Y_ROW4;
            end

            // Bottom strip: exact 48-char rendering
            else if ((hcount >= X_BOTTOM) &&
                     (hcount <  X_BOTTOM + text_width_px(48, DEBUG_ADV_X)) &&
                     (vcount >= Y_BOTTOM) &&
                     (vcount <  Y_BOTTOM + text_height_px(DEBUG_SCALE)) &&
                     in_y_clip(vcount, CLIP_BOTTOM_Y0, CLIP_BOTTOM_Y1)) begin
                sel_valid      = 1'b1;
                sel_line_norm  = bottom_line;
                sel_chars      = 6'd48;
                sel_rgb        = bottom_rgb;
                sel_x0         = X_BOTTOM;
                sel_y0         = Y_BOTTOM;
                sel_debug_text = 1'b1;
            end
        end
    end

    //--------------------------------------------------------------------------
    // Shared glyph renderer
    //--------------------------------------------------------------------------
    always @(*) begin
        overlay_on     = 1'b0;
        overlay_rgb    = 24'h000000;

        local_x        = 10'd0;
        local_y        = 10'd0;

        char_idx_int   = 0;
        glyph_row_int  = 0;
        glyph_col_int  = 0;

        glyph_char     = 8'h20;
        glyph_row_bits = 3'b000;
        glyph_pixel_on = 1'b0;

        if (sel_valid && active_video) begin
            local_x = hcount - sel_x0;
            local_y = vcount - sel_y0;

            if (sel_debug_text) begin
                char_idx_int  = local_x >> 2;       // DEBUG_ADV_X = 4
                glyph_row_int = local_y;            // DEBUG_SCALE = 1
                glyph_col_int = local_x[1:0];
            end else begin
                char_idx_int  = local_x / CHAR_ADV_X;
                glyph_row_int = local_y / GLYPH_SCALE;
                glyph_col_int = (local_x % CHAR_ADV_X) / GLYPH_SCALE;
            end

            if ((char_idx_int >= 0) &&
                (char_idx_int < sel_chars) &&
                (glyph_row_int >= 0) &&
                (glyph_row_int < GLYPH_H) &&
                (glyph_col_int >= 0) &&
                (glyph_col_int < GLYPH_W)) begin

                glyph_char     = norm_char_at(sel_line_norm, char_idx_int[5:0]);
                glyph_row_bits = glyph3x5_row(glyph_char, glyph_row_int[2:0]);

                case (glyph_col_int[1:0])
                    2'd0: glyph_pixel_on = glyph_row_bits[2];
                    2'd1: glyph_pixel_on = glyph_row_bits[1];
                    default: glyph_pixel_on = glyph_row_bits[0];
                endcase

                if (glyph_pixel_on) begin
                    overlay_on  = 1'b1;
                    overlay_rgb = sel_rgb;
                end
            end
        end
    end

endmodule

`default_nettype wire
