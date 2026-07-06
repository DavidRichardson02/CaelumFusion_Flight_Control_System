`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// vga_char_glyph_3x5
//------------------------------------------------------------------------------
// Draw one 3x5 character at (x0,y0) in the pixel domain.
// Telemetry-safe glyph subset:
//   - A..Z
//   - 0..9
//   - space
//   - = / + - :
//==============================================================================

module vga_char_glyph_3x5 #(
    parameter integer REGISTER_OUTPUT = 0,
    parameter integer ASCII_MIN       = 7'h20,
    parameter integer ASCII_MAX       = 7'h5F,
    parameter integer EN_RANGE_GATING = 1,
    parameter integer MAP_LOWERCASE   = 1,
    parameter [14:0]  FALLBACK_GLYPH  = 15'b000_000_000_000_000
)(
    input  wire       clk_pix,
    input  wire       rst_pix,

    input  wire [9:0] hcount,
    input  wire [9:0] vcount,
    input  wire       active_video,

    input  wire [9:0] x0,
    input  wire [9:0] y0,
    input  wire [7:0] char_code,
    input  wire [3:0] scale,

    output wire       pixel_on
);

    localparam integer GLYPH_W = 3;
    localparam integer GLYPH_H = 5;

    wire [3:0] sc = (scale == 4'd0) ? 4'd1 : scale;

    wire [7:0] c_norm =
        (MAP_LOWERCASE != 0 && (char_code >= 8'h61) && (char_code <= 8'h7A))
            ? (char_code - 8'h20)
            : char_code;

    wire in_ascii_range =
        (c_norm[6:0] >= ASCII_MIN[6:0]) &&
        (c_norm[6:0] <= ASCII_MAX[6:0]);

    wire [11:0] x0x = {2'd0, x0};
    wire [11:0] y0x = {2'd0, y0};
    wire [11:0] hx  = {2'd0, hcount};
    wire [11:0] vx  = {2'd0, vcount};

    wire [11:0] w_scaled = 12'd3 * {8'd0, sc};
    wire [11:0] h_scaled = 12'd5 * {8'd0, sc};

    wire in_box =
        active_video &&
        (hx >= x0x) && (hx < (x0x + w_scaled)) &&
        (vx >= y0x) && (vx < (y0x + h_scaled));

    wire [11:0] rel_x = hx - x0x;
    wire [11:0] rel_y = vx - y0x;

    reg [1:0]  col;
    reg [2:0]  row;
    reg [11:0] tx;
    reg [11:0] ty;
    integer k;

    always @* begin
        col = 2'd0;
        tx  = rel_x;
        for (k = 0; k < GLYPH_W; k = k + 1) begin
            if ((col != 2'd2) && (tx >= {8'd0, sc})) begin
                tx  = tx - {8'd0, sc};
                col = col + 2'd1;
            end
        end

        row = 3'd0;
        ty  = rel_y;
        for (k = 0; k < GLYPH_H; k = k + 1) begin
            if ((row != 3'd4) && (ty >= {8'd0, sc})) begin
                ty  = ty - {8'd0, sc};
                row = row + 3'd1;
            end
        end
    end

    function [14:0] glyph3x5;
        input [7:0] c;
        begin
            case (c)
                //--------------------------------------------------------------------------
                // Space / punctuation
                //--------------------------------------------------------------------------
                8'h20: glyph3x5 = 15'b000_000_000_000_000; // space
                8'h2B: glyph3x5 = 15'b000_010_111_010_000; // +
                8'h2D: glyph3x5 = 15'b000_000_111_000_000; // -
                8'h2F: glyph3x5 = 15'b001_001_010_100_100; // /
                8'h3A: glyph3x5 = 15'b000_010_000_010_000; // :
                8'h3D: glyph3x5 = 15'b000_111_000_111_000; // =

                //--------------------------------------------------------------------------
                // Digits
                //--------------------------------------------------------------------------
                8'h30: glyph3x5 = 15'b111_101_101_101_111; // 0
                8'h31: glyph3x5 = 15'b010_110_010_010_111; // 1
                8'h32: glyph3x5 = 15'b111_001_111_100_111; // 2
                8'h33: glyph3x5 = 15'b111_001_111_001_111; // 3
                8'h34: glyph3x5 = 15'b101_101_111_001_001; // 4
                8'h35: glyph3x5 = 15'b111_100_111_001_111; // 5
                8'h36: glyph3x5 = 15'b111_100_111_101_111; // 6
                8'h37: glyph3x5 = 15'b111_001_001_001_001; // 7
                8'h38: glyph3x5 = 15'b111_101_111_101_111; // 8
                8'h39: glyph3x5 = 15'b111_101_111_001_111; // 9

                //--------------------------------------------------------------------------
                // Uppercase letters
                //--------------------------------------------------------------------------
                8'h41: glyph3x5 = 15'b111_101_111_101_101; // A
                8'h42: glyph3x5 = 15'b110_101_110_101_110; // B
                8'h43: glyph3x5 = 15'b111_100_100_100_111; // C
                8'h44: glyph3x5 = 15'b110_101_101_101_110; // D
                8'h45: glyph3x5 = 15'b111_100_111_100_111; // E
                8'h46: glyph3x5 = 15'b111_100_111_100_100; // F
                8'h47: glyph3x5 = 15'b111_100_101_101_111; // G
                8'h48: glyph3x5 = 15'b101_101_111_101_101; // H
                8'h49: glyph3x5 = 15'b111_010_010_010_111; // I
                8'h4A: glyph3x5 = 15'b001_001_001_101_111; // J
                8'h4B: glyph3x5 = 15'b101_101_110_101_101; // K
                8'h4C: glyph3x5 = 15'b100_100_100_100_111; // L
                8'h4D: glyph3x5 = 15'b101_111_111_101_101; // M
                8'h4E: glyph3x5 = 15'b101_111_111_111_101; // N
                8'h4F: glyph3x5 = 15'b111_101_101_101_111; // O
                8'h50: glyph3x5 = 15'b111_101_111_100_100; // P
                8'h51: glyph3x5 = 15'b111_101_101_111_001; // Q
                8'h52: glyph3x5 = 15'b111_101_111_110_101; // R
                8'h53: glyph3x5 = 15'b111_100_111_001_111; // S
                8'h54: glyph3x5 = 15'b111_010_010_010_010; // T
                8'h55: glyph3x5 = 15'b101_101_101_101_111; // U
                8'h56: glyph3x5 = 15'b101_101_101_101_010; // V
                8'h57: glyph3x5 = 15'b101_101_111_111_101; // W
                8'h58: glyph3x5 = 15'b101_101_010_101_101; // X
                8'h59: glyph3x5 = 15'b101_101_010_010_010; // Y
                8'h5A: glyph3x5 = 15'b111_001_010_100_111; // Z

                default: glyph3x5 = FALLBACK_GLYPH;
            endcase
        end
    endfunction

    wire [14:0] g =
        ((EN_RANGE_GATING != 0) && !in_ascii_range) ? FALLBACK_GLYPH : glyph3x5(c_norm);

    reg [2:0] row_bits;
    always @* begin
        case (row)
            3'd0: row_bits = g[14:12];
            3'd1: row_bits = g[11:9];
            3'd2: row_bits = g[8:6];
            3'd3: row_bits = g[5:3];
            default: row_bits = g[2:0];
        endcase
    end

    wire font_bit = row_bits[2 - col];

    reg pixel_r;
    always @(posedge clk_pix) begin
        if (rst_pix) begin
            pixel_r <= 1'b0;
        end else if (REGISTER_OUTPUT != 0) begin
            pixel_r <= in_box && font_bit;
        end
    end

    assign pixel_on = (REGISTER_OUTPUT != 0) ? pixel_r : (in_box && font_bit);

endmodule

`default_nettype wire