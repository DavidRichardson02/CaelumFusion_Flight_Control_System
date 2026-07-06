`timescale 1ns/1ps
`default_nettype none
`include "telemetry_defs_vh.vh"

//==============================================================================
// planar_compass_truth_page_vga
//------------------------------------------------------------------------------
// Full-screen VGA diagnostic overlay for the CMPS2/MMC34160PJ planar compass
// truth page. This module is intentionally additive: it does not rename or
// replace existing visualizer modules. When page_active is false it passes the
// existing VGA stream through unchanged.
//==============================================================================
module planar_compass_truth_page_vga #(
    parameter integer SYS_CLK_HZ = 100_000_000,
    parameter integer UI_UPDATE_HZ = 1000,
    parameter integer H_ACTIVE = 640,
    parameter integer H_FP     = 16,
    parameter integer H_SYNC   = 96,
    parameter integer H_BP     = 48,
    parameter integer V_ACTIVE = 480,
    parameter integer V_FP     = 10,
    parameter integer V_SYNC   = 2,
    parameter integer V_BP     = 33,
    parameter integer MAG_PLOT_SHIFT = 8,
    parameter integer COMPASS_TRUTH_PAGE_DEFAULT = 0
)(
    input  wire        sys_clk,
    input  wire        sys_rst,
    input  wire        page_enable_sys,

    input  wire [15:0] mag_seq,
    input  wire        mag_valid,
    input  wire [7:0]  mag_status,
    input  wire [47:0] mag_payload,
    input  wire [15:0] mag_age_ms,
    input  wire [15:0] mag1_seq,
    input  wire        mag1_valid,
    input  wire [7:0]  mag1_status,
    input  wire [47:0] mag1_payload,
    input  wire [15:0] mag1_age_ms,

    input  wire        der_valid,
    input  wire [7:0]  der_status,
    input  wire        der_head_fresh,
    input  wire [15:0] der_mag_seq_ref,
    input  wire [31:0] der_heading_mdeg,

    input  wire        ext_valid,
    input  wire [7:0]  ext_status,
    input  wire [15:0] ext_present_flags,
    input  wire [15:0] ext_fault_flags,
    input  wire [15:0] ext_mag_delta_l1,
    input  wire [15:0] ext_mag_norm_primary,
    input  wire [15:0] ext_mag_norm_secondary,
    input  wire        ext_mag_sequence_aligned,
    input  wire        ext_mag_disagreement,
    input  wire [3:0]  ext_mag_sector_delta,
    input  wire [15:0] ext_mag_norm_delta_l1,
    input  wire [15:0] ext_mag_iron_residual,
    input  wire [7:0]  ext_mag_cal_state,
    input  wire [7:0]  ext_mag_source_flags,
    input  wire [15:0] ext_mag_bridge_checksum,
    input  wire [15:0] ext_max_age_ms,

    input  wire [15:0] i2c_nack_count,
    input  wire [15:0] i2c_timeout_count,
    input  wire [15:0] txn_rate_hz,

    input  wire        pix_clk,
    input  wire        pix_rst,
    input  wire        vga_hsync_in,
    input  wire        vga_vsync_in,
    input  wire [11:0] vga_rgb_in,
    output wire        vga_hsync_out,
    output wire        vga_vsync_out,
    output wire [11:0] vga_rgb_out
);

    localparam integer H_TOTAL = H_ACTIVE + H_FP + H_SYNC + H_BP;
    localparam integer V_TOTAL = V_ACTIVE + V_FP + V_SYNC + V_BP;
    localparam integer UI_DIV_MAX = (SYS_CLK_HZ / UI_UPDATE_HZ) - 1;
    localparam [31:0] UI_DIV_MAX_U32 = UI_DIV_MAX;
    localparam [11:0] H_TOTAL_U12 = H_TOTAL;
    localparam [11:0] V_TOTAL_U12 = V_TOTAL;
    localparam [11:0] H_ACTIVE_U12 = H_ACTIVE;
    localparam [11:0] V_ACTIVE_U12 = V_ACTIVE;
    localparam integer BASE_BUNDLE_W = 196;
    localparam integer EXT_BUNDLE_W  = 105;
    localparam integer MAG1_BUNDLE_W = 89;
    localparam integer MAG_EVIDENCE_BUNDLE_W = 70;
    localparam integer BUNDLE_W      = BASE_BUNDLE_W + EXT_BUNDLE_W +
                                       MAG1_BUNDLE_W + MAG_EVIDENCE_BUNDLE_W;

    localparam [11:0] C_BLACK  = 12'h000;
    localparam [11:0] C_GRID   = 12'h022;
    localparam [11:0] C_DIM    = 12'h044;
    localparam [11:0] C_CYAN   = 12'h0FF;
    localparam [11:0] C_GREEN  = 12'h0F4;
    localparam [11:0] C_YELLOW = 12'hFF0;
    localparam [11:0] C_ORANGE = 12'hF80;
    localparam [11:0] C_RED    = 12'hF22;
    localparam [11:0] C_WHITE  = 12'hFFF;
    localparam [11:0] C_BLUE   = 12'h08F;
    localparam [11:0] C_MAG    = 12'hF0F;

    //--------------------------------------------------------------------------
    // SYS -> PIX semantic bundle CDC. The held data bus is sampled only after
    // the synchronized toggle reaches the pixel domain. XDC contains a scoped
    // false path for this snapshot bus, matching the existing visualization
    // bundle CDC style.
    //--------------------------------------------------------------------------
    reg [31:0] ui_div_r;
    reg [BUNDLE_W-1:0] src_bundle_hold;
    reg src_toggle;
    wire src_sample_tick;
    assign src_sample_tick = (ui_div_r == UI_DIV_MAX_U32);

    always @(posedge sys_clk) begin
        if (sys_rst) begin
            ui_div_r <= 32'd0;
            src_bundle_hold <= {BUNDLE_W{1'b0}};
            src_toggle <= 1'b0;
        end else begin
            if (src_sample_tick)
                ui_div_r <= 32'd0;
            else
                ui_div_r <= ui_div_r + 32'd1;

            if (src_sample_tick) begin
                src_bundle_hold <= {
                    mag1_seq,
                    mag1_valid,
                    mag1_status,
                    mag1_payload,
                    mag1_age_ms,
                    ext_mag_sequence_aligned,
                    ext_mag_disagreement,
                    ext_mag_sector_delta,
                    ext_mag_norm_delta_l1,
                    ext_mag_iron_residual,
                    ext_mag_cal_state,
                    ext_mag_source_flags,
                    ext_mag_bridge_checksum,
                    ext_valid,
                    ext_status,
                    ext_present_flags,
                    ext_fault_flags,
                    ext_mag_delta_l1,
                    ext_mag_norm_primary,
                    ext_mag_norm_secondary,
                    ext_max_age_ms,
                    page_enable_sys,
                    mag_seq,
                    mag_valid,
                    mag_status,
                    mag_payload,
                    mag_age_ms,
                    der_valid,
                    der_status,
                    der_head_fresh,
                    der_mag_seq_ref,
                    der_heading_mdeg,
                    i2c_nack_count,
                    i2c_timeout_count,
                    txn_rate_hz
                };
                src_toggle <= ~src_toggle;
            end
        end
    end

    reg [2:0] dst_toggle_sync_ff;
    reg       dst_toggle_seen;
    reg [BUNDLE_W-1:0] dst_bundle_shadow;
    wire dst_update_pulse;
    assign dst_update_pulse = dst_toggle_sync_ff[2] ^ dst_toggle_seen;

    always @(posedge pix_clk) begin
        if (pix_rst) begin
            dst_toggle_sync_ff <= 3'b000;
            dst_toggle_seen <= 1'b0;
            dst_bundle_shadow <= {BUNDLE_W{1'b0}};
        end else begin
            dst_toggle_sync_ff <= {dst_toggle_sync_ff[1:0], src_toggle};
            if (dst_update_pulse) begin
                dst_toggle_seen <= dst_toggle_sync_ff[2];
                dst_bundle_shadow <= src_bundle_hold;
            end
        end
    end

    wire        page_enable_pix      = dst_bundle_shadow[195];
    wire [15:0] mag_seq_pix          = dst_bundle_shadow[194:179];
    wire        mag_valid_pix        = dst_bundle_shadow[178];
    wire [7:0]  mag_status_pix       = dst_bundle_shadow[177:170];
    wire [47:0] mag_payload_pix      = dst_bundle_shadow[169:122];
    wire [15:0] mag_age_ms_pix       = dst_bundle_shadow[121:106];
    wire        der_valid_pix        = dst_bundle_shadow[105];
    wire [7:0]  der_status_pix       = dst_bundle_shadow[104:97];
    wire        der_head_fresh_pix   = dst_bundle_shadow[96];
    wire [15:0] der_mag_seq_ref_pix  = dst_bundle_shadow[95:80];
    wire [31:0] der_heading_mdeg_pix = dst_bundle_shadow[79:48];
    wire [15:0] i2c_nack_count_pix   = dst_bundle_shadow[47:32];
    wire [15:0] i2c_timeout_count_pix= dst_bundle_shadow[31:16];
    wire [15:0] txn_rate_hz_pix      = dst_bundle_shadow[15:0];

    wire        ext_valid_pix             = dst_bundle_shadow[300];
    wire [7:0]  ext_status_pix            = dst_bundle_shadow[299:292];
    wire [15:0] ext_present_flags_pix     = dst_bundle_shadow[291:276];
    wire [15:0] ext_fault_flags_pix       = dst_bundle_shadow[275:260];
    wire [15:0] ext_mag_delta_l1_pix      = dst_bundle_shadow[259:244];
    wire [15:0] ext_mag_norm_primary_pix  = dst_bundle_shadow[243:228];
    wire [15:0] ext_mag_norm_secondary_pix= dst_bundle_shadow[227:212];
    wire [15:0] ext_max_age_ms_pix        = dst_bundle_shadow[211:196];

    wire [15:0] mag1_seq_pix              = dst_bundle_shadow[459:444];
    wire        mag1_valid_pix            = dst_bundle_shadow[443];
    wire [7:0]  mag1_status_pix           = dst_bundle_shadow[442:435];
    wire [47:0] mag1_payload_pix          = dst_bundle_shadow[434:387];
    wire [15:0] mag1_age_ms_pix           = dst_bundle_shadow[386:371];
    wire        ext_mag_sequence_aligned_pix = dst_bundle_shadow[370];
    wire        ext_mag_disagreement_pix  = dst_bundle_shadow[369];
    wire [3:0]  ext_mag_sector_delta_pix  = dst_bundle_shadow[368:365];
    wire [15:0] ext_mag_norm_delta_l1_pix = dst_bundle_shadow[364:349];
    wire [15:0] ext_mag_iron_residual_pix = dst_bundle_shadow[348:333];
    wire [7:0]  ext_mag_cal_state_pix     = dst_bundle_shadow[332:325];
    wire [7:0]  ext_mag_source_flags_pix  = dst_bundle_shadow[324:317];
    wire [15:0] ext_mag_bridge_checksum_pix = dst_bundle_shadow[316:301];

    wire page_active;
    assign page_active = (COMPASS_TRUTH_PAGE_DEFAULT != 0) ? 1'b1 : page_enable_pix;

    //--------------------------------------------------------------------------
    // Pixel coordinate generator. It mirrors the active 640x480 timing so the
    // overlay can pass sync through while drawing a full replacement page.
    //--------------------------------------------------------------------------
    reg [11:0] pix_x;
    reg [11:0] pix_y;
    always @(posedge pix_clk) begin
        if (pix_rst) begin
            pix_x <= 12'd0;
            pix_y <= 12'd0;
        end else begin
            if (pix_x == H_TOTAL_U12 - 12'd1) begin
                pix_x <= 12'd0;
                if (pix_y == V_TOTAL_U12 - 12'd1)
                    pix_y <= 12'd0;
                else
                    pix_y <= pix_y + 12'd1;
            end else begin
                pix_x <= pix_x + 12'd1;
            end
        end
    end

    wire active_video;
    assign active_video = (pix_x < H_ACTIVE_U12) && (pix_y < V_ACTIVE_U12);

    // MAG_PAYLOAD_ZYX convention: {MZ[15:0], MY[15:0], MX[15:0]}.
    wire signed [15:0] mx_pix = mag_payload_pix[15:0];
    wire signed [15:0] my_pix = mag_payload_pix[31:16];
    wire signed [15:0] mz_pix = mag_payload_pix[47:32];
    wire signed [15:0] m1x_pix = mag1_payload_pix[15:0];
    wire signed [15:0] m1y_pix = mag1_payload_pix[31:16];
    wire signed [15:0] m1z_pix = mag1_payload_pix[47:32];

    function [15:0] abs16;
        input signed [15:0] v;
        begin
            if (v < 0)
                abs16 = (~v) + 16'd1;
            else
                abs16 = v[15:0];
        end
    endfunction

    function [15:0] max16;
        input [15:0] a;
        input [15:0] b;
        begin
            max16 = (a > b) ? a : b;
        end
    endfunction

    function signed [8:0] scale_axis;
        input signed [15:0] v;
        reg signed [15:0] s;
        begin
            s = v >>> MAG_PLOT_SHIFT;
            if (s > 16'sd82)
                scale_axis = 9'sd82;
            else if (s < -16'sd82)
                scale_axis = -9'sd82;
            else
                scale_axis = s[8:0];
        end
    endfunction

    function [7:0] hex_char;
        input [3:0] n;
        begin
            case (n)
                4'h0: hex_char = 8'd48;
                4'h1: hex_char = 8'd49;
                4'h2: hex_char = 8'd50;
                4'h3: hex_char = 8'd51;
                4'h4: hex_char = 8'd52;
                4'h5: hex_char = 8'd53;
                4'h6: hex_char = 8'd54;
                4'h7: hex_char = 8'd55;
                4'h8: hex_char = 8'd56;
                4'h9: hex_char = 8'd57;
                4'hA: hex_char = 8'd65;
                4'hB: hex_char = 8'd66;
                4'hC: hex_char = 8'd67;
                4'hD: hex_char = 8'd68;
                4'hE: hex_char = 8'd69;
                4'hF: hex_char = 8'd70;
                default: hex_char = 8'd32;
            endcase
        end
    endfunction

    function [4:0] glyph5x7;
        input [7:0] ch;
        input [2:0] row;
        begin
            glyph5x7 = 5'b00000;
            case (ch)
                8'd32: begin
                    case (row)
                        3'd0: glyph5x7 = 5'b00000;
                        3'd1: glyph5x7 = 5'b00000;
                        3'd2: glyph5x7 = 5'b00000;
                        3'd3: glyph5x7 = 5'b00000;
                        3'd4: glyph5x7 = 5'b00000;
                        3'd5: glyph5x7 = 5'b00000;
                        3'd6: glyph5x7 = 5'b00000;
                        default: glyph5x7 = 5'b00000;
                    endcase
                end
                8'd37: begin
                    case (row)
                        3'd0: glyph5x7 = 5'b11000;
                        3'd1: glyph5x7 = 5'b11001;
                        3'd2: glyph5x7 = 5'b00010;
                        3'd3: glyph5x7 = 5'b00100;
                        3'd4: glyph5x7 = 5'b01000;
                        3'd5: glyph5x7 = 5'b10011;
                        3'd6: glyph5x7 = 5'b00011;
                        default: glyph5x7 = 5'b00000;
                    endcase
                end
                8'd39: begin
                    case (row)
                        3'd0: glyph5x7 = 5'b00100;
                        3'd1: glyph5x7 = 5'b00100;
                        3'd2: glyph5x7 = 5'b01000;
                        3'd3: glyph5x7 = 5'b00000;
                        3'd4: glyph5x7 = 5'b00000;
                        3'd5: glyph5x7 = 5'b00000;
                        3'd6: glyph5x7 = 5'b00000;
                        default: glyph5x7 = 5'b00000;
                    endcase
                end
                8'd40: begin
                    case (row)
                        3'd0: glyph5x7 = 5'b00010;
                        3'd1: glyph5x7 = 5'b00100;
                        3'd2: glyph5x7 = 5'b01000;
                        3'd3: glyph5x7 = 5'b01000;
                        3'd4: glyph5x7 = 5'b01000;
                        3'd5: glyph5x7 = 5'b00100;
                        3'd6: glyph5x7 = 5'b00010;
                        default: glyph5x7 = 5'b00000;
                    endcase
                end
                8'd41: begin
                    case (row)
                        3'd0: glyph5x7 = 5'b01000;
                        3'd1: glyph5x7 = 5'b00100;
                        3'd2: glyph5x7 = 5'b00010;
                        3'd3: glyph5x7 = 5'b00010;
                        3'd4: glyph5x7 = 5'b00010;
                        3'd5: glyph5x7 = 5'b00100;
                        3'd6: glyph5x7 = 5'b01000;
                        default: glyph5x7 = 5'b00000;
                    endcase
                end
                8'd43: begin
                    case (row)
                        3'd0: glyph5x7 = 5'b00000;
                        3'd1: glyph5x7 = 5'b00100;
                        3'd2: glyph5x7 = 5'b00100;
                        3'd3: glyph5x7 = 5'b11111;
                        3'd4: glyph5x7 = 5'b00100;
                        3'd5: glyph5x7 = 5'b00100;
                        3'd6: glyph5x7 = 5'b00000;
                        default: glyph5x7 = 5'b00000;
                    endcase
                end
                8'd44: begin
                    case (row)
                        3'd0: glyph5x7 = 5'b00000;
                        3'd1: glyph5x7 = 5'b00000;
                        3'd2: glyph5x7 = 5'b00000;
                        3'd3: glyph5x7 = 5'b00000;
                        3'd4: glyph5x7 = 5'b00000;
                        3'd5: glyph5x7 = 5'b00100;
                        3'd6: glyph5x7 = 5'b01000;
                        default: glyph5x7 = 5'b00000;
                    endcase
                end
                8'd45: begin
                    case (row)
                        3'd0: glyph5x7 = 5'b00000;
                        3'd1: glyph5x7 = 5'b00000;
                        3'd2: glyph5x7 = 5'b00000;
                        3'd3: glyph5x7 = 5'b11111;
                        3'd4: glyph5x7 = 5'b00000;
                        3'd5: glyph5x7 = 5'b00000;
                        3'd6: glyph5x7 = 5'b00000;
                        default: glyph5x7 = 5'b00000;
                    endcase
                end
                8'd46: begin
                    case (row)
                        3'd0: glyph5x7 = 5'b00000;
                        3'd1: glyph5x7 = 5'b00000;
                        3'd2: glyph5x7 = 5'b00000;
                        3'd3: glyph5x7 = 5'b00000;
                        3'd4: glyph5x7 = 5'b00000;
                        3'd5: glyph5x7 = 5'b01100;
                        3'd6: glyph5x7 = 5'b01100;
                        default: glyph5x7 = 5'b00000;
                    endcase
                end
                8'd47: begin
                    case (row)
                        3'd0: glyph5x7 = 5'b00001;
                        3'd1: glyph5x7 = 5'b00010;
                        3'd2: glyph5x7 = 5'b00010;
                        3'd3: glyph5x7 = 5'b00100;
                        3'd4: glyph5x7 = 5'b01000;
                        3'd5: glyph5x7 = 5'b01000;
                        3'd6: glyph5x7 = 5'b10000;
                        default: glyph5x7 = 5'b00000;
                    endcase
                end
                8'd48: begin
                    case (row)
                        3'd0: glyph5x7 = 5'b01110;
                        3'd1: glyph5x7 = 5'b10001;
                        3'd2: glyph5x7 = 5'b10011;
                        3'd3: glyph5x7 = 5'b10101;
                        3'd4: glyph5x7 = 5'b11001;
                        3'd5: glyph5x7 = 5'b10001;
                        3'd6: glyph5x7 = 5'b01110;
                        default: glyph5x7 = 5'b00000;
                    endcase
                end
                8'd49: begin
                    case (row)
                        3'd0: glyph5x7 = 5'b00100;
                        3'd1: glyph5x7 = 5'b01100;
                        3'd2: glyph5x7 = 5'b00100;
                        3'd3: glyph5x7 = 5'b00100;
                        3'd4: glyph5x7 = 5'b00100;
                        3'd5: glyph5x7 = 5'b00100;
                        3'd6: glyph5x7 = 5'b01110;
                        default: glyph5x7 = 5'b00000;
                    endcase
                end
                8'd50: begin
                    case (row)
                        3'd0: glyph5x7 = 5'b01110;
                        3'd1: glyph5x7 = 5'b10001;
                        3'd2: glyph5x7 = 5'b00001;
                        3'd3: glyph5x7 = 5'b00010;
                        3'd4: glyph5x7 = 5'b00100;
                        3'd5: glyph5x7 = 5'b01000;
                        3'd6: glyph5x7 = 5'b11111;
                        default: glyph5x7 = 5'b00000;
                    endcase
                end
                8'd51: begin
                    case (row)
                        3'd0: glyph5x7 = 5'b11110;
                        3'd1: glyph5x7 = 5'b00001;
                        3'd2: glyph5x7 = 5'b00001;
                        3'd3: glyph5x7 = 5'b01110;
                        3'd4: glyph5x7 = 5'b00001;
                        3'd5: glyph5x7 = 5'b00001;
                        3'd6: glyph5x7 = 5'b11110;
                        default: glyph5x7 = 5'b00000;
                    endcase
                end
                8'd52: begin
                    case (row)
                        3'd0: glyph5x7 = 5'b00010;
                        3'd1: glyph5x7 = 5'b00110;
                        3'd2: glyph5x7 = 5'b01010;
                        3'd3: glyph5x7 = 5'b10010;
                        3'd4: glyph5x7 = 5'b11111;
                        3'd5: glyph5x7 = 5'b00010;
                        3'd6: glyph5x7 = 5'b00010;
                        default: glyph5x7 = 5'b00000;
                    endcase
                end
                8'd53: begin
                    case (row)
                        3'd0: glyph5x7 = 5'b11111;
                        3'd1: glyph5x7 = 5'b10000;
                        3'd2: glyph5x7 = 5'b10000;
                        3'd3: glyph5x7 = 5'b11110;
                        3'd4: glyph5x7 = 5'b00001;
                        3'd5: glyph5x7 = 5'b00001;
                        3'd6: glyph5x7 = 5'b11110;
                        default: glyph5x7 = 5'b00000;
                    endcase
                end
                8'd54: begin
                    case (row)
                        3'd0: glyph5x7 = 5'b00110;
                        3'd1: glyph5x7 = 5'b01000;
                        3'd2: glyph5x7 = 5'b10000;
                        3'd3: glyph5x7 = 5'b11110;
                        3'd4: glyph5x7 = 5'b10001;
                        3'd5: glyph5x7 = 5'b10001;
                        3'd6: glyph5x7 = 5'b01110;
                        default: glyph5x7 = 5'b00000;
                    endcase
                end
                8'd55: begin
                    case (row)
                        3'd0: glyph5x7 = 5'b11111;
                        3'd1: glyph5x7 = 5'b00001;
                        3'd2: glyph5x7 = 5'b00010;
                        3'd3: glyph5x7 = 5'b00100;
                        3'd4: glyph5x7 = 5'b01000;
                        3'd5: glyph5x7 = 5'b01000;
                        3'd6: glyph5x7 = 5'b01000;
                        default: glyph5x7 = 5'b00000;
                    endcase
                end
                8'd56: begin
                    case (row)
                        3'd0: glyph5x7 = 5'b01110;
                        3'd1: glyph5x7 = 5'b10001;
                        3'd2: glyph5x7 = 5'b10001;
                        3'd3: glyph5x7 = 5'b01110;
                        3'd4: glyph5x7 = 5'b10001;
                        3'd5: glyph5x7 = 5'b10001;
                        3'd6: glyph5x7 = 5'b01110;
                        default: glyph5x7 = 5'b00000;
                    endcase
                end
                8'd57: begin
                    case (row)
                        3'd0: glyph5x7 = 5'b01110;
                        3'd1: glyph5x7 = 5'b10001;
                        3'd2: glyph5x7 = 5'b10001;
                        3'd3: glyph5x7 = 5'b01111;
                        3'd4: glyph5x7 = 5'b00001;
                        3'd5: glyph5x7 = 5'b00010;
                        3'd6: glyph5x7 = 5'b01100;
                        default: glyph5x7 = 5'b00000;
                    endcase
                end
                8'd58: begin
                    case (row)
                        3'd0: glyph5x7 = 5'b00000;
                        3'd1: glyph5x7 = 5'b01100;
                        3'd2: glyph5x7 = 5'b01100;
                        3'd3: glyph5x7 = 5'b00000;
                        3'd4: glyph5x7 = 5'b01100;
                        3'd5: glyph5x7 = 5'b01100;
                        3'd6: glyph5x7 = 5'b00000;
                        default: glyph5x7 = 5'b00000;
                    endcase
                end
                8'd61: begin
                    case (row)
                        3'd0: glyph5x7 = 5'b00000;
                        3'd1: glyph5x7 = 5'b00000;
                        3'd2: glyph5x7 = 5'b11111;
                        3'd3: glyph5x7 = 5'b00000;
                        3'd4: glyph5x7 = 5'b11111;
                        3'd5: glyph5x7 = 5'b00000;
                        3'd6: glyph5x7 = 5'b00000;
                        default: glyph5x7 = 5'b00000;
                    endcase
                end
                8'd64: begin
                    case (row)
                        3'd0: glyph5x7 = 5'b01110;
                        3'd1: glyph5x7 = 5'b10001;
                        3'd2: glyph5x7 = 5'b10111;
                        3'd3: glyph5x7 = 5'b10101;
                        3'd4: glyph5x7 = 5'b10111;
                        3'd5: glyph5x7 = 5'b10000;
                        3'd6: glyph5x7 = 5'b01110;
                        default: glyph5x7 = 5'b00000;
                    endcase
                end
                8'd65: begin
                    case (row)
                        3'd0: glyph5x7 = 5'b01110;
                        3'd1: glyph5x7 = 5'b10001;
                        3'd2: glyph5x7 = 5'b10001;
                        3'd3: glyph5x7 = 5'b11111;
                        3'd4: glyph5x7 = 5'b10001;
                        3'd5: glyph5x7 = 5'b10001;
                        3'd6: glyph5x7 = 5'b10001;
                        default: glyph5x7 = 5'b00000;
                    endcase
                end
                8'd66: begin
                    case (row)
                        3'd0: glyph5x7 = 5'b11110;
                        3'd1: glyph5x7 = 5'b10001;
                        3'd2: glyph5x7 = 5'b10001;
                        3'd3: glyph5x7 = 5'b11110;
                        3'd4: glyph5x7 = 5'b10001;
                        3'd5: glyph5x7 = 5'b10001;
                        3'd6: glyph5x7 = 5'b11110;
                        default: glyph5x7 = 5'b00000;
                    endcase
                end
                8'd67: begin
                    case (row)
                        3'd0: glyph5x7 = 5'b01110;
                        3'd1: glyph5x7 = 5'b10001;
                        3'd2: glyph5x7 = 5'b10000;
                        3'd3: glyph5x7 = 5'b10000;
                        3'd4: glyph5x7 = 5'b10000;
                        3'd5: glyph5x7 = 5'b10001;
                        3'd6: glyph5x7 = 5'b01110;
                        default: glyph5x7 = 5'b00000;
                    endcase
                end
                8'd68: begin
                    case (row)
                        3'd0: glyph5x7 = 5'b11110;
                        3'd1: glyph5x7 = 5'b10001;
                        3'd2: glyph5x7 = 5'b10001;
                        3'd3: glyph5x7 = 5'b10001;
                        3'd4: glyph5x7 = 5'b10001;
                        3'd5: glyph5x7 = 5'b10001;
                        3'd6: glyph5x7 = 5'b11110;
                        default: glyph5x7 = 5'b00000;
                    endcase
                end
                8'd69: begin
                    case (row)
                        3'd0: glyph5x7 = 5'b11111;
                        3'd1: glyph5x7 = 5'b10000;
                        3'd2: glyph5x7 = 5'b10000;
                        3'd3: glyph5x7 = 5'b11110;
                        3'd4: glyph5x7 = 5'b10000;
                        3'd5: glyph5x7 = 5'b10000;
                        3'd6: glyph5x7 = 5'b11111;
                        default: glyph5x7 = 5'b00000;
                    endcase
                end
                8'd70: begin
                    case (row)
                        3'd0: glyph5x7 = 5'b11111;
                        3'd1: glyph5x7 = 5'b10000;
                        3'd2: glyph5x7 = 5'b10000;
                        3'd3: glyph5x7 = 5'b11110;
                        3'd4: glyph5x7 = 5'b10000;
                        3'd5: glyph5x7 = 5'b10000;
                        3'd6: glyph5x7 = 5'b10000;
                        default: glyph5x7 = 5'b00000;
                    endcase
                end
                8'd71: begin
                    case (row)
                        3'd0: glyph5x7 = 5'b01110;
                        3'd1: glyph5x7 = 5'b10001;
                        3'd2: glyph5x7 = 5'b10000;
                        3'd3: glyph5x7 = 5'b10111;
                        3'd4: glyph5x7 = 5'b10001;
                        3'd5: glyph5x7 = 5'b10001;
                        3'd6: glyph5x7 = 5'b01110;
                        default: glyph5x7 = 5'b00000;
                    endcase
                end
                8'd72: begin
                    case (row)
                        3'd0: glyph5x7 = 5'b10001;
                        3'd1: glyph5x7 = 5'b10001;
                        3'd2: glyph5x7 = 5'b10001;
                        3'd3: glyph5x7 = 5'b11111;
                        3'd4: glyph5x7 = 5'b10001;
                        3'd5: glyph5x7 = 5'b10001;
                        3'd6: glyph5x7 = 5'b10001;
                        default: glyph5x7 = 5'b00000;
                    endcase
                end
                8'd73: begin
                    case (row)
                        3'd0: glyph5x7 = 5'b01110;
                        3'd1: glyph5x7 = 5'b00100;
                        3'd2: glyph5x7 = 5'b00100;
                        3'd3: glyph5x7 = 5'b00100;
                        3'd4: glyph5x7 = 5'b00100;
                        3'd5: glyph5x7 = 5'b00100;
                        3'd6: glyph5x7 = 5'b01110;
                        default: glyph5x7 = 5'b00000;
                    endcase
                end
                8'd74: begin
                    case (row)
                        3'd0: glyph5x7 = 5'b00001;
                        3'd1: glyph5x7 = 5'b00001;
                        3'd2: glyph5x7 = 5'b00001;
                        3'd3: glyph5x7 = 5'b00001;
                        3'd4: glyph5x7 = 5'b10001;
                        3'd5: glyph5x7 = 5'b10001;
                        3'd6: glyph5x7 = 5'b01110;
                        default: glyph5x7 = 5'b00000;
                    endcase
                end
                8'd75: begin
                    case (row)
                        3'd0: glyph5x7 = 5'b10001;
                        3'd1: glyph5x7 = 5'b10010;
                        3'd2: glyph5x7 = 5'b10100;
                        3'd3: glyph5x7 = 5'b11000;
                        3'd4: glyph5x7 = 5'b10100;
                        3'd5: glyph5x7 = 5'b10010;
                        3'd6: glyph5x7 = 5'b10001;
                        default: glyph5x7 = 5'b00000;
                    endcase
                end
                8'd76: begin
                    case (row)
                        3'd0: glyph5x7 = 5'b10000;
                        3'd1: glyph5x7 = 5'b10000;
                        3'd2: glyph5x7 = 5'b10000;
                        3'd3: glyph5x7 = 5'b10000;
                        3'd4: glyph5x7 = 5'b10000;
                        3'd5: glyph5x7 = 5'b10000;
                        3'd6: glyph5x7 = 5'b11111;
                        default: glyph5x7 = 5'b00000;
                    endcase
                end
                8'd77: begin
                    case (row)
                        3'd0: glyph5x7 = 5'b10001;
                        3'd1: glyph5x7 = 5'b11011;
                        3'd2: glyph5x7 = 5'b10101;
                        3'd3: glyph5x7 = 5'b10101;
                        3'd4: glyph5x7 = 5'b10001;
                        3'd5: glyph5x7 = 5'b10001;
                        3'd6: glyph5x7 = 5'b10001;
                        default: glyph5x7 = 5'b00000;
                    endcase
                end
                8'd78: begin
                    case (row)
                        3'd0: glyph5x7 = 5'b10001;
                        3'd1: glyph5x7 = 5'b11001;
                        3'd2: glyph5x7 = 5'b10101;
                        3'd3: glyph5x7 = 5'b10011;
                        3'd4: glyph5x7 = 5'b10001;
                        3'd5: glyph5x7 = 5'b10001;
                        3'd6: glyph5x7 = 5'b10001;
                        default: glyph5x7 = 5'b00000;
                    endcase
                end
                8'd79: begin
                    case (row)
                        3'd0: glyph5x7 = 5'b01110;
                        3'd1: glyph5x7 = 5'b10001;
                        3'd2: glyph5x7 = 5'b10001;
                        3'd3: glyph5x7 = 5'b10001;
                        3'd4: glyph5x7 = 5'b10001;
                        3'd5: glyph5x7 = 5'b10001;
                        3'd6: glyph5x7 = 5'b01110;
                        default: glyph5x7 = 5'b00000;
                    endcase
                end
                8'd80: begin
                    case (row)
                        3'd0: glyph5x7 = 5'b11110;
                        3'd1: glyph5x7 = 5'b10001;
                        3'd2: glyph5x7 = 5'b10001;
                        3'd3: glyph5x7 = 5'b11110;
                        3'd4: glyph5x7 = 5'b10000;
                        3'd5: glyph5x7 = 5'b10000;
                        3'd6: glyph5x7 = 5'b10000;
                        default: glyph5x7 = 5'b00000;
                    endcase
                end
                8'd81: begin
                    case (row)
                        3'd0: glyph5x7 = 5'b01110;
                        3'd1: glyph5x7 = 5'b10001;
                        3'd2: glyph5x7 = 5'b10001;
                        3'd3: glyph5x7 = 5'b10001;
                        3'd4: glyph5x7 = 5'b10101;
                        3'd5: glyph5x7 = 5'b10010;
                        3'd6: glyph5x7 = 5'b01101;
                        default: glyph5x7 = 5'b00000;
                    endcase
                end
                8'd82: begin
                    case (row)
                        3'd0: glyph5x7 = 5'b11110;
                        3'd1: glyph5x7 = 5'b10001;
                        3'd2: glyph5x7 = 5'b10001;
                        3'd3: glyph5x7 = 5'b11110;
                        3'd4: glyph5x7 = 5'b10100;
                        3'd5: glyph5x7 = 5'b10010;
                        3'd6: glyph5x7 = 5'b10001;
                        default: glyph5x7 = 5'b00000;
                    endcase
                end
                8'd83: begin
                    case (row)
                        3'd0: glyph5x7 = 5'b01111;
                        3'd1: glyph5x7 = 5'b10000;
                        3'd2: glyph5x7 = 5'b10000;
                        3'd3: glyph5x7 = 5'b01110;
                        3'd4: glyph5x7 = 5'b00001;
                        3'd5: glyph5x7 = 5'b00001;
                        3'd6: glyph5x7 = 5'b11110;
                        default: glyph5x7 = 5'b00000;
                    endcase
                end
                8'd84: begin
                    case (row)
                        3'd0: glyph5x7 = 5'b11111;
                        3'd1: glyph5x7 = 5'b00100;
                        3'd2: glyph5x7 = 5'b00100;
                        3'd3: glyph5x7 = 5'b00100;
                        3'd4: glyph5x7 = 5'b00100;
                        3'd5: glyph5x7 = 5'b00100;
                        3'd6: glyph5x7 = 5'b00100;
                        default: glyph5x7 = 5'b00000;
                    endcase
                end
                8'd85: begin
                    case (row)
                        3'd0: glyph5x7 = 5'b10001;
                        3'd1: glyph5x7 = 5'b10001;
                        3'd2: glyph5x7 = 5'b10001;
                        3'd3: glyph5x7 = 5'b10001;
                        3'd4: glyph5x7 = 5'b10001;
                        3'd5: glyph5x7 = 5'b10001;
                        3'd6: glyph5x7 = 5'b01110;
                        default: glyph5x7 = 5'b00000;
                    endcase
                end
                8'd86: begin
                    case (row)
                        3'd0: glyph5x7 = 5'b10001;
                        3'd1: glyph5x7 = 5'b10001;
                        3'd2: glyph5x7 = 5'b10001;
                        3'd3: glyph5x7 = 5'b10001;
                        3'd4: glyph5x7 = 5'b01010;
                        3'd5: glyph5x7 = 5'b01010;
                        3'd6: glyph5x7 = 5'b00100;
                        default: glyph5x7 = 5'b00000;
                    endcase
                end
                8'd87: begin
                    case (row)
                        3'd0: glyph5x7 = 5'b10001;
                        3'd1: glyph5x7 = 5'b10001;
                        3'd2: glyph5x7 = 5'b10001;
                        3'd3: glyph5x7 = 5'b10101;
                        3'd4: glyph5x7 = 5'b10101;
                        3'd5: glyph5x7 = 5'b11011;
                        3'd6: glyph5x7 = 5'b10001;
                        default: glyph5x7 = 5'b00000;
                    endcase
                end
                8'd88: begin
                    case (row)
                        3'd0: glyph5x7 = 5'b10001;
                        3'd1: glyph5x7 = 5'b01010;
                        3'd2: glyph5x7 = 5'b00100;
                        3'd3: glyph5x7 = 5'b00100;
                        3'd4: glyph5x7 = 5'b00100;
                        3'd5: glyph5x7 = 5'b01010;
                        3'd6: glyph5x7 = 5'b10001;
                        default: glyph5x7 = 5'b00000;
                    endcase
                end
                8'd89: begin
                    case (row)
                        3'd0: glyph5x7 = 5'b10001;
                        3'd1: glyph5x7 = 5'b01010;
                        3'd2: glyph5x7 = 5'b00100;
                        3'd3: glyph5x7 = 5'b00100;
                        3'd4: glyph5x7 = 5'b00100;
                        3'd5: glyph5x7 = 5'b00100;
                        3'd6: glyph5x7 = 5'b00100;
                        default: glyph5x7 = 5'b00000;
                    endcase
                end
                8'd90: begin
                    case (row)
                        3'd0: glyph5x7 = 5'b11111;
                        3'd1: glyph5x7 = 5'b00001;
                        3'd2: glyph5x7 = 5'b00010;
                        3'd3: glyph5x7 = 5'b00100;
                        3'd4: glyph5x7 = 5'b01000;
                        3'd5: glyph5x7 = 5'b10000;
                        3'd6: glyph5x7 = 5'b11111;
                        default: glyph5x7 = 5'b00000;
                    endcase
                end
                default: glyph5x7 = 5'b00000;
            endcase
        end
    endfunction

    function [7:0] static_char;
        input [7:0] line_id;
        input [7:0] idx;
        begin
            static_char = 8'd32;
            case (line_id)
                8'd0: begin
                    case (idx)
                        8'd0: static_char = 8'd67;
                        8'd1: static_char = 8'd77;
                        8'd2: static_char = 8'd80;
                        8'd3: static_char = 8'd83;
                        8'd4: static_char = 8'd50;
                        8'd5: static_char = 8'd47;
                        8'd6: static_char = 8'd77;
                        8'd7: static_char = 8'd77;
                        8'd8: static_char = 8'd67;
                        8'd9: static_char = 8'd51;
                        8'd10: static_char = 8'd52;
                        8'd11: static_char = 8'd49;
                        8'd12: static_char = 8'd54;
                        8'd13: static_char = 8'd48;
                        8'd14: static_char = 8'd80;
                        8'd15: static_char = 8'd74;
                        8'd16: static_char = 8'd32;
                        8'd17: static_char = 8'd64;
                        8'd18: static_char = 8'd32;
                        8'd19: static_char = 8'd55;
                        8'd20: static_char = 8'd39;
                        8'd21: static_char = 8'd72;
                        8'd22: static_char = 8'd51;
                        8'd23: static_char = 8'd48;
                        default: static_char = 8'd32;
                    endcase
                end
                8'd1: begin
                    case (idx)
                        8'd0: static_char = 8'd80;
                        8'd1: static_char = 8'd76;
                        8'd2: static_char = 8'd65;
                        8'd3: static_char = 8'd78;
                        8'd4: static_char = 8'd65;
                        8'd5: static_char = 8'd82;
                        8'd6: static_char = 8'd32;
                        8'd7: static_char = 8'd72;
                        8'd8: static_char = 8'd69;
                        8'd9: static_char = 8'd65;
                        8'd10: static_char = 8'd68;
                        8'd11: static_char = 8'd73;
                        8'd12: static_char = 8'd78;
                        8'd13: static_char = 8'd71;
                        8'd14: static_char = 8'd32;
                        8'd15: static_char = 8'd65;
                        8'd16: static_char = 8'd84;
                        8'd17: static_char = 8'd65;
                        8'd18: static_char = 8'd78;
                        8'd19: static_char = 8'd50;
                        8'd20: static_char = 8'd40;
                        8'd21: static_char = 8'd77;
                        8'd22: static_char = 8'd89;
                        8'd23: static_char = 8'd44;
                        8'd24: static_char = 8'd77;
                        8'd25: static_char = 8'd88;
                        8'd26: static_char = 8'd41;
                        default: static_char = 8'd32;
                    endcase
                end
                8'd2: begin
                    case (idx)
                        8'd0: static_char = 8'd78;
                        8'd1: static_char = 8'd79;
                        8'd2: static_char = 8'd84;
                        8'd3: static_char = 8'd32;
                        8'd4: static_char = 8'd84;
                        8'd5: static_char = 8'd73;
                        8'd6: static_char = 8'd76;
                        8'd7: static_char = 8'd84;
                        8'd8: static_char = 8'd32;
                        8'd9: static_char = 8'd67;
                        8'd10: static_char = 8'd79;
                        8'd11: static_char = 8'd77;
                        8'd12: static_char = 8'd80;
                        8'd13: static_char = 8'd69;
                        8'd14: static_char = 8'd78;
                        8'd15: static_char = 8'd83;
                        8'd16: static_char = 8'd65;
                        8'd17: static_char = 8'd84;
                        8'd18: static_char = 8'd69;
                        8'd19: static_char = 8'd68;
                        default: static_char = 8'd32;
                    endcase
                end
                8'd3: begin
                    case (idx)
                        8'd0: static_char = 8'd80;
                        8'd1: static_char = 8'd76;
                        8'd2: static_char = 8'd65;
                        8'd3: static_char = 8'd78;
                        8'd4: static_char = 8'd65;
                        8'd5: static_char = 8'd82;
                        8'd6: static_char = 8'd32;
                        8'd7: static_char = 8'd67;
                        8'd8: static_char = 8'd79;
                        8'd9: static_char = 8'd77;
                        8'd10: static_char = 8'd80;
                        8'd11: static_char = 8'd65;
                        8'd12: static_char = 8'd83;
                        8'd13: static_char = 8'd83;
                        default: static_char = 8'd32;
                    endcase
                end
                8'd4: begin
                    case (idx)
                        8'd0: static_char = 8'd76;
                        8'd1: static_char = 8'd73;
                        8'd2: static_char = 8'd86;
                        8'd3: static_char = 8'd69;
                        8'd4: static_char = 8'd32;
                        8'd5: static_char = 8'd77;
                        8'd6: static_char = 8'd88;
                        8'd7: static_char = 8'd47;
                        8'd8: static_char = 8'd77;
                        8'd9: static_char = 8'd89;
                        8'd10: static_char = 8'd32;
                        8'd11: static_char = 8'd86;
                        8'd12: static_char = 8'd69;
                        8'd13: static_char = 8'd67;
                        8'd14: static_char = 8'd84;
                        8'd15: static_char = 8'd79;
                        8'd16: static_char = 8'd82;
                        default: static_char = 8'd32;
                    endcase
                end
                8'd5: begin
                    case (idx)
                        8'd0: static_char = 8'd77;
                        8'd1: static_char = 8'd65;
                        8'd2: static_char = 8'd71;
                        8'd3: static_char = 8'd32;
                        8'd4: static_char = 8'd70;
                        8'd5: static_char = 8'd82;
                        8'd6: static_char = 8'd69;
                        8'd7: static_char = 8'd83;
                        8'd8: static_char = 8'd72;
                        8'd9: static_char = 8'd78;
                        8'd10: static_char = 8'd69;
                        8'd11: static_char = 8'd83;
                        8'd12: static_char = 8'd83;
                        default: static_char = 8'd32;
                    endcase
                end
                8'd6: begin
                    case (idx)
                        8'd0: static_char = 8'd70;
                        8'd1: static_char = 8'd73;
                        8'd2: static_char = 8'd69;
                        8'd3: static_char = 8'd76;
                        8'd4: static_char = 8'd68;
                        8'd5: static_char = 8'd32;
                        8'd6: static_char = 8'd78;
                        8'd7: static_char = 8'd79;
                        8'd8: static_char = 8'd82;
                        8'd9: static_char = 8'd77;
                        8'd10: static_char = 8'd32;
                        8'd11: static_char = 8'd65;
                        8'd12: static_char = 8'd80;
                        8'd13: static_char = 8'd80;
                        8'd14: static_char = 8'd82;
                        8'd15: static_char = 8'd79;
                        8'd16: static_char = 8'd88;
                        default: static_char = 8'd32;
                    endcase
                end
                8'd7: begin
                    case (idx)
                        8'd0: static_char = 8'd72;
                        8'd1: static_char = 8'd69;
                        8'd2: static_char = 8'd65;
                        8'd3: static_char = 8'd68;
                        8'd4: static_char = 8'd73;
                        8'd5: static_char = 8'd78;
                        8'd6: static_char = 8'd71;
                        8'd7: static_char = 8'd32;
                        8'd8: static_char = 8'd67;
                        8'd9: static_char = 8'd82;
                        8'd10: static_char = 8'd79;
                        8'd11: static_char = 8'd83;
                        8'd12: static_char = 8'd83;
                        8'd13: static_char = 8'd67;
                        8'd14: static_char = 8'd72;
                        8'd15: static_char = 8'd69;
                        8'd16: static_char = 8'd67;
                        8'd17: static_char = 8'd75;
                        default: static_char = 8'd32;
                    endcase
                end
                8'd8: begin
                    case (idx)
                        8'd0: static_char = 8'd67;
                        8'd1: static_char = 8'd77;
                        8'd2: static_char = 8'd80;
                        8'd3: static_char = 8'd83;
                        8'd4: static_char = 8'd50;
                        8'd5: static_char = 8'd32;
                        8'd6: static_char = 8'd82;
                        8'd7: static_char = 8'd65;
                        8'd8: static_char = 8'd87;
                        8'd9: static_char = 8'd32;
                        8'd10: static_char = 8'd77;
                        8'd11: static_char = 8'd65;
                        8'd12: static_char = 8'd71;
                        default: static_char = 8'd32;
                    endcase
                end
                8'd9: begin
                    case (idx)
                        8'd0: static_char = 8'd48;
                        8'd1: static_char = 8'd61;
                        8'd2: static_char = 8'd43;
                        8'd3: static_char = 8'd77;
                        8'd4: static_char = 8'd88;
                        8'd5: static_char = 8'd32;
                        8'd6: static_char = 8'd32;
                        8'd7: static_char = 8'd57;
                        8'd8: static_char = 8'd48;
                        8'd9: static_char = 8'd61;
                        8'd10: static_char = 8'd43;
                        8'd11: static_char = 8'd77;
                        8'd12: static_char = 8'd89;
                        default: static_char = 8'd32;
                    endcase
                end
                8'd10: begin
                    case (idx)
                        8'd0: static_char = 8'd77;
                        8'd1: static_char = 8'd65;
                        8'd2: static_char = 8'd71;
                        8'd3: static_char = 8'd32;
                        8'd4: static_char = 8'd70;
                        8'd5: static_char = 8'd82;
                        8'd6: static_char = 8'd69;
                        8'd7: static_char = 8'd83;
                        8'd8: static_char = 8'd72;
                        default: static_char = 8'd32;
                    endcase
                end
                8'd11: begin
                    case (idx)
                        8'd0: static_char = 8'd77;
                        8'd1: static_char = 8'd65;
                        8'd2: static_char = 8'd71;
                        8'd3: static_char = 8'd32;
                        8'd4: static_char = 8'd83;
                        8'd5: static_char = 8'd84;
                        8'd6: static_char = 8'd65;
                        8'd7: static_char = 8'd76;
                        8'd8: static_char = 8'd69;
                        default: static_char = 8'd32;
                    endcase
                end
                8'd12: begin
                    case (idx)
                        8'd0: static_char = 8'd66;
                        8'd1: static_char = 8'd65;
                        8'd2: static_char = 8'd68;
                        8'd3: static_char = 8'd32;
                        8'd4: static_char = 8'd83;
                        8'd5: static_char = 8'd84;
                        8'd6: static_char = 8'd65;
                        8'd7: static_char = 8'd84;
                        8'd8: static_char = 8'd85;
                        8'd9: static_char = 8'd83;
                        default: static_char = 8'd32;
                    endcase
                end
                8'd13: begin
                    case (idx)
                        8'd0: static_char = 8'd83;
                        8'd1: static_char = 8'd84;
                        8'd2: static_char = 8'd65;
                        8'd3: static_char = 8'd76;
                        8'd4: static_char = 8'd69;
                        8'd5: static_char = 8'd32;
                        8'd6: static_char = 8'd79;
                        8'd7: static_char = 8'd82;
                        8'd8: static_char = 8'd32;
                        8'd9: static_char = 8'd73;
                        8'd10: static_char = 8'd78;
                        8'd11: static_char = 8'd86;
                        8'd12: static_char = 8'd65;
                        8'd13: static_char = 8'd76;
                        8'd14: static_char = 8'd73;
                        8'd15: static_char = 8'd68;
                        default: static_char = 8'd32;
                    endcase
                end
                8'd14: begin
                    case (idx)
                        8'd0: static_char = 8'd80;
                        8'd1: static_char = 8'd76;
                        8'd2: static_char = 8'd79;
                        8'd3: static_char = 8'd84;
                        8'd4: static_char = 8'd84;
                        8'd5: static_char = 8'd69;
                        8'd6: static_char = 8'd68;
                        8'd7: static_char = 8'd32;
                        8'd8: static_char = 8'd82;
                        8'd9: static_char = 8'd65;
                        8'd10: static_char = 8'd87;
                        8'd11: static_char = 8'd32;
                        8'd12: static_char = 8'd77;
                        8'd13: static_char = 8'd88;
                        8'd14: static_char = 8'd47;
                        8'd15: static_char = 8'd77;
                        8'd16: static_char = 8'd89;
                        default: static_char = 8'd32;
                    endcase
                end
                8'd15: begin
                    case (idx)
                        8'd0: static_char = 8'd82;
                        8'd1: static_char = 8'd65;
                        8'd2: static_char = 8'd87;
                        8'd3: static_char = 8'd32;
                        8'd4: static_char = 8'd86;
                        8'd5: static_char = 8'd69;
                        8'd6: static_char = 8'd67;
                        8'd7: static_char = 8'd84;
                        8'd8: static_char = 8'd79;
                        8'd9: static_char = 8'd82;
                        default: static_char = 8'd32;
                    endcase
                end
                8'd16: begin
                    case (idx)
                        8'd0: static_char = 8'd70;
                        8'd1: static_char = 8'd80;
                        8'd2: static_char = 8'd71;
                        8'd3: static_char = 8'd65;
                        8'd4: static_char = 8'd32;
                        8'd5: static_char = 8'd86;
                        8'd6: static_char = 8'd83;
                        8'd7: static_char = 8'd32;
                        8'd8: static_char = 8'd82;
                        8'd9: static_char = 8'd65;
                        8'd10: static_char = 8'd87;
                        8'd11: static_char = 8'd32;
                        8'd12: static_char = 8'd65;
                        8'd13: static_char = 8'd84;
                        8'd14: static_char = 8'd65;
                        8'd15: static_char = 8'd78;
                        8'd16: static_char = 8'd50;
                        default: static_char = 8'd32;
                    endcase
                end
                8'd17: begin
                    case (idx)
                        8'd0: static_char = 8'd70;
                        8'd1: static_char = 8'd82;
                        8'd2: static_char = 8'd69;
                        8'd3: static_char = 8'd83;
                        8'd4: static_char = 8'd72;
                        default: static_char = 8'd32;
                    endcase
                end
                8'd18: begin
                    case (idx)
                        8'd0: static_char = 8'd83;
                        8'd1: static_char = 8'd84;
                        8'd2: static_char = 8'd65;
                        8'd3: static_char = 8'd76;
                        8'd4: static_char = 8'd69;
                        default: static_char = 8'd32;
                    endcase
                end
                8'd19: begin
                    case (idx)
                        8'd0: static_char = 8'd66;
                        8'd1: static_char = 8'd65;
                        8'd2: static_char = 8'd68;
                        default: static_char = 8'd32;
                    endcase
                end
                8'd20: begin
                    case (idx)
                        8'd0:  static_char = 8'd80; // P
                        8'd1:  static_char = 8'd76; // L
                        8'd2:  static_char = 8'd65; // A
                        8'd3:  static_char = 8'd78; // N
                        8'd4:  static_char = 8'd65; // A
                        8'd5:  static_char = 8'd82; // R
                        8'd6:  static_char = 8'd32;
                        8'd7:  static_char = 8'd72; // H
                        8'd8:  static_char = 8'd69; // E
                        8'd9:  static_char = 8'd65; // A
                        8'd10: static_char = 8'd68; // D
                        8'd11: static_char = 8'd73; // I
                        8'd12: static_char = 8'd78; // N
                        8'd13: static_char = 8'd71; // G
                        8'd14: static_char = 8'd32;
                        8'd15: static_char = 8'd79; // O
                        8'd16: static_char = 8'd78; // N
                        8'd17: static_char = 8'd76; // L
                        8'd18: static_char = 8'd89; // Y
                        default: static_char = 8'd32;
                    endcase
                end
                default: static_char = 8'd32;
            endcase
        end
    endfunction

    function [7:0] sector_char_a;
        input [2:0] sec;
        begin
            case (sec)
                3'd0: sector_char_a = 8'd69;  // E
                3'd1: sector_char_a = 8'd78;  // N
                3'd2: sector_char_a = 8'd78;  // N
                3'd3: sector_char_a = 8'd78;  // N
                3'd4: sector_char_a = 8'd87;  // W
                3'd5: sector_char_a = 8'd83;  // S
                3'd6: sector_char_a = 8'd83;  // S
                3'd7: sector_char_a = 8'd83;  // S
                default: sector_char_a = 8'd32;
            endcase
        end
    endfunction

    function [7:0] sector_char_b;
        input [2:0] sec;
        begin
            case (sec)
                3'd1: sector_char_b = 8'd69;  // E
                3'd3: sector_char_b = 8'd87;  // W
                3'd5: sector_char_b = 8'd87;  // W
                3'd7: sector_char_b = 8'd69;  // E
                default: sector_char_b = 8'd32;
            endcase
        end
    endfunction

    function [2:0] raw_sector8;
        input signed [15:0] x;
        input signed [15:0] y;
        reg [15:0] ax;
        reg [15:0] ay;
        begin
            ax = abs16(x);
            ay = abs16(y);
            if ({1'b0, ax} >= ({1'b0, ay} << 1))
                raw_sector8 = (x < 0) ? 3'd4 : 3'd0;
            else if ({1'b0, ay} >= ({1'b0, ax} << 1))
                raw_sector8 = (y < 0) ? 3'd6 : 3'd2;
            else if ((x >= 0) && (y >= 0))
                raw_sector8 = 3'd1;
            else if ((x < 0) && (y >= 0))
                raw_sector8 = 3'd3;
            else if ((x < 0) && (y < 0))
                raw_sector8 = 3'd5;
            else
                raw_sector8 = 3'd7;
        end
    endfunction

    function [3:0] sector_diff8;
        input [2:0] a;
        input [2:0] b;
        reg [3:0] d;
        begin
            if (a >= b)
                d = {1'b0, a} - {1'b0, b};
            else
                d = {1'b0, b} - {1'b0, a};
            if (d > 4'd4)
                sector_diff8 = 4'd8 - d;
            else
                sector_diff8 = d;
        end
    endfunction

    function [7:0] dyn_char;
        input [7:0] line_id;
        input [7:0] idx;
        input [8:0] hdg_deg_i;
        input [9:0] age_i;
        input [15:0] seq_i;
        input [7:0] stat_i;
        input [15:0] mx_i;
        input [15:0] my_i;
        input [15:0] mz_i;
        input [15:0] norm_i;
        input [2:0] pub_sec_i;
        input [2:0] raw_sec_i;
        input [15:0] nack_i;
        input [15:0] timeout_i;
        input [15:0] txn_i;
        input        head_fresh_i;
        reg [3:0] h;
        reg [3:0] t;
        reg [3:0] o;
        begin
            dyn_char = 8'd32;
            h = hdg_deg_i / 9'd100;
            t = (hdg_deg_i - (h * 9'd100)) / 9'd10;
            o = hdg_deg_i - (h * 9'd100) - (t * 9'd10);
            case (line_id)
                8'd100: begin
                    case (idx)
                        8'd0: dyn_char = 8'd72;
                        8'd1: dyn_char = 8'd68;
                        8'd2: dyn_char = 8'd71;
                        8'd3: dyn_char = 8'd32;
                        8'd4: dyn_char = 8'd48 + h;
                        8'd5: dyn_char = 8'd48 + t;
                        8'd6: dyn_char = 8'd48 + o;
                        8'd7: dyn_char = 8'd32;
                        8'd8: dyn_char = 8'd68;
                        8'd9: dyn_char = 8'd69;
                        8'd10: dyn_char = 8'd71;
                        default: dyn_char = 8'd32;
                    endcase
                end
                8'd101: begin
                    case (idx)
                        8'd0: dyn_char = 8'd65;
                        8'd1: dyn_char = 8'd71;
                        8'd2: dyn_char = 8'd69;
                        8'd3: dyn_char = 8'd32;
                        8'd4: dyn_char = 8'd48 + (age_i / 10'd100);
                        8'd5: dyn_char = 8'd48 + ((age_i / 10'd10) % 10'd10);
                        8'd6: dyn_char = 8'd48 + (age_i % 10'd10);
                        8'd7: dyn_char = 8'd32;
                        8'd8: dyn_char = 8'd77;
                        8'd9: dyn_char = 8'd83;
                        default: dyn_char = 8'd32;
                    endcase
                end
                8'd102: begin
                    case (idx)
                        8'd0: dyn_char = 8'd83;
                        8'd1: dyn_char = 8'd69;
                        8'd2: dyn_char = 8'd81;
                        8'd3: dyn_char = 8'd32;
                        8'd4: dyn_char = hex_char(seq_i[15:12]);
                        8'd5: dyn_char = hex_char(seq_i[11:8]);
                        8'd6: dyn_char = hex_char(seq_i[7:4]);
                        8'd7: dyn_char = hex_char(seq_i[3:0]);
                        default: dyn_char = 8'd32;
                    endcase
                end
                8'd103: begin
                    case (idx)
                        8'd0: dyn_char = 8'd83;
                        8'd1: dyn_char = 8'd84;
                        8'd2: dyn_char = 8'd65;
                        8'd3: dyn_char = 8'd84;
                        8'd4: dyn_char = 8'd32;
                        8'd5: dyn_char = hex_char(stat_i[7:4]);
                        8'd6: dyn_char = hex_char(stat_i[3:0]);
                        default: dyn_char = 8'd32;
                    endcase
                end
                8'd104: begin
                    case (idx)
                        8'd0: dyn_char = 8'd77;
                        8'd1: dyn_char = 8'd88;
                        8'd2: dyn_char = 8'd32;
                        8'd3: dyn_char = hex_char(mx_i[15:12]);
                        8'd4: dyn_char = hex_char(mx_i[11:8]);
                        8'd5: dyn_char = hex_char(mx_i[7:4]);
                        8'd6: dyn_char = hex_char(mx_i[3:0]);
                        default: dyn_char = 8'd32;
                    endcase
                end
                8'd105: begin
                    case (idx)
                        8'd0: dyn_char = 8'd77;
                        8'd1: dyn_char = 8'd89;
                        8'd2: dyn_char = 8'd32;
                        8'd3: dyn_char = hex_char(my_i[15:12]);
                        8'd4: dyn_char = hex_char(my_i[11:8]);
                        8'd5: dyn_char = hex_char(my_i[7:4]);
                        8'd6: dyn_char = hex_char(my_i[3:0]);
                        default: dyn_char = 8'd32;
                    endcase
                end
                8'd106: begin
                    case (idx)
                        8'd0: dyn_char = 8'd77;
                        8'd1: dyn_char = 8'd90;
                        8'd2: dyn_char = 8'd32;
                        8'd3: dyn_char = hex_char(mz_i[15:12]);
                        8'd4: dyn_char = hex_char(mz_i[11:8]);
                        8'd5: dyn_char = hex_char(mz_i[7:4]);
                        8'd6: dyn_char = hex_char(mz_i[3:0]);
                        default: dyn_char = 8'd32;
                    endcase
                end
                8'd107: begin
                    case (idx)
                        8'd0: dyn_char = 8'd78;
                        8'd1: dyn_char = 8'd79;
                        8'd2: dyn_char = 8'd82;
                        8'd3: dyn_char = 8'd77;
                        8'd4: dyn_char = 8'd32;
                        8'd5: dyn_char = hex_char(norm_i[15:12]);
                        8'd6: dyn_char = hex_char(norm_i[11:8]);
                        8'd7: dyn_char = hex_char(norm_i[7:4]);
                        8'd8: dyn_char = hex_char(norm_i[3:0]);
                        default: dyn_char = 8'd32;
                    endcase
                end
                8'd108: begin
                    case (idx)
                        8'd0: dyn_char = 8'd70;
                        8'd1: dyn_char = 8'd80;
                        8'd2: dyn_char = 8'd71;
                        8'd3: dyn_char = 8'd65;
                        8'd4: dyn_char = 8'd32;
                        8'd5: dyn_char = 8'd48 + h;
                        8'd6: dyn_char = 8'd48 + t;
                        8'd7: dyn_char = 8'd48 + o;
                        8'd8: dyn_char = 8'd32;
                        8'd9: dyn_char = 8'd68;
                        8'd10: dyn_char = 8'd69;
                        8'd11: dyn_char = 8'd71;
                        default: dyn_char = 8'd32;
                    endcase
                end
                8'd109: begin
                    case (idx)
                        8'd0: dyn_char = 8'd80;
                        8'd1: dyn_char = 8'd83;
                        8'd2: dyn_char = 8'd69;
                        8'd3: dyn_char = 8'd67;
                        8'd4: dyn_char = 8'd32;
                        8'd5: dyn_char = sector_char_a(pub_sec_i);
                        8'd6: dyn_char = sector_char_b(pub_sec_i);
                        8'd7: dyn_char = 8'd32;
                        8'd8: dyn_char = 8'd82;
                        8'd9: dyn_char = 8'd83;
                        8'd10: dyn_char = 8'd69;
                        8'd11: dyn_char = 8'd67;
                        8'd12: dyn_char = 8'd32;
                        8'd13: dyn_char = sector_char_a(raw_sec_i);
                        8'd14: dyn_char = sector_char_b(raw_sec_i);
                        default: dyn_char = 8'd32;
                    endcase
                end
                8'd110: begin
                    case (idx)
                        8'd0: dyn_char = 8'd72;
                        8'd1: dyn_char = 8'd70;
                        8'd2: dyn_char = 8'd82;
                        8'd3: dyn_char = 8'd69;
                        8'd4: dyn_char = 8'd83;
                        8'd5: dyn_char = 8'd72;
                        8'd6: dyn_char = 8'd32;
                        8'd7: dyn_char = head_fresh_i ? 8'd49 : 8'd48;
                        default: dyn_char = 8'd32;
                    endcase
                end
                8'd111: begin
                    case (idx)
                        8'd0: dyn_char = 8'd73;
                        8'd1: dyn_char = 8'd50;
                        8'd2: dyn_char = 8'd67;
                        8'd3: dyn_char = 8'd32;
                        8'd4: dyn_char = 8'd78;
                        8'd5: dyn_char = 8'd32;
                        8'd6: dyn_char = hex_char(nack_i[15:12]);
                        8'd7: dyn_char = hex_char(nack_i[11:8]);
                        8'd8: dyn_char = hex_char(nack_i[7:4]);
                        8'd9: dyn_char = hex_char(nack_i[3:0]);
                        default: dyn_char = 8'd32;
                    endcase
                end
                8'd112: begin
                    case (idx)
                        8'd0: dyn_char = 8'd84;
                        8'd1: dyn_char = 8'd73;
                        8'd2: dyn_char = 8'd77;
                        8'd3: dyn_char = 8'd69;
                        8'd4: dyn_char = 8'd79;
                        8'd5: dyn_char = 8'd85;
                        8'd6: dyn_char = 8'd84;
                        8'd7: dyn_char = 8'd32;
                        8'd8: dyn_char = hex_char(timeout_i[15:12]);
                        8'd9: dyn_char = hex_char(timeout_i[11:8]);
                        8'd10: dyn_char = hex_char(timeout_i[7:4]);
                        8'd11: dyn_char = hex_char(timeout_i[3:0]);
                        default: dyn_char = 8'd32;
                    endcase
                end
                8'd113: begin
                    case (idx)
                        8'd0: dyn_char = 8'd84;
                        8'd1: dyn_char = 8'd88;
                        8'd2: dyn_char = 8'd78;
                        8'd3: dyn_char = 8'd47;
                        8'd4: dyn_char = 8'd83;
                        8'd5: dyn_char = 8'd32;
                        8'd6: dyn_char = hex_char(txn_i[15:12]);
                        8'd7: dyn_char = hex_char(txn_i[11:8]);
                        8'd8: dyn_char = hex_char(txn_i[7:4]);
                        8'd9: dyn_char = hex_char(txn_i[3:0]);
                        default: dyn_char = 8'd32;
                    endcase
                end
                8'd114: begin
                    case (idx)
                        8'd0: dyn_char = 8'd68;
                        8'd1: dyn_char = 8'd83;
                        8'd2: dyn_char = 8'd69;
                        8'd3: dyn_char = 8'd81;
                        8'd4: dyn_char = 8'd32;
                        8'd5: dyn_char = hex_char(seq_i[15:12]);
                        8'd6: dyn_char = hex_char(seq_i[11:8]);
                        8'd7: dyn_char = hex_char(seq_i[7:4]);
                        8'd8: dyn_char = hex_char(seq_i[3:0]);
                        default: dyn_char = 8'd32;
                    endcase
                end
                default: dyn_char = 8'd32;
            endcase
        end
    endfunction

    function [7:0] ext_char;
        input [7:0] line_id;
        input [7:0] idx;
        input       ext_valid_i;
        input [7:0] ext_status_i;
        input [15:0] present_i;
        input [15:0] fault_i;
        input [15:0] delta_i;
        input [15:0] norm0_i;
        input [15:0] norm1_i;
        input [15:0] age_i;
        reg pair_missing;
        reg mag_disagree;
        reg norm_fault;
        reg raw_fault;
        begin
            pair_missing = fault_i[`EXT_FLG_MAG_PAIR_MISSING_BIT];
            mag_disagree = fault_i[`EXT_FLG_MAG_DISAGREE_BIT];
            norm_fault   = fault_i[`EXT_FLG_MAG0_NORM_OOR_BIT] |
                           fault_i[`EXT_FLG_MAG1_NORM_OOR_BIT] |
                           fault_i[`EXT_FLG_MAG_NORM_MISMATCH_BIT];
            raw_fault    = fault_i[`EXT_FLG_RAW_STATUS_ERR_BIT];
            ext_char = 8'd32;
            case (line_id)
                8'd200: begin
                    case (idx)
                        8'd0:  ext_char = 8'd82; // R
                        8'd1:  ext_char = 8'd69; // E
                        8'd2:  ext_char = 8'd68; // D
                        8'd3:  ext_char = 8'd32;
                        8'd4:  ext_char = 8'd77; // M
                        8'd5:  ext_char = 8'd65; // A
                        8'd6:  ext_char = 8'd71; // G
                        8'd7:  ext_char = 8'd32;
                        8'd8:  ext_char = 8'd69; // E
                        8'd9:  ext_char = 8'd86; // V
                        8'd10: ext_char = 8'd73; // I
                        8'd11: ext_char = 8'd68; // D
                        8'd12: ext_char = 8'd69; // E
                        8'd13: ext_char = 8'd78; // N
                        8'd14: ext_char = 8'd67; // C
                        8'd15: ext_char = 8'd69; // E
                        default: ext_char = 8'd32;
                    endcase
                end
                8'd201: begin
                    case (idx)
                        8'd0:  ext_char = 8'd77; // M
                        8'd1:  ext_char = 8'd48; // 0
                        8'd2:  ext_char = 8'd80; // P
                        8'd3:  ext_char = present_i[`EXT_PRESENT_MAG0_BIT] ? 8'd49 : 8'd48;
                        8'd4:  ext_char = 8'd32;
                        8'd5:  ext_char = 8'd77; // M
                        8'd6:  ext_char = 8'd49; // 1
                        8'd7:  ext_char = 8'd80; // P
                        8'd8:  ext_char = present_i[`EXT_PRESENT_MAG1_BIT] ? 8'd49 : 8'd48;
                        8'd9:  ext_char = 8'd32;
                        8'd10: ext_char = 8'd83; // S
                        8'd11: ext_char = 8'd84; // T
                        8'd12: ext_char = hex_char(ext_status_i[7:4]);
                        8'd13: ext_char = hex_char(ext_status_i[3:0]);
                        default: ext_char = 8'd32;
                    endcase
                end
                8'd202: begin
                    case (idx)
                        8'd0:  ext_char = 8'd68; // D
                        8'd1:  ext_char = hex_char(delta_i[15:12]);
                        8'd2:  ext_char = hex_char(delta_i[11:8]);
                        8'd3:  ext_char = hex_char(delta_i[7:4]);
                        8'd4:  ext_char = hex_char(delta_i[3:0]);
                        8'd5:  ext_char = 8'd32;
                        8'd6:  ext_char = 8'd70; // F
                        8'd7:  ext_char = hex_char(fault_i[15:12]);
                        8'd8:  ext_char = hex_char(fault_i[11:8]);
                        8'd9:  ext_char = hex_char(fault_i[7:4]);
                        8'd10: ext_char = hex_char(fault_i[3:0]);
                        8'd11: ext_char = 8'd32;
                        8'd12: ext_char = 8'd65; // A
                        8'd13: ext_char = 8'd71; // G
                        8'd14: ext_char = hex_char(age_i[15:12]);
                        8'd15: ext_char = hex_char(age_i[11:8]);
                        8'd16: ext_char = hex_char(age_i[7:4]);
                        8'd17: ext_char = hex_char(age_i[3:0]);
                        default: ext_char = 8'd32;
                    endcase
                end
                8'd203: begin
                    case (idx)
                        8'd0:  ext_char = 8'd78; // N
                        8'd1:  ext_char = 8'd48; // 0
                        8'd2:  ext_char = hex_char(norm0_i[15:12]);
                        8'd3:  ext_char = hex_char(norm0_i[11:8]);
                        8'd4:  ext_char = hex_char(norm0_i[7:4]);
                        8'd5:  ext_char = hex_char(norm0_i[3:0]);
                        8'd6:  ext_char = 8'd32;
                        8'd7:  ext_char = 8'd78; // N
                        8'd8:  ext_char = 8'd49; // 1
                        8'd9:  ext_char = hex_char(norm1_i[15:12]);
                        8'd10: ext_char = hex_char(norm1_i[11:8]);
                        8'd11: ext_char = hex_char(norm1_i[7:4]);
                        8'd12: ext_char = hex_char(norm1_i[3:0]);
                        default: ext_char = 8'd32;
                    endcase
                end
                8'd204: begin
                    case (idx)
                        8'd0: ext_char = 8'd65; // A
                        8'd1: ext_char = 8'd71; // G
                        8'd2: ext_char = hex_char(age_i[15:12]);
                        8'd3: ext_char = hex_char(age_i[11:8]);
                        8'd4: ext_char = hex_char(age_i[7:4]);
                        8'd5: ext_char = hex_char(age_i[3:0]);
                        default: ext_char = 8'd32;
                    endcase
                end
                8'd205: begin
                    if (!ext_valid_i) begin
                        case (idx)
                            8'd0:  ext_char = 8'd69; // E
                            8'd1:  ext_char = 8'd88; // X
                            8'd2:  ext_char = 8'd84; // T
                            8'd3:  ext_char = 8'd32;
                            8'd4:  ext_char = 8'd78; // N
                            8'd5:  ext_char = 8'd79; // O
                            8'd6:  ext_char = 8'd84; // T
                            8'd7:  ext_char = 8'd32;
                            8'd8:  ext_char = 8'd82; // R
                            8'd9:  ext_char = 8'd68; // D
                            8'd10: ext_char = 8'd89; // Y
                            default: ext_char = 8'd32;
                        endcase
                    end else if (mag_disagree) begin
                        case (idx)
                            8'd0:  ext_char = 8'd77; // M
                            8'd1:  ext_char = 8'd65; // A
                            8'd2:  ext_char = 8'd71; // G
                            8'd3:  ext_char = 8'd32;
                            8'd4:  ext_char = 8'd68; // D
                            8'd5:  ext_char = 8'd73; // I
                            8'd6:  ext_char = 8'd83; // S
                            8'd7:  ext_char = 8'd65; // A
                            8'd8:  ext_char = 8'd71; // G
                            8'd9:  ext_char = 8'd82; // R
                            8'd10: ext_char = 8'd69; // E
                            8'd11: ext_char = 8'd69; // E
                            default: ext_char = 8'd32;
                        endcase
                    end else if (norm_fault) begin
                        case (idx)
                            8'd0: ext_char = 8'd78; // N
                            8'd1: ext_char = 8'd79; // O
                            8'd2: ext_char = 8'd82; // R
                            8'd3: ext_char = 8'd77; // M
                            8'd4: ext_char = 8'd32;
                            8'd5: ext_char = 8'd70; // F
                            8'd6: ext_char = 8'd65; // A
                            8'd7: ext_char = 8'd85; // U
                            8'd8: ext_char = 8'd76; // L
                            8'd9: ext_char = 8'd84; // T
                            default: ext_char = 8'd32;
                        endcase
                    end else if (raw_fault) begin
                        case (idx)
                            8'd0: ext_char = 8'd82; // R
                            8'd1: ext_char = 8'd65; // A
                            8'd2: ext_char = 8'd87; // W
                            8'd3: ext_char = 8'd32;
                            8'd4: ext_char = 8'd83; // S
                            8'd5: ext_char = 8'd84; // T
                            8'd6: ext_char = 8'd65; // A
                            8'd7: ext_char = 8'd84; // T
                            8'd8: ext_char = 8'd85; // U
                            8'd9: ext_char = 8'd83; // S
                            default: ext_char = 8'd32;
                        endcase
                    end else if (pair_missing) begin
                        case (idx)
                            8'd0:  ext_char = 8'd80; // P
                            8'd1:  ext_char = 8'd65; // A
                            8'd2:  ext_char = 8'd73; // I
                            8'd3:  ext_char = 8'd82; // R
                            8'd4:  ext_char = 8'd32;
                            8'd5:  ext_char = 8'd77; // M
                            8'd6:  ext_char = 8'd73; // I
                            8'd7:  ext_char = 8'd83; // S
                            8'd8:  ext_char = 8'd83; // S
                            8'd9:  ext_char = 8'd73; // I
                            8'd10: ext_char = 8'd78; // N
                            8'd11: ext_char = 8'd71; // G
                            default: ext_char = 8'd32;
                        endcase
                    end else begin
                        case (idx)
                            8'd0: ext_char = 8'd80; // P
                            8'd1: ext_char = 8'd65; // A
                            8'd2: ext_char = 8'd73; // I
                            8'd3: ext_char = 8'd82; // R
                            8'd4: ext_char = 8'd32;
                            8'd5: ext_char = 8'd79; // O
                            8'd6: ext_char = 8'd75; // K
                            default: ext_char = 8'd32;
                        endcase
                    end
                end
                default: ext_char = 8'd32;
            endcase
        end
    endfunction

    function [7:0] mag1_char;
        input [7:0] line_id;
        input [7:0] idx;
        input [15:0] seq_i;
        input        valid_i;
        input [7:0]  status_i;
        input [15:0] x_i;
        input [15:0] y_i;
        input [15:0] z_i;
        input [15:0] age_i;
        input        seq_aligned_i;
        input        disagree_i;
        input [3:0]  sector_delta_i;
        input [15:0] norm_delta_i;
        input [15:0] iron_i;
        input [7:0]  cal_i;
        input [7:0]  source_i;
        input [15:0] checksum_i;
        begin
            mag1_char = 8'd32;
            case (line_id)
                8'd210: begin
                    case (idx)
                        8'd0: mag1_char = 8'd77; // M
                        8'd1: mag1_char = 8'd49; // 1
                        8'd2: mag1_char = 8'd88; // X
                        8'd3: mag1_char = 8'd32;
                        8'd4: mag1_char = hex_char(x_i[15:12]);
                        8'd5: mag1_char = hex_char(x_i[11:8]);
                        8'd6: mag1_char = hex_char(x_i[7:4]);
                        8'd7: mag1_char = hex_char(x_i[3:0]);
                        default: mag1_char = 8'd32;
                    endcase
                end
                8'd211: begin
                    case (idx)
                        8'd0: mag1_char = 8'd77; // M
                        8'd1: mag1_char = 8'd49; // 1
                        8'd2: mag1_char = 8'd89; // Y
                        8'd3: mag1_char = 8'd32;
                        8'd4: mag1_char = hex_char(y_i[15:12]);
                        8'd5: mag1_char = hex_char(y_i[11:8]);
                        8'd6: mag1_char = hex_char(y_i[7:4]);
                        8'd7: mag1_char = hex_char(y_i[3:0]);
                        default: mag1_char = 8'd32;
                    endcase
                end
                8'd212: begin
                    case (idx)
                        8'd0: mag1_char = 8'd77; // M
                        8'd1: mag1_char = 8'd49; // 1
                        8'd2: mag1_char = 8'd90; // Z
                        8'd3: mag1_char = 8'd32;
                        8'd4: mag1_char = hex_char(z_i[15:12]);
                        8'd5: mag1_char = hex_char(z_i[11:8]);
                        8'd6: mag1_char = hex_char(z_i[7:4]);
                        8'd7: mag1_char = hex_char(z_i[3:0]);
                        default: mag1_char = 8'd32;
                    endcase
                end
                8'd213: begin
                    case (idx)
                        8'd0: mag1_char = 8'd77; // M
                        8'd1: mag1_char = 8'd49; // 1
                        8'd2: mag1_char = 8'd32;
                        8'd3: mag1_char = 8'd86; // V
                        8'd4: mag1_char = valid_i ? 8'd49 : 8'd48;
                        8'd5: mag1_char = 8'd32;
                        8'd6: mag1_char = 8'd83; // S
                        8'd7: mag1_char = hex_char(status_i[7:4]);
                        8'd8: mag1_char = hex_char(status_i[3:0]);
                        default: mag1_char = 8'd32;
                    endcase
                end
                8'd214: begin
                    case (idx)
                        8'd0:  mag1_char = 8'd81; // Q
                        8'd1:  mag1_char = hex_char(seq_i[15:12]);
                        8'd2:  mag1_char = hex_char(seq_i[11:8]);
                        8'd3:  mag1_char = hex_char(seq_i[7:4]);
                        8'd4:  mag1_char = hex_char(seq_i[3:0]);
                        8'd5:  mag1_char = 8'd32;
                        8'd6:  mag1_char = 8'd65; // A
                        8'd7:  mag1_char = hex_char(age_i[15:12]);
                        8'd8:  mag1_char = hex_char(age_i[11:8]);
                        8'd9:  mag1_char = hex_char(age_i[7:4]);
                        8'd10: mag1_char = hex_char(age_i[3:0]);
                        default: mag1_char = 8'd32;
                    endcase
                end
                8'd215: begin
                    case (idx)
                        8'd0:  mag1_char = 8'd65; // A
                        8'd1:  mag1_char = 8'd76; // L
                        8'd2:  mag1_char = seq_aligned_i ? 8'd49 : 8'd48;
                        8'd3:  mag1_char = 8'd32;
                        8'd4:  mag1_char = 8'd83; // S
                        8'd5:  mag1_char = 8'd68; // D
                        8'd6:  mag1_char = hex_char(sector_delta_i[3:0]);
                        8'd7:  mag1_char = 8'd32;
                        8'd8:  mag1_char = 8'd68; // D
                        8'd9:  mag1_char = 8'd71; // G
                        8'd10: mag1_char = disagree_i ? 8'd49 : 8'd48;
                        default: mag1_char = 8'd32;
                    endcase
                end
                8'd216: begin
                    case (idx)
                        8'd0:  mag1_char = 8'd78; // N
                        8'd1:  mag1_char = 8'd68; // D
                        8'd2:  mag1_char = hex_char(norm_delta_i[15:12]);
                        8'd3:  mag1_char = hex_char(norm_delta_i[11:8]);
                        8'd4:  mag1_char = hex_char(norm_delta_i[7:4]);
                        8'd5:  mag1_char = hex_char(norm_delta_i[3:0]);
                        8'd6:  mag1_char = 8'd32;
                        8'd7:  mag1_char = 8'd73; // I
                        8'd8:  mag1_char = 8'd82; // R
                        8'd9:  mag1_char = hex_char(iron_i[15:12]);
                        8'd10: mag1_char = hex_char(iron_i[11:8]);
                        8'd11: mag1_char = hex_char(iron_i[7:4]);
                        8'd12: mag1_char = hex_char(iron_i[3:0]);
                        default: mag1_char = 8'd32;
                    endcase
                end
                8'd217: begin
                    case (idx)
                        8'd0:  mag1_char = 8'd67; // C
                        8'd1:  mag1_char = 8'd65; // A
                        8'd2:  mag1_char = 8'd76; // L
                        8'd3:  mag1_char = hex_char(cal_i[7:4]);
                        8'd4:  mag1_char = hex_char(cal_i[3:0]);
                        8'd5:  mag1_char = 8'd32;
                        8'd6:  mag1_char = 8'd83; // S
                        8'd7:  mag1_char = 8'd82; // R
                        8'd8:  mag1_char = 8'd67; // C
                        8'd9:  mag1_char = hex_char(source_i[7:4]);
                        8'd10: mag1_char = hex_char(source_i[3:0]);
                        default: mag1_char = 8'd32;
                    endcase
                end
                8'd218: begin
                    case (idx)
                        8'd0: mag1_char = 8'd67; // C
                        8'd1: mag1_char = 8'd75; // K
                        8'd2: mag1_char = hex_char(checksum_i[15:12]);
                        8'd3: mag1_char = hex_char(checksum_i[11:8]);
                        8'd4: mag1_char = hex_char(checksum_i[7:4]);
                        8'd5: mag1_char = hex_char(checksum_i[3:0]);
                        default: mag1_char = 8'd32;
                    endcase
                end
                default: mag1_char = 8'd32;
            endcase
        end
    endfunction

    function text_line_ext_hit;
        input [11:0] x;
        input [11:0] y;
        input integer x0;
        input integer y0;
        input integer len;
        input [7:0] line_id;
        input integer scale2;
        input       ext_valid_i;
        input [7:0] ext_status_i;
        input [15:0] present_i;
        input [15:0] fault_i;
        input [15:0] delta_i;
        input [15:0] norm0_i;
        input [15:0] norm1_i;
        input [15:0] age_i;
        integer rx;
        integer ry;
        integer ci;
        integer gc;
        integer gr;
        reg [7:0] ch;
        reg [4:0] glyph;
        begin
            text_line_ext_hit = 1'b0;
            if ((x >= x0) && (y >= y0)) begin
                rx = x - x0;
                ry = y - y0;
                if (scale2 != 0) begin
                    if ((rx < (len << 4)) && (ry < 16)) begin
                        ci = rx >> 4;
                        gc = (rx & 15) >> 1;
                        gr = (ry & 15) >> 1;
                        ch = ext_char(line_id, ci[7:0], ext_valid_i, ext_status_i, present_i, fault_i, delta_i, norm0_i, norm1_i, age_i);
                        glyph = glyph5x7(ch, gr[2:0]);
                        if ((gc < 5) && (gr < 7))
                            text_line_ext_hit = glyph[4-gc];
                    end
                end else begin
                    if ((rx < (len << 3)) && (ry < 8)) begin
                        ci = rx >> 3;
                        gc = rx & 7;
                        gr = ry & 7;
                        ch = ext_char(line_id, ci[7:0], ext_valid_i, ext_status_i, present_i, fault_i, delta_i, norm0_i, norm1_i, age_i);
                        glyph = glyph5x7(ch, gr[2:0]);
                        if ((gc < 5) && (gr < 7))
                            text_line_ext_hit = glyph[4-gc];
                    end
                end
            end
        end
    endfunction

    function text_line_mag1_hit;
        input [11:0] x;
        input [11:0] y;
        input integer x0;
        input integer y0;
        input integer len;
        input [7:0] line_id;
        input integer scale2;
        input [15:0] seq_i;
        input        valid_i;
        input [7:0]  status_i;
        input [15:0] x_i;
        input [15:0] y_i;
        input [15:0] z_i;
        input [15:0] age_i;
        input        seq_aligned_i;
        input        disagree_i;
        input [3:0]  sector_delta_i;
        input [15:0] norm_delta_i;
        input [15:0] iron_i;
        input [7:0]  cal_i;
        input [7:0]  source_i;
        input [15:0] checksum_i;
        integer rx;
        integer ry;
        integer ci;
        integer gc;
        reg [7:0] ch;
        reg [4:0] glyph;
        begin
            text_line_mag1_hit = 1'b0;
            rx = x - x0;
            ry = y - y0;
            if (!scale2) begin
                if ((rx >= 0) && (ry >= 0) && (ry < 8)) begin
                    ci = rx / 6;
                    if (ci < len) begin
                        gc = rx - (ci * 6);
                        if (gc < 5)
                            begin
                                ch = mag1_char(line_id, ci[7:0], seq_i, valid_i,
                                               status_i, x_i, y_i, z_i, age_i,
                                               seq_aligned_i, disagree_i,
                                               sector_delta_i, norm_delta_i, iron_i,
                                               cal_i, source_i, checksum_i);
                                glyph = glyph5x7(ch, ry[2:0]);
                                text_line_mag1_hit = glyph[4-gc];
                            end
                    end
                end
            end else begin
                if ((rx >= 0) && (ry >= 0) && (ry < 16)) begin
                    ci = rx / 12;
                    if (ci < len) begin
                        gc = (rx - (ci * 12)) >> 1;
                        if (gc < 5)
                            begin
                                ch = mag1_char(line_id, ci[7:0], seq_i, valid_i,
                                               status_i, x_i, y_i, z_i, age_i,
                                               seq_aligned_i, disagree_i,
                                               sector_delta_i, norm_delta_i, iron_i,
                                               cal_i, source_i, checksum_i);
                                glyph = glyph5x7(ch, ry[3:1]);
                                text_line_mag1_hit = glyph[4-gc];
                            end
                    end
                end
            end
        end
    endfunction

    function text_line_static_hit;
        input [11:0] x;
        input [11:0] y;
        input integer x0;
        input integer y0;
        input integer len;
        input [7:0] line_id;
        input integer scale2;
        integer rx;
        integer ry;
        integer ci;
        integer gc;
        integer gr;
        reg [7:0] ch;
        reg [4:0] glyph;
        begin
            text_line_static_hit = 1'b0;
            if ((x >= x0) && (y >= y0)) begin
                rx = x - x0;
                ry = y - y0;
                if (scale2 != 0) begin
                    if ((rx < (len << 4)) && (ry < 16)) begin
                        ci = rx >> 4;
                        gc = (rx & 15) >> 1;
                        gr = (ry & 15) >> 1;
                        ch = static_char(line_id, ci[7:0]);
                        glyph = glyph5x7(ch, gr[2:0]);
                        if ((gc < 5) && (gr < 7))
                            text_line_static_hit = glyph[4-gc];
                    end
                end else begin
                    if ((rx < (len << 3)) && (ry < 8)) begin
                        ci = rx >> 3;
                        gc = rx & 7;
                        gr = ry & 7;
                        ch = static_char(line_id, ci[7:0]);
                        glyph = glyph5x7(ch, gr[2:0]);
                        if ((gc < 5) && (gr < 7))
                            text_line_static_hit = glyph[4-gc];
                    end
                end
            end
        end
    endfunction

    function text_line_dyn_hit;
        input [11:0] x;
        input [11:0] y;
        input integer x0;
        input integer y0;
        input integer len;
        input [7:0] line_id;
        input integer scale2;
        input [8:0] hdg_deg_i;
        input [9:0] age_i;
        input [15:0] seq_i;
        input [7:0] stat_i;
        input [15:0] mx_i;
        input [15:0] my_i;
        input [15:0] mz_i;
        input [15:0] norm_i;
        input [2:0] pub_sec_i;
        input [2:0] raw_sec_i;
        input [15:0] nack_i;
        input [15:0] timeout_i;
        input [15:0] txn_i;
        input head_fresh_i;
        integer rx;
        integer ry;
        integer ci;
        integer gc;
        integer gr;
        reg [7:0] ch;
        reg [4:0] glyph;
        begin
            text_line_dyn_hit = 1'b0;
            if ((x >= x0) && (y >= y0)) begin
                rx = x - x0;
                ry = y - y0;
                if (scale2 != 0) begin
                    if ((rx < (len << 4)) && (ry < 16)) begin
                        ci = rx >> 4;
                        gc = (rx & 15) >> 1;
                        gr = (ry & 15) >> 1;
                        ch = dyn_char(line_id, ci[7:0], hdg_deg_i, age_i, seq_i, stat_i, mx_i, my_i, mz_i, norm_i, pub_sec_i, raw_sec_i, nack_i, timeout_i, txn_i, head_fresh_i);
                        glyph = glyph5x7(ch, gr[2:0]);
                        if ((gc < 5) && (gr < 7))
                            text_line_dyn_hit = glyph[4-gc];
                    end
                end else begin
                    if ((rx < (len << 3)) && (ry < 8)) begin
                        ci = rx >> 3;
                        gc = rx & 7;
                        gr = ry & 7;
                        ch = dyn_char(line_id, ci[7:0], hdg_deg_i, age_i, seq_i, stat_i, mx_i, my_i, mz_i, norm_i, pub_sec_i, raw_sec_i, nack_i, timeout_i, txn_i, head_fresh_i);
                        glyph = glyph5x7(ch, gr[2:0]);
                        if ((gc < 5) && (gr < 7))
                            text_line_dyn_hit = glyph[4-gc];
                    end
                end
            end
        end
    endfunction

    function rect_frame;
        input [11:0] x;
        input [11:0] y;
        input integer x0;
        input integer y0;
        input integer x1;
        input integer y1;
        begin
            rect_frame = (((x == x0) || (x == x1)) && (y >= y0) && (y <= y1)) ||
                         (((y == y0) || (y == y1)) && (x >= x0) && (x <= x1));
        end
    endfunction

    function rect_fill;
        input [11:0] x;
        input [11:0] y;
        input integer x0;
        input integer y0;
        input integer x1;
        input integer y1;
        begin
            rect_fill = (x >= x0) && (x <= x1) && (y >= y0) && (y <= y1);
        end
    endfunction

    function circle_ring;
        input [11:0] x;
        input [11:0] y;
        input integer cx;
        input integer cy;
        input integer r;
        integer dx;
        integer dy;
        integer d2;
        begin
            dx = x - cx;
            dy = y - cy;
            d2 = dx*dx + dy*dy;
            circle_ring = (d2 >= ((r-1)*(r-1))) && (d2 <= ((r+1)*(r+1)));
        end
    endfunction

    function line_near;
        input [11:0] x;
        input [11:0] y;
        input integer x0;
        input integer y0;
        input integer dx;
        input integer dy;
        integer vx;
        integer vy;
        integer dot;
        integer len2;
        integer cross;
        integer tol;
        begin
            vx = x - x0;
            vy = y - y0;
            dot = vx*dx + vy*dy;
            len2 = dx*dx + dy*dy;
            cross = vx*dy - vy*dx;
            if (cross < 0)
                cross = -cross;
            tol = 120;
            line_near = (dot >= 0) && (dot <= len2) && (cross <= tol);
        end
    endfunction

    function [8:0] hdg_deg_from_mdeg;
        input [31:0] mdeg;
        reg [31:0] d;
        begin
            d = mdeg / 32'd1000;
            if (d >= 32'd360)
                hdg_deg_from_mdeg = 9'd359;
            else
                hdg_deg_from_mdeg = d[8:0];
        end
    endfunction

    function [2:0] pub_sector8_from_deg;
        input [8:0] deg;
        reg [8:0] s;
        begin
            s = (deg + 9'd22) / 9'd45;
            if (s >= 9'd8)
                pub_sector8_from_deg = 3'd0;
            else
                pub_sector8_from_deg = s[2:0];
        end
    endfunction

    function [4:0] heading_sector16;
        input [8:0] deg;
        reg [9:0] s;
        begin
            s = (deg + 9'd11) / 9'd22;
            if (s >= 10'd16)
                heading_sector16 = 5'd0;
            else
                heading_sector16 = s[4:0];
        end
    endfunction

    function signed [8:0] sec16_dx;
        input [4:0] s;
        begin
            case (s)
                5'd0: sec16_dx = 9'sd72;
                5'd1: sec16_dx = 9'sd67;
                5'd2: sec16_dx = 9'sd51;
                5'd3: sec16_dx = 9'sd28;
                5'd4: sec16_dx = 9'sd0;
                5'd5: sec16_dx = -9'sd28;
                5'd6: sec16_dx = -9'sd51;
                5'd7: sec16_dx = -9'sd67;
                5'd8: sec16_dx = -9'sd72;
                5'd9: sec16_dx = -9'sd67;
                5'd10: sec16_dx = -9'sd51;
                5'd11: sec16_dx = -9'sd28;
                5'd12: sec16_dx = 9'sd0;
                5'd13: sec16_dx = 9'sd28;
                5'd14: sec16_dx = 9'sd51;
                5'd15: sec16_dx = 9'sd67;
                default: sec16_dx = 9'sd72;
            endcase
        end
    endfunction

    function signed [8:0] sec16_dy;
        input [4:0] s;
        begin
            case (s)
                5'd0: sec16_dy = 9'sd0;
                5'd1: sec16_dy = -9'sd28;
                5'd2: sec16_dy = -9'sd51;
                5'd3: sec16_dy = -9'sd67;
                5'd4: sec16_dy = -9'sd72;
                5'd5: sec16_dy = -9'sd67;
                5'd6: sec16_dy = -9'sd51;
                5'd7: sec16_dy = -9'sd28;
                5'd8: sec16_dy = 9'sd0;
                5'd9: sec16_dy = 9'sd28;
                5'd10: sec16_dy = 9'sd51;
                5'd11: sec16_dy = 9'sd67;
                5'd12: sec16_dy = 9'sd72;
                5'd13: sec16_dy = 9'sd67;
                5'd14: sec16_dy = 9'sd51;
                5'd15: sec16_dy = 9'sd28;
                default: sec16_dy = 9'sd0;
            endcase
        end
    endfunction

    wire [8:0] hdg_deg_w;
    assign hdg_deg_w = hdg_deg_from_mdeg(der_heading_mdeg_pix);
    wire [2:0] pub_sector8_w;
    wire [2:0] raw_sector8_w;
    assign pub_sector8_w = pub_sector8_from_deg(hdg_deg_w);
    assign raw_sector8_w = raw_sector8(mx_pix, my_pix);
    wire [3:0] sector_diff_w;
    assign sector_diff_w = sector_diff8(pub_sector8_w, raw_sector8_w);
    wire heading_crosscheck_ok;
    assign heading_crosscheck_ok = (sector_diff_w <= 4'd1);

    wire mag_good_w;
    wire mag_bad_w;
    wire mag_stale_w;
    assign mag_bad_w   = (!mag_valid_pix) || (mag_status_pix != 8'h00) || (!der_valid_pix) || (der_status_pix != 8'h00);
    assign mag_stale_w = (!der_head_fresh_pix) || (mag_age_ms_pix > 16'd250) || (der_mag_seq_ref_pix != mag_seq_pix);
    assign mag_good_w  = (!mag_bad_w) && (!mag_stale_w);

    wire [15:0] abs_mx_w = abs16(mx_pix);
    wire [15:0] abs_my_w = abs16(my_pix);
    wire [15:0] abs_mz_w = abs16(mz_pix);
    wire [15:0] max_xy_w = max16(abs_mx_w, abs_my_w);
    wire [15:0] max_xyz_w = max16(max_xy_w, abs_mz_w);
    wire [17:0] norm_sum_w = {2'b00, abs_mx_w} + {2'b00, abs_my_w} + {2'b00, abs_mz_w};
    wire [15:0] norm_approx_w = (norm_sum_w[17:8] > 10'd255) ? 16'hFFFF : {6'd0, norm_sum_w[17:8]};
    wire [8:0] norm_bar_w = (norm_sum_w[17:8] > 10'd240) ? 9'd240 : {1'b0, norm_sum_w[15:8]};

    wire ext_mag0_present_w = ext_present_flags_pix[`EXT_PRESENT_MAG0_BIT];
    wire ext_mag1_present_w = ext_present_flags_pix[`EXT_PRESENT_MAG1_BIT];
    wire ext_mag_pair_missing_w = ext_fault_flags_pix[`EXT_FLG_MAG_PAIR_MISSING_BIT];
    wire ext_mag_disagree_w     = ext_fault_flags_pix[`EXT_FLG_MAG_DISAGREE_BIT];
    wire ext_mag0_norm_oor_w    = ext_fault_flags_pix[`EXT_FLG_MAG0_NORM_OOR_BIT];
    wire ext_mag1_norm_oor_w    = ext_fault_flags_pix[`EXT_FLG_MAG1_NORM_OOR_BIT];
    wire ext_mag_norm_mismatch_w= ext_fault_flags_pix[`EXT_FLG_MAG_NORM_MISMATCH_BIT];
    wire ext_mag_norm_fault_w   = ext_mag0_norm_oor_w | ext_mag1_norm_oor_w | ext_mag_norm_mismatch_w;
    wire ext_raw_status_fault_w = ext_fault_flags_pix[`EXT_FLG_RAW_STATUS_ERR_BIT];
    wire ext_mag_hard_fault_w   = ext_mag_disagree_w | ext_mag_disagreement_pix |
                                  ext_mag_norm_fault_w | ext_raw_status_fault_w;
    wire mag1_synthetic_w       = ext_mag_source_flags_pix[`EXT_SRC_SYNTHETIC_BIT];
    wire ext_mag_good_w         = ext_valid_pix &&
                                  (ext_status_pix == `ST_OK) &&
                                  ext_mag0_present_w &&
                                  ext_mag1_present_w &&
                                  (!ext_mag_pair_missing_w) &&
                                  (!ext_mag_hard_fault_w);
    wire [8:0] ext_delta_bar_w = (ext_mag_delta_l1_pix > 16'd240) ?
                                  9'd240 : {1'b0, ext_mag_delta_l1_pix[7:0]};
    wire [8:0] ext_norm0_bar_w = (ext_mag_norm_primary_pix[15:8] > 8'd240) ?
                                  9'd240 : {1'b0, ext_mag_norm_primary_pix[15:8]};
    wire [8:0] ext_norm1_bar_w = (ext_mag_norm_secondary_pix[15:8] > 8'd240) ?
                                  9'd240 : {1'b0, ext_mag_norm_secondary_pix[15:8]};

    //--------------------------------------------------------------------------
    // Pixel-domain MX/MY trail. The plot is intentionally small and bounded:
    // 16 retained samples are enough to show rotation, axis swaps, sign errors,
    // stale samples, and local magnetic distortion without BRAM.
    //--------------------------------------------------------------------------
    reg signed [8:0] hist_x [0:15];
    reg signed [8:0] hist_y [0:15];
    reg [3:0] hist_wr;
    reg [15:0] mag_seq_seen;
    integer hi;
    always @(posedge pix_clk) begin
        if (pix_rst) begin
            hist_wr <= 4'd0;
            mag_seq_seen <= 16'd0;
            for (hi = 0; hi < 16; hi = hi + 1) begin
                hist_x[hi] <= 9'sd0;
                hist_y[hi] <= 9'sd0;
            end
        end else if (dst_update_pulse) begin
            if (mag_seq_pix != mag_seq_seen) begin
                mag_seq_seen <= mag_seq_pix;
                hist_x[hist_wr] <= scale_axis(mx_pix);
                hist_y[hist_wr] <= scale_axis(my_pix);
                hist_wr <= hist_wr + 4'd1;
            end
        end
    end

    reg [11:0] page_rgb_r;
    reg [11:0] text_rgb_r;
    reg text_on_r;
    reg trail_hit_r;
    reg current_pt_hit_r;
    reg current_m1_pt_hit_r;
    reg [4:0] sec16_r;
    reg signed [8:0] arrow_dx_r;
    reg signed [8:0] arrow_dy_r;
    reg signed [8:0] cur_x_s;
    reg signed [8:0] cur_y_s;
    reg signed [8:0] cur1_x_s;
    reg signed [8:0] cur1_y_s;
    integer dx_i;
    integer dy_i;
    integer d2_i;
    integer px_i;
    integer py_i;
    integer trail_i;

    always @* begin
        page_rgb_r = C_BLACK;
        text_on_r = 1'b0;
        text_rgb_r = C_WHITE;
        trail_hit_r = 1'b0;
        current_pt_hit_r = 1'b0;
        current_m1_pt_hit_r = 1'b0;
        cur_x_s = scale_axis(mx_pix);
        cur_y_s = scale_axis(my_pix);
        cur1_x_s = scale_axis(m1x_pix);
        cur1_y_s = scale_axis(m1y_pix);
        sec16_r = heading_sector16(hdg_deg_w);
        arrow_dx_r = sec16_dx(sec16_r);
        arrow_dy_r = sec16_dy(sec16_r);

        if (active_video) begin
            // Background grid with a dim scientific-instrument look.
            if ((pix_x[5:0] == 6'd0) || (pix_y[5:0] == 6'd0))
                page_rgb_r = C_GRID;
            else if ((pix_x[4:0] == 5'd0) || (pix_y[4:0] == 5'd0))
                page_rgb_r = 12'h011;

            // Panel frames.
            if (rect_frame(pix_x, pix_y, 16, 44, 206, 264) ||
                rect_frame(pix_x, pix_y, 218, 44, 422, 264) ||
                rect_frame(pix_x, pix_y, 434, 44, 624, 264) ||
                rect_frame(pix_x, pix_y, 16, 278, 312, 458) ||
                rect_frame(pix_x, pix_y, 326, 278, 624, 458))
                page_rgb_r = C_CYAN;

            // Compass rose: left panel. 0 degrees is +MX/right, 90 is +MY/up.
            if (circle_ring(pix_x, pix_y, 111, 162, 74))
                page_rgb_r = C_WHITE;
            if (((pix_y == 162) && (pix_x >= 42) && (pix_x <= 180)) ||
                ((pix_x == 111) && (pix_y >= 93) && (pix_y <= 231)))
                page_rgb_r = C_DIM;
            if (line_near(pix_x, pix_y, 111, 162, arrow_dx_r, arrow_dy_r))
                page_rgb_r = mag_good_w ? C_GREEN : (mag_bad_w ? C_RED : C_YELLOW);
            if (rect_fill(pix_x, pix_y, 108, 159, 114, 165))
                page_rgb_r = C_WHITE;

            // Live MX/MY plot: center panel.
            if (rect_frame(pix_x, pix_y, 238, 76, 402, 240))
                page_rgb_r = C_DIM;
            if (((pix_y == 158) && (pix_x >= 238) && (pix_x <= 402)) ||
                ((pix_x == 320) && (pix_y >= 76) && (pix_y <= 240)))
                page_rgb_r = C_DIM;
            // Trail samples are small green dots; current sample is larger.
            for (trail_i = 0; trail_i < 16; trail_i = trail_i + 1) begin
                px_i = 320 + hist_x[trail_i];
                py_i = 158 - hist_y[trail_i];
                if ((pix_x >= (px_i - 1)) && (pix_x <= (px_i + 1)) &&
                    (pix_y >= (py_i - 1)) && (pix_y <= (py_i + 1)))
                    trail_hit_r = 1'b1;
            end
            px_i = 320 + cur_x_s;
            py_i = 158 - cur_y_s;
            if ((pix_x >= (px_i - 3)) && (pix_x <= (px_i + 3)) &&
                (pix_y >= (py_i - 3)) && (pix_y <= (py_i + 3)))
                current_pt_hit_r = 1'b1;
            px_i = 320 + cur1_x_s;
            py_i = 158 - cur1_y_s;
            if (mag1_valid_pix &&
                (pix_x >= (px_i - 2)) && (pix_x <= (px_i + 2)) &&
                (pix_y >= (py_i - 2)) && (pix_y <= (py_i + 2)))
                current_m1_pt_hit_r = 1'b1;
            if (trail_hit_r)
                page_rgb_r = C_GREEN;
            if (mag1_valid_pix && line_near(pix_x, pix_y, 320, 158, cur1_x_s, -cur1_y_s))
                page_rgb_r = ext_mag_good_w ? C_BLUE : C_ORANGE;
            if (current_m1_pt_hit_r)
                page_rgb_r = ext_mag_good_w ? C_CYAN : C_ORANGE;
            if (current_pt_hit_r)
                page_rgb_r = mag_good_w ? C_WHITE : C_ORANGE;
            if (line_near(pix_x, pix_y, 320, 158, cur_x_s, -cur_y_s))
                page_rgb_r = C_MAG;

            // Right status badge: filled status stripe.
            if (rect_fill(pix_x, pix_y, 448, 72, 610, 104))
                page_rgb_r = mag_good_w ? 12'h041 : (mag_bad_w ? 12'h401 : 12'h440);
            if (rect_frame(pix_x, pix_y, 448, 72, 610, 104))
                page_rgb_r = mag_good_w ? C_GREEN : (mag_bad_w ? C_RED : C_YELLOW);

            // Norm gauge and raw axes: bottom-left.
            if (rect_frame(pix_x, pix_y, 42, 356, 282, 384))
                page_rgb_r = C_DIM;
            if (rect_fill(pix_x, pix_y, 43, 357, 43 + norm_bar_w, 383))
                page_rgb_r = mag_good_w ? C_GREEN : (mag_bad_w ? C_RED : C_YELLOW);
            if ((pix_x == 122) || (pix_x == 202)) begin
                if ((pix_y >= 356) && (pix_y <= 384))
                    page_rgb_r = C_WHITE;
            end
            if (rect_frame(pix_x, pix_y, 42, 392, 282, 402))
                page_rgb_r = C_DIM;
            if (rect_fill(pix_x, pix_y, 43, 393, 43 + ext_delta_bar_w, 401))
                page_rgb_r = ext_mag_good_w ? C_GREEN :
                             (ext_mag_hard_fault_w ? C_RED : C_YELLOW);
            if (rect_frame(pix_x, pix_y, 42, 406, 282, 416))
                page_rgb_r = C_DIM;
            if (rect_fill(pix_x, pix_y, 43, 407, 43 + ext_norm0_bar_w, 415))
                page_rgb_r = ext_mag0_norm_oor_w ? C_RED : C_BLUE;
            if (rect_frame(pix_x, pix_y, 42, 420, 282, 430))
                page_rgb_r = C_DIM;
            if (rect_fill(pix_x, pix_y, 43, 421, 43 + ext_norm1_bar_w, 429))
                page_rgb_r = ext_mag1_norm_oor_w ? C_RED :
                             (ext_mag1_present_w ? C_MAG : C_YELLOW);

            // Crosscheck area: bottom-right.
            if (rect_fill(pix_x, pix_y, 348, 408, 604, 438))
                page_rgb_r = heading_crosscheck_ok ? 12'h031 : 12'h301;
            if (rect_frame(pix_x, pix_y, 348, 408, 604, 438))
                page_rgb_r = heading_crosscheck_ok ? C_GREEN : C_RED;

            if (text_line_static_hit(pix_x, pix_y, 8, 8, 27, 8'd0, 0)) begin
                text_on_r = 1'b1;
                text_rgb_r = C_CYAN;
            end
            if (text_line_static_hit(pix_x, pix_y, 246, 8, 29, 8'd1, 0)) begin
                text_on_r = 1'b1;
                text_rgb_r = C_WHITE;
            end
            if (text_line_static_hit(pix_x, pix_y, 246, 22, 21, 8'd2, 0)) begin
                text_on_r = 1'b1;
                text_rgb_r = C_YELLOW;
            end
            if (text_line_static_hit(pix_x, pix_y, 8, 22, 19, 8'd20, 0)) begin
                text_on_r = 1'b1;
                text_rgb_r = C_YELLOW;
            end
            if (text_line_static_hit(pix_x, pix_y, 28, 52, 14, 8'd3, 0)) begin
                text_on_r = 1'b1;
                text_rgb_r = C_CYAN;
            end
            if (text_line_static_hit(pix_x, pix_y, 230, 52, 17, 8'd4, 0)) begin
                text_on_r = 1'b1;
                text_rgb_r = C_CYAN;
            end
            if (text_line_static_hit(pix_x, pix_y, 446, 52, 13, 8'd5, 0)) begin
                text_on_r = 1'b1;
                text_rgb_r = C_CYAN;
            end
            if (text_line_static_hit(pix_x, pix_y, 28, 288, 18, 8'd6, 0)) begin
                text_on_r = 1'b1;
                text_rgb_r = C_CYAN;
            end
            if (text_line_static_hit(pix_x, pix_y, 338, 288, 18, 8'd7, 0)) begin
                text_on_r = 1'b1;
                text_rgb_r = C_CYAN;
            end
            if (text_line_static_hit(pix_x, pix_y, 45, 234, 14, 8'd9, 0)) begin
                text_on_r = 1'b1;
                text_rgb_r = C_DIM;
            end
            if (text_line_static_hit(pix_x, pix_y, 252, 246, 17, 8'd14, 0)) begin
                text_on_r = 1'b1;
                text_rgb_r = C_DIM;
            end
            if (text_line_static_hit(pix_x, pix_y, 32, 304, 13, 8'd8, 0)) begin
                text_on_r = 1'b1;
                text_rgb_r = C_DIM;
            end
            if (text_line_dyn_hit(pix_x, pix_y, 38, 72, 11, 8'd100, 1, hdg_deg_w, ((mag_age_ms_pix > 16'd999) ? 10'd999 : mag_age_ms_pix[9:0]), mag_seq_pix, mag_status_pix, mx_pix[15:0], my_pix[15:0], mz_pix[15:0], norm_approx_w, pub_sector8_w, raw_sector8_w, i2c_nack_count_pix, i2c_timeout_count_pix, txn_rate_hz_pix, der_head_fresh_pix)) begin
                text_on_r = 1'b1;
                text_rgb_r = mag_good_w ? C_GREEN : (mag_bad_w ? C_RED : C_YELLOW);
            end
            if (mag_good_w && text_line_static_hit(pix_x, pix_y, 462, 84, 9, 8'd10, 1)) begin text_on_r = 1'b1; text_rgb_r = C_GREEN; end
            if ((!mag_good_w) && (!mag_bad_w) && text_line_static_hit(pix_x, pix_y, 462, 84, 9, 8'd11, 1)) begin text_on_r = 1'b1; text_rgb_r = C_YELLOW; end
            if (mag_bad_w && text_line_static_hit(pix_x, pix_y, 462, 84, 10, 8'd12, 1)) begin text_on_r = 1'b1; text_rgb_r = C_RED; end
            if (text_line_dyn_hit(pix_x, pix_y, 450, 118, 10, 8'd101, 0, hdg_deg_w, ((mag_age_ms_pix > 16'd999) ? 10'd999 : mag_age_ms_pix[9:0]), mag_seq_pix, mag_status_pix, mx_pix[15:0], my_pix[15:0], mz_pix[15:0], norm_approx_w, pub_sector8_w, raw_sector8_w, i2c_nack_count_pix, i2c_timeout_count_pix, txn_rate_hz_pix, der_head_fresh_pix)) begin
                text_on_r = 1'b1;
                text_rgb_r = mag_good_w ? C_GREEN : (mag_bad_w ? C_RED : C_YELLOW);
            end
            if (text_line_dyn_hit(pix_x, pix_y, 450, 132, 8, 8'd102, 0, hdg_deg_w, ((mag_age_ms_pix > 16'd999) ? 10'd999 : mag_age_ms_pix[9:0]), mag_seq_pix, mag_status_pix, mx_pix[15:0], my_pix[15:0], mz_pix[15:0], norm_approx_w, pub_sector8_w, raw_sector8_w, i2c_nack_count_pix, i2c_timeout_count_pix, txn_rate_hz_pix, der_head_fresh_pix)) begin
                text_on_r = 1'b1;
                text_rgb_r = C_WHITE;
            end
            if (text_line_dyn_hit(pix_x, pix_y, 450, 146, 7, 8'd103, 0, hdg_deg_w, ((mag_age_ms_pix > 16'd999) ? 10'd999 : mag_age_ms_pix[9:0]), mag_seq_pix, mag_status_pix, mx_pix[15:0], my_pix[15:0], mz_pix[15:0], norm_approx_w, pub_sector8_w, raw_sector8_w, i2c_nack_count_pix, i2c_timeout_count_pix, txn_rate_hz_pix, der_head_fresh_pix)) begin
                text_on_r = 1'b1;
                text_rgb_r = (mag_status_pix == 8'h00) ? C_GREEN : C_RED;
            end
            if (text_line_dyn_hit(pix_x, pix_y, 450, 160, 8, 8'd110, 0, hdg_deg_w, ((mag_age_ms_pix > 16'd999) ? 10'd999 : mag_age_ms_pix[9:0]), mag_seq_pix, mag_status_pix, mx_pix[15:0], my_pix[15:0], mz_pix[15:0], norm_approx_w, pub_sector8_w, raw_sector8_w, i2c_nack_count_pix, i2c_timeout_count_pix, txn_rate_hz_pix, der_head_fresh_pix)) begin
                text_on_r = 1'b1;
                text_rgb_r = der_head_fresh_pix ? C_GREEN : C_YELLOW;
            end
            if (text_line_dyn_hit(pix_x, pix_y, 450, 174, 8, 8'd114, 0, hdg_deg_w, ((mag_age_ms_pix > 16'd999) ? 10'd999 : mag_age_ms_pix[9:0]), mag_seq_pix, mag_status_pix, mx_pix[15:0], my_pix[15:0], mz_pix[15:0], norm_approx_w, pub_sector8_w, raw_sector8_w, i2c_nack_count_pix, i2c_timeout_count_pix, txn_rate_hz_pix, der_head_fresh_pix)) begin
                text_on_r = 1'b1;
                text_rgb_r = (der_mag_seq_ref_pix == mag_seq_pix) ? C_GREEN : C_YELLOW;
            end
            if (text_line_ext_hit(pix_x, pix_y, 450, 194, 16, 8'd200, 0, ext_valid_pix, ext_status_pix, ext_present_flags_pix, ext_fault_flags_pix, ext_mag_delta_l1_pix, ext_mag_norm_primary_pix, ext_mag_norm_secondary_pix, ext_max_age_ms_pix)) begin
                text_on_r = 1'b1;
                text_rgb_r = C_CYAN;
            end
            if (text_line_ext_hit(pix_x, pix_y, 450, 208, 12, 8'd205, 0, ext_valid_pix, ext_status_pix, ext_present_flags_pix, ext_fault_flags_pix, ext_mag_delta_l1_pix, ext_mag_norm_primary_pix, ext_mag_norm_secondary_pix, ext_max_age_ms_pix)) begin
                text_on_r = 1'b1;
                text_rgb_r = ext_mag_good_w ? C_GREEN :
                             (ext_mag_hard_fault_w ? C_RED : C_YELLOW);
            end
            if (text_line_ext_hit(pix_x, pix_y, 450, 222, 14, 8'd201, 0, ext_valid_pix, ext_status_pix, ext_present_flags_pix, ext_fault_flags_pix, ext_mag_delta_l1_pix, ext_mag_norm_primary_pix, ext_mag_norm_secondary_pix, ext_max_age_ms_pix)) begin
                text_on_r = 1'b1;
                text_rgb_r = (ext_status_pix == `ST_OK) ? C_GREEN :
                             (ext_mag_hard_fault_w ? C_RED : C_YELLOW);
            end
            if (text_line_ext_hit(pix_x, pix_y, 450, 236, 18, 8'd202, 0, ext_valid_pix, ext_status_pix, ext_present_flags_pix, ext_fault_flags_pix, ext_mag_delta_l1_pix, ext_mag_norm_primary_pix, ext_mag_norm_secondary_pix, ext_max_age_ms_pix)) begin
                text_on_r = 1'b1;
                text_rgb_r = ext_mag_hard_fault_w ? C_RED :
                             (ext_mag_pair_missing_w ? C_YELLOW : C_WHITE);
            end
            if (text_line_ext_hit(pix_x, pix_y, 450, 250, 13, 8'd203, 0, ext_valid_pix, ext_status_pix, ext_present_flags_pix, ext_fault_flags_pix, ext_mag_delta_l1_pix, ext_mag_norm_primary_pix, ext_mag_norm_secondary_pix, ext_max_age_ms_pix)) begin
                text_on_r = 1'b1;
                text_rgb_r = ext_mag_norm_fault_w ? C_RED : C_WHITE;
            end
            if (text_line_dyn_hit(pix_x, pix_y, 36, 320, 7, 8'd104, 0, hdg_deg_w, ((mag_age_ms_pix > 16'd999) ? 10'd999 : mag_age_ms_pix[9:0]), mag_seq_pix, mag_status_pix, mx_pix[15:0], my_pix[15:0], mz_pix[15:0], norm_approx_w, pub_sector8_w, raw_sector8_w, i2c_nack_count_pix, i2c_timeout_count_pix, txn_rate_hz_pix, der_head_fresh_pix)) begin
                text_on_r = 1'b1;
                text_rgb_r = C_WHITE;
            end
            if (text_line_dyn_hit(pix_x, pix_y, 126, 320, 7, 8'd105, 0, hdg_deg_w, ((mag_age_ms_pix > 16'd999) ? 10'd999 : mag_age_ms_pix[9:0]), mag_seq_pix, mag_status_pix, mx_pix[15:0], my_pix[15:0], mz_pix[15:0], norm_approx_w, pub_sector8_w, raw_sector8_w, i2c_nack_count_pix, i2c_timeout_count_pix, txn_rate_hz_pix, der_head_fresh_pix)) begin
                text_on_r = 1'b1;
                text_rgb_r = C_WHITE;
            end
            if (text_line_dyn_hit(pix_x, pix_y, 216, 320, 7, 8'd106, 0, hdg_deg_w, ((mag_age_ms_pix > 16'd999) ? 10'd999 : mag_age_ms_pix[9:0]), mag_seq_pix, mag_status_pix, mx_pix[15:0], my_pix[15:0], mz_pix[15:0], norm_approx_w, pub_sector8_w, raw_sector8_w, i2c_nack_count_pix, i2c_timeout_count_pix, txn_rate_hz_pix, der_head_fresh_pix)) begin
                text_on_r = 1'b1;
                text_rgb_r = C_WHITE;
            end
            if (text_line_dyn_hit(pix_x, pix_y, 36, 338, 9, 8'd107, 0, hdg_deg_w, ((mag_age_ms_pix > 16'd999) ? 10'd999 : mag_age_ms_pix[9:0]), mag_seq_pix, mag_status_pix, mx_pix[15:0], my_pix[15:0], mz_pix[15:0], norm_approx_w, pub_sector8_w, raw_sector8_w, i2c_nack_count_pix, i2c_timeout_count_pix, txn_rate_hz_pix, der_head_fresh_pix)) begin
                text_on_r = 1'b1;
                text_rgb_r = C_YELLOW;
            end
            if (text_line_dyn_hit(pix_x, pix_y, 344, 318, 12, 8'd108, 0, hdg_deg_w, ((mag_age_ms_pix > 16'd999) ? 10'd999 : mag_age_ms_pix[9:0]), mag_seq_pix, mag_status_pix, mx_pix[15:0], my_pix[15:0], mz_pix[15:0], norm_approx_w, pub_sector8_w, raw_sector8_w, i2c_nack_count_pix, i2c_timeout_count_pix, txn_rate_hz_pix, der_head_fresh_pix)) begin
                text_on_r = 1'b1;
                text_rgb_r = C_WHITE;
            end
            if (text_line_dyn_hit(pix_x, pix_y, 344, 336, 15, 8'd109, 0, hdg_deg_w, ((mag_age_ms_pix > 16'd999) ? 10'd999 : mag_age_ms_pix[9:0]), mag_seq_pix, mag_status_pix, mx_pix[15:0], my_pix[15:0], mz_pix[15:0], norm_approx_w, pub_sector8_w, raw_sector8_w, i2c_nack_count_pix, i2c_timeout_count_pix, txn_rate_hz_pix, der_head_fresh_pix)) begin
                text_on_r = 1'b1;
                text_rgb_r = heading_crosscheck_ok ? C_GREEN : C_YELLOW;
            end
            if (heading_crosscheck_ok && text_line_static_hit(pix_x, pix_y, 374, 416, 5, 8'd15, 2)) begin text_on_r = 1'b1; text_rgb_r = C_GREEN; end
            if ((!heading_crosscheck_ok) && text_line_static_hit(pix_x, pix_y, 358, 416, 8, 8'd16, 2)) begin text_on_r = 1'b1; text_rgb_r = C_RED; end
            if (text_line_dyn_hit(pix_x, pix_y, 344, 360, 10, 8'd111, 0, hdg_deg_w, ((mag_age_ms_pix > 16'd999) ? 10'd999 : mag_age_ms_pix[9:0]), mag_seq_pix, mag_status_pix, mx_pix[15:0], my_pix[15:0], mz_pix[15:0], norm_approx_w, pub_sector8_w, raw_sector8_w, i2c_nack_count_pix, i2c_timeout_count_pix, txn_rate_hz_pix, der_head_fresh_pix)) begin
                text_on_r = 1'b1;
                text_rgb_r = (i2c_nack_count_pix == 16'd0) ? C_GREEN : C_YELLOW;
            end
            if (text_line_dyn_hit(pix_x, pix_y, 344, 374, 12, 8'd112, 0, hdg_deg_w, ((mag_age_ms_pix > 16'd999) ? 10'd999 : mag_age_ms_pix[9:0]), mag_seq_pix, mag_status_pix, mx_pix[15:0], my_pix[15:0], mz_pix[15:0], norm_approx_w, pub_sector8_w, raw_sector8_w, i2c_nack_count_pix, i2c_timeout_count_pix, txn_rate_hz_pix, der_head_fresh_pix)) begin
                text_on_r = 1'b1;
                text_rgb_r = (i2c_timeout_count_pix == 16'd0) ? C_GREEN : C_YELLOW;
            end
            if (text_line_dyn_hit(pix_x, pix_y, 344, 388, 10, 8'd113, 0, hdg_deg_w, ((mag_age_ms_pix > 16'd999) ? 10'd999 : mag_age_ms_pix[9:0]), mag_seq_pix, mag_status_pix, mx_pix[15:0], my_pix[15:0], mz_pix[15:0], norm_approx_w, pub_sector8_w, raw_sector8_w, i2c_nack_count_pix, i2c_timeout_count_pix, txn_rate_hz_pix, der_head_fresh_pix)) begin
                text_on_r = 1'b1;
                text_rgb_r = C_CYAN;
            end
            if (text_line_mag1_hit(pix_x, pix_y, 470, 304, 8, 8'd210, 0, mag1_seq_pix, mag1_valid_pix, mag1_status_pix, m1x_pix[15:0], m1y_pix[15:0], m1z_pix[15:0], mag1_age_ms_pix, ext_mag_sequence_aligned_pix, ext_mag_disagreement_pix, ext_mag_sector_delta_pix, ext_mag_norm_delta_l1_pix, ext_mag_iron_residual_pix, ext_mag_cal_state_pix, ext_mag_source_flags_pix, ext_mag_bridge_checksum_pix)) begin
                text_on_r = 1'b1;
                text_rgb_r = mag1_valid_pix ? C_BLUE : C_DIM;
            end
            if (text_line_mag1_hit(pix_x, pix_y, 470, 318, 8, 8'd211, 0, mag1_seq_pix, mag1_valid_pix, mag1_status_pix, m1x_pix[15:0], m1y_pix[15:0], m1z_pix[15:0], mag1_age_ms_pix, ext_mag_sequence_aligned_pix, ext_mag_disagreement_pix, ext_mag_sector_delta_pix, ext_mag_norm_delta_l1_pix, ext_mag_iron_residual_pix, ext_mag_cal_state_pix, ext_mag_source_flags_pix, ext_mag_bridge_checksum_pix)) begin
                text_on_r = 1'b1;
                text_rgb_r = mag1_valid_pix ? C_BLUE : C_DIM;
            end
            if (text_line_mag1_hit(pix_x, pix_y, 470, 332, 8, 8'd212, 0, mag1_seq_pix, mag1_valid_pix, mag1_status_pix, m1x_pix[15:0], m1y_pix[15:0], m1z_pix[15:0], mag1_age_ms_pix, ext_mag_sequence_aligned_pix, ext_mag_disagreement_pix, ext_mag_sector_delta_pix, ext_mag_norm_delta_l1_pix, ext_mag_iron_residual_pix, ext_mag_cal_state_pix, ext_mag_source_flags_pix, ext_mag_bridge_checksum_pix)) begin
                text_on_r = 1'b1;
                text_rgb_r = mag1_valid_pix ? C_BLUE : C_DIM;
            end
            if (text_line_mag1_hit(pix_x, pix_y, 470, 346, 9, 8'd213, 0, mag1_seq_pix, mag1_valid_pix, mag1_status_pix, m1x_pix[15:0], m1y_pix[15:0], m1z_pix[15:0], mag1_age_ms_pix, ext_mag_sequence_aligned_pix, ext_mag_disagreement_pix, ext_mag_sector_delta_pix, ext_mag_norm_delta_l1_pix, ext_mag_iron_residual_pix, ext_mag_cal_state_pix, ext_mag_source_flags_pix, ext_mag_bridge_checksum_pix)) begin
                text_on_r = 1'b1;
                text_rgb_r = (mag1_status_pix == `ST_OK) ? (mag1_synthetic_w ? C_YELLOW : C_GREEN) : C_ORANGE;
            end
            if (text_line_mag1_hit(pix_x, pix_y, 470, 360, 11, 8'd214, 0, mag1_seq_pix, mag1_valid_pix, mag1_status_pix, m1x_pix[15:0], m1y_pix[15:0], m1z_pix[15:0], mag1_age_ms_pix, ext_mag_sequence_aligned_pix, ext_mag_disagreement_pix, ext_mag_sector_delta_pix, ext_mag_norm_delta_l1_pix, ext_mag_iron_residual_pix, ext_mag_cal_state_pix, ext_mag_source_flags_pix, ext_mag_bridge_checksum_pix)) begin
                text_on_r = 1'b1;
                text_rgb_r = mag1_synthetic_w ? C_YELLOW : C_WHITE;
            end
            if (text_line_mag1_hit(pix_x, pix_y, 470, 374, 11, 8'd215, 0, mag1_seq_pix, mag1_valid_pix, mag1_status_pix, m1x_pix[15:0], m1y_pix[15:0], m1z_pix[15:0], mag1_age_ms_pix, ext_mag_sequence_aligned_pix, ext_mag_disagreement_pix, ext_mag_sector_delta_pix, ext_mag_norm_delta_l1_pix, ext_mag_iron_residual_pix, ext_mag_cal_state_pix, ext_mag_source_flags_pix, ext_mag_bridge_checksum_pix)) begin
                text_on_r = 1'b1;
                text_rgb_r = ext_mag_hard_fault_w ? C_RED :
                             (ext_mag_sequence_aligned_pix ? C_GREEN : C_YELLOW);
            end
            if (text_line_mag1_hit(pix_x, pix_y, 470, 388, 13, 8'd216, 0, mag1_seq_pix, mag1_valid_pix, mag1_status_pix, m1x_pix[15:0], m1y_pix[15:0], m1z_pix[15:0], mag1_age_ms_pix, ext_mag_sequence_aligned_pix, ext_mag_disagreement_pix, ext_mag_sector_delta_pix, ext_mag_norm_delta_l1_pix, ext_mag_iron_residual_pix, ext_mag_cal_state_pix, ext_mag_source_flags_pix, ext_mag_bridge_checksum_pix)) begin
                text_on_r = 1'b1;
                text_rgb_r = ext_mag_norm_fault_w ? C_RED : C_CYAN;
            end
            if (text_line_mag1_hit(pix_x, pix_y, 470, 402, 11, 8'd217, 0, mag1_seq_pix, mag1_valid_pix, mag1_status_pix, m1x_pix[15:0], m1y_pix[15:0], m1z_pix[15:0], mag1_age_ms_pix, ext_mag_sequence_aligned_pix, ext_mag_disagreement_pix, ext_mag_sector_delta_pix, ext_mag_norm_delta_l1_pix, ext_mag_iron_residual_pix, ext_mag_cal_state_pix, ext_mag_source_flags_pix, ext_mag_bridge_checksum_pix)) begin
                text_on_r = 1'b1;
                text_rgb_r = mag1_synthetic_w ? C_YELLOW : C_WHITE;
            end
            if (text_line_mag1_hit(pix_x, pix_y, 470, 416, 6, 8'd218, 0, mag1_seq_pix, mag1_valid_pix, mag1_status_pix, m1x_pix[15:0], m1y_pix[15:0], m1z_pix[15:0], mag1_age_ms_pix, ext_mag_sequence_aligned_pix, ext_mag_disagreement_pix, ext_mag_sector_delta_pix, ext_mag_norm_delta_l1_pix, ext_mag_iron_residual_pix, ext_mag_cal_state_pix, ext_mag_source_flags_pix, ext_mag_bridge_checksum_pix)) begin
                text_on_r = 1'b1;
                text_rgb_r = C_CYAN;
            end
            if (text_line_static_hit(pix_x, pix_y, 32, 438, 15, 8'd15, 0)) begin
                text_on_r = 1'b1;
                text_rgb_r = C_DIM;
            end
            if (text_line_static_hit(pix_x, pix_y, 344, 438, 17, 8'd16, 0)) begin
                text_on_r = 1'b1;
                text_rgb_r = C_DIM;
            end
            if (text_on_r)
                page_rgb_r = text_rgb_r;
        end
    end

    assign vga_hsync_out = vga_hsync_in;
    assign vga_vsync_out = vga_vsync_in;
    assign vga_rgb_out   = page_active ? page_rgb_r : vga_rgb_in;

endmodule

`default_nettype wire
