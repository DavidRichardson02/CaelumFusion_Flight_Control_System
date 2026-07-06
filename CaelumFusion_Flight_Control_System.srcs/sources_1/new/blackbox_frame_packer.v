`timescale 1ns/1ps
`default_nettype none

`include "telemetry_defs_vh.vh"

//==============================================================================
// blackbox_frame_packer
//------------------------------------------------------------------------------
// Deterministic raw black-box frame source.
//
// This block is intentionally storage-agnostic: it produces a ready/valid stream
// of 32-bit words that can later feed an SD-card SPI writer, a Teensy bridge, or
// a UART/debug transport. A frame latches all input snapshots on emit_req so the
// downstream writer never sees a mixed-time record.
//==============================================================================
module blackbox_frame_packer #(
    parameter integer PAYLOAD_W   = 48,
    parameter integer FRAME_WORDS = 29
)(
    input  wire                 clk,
    input  wire                 rst,
    input  wire                 enable,
    input  wire                 emit_req,

    input  wire [31:0]          bmp_t_us,
    input  wire [15:0]          bmp_seq,
    input  wire                 bmp_valid,
    input  wire [7:0]           bmp_status,
    input  wire [PAYLOAD_W-1:0] bmp_payload,
    input  wire [15:0]          bmp_age_ms,

    input  wire [31:0]          acc_t_us,
    input  wire [15:0]          acc_seq,
    input  wire                 acc_valid,
    input  wire [7:0]           acc_status,
    input  wire [PAYLOAD_W-1:0] acc_payload,
    input  wire [15:0]          acc_age_ms,

    input  wire [31:0]          mag_t_us,
    input  wire [15:0]          mag_seq,
    input  wire                 mag_valid,
    input  wire [7:0]           mag_status,
    input  wire [PAYLOAD_W-1:0] mag_payload,
    input  wire [15:0]          mag_age_ms,

    input  wire [31:0]          pwr_t_us,
    input  wire [15:0]          pwr_seq,
    input  wire                 pwr_valid,
    input  wire [7:0]           pwr_status,
    input  wire [PAYLOAD_W-1:0] pwr_payload,
    input  wire [15:0]          pwr_age_ms,

    input  wire                 ext_valid,
    input  wire [7:0]           ext_status,
    input  wire [15:0]          ext_present_flags,
    input  wire [15:0]          ext_fault_flags,
    input  wire [15:0]          ext_mag_delta_l1,
    input  wire [15:0]          ext_mag_norm_primary,
    input  wire [15:0]          ext_mag_norm_secondary,
    input  wire                 ext_mag_sequence_aligned,
    input  wire                 ext_mag_disagreement,
    input  wire [3:0]           ext_mag_sector_delta,
    input  wire [15:0]          ext_mag_norm_delta_l1,
    input  wire [15:0]          ext_mag_iron_residual,
    input  wire [7:0]           ext_mag_cal_state,
    input  wire [7:0]           ext_mag_source_flags,
    input  wire [15:0]          ext_mag_bridge_checksum,
    input  wire [15:0]          ext_rng_height_cm,
    input  wire [15:0]          ext_air_dp_pa,
    input  wire [15:0]          ext_air_speed_cms,
    input  wire [15:0]          ext_env_temp_cdeg,
    input  wire [15:0]          ext_env_rh_centi,
    input  wire [15:0]          ext_sun_luma,
    input  wire [15:0]          ext_flow_dx,
    input  wire [15:0]          ext_flow_dy,
    input  wire [15:0]          ext_max_age_ms,

    output reg                  stream_valid,
    input  wire                 stream_ready,
    output reg  [31:0]          stream_word,
    output reg                  stream_last,

    output reg  [15:0]          log_seq,
    output reg  [15:0]          drop_count,
    output wire                 busy
);
    localparam [7:0] FRAME_VERSION = 8'h02;
    localparam [7:0] FRAME_WORDS_U8 = FRAME_WORDS;

    reg [7:0] word_idx_r;
    reg [15:0] frame_seq_r;

    reg [31:0] bmp_t_us_r;
    reg [15:0] bmp_seq_r;
    reg        bmp_valid_r;
    reg [7:0]  bmp_status_r;
    reg [PAYLOAD_W-1:0] bmp_payload_r;
    reg [15:0] bmp_age_ms_r;

    reg [31:0] acc_t_us_r;
    reg [15:0] acc_seq_r;
    reg        acc_valid_r;
    reg [7:0]  acc_status_r;
    reg [PAYLOAD_W-1:0] acc_payload_r;
    reg [15:0] acc_age_ms_r;

    reg [31:0] mag_t_us_r;
    reg [15:0] mag_seq_r;
    reg        mag_valid_r;
    reg [7:0]  mag_status_r;
    reg [PAYLOAD_W-1:0] mag_payload_r;
    reg [15:0] mag_age_ms_r;

    reg [31:0] pwr_t_us_r;
    reg [15:0] pwr_seq_r;
    reg        pwr_valid_r;
    reg [7:0]  pwr_status_r;
    reg [PAYLOAD_W-1:0] pwr_payload_r;
    reg [15:0] pwr_age_ms_r;

    reg        ext_valid_r;
    reg [7:0]  ext_status_r;
    reg [15:0] ext_present_flags_r;
    reg [15:0] ext_fault_flags_r;
    reg [15:0] ext_mag_delta_l1_r;
    reg [15:0] ext_mag_norm_primary_r;
    reg [15:0] ext_mag_norm_secondary_r;
    reg        ext_mag_sequence_aligned_r;
    reg        ext_mag_disagreement_r;
    reg [3:0]  ext_mag_sector_delta_r;
    reg [15:0] ext_mag_norm_delta_l1_r;
    reg [15:0] ext_mag_iron_residual_r;
    reg [7:0]  ext_mag_cal_state_r;
    reg [7:0]  ext_mag_source_flags_r;
    reg [15:0] ext_mag_bridge_checksum_r;
    reg [15:0] ext_rng_height_cm_r;
    reg [15:0] ext_air_dp_pa_r;
    reg [15:0] ext_air_speed_cms_r;
    reg [15:0] ext_env_temp_cdeg_r;
    reg [15:0] ext_env_rh_centi_r;
    reg [15:0] ext_sun_luma_r;
    reg [15:0] ext_flow_dx_r;
    reg [15:0] ext_flow_dy_r;
    reg [15:0] ext_max_age_ms_r;

    assign busy = stream_valid;

    function [31:0] meta_word;
        input [15:0] seq_i;
        input        valid_i;
        input [7:0]  status_i;
        begin
            meta_word = {seq_i, 7'd0, valid_i, status_i};
        end
    endfunction

    function [31:0] mag_meta_word;
        input       sequence_aligned_i;
        input       disagreement_i;
        input [3:0] sector_delta_i;
        input [7:0] cal_state_i;
        input [7:0] source_flags_i;
        begin
            mag_meta_word = 32'd0;
            mag_meta_word[`EXT_MAG_META_SRC_FLAGS_MSB:`EXT_MAG_META_SRC_FLAGS_LSB] =
                source_flags_i;
            mag_meta_word[`EXT_MAG_META_CAL_STATE_MSB:`EXT_MAG_META_CAL_STATE_LSB] =
                cal_state_i;
            mag_meta_word[`EXT_MAG_META_SECTOR_DELTA_MSB:`EXT_MAG_META_SECTOR_DELTA_LSB] =
                sector_delta_i;
            mag_meta_word[`EXT_MAG_META_DISAGREE_BIT] = disagreement_i;
            mag_meta_word[`EXT_MAG_META_SEQ_ALIGNED_BIT] = sequence_aligned_i;
        end
    endfunction

    function [31:0] frame_word_at;
        input [7:0] idx;
        begin
            case (idx)
                8'd0:  frame_word_at = {`TELEM_PKT_SYNC, `PKT_BLACKBOX_WORD, FRAME_VERSION};
                8'd1:  frame_word_at = {frame_seq_r, FRAME_WORDS_U8, 8'd0};
                8'd2:  frame_word_at = bmp_t_us_r;
                8'd3:  frame_word_at = meta_word(bmp_seq_r, bmp_valid_r, bmp_status_r);
                8'd4:  frame_word_at = bmp_payload_r[47:16];
                8'd5:  frame_word_at = {bmp_payload_r[15:0], bmp_age_ms_r};
                8'd6:  frame_word_at = acc_t_us_r;
                8'd7:  frame_word_at = meta_word(acc_seq_r, acc_valid_r, acc_status_r);
                8'd8:  frame_word_at = acc_payload_r[47:16];
                8'd9:  frame_word_at = {acc_payload_r[15:0], acc_age_ms_r};
                8'd10: frame_word_at = mag_t_us_r;
                8'd11: frame_word_at = meta_word(mag_seq_r, mag_valid_r, mag_status_r);
                8'd12: frame_word_at = mag_payload_r[47:16];
                8'd13: frame_word_at = {mag_payload_r[15:0], mag_age_ms_r};
                8'd14: frame_word_at = pwr_t_us_r;
                8'd15: frame_word_at = meta_word(pwr_seq_r, pwr_valid_r, pwr_status_r);
                8'd16: frame_word_at = pwr_payload_r[47:16];
                8'd17: frame_word_at = {pwr_payload_r[15:0], pwr_age_ms_r};
                8'd18: frame_word_at = {ext_valid_r, 7'd0, ext_status_r, ext_present_flags_r};
                8'd19: frame_word_at = {ext_fault_flags_r, ext_mag_delta_l1_r};
                8'd20: frame_word_at = {ext_mag_norm_primary_r, ext_mag_norm_secondary_r};
                8'd21: frame_word_at = {ext_rng_height_cm_r, ext_air_dp_pa_r};
                8'd22: frame_word_at = {ext_air_speed_cms_r, ext_env_temp_cdeg_r};
                8'd23: frame_word_at = {ext_env_rh_centi_r, ext_sun_luma_r};
                8'd24: frame_word_at = {ext_flow_dx_r, ext_flow_dy_r};
                8'd25: frame_word_at = {drop_count, ext_max_age_ms_r};
                8'd26: frame_word_at = mag_meta_word(ext_mag_sequence_aligned_r,
                                                       ext_mag_disagreement_r,
                                                       ext_mag_sector_delta_r,
                                                       ext_mag_cal_state_r,
                                                       ext_mag_source_flags_r);
                8'd27: frame_word_at = {ext_mag_norm_delta_l1_r, ext_mag_iron_residual_r};
                8'd28: frame_word_at = {ext_mag_bridge_checksum_r, 16'd0};
                default: frame_word_at = 32'd0;
            endcase
        end
    endfunction

    always @(posedge clk) begin
        if (rst) begin
            word_idx_r    <= 8'd0;
            frame_seq_r   <= 16'd0;
            stream_valid  <= 1'b0;
            stream_word   <= 32'd0;
            stream_last   <= 1'b0;
            log_seq       <= 16'd0;
            drop_count    <= 16'd0;

            bmp_t_us_r    <= 32'd0;
            bmp_seq_r     <= 16'd0;
            bmp_valid_r   <= 1'b0;
            bmp_status_r  <= `ST_NOT_INITIALIZED;
            bmp_payload_r <= {PAYLOAD_W{1'b0}};
            bmp_age_ms_r  <= 16'hFFFF;

            acc_t_us_r    <= 32'd0;
            acc_seq_r     <= 16'd0;
            acc_valid_r   <= 1'b0;
            acc_status_r  <= `ST_NOT_INITIALIZED;
            acc_payload_r <= {PAYLOAD_W{1'b0}};
            acc_age_ms_r  <= 16'hFFFF;

            mag_t_us_r    <= 32'd0;
            mag_seq_r     <= 16'd0;
            mag_valid_r   <= 1'b0;
            mag_status_r  <= `ST_NOT_INITIALIZED;
            mag_payload_r <= {PAYLOAD_W{1'b0}};
            mag_age_ms_r  <= 16'hFFFF;

            pwr_t_us_r    <= 32'd0;
            pwr_seq_r     <= 16'd0;
            pwr_valid_r   <= 1'b0;
            pwr_status_r  <= `ST_NOT_INITIALIZED;
            pwr_payload_r <= {PAYLOAD_W{1'b0}};
            pwr_age_ms_r  <= 16'hFFFF;

            ext_valid_r             <= 1'b0;
            ext_status_r            <= `ST_NOT_INITIALIZED;
            ext_present_flags_r     <= 16'd0;
            ext_fault_flags_r       <= 16'd0;
            ext_mag_delta_l1_r      <= 16'd0;
            ext_mag_norm_primary_r  <= 16'd0;
            ext_mag_norm_secondary_r<= 16'd0;
            ext_mag_sequence_aligned_r <= 1'b0;
            ext_mag_disagreement_r    <= 1'b0;
            ext_mag_sector_delta_r    <= 4'd0;
            ext_mag_norm_delta_l1_r   <= 16'd0;
            ext_mag_iron_residual_r   <= 16'd0;
            ext_mag_cal_state_r       <= 8'd0;
            ext_mag_source_flags_r    <= 8'd0;
            ext_mag_bridge_checksum_r <= 16'd0;
            ext_rng_height_cm_r     <= 16'd0;
            ext_air_dp_pa_r         <= 16'd0;
            ext_air_speed_cms_r     <= 16'd0;
            ext_env_temp_cdeg_r     <= 16'd0;
            ext_env_rh_centi_r      <= 16'd0;
            ext_sun_luma_r          <= 16'd0;
            ext_flow_dx_r           <= 16'd0;
            ext_flow_dy_r           <= 16'd0;
            ext_max_age_ms_r        <= 16'hFFFF;
        end else begin
            if (emit_req && enable && stream_valid) begin
                if (drop_count != 16'hFFFF)
                    drop_count <= drop_count + 16'd1;
            end

            if (emit_req && enable && !stream_valid) begin
                frame_seq_r <= log_seq + 16'd1;
                log_seq     <= log_seq + 16'd1;

                bmp_t_us_r    <= bmp_t_us;
                bmp_seq_r     <= bmp_seq;
                bmp_valid_r   <= bmp_valid;
                bmp_status_r  <= bmp_status;
                bmp_payload_r <= bmp_payload;
                bmp_age_ms_r  <= bmp_age_ms;

                acc_t_us_r    <= acc_t_us;
                acc_seq_r     <= acc_seq;
                acc_valid_r   <= acc_valid;
                acc_status_r  <= acc_status;
                acc_payload_r <= acc_payload;
                acc_age_ms_r  <= acc_age_ms;

                mag_t_us_r    <= mag_t_us;
                mag_seq_r     <= mag_seq;
                mag_valid_r   <= mag_valid;
                mag_status_r  <= mag_status;
                mag_payload_r <= mag_payload;
                mag_age_ms_r  <= mag_age_ms;

                pwr_t_us_r    <= pwr_t_us;
                pwr_seq_r     <= pwr_seq;
                pwr_valid_r   <= pwr_valid;
                pwr_status_r  <= pwr_status;
                pwr_payload_r <= pwr_payload;
                pwr_age_ms_r  <= pwr_age_ms;

                ext_valid_r              <= ext_valid;
                ext_status_r             <= ext_status;
                ext_present_flags_r      <= ext_present_flags;
                ext_fault_flags_r        <= ext_fault_flags;
                ext_mag_delta_l1_r       <= ext_mag_delta_l1;
                ext_mag_norm_primary_r   <= ext_mag_norm_primary;
                ext_mag_norm_secondary_r <= ext_mag_norm_secondary;
                ext_mag_sequence_aligned_r <= ext_mag_sequence_aligned;
                ext_mag_disagreement_r    <= ext_mag_disagreement;
                ext_mag_sector_delta_r    <= ext_mag_sector_delta;
                ext_mag_norm_delta_l1_r   <= ext_mag_norm_delta_l1;
                ext_mag_iron_residual_r   <= ext_mag_iron_residual;
                ext_mag_cal_state_r       <= ext_mag_cal_state;
                ext_mag_source_flags_r    <= ext_mag_source_flags;
                ext_mag_bridge_checksum_r <= ext_mag_bridge_checksum;
                ext_rng_height_cm_r      <= ext_rng_height_cm;
                ext_air_dp_pa_r          <= ext_air_dp_pa;
                ext_air_speed_cms_r      <= ext_air_speed_cms;
                ext_env_temp_cdeg_r      <= ext_env_temp_cdeg;
                ext_env_rh_centi_r       <= ext_env_rh_centi;
                ext_sun_luma_r           <= ext_sun_luma;
                ext_flow_dx_r            <= ext_flow_dx;
                ext_flow_dy_r            <= ext_flow_dy;
                ext_max_age_ms_r         <= ext_max_age_ms;

                word_idx_r   <= 8'd0;
                stream_valid <= 1'b1;
                stream_word  <= {`TELEM_PKT_SYNC, `PKT_BLACKBOX_WORD, FRAME_VERSION};
                stream_last  <= (FRAME_WORDS_U8 == 8'd1);
            end else if (stream_valid && stream_ready) begin
                if (word_idx_r == (FRAME_WORDS_U8 - 8'd1)) begin
                    stream_valid <= 1'b0;
                    stream_last  <= 1'b0;
                    word_idx_r   <= 8'd0;
                end else begin
                    word_idx_r  <= word_idx_r + 8'd1;
                    stream_word <= frame_word_at(word_idx_r + 8'd1);
                    stream_last <= ((word_idx_r + 8'd1) == (FRAME_WORDS_U8 - 8'd1));
                end
            end
        end
    end
endmodule

`default_nettype wire
