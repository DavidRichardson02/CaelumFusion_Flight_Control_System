`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// telemetry_pix_snapshot_bank
//------------------------------------------------------------------------------
// ROLE
//   PIX-domain latch bank for raw + derived telemetry after SYS->PIX CDC.
//
// CONTRACT
//   - All outputs update only on pix_commit.
//   - Text/render logic must consume only these PIX-local registers.
//   - No direct formatting from live CDC-transfer wires.
//==============================================================================

module telemetry_pix_snapshot_bank (
    input  wire        pix_clk,
    input  wire        pix_rst,
    input  wire        pix_commit,

    //--------------------------------------------------------------------------
    // Raw sensor summaries transferred from SYS
    //--------------------------------------------------------------------------
    input  wire        bmp_valid_in,
    input  wire [7:0]  bmp_status_in,
    input  wire [15:0] bmp_age_ms_in,
    input  wire [15:0] bmp_seq_in,

    input  wire        acc_valid_in,
    input  wire [7:0]  acc_status_in,
    input  wire [15:0] acc_age_ms_in,
    input  wire [15:0] acc_seq_in,

    input  wire        mag_valid_in,
    input  wire [7:0]  mag_status_in,
    input  wire [15:0] mag_age_ms_in,
    input  wire [15:0] mag_seq_in,

    //--------------------------------------------------------------------------
    // Derived-state summary transferred from SYS
    //--------------------------------------------------------------------------
    input  wire        der_valid_in,
    input  wire [7:0]  der_status_in,

    input  wire        der_alt_fresh_in,
    input  wire        der_vspd_fresh_in,
    input  wire        der_roll_fresh_in,
    input  wire        der_head_fresh_in,

    input  wire [15:0] der_bmp_seq_ref_in,
    input  wire [15:0] der_acc_seq_ref_in,
    input  wire [15:0] der_mag_seq_ref_in,

    input  wire [15:0] der_bmp_age_ms_in,
    input  wire [15:0] der_acc_age_ms_in,
    input  wire [15:0] der_mag_age_ms_in,

    input  wire        der_bmp_valid_ref_in,
    input  wire        der_acc_valid_ref_in,
    input  wire        der_mag_valid_ref_in,

    input  wire [31:0] der_altitude_cm_in,
    input  wire [31:0] der_vertical_speed_cms_in,
    input  wire [31:0] der_roll_mdeg_in,
    input  wire [31:0] der_heading_mdeg_in,

    //--------------------------------------------------------------------------
    // Platform health
    //--------------------------------------------------------------------------
    input  wire [15:0] i2c_nack_count_in,
    input  wire [15:0] i2c_timeout_count_in,
    input  wire [15:0] txn_rate_hz_in,
    input  wire [31:0] cdc_update_count_in,
    input  wire [31:0] frame_count_in,
    input  wire [31:0] build_id_in,
    input  wire [15:0] schema_word_in,

    //--------------------------------------------------------------------------
    // PIX-local outputs
    //--------------------------------------------------------------------------
    output reg         bmp_valid_pix,
    output reg  [7:0]  bmp_status_pix,
    output reg  [15:0] bmp_age_ms_pix,
    output reg  [15:0] bmp_seq_pix,

    output reg         acc_valid_pix,
    output reg  [7:0]  acc_status_pix,
    output reg  [15:0] acc_age_ms_pix,
    output reg  [15:0] acc_seq_pix,

    output reg         mag_valid_pix,
    output reg  [7:0]  mag_status_pix,
    output reg  [15:0] mag_age_ms_pix,
    output reg  [15:0] mag_seq_pix,

    output reg         der_valid_pix,
    output reg  [7:0]  der_status_pix,

    output reg         der_alt_fresh_pix,
    output reg         der_vspd_fresh_pix,
    output reg         der_roll_fresh_pix,
    output reg         der_head_fresh_pix,

    output reg  [15:0] der_bmp_seq_ref_pix,
    output reg  [15:0] der_acc_seq_ref_pix,
    output reg  [15:0] der_mag_seq_ref_pix,

    output reg  [15:0] der_bmp_age_ms_pix,
    output reg  [15:0] der_acc_age_ms_pix,
    output reg  [15:0] der_mag_age_ms_pix,

    output reg         der_bmp_valid_ref_pix,
    output reg         der_acc_valid_ref_pix,
    output reg         der_mag_valid_ref_pix,

    output reg  [31:0] der_altitude_cm_pix,
    output reg  [31:0] der_vertical_speed_cms_pix,
    output reg  [31:0] der_roll_mdeg_pix,
    output reg  [31:0] der_heading_mdeg_pix,

    output reg  [15:0] i2c_nack_count_pix,
    output reg  [15:0] i2c_timeout_count_pix,
    output reg  [15:0] txn_rate_hz_pix,
    output reg  [31:0] cdc_update_count_pix,
    output reg  [31:0] frame_count_pix,
    output reg  [31:0] build_id_pix,
    output reg  [15:0] schema_word_pix
);

    always @(posedge pix_clk) begin
        if (pix_rst) begin
            bmp_valid_pix          <= 1'b0;
            bmp_status_pix         <= 8'h01;
            bmp_age_ms_pix         <= 16'hFFFF;
            bmp_seq_pix            <= 16'd0;

            acc_valid_pix          <= 1'b0;
            acc_status_pix         <= 8'h01;
            acc_age_ms_pix         <= 16'hFFFF;
            acc_seq_pix            <= 16'd0;

            mag_valid_pix          <= 1'b0;
            mag_status_pix         <= 8'h01;
            mag_age_ms_pix         <= 16'hFFFF;
            mag_seq_pix            <= 16'd0;

            der_valid_pix          <= 1'b0;
            der_status_pix         <= 8'h01;

            der_alt_fresh_pix      <= 1'b0;
            der_vspd_fresh_pix     <= 1'b0;
            der_roll_fresh_pix     <= 1'b0;
            der_head_fresh_pix     <= 1'b0;

            der_bmp_seq_ref_pix    <= 16'd0;
            der_acc_seq_ref_pix    <= 16'd0;
            der_mag_seq_ref_pix    <= 16'd0;

            der_bmp_age_ms_pix     <= 16'hFFFF;
            der_acc_age_ms_pix     <= 16'hFFFF;
            der_mag_age_ms_pix     <= 16'hFFFF;

            der_bmp_valid_ref_pix  <= 1'b0;
            der_acc_valid_ref_pix  <= 1'b0;
            der_mag_valid_ref_pix  <= 1'b0;

            der_altitude_cm_pix    <= 32'd0;
            der_vertical_speed_cms_pix <= 32'd0;
            der_roll_mdeg_pix      <= 32'd0;
            der_heading_mdeg_pix   <= 32'd0;

            i2c_nack_count_pix     <= 16'd0;
            i2c_timeout_count_pix  <= 16'd0;
            txn_rate_hz_pix        <= 16'd0;
            cdc_update_count_pix   <= 32'd0;
            frame_count_pix        <= 32'd0;
            build_id_pix           <= 32'd0;
            schema_word_pix        <= 16'd0;
        end else if (pix_commit) begin
            bmp_valid_pix          <= bmp_valid_in;
            bmp_status_pix         <= bmp_status_in;
            bmp_age_ms_pix         <= bmp_age_ms_in;
            bmp_seq_pix            <= bmp_seq_in;

            acc_valid_pix          <= acc_valid_in;
            acc_status_pix         <= acc_status_in;
            acc_age_ms_pix         <= acc_age_ms_in;
            acc_seq_pix            <= acc_seq_in;

            mag_valid_pix          <= mag_valid_in;
            mag_status_pix         <= mag_status_in;
            mag_age_ms_pix         <= mag_age_ms_in;
            mag_seq_pix            <= mag_seq_in;

            der_valid_pix          <= der_valid_in;
            der_status_pix         <= der_status_in;

            der_alt_fresh_pix      <= der_alt_fresh_in;
            der_vspd_fresh_pix     <= der_vspd_fresh_in;
            der_roll_fresh_pix     <= der_roll_fresh_in;
            der_head_fresh_pix     <= der_head_fresh_in;

            der_bmp_seq_ref_pix    <= der_bmp_seq_ref_in;
            der_acc_seq_ref_pix    <= der_acc_seq_ref_in;
            der_mag_seq_ref_pix    <= der_mag_seq_ref_in;

            der_bmp_age_ms_pix     <= der_bmp_age_ms_in;
            der_acc_age_ms_pix     <= der_acc_age_ms_in;
            der_mag_age_ms_pix     <= der_mag_age_ms_in;

            der_bmp_valid_ref_pix  <= der_bmp_valid_ref_in;
            der_acc_valid_ref_pix  <= der_acc_valid_ref_in;
            der_mag_valid_ref_pix  <= der_mag_valid_ref_in;

            der_altitude_cm_pix    <= der_altitude_cm_in;
            der_vertical_speed_cms_pix <= der_vertical_speed_cms_in;
            der_roll_mdeg_pix      <= der_roll_mdeg_in;
            der_heading_mdeg_pix   <= der_heading_mdeg_in;

            i2c_nack_count_pix     <= i2c_nack_count_in;
            i2c_timeout_count_pix  <= i2c_timeout_count_in;
            txn_rate_hz_pix        <= txn_rate_hz_in;
            cdc_update_count_pix   <= cdc_update_count_in;
            frame_count_pix        <= frame_count_in;
            build_id_pix           <= build_id_in;
            schema_word_pix        <= schema_word_in;
        end
    end

endmodule

`default_nettype wire