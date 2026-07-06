`timescale 1ns/1ps
`default_nettype none

`include "telemetry_defs_vh.vh"

//==============================================================================
// derived_snapshot_regs
//------------------------------------------------------------------------------
// Publish one coherent derived-state telemetry record.
//
// Freshness is supplied by the producer instead of inferred from raw-valid alone.
// That keeps delayed math outputs, especially CORDIC-based roll/heading, from
// being advertised fresh until the result is sequence-qualified.
//==============================================================================

module derived_snapshot_regs #(
    parameter [31:0] BUILD_ID_CONST    = `TELEM_BUILD_ID,
    parameter [15:0] SCHEMA_WORD_CONST = `TELEM_SCHEMA_WORD
)(
    input  wire        clk,
    input  wire        rst,

    //--------------------------------------------------------------------------
    // Timebase
    //--------------------------------------------------------------------------
    input  wire [31:0] now_us,

    //--------------------------------------------------------------------------
    // Publish command
    //--------------------------------------------------------------------------
    input  wire        pub_commit,

    //--------------------------------------------------------------------------
    // Derived-state candidate inputs
    //--------------------------------------------------------------------------
    input  wire        in_valid,
    input  wire [7:0]  in_status,
    input  wire        in_alt_fresh,
    input  wire        in_vspd_fresh,
    input  wire        in_roll_fresh,
    input  wire        in_head_fresh,
    input  wire [31:0] in_altitude_cm,
    input  wire [31:0] in_vertical_speed_cms,
    input  wire [31:0] in_roll_mdeg,
    input  wire [31:0] in_heading_mdeg,

    //--------------------------------------------------------------------------
    // Source provenance
    //--------------------------------------------------------------------------
    input  wire [15:0] in_bmp_seq_ref,
    input  wire [15:0] in_acc_seq_ref,
    input  wire [15:0] in_mag_seq_ref,

    input  wire [31:0] in_bmp_t_us,
    input  wire [31:0] in_acc_t_us,
    input  wire [31:0] in_mag_t_us,

    input  wire [15:0] in_bmp_age_ms,
    input  wire [15:0] in_acc_age_ms,
    input  wire [15:0] in_mag_age_ms,

    input  wire        in_bmp_valid,
    input  wire        in_acc_valid,
    input  wire        in_mag_valid,

    //--------------------------------------------------------------------------
    // System observability sideband
    //--------------------------------------------------------------------------
    input  wire [15:0] in_i2c_nack_count,
    input  wire [15:0] in_i2c_timeout_count,
    input  wire [15:0] in_txn_rate_hz,
    input  wire [31:0] in_cdc_update_count,
    input  wire [31:0] in_frame_count,

    //--------------------------------------------------------------------------
    // Published record
    //--------------------------------------------------------------------------
    output reg  [31:0] der_t_us,
    output reg  [15:0] der_seq,
    output reg  [7:0]  der_source_id,
    output reg  [7:0]  der_status,
    output reg         der_valid,

    output reg         der_alt_fresh,
    output reg         der_vspd_fresh,
    output reg         der_roll_fresh,
    output reg         der_head_fresh,

    output reg  [15:0] der_bmp_seq_ref,
    output reg  [15:0] der_acc_seq_ref,
    output reg  [15:0] der_mag_seq_ref,

    output reg  [15:0] der_bmp_age_ms,
    output reg  [15:0] der_acc_age_ms,
    output reg  [15:0] der_mag_age_ms,

    output reg         der_bmp_valid_ref,
    output reg         der_acc_valid_ref,
    output reg         der_mag_valid_ref,

    output reg  [31:0] der_altitude_cm,
    output reg  [31:0] der_vertical_speed_cms,
    output reg  [31:0] der_roll_mdeg,
    output reg  [31:0] der_heading_mdeg,

    output reg  [15:0] der_i2c_nack_count,
    output reg  [15:0] der_i2c_timeout_count,
    output reg  [15:0] der_txn_rate_hz,
    output reg  [31:0] der_cdc_update_count,
    output reg  [31:0] der_frame_count,
    output reg  [31:0] der_build_id,
    output reg  [15:0] der_schema_word
);

    wire _unused_time_ref_ok;
    assign _unused_time_ref_ok = in_bmp_t_us[0] ^ in_acc_t_us[0] ^ in_mag_t_us[0];

    //--------------------------------------------------------------------------
    // Coherent publish
    //--------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            der_t_us               <= 32'd0;
            der_seq                <= 16'd0;
            der_source_id          <= `SRC_DERIVED_STATE;
            der_status             <= `ST_NOT_INITIALIZED;
            der_valid              <= 1'b0;

            der_alt_fresh          <= 1'b0;
            der_vspd_fresh         <= 1'b0;
            der_roll_fresh         <= 1'b0;
            der_head_fresh         <= 1'b0;

            der_bmp_seq_ref        <= 16'd0;
            der_acc_seq_ref        <= 16'd0;
            der_mag_seq_ref        <= 16'd0;

            der_bmp_age_ms         <= 16'hFFFF;
            der_acc_age_ms         <= 16'hFFFF;
            der_mag_age_ms         <= 16'hFFFF;

            der_bmp_valid_ref      <= 1'b0;
            der_acc_valid_ref      <= 1'b0;
            der_mag_valid_ref      <= 1'b0;

            der_altitude_cm        <= 32'd0;
            der_vertical_speed_cms <= 32'd0;
            der_roll_mdeg          <= 32'd0;
            der_heading_mdeg       <= 32'd0;

            der_i2c_nack_count     <= 16'd0;
            der_i2c_timeout_count  <= 16'd0;
            der_txn_rate_hz        <= 16'd0;
            der_cdc_update_count   <= 32'd0;
            der_frame_count        <= 32'd0;
            der_build_id           <= BUILD_ID_CONST;
            der_schema_word        <= SCHEMA_WORD_CONST;
        end else begin
            if (pub_commit) begin
                der_t_us               <= now_us;
                der_seq                <= der_seq + 16'd1;
                der_source_id          <= `SRC_DERIVED_STATE;
                der_status             <= in_status;
                der_valid              <= in_valid;

                der_alt_fresh          <= in_alt_fresh;
                der_vspd_fresh         <= in_vspd_fresh;
                der_roll_fresh         <= in_roll_fresh;
                der_head_fresh         <= in_head_fresh;

                der_bmp_seq_ref        <= in_bmp_seq_ref;
                der_acc_seq_ref        <= in_acc_seq_ref;
                der_mag_seq_ref        <= in_mag_seq_ref;

                der_bmp_age_ms         <= in_bmp_age_ms;
                der_acc_age_ms         <= in_acc_age_ms;
                der_mag_age_ms         <= in_mag_age_ms;

                der_bmp_valid_ref      <= in_bmp_valid;
                der_acc_valid_ref      <= in_acc_valid;
                der_mag_valid_ref      <= in_mag_valid;

                der_altitude_cm        <= in_altitude_cm;
                der_vertical_speed_cms <= in_vertical_speed_cms;
                der_roll_mdeg          <= in_roll_mdeg;
                der_heading_mdeg       <= in_heading_mdeg;

                der_i2c_nack_count     <= in_i2c_nack_count;
                der_i2c_timeout_count  <= in_i2c_timeout_count;
                der_txn_rate_hz        <= in_txn_rate_hz;
                der_cdc_update_count   <= in_cdc_update_count;
                der_frame_count        <= in_frame_count;
                der_build_id           <= BUILD_ID_CONST;
                der_schema_word        <= SCHEMA_WORD_CONST;
            end
        end
    end

endmodule

`default_nettype wire
