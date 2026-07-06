`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// altitude_lut_rom_u8_to_u16
//------------------------------------------------------------------------------
// ROLE
//   Small combinational ROM mapping an 8-bit lookup index to a 16-bit altitude
//   quantity for the visualization path.
//
// PURPOSE
//   Provide a deterministic altitude-like scalar for early barometric
//   visualization bring-up inside flight_viz_model_sys.
//
// CURRENT MAPPING
//   alt_m_u16 = idx * STEP_M
//
// NOTES
//   - This is a placeholder engineering LUT, not a physically calibrated
//     barometric conversion.
//   - The monotone mapping is useful for validating:
//       * BMP snapshot publication
//       * derived-state update flow
//       * BRAM strip-chart writes
//       * CDC transfer into the PIX renderer
//       * altitude tape / chart motion
//   - A later calibrated revision can replace this case table with a real
//     pressure-code-to-altitude table without changing any external interface.
//==============================================================================
module altitude_lut_rom_u8_to_u16 #(
    parameter integer STEP_M = 8
)(
    input  wire [7:0]  idx,
    output reg  [15:0] alt_m_u16
);

    always @(*) begin
        alt_m_u16 = {5'd0, idx, 3'b000};
    end

endmodule

`default_nettype wire