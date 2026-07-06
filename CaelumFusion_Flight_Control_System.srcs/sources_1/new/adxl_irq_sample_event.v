`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// adxl_irq_sample_event
//------------------------------------------------------------------------------
// Select the ADXL362 sample request source after INT1/INT2 have already been
// synchronized into the local clock domain.
//
// POLICY
//   0: 100 Hz poll event
//   1: INT1 synchronized rising edge
//   2: INT1 synchronized falling edge
//   3: INT2 synchronized rising edge
//   4: INT2 synchronized falling edge
//
// Invalid policy values fall back to the poll event so a bad parameter does not
// silently strand accelerometer acquisition.
//==============================================================================
module adxl_irq_sample_event #(
    parameter integer POLICY = 0
)(
    input  wire poll_event,

    input  wire int1_rise,
    input  wire int1_fall,
    input  wire int2_rise,
    input  wire int2_fall,

    output wire sample_event
);
    localparam integer POLICY_POLL_100HZ = 0;
    localparam integer POLICY_INT1_RISE  = 1;
    localparam integer POLICY_INT1_FALL  = 2;
    localparam integer POLICY_INT2_RISE  = 3;
    localparam integer POLICY_INT2_FALL  = 4;

    assign sample_event =
        (POLICY == POLICY_INT1_RISE)  ? int1_rise  :
        (POLICY == POLICY_INT1_FALL)  ? int1_fall  :
        (POLICY == POLICY_INT2_RISE)  ? int2_rise  :
        (POLICY == POLICY_INT2_FALL)  ? int2_fall  :
                                        poll_event;
endmodule

`default_nettype wire
