`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// clock_gen_xilinx_7series
//------------------------------------------------------------------------------
// ROLE
//   Vendor-isolated Xilinx 7-series MMCM clock-generation wrapper.
//
// PURPOSE
//   Convert the Basys-3 100.000 MHz board clock into the 25.000 MHz pixel clock
//   used by the VGA rendering path while keeping Xilinx primitive details out of
//   the rest of the design.
//
// CLEAN INTERFACE
//   clk_100m_in
//     Board/system input clock. The board-level XDC must constrain this pin.
//
//   mmcm_reset
//     Active-high MMCM reset.
//       1 = MMCM held in reset, output clock not considered valid.
//       0 = MMCM allowed to run and lock.
//
//   clk_25m_out
//     Buffered 25.000 MHz generated clock.
//
//   mmcm_locked
//     Raw MMCME2_BASE LOCKED output after the MMCM has achieved lock.
//
//   clk_valid
//     Alias for mmcm_locked. A higher-level clk_reset_mgr may use this as a
//     clock-ready indication, but reset sequencing is intentionally not done here.
//
// FREQUENCY PLAN
//   Target family: Xilinx 7-series.
//
//     Fin             = 100.000 MHz
//     CLKIN1_PERIOD   = 10.000 ns
//     DIVCLK_DIVIDE   = 1
//     CLKFBOUT_MULT_F = 10.000
//     FVCO            = Fin * CLKFBOUT_MULT_F / DIVCLK_DIVIDE
//                     = 100.000 MHz * 10.000 / 1
//                     = 1000.000 MHz
//     CLKOUT0_DIVIDE_F = 40.000
//     Fout0            = FVCO / CLKOUT0_DIVIDE_F
//                      = 1000.000 MHz / 40.000
//                      = 25.000 MHz
//
//   FVCO must remain inside the valid MMCME2 VCO range for the selected 7-series
//   device and speed grade. Recheck the Xilinx 7-series clocking resources guide
//   before changing multiplier/divider values.
//
// BUFFERING DISCIPLINE
//   - The MMCM feedback path is routed through BUFG. This gives the MMCM a
//     buffered global-clock feedback reference and matches the intended global
//     clock routing delay.
//   - The generated 25 MHz output is routed through BUFG before it leaves this
//     wrapper.
//   - This module does not implement fabric clock division or clock gating.
//
// RESET / LOCK RESPONSIBILITY
//   This wrapper only controls the MMCM primitive and exposes lock state. A
//   higher-level clk_reset_mgr should:
//     - combine external reset with mmcm_locked / clk_valid,
//     - stretch reset after lock,
//     - synchronize reset release into each generated clock domain,
//     - export any project-level clocks_ready signal.
//
// FUTURE OUTPUT CLOCKS
//   Additional generated clocks such as clk_sys, clk_hdmi, clk_aux, or clk_debug
//   should be added by:
//     1) assigning a valid MMCM CLKOUTn divide/phase/duty plan,
//     2) routing that CLKOUTn through its own BUFG,
//     3) exposing an intentionally named output port,
//     4) updating timing constraints and the clk_reset_mgr reset synchronizers.
//
// XDC GUIDANCE
//   Board input clock:
//     create_clock -name clk -period 10.000 [get_ports clk]
//
//   Vivado normally derives generated clocks through MMCME2_BASE and BUFG. If a
//   flow does not infer the generated clock, add a scoped create_generated_clock
//   on the BUFG output pin for clk_25m_out.
//
//   Keep asynchronous reset constraints scoped. Do not false-path whole clock
//   domains just because this MMCM creates a generated clock; normal setup/hold
//   timing between related clocks should remain visible.
//==============================================================================
module clock_gen_xilinx_7series (
    input  wire clk_100m_in,
    input  wire mmcm_reset,
    output wire clk_25m_out,
    output wire mmcm_locked,
    output wire clk_valid
);

    // Documentation constants. The MMCME2_BASE primitive still requires real
    // literal parameters for the clocking plan.
    localparam integer FIN_KHZ   = 100000;
    localparam integer FVCO_KHZ  = 1000000;
    localparam integer FOUT0_KHZ = 25000;

    wire clkfb_mmcm;
    wire clkfb_global;
    wire clk_25m_mmcm;
    wire clkfb_mmcm_b_unused;
    wire clk_25m_mmcm_b_unused;
    wire clkout1_mmcm_unused;
    wire clkout1_mmcm_b_unused;
    wire clkout2_mmcm_unused;
    wire clkout2_mmcm_b_unused;
    wire clkout3_mmcm_unused;
    wire clkout3_mmcm_b_unused;
    wire clkout4_mmcm_unused;
    wire clkout5_mmcm_unused;
    wire clkout6_mmcm_unused;

    assign clk_valid = mmcm_locked;

    //--------------------------------------------------------------------------
    // Xilinx 7-series MMCM
    //--------------------------------------------------------------------------
    MMCME2_BASE #(
        .BANDWIDTH("OPTIMIZED"),

        // Input clock period in ns. 100 MHz -> 10.000 ns.
        .CLKIN1_PERIOD(10.000),

        // VCO = CLKIN1 * CLKFBOUT_MULT_F / DIVCLK_DIVIDE = 1000 MHz.
        .CLKFBOUT_MULT_F(10.000),
        .DIVCLK_DIVIDE(1),

        // Output 0 = VCO / CLKOUT0_DIVIDE_F = 25 MHz.
        .CLKOUT0_DIVIDE_F(40.000),
        .CLKOUT0_PHASE(0.000),
        .CLKOUT0_DUTY_CYCLE(0.500),

        // Reserved for future generated clocks. Leave unused outputs unbuffered
        // and unexported until the clock tree and reset manager are updated.
        .CLKOUT1_DIVIDE(1),
        .CLKOUT2_DIVIDE(1),
        .CLKOUT3_DIVIDE(1),
        .CLKOUT4_DIVIDE(1),
        .CLKOUT5_DIVIDE(1),
        .CLKOUT6_DIVIDE(1),

        .CLKOUT1_PHASE(0.000),
        .CLKOUT2_PHASE(0.000),
        .CLKOUT3_PHASE(0.000),
        .CLKOUT4_PHASE(0.000),
        .CLKOUT5_PHASE(0.000),
        .CLKOUT6_PHASE(0.000),

        .CLKOUT1_DUTY_CYCLE(0.500),
        .CLKOUT2_DUTY_CYCLE(0.500),
        .CLKOUT3_DUTY_CYCLE(0.500),
        .CLKOUT4_DUTY_CYCLE(0.500),
        .CLKOUT5_DUTY_CYCLE(0.500),
        .CLKOUT6_DUTY_CYCLE(0.500),

        .CLKFBOUT_PHASE(0.000),
        .REF_JITTER1(0.010),
        .STARTUP_WAIT("FALSE")
    ) u_mmcm (
        .CLKIN1   (clk_100m_in),
        .RST      (mmcm_reset),
        .PWRDWN   (1'b0),

        .CLKFBIN  (clkfb_global),
        .CLKFBOUT (clkfb_mmcm),
        .CLKFBOUTB(clkfb_mmcm_b_unused),

        .CLKOUT0  (clk_25m_mmcm),
        .CLKOUT0B (clk_25m_mmcm_b_unused),
        .CLKOUT1  (clkout1_mmcm_unused),
        .CLKOUT1B (clkout1_mmcm_b_unused),
        .CLKOUT2  (clkout2_mmcm_unused),
        .CLKOUT2B (clkout2_mmcm_b_unused),
        .CLKOUT3  (clkout3_mmcm_unused),
        .CLKOUT3B (clkout3_mmcm_b_unused),
        .CLKOUT4  (clkout4_mmcm_unused),
        .CLKOUT5  (clkout5_mmcm_unused),
        .CLKOUT6  (clkout6_mmcm_unused),

        .LOCKED   (mmcm_locked)
    );

    //--------------------------------------------------------------------------
    // Global clock buffers
    //--------------------------------------------------------------------------
    BUFG u_bufg_feedback (
        .I(clkfb_mmcm),
        .O(clkfb_global)
    );

    BUFG u_bufg_clk_25m (
        .I(clk_25m_mmcm),
        .O(clk_25m_out)
    );

endmodule

`default_nettype wire
