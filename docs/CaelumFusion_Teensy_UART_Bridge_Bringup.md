# Deprecated: CaelumFusion Teensy UART Bridge Bring-Up

The Teensy 4.1 producer is no longer an active hardware path for this project.
Do not wire the failed Teensy into the Basys-3 system, and do not use it as a
voltage reference, level shifter, pass-through UART device, or diagnostic load.

Use the EK-TM4C123GXL LaunchPad replacement path instead:

- Active bring-up guide:
  `docs/CaelumFusion_TM4C123GXL_UART_Bridge_Bringup.md`
- Active firmware:
  `firmware/tm4c123gxl_bridge_range_producer/main.c`
- TM4C-named FPGA build script:
  `tools/vivado/synth_caelumfusion_top_vga_tm4c_bridge.tcl`

The FPGA RTL still contains historical `teensy_*` names for the UART bridge
ports, generic, and wrapper modules. Those names are compatibility labels only;
the active physical producer is the EK-TM4C123GXL LaunchPad UART1 interface.
