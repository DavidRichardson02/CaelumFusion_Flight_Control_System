# Deliberate legacy-named UART bridge synthesis build for CaelumFusion.
#
# This script intentionally enables USE_TEENSY_UART_RANGE_BRIDGE without
# changing the default top-level parameter in caelumfusion_top_vga.v. It also
# trims optional heavy VGA diagnostic renderers for Basys-3 bridge bring-up
# headroom; the normal rich diagnostic build remains controlled by the defaults
# in the top-level RTL. The active external producer is now EK-TM4C123GXL; use
# synth_caelumfusion_top_vga_tm4c_bridge.tcl for a TM4C-named report directory.
#
# Usage from the project root:
#   vivado -mode batch -source tools/vivado/synth_caelumfusion_top_vga_teensy_bridge.tcl

set SCRIPT_DIR [file dirname [file normalize [info script]]]

set OUT_DIR .codex_build/synth_teensy_uart_bridge
set SYNTH_DESIGN_ARGS [list \
    -generic USE_TEENSY_UART_RANGE_BRIDGE=1 \
    -generic USE_HYGRO_ENV=0 \
    -generic USE_GYRO_I2C=0 \
    -generic USE_LIS2MDL_MAG1=0 \
    -generic USE_COMPASS_TRUTH_PAGE=0 \
    -generic USE_SENSOR_DIAG_PAGE=0 \
    -generic USE_TELEMETRY_TEXT_OVERLAY=0 \
]

source [file join $SCRIPT_DIR synth_caelumfusion_top_vga.tcl]
