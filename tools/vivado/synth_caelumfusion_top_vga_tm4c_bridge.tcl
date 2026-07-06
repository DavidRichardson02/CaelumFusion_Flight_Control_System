# Deliberate TM4C123GXL UART bridge synthesis build for CaelumFusion.
#
# The RTL generic still has its historical USE_TEENSY_UART_RANGE_BRIDGE name.
# This script gives the LaunchPad bring-up path a TM4C-named report directory
# while preserving the same byte-compatible FPGA packet receiver.

set SCRIPT_DIR [file dirname [file normalize [info script]]]

set OUT_DIR .codex_build/synth_tm4c_uart_bridge
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
