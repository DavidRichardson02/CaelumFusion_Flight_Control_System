# Write a bitstream from the routed implementation checkpoint produced by
# tools/vivado/impl_caelumfusion_top_vga_from_synth.tcl.
#
# Usage from the project root:
#   vivado -mode batch -source tools/vivado/write_bitstream_from_impl.tcl

set SCRIPT_DIR [file dirname [file normalize [info script]]]
set ROOT_DIR   [file normalize [file join $SCRIPT_DIR .. ..]]
cd $ROOT_DIR

set IMPL_DCP .codex_build/impl_baseline/caelumfusion_top_vga_impl_routed.dcp
set OUT_BIT  .codex_build/impl_baseline/caelumfusion_top_vga_adxl_irq_int1.bit

if {![file exists $IMPL_DCP]} {
    error "Missing routed checkpoint: $IMPL_DCP. Run tools/vivado/impl_caelumfusion_top_vga_from_synth.tcl first."
}

open_checkpoint $IMPL_DCP
write_bitstream -force $OUT_BIT
close_design
