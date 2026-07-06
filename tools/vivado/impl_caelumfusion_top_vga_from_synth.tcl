# Reproducible implementation baseline for the CaelumFusion VGA/I2C top.
#
# Usage from the project root after synthesis:
#   vivado -mode batch -source tools/vivado/impl_caelumfusion_top_vga_from_synth.tcl

set SCRIPT_DIR [file dirname [file normalize [info script]]]
set ROOT_DIR   [file normalize [file join $SCRIPT_DIR .. ..]]
cd $ROOT_DIR

set SYNTH_DCP .codex_build/synth_baseline/caelumfusion_top_vga_synth.dcp
set OUT_DIR   .codex_build/impl_baseline
file mkdir $OUT_DIR

if {![file exists $SYNTH_DCP]} {
    error "Missing synthesis checkpoint: $SYNTH_DCP. Run tools/vivado/synth_caelumfusion_top_vga.tcl first."
}

open_checkpoint $SYNTH_DCP
opt_design
place_design
phys_opt_design
route_design

report_timing_summary -file [file join $OUT_DIR caelumfusion_top_vga_timing_impl.rpt] -max_paths 20 -warn_on_violation
report_drc -file [file join $OUT_DIR caelumfusion_top_vga_drc_impl.rpt]
report_utilization -file [file join $OUT_DIR caelumfusion_top_vga_utilization_impl.rpt]
write_checkpoint -force [file join $OUT_DIR caelumfusion_top_vga_impl_routed.dcp]
