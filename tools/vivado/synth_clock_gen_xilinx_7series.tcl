# Out-of-context synthesis check for the Xilinx 7-series clock generator.
#
# Usage from the project root:
#   vivado -mode batch -source tools/vivado/synth_clock_gen_xilinx_7series.tcl

set SCRIPT_DIR [file dirname [file normalize [info script]]]
set ROOT_DIR   [file normalize [file join $SCRIPT_DIR .. ..]]
cd $ROOT_DIR

set OUT_DIR .codex_build/clock_gen_xilinx_7series
file mkdir $OUT_DIR

set PART xc7a35tcpg236-3
set TOP  clock_gen_xilinx_7series

read_verilog CaelumFusion_Flight_Control.srcs/sources_1/new/clk_wiz_0.v
synth_design -top $TOP -part $PART -mode out_of_context

report_utilization -file [file join $OUT_DIR clock_gen_xilinx_7series_utilization_ooc.rpt]
report_timing_summary -file [file join $OUT_DIR clock_gen_xilinx_7series_timing_ooc.rpt] -max_paths 10 -warn_on_violation
write_checkpoint -force [file join $OUT_DIR clock_gen_xilinx_7series_ooc.dcp]
